# Troubleshooting

## Init Times Out Connecting To CARLA

Check:

- `launch_carla_server` matches the intended ownership mode.
- `CARLA_HOST` and `CARLA_PORT` point to the correct endpoint.

Init logs the resolved mode and endpoint:

```text
CARLA mode=external endpoint=127.0.0.1:2000
```

For an external server, this must say `external` and the log must not contain
`Launched owned CARLA server process`.

- no other process occupies the selected CARLA port.
- `<output_dir>/carla_server/stderr.log` for internal-server failures.
- `carla_connect_timeout_seconds` is long enough for the host.

The normal RPC timeout is restored after every short connection attempt.

## `//carlaCache/.../Carla/Maps/Nav` Creation Failure

This indicates that CARLA resolved its user home to `/` or an empty value and
attempted to create navigation cache under `//carlaCache`.

Use a writable cache home:

```bash
-e CARLA_HOME=/mnt/output/.carla-home
-v /host/output:/mnt/output
```

For wrapper-owned CARLA, `carla_home` provides the config equivalent. The
wrapper creates `<carla_home>/carlaCache` and sets the child process `HOME`
before launch. Rebuild the image after upgrading from a version that did not
set this environment.

## OpenDRIVE Map Not Found

Reset resolves:

```text
<xodr_root>/<ScenarioPack.map_name>.xodr
```

Verify the filename, case, mount path, and read permission. The wrapper does not
fall back to the current CARLA world when the requested map is unavailable.

## Route Generation Fails

Verify:

- the initial observation contains the ego state,
- the scenario has an ego goal position,
- start and goal project onto the generated CARLA map,
- the route planner returns at least two waypoints,
- the reset output directory is writable.

To bypass generation, provide a readable `route_xml_path`.

## Permission Denied Creating `concrete/pcla_routes`

`ResetRequest.output_dir` may be a relative case name. Current images resolve it
below `InitRequest.output_dir`, so Init base `/mnt/output` plus Reset path
`concrete` writes `/mnt/output/concrete/pcla_routes`.

If the traceback shows an attempt to create `concrete` directly below `/app`,
rebuild the image with the output-base fix. Also confirm `/mnt/output` is mounted
writable and Init sends that path as its output directory.

## Permission Denied Creating `plant_viz` Or Another Relative Agent Path

PCLA agents may create relative directories during setup or inference. Current
images run those calls from the writable
`<ResetRequest.output_dir>/pcla_runtime` directory, so `plant_viz` becomes:

```text
<ResetRequest.output_dir>/pcla_runtime/plant_viz
```

If the traceback shows `Permission denied: 'plant_viz'` while the service is
running below `/app`, rebuild the wrapper image with the runtime-directory fix.
Confirm the Init output base is a writable mount. Use `pcla_runtime_dir` only
when the default location must be changed; relative values remain confined to
the current Reset output directory.

## Unknown Agent

Agent names must use `<family>_<variant>[_seed]`. Inspect:

```bash
python -m json.tool PCLA/agents.json
```

The optional seed must be numeric. Do not infer a variant from a directory name;
use the registry keys.

## Missing Weights Or Python Dependencies

Inspect the selected `agents.json` entry and verify both its `agent` and
`config` paths inside the container. Also check whether a volume mount hides
files supplied by the image.

Model dependency or checkpoint failures are reported as `AvUnavailable`.
Explicit model loading timeouts are reported as `AvTimeout`.

## Sensors Do Not Produce Data

For camera agents, use:

```yaml
no_rendering: false
sensor_warmup_ticks: 1
```

Current images also detect camera sensors and disable no-rendering mode
automatically. Run internal CARLA with `--gpus all` and do not set
`CARLA_NULLRHI=1`. NullRHI disables the camera rendering pipeline.
Inside the container, `NVIDIA_DRIVER_CAPABILITIES` must include graphics/display
support; current images use `all`, matching the other CARLA wrappers. The image
must also contain
`/etc/vulkan/icd.d/nvidia_icd.json`; without it, Unreal can exit with code `1`
before opening the CARLA RPC port. The same symptom occurs when `libEGL.so.1`
is unavailable; current images install the `libegl1` runtime package.

Mount `/tmp/.X11-unix`, pass `DISPLAY`, and mount the host Xauthority to
`/home/carla/.Xauthority`. This is required on the tested NVIDIA host even with
`-RenderOffScreen`: without it, the version RPC can answer while `get_world()`
continues to time out. The image also installs `xdg-user-dirs`; without it,
Unreal may exit before producing useful logs.

The bundled Python API may print a client identifier such as a source commit
while the simulator reports `0.9.16`. Treat the warning as diagnostic, then
verify `get_world()` and sensor frames. The tested image successfully receives
RGB camera and IMU data from the bundled CARLA 0.9.16 server.

If CARLA prints `Refusing to run with the root privileges`, rebuild the image
with the privilege-drop launcher. The PISA service may remain root, but its
CARLA child runs as `CARLA_RUN_UID`/`CARLA_RUN_GID` (default `1000:1000`).

If the sensor timeout is preceded by a CARLA crash, inspect:

```text
<InitRequest.output_dir>/carla_server/stderr.log
<InitRequest.output_dir>/carla_server/stdout.log
```

The wrapper reports the owned process exit code directly. `Signal 11` or
`Segmentation fault` indicates a server/rendering failure rather than an agent
queue timeout. Also verify sensor blueprint availability and CARLA 0.9.16
compatibility.

For a PlanT config with `visualize: false`, `CARLA_NULLRHI=1` can run without
X11. The launcher intentionally omits `-quality-level` in this mode because
CARLA 0.9.16 exits with Signal 11 when NullRHI and a quality level are combined.

## Non-Ego Actors Do Not Retain Identity

Persistent shadow actors require every observed agent in the frame to carry a
`tracking_id`. Frames with missing or mixed tracking IDs intentionally rebuild all
non-ego actors; list position and `entity_name` are never used as identity.

## PCLA Returns No Action

With the default:

```yaml
action_none_timeout_seconds: 0.0
```

the wrapper raises an immediate precondition failure. Set a small positive
timeout only for agents that legitimately need several polls after the CARLA
tick. A persistent missing action becomes `AvTimeout`, never normal zero
control.

## Cleanup Appears Stuck

The wrapper switches CARLA to asynchronous mode and clears fixed delta time
before destroying actors. If an external process still blocks, confirm that it
is not independently forcing synchronous mode. Enable
`manage_traffic_manager_sync` only when this wrapper owns TrafficManager sync.
