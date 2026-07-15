# PCLA Wrapper

PISA AV service adapter for the [PCLA](https://github.com/sysnycu/PCLA)
multi-agent driving framework. The wrapper mirrors each PISA observation into a
CARLA shadow world and returns the selected PCLA agent's throttle, brake, and
steer command.

## Common Runtime

The current production target is the `common` image profile:

- Ubuntu 24.04 final runtime
- CARLA server and Python API 0.9.16
- Python 3.8.18 in `/opt/pcla-venv`
- PyTorch 2.2.0+cu121
- Plant 1.0, Plant 2.0, CaRL, and Roach dependencies

The image is built directly from Ubuntu and CARLA. Python dependencies are
installed into `/opt/pcla-venv`, CUDA user libraries come from the PyTorch
wheels, and the host supplies the NVIDIA driver through `--gpus all`.

The supported `pcla_agent` values are:

```text
plant2_plant2_0
plant2_plant2_1
plant2_plant2_2
carl_plant_0 ... carl_plant_4
carl_carl_0
carl_carl_1
carl_carlv11
carl_roach_0 ... carl_roach_4
```

Other entries may exist in upstream `PCLA/agents.json`, but the common image
rejects them before model import because their dependency sets are not yet part
of this image.

## Build Variants

Initialize the submodule, then build the reusable runtime:

```bash
git submodule update --init --recursive
docker build --target common-slim -t pcla-wrapper:common-slim .
```

`common-slim` excludes weights. Mount the selected pretrained directory at
runtime:

```text
/mnt/weights/last-v3.ckpt
```

To download the upstream PCLA weight archive for a local PISA host, run:

```bash
scripts/download_pcla_weights.sh "${PISA_DATA_DIR:-/PISA_DATA_DIR}/weights"
```

For the default `pcla` AV, set `weight_path` to `weights/plant_pretrained`.
The executor resolves that under `PISA_DATA_DIR` and mounts the selected
directory into the AV container as `/mnt/weights`. A common-profile root that
contains `plant_pretrained`, `plant2_pretrained`, and `carl_pretrained` may also
be mounted there; the wrapper detects both layouts without modifying the image
filesystem. CI and image builds do
not run this download script; they build the runtime image without model
weights.

## Run

The default mode launches CARLA inside the same container:

```bash
docker run --rm --gpus all --network host \
  -e DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$HOME/.Xauthority":/home/carla/.Xauthority:ro \
  -v /path/to/xodr:/mnt/map/xodr:ro \
  -v /path/to/output:/mnt/output \
  -v /path/to/plant_pretrained:/mnt/weights:ro \
  pcla-wrapper:common-slim
```

The common images default to `CARLA_NULLRHI=1` because PlanT 1.0, PlanT 2.0,
CaRL, and Roach do not use RGB camera input. This avoids requiring X11 for the
internal CARLA server. Set `CARLA_NULLRHI=0` only for a rendered configuration;
that mode may require `DISPLAY`, X11 authorization, and the NVIDIA Vulkan
runtime.

For an external CARLA server, set `launch_carla_server: false` in the request
configuration and provide `CARLA_HOST` and `CARLA_PORT`.

## Lifecycle

- `Init`: validate configuration, optionally launch CARLA, and connect.
- `Reset`: load OpenDRIVE, spawn ego, generate a route, and initialize PCLA.
- `Step`: synchronize actors, tick CARLA once, and run the selected agent.
- `ShouldQuit`: report completion or runtime failure.
- `Stop`: clean agent sensors and wrapper actors while retaining owned CARLA.

The Python package is `pcla_wrapper`. The upstream fork is the top-level
`PCLA/` submodule.

## Service identity and initialization metadata

This service uses `pcla-wrapper` as its stable wrapper artifact identity.
`Ping` therefore returns `Pong.name = "pcla-wrapper"`, while `Pong.version`
is the installed `pcla-wrapper` package/build version (with the repository's
`pyproject.toml` version used when running an uninstalled source checkout).

After successful Init, `InitResponse.name` identifies the actual validated PCLA
component selected from `PCLA/agents.json`, for example `carl_plant_3` or
`plant2_plant2_0`; it is intentionally different from the wrapper artifact
name. `InitResponse.metadata.effective_config` records only normalized,
validated wrapper-specific values that are actually in effect. For example:

```yaml
name: carl_plant_3
metadata:
  effective_config:
    pcla_agent: carl_plant_3
    route_waypoint_distance: 2.0
    route_draw: false
    launch_carla_server: true
    coordinate_y_sign: -1.0
    yaw_sign: -1.0
    steer_sign: -1.0
    sensor_warmup_ticks: 1
```

The complete effective config also reports world-reuse/Traffic Manager flags,
ego blueprint settings, spawn and yaw offsets, the no-action timeout, and the
debug-log interval. It does not copy raw config, paths, environment variables,
credentials, `dt`, map/scenario identity, or output directories. This metadata
is written into the execution manifest, so new fields must never contain
secrets. Shared execution data such as `dt` and map must remain in its canonical
runner-owned manifest fields rather than being duplicated here.

The wrapper and runner must both use a compatible pisa-api with the Ping/Init
identity contract (`pisa-api>=0.4.1` for this package).

## Documentation

- [Configuration](docs/configuration.md)
- [Agents and weights](docs/agents.md)
- [Deployment](docs/deployment.md)
- [Lifecycle and ownership](docs/lifecycle.md)
- [Troubleshooting](docs/troubleshooting.md)

## Development

```bash
uv sync --locked --all-groups
uv run ruff check .
uv run ruff format --check .
uv run pytest
```

The regular CI uses fakes and does not require weights or a GPU. The manually
triggered `Common Runtime` workflow expects weights to already exist on the
runner and mounts the selected directory into `/mnt/weights`.
