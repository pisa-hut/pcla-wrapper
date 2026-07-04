# Configuration

The wrapper receives configuration through `pisa_api.av.InitRequest.config`.
Use the flat mapping below as the canonical request configuration.
The simulation time step is supplied separately as `InitRequest.dt`.

Output paths are lifecycle fields:

- `InitRequest.output_dir` is the writable output base.
- An absolute `ResetRequest.output_dir` is used directly.
- A relative `ResetRequest.output_dir` is resolved below the Init base.
- Relative reset paths containing `..` may not escape the Init base.

For example, Init base `/mnt/output` and Reset path `concrete` produce
`/mnt/output/concrete`.

## Precedence

Values are resolved in this order:

1. Environment overrides: `PCLA_AGENT`, `PCLA_ROUTE`,
   `PCLA_PRETRAINED_ROOT`, `CARLA_HOST`, `CARLA_PORT`, `CARLA_TIMEOUT`,
   `CARLA_HOME`, and `CARLA_ROOT`. `CARLA_TM_PORT` configures the internal
   Traffic Manager port and has no flat config equivalent.
2. Flat `InitRequest.config` keys.
3. Legacy nested `pcla:` and `carla:` keys.
4. Wrapper defaults.

If a flat key and its legacy nested equivalent are both present with different
values, Init fails with `InvalidAvRequest`.

## PCLA

| Key | Default | Description |
| --- | --- | --- |
| `pcla_agent` | `carl_plant_3` | Registry name `<family>_<variant>[_seed]`. |
| `pcla_root` | Repository PCLA submodule | Directory containing `PCLA.py` and `agents.json`. |
| `pcla_pretrained_root` | `/opt/pcla-pretrained` | Root containing validated PCLA weight profiles. |
| `pcla_runtime_dir` | `<reset output>/pcla_runtime` | Writable working directory for upstream agent code. |
| `route_xml_path` | `null` | Existing route XML, or generate one from scenario start/goal. |
| `route_waypoint_distance` | `2.0` | Route planner sampling distance in meters. |
| `route_draw` | `false` | Draw route debug markers in CARLA. |
| `action_none_timeout_seconds` | `0.0` | Retry window when PCLA returns no action. |
| `debug_log_interval_steps` | `20` | Driving-state debug log interval; `0` disables these debug logs. |

`route_xml_path` is resolved relative to `pcla_root` unless it is absolute.
Generated routes are written below
`<ResetRequest.output_dir>/pcla_routes/`.

Relative `pcla_runtime_dir` values are resolved below the current Reset output
directory and may not escape it. Absolute values are used directly. The wrapper
temporarily enters this directory only while constructing, running, or cleaning
PCLA, then restores its original working directory. Relative upstream outputs
such as `plant_viz` therefore remain isolated in the current scenario output.

## CARLA Connection And Process

| Key | Default | Description |
| --- | --- | --- |
| `launch_carla_server` | `true` | Launch a wrapper-owned CARLA process. |
| `carla_server_script` | `/app/carla_server.sh` | Server launcher executable. |
| `carla_host` | `localhost` | CARLA RPC host when `CARLA_HOST` is absent. |
| `carla_port` | `2000` | CARLA RPC port when `CARLA_PORT` is absent. |
| `carla_connect_timeout_seconds` | `30.0` | Total connection retry window. |
| `retry_interval_seconds` | `2.0` | Delay between connection attempts. |
| `max_retry_times` | `15` | Legacy fallback used only when total timeout is omitted. |
| `carla_timeout` | `10.0` | Normal CARLA RPC timeout. |
| `carla_home` | `<output_dir>/carla_home` | Writable HOME containing CARLA navigation cache. |
| `carla_root` | unset | Optional local CARLA Python API root. |
| `carla_egg` | unset | Optional CARLA wheel/egg path. |

The wrapper terminates CARLA only when it launched that process. For an
external server, set `launch_carla_server: false`.

`CARLA_HOME` overrides `carla_home`. The Docker image sets it to
`/mnt/output/.carla-home`. The wrapper creates `carlaCache` and `.cache`
below this directory before starting CARLA. Do not set it to `/` or a
read-only path.

## World And Actors

| Key | Default | Description |
| --- | --- | --- |
| `sync` | `true` | Use CARLA synchronous mode. |
| `no_rendering` | `true` | Disable rendering initially; camera sensors automatically switch it to `false`. |
| `sensor_warmup_ticks` | `1` | CARLA ticks after sensor attachment and before initial inference. |
| `xodr_root` | `/mnt/map/xodr` | OpenDRIVE map directory. |
| `reuse_generated_world` | `true` | Reuse an unchanged generated map between resets. |
| `manage_traffic_manager_sync` | `false` | Set TrafficManager async during cleanup. |
| `ego_role_name` | `hero` | Ego blueprint role name. |
| `ego_bp_id` | `vehicle.tesla.model3` | Preferred ego blueprint. |
| `spawn_z_offset` | `3.0` | Ego spawn height offset in meters. |
Agent presentation order never carries identity. When every observed agent has a
`tracking_id`, the wrapper retains its shadow actor by that ID. If any agent lacks a
tracking ID, all non-ego actors use stateless destroy/recreate behavior for that frame.
`entity_name` is optional privileged metadata and is not used as actor identity.

Observation-controlled non-ego actors have physics and gravity disabled. Ego
physics remains enabled.

The owned CARLA launcher uses offscreen rendering with low quality by default.
`CARLA_QUALITY_LEVEL` changes the rendered-mode quality level.
`CARLA_NULLRHI=1` is an explicit sensorless mode and prevents camera sensors
from working. Quality arguments are omitted in NullRHI mode because CARLA
0.9.16 crashes when both options are supplied.
When the wrapper service runs as root, only the CARLA child is dropped to
`CARLA_RUN_UID`/`CARLA_RUN_GID`, both defaulting to `1000`.

## Coordinates

| Key | Default | Formula |
| --- | --- | --- |
| `coordinate_y_sign` | `-1.0` | `carla_y = pisa_y * sign` |
| `yaw_sign` | `-1.0` | `carla_yaw_deg = sign * degrees(pisa_yaw) + offset` |
| `steer_sign` | `-1.0` | `pisa_steer = pcla_steer / sign` |
| `yaw_offset_deg` | `0.0` | Constant heading offset in degrees. |

All sign values must be non-zero and are normalized to `+1` or `-1`. PISA yaw
and yaw rate are always interpreted as radians. `coordinate_y_sign` and
`yaw_sign` must be equal so position and 3-D orientation use one consistent
handedness conversion.

## Observation Geometry Profile

PCLA shadow actors support `BOUNDING_BOX` shapes. Dimensions are positive,
finite full extents in metres; the center is an actor-local relative transform
in metres/radians. Polygon and cylinder observations fail explicitly.

CARLA vehicle assets cannot be resized through the Python API. For each
observed box, the wrapper deterministically selects the same-type blueprint
whose native full dimensions have the lowest normalized 3-D error, then aligns
that native box with `Shape.center`. The requested/native dimensions and error
are logged once per mapping. Cameras, LiDAR and physics therefore use the
nearest available CARLA asset and may not exactly match the requested box.

## Canonical Control Contract

Control follows the canonical simcore `THROTTLE_STEER_BREAK` contract in
`runner/docs/data-contracts/README.md`; this wrapper does not define a private
AV/simulator profile. It emits exactly three finite numeric keys:

| Key | Canonical range |
| --- | --- |
| `throttle` | `[0, 1]` |
| `brake` | `[0, 1]` |
| `steer` | `[-1, 1]`, positive left |

`steer_sign` converts PCLA's native CARLA-oriented output to the canonical
positive-left boundary convention. If brake is positive, throttle is set to
zero. Missing, nonnumeric, non-finite, or out-of-range fields fail explicitly;
values are never silently clipped. The payload key is `brake`; the historical
wire enum spelling remains `THROTTLE_STEER_BREAK`.

## Legacy Nested Form

This compatibility form remains accepted:

```yaml
pcla:
  agent: carl_plant_3
  pcla_root: /app/PCLA
  route_path: null

carla:
  host: localhost
  port: 2000
  timeout: 10.0
  sync: true
```

New deployments should use flat keys because the nested form is temporary.
