# Agents And Weights

The wrapper first validates names against upstream `PCLA/agents.json`, then
applies the image profile. The current Docker images set
`PCLA_IMAGE_PROFILE=common`.

## Supported Agents

| Family | `pcla_agent` values | Driving camera input |
| --- | --- | --- |
| Plant 2.0 | `plant2_plant2_0`, `_1`, `_2` | No |
| Plant 1.0 | `carl_plant_0` through `_4` | No |
| CaRL | `carl_carl_0`, `_1`, `carl_carlv11` | No |
| Roach | `carl_roach_0` through `_4` | No |

Plant visualization can add an RGB camera, but it is not model input and should
remain disabled for the common runtime. NullRHI is not the tested default
because CARLA sensor and generated OpenDRIVE behavior is less reliable there.

All 16 exact names in this table are state-based. See
[State-based agents](state-based-agents.md) for the complete name list, each
family's observation, and why no camera or LiDAR input does not yet mean the
wrapper can run without shadow CARLA.

## Required Weight Layout

The slim image accepts either a selected weight directory:

```text
/mnt/weights/
├── last.ckpt
├── last-v1.ckpt
├── last-v2.ckpt
├── last-v3.ckpt
└── last-v4.ckpt   # with AV weight_path=weights/plant_pretrained
```

or a common-profile root containing all three families:

```text
/mnt/weights/
├── plant_pretrained/
├── plant2_pretrained/
└── carl_pretrained/
```

When resolving an agent, the wrapper detects nested family directories. If
they are absent, it treats `/mnt/weights` as the directly selected family
directory. Resolution is read-only and works when the container runs as a
non-root UID.

The exact required checkpoint paths are versioned in
`pcla_wrapper/agent_profiles.json`. Init validates the selected agent's files
before importing its model and reports every missing path.

## Validation

Validate a slim image with mounted weights:

```bash
docker run --rm --gpus all \
  -v /path/to/plant_pretrained:/mnt/weights:ro \
  pcla-wrapper:common-slim \
  /app/scripts/validate_common_runtime.py --check-weights
```

Load one model without starting CARLA:

```bash
docker run --rm --gpus all \
  -v /path/to/plant_pretrained:/mnt/weights:ro \
  pcla-wrapper:common-slim \
  /app/scripts/smoke_common_agent.py carl_plant_3
```

This checks dependency import, registry resolution, configuration, and
checkpoint loading. Full driving validation still requires a GPU-capable CARLA
host and a PISA scenario.
