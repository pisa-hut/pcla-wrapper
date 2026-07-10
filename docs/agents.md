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

## Required Weight Layout

The slim image expects:

```text
/mnt/weights/
└── last-v3.ckpt   # with AV weight_path=weights/plant_pretrained
```

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
