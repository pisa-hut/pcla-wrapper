# Deployment

## Image Contract

`common-slim` is the base deployment image:

- Ubuntu 24.04
- CARLA 0.9.16 server and CPython 3.8 API
- Python 3.8.18 in `/opt/pcla-venv`
- PyTorch 2.2.0+cu121
- Plant 1.0, Plant 2.0, CaRL, and Roach dependencies
- no Conda environment or complete CUDA toolkit

Build it with:

```bash
git submodule update --init --recursive
docker build --target common-slim -t pcla-wrapper:common-slim .
```

The runtime dependency layers are before `COPY . /app`, so source-only changes
reuse the Python package cache. `.dockerignore` excludes pretrained assets,
runtime map caches, logs, and generated files.

## Slim And Bundled

Use `common-slim` during development. Mount the selected agent weight directory read-only:

```bash
-v /host/plant_pretrained:/mnt/weights:ro
```

## Internal CARLA

The default configuration launches `/app/carla_server.sh`. Run with the NVIDIA
container runtime:

```bash
docker run --rm --gpus all --network host \
  -e DISPLAY \
  -e CARLA_HOME=/mnt/output/.carla-home \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$HOME/.Xauthority":/home/carla/.Xauthority:ro \
  -v /host/maps:/mnt/map/xodr:ro \
  -v /host/output:/mnt/output \
  -v /host/plant_pretrained:/mnt/weights:ro \
  pcla-wrapper:common-slim
```

The common images default to `CARLA_NULLRHI=1` because their supported agents
do not use RGB camera input. Set `CARLA_NULLRHI=0` only when a rendered CARLA
process is required; rendered mode may also require X11 and Vulkan access.

CARLA logs are written below
`<InitRequest.output_dir>/carla_server/`. The server is started once and reused
across Reset calls. Stop removes agent sensors and dynamic actors without
terminating the server.

The output volume must be writable because it also holds CARLA navigation and
XDG caches. When using `--user`, set a writable `CARLA_HOME`.

## External CARLA

Set:

```yaml
launch_carla_server: false
```

Then provide `CARLA_HOST`, `CARLA_PORT`, and a compatible CARLA 0.9.16 server.
The wrapper never terminates an external server.

## Ports

- `PORT`: PISA AV gRPC service, default `50051`
- `CARLA_PORT`: CARLA RPC, default `2000`
- `CARLA_TM_PORT`: TrafficManager, default `8000`

## CI

The standard workflow runs formatting and fake-based unit tests on GitHub-hosted
runners. `.github/workflows/common-runtime.yml` is a manual workflow for a
self-hosted runner labelled `gpu`. It builds `common-slim`, checks the mounted
weights, and loads the default `carl_plant_3` model.
