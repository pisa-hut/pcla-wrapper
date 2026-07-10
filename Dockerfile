# syntax=docker/dockerfile:1.7
FROM docker.io/tonychi/carla:0.9.16 AS carla-runtime

FROM ubuntu:24.04 AS common-runtime
ENV DEBIAN_FRONTEND=noninteractive

RUN groupmod --new-name carla ubuntu \
    && usermod --login carla --home /home/carla --move-home ubuntu

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    git \
    libegl1 \
    libfontconfig1 \
    libgl-dev \
    libglib2.0-0t64 \
    libjpeg-dev \
    libpng-dev \
    libsm6 \
    libtiff5-dev \
    libvulkan1 \
    libxext6 \
    libxrender1 \
    mesa-vulkan-drivers \
    unzip \
    wget \
    xdg-user-dirs \
    && rm -rf /var/lib/apt/lists/*

# CARLA 0.9.16 still links against libtiff.so.5. Install the legacy runtime
# after Noble's development package has pulled in the codec dependencies.
ADD https://security.ubuntu.com/ubuntu/pool/main/t/tiff/libtiff5_4.3.0-6_amd64.deb /tmp/libtiff5.deb
RUN dpkg -i /tmp/libtiff5.deb && rm /tmp/libtiff5.deb

COPY --from=carla-runtime --chown=carla:carla /opt/carla /opt/carla
COPY --from=ghcr.io/astral-sh/uv:0.10.4 /uv /uvx /bin/
COPY docker/nvidia_icd.json /etc/vulkan/icd.d/nvidia_icd.json

ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python
ENV UV_PYTHON_CACHE_DIR=/tmp/uv-python-cache
RUN uv python install 3.8.18 \
    && uv venv --python 3.8.18 /opt/pcla-venv

WORKDIR /app
COPY docker/requirements /tmp/requirements
COPY docker/constraints /tmp/constraints
RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install --python /opt/pcla-venv/bin/python \
        --index-url https://download.pytorch.org/whl/cu121 \
        "torch==2.2.0+cu121" \
        "torchvision==0.17.0+cu121" \
        "torchaudio==2.2.0+cu121" \
    && uv pip install --python /opt/pcla-venv/bin/python \
        --constraint /tmp/constraints/python38.txt \
        --requirement /tmp/requirements/core.txt \
    && uv pip install --python /opt/pcla-venv/bin/python \
        --constraint /tmp/constraints/python38.txt \
        --requirement /tmp/requirements/common.txt \
    && uv pip check --python /opt/pcla-venv/bin/python \
    && rm -rf /tmp/requirements /tmp/constraints /tmp/uv-python-cache

COPY PCLA/dist/carla-0.9.16-cp38-cp38-linux_x86_64.whl /tmp/
RUN uv pip install --python /opt/pcla-venv/bin/python \
        /tmp/carla-0.9.16-cp38-cp38-linux_x86_64.whl \
    && rm /tmp/carla-0.9.16-cp38-cp38-linux_x86_64.whl

COPY . /app

# The common runtime supports PlanT 1.0, PlanT 2.0, CaRL, and Roach. Keep one
# dynamic-map cache and expose only the three weight directories they require.
RUN set -eux; \
    ln -s \
        /app/PCLA/pcla_agents/plant2/carla_garage/birds_eye_view/maps_2ppm_cv \
        /app/PCLA/pcla_agents/carl/birds_eye_view/maps_2ppm_cv; \
    canonical_speed_limits=/app/PCLA/pcla_agents/plant/carla_garage/speed_limits; \
    for speed_limit in "${canonical_speed_limits}"/*_speed_limits.npy; do \
        ln -s "${speed_limit}" \
            "/app/PCLA/pcla_agents/plant2/carla_garage/speed_limits/$(basename "${speed_limit}")"; \
    done; \
    for name in carl_pretrained plant2_pretrained plant_pretrained; do \
        ln -s "/mnt/weights" \
            "/app/PCLA/pcla_agents/${name}"; \
    done; \
    /opt/pcla-venv/bin/python -c 'from pathlib import Path; p = Path("/app/PCLA/pcla_agents/plant/PlanT_agent.py"); s = p.read_text(); s = s.replace("Mount the official pcla_agents directory at /opt/" + "pcla-" + "pretrained:ro or set PCLA_PRETRAINED_ROOT.", "Mount the selected weight directory at /mnt/weights:ro."); p.write_text(s)'

RUN test -f /app/PCLA/PCLA.py \
    && grep -q 'map_name == "OpenDriveMap"' \
        /app/PCLA/pcla_agents/plant2/carla_garage/privileged_route_planner.py \
    && grep -q 'MapImage.draw_map_image' \
        /app/PCLA/pcla_agents/plant2/carla_garage/birds_eye_view/chauffeurnet.py \
    && chmod +x \
        /app/entrypoint.sh \
        /app/carla_server.sh \
        /app/scripts/smoke_common_agent.py \
        /app/scripts/validate_common_runtime.py \
    && PYTHONPATH=/app:/app/PCLA \
        /opt/pcla-venv/bin/python /app/scripts/validate_common_runtime.py

ENV PATH=/opt/pcla-venv/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ENV PYTHONPATH=/app:/app/PCLA
ENV LD_LIBRARY_PATH=/usr/local/nvidia/lib:/usr/local/nvidia/lib64
ENV NVIDIA_REQUIRE_CUDA="cuda>=12.1"
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

ENV PORT=50051
ENV CARLA_HOST=localhost
ENV CARLA_PORT=2000
ENV CARLA_TIMEOUT=120
ENV CARLA_TM_PORT=8000
ENV CARLA_NULLRHI=1
ENV CARLA_HOME=/mnt/output/.carla-home
ENV HOME=/mnt/output/.carla-home
ENV PCLA_IMAGE_PROFILE=common
ENV PCLA_PRETRAINED_ROOT=/mnt/weights
ENV CUBLAS_WORKSPACE_CONFIG=:4096:8

ENTRYPOINT ["/app/entrypoint.sh"]

FROM common-runtime AS common-slim
