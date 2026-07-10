#!/usr/bin/env bash
set -euo pipefail

CARLA_HOME="${CARLA_HOME:-${HOME:-/tmp/pcla-carla-home}}"
export HOME="${CARLA_HOME}"
export XDG_CACHE_HOME="${PCLA_XDG_CACHE_HOME:-${CARLA_HOME}/.cache}"
mkdir -p "${HOME}/carlaCache" "${XDG_CACHE_HOME}"

export PCLA_PRETRAINED_ROOT="${PCLA_PRETRAINED_ROOT:-/mnt/weights}"
export CUBLAS_WORKSPACE_CONFIG="${CUBLAS_WORKSPACE_CONFIG:-:4096:8}"

if [[ "${PCLA_PRETRAINED_ROOT}" != "/mnt/weights" ]]; then
    echo "PCLA_PRETRAINED_ROOT must be /mnt/weights; mount the selected weight directory there." >&2
    exit 1
fi

if (( $# > 0 )); then
    exec "$@"
fi

exec /opt/pcla-venv/bin/python -m pcla_wrapper.server
