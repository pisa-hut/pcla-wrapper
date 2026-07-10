#!/usr/bin/env bash
set -euo pipefail

readonly ARCHIVE_URL="${PCLA_WEIGHTS_URL:-https://huggingface.co/datasets/MasoudJTehrani/PCLA/resolve/main/pretrained.zip}"
readonly ARCHIVE_SHA256="${PCLA_WEIGHTS_SHA256:-0d02c1aaf9ea81b892fef8815c1a8ab617c1906b89ee984ba8163332d659fa93}"
readonly DESTINATION="${1:-/opt/pisa/weights}"
readonly ARCHIVE="${PCLA_WEIGHTS_ARCHIVE:-/tmp/pcla_weights.zip}"

command -v curl >/dev/null
command -v sha256sum >/dev/null
command -v unzip >/dev/null

mkdir -p "${DESTINATION}"

echo "Downloading PCLA weights to ${ARCHIVE}"
curl -fL --retry 5 --retry-delay 5 --continue-at - \
    --output "${ARCHIVE}" "${ARCHIVE_URL}"

echo "${ARCHIVE_SHA256}  ${ARCHIVE}" | sha256sum --check -

echo "Extracting PCLA weights to ${DESTINATION}"
unzip -n -q "${ARCHIVE}" -d "${DESTINATION}"

if [[ "${PCLA_KEEP_WEIGHTS_ARCHIVE:-0}" != "1" ]]; then
    rm -f "${ARCHIVE}"
fi

cat <<EOF
PCLA weights are ready under ${DESTINATION}.

For the default PISA pcla AV row, keep the selected PlanT directory at:
  ${DESTINATION}/plant_pretrained

The executor mounts that directory into the AV container at:
  /mnt/weights
EOF
