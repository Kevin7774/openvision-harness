#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODULE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BASE_IMAGE="${ASTRABOT_RTC_SDK_IMAGE:-registry-01-registry.ap-southeast-1.cr.aliyuncs.com/robot/nx-ubuntu2404-x86@sha256:bd313d470a6bdacee057a920a56bb698fd888bcbe6281b8a498f4677fefe8e4e}"
IMAGE="${ASTRABOT_RTC_FORMAT_IMAGE:-astrabot-rtc-format-ci:local}"

docker build --build-arg "BASE_IMAGE=${BASE_IMAGE}" --tag "${IMAGE}" "${MODULE_ROOT}/docker"
docker run --rm \
  --volume "${MODULE_ROOT}:/workspace/src/astrabot_rtc:ro" \
  --workdir /workspace/src/astrabot_rtc \
  "${IMAGE}" \
  ./scripts/check_format.sh
