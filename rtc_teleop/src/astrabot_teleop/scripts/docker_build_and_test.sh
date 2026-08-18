#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODULE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROBOT_ROOT="$(cd "${MODULE_ROOT}/../.." && pwd)"
BASE_IMAGE="${ASTRABOT_SDK_BASE_IMAGE:-registry-01-registry.ap-southeast-1.cr.aliyuncs.com/robot/nx-ubuntu2404-x86@sha256:bd313d470a6bdacee057a920a56bb698fd888bcbe6281b8a498f4677fefe8e4e}"
IMAGE="${ASTRABOT_TELEOP_BUILD_IMAGE:-astrabot-teleop-jazzy-ci:local}"
RUN_ARM64="${ASTRABOT_TELEOP_RUN_ARM64:-auto}"
RTC_BACKEND="${ASTRABOT_TELEOP_RTC_BACKEND:-disabled}"
ARM64_LIBDATACHANNEL_PREFIX="${ASTRABOT_TELEOP_ARM64_LIBDATACHANNEL_PREFIX:-}"

if [[ "${RUN_ARM64}" != "auto" && "${RUN_ARM64}" != "0" && "${RUN_ARM64}" != "1" ]]; then
  echo "ASTRABOT_TELEOP_RUN_ARM64 must be auto, 0 or 1" >&2
  exit 2
fi
if [[ "${RUN_ARM64}" == "1" && ! -d /opt/astrabot_sdk_2404 ]]; then
  echo "ARM64 validation was required but /opt/astrabot_sdk_2404 is unavailable" >&2
  exit 1
fi
if [[ "${RTC_BACKEND}" != "disabled" && "${RTC_BACKEND}" != "libdatachannel" ]]; then
  echo "ASTRABOT_TELEOP_RTC_BACKEND must be disabled or libdatachannel" >&2
  exit 2
fi
if [[ "${RTC_BACKEND}" == "libdatachannel" && \
      (! -d "${ARM64_LIBDATACHANNEL_PREFIX}" || -L "${ARM64_LIBDATACHANNEL_PREFIX}") ]]; then
  echo "libdatachannel ARM64 SDK directory is required for production RTC cross build" >&2
  exit 1
fi

docker build \
  --build-arg "BASE_IMAGE=${BASE_IMAGE}" \
  --tag "${IMAGE}" \
  "${MODULE_ROOT}/docker"

docker_args=(
  run --rm
  --volume "${ROBOT_ROOT}:/workspace/robot"
  --workdir /workspace/robot/robot_motion/astrabot_teleop
)
if [[ -d /opt/astrabot_sdk_2404 ]]; then
  docker_args+=(--volume /opt/astrabot_sdk_2404:/opt/astrabot_sdk_2404:ro)
fi
docker_args+=(--env "ASTRABOT_TELEOP_RUN_ARM64=${RUN_ARM64}")
docker_args+=(--env "ASTRABOT_TELEOP_RTC_BACKEND=${RTC_BACKEND}")
if [[ "${RTC_BACKEND}" == "libdatachannel" ]]; then
  docker_args+=(--volume "${ARM64_LIBDATACHANNEL_PREFIX}:/opt/libdatachannel-arm64:ro")
  docker_args+=(--env "ASTRABOT_TELEOP_ARM64_LIBDATACHANNEL_PREFIX=/opt/libdatachannel-arm64")
fi

docker "${docker_args[@]}" "${IMAGE}" bash -lc '
  set -euo pipefail
  ./scripts/build_native.sh
  if [[ "${ASTRABOT_TELEOP_RUN_ARM64}" == "1" || \
        ("${ASTRABOT_TELEOP_RUN_ARM64}" == "auto" && -d /opt/astrabot_sdk_2404) ]]; then
    ./scripts/build_cross_arm64.sh
  else
    echo "skip ARM64 cross build: not requested for this job" >&2
  fi
'
