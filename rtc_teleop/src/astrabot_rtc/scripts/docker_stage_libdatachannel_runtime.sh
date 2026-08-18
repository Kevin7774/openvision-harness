#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODULE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE="${ASTRABOT_RTC_SDK_IMAGE:-registry-01-registry.ap-southeast-1.cr.aliyuncs.com/robot/nx-ubuntu2404-x86@sha256:bd313d470a6bdacee057a920a56bb698fd888bcbe6281b8a498f4677fefe8e4e}"

usage() {
  echo "usage: $0 <validated-libdatachannel-sdk-prefix> <existing-empty-output-directory>" >&2
}

fail() {
  echo "error: $*" >&2
  exit 1
}

if [[ "$#" -ne 2 ]]; then
  usage
  exit 2
fi
command -v docker >/dev/null 2>&1 || fail "docker is required"
command -v realpath >/dev/null 2>&1 || fail "realpath is required"

sdk_prefix="$(realpath -- "$1")"
output_root="$(realpath -- "$2")"
[[ -d "${sdk_prefix}" && ! -L "$1" ]] || fail "SDK prefix must be a non-symlink directory"
[[ -d "${output_root}" && ! -L "$2" ]] || fail "output must be a non-symlink directory"

docker run --rm --network none \
  --volume "${MODULE_ROOT}:/workspace/src/astrabot_rtc:ro" \
  --volume "${sdk_prefix}:/input/libdatachannel:ro" \
  --volume "${output_root}:/output" \
  --workdir /workspace/src/astrabot_rtc \
  "${IMAGE}" \
  ./scripts/stage_libdatachannel_runtime.sh /input/libdatachannel /output

docker run --rm --network none \
  --volume "${output_root}:/runtime:ro" \
  "${IMAGE}" \
  bash -lc '
    set -euo pipefail
    cd /runtime
    sha256sum --check share/astrabot_rtc/third_party/libdatachannel/runtime-package.sha256
    library=lib/astrabot_rtc/third_party/libdatachannel.so.0.24.2
    LC_ALL=C readelf -h "${library}" | grep -q "Machine:.*AArch64"
    LC_ALL=C readelf -d "${library}" | grep -q "SONAME.*\[libdatachannel.so.0.24\]"
    [[ "$(readlink lib/astrabot_rtc/third_party/libdatachannel.so.0.24)" == \
      "libdatachannel.so.0.24.2" ]]
  '
