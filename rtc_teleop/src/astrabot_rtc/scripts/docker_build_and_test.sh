#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODULE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE="${ASTRABOT_RTC_SDK_IMAGE:-registry-01-registry.ap-southeast-1.cr.aliyuncs.com/robot/nx-ubuntu2404-x86:latest}"

docker run --rm \
  --volume "${MODULE_ROOT}:/workspace/src/astrabot_rtc:ro" \
  --workdir /workspace \
  "${IMAGE}" \
  bash -lc '
    set -eo pipefail
    source /opt/ros/jazzy/setup.bash
    set -u
    colcon build \
      --base-paths /workspace/src/astrabot_rtc \
      --packages-select astrabot_rtc \
      --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
    test ! -e /workspace/install/astrabot_rtc/share/astrabot_rtc/capabilities/libdatachannel.enabled
    set +u
    source /workspace/install/setup.bash
    set -u
    ctest --test-dir /workspace/build/astrabot_rtc --output-on-failure
    cmake --build /workspace/build/astrabot_rtc --target no_exceptions_check
    set +e
    timeout --signal=INT 3s ros2 run astrabot_rtc astrabot_rtc_node \
      --ros-args -p rtc_config_path:=/workspace/src/astrabot_rtc/config/rtc.yaml
    smoke_status=$?
    set -e
    if [[ "${smoke_status}" -ne 0 && "${smoke_status}" -ne 124 ]]; then
      echo "astrabot_rtc disabled-backend smoke test failed: status=${smoke_status}" >&2
      exit "${smoke_status}"
    fi
  '
