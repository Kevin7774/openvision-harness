#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODULE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROBOT_ROOT="${ROBOT_WORKSPACE_ROOT:-$(cd "${MODULE_ROOT}/../.." && pwd)}"
RTC_ROOT="${ROBOT_ROOT}/robot_system/astrabot_rtc"
DATA_INTERFACES_ROOT="${ROBOT_ROOT}/robot_data/astrabot_data_interfaces"
BUILD_ROOT="${ASTRABOT_TELEOP_NATIVE_BUILD_ROOT:-${ROBOT_ROOT}/.colcon/astrabot_teleop/native}"

if [[ ! -e /.dockerenv ]]; then
  echo "native validation must run inside the astrabot_teleop Docker image" >&2
  exit 1
fi
if [[ ! -f "${RTC_ROOT}/package.xml" ]]; then
  echo "astrabot_rtc source is required: ${RTC_ROOT}" >&2
  exit 1
fi
if [[ ! -f "${DATA_INTERFACES_ROOT}/package.xml" ]]; then
  echo "astrabot_data_interfaces source is required: ${DATA_INTERFACES_ROOT}" >&2
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
set -u

"${SCRIPT_DIR}/check_format.sh"
rm -rf "${BUILD_ROOT}"
mkdir -p "${BUILD_ROOT}"

cd "${ROBOT_ROOT}"
colcon --log-base "${BUILD_ROOT}/log" build \
  --base-paths "${DATA_INTERFACES_ROOT}" "${RTC_ROOT}" "${MODULE_ROOT}" \
  --merge-install \
  --build-base "${BUILD_ROOT}/build" \
  --install-base "${BUILD_ROOT}/install" \
  --packages-select astrabot_data_interfaces astrabot_rtc astrabot_teleop \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON

"${MODULE_ROOT}/test/disabled_config_launch_test.sh" \
  "${BUILD_ROOT}/install" \
  "${BUILD_ROOT}/disabled-config-launch.log"

colcon --log-base "${BUILD_ROOT}/log" test \
  --base-paths "${DATA_INTERFACES_ROOT}" "${RTC_ROOT}" "${MODULE_ROOT}" \
  --merge-install \
  --build-base "${BUILD_ROOT}/build" \
  --install-base "${BUILD_ROOT}/install" \
  --packages-select astrabot_data_interfaces astrabot_rtc astrabot_teleop \
  --event-handlers console_direct+

colcon test-result --test-result-base "${BUILD_ROOT}/build" --verbose
cmake --build "${BUILD_ROOT}/build/astrabot_teleop" --target no_exceptions_check
test -x "${BUILD_ROOT}/install/lib/astrabot_teleop/astrabot_teleop_node"
