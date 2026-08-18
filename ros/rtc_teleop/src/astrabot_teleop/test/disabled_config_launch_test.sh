#!/usr/bin/env bash
set -euo pipefail

install_root="$1"
log_file="$2"

set +u
# shellcheck disable=SC1090
source "${install_root}/setup.bash"
set -u

set +e
timeout --signal=INT --kill-after=2s 3s \
  ros2 launch astrabot_teleop teleop.launch.py \
  >"${log_file}" 2>&1
launch_status=$?
set -e

if [[ "${launch_status}" -ne 124 ]]; then
  cat "${log_file}" >&2
  echo "disabled 配置未能保持 Teleop launch 运行，status=${launch_status}" >&2
  exit 1
fi

if grep -Fq "parameter_value_from failed" "${log_file}"; then
  cat "${log_file}" >&2
  echo "disabled 配置仍包含 ROS 2 无法定型的空数组参数。" >&2
  exit 1
fi
