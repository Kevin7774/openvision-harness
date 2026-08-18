#!/usr/bin/env bash
set -Eeuo pipefail

set +u
# 与 robot_start 生成的其他模块使用同一环境入口：Domain ID 来自 .bashrc，
# 未配置时由 environment.sh 回退到当前默认值。
source /opt/ros/start_up/run/environment.sh
source /opt/ros/start_up/config/ros_config.sh
source /opt/ros/astrabot/setup.bash
set -u

for device in /dev/l_arm_cam /dev/r_arm_cam; do
    if [[ ! -e "$device" ]]; then
        echo "wrist camera device is unavailable: $device" >&2
        exit 1
    fi
done

exec ros2 launch astrabot_wrist_camera wrist_camera.launch.xml
