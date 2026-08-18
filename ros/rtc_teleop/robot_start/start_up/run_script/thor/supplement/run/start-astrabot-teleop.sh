#!/usr/bin/env bash
set -Eeuo pipefail

CONFIG_FILE="${ASTRABOT_TELEOP_CONFIG:-/etc/astrabot/teleop.yaml}"
RTC_CONFIG_FILE="${ASTRABOT_RTC_CONFIG:-/etc/astrabot/rtc.yaml}"
ROS_SETUP_FILE="${ASTRABOT_ROS_SETUP:-/opt/ros/astrabot/setup.bash}"
STARTUP_ENVIRONMENT_FILE="${ASTRABOT_STARTUP_ENVIRONMENT:-/opt/ros/start_up/run/environment.sh}"
ROS_CONFIG_FILE="${ASTRABOT_ROS_CONFIG:-/opt/ros/start_up/config/ros_config.sh}"
RUNTIME_HELPER="${ASTRABOT_RTC_TELEOP_RUNTIME_HELPER:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/astrabot-rtc-teleop-runtime.sh}"

[[ -r "$RUNTIME_HELPER" ]] || {
    echo "错误：RTC/Teleop 启动契约脚本不存在或不可读：$RUNTIME_HELPER" >&2
    exit 1
}
# shellcheck disable=SC1090
source "$RUNTIME_HELPER"
Require_Readable_File "Teleop 配置" "$CONFIG_FILE"

backend="$(Read_Unique_Yaml_Scalar backend "$CONFIG_FILE")"
case "$backend" in
    disabled)
        ;;
    shadow|cpp)
        device_id="$(Read_Unique_Yaml_Scalar device_id "$CONFIG_FILE")"
        if [[ -z "$device_id" || "$device_id" == "CHANGE_ME_BEFORE_SHADOW" ]]; then
            echo "错误：切换 $backend 前必须配置真实 device_id。" >&2
            exit 1
        fi
        Validate_Teleop_Grant_Keys "$CONFIG_FILE"
        if [[ "$backend" == "cpp" ]]; then
            Require_Production_Gate ASTRABOT_TELEOP_CPP_ENABLED
            Require_Readable_File "RTC 配置" "$RTC_CONFIG_FILE"
            rtc_backend="$(Read_Unique_Yaml_Scalar backend "$RTC_CONFIG_FILE")"
            [[ "$rtc_backend" == "libdatachannel" ]] || {
                echo "错误：Teleop cpp backend 要求 RTC transport.backend=libdatachannel。" >&2
                exit 1
            }
        fi
        ;;
    *)
        echo "错误：Teleop backend 只允许 disabled|shadow|cpp，实际值为：$backend" >&2
        exit 1
        ;;
esac

Load_Astrabot_Ros_Environment "$STARTUP_ENVIRONMENT_FILE" "$ROS_CONFIG_FILE" "$ROS_SETUP_FILE"
if [[ "$backend" == "cpp" ]]; then
    Require_Ros_Service "$(Read_Unique_Yaml_Scalar acquire_owner_service "$CONFIG_FILE")"
    Require_Ros_Service "$(Read_Unique_Yaml_Scalar renew_owner_service "$CONFIG_FILE")"
    Require_Ros_Service "$(Read_Unique_Yaml_Scalar release_owner_service "$CONFIG_FILE")"
    Require_Ros_Service "$(Read_Unique_Yaml_Scalar report_teleop_status_service "$CONFIG_FILE")"
fi

echo "启动 astrabot_teleop：backend=$backend。"
exec ros2 launch astrabot_teleop teleop.launch.py config_file:="$CONFIG_FILE"
