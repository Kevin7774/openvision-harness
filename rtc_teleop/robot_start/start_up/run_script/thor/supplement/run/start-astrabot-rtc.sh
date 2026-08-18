#!/usr/bin/env bash
set -Eeuo pipefail

CONFIG_FILE="${ASTRABOT_RTC_CONFIG:-/etc/astrabot/rtc.yaml}"
ROS_SETUP_FILE="${ASTRABOT_ROS_SETUP:-/opt/ros/astrabot/setup.bash}"
STARTUP_ENVIRONMENT_FILE="${ASTRABOT_STARTUP_ENVIRONMENT:-/opt/ros/start_up/run/environment.sh}"
ROS_CONFIG_FILE="${ASTRABOT_ROS_CONFIG:-/opt/ros/start_up/config/ros_config.sh}"
RUNTIME_HELPER="${ASTRABOT_RTC_TELEOP_RUNTIME_HELPER:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/astrabot-rtc-teleop-runtime.sh}"
LIBDATACHANNEL_LIBRARY="${ASTRABOT_RTC_LIBDATACHANNEL_LIBRARY:-/opt/ros/astrabot/lib/astrabot_rtc/third_party/libdatachannel.so.0.24.2}"
BACKEND_CAPABILITY="${ASTRABOT_RTC_BACKEND_CAPABILITY:-/opt/ros/astrabot/share/astrabot_rtc/capabilities/libdatachannel.enabled}"

[[ -r "$RUNTIME_HELPER" ]] || {
    echo "错误：RTC/Teleop 启动契约脚本不存在或不可读：$RUNTIME_HELPER" >&2
    exit 1
}
# shellcheck disable=SC1090
source "$RUNTIME_HELPER"
Require_Readable_File "RTC 配置" "$CONFIG_FILE"

backend="$(Read_Unique_Yaml_Scalar backend "$CONFIG_FILE")"
media_enabled="$(Read_Unique_Yaml_Scalar enabled "$CONFIG_FILE")"
case "$backend" in
    disabled)
        [[ "$media_enabled" == "false" ]] || {
            echo "错误：disabled RTC backend 必须配置 media.enabled=false，实际值为：$media_enabled" >&2
            exit 1
        }
        ;;
    libdatachannel)
        Require_Production_Gate ASTRABOT_RTC_PRODUCTION_ENABLED
        Require_Readable_File "libdatachannel runtime" "$LIBDATACHANNEL_LIBRARY"
        Require_Readable_File "RTC libdatachannel 构建能力标记" "$BACKEND_CAPABILITY"
        [[ "$media_enabled" == "true" || "$media_enabled" == "false" ]] || {
            echo "错误：media.enabled 必须是 true 或 false，实际值为：$media_enabled" >&2
            exit 1
        }
        ;;
    *)
        echo "错误：RTC transport.backend 只允许 disabled|libdatachannel，实际值为：$backend" >&2
        exit 1
        ;;
esac

Load_Astrabot_Ros_Environment "$STARTUP_ENVIRONMENT_FILE" "$ROS_CONFIG_FILE" "$ROS_SETUP_FILE"

echo "启动 astrabot_rtc：backend=$backend，media=$media_enabled。"
exec ros2 launch astrabot_rtc rtc.launch.py rtc_config_path:="$CONFIG_FILE"
