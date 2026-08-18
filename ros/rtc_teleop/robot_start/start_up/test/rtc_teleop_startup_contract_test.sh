#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
START_UP_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
RTC_SERVICE="$START_UP_DIR/run_script/thor/supplement/service/Astrabot_Rtc.service"
TELEOP_SERVICE="$START_UP_DIR/run_script/thor/supplement/service/Astrabot_Teleop.service"
DATA_COLLECTION_SERVICE="$START_UP_DIR/run_script/thor/supplement/service/Astrabot_Data_Collection.service"
RTC_RUNNER="$START_UP_DIR/run_script/thor/supplement/run/start-astrabot-rtc.sh"
TELEOP_RUNNER="$START_UP_DIR/run_script/thor/supplement/run/start-astrabot-teleop.sh"
RUNTIME_HELPER="$START_UP_DIR/run_script/thor/supplement/run/astrabot-rtc-teleop-runtime.sh"
ROBOT_WORKSPACE_ROOT="$(cd -- "$START_UP_DIR/../.." && pwd)"
RTC_PACKAGE_DIR="$ROBOT_WORKSPACE_ROOT/robot_system/astrabot_rtc"
RTC_PACKAGE_CONFIG="$RTC_PACKAGE_DIR/config/rtc.yaml"
TELEOP_PACKAGE_CONFIG="$ROBOT_WORKSPACE_ROOT/robot_motion/astrabot_teleop/config/teleop.yaml"
FAST_DDS_SHM_GATE="$START_UP_DIR/fastdds_shm_refresh_gate.sh"
LIVEKIT_SERVICE="$START_UP_DIR/run_script/thor/supplement/service/Astrabot_LiveKit.service"
LIVEKIT_SRV_SERVICE="$START_UP_DIR/run_script/thor/supplement/service/Astrabot_LiveKit_Srv.service"

Assert_Contains() {
    local file="$1"
    local expected="$2"

    grep -Fq -- "$expected" "$file" || {
        echo "契约缺失：$file 未包含 $expected" >&2
        return 1
    }
}

Assert_Not_Contains() {
    local file="$1"
    local unexpected="$2"

    if grep -Fq -- "$unexpected" "$file"; then
        echo "契约违规：$file 不应包含 $unexpected" >&2
        return 1
    fi
}

Assert_Yaml_Key_Set_Equals() {
    local expected_file="$1"
    local actual_file="$2"
    local output_dir="$3"

    mkdir -p "$output_dir"

    Extract_Yaml_Keys() {
        awk '
            /^[[:space:]]*#/ || /^[[:space:]]*$/ { next }
            {
                match($0, /^[ ]*/)
                indent = RLENGTH
                line = substr($0, indent + 1)
                if (line !~ /^[A-Za-z0-9_]+[[:space:]]*:/) { next }
                key = line
                sub(/[[:space:]]*:.*$/, "", key)
                for (level in path) {
                    if (level >= indent) delete path[level]
                }
                path[indent] = key
                full = ""
                for (level = 0; level <= indent; level += 2) {
                    if (level in path) full = (full == "" ? path[level] : full "." path[level])
                }
                print full
            }
        ' "$1" | LC_ALL=C sort -u
    }

    Extract_Yaml_Keys "$expected_file" > "$output_dir/expected-keys"
    Extract_Yaml_Keys "$actual_file" > "$output_dir/actual-keys"
    diff -u "$output_dir/expected-keys" "$output_dir/actual-keys"
}

for script in \
    "$START_UP_DIR/execute.sh" \
    "$START_UP_DIR/install.sh" \
    "$START_UP_DIR/install_rtc_teleop_config.sh" \
    "$FAST_DDS_SHM_GATE" \
    "$START_UP_DIR/reload_auto_start_script.sh" \
    "$RTC_RUNNER" \
    "$TELEOP_RUNNER" \
    "$RUNTIME_HELPER"; do
    bash -n "$script"
done

[[ -f "$LIVEKIT_SERVICE" && -f "$LIVEKIT_SRV_SERVICE" ]] || {
    echo "LiveKit 回滚服务必须保留。" >&2
    exit 1
}

Assert_Contains "$RTC_SERVICE" "Wants=network-online.target time-sync.target"
Assert_Contains "$TELEOP_SERVICE" "Requires=Astrabot_Rtc.service"
Assert_Not_Contains "$RTC_SERVICE" "Astrabot_Gateway.service"
Assert_Not_Contains "$TELEOP_SERVICE" "Astrabot_Controller.service"
Assert_Not_Contains "$TELEOP_SERVICE" "Astrabot_Data_Collection.service"
Assert_Not_Contains "$RTC_SERVICE" "Requires=Astrabot_Gateway.service"
Assert_Contains "$RTC_SERVICE" "ExecStart=/bin/bash /opt/ros/start_up/run/start-astrabot-rtc.sh"
Assert_Contains "$TELEOP_SERVICE" "ExecStart=/bin/bash /opt/ros/start_up/run/start-astrabot-teleop.sh"
Assert_Contains "$RTC_SERVICE" "Environment=ASTRABOT_ROS_CONFIG=/opt/ros/start_up/config/ros_config.sh"
Assert_Contains "$TELEOP_SERVICE" "Environment=ASTRABOT_ROS_CONFIG=/opt/ros/start_up/config/ros_config.sh"
Assert_Contains "$RTC_RUNNER" 'ASTRABOT_STARTUP_ENVIRONMENT:-/opt/ros/start_up/run/environment.sh'
Assert_Contains "$TELEOP_RUNNER" 'ASTRABOT_STARTUP_ENVIRONMENT:-/opt/ros/start_up/run/environment.sh'
Assert_Contains "$RTC_SERVICE" "Environment=LD_LIBRARY_PATH=/opt/ros/astrabot/lib/astrabot_rtc/third_party"
Assert_Contains "$RTC_SERVICE" "EnvironmentFile=-/etc/astrabot/rtc-teleop.env"
Assert_Contains "$TELEOP_SERVICE" "EnvironmentFile=-/etc/astrabot/rtc-teleop.env"
Assert_Contains "$RTC_SERVICE" "StartLimitBurst=5"
Assert_Contains "$TELEOP_SERVICE" "StartLimitBurst=5"
Assert_Contains "$RTC_SERVICE" "TimeoutStopSec=15"
Assert_Contains "$TELEOP_SERVICE" "TimeoutStopSec=15"
Assert_Not_Contains "$RTC_SERVICE" "Conflicts=Astrabot_LiveKit"
Assert_Not_Contains "$TELEOP_SERVICE" "Conflicts=Astrabot_LiveKit"
Assert_Not_Contains "$DATA_COLLECTION_SERVICE" "Astrabot_LiveKit.service"
Assert_Not_Contains "$DATA_COLLECTION_SERVICE" "Astrabot_LiveKit_Srv.service"
Assert_Not_Contains "$DATA_COLLECTION_SERVICE" "wait-livekit-ready.sh"
Assert_Contains "$DATA_COLLECTION_SERVICE" "ExecStartPre=/usr/local/bin/wait-controller-node.sh"

Assert_Contains "$START_UP_DIR/execute.sh" "Is_Rtc_Teleop_Migration_Service"
Assert_Contains "$START_UP_DIR/execute.sh" "migration service requires explicit single-service startup"
Assert_Contains "$START_UP_DIR/install.sh" "Astrabot_Rtc.service"
Assert_Contains "$START_UP_DIR/install.sh" "Astrabot_Teleop.service"
Assert_Contains "$START_UP_DIR/install.sh" "cp install_rtc_teleop_config.sh"
Assert_Contains "$START_UP_DIR/install.sh" "copy supplement run file"
if [[ -f "$RTC_PACKAGE_DIR/CMakeLists.txt" ]]; then
    Assert_Contains "$RTC_PACKAGE_DIR/CMakeLists.txt" "libdatachannel.enabled"
fi
Assert_Contains "$START_UP_DIR/install.sh" 'FAST_DDS_SHM_GATE_SCRIPT" clean'
Assert_Contains "$START_UP_DIR/install.sh" 'FAST_DDS_SHM_GATE_SCRIPT" verify'
Assert_Contains "$START_UP_DIR/install.sh" 'sudo -u "$THE_USER" bash "$FAST_DDS_SHM_GATE_SCRIPT" verify'
Assert_Contains "$START_UP_DIR/install.sh" "Ensure_Astrabot_Data_Dirs"
Assert_Contains "$START_UP_DIR/install.sh" "/data/astrabot/file_transfer"
Assert_Contains "$START_UP_DIR/install.sh" "/data/astrabot/log/agent"
Assert_Contains "$START_UP_DIR/install.sh" "/data/astrabot/log/hub"
Assert_Contains "$START_UP_DIR/install.sh" 'sudo -u "$THE_USER" test -w "$data_dir"'
Assert_Contains "$START_UP_DIR/reload_auto_start_script.sh" "RTC_WAS_ENABLED"
Assert_Contains "$START_UP_DIR/reload_auto_start_script.sh" "TELEOP_WAS_ENABLED"
Assert_Contains "$START_UP_DIR/reload_auto_start_script.sh" "LIVEKIT_WAS_ENABLED"
Assert_Contains "$START_UP_DIR/reload_auto_start_script.sh" "LIVEKIT_SRV_WAS_ENABLED"
Assert_Not_Contains "$START_UP_DIR/reload_auto_start_script.sh" \
    'systemctl disable Astrabot_LiveKit.service Astrabot_LiveKit_Srv.service'
Assert_Contains "$START_UP_DIR/reload_auto_start_script.sh" 'FAST_DDS_SHM_GATE_SCRIPT" clean'
Assert_Contains "$START_UP_DIR/reload_auto_start_script.sh" 'FAST_DDS_SHM_GATE_SCRIPT" verify'
Assert_Contains "$START_UP_DIR/reload_auto_start_script.sh" 'sudo -S -u "$THE_USER"'

test_root="$(mktemp -d /tmp/astrabot-rtc-teleop-test.XXXXXX)"
trap 'rm -rf -- "$test_root"' EXIT

if command -v systemd-analyze >/dev/null 2>&1; then
    if ! systemd-analyze verify \
        "$RTC_SERVICE" \
        "$TELEOP_SERVICE" \
        > "$test_root/systemd-verify.log" 2>&1; then
        cat "$test_root/systemd-verify.log" >&2
        exit 1
    fi
fi

ASTRABOT_CONFIG_DIR="$test_root/etc/astrabot" \
ASTRABOT_CONFIG_OWNER=root \
ASTRABOT_CONFIG_GROUP=root \
    bash "$START_UP_DIR/install_rtc_teleop_config.sh"

rtc_config="$test_root/etc/astrabot/rtc.yaml"
teleop_config="$test_root/etc/astrabot/teleop.yaml"
gate_config="$test_root/etc/astrabot/rtc-teleop.env"
[[ "$(stat -c '%a' "$rtc_config")" == "640" ]]
[[ "$(stat -c '%a' "$teleop_config")" == "640" ]]
[[ "$(stat -c '%a' "$gate_config")" == "640" ]]
[[ "$(stat -c '%a' "$test_root/etc/astrabot")" == "750" ]]
Assert_Contains "$rtc_config" "backend: disabled"
Assert_Contains "$rtc_config" "enabled: false"
Assert_Contains "$teleop_config" "backend: disabled"
Assert_Contains "$teleop_config" "device_id: CHANGE_ME_BEFORE_SHADOW"
if grep -Eq '^[[:space:]]*grant_(key_ids|public_keys):' "$teleop_config"; then
    echo "disabled Teleop 样例不得写入 ROS 2 无法定型的空数组参数。" >&2
    exit 1
fi
Assert_Contains "$teleop_config" "production_command_topic: /astrabot/teleop/command"
Assert_Contains "$teleop_config" "run_context_topic: /astrabot/data_collection/run_context"
Assert_Contains "$teleop_config" "acquire_owner_service: /astrabot/arbitration/acquire_owner"
Assert_Contains "$teleop_config" "report_teleop_status_service: /astrabot/data_collection/report_teleop_status"
Assert_Contains "$teleop_config" "owner_ttl_ms: 150"
Assert_Contains "$teleop_config" "status_report_service_timeout_ms: 100"
Assert_Contains "$teleop_config" "status_report_retry_period_ms: 1000"
Assert_Contains "$teleop_config" "status_report_poll_period_ms: 10"
Assert_Contains "$teleop_config" "max_position_velocity_mps: 1.0"
Assert_Contains "$teleop_config" "max_position_acceleration_mps2: 5.0"
Assert_Contains "$gate_config" "ASTRABOT_RTC_PRODUCTION_ENABLED=0"
Assert_Contains "$gate_config" "ASTRABOT_TELEOP_CPP_ENABLED=0"
if [[ -f "$RTC_PACKAGE_CONFIG" && -f "$TELEOP_PACKAGE_CONFIG" ]]; then
    Assert_Yaml_Key_Set_Equals "$RTC_PACKAGE_CONFIG" "$rtc_config" "$test_root/rtc-key-contract"
    Assert_Yaml_Key_Set_Equals "$TELEOP_PACKAGE_CONFIG" "$teleop_config" "$test_root/teleop-key-contract"
else
    echo "跳过跨仓配置 key 检查：当前是 robot_start 独立 checkout。"
fi

printf '%s\n' '# preserve-existing-config' >> "$teleop_config"
ASTRABOT_CONFIG_DIR="$test_root/etc/astrabot" \
ASTRABOT_CONFIG_OWNER=root \
ASTRABOT_CONFIG_GROUP=root \
    bash "$START_UP_DIR/install_rtc_teleop_config.sh"
Assert_Contains "$teleop_config" "# preserve-existing-config"

mkdir -p "$test_root/existing-config-dir"
chmod 0711 "$test_root/existing-config-dir"
ASTRABOT_CONFIG_DIR="$test_root/existing-config-dir" \
ASTRABOT_CONFIG_OWNER=root \
ASTRABOT_CONFIG_GROUP=root \
    bash "$START_UP_DIR/install_rtc_teleop_config.sh"
[[ "$(stat -c '%a' "$test_root/existing-config-dir")" == "711" ]]

mkdir -p \
    "$test_root/installed/function" \
    "$test_root/installed/config/rtc" \
    "$test_root/installed/config/teleop"
cp "$START_UP_DIR/install_rtc_teleop_config.sh" "$test_root/installed/function/"
cp "$START_UP_DIR/run_script/thor/supplement/config/rtc/rtc.yaml.example" \
    "$test_root/installed/config/rtc/"
cp "$START_UP_DIR/run_script/thor/supplement/config/teleop/teleop.yaml.example" \
    "$test_root/installed/config/teleop/"
cp "$START_UP_DIR/run_script/thor/supplement/config/rtc/rtc-teleop.env.example" \
    "$test_root/installed/config/rtc/"
ASTRABOT_CONFIG_DIR="$test_root/installed-etc/astrabot" \
ASTRABOT_CONFIG_OWNER=root \
ASTRABOT_CONFIG_GROUP=root \
    bash "$test_root/installed/function/install_rtc_teleop_config.sh"
[[ -f "$test_root/installed-etc/astrabot/rtc.yaml" ]]
[[ -f "$test_root/installed-etc/astrabot/teleop.yaml" ]]
[[ -f "$test_root/installed-etc/astrabot/rtc-teleop.env" ]]

if ASTRABOT_CONFIG_DIR=/ \
    ASTRABOT_CONFIG_OWNER=root \
    ASTRABOT_CONFIG_GROUP=root \
        bash "$START_UP_DIR/install_rtc_teleop_config.sh"; then
    echo "配置安装器不得接受根目录目标。" >&2
    exit 1
fi

mkdir -p "$test_root/config-dir-target"
ln -s "$test_root/config-dir-target" "$test_root/config-dir-link"
if ASTRABOT_CONFIG_DIR="$test_root/config-dir-link" \
    ASTRABOT_CONFIG_OWNER=root \
    ASTRABOT_CONFIG_GROUP=root \
        bash "$START_UP_DIR/install_rtc_teleop_config.sh"; then
    echo "配置安装器不得接受符号链接目录。" >&2
    exit 1
fi

mkdir -p "$test_root/symlink-config"
touch "$test_root/symlink-target"
ln -s "$test_root/symlink-target" "$test_root/symlink-config/rtc.yaml"
if ASTRABOT_CONFIG_DIR="$test_root/symlink-config" \
    ASTRABOT_CONFIG_OWNER=root \
    ASTRABOT_CONFIG_GROUP=root \
        bash "$START_UP_DIR/install_rtc_teleop_config.sh"; then
    echo "配置安装器不得覆盖符号链接目标。" >&2
    exit 1
fi

mkdir -p "$test_root/bin"
cat > "$test_root/setup.bash" <<'EOF'
export PATH="${FAKE_ROS_BIN}:$PATH"
EOF
cat > "$test_root/environment.sh" <<'EOF'
export ROS_DOMAIN_ID=77
EOF
cat > "$test_root/ros_config.sh" <<'EOF'
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
EOF
cat > "$test_root/refresh_ros_config.sh" <<'EOF'
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
EOF
cat > "$test_root/board.yaml" <<'EOF'
board:
  resource_id: thor
  ros_namespace: /astrabot/thor
EOF
cat > "$test_root/bin/ros2" <<'EOF'
#!/usr/bin/env bash
printf '%s|domain=%s|rmw=%s\n' "$*" "${ROS_DOMAIN_ID:-}" "${RMW_IMPLEMENTATION:-}" >> "$ROS2_CAPTURE"
if [[ "${1:-}" == "topic" && "${2:-}" == "type" ]]; then
    printf '%s\n' diagnostic_msgs/msg/DiagnosticArray
fi
if [[ "${1:-}" == "service" && "${2:-}" == "type" && "${3:-}" == "${FAKE_MISSING_SERVICE:-__none__}" ]]; then
    exit 1
fi
EOF
cat > "$test_root/bin/systemctl" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' "$*" >> "$SYSTEMCTL_CAPTURE"
if [[ "${1:-}" == "show" ]]; then
    printf '\n'
fi
exit 0
EOF
cat > "$test_root/bin/sudo" <<'EOF'
#!/usr/bin/env bash
if [[ "${1:-}" == "-S" ]]; then
    shift
fi
exec "$@"
EOF
cat > "$test_root/bin/fastdds" <<'EOF'
#!/usr/bin/env bash
printf '%s\n' "$*" > "$FASTDDS_CAPTURE"
EOF
chmod 0755 "$test_root/bin/ros2"
chmod 0755 "$test_root/bin/systemctl"
chmod 0755 "$test_root/bin/sudo"
chmod 0755 "$test_root/bin/fastdds"
export ASTRABOT_ROS_CONFIG="$test_root/ros_config.sh"
export ASTRABOT_STARTUP_ENVIRONMENT="$test_root/environment.sh"

mkdir -p "$test_root/proc/123/fd" "$test_root/shm"
printf '%s\n' zed_wrapper > "$test_root/proc/123/comm"
printf 'Name:\tzed_wrapper\nPPid:\t1\n' > "$test_root/proc/123/status"
touch "$test_root/shm/fastrtps_port123"
ln -s "$test_root/shm/fastrtps_port123" "$test_root/proc/123/fd/4"
if ASTRABOT_PROC_ROOT="$test_root/proc" \
    ASTRABOT_FASTDDS_SHM_DIR="$test_root/shm" \
    ASTRABOT_FASTDDS_COMMAND="$test_root/bin/fastdds" \
    FASTDDS_CAPTURE="$test_root/fastdds-command" \
        bash "$FAST_DDS_SHM_GATE" clean; then
    echo "Fast DDS SHM Gate 不得在活跃映射存在时清理。" >&2
    exit 1
fi
[[ ! -e "$test_root/fastdds-command" ]]

find "$test_root/proc/123" -depth -delete
mkdir -p "$test_root/proc/124"
printf '%s\n' python3 > "$test_root/proc/124/comm"
printf 'Name:\tpython3\nPPid:\t1\n' > "$test_root/proc/124/status"
printf 'ROS_DISTRO=jazzy\0RMW_IMPLEMENTATION=rmw_fastrtps_cpp\0' > "$test_root/proc/124/environ"
if ASTRABOT_PROC_ROOT="$test_root/proc" \
    ASTRABOT_FASTDDS_SHM_DIR="$test_root/shm" \
    ASTRABOT_FASTDDS_COMMAND="$test_root/bin/fastdds" \
    FASTDDS_CAPTURE="$test_root/fastdds-command" \
        bash "$FAST_DDS_SHM_GATE" clean; then
    echo "Fast DDS SHM Gate 不得忽略未持有 fd 的活跃 ROS 进程。" >&2
    exit 1
fi
[[ ! -e "$test_root/fastdds-command" ]]

find "$test_root/proc/124" -depth -delete
ASTRABOT_PROC_ROOT="$test_root/proc" \
ASTRABOT_FASTDDS_SHM_DIR="$test_root/shm" \
ASTRABOT_FASTDDS_COMMAND="$test_root/bin/fastdds" \
FASTDDS_CAPTURE="$test_root/fastdds-command" \
    bash "$FAST_DDS_SHM_GATE" clean
Assert_Contains "$test_root/fastdds-command" "shm clean"

mkdir -p "$test_root/ros-prefix/bin"
cp "$test_root/bin/fastdds" "$test_root/ros-prefix/bin/fastdds"
rm -f "$test_root/fastdds-command"
ASTRABOT_PROC_ROOT="$test_root/proc" \
ASTRABOT_FASTDDS_SHM_DIR="$test_root/shm" \
ASTRABOT_ROS_DISTRO_PREFIX="$test_root/ros-prefix" \
FASTDDS_CAPTURE="$test_root/fastdds-command" \
PATH=/usr/bin:/bin \
    bash "$FAST_DDS_SHM_GATE" clean
Assert_Contains "$test_root/fastdds-command" "shm clean"

FAKE_ROS_BIN="$test_root/bin" \
ROS2_CAPTURE="$test_root/refresh-topic-command" \
ASTRABOT_ROS_SETUP="$test_root/setup.bash" \
ASTRABOT_REFRESH_VERIFY_TOPIC=/diagnostics \
ASTRABOT_REFRESH_VERIFY_TIMEOUT_SEC=2 \
ASTRABOT_ROS_CONFIG="$test_root/refresh_ros_config.sh" \
ASTRABOT_STARTUP_ENVIRONMENT="$test_root/environment.sh" \
    bash "$FAST_DDS_SHM_GATE" verify
Assert_Contains "$test_root/refresh-topic-command" \
    "topic echo /diagnostics diagnostic_msgs/msg/DiagnosticArray --full-length --once --no-daemon"
Assert_Contains "$test_root/refresh-topic-command" \
    "topic echo /astrabot/refresh_probe_"
Assert_Contains "$test_root/refresh-topic-command" \
    "topic pub /astrabot/refresh_probe_"
Assert_Not_Contains "$test_root/refresh-topic-command" \
    "local_fastdds_probe} --once --no-daemon"
Assert_Contains "$test_root/refresh-topic-command" "domain=77|rmw=rmw_fastrtps_cpp"

rm -f "$test_root/refresh-topic-command"
FAKE_ROS_BIN="$test_root/bin" \
ROS2_CAPTURE="$test_root/refresh-topic-command" \
ASTRABOT_ROS_SETUP="$test_root/setup.bash" \
ASTRABOT_REFRESH_VERIFY_TIMEOUT_SEC=2 \
ASTRABOT_ROS_CONFIG="$test_root/refresh_ros_config.sh" \
ASTRABOT_STARTUP_ENVIRONMENT="$test_root/environment.sh" \
ASTRABOT_BOARD_CONFIG="$test_root/board.yaml" \
    bash "$FAST_DDS_SHM_GATE" verify
Assert_Contains "$test_root/refresh-topic-command" \
    "topic echo /astrabot/thor/diagnostics/system diagnostic_msgs/msg/DiagnosticArray --full-length --once --no-daemon"
Assert_Contains "$test_root/refresh-topic-command" \
    "topic type /astrabot/thor/diagnostics/system --no-daemon"
Assert_Contains "$test_root/refresh-topic-command" \
    "topic echo /astrabot/refresh_probe_"
Assert_Contains "$test_root/refresh-topic-command" \
    "topic pub /astrabot/refresh_probe_"
Assert_Not_Contains "$test_root/refresh-topic-command" \
    "local_fastdds_probe} --once --no-daemon"
Assert_Contains "$test_root/refresh-topic-command" "domain=77|rmw=rmw_fastrtps_cpp"

mkdir -p "$test_root/start-up/service"
touch \
    "$test_root/start-up/service/Astrabot_Gateway.service" \
    "$test_root/start-up/service/Astrabot_Rtc.service" \
    "$test_root/start-up/service/Astrabot_Teleop.service"
cat > "$test_root/bin/astrabot_environment" <<EOF
START_UP_DIR='$test_root/start-up'
SERVICE_DIR='service'
LOG_DIR='log'
FUNCTION_DIR='function'
CONFIG_DIR='config'
THE_USER='astrabot'
THE_PASSWORD='test-only'
EOF

SYSTEMCTL_CAPTURE="$test_root/systemctl-start" \
PATH="$test_root/bin:$PATH" \
    bash "$START_UP_DIR/execute.sh" start > "$test_root/bulk-start.log"
Assert_Contains "$test_root/systemctl-start" "start Astrabot_Gateway.service"
Assert_Not_Contains "$test_root/systemctl-start" "start Astrabot_Rtc.service"
Assert_Not_Contains "$test_root/systemctl-start" "start Astrabot_Teleop.service"

SYSTEMCTL_CAPTURE="$test_root/systemctl-enable" \
PATH="$test_root/bin:$PATH" \
    bash "$START_UP_DIR/execute.sh" enable > "$test_root/bulk-enable.log"
Assert_Not_Contains "$test_root/systemctl-enable" "enable Astrabot_Rtc.service"
Assert_Not_Contains "$test_root/systemctl-enable" "enable Astrabot_Teleop.service"

SYSTEMCTL_CAPTURE="$test_root/systemctl-explicit" \
PATH="$test_root/bin:$PATH" \
    bash "$START_UP_DIR/execute.sh" start Astrabot_Teleop > "$test_root/explicit-start.log"
Assert_Contains "$test_root/systemctl-explicit" "start Astrabot_Teleop.service"

SYSTEMCTL_CAPTURE="$test_root/systemctl-stop" \
PATH="$test_root/bin:$PATH" \
    bash "$START_UP_DIR/execute.sh" stop > "$test_root/bulk-stop.log"
Assert_Contains "$test_root/systemctl-stop" "stop Astrabot_Rtc.service"
Assert_Contains "$test_root/systemctl-stop" "stop Astrabot_Teleop.service"

FAKE_ROS_BIN="$test_root/bin" \
ROS2_CAPTURE="$test_root/rtc-command" \
ASTRABOT_ROS_SETUP="$test_root/setup.bash" \
ASTRABOT_RTC_CONFIG="$rtc_config" \
    bash "$RTC_RUNNER"
Assert_Contains "$test_root/rtc-command" "launch astrabot_rtc rtc.launch.py"
Assert_Contains "$test_root/rtc-command" "rtc_config_path:=$rtc_config"
Assert_Contains "$test_root/rtc-command" "domain=77|rmw=rmw_fastrtps_cpp"

FAKE_ROS_BIN="$test_root/bin" \
ROS2_CAPTURE="$test_root/teleop-command" \
ASTRABOT_ROS_SETUP="$test_root/setup.bash" \
ASTRABOT_TELEOP_CONFIG="$teleop_config" \
    bash "$TELEOP_RUNNER"
Assert_Contains "$test_root/teleop-command" "launch astrabot_teleop teleop.launch.py"
Assert_Contains "$test_root/teleop-command" "config_file:=$teleop_config"
Assert_Contains "$test_root/teleop-command" "domain=77|rmw=rmw_fastrtps_cpp"

sed 's/device_id: CHANGE_ME_BEFORE_SHADOW/device_id: ""/' "$teleop_config" > "$test_root/disabled-empty-device.yaml"
FAKE_ROS_BIN="$test_root/bin" \
ROS2_CAPTURE="$test_root/disabled-empty-device-command" \
ASTRABOT_ROS_SETUP="$test_root/setup.bash" \
ASTRABOT_TELEOP_CONFIG="$test_root/disabled-empty-device.yaml" \
    bash "$TELEOP_RUNNER"
Assert_Contains "$test_root/disabled-empty-device-command" "launch astrabot_teleop teleop.launch.py"

sed 's/backend: disabled/backend: libdatachannel/' "$rtc_config" > "$test_root/unsafe-rtc.yaml"
if FAKE_ROS_BIN="$test_root/bin" \
    ROS2_CAPTURE="$test_root/unsafe-rtc-command" \
    ASTRABOT_ROS_SETUP="$test_root/setup.bash" \
    ASTRABOT_RTC_CONFIG="$test_root/unsafe-rtc.yaml" \
        bash "$RTC_RUNNER"; then
    echo "RTC runner 不得接受未开放的 transport backend。" >&2
    exit 1
fi

mkdir -p "$test_root/runtime" "$test_root/capabilities"
touch "$test_root/runtime/libdatachannel.so.0.24.2"
printf '%s\n' 'backend=libdatachannel' > "$test_root/capabilities/libdatachannel.enabled"
FAKE_ROS_BIN="$test_root/bin" \
ROS2_CAPTURE="$test_root/production-rtc-command" \
ASTRABOT_ROS_SETUP="$test_root/setup.bash" \
ASTRABOT_RTC_CONFIG="$test_root/unsafe-rtc.yaml" \
ASTRABOT_RTC_PRODUCTION_ENABLED=1 \
ASTRABOT_RTC_LIBDATACHANNEL_LIBRARY="$test_root/runtime/libdatachannel.so.0.24.2" \
ASTRABOT_RTC_BACKEND_CAPABILITY="$test_root/capabilities/libdatachannel.enabled" \
    bash "$RTC_RUNNER"
Assert_Contains "$test_root/production-rtc-command" "launch astrabot_rtc rtc.launch.py"

sed 's/enabled: false/enabled: true/' "$rtc_config" > "$test_root/misleading-media.yaml"
if FAKE_ROS_BIN="$test_root/bin" \
    ROS2_CAPTURE="$test_root/misleading-media-command" \
    ASTRABOT_ROS_SETUP="$test_root/setup.bash" \
    ASTRABOT_RTC_CONFIG="$test_root/misleading-media.yaml" \
        bash "$RTC_RUNNER"; then
    echo "disabled RTC runner 不得宣称 media enabled。" >&2
    exit 1
fi

sed 's/backend: disabled/backend: cpp/' "$teleop_config" > "$test_root/unsafe-teleop.yaml"
if FAKE_ROS_BIN="$test_root/bin" \
    ROS2_CAPTURE="$test_root/unsafe-teleop-command" \
    ASTRABOT_ROS_SETUP="$test_root/setup.bash" \
    ASTRABOT_TELEOP_CONFIG="$test_root/unsafe-teleop.yaml" \
        bash "$TELEOP_RUNNER"; then
    echo "Teleop runner 不得接受生产 cpp backend。" >&2
    exit 1
fi

sed 's/backend: disabled/backend: shadow/' "$teleop_config" > "$test_root/unconfigured-shadow.yaml"
if FAKE_ROS_BIN="$test_root/bin" \
    ROS2_CAPTURE="$test_root/unconfigured-shadow-command" \
    ASTRABOT_ROS_SETUP="$test_root/setup.bash" \
    ASTRABOT_TELEOP_CONFIG="$test_root/unconfigured-shadow.yaml" \
        bash "$TELEOP_RUNNER"; then
    echo "Teleop shadow runner 不得接受占位 device_id。" >&2
    exit 1
fi


sed \
    -e 's/backend: disabled/backend: shadow/' \
    -e 's/device_id: CHANGE_ME_BEFORE_SHADOW/device_id: thor-test/' \
    -e '/robot_base_frame: base_link/a\    grant_key_ids: [test-key]\n    grant_public_keys: [AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=]' \
    "$teleop_config" > "$test_root/configured-shadow.yaml"
FAKE_ROS_BIN="$test_root/bin" \
ROS2_CAPTURE="$test_root/configured-shadow-command" \
ASTRABOT_ROS_SETUP="$test_root/setup.bash" \
ASTRABOT_TELEOP_CONFIG="$test_root/configured-shadow.yaml" \
    bash "$TELEOP_RUNNER"
Assert_Contains "$test_root/configured-shadow-command" "launch astrabot_teleop teleop.launch.py"

sed 's/backend: shadow/backend: cpp/' "$test_root/configured-shadow.yaml" > "$test_root/configured-cpp.yaml"
if FAKE_ROS_BIN="$test_root/bin" \
    ROS2_CAPTURE="$test_root/cpp-without-gate-command" \
    ASTRABOT_ROS_SETUP="$test_root/setup.bash" \
    ASTRABOT_TELEOP_CONFIG="$test_root/configured-cpp.yaml" \
    ASTRABOT_RTC_CONFIG="$test_root/unsafe-rtc.yaml" \
        bash "$TELEOP_RUNNER"; then
    echo "Teleop cpp runner 不得绕过显式生产 Gate。" >&2
    exit 1
fi

if FAKE_ROS_BIN="$test_root/bin" \
    ROS2_CAPTURE="$test_root/cpp-missing-service-command" \
    FAKE_MISSING_SERVICE=/astrabot/data_collection/report_teleop_status \
    ASTRABOT_ROS_SETUP="$test_root/setup.bash" \
    ASTRABOT_TELEOP_CONFIG="$test_root/configured-cpp.yaml" \
    ASTRABOT_RTC_CONFIG="$test_root/unsafe-rtc.yaml" \
    ASTRABOT_TELEOP_CPP_ENABLED=1 \
        bash "$TELEOP_RUNNER"; then
    echo "Teleop cpp runner 不得在生产依赖 service 缺失时启动。" >&2
    exit 1
fi

FAKE_ROS_BIN="$test_root/bin" \
ROS2_CAPTURE="$test_root/configured-cpp-command" \
ASTRABOT_ROS_SETUP="$test_root/setup.bash" \
ASTRABOT_TELEOP_CONFIG="$test_root/configured-cpp.yaml" \
ASTRABOT_RTC_CONFIG="$test_root/unsafe-rtc.yaml" \
ASTRABOT_TELEOP_CPP_ENABLED=1 \
    bash "$TELEOP_RUNNER"
Assert_Contains "$test_root/configured-cpp-command" "launch astrabot_teleop teleop.launch.py"
Assert_Contains "$test_root/configured-cpp-command" "service type /astrabot/arbitration/acquire_owner"
Assert_Contains "$test_root/configured-cpp-command" "service type /astrabot/arbitration/renew_owner"
Assert_Contains "$test_root/configured-cpp-command" "service type /astrabot/arbitration/release_owner"
Assert_Contains "$test_root/configured-cpp-command" "service type /astrabot/data_collection/report_teleop_status"

echo "RTC/Teleop robot_start contract tests passed."
