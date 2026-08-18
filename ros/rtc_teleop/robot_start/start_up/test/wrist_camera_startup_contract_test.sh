#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
START_UP_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
CAMERA_SERVICE="$START_UP_DIR/run_script/thor/supplement/service/Astrabot_Wrist_Camera.service"
CAMERA_RUNNER="$START_UP_DIR/run_script/thor/supplement/run/start-astrabot-wrist-camera.sh"
RECORDER_REPO="$(cd -- "$START_UP_DIR/../../robot_data/astrabot_data_collection" && pwd)"
RECORDER_SERVICE="$RECORDER_REPO/systemd/Astrabot_Recorder.service"
RECORDER_CONFIG="$RECORDER_REPO/astrabot_recorder/config/recorder.yaml"

assert_contains() {
    local file="$1"
    local expected="$2"
    grep -Fq -- "$expected" "$file" || {
        echo "contract missing: $file does not contain $expected" >&2
        return 1
    }
}

assert_not_contains() {
    local file="$1"
    local unexpected="$2"
    if grep -Fq -- "$unexpected" "$file"; then
        echo "contract violation: $file contains $unexpected" >&2
        return 1
    fi
}

bash -n "$CAMERA_RUNNER"

assert_contains "$CAMERA_SERVICE" "Description=AstraBot Wrist Camera"
assert_not_contains "$CAMERA_SERVICE" "After=multi-user.target"
assert_contains "$CAMERA_SERVICE" "ConditionPathExists=/dev/l_arm_cam"
assert_contains "$CAMERA_SERVICE" "ConditionPathExists=/dev/r_arm_cam"
assert_contains "$CAMERA_SERVICE" "Before=Astrabot_Recorder.service"
assert_contains "$CAMERA_SERVICE" "StartLimitBurst=5"
assert_contains "$CAMERA_SERVICE" "ExecStart=/bin/bash /opt/ros/start_up/run/start-astrabot-wrist-camera.sh"
assert_contains "$CAMERA_SERVICE" "KillSignal=SIGINT"
assert_contains "$CAMERA_SERVICE" "KillMode=control-group"
assert_contains "$CAMERA_RUNNER" "ros2 launch astrabot_wrist_camera wrist_camera.launch.xml"
assert_contains "$CAMERA_RUNNER" "set +u"
assert_contains "$CAMERA_RUNNER" "source /opt/ros/start_up/run/environment.sh"
assert_contains "$CAMERA_RUNNER" "source /opt/ros/start_up/config/ros_config.sh"
assert_not_contains "$CAMERA_RUNNER" "export ROS_DOMAIN_ID="
assert_not_contains "$CAMERA_SERVICE" "Environment=ROS_DOMAIN_ID="
assert_contains "$START_UP_DIR/install.sh" "copy supplement service file"
assert_contains "$START_UP_DIR/install.sh" "copy supplement run file"
assert_contains "$START_UP_DIR/reload_auto_start_script.sh" "copy supplement service files"
assert_contains "$START_UP_DIR/reload_auto_start_script.sh" "copy supplement run files"

# Camera publisher and Recorder subscriber must remain in the same DDS domain and lifecycle.
assert_contains "$RECORDER_SERVICE" "Requires=Astrabot_Wrist_Camera.service"
assert_contains "$RECORDER_SERVICE" "BindsTo=Astrabot_Wrist_Camera.service"
assert_contains "$RECORDER_SERVICE" "source /opt/ros/start_up/run/environment.sh"
assert_contains "$RECORDER_SERVICE" "source /opt/ros/start_up/config/ros_config.sh"
assert_contains "$RECORDER_SERVICE" "ProtectHome=read-only"
assert_not_contains "$RECORDER_SERVICE" "Environment=ROS_DOMAIN_ID="
assert_contains "$RECORDER_CONFIG" "native_fps.observation_images_left_wrist: 15.0"
assert_contains "$RECORDER_CONFIG" "native_fps.observation_images_right_wrist: 15.0"
assert_not_contains "$RECORDER_CONFIG" "observation_images_chest"

if command -v systemd-analyze >/dev/null 2>&1; then
    # Data Collection 依赖目标机已有的 /usr/local/bin 与 /opt/ros 可执行文件；
    # 在隔离 CI 容器中只对本次新增的 unit 做 systemd 语法验证。
    systemd-analyze verify "$CAMERA_SERVICE"
fi
