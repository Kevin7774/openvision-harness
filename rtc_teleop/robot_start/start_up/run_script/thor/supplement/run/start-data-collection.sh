#!/usr/bin/env bash
set -Eeuo pipefail

DEPLOY_DIR="${DATA_COLLECTION_DEPLOY_DIR:-$HOME/deploy}"
VENV_DIR="$DEPLOY_DIR/.venv"
CONFIG_FILE="${DATA_COLLECTION_CONFIG:-$DEPLOY_DIR/data_collection.yaml}"
CONTROL_LAUNCHER="${ASTRABOT_CONTROL_LAUNCHER:-/opt/astrabot/run_control_node.sh}"

[[ -x "$VENV_DIR/bin/astra" ]] || {
  echo "错误：数采环境不存在，请先执行 install_data_collection.sh。" >&2
  exit 1
}
[[ -f "$CONFIG_FILE" ]] || {
  echo "错误：数采 YAML 不存在：$CONFIG_FILE" >&2
  exit 1
}

set +u
[[ -f /opt/ros/jazzy/setup.bash ]] && source /opt/ros/jazzy/setup.bash
[[ -f /opt/ros/astrabot/setup.bash ]] && source /opt/ros/astrabot/setup.bash
[[ -f /opt/ros/start_up/config/ros_config.sh ]] && \
  source /opt/ros/start_up/config/ros_config.sh
set -u

cd "$DEPLOY_DIR"
export ASTRABOT_CONTROL_LAUNCHER="$CONTROL_LAUNCHER"
exec "$VENV_DIR/bin/astra" run "$CONFIG_FILE"
