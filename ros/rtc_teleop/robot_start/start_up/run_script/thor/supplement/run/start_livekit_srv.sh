#!/usr/bin/env bash
set -Eeuo pipefail

DEPLOY_DIR="${DATA_COLLECTION_DEPLOY_DIR:-/home/astrabot/deploy}"
VENV_DIR="$DEPLOY_DIR/.venv"
LIVEKIT_ENV_FILE="${LIVEKIT_ENV_FILE:-$DEPLOY_DIR/livekit.env}"

[[ -x "$VENV_DIR/bin/python" ]] || {
  echo "错误：数采 Python 环境不存在：$VENV_DIR" >&2
  exit 1
}
[[ -f "$LIVEKIT_ENV_FILE" ]] || {
  echo "错误：LiveKit 配置不存在：$LIVEKIT_ENV_FILE" >&2
  exit 1
}

set -a
# shellcheck disable=SC1090
source "$LIVEKIT_ENV_FILE"
set +a

: "${LIVEKIT_API_KEY:?错误：请在 $LIVEKIT_ENV_FILE 中填写 LIVEKIT_API_KEY}"
: "${LIVEKIT_API_SECRET:?错误：请在 $LIVEKIT_ENV_FILE 中填写 LIVEKIT_API_SECRET}"
if [[ "$LIVEKIT_API_KEY" == \<*\> || "$LIVEKIT_API_SECRET" == \<*\> ]]; then
  echo "错误：请替换 $LIVEKIT_ENV_FILE 中的 LiveKit 密钥占位符" >&2
  exit 1
fi

"$VENV_DIR/bin/python" -c \
  'import robot_teleop.livekit.streaming.livekit_server' || {
  echo "错误：Python 环境中缺少 robot_teleop LiveKit token server 模块" >&2
  exit 1
}

cd "$DEPLOY_DIR"
exec "$VENV_DIR/bin/python" -m robot_teleop.livekit.streaming.livekit_server
