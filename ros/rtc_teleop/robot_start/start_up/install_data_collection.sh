#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_SOURCE_CONFIG="$SCRIPT_DIR/run_script/thor/supplement/config/data_collection/astrabot.yaml"
EXAMPLE_CONFIG="${DEFAULT_SOURCE_CONFIG}.example"
LIVEKIT_EXAMPLE_CONFIG="$SCRIPT_DIR/run_script/thor/supplement/config/livekit/livekit.env.example"

if [[ $# -gt 1 ]]; then
  echo "用法：$0 [data_collection.yaml]" >&2
  echo "默认配置位置：$DEFAULT_SOURCE_CONFIG" >&2
  echo "可从同目录的 astrabot.yaml.example 复制后填写；真实密钥不要提交到 Git。" >&2
  exit 2
fi

DATA_COLLECTION_USER="${DATA_COLLECTION_USER:-astrabot}"
DATA_COLLECTION_HOME="${DATA_COLLECTION_HOME:-/home/$DATA_COLLECTION_USER}"
DEPLOY_DIR="${DATA_COLLECTION_DEPLOY_DIR:-$DATA_COLLECTION_HOME/deploy}"
VENV_DIR="$DEPLOY_DIR/.venv"
CONFIG_FILE="$DEPLOY_DIR/data_collection.yaml"
LIVEKIT_CONFIG_FILE="$DEPLOY_DIR/livekit.env"
PYPI_URL="${ASTRA_PYPI_URL:-https://nexus-public.astrabot.com/repository/pypi-internal/simple/}"

SOURCE_CONFIG=""
INSTALL_CONFIG=false
if [[ $# -eq 1 ]]; then
  SOURCE_CONFIG="$1"
  [[ -f "$SOURCE_CONFIG" ]] || {
    echo "错误：配置文件不存在：$SOURCE_CONFIG" >&2
    exit 2
  }
  INSTALL_CONFIG=true
elif [[ -f "$DEFAULT_SOURCE_CONFIG" ]]; then
  SOURCE_CONFIG="$DEFAULT_SOURCE_CONFIG"
  INSTALL_CONFIG=true
elif [[ -f "$CONFIG_FILE" ]]; then
  echo "保留已有数采配置：$CONFIG_FILE"
elif [[ -f "$EXAMPLE_CONFIG" ]]; then
  SOURCE_CONFIG="$EXAMPLE_CONFIG"
  INSTALL_CONFIG=true
  echo "首次安装将部署配置模板，请安装后填写硬件信息：$CONFIG_FILE"
else
  echo "错误：找不到数采配置或模板：$EXAMPLE_CONFIG" >&2
  exit 2
fi

UV_BIN="$(command -v uv || true)"
if [[ -z "$UV_BIN" ]]; then
  for uv_candidate in \
    "/home/$DATA_COLLECTION_USER/.local/bin/uv" \
    "/usr/local/bin/uv" \
    "/usr/bin/uv"; do
    if [[ -x "$uv_candidate" ]]; then
      UV_BIN="$uv_candidate"
      break
    fi
  done
fi
[[ -n "$UV_BIN" ]] || {
  echo "错误：找不到 uv，无法创建数采 Python 环境。" >&2
  exit 1
}

sudo install -d -o "$DATA_COLLECTION_USER" -g "$DATA_COLLECTION_USER" "$DEPLOY_DIR"
if [[ "$INSTALL_CONFIG" = true ]]; then
  sudo install -o "$DATA_COLLECTION_USER" -g "$DATA_COLLECTION_USER" -m 0660 \
    "$SOURCE_CONFIG" "$CONFIG_FILE"
fi
sudo chown "$DATA_COLLECTION_USER:$DATA_COLLECTION_USER" "$CONFIG_FILE"
sudo chmod 0660 "$CONFIG_FILE"

if [[ ! -f "$LIVEKIT_CONFIG_FILE" ]]; then
  [[ -f "$LIVEKIT_EXAMPLE_CONFIG" ]] || {
    echo "错误：找不到 LiveKit 配置模板：$LIVEKIT_EXAMPLE_CONFIG" >&2
    exit 2
  }
  sudo install -o "$DATA_COLLECTION_USER" -g "$DATA_COLLECTION_USER" -m 0660 \
    "$LIVEKIT_EXAMPLE_CONFIG" "$LIVEKIT_CONFIG_FILE"
else
  echo "保留已有 LiveKit 配置：$LIVEKIT_CONFIG_FILE"
fi
sudo chown "$DATA_COLLECTION_USER:$DATA_COLLECTION_USER" "$LIVEKIT_CONFIG_FILE"
sudo chmod 0660 "$LIVEKIT_CONFIG_FILE"

if [[ ! -x "$VENV_DIR/bin/python" ]]; then
  sudo -u "$DATA_COLLECTION_USER" env HOME="$DATA_COLLECTION_HOME" "$UV_BIN" venv \
    --python python3.10 --seed "$VENV_DIR"
fi

sudo -u "$DATA_COLLECTION_USER" env HOME="$DATA_COLLECTION_HOME" "$UV_BIN" pip install \
  --python "$VENV_DIR/bin/python" \
  --upgrade \
  --extra-index-url "$PYPI_URL" \
  'robot-data-collection[astrabot]'

sudo -u "$DATA_COLLECTION_USER" env HOME="$DATA_COLLECTION_HOME" \
  "$VENV_DIR/bin/astra-setup-control-venv"

echo "数采环境已部署：$VENV_DIR"
echo "数采配置已安装：$CONFIG_FILE"
echo "LiveKit 配置已安装：$LIVEKIT_CONFIG_FILE"
echo "启动前请检查并填写数采配置中的硬件信息。"
echo "安装 robot_start 后，服务名为 Astrabot_Data_Collection.service"
