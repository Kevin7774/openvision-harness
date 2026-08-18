#!/usr/bin/env bash
set -Eeuo pipefail

LIVEKIT_BIN="${LIVEKIT_SERVER_BIN:-/usr/local/bin/livekit-server}"
LIVEKIT_BIND="${LIVEKIT_BIND_ADDRESS:-0.0.0.0}"
LIVEKIT_NODE_IP="${LIVEKIT_NODE_IP:-192.168.123.102}"

[[ -x "$LIVEKIT_BIN" ]] || {
  echo "错误：LiveKit server 不存在或不可执行：$LIVEKIT_BIN" >&2
  exit 1
}

exec "$LIVEKIT_BIN" --dev --bind "$LIVEKIT_BIND" --node-ip "$LIVEKIT_NODE_IP"
