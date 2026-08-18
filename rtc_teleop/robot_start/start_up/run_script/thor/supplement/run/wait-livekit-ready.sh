#!/usr/bin/env bash
set -Eeuo pipefail

TIMEOUT_SECONDS="${LIVEKIT_READY_TIMEOUT:-60}"
LIVEKIT_HOST="${LIVEKIT_READY_HOST:-127.0.0.1}"
LIVEKIT_PORT="${LIVEKIT_READY_PORT:-7880}"
TOKEN_URL="${LIVEKIT_TOKEN_URL:-http://127.0.0.1:5000/token}"
deadline=$((SECONDS + TIMEOUT_SECONDS))

command -v curl >/dev/null 2>&1 || {
  echo "错误：找不到 curl，无法检查 LiveKit token server" >&2
  exit 1
}

while (( SECONDS < deadline )); do
  livekit_ready=false
  token_server_ready=false

  if timeout 1 bash -c ": >/dev/tcp/$LIVEKIT_HOST/$LIVEKIT_PORT" 2>/dev/null; then
    livekit_ready=true
  fi
  if curl --silent --show-error --max-time 1 \
      --output /dev/null "$TOKEN_URL" 2>/dev/null; then
    token_server_ready=true
  fi

  if [[ "$livekit_ready" = true && "$token_server_ready" = true ]]; then
    echo "LiveKit 已就绪：${LIVEKIT_HOST}:${LIVEKIT_PORT}，${TOKEN_URL}"
    exit 0
  fi

  echo "等待 LiveKit：media=${livekit_ready}, token=${token_server_ready}"
  sleep 2
done

echo "错误：等待 LiveKit 就绪超时（${TIMEOUT_SECONDS}s）" >&2
exit 1
