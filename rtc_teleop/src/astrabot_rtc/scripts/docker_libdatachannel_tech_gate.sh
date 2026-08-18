#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODULE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE="${ASTRABOT_RTC_LIBDATACHANNEL_IMAGE:-webrtc-device:latest}"

# 该 Gate 复用已验证 prototype 镜像中安装的 libdatachannel 0.24 API。
# 它只验证 PeerConnection、H264 packetizer 和 unordered/20 ms DataChannel API，
# 不代表平台信令、多 peer、授权或真机媒体链路已经验收。
docker run --rm \
  --entrypoint bash \
  --volume "${MODULE_ROOT}:/workspace/astrabot_rtc:ro" \
  --workdir /workspace/astrabot_rtc \
  "${IMAGE}" \
  -lc '
    set -euo pipefail
    c++ -std=c++17 -Wall -Wextra -Wpedantic \
      test/libdatachannel_tech_gate.cpp \
      -o /tmp/astrabot_rtc_libdatachannel_tech_gate \
      -L/usr/local/lib -Wl,-rpath,/usr/local/lib -ldatachannel -pthread
    /tmp/astrabot_rtc_libdatachannel_tech_gate
  '
