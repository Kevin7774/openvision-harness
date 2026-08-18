#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODULE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_IMAGE="${ASTRABOT_RTC_SDK_IMAGE:-registry-01-registry.ap-southeast-1.cr.aliyuncs.com/robot/nx-ubuntu2404-x86:latest}"
WEBRTC_IMAGE="${ASTRABOT_RTC_LIBDATACHANNEL_IMAGE:-webrtc-device:latest}"
RTC_SDK_DIR="$(mktemp -d)"
RTC_COPY_CONTAINER="astrabot-rtc-sdk-copy-${RANDOM}-$$"

cleanup() {
  docker rm -f "${RTC_COPY_CONTAINER}" >/dev/null 2>&1 || true
  rm -rf "${RTC_SDK_DIR}"
}
trap cleanup EXIT

mkdir -p "${RTC_SDK_DIR}/include" "${RTC_SDK_DIR}/lib/cmake"
docker create --name "${RTC_COPY_CONTAINER}" "${WEBRTC_IMAGE}" >/dev/null
docker cp "${RTC_COPY_CONTAINER}:/usr/local/include/rtc" "${RTC_SDK_DIR}/include/"
docker cp "${RTC_COPY_CONTAINER}:/usr/local/lib/libdatachannel.so.0.24.2" "${RTC_SDK_DIR}/lib/"
docker cp "${RTC_COPY_CONTAINER}:/usr/local/lib/cmake/LibDataChannel" "${RTC_SDK_DIR}/lib/cmake/"
docker cp "${RTC_COPY_CONTAINER}:/usr/include/nlohmann" "${RTC_SDK_DIR}/include/"
docker cp "${RTC_COPY_CONTAINER}:/usr/share/cmake/nlohmann_json" "${RTC_SDK_DIR}/lib/cmake/"
docker rm "${RTC_COPY_CONTAINER}" >/dev/null

ln -s libdatachannel.so.0.24.2 "${RTC_SDK_DIR}/lib/libdatachannel.so.0.24"
ln -s libdatachannel.so.0.24 "${RTC_SDK_DIR}/lib/libdatachannel.so"

docker run --rm \
  --volume "${MODULE_ROOT}:/workspace/src/astrabot_rtc:ro" \
  --volume "${RTC_SDK_DIR}:/opt/libdatachannel:ro" \
  --workdir /workspace \
  "${ROS_IMAGE}" \
  bash -lc '
    set -eo pipefail
    source /opt/ros/jazzy/setup.bash
    set -u
    colcon build \
      --base-paths /workspace/src/astrabot_rtc \
      --packages-select astrabot_rtc \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DBUILD_TESTING=ON \
        -DASTRABOT_RTC_ENABLE_LIBDATACHANNEL_BACKEND=ON \
        -DCMAKE_PREFIX_PATH=/opt/libdatachannel
    capability=/workspace/install/astrabot_rtc/share/astrabot_rtc/capabilities/libdatachannel.enabled
    test -f "${capability}"
    grep -Fqx "backend=libdatachannel" "${capability}"
    grep -Fqx "soname=libdatachannel.so.0.24" "${capability}"
    LD_LIBRARY_PATH=/opt/libdatachannel/lib:${LD_LIBRARY_PATH:-} \
      ctest --test-dir /workspace/build/astrabot_rtc --output-on-failure
    cmake --build /workspace/build/astrabot_rtc --target no_exceptions_check
  '
