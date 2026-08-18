#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODULE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROBOT_ROOT="${ROBOT_WORKSPACE_ROOT:-$(cd "${MODULE_ROOT}/../.." && pwd)}"
RTC_ROOT="${ROBOT_ROOT}/robot_system/astrabot_rtc"
DATA_INTERFACES_ROOT="${ROBOT_ROOT}/robot_data/astrabot_data_interfaces"
TOOLCHAIN="${MODULE_ROOT}/cmake/aarch64-linux-gnu.cmake"
SDK_ROOT="/opt/astrabot_sdk_2404"
BUILD_ROOT="${ASTRABOT_TELEOP_CROSS_BUILD_ROOT:-${ROBOT_ROOT}/.colcon/astrabot_teleop/cross}"
PROTOBUF_LIBRARY_DIR="${SDK_ROOT}/usr/lib/aarch64-linux-gnu"
PROTOBUF_LIBRARY="${PROTOBUF_LIBRARY_DIR}/libprotobuf.so"
PROTOBUF_HOST_INCLUDE_ROOT="${BUILD_ROOT}/host_protobuf_include"
RTC_BACKEND="${ASTRABOT_TELEOP_RTC_BACKEND:-disabled}"
LIBDATACHANNEL_PREFIX="${ASTRABOT_TELEOP_ARM64_LIBDATACHANNEL_PREFIX:-}"

if [[ ! -e /.dockerenv ]]; then
  echo "ARM64 validation must run inside the astrabot_teleop Docker image" >&2
  exit 1
fi
if [[ "${RTC_BACKEND}" != "disabled" && "${RTC_BACKEND}" != "libdatachannel" ]]; then
    echo "ASTRABOT_TELEOP_RTC_BACKEND must be disabled or libdatachannel" >&2
    exit 2
fi
rtc_backend_args=(-DASTRABOT_RTC_ENABLE_LIBDATACHANNEL_BACKEND=OFF)
if [[ "${RTC_BACKEND}" == "libdatachannel" ]]; then
    if [[ ! -f "${LIBDATACHANNEL_PREFIX}/include/rtc/rtc.h" || \
          ! -f "${LIBDATACHANNEL_PREFIX}/lib/libdatachannel.so" || \
          ! -f "${LIBDATACHANNEL_PREFIX}/lib/cmake/LibDataChannel/LibDataChannelConfig.cmake" ]]; then
        echo "ARM64 libdatachannel SDK import is incomplete" >&2
        exit 1
    fi
    rtc_backend_args=(
        -DASTRABOT_RTC_ENABLE_LIBDATACHANNEL_BACKEND=ON
        -DCMAKE_PREFIX_PATH="${LIBDATACHANNEL_PREFIX}"
        -DLibDataChannel_DIR="${LIBDATACHANNEL_PREFIX}/lib/cmake/LibDataChannel"
    )
fi
if [[ ! -f "${TOOLCHAIN}" || ! -d "${SDK_ROOT}" ]]; then
    echo "THOR toolchain, SDK sysroot or target protobuf runtime is unavailable" >&2
    exit 1
fi
if [[ ! -f "${SDK_ROOT}/usr/lib/aarch64-linux-gnu/pkgconfig/libavcodec.pc" || \
      ! -d "${SDK_ROOT}/usr/include/aarch64-linux-gnu/libavcodec" ]]; then
    echo "THOR SDK sysroot does not contain the ARM64 FFmpeg development files" >&2
    exit 1
fi
if [[ ! -e "${PROTOBUF_LIBRARY}" ]]; then
    mapfile -t protobuf_candidates < <(
        find "${PROTOBUF_LIBRARY_DIR}" -maxdepth 1 \( -type f -o -type l \) -name 'libprotobuf.so.*' -print | sort -V
    )
    if [[ "${#protobuf_candidates[@]}" -eq 0 ]]; then
        echo "target protobuf runtime is unavailable under ${PROTOBUF_LIBRARY_DIR}" >&2
        exit 1
    fi
    PROTOBUF_LIBRARY="${protobuf_candidates[0]}"
fi

set +u
source /opt/ros/jazzy/setup.bash
set -u

# RTC 通过 pkg-config 发现 FFmpeg。交叉构建必须把 pc 文件中的 /usr 路径重定位到 THOR sysroot，
# 否则 CMake 会把宿主容器的 /usr/include/aarch64-linux-gnu 当作真实目录并在 generate 阶段失败。
export PKG_CONFIG_SYSROOT_DIR="${SDK_ROOT}"
export PKG_CONFIG_LIBDIR="${SDK_ROOT}/usr/lib/aarch64-linux-gnu/pkgconfig"
unset PKG_CONFIG_PATH

rm -rf "${BUILD_ROOT}"
mkdir -p "${BUILD_ROOT}"
mkdir -p "${PROTOBUF_HOST_INCLUDE_ROOT}"
# protobuf headers are architecture-independent. Use the build-image headers matching protoc 3.21 while linking
# against the SDK ARM64 runtime; the non-system path prevents CMake from dropping /usr/include as an implicit path.
ln -s /usr/include/google "${PROTOBUF_HOST_INCLUDE_ROOT}/google"
cd "${ROBOT_ROOT}"

# 先生成并安装 RTC ARM64 接口/运行库，再显式传给 Teleop；SDK toolchain 会重置 CMAKE_PREFIX_PATH。
colcon --log-base "${BUILD_ROOT}/log" build \
  --base-paths "${DATA_INTERFACES_ROOT}" "${RTC_ROOT}" \
  --merge-install \
  --build-base "${BUILD_ROOT}/build" \
  --install-base "${BUILD_ROOT}/install" \
  --packages-select astrabot_data_interfaces astrabot_rtc \
  --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF \
    "${rtc_backend_args[@]}"

if [[ "${RTC_BACKEND}" == "libdatachannel" ]]; then
    RTC_WEBRTC_LIBRARY="${BUILD_ROOT}/install/lib/libastrabot_rtc_webrtc.so"
    RTC_CAPABILITY="${BUILD_ROOT}/install/share/astrabot_rtc/capabilities/libdatachannel.enabled"
    readelf -d "${RTC_WEBRTC_LIBRARY}" | grep -q 'Shared library: \[libdatachannel.so.0.24\]'
    grep -Fqx 'backend=libdatachannel' "${RTC_CAPABILITY}"
    if find "${BUILD_ROOT}/install" -name 'libdatachannel.so*' -print -quit | grep -q .; then
        echo "RTC/Teleop install tree must receive libdatachannel only from the runtime package" >&2
        exit 1
    fi
fi

colcon --log-base "${BUILD_ROOT}/log" build \
  --base-paths "${MODULE_ROOT}" \
  --merge-install \
  --build-base "${BUILD_ROOT}/build" \
  --install-base "${BUILD_ROOT}/install" \
  --packages-select astrabot_teleop \
  --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF \
    -Dastrabot_data_interfaces_DIR="${BUILD_ROOT}/install/share/astrabot_data_interfaces/cmake" \
    -Dastrabot_rtc_DIR="${BUILD_ROOT}/install/share/astrabot_rtc/cmake" \
    -DProtobuf_PROTOC_EXECUTABLE=/usr/bin/protoc \
    -DProtobuf_INCLUDE_DIR="${PROTOBUF_HOST_INCLUDE_ROOT}" \
    -DProtobuf_LIBRARY="${PROTOBUF_LIBRARY}" \
    -Dnlohmann_json_DIR="${SDK_ROOT}/usr/share/cmake/nlohmann_json"

readelf -h "${BUILD_ROOT}/install/lib/astrabot_teleop/astrabot_teleop_node" | grep -Eq 'Machine:[[:space:]]+AArch64'
