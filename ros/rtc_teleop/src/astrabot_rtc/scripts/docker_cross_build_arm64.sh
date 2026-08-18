#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODULE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE="${ASTRABOT_RTC_SDK_IMAGE:-registry-01-registry.ap-southeast-1.cr.aliyuncs.com/robot/nx-ubuntu2404-x86@sha256:bd313d470a6bdacee057a920a56bb698fd888bcbe6281b8a498f4677fefe8e4e}"
SDK_ROOT="${ASTRABOT_RTC_ARM64_SDK_ROOT:-/opt/astrabot_sdk_2404}"
ENABLE_BACKEND="${ASTRABOT_RTC_CROSS_ENABLE_LIBDATACHANNEL:-0}"
LIBDATACHANNEL_PREFIX="${ASTRABOT_RTC_ARM64_LIBDATACHANNEL_PREFIX:-}"

if [[ ! -d "${SDK_ROOT}/opt/ros/jazzy" || ! -f "${SDK_ROOT}/usr/lib/aarch64-linux-gnu/pkgconfig/libavcodec.pc" ]]; then
  echo "THOR SDK sysroot or ARM64 FFmpeg pkg-config files are unavailable: ${SDK_ROOT}" >&2
  exit 1
fi

docker_arguments=(
  --rm
  --network none
  --volume "${MODULE_ROOT}:/workspace/src/astrabot_rtc:ro"
  --volume "${SDK_ROOT}:/opt/astrabot_sdk_2404:ro"
  --workdir /workspace
  --env "ASTRABOT_RTC_CROSS_ENABLE_LIBDATACHANNEL=${ENABLE_BACKEND}"
)

if [[ "${ENABLE_BACKEND}" == "1" ]]; then
  if [[ -z "${LIBDATACHANNEL_PREFIX}" || -L "${LIBDATACHANNEL_PREFIX}" || \
        ! -f "${LIBDATACHANNEL_PREFIX}/include/rtc/rtc.h" || \
        ! -f "${LIBDATACHANNEL_PREFIX}/include/rtc/version.h" || \
        ! -f "${LIBDATACHANNEL_PREFIX}/lib/libdatachannel.so" || \
        ! -f "${LIBDATACHANNEL_PREFIX}/lib/cmake/LibDataChannel/LibDataChannelConfig.cmake" || \
        ! -f "${LIBDATACHANNEL_PREFIX}/share/libdatachannel/source-manifest.txt" || \
        ! -f "${LIBDATACHANNEL_PREFIX}/share/licenses/libdatachannel/LICENSE" ]]; then
    echo "ARM64 libdatachannel import is incomplete; set ASTRABOT_RTC_ARM64_LIBDATACHANNEL_PREFIX" >&2
    exit 1
  fi
  if ! grep -Fqx '#define RTC_VERSION "0.24.2"' "${LIBDATACHANNEL_PREFIX}/include/rtc/version.h"; then
    echo "libdatachannel import is not version 0.24.2" >&2
    exit 1
  fi
  if ! grep -Fqx 'libdatachannel_version=0.24.2' \
      "${LIBDATACHANNEL_PREFIX}/share/libdatachannel/source-manifest.txt" || \
    ! grep -Fqx 'libdatachannel_commit=4e4f4892dccb2a57fe3a490d0c9d958de4244e74' \
      "${LIBDATACHANNEL_PREFIX}/share/libdatachannel/source-manifest.txt" || \
    ! grep -Fqx 'json_commit=55f93686c01528224f448c19128836e7df245f72' \
      "${LIBDATACHANNEL_PREFIX}/share/libdatachannel/source-manifest.txt" || \
    ! grep -Fqx 'libjuice_commit=5948a4162d37bc213d6051b67ee2876ccc5a99a6' \
      "${LIBDATACHANNEL_PREFIX}/share/libdatachannel/source-manifest.txt" || \
    ! grep -Fqx 'libsrtp_commit=ee1a77c9f9dc02c42bda9901038c500c5efe4cfa' \
      "${LIBDATACHANNEL_PREFIX}/share/libdatachannel/source-manifest.txt" || \
    ! grep -Fqx 'plog_commit=94899e0b926ac1b0f4750bfbd495167b4a6ae9ef' \
      "${LIBDATACHANNEL_PREFIX}/share/libdatachannel/source-manifest.txt" || \
    ! grep -Fqx 'usrsctp_commit=fec583d54493f879d2ae44a743423bf8a04371ab' \
      "${LIBDATACHANNEL_PREFIX}/share/libdatachannel/source-manifest.txt" || \
    ! grep -Fqx 'soname=libdatachannel.so.0.24' \
      "${LIBDATACHANNEL_PREFIX}/share/libdatachannel/source-manifest.txt"; then
    echo "libdatachannel import source manifest does not match the approved SDK" >&2
    exit 1
  fi

  LIBDATACHANNEL_LIBRARY="$(readlink -f "${LIBDATACHANNEL_PREFIX}/lib/libdatachannel.so")"
  if [[ "$(basename "${LIBDATACHANNEL_LIBRARY}")" != "libdatachannel.so.0.24.2" || \
        ! -f "${LIBDATACHANNEL_LIBRARY}" ]] || \
    ! LC_ALL=C readelf -h "${LIBDATACHANNEL_LIBRARY}" | grep -q "Machine:.*AArch64"; then
    echo "libdatachannel import is not an AArch64 ELF: ${LIBDATACHANNEL_LIBRARY}" >&2
    exit 1
  fi
  if ! LC_ALL=C readelf -d "${LIBDATACHANNEL_LIBRARY}" | grep -q "SONAME.*\[libdatachannel.so.0.24\]"; then
    echo "libdatachannel import has an unexpected SONAME: ${LIBDATACHANNEL_LIBRARY}" >&2
    exit 1
  fi
  if LC_ALL=C readelf -d "${LIBDATACHANNEL_LIBRARY}" | grep -Eq "\((RPATH|RUNPATH)\)"; then
    echo "libdatachannel import must not contain RPATH/RUNPATH" >&2
    exit 1
  fi

  mapfile -t imported_dependencies < <(
    LC_ALL=C readelf -d "${LIBDATACHANNEL_LIBRARY}" |
      sed -n 's/.*Shared library: \[\([^]]*\)\].*/\1/p'
  )
  expected_dependencies=(
    libssl.so.3
    libcrypto.so.3
    libstdc++.so.6
    libm.so.6
    libgcc_s.so.1
    libc.so.6
    ld-linux-aarch64.so.1
  )
  for dependency in "${expected_dependencies[@]}"; do
    dependency_found=0
    for imported_dependency in "${imported_dependencies[@]}"; do
      if [[ "${imported_dependency}" == "${dependency}" ]]; then
        dependency_found=1
        break
      fi
    done
    if [[ "${dependency_found}" -ne 1 ]]; then
      echo "libdatachannel import is missing expected runtime dependency: ${dependency}" >&2
      exit 1
    fi
    if [[ ! -e "${SDK_ROOT}/usr/lib/aarch64-linux-gnu/${dependency}" && \
          ! -e "${SDK_ROOT}/lib/aarch64-linux-gnu/${dependency}" && \
          ! -e "${SDK_ROOT}/usr/lib/${dependency}" && \
          ! -e "${SDK_ROOT}/lib/${dependency}" ]]; then
      echo "THOR sysroot does not provide libdatachannel runtime dependency: ${dependency}" >&2
      exit 1
    fi
  done
  for dependency in "${imported_dependencies[@]}"; do
    dependency_found=0
    for expected_dependency in "${expected_dependencies[@]}"; do
      if [[ "${expected_dependency}" == "${dependency}" ]]; then
        dependency_found=1
        break
      fi
    done
    if [[ "${dependency_found}" -ne 1 ]]; then
      echo "libdatachannel import has an undeclared runtime dependency: ${dependency}" >&2
      exit 1
    fi
  done

  required_license_files=(
    share/licenses/libdatachannel/LICENSE
    share/licenses/libdatachannel/deps/json/LICENSE.MIT
    share/licenses/libdatachannel/deps/libjuice/LICENSE
    share/licenses/libdatachannel/deps/libsrtp/LICENSE
    share/licenses/libdatachannel/deps/plog/LICENSE
    share/licenses/libdatachannel/deps/usrsctp/LICENSE.md
  )
  for license_file in "${required_license_files[@]}"; do
    if [[ ! -s "${LIBDATACHANNEL_PREFIX}/${license_file}" ]]; then
      echo "libdatachannel import is missing license file: ${license_file}" >&2
      exit 1
    fi
  done
  docker_arguments+=(--volume "${LIBDATACHANNEL_PREFIX}:/opt/libdatachannel-arm64:ro")
else
  if [[ "${ENABLE_BACKEND}" != "0" ]]; then
    echo "ASTRABOT_RTC_CROSS_ENABLE_LIBDATACHANNEL must be 0 or 1" >&2
    exit 1
  fi
  echo "warning: building ARM64 RTC with disabled transport because no target libdatachannel SDK was requested" >&2
fi

docker run "${docker_arguments[@]}" \
  "${IMAGE}" \
  bash -lc '
    set -eo pipefail
    source /opt/ros/jazzy/setup.bash
    set -u
    export PKG_CONFIG_SYSROOT_DIR=/opt/astrabot_sdk_2404
    export PKG_CONFIG_LIBDIR=/opt/astrabot_sdk_2404/usr/lib/aarch64-linux-gnu/pkgconfig
    backend_args=(-DASTRABOT_RTC_ENABLE_LIBDATACHANNEL_BACKEND=OFF)
    if [[ "${ASTRABOT_RTC_CROSS_ENABLE_LIBDATACHANNEL}" == "1" ]]; then
      backend_args=(
        -DASTRABOT_RTC_ENABLE_LIBDATACHANNEL_BACKEND=ON
        -DCMAKE_PREFIX_PATH=/opt/libdatachannel-arm64
        -DLibDataChannel_DIR=/opt/libdatachannel-arm64/lib/cmake/LibDataChannel
      )
    fi
    colcon build \
      --base-paths /workspace/src/astrabot_rtc \
      --merge-install \
      --build-base /tmp/astrabot-rtc-cross/build \
      --install-base /tmp/astrabot-rtc-cross/install \
      --packages-select astrabot_rtc \
      --cmake-args \
        -DCMAKE_TOOLCHAIN_FILE=/etc/conan/xr1-toolchain.cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
        "${backend_args[@]}"
    node=/tmp/astrabot-rtc-cross/install/lib/astrabot_rtc/astrabot_rtc_node
    LC_ALL=C readelf -h "${node}" | grep -q "Machine:.*AArch64"
    if [[ "${ASTRABOT_RTC_CROSS_ENABLE_LIBDATACHANNEL}" == "1" ]]; then
      webrtc_library=/tmp/astrabot-rtc-cross/install/lib/libastrabot_rtc_webrtc.so
      LC_ALL=C readelf -h "${webrtc_library}" | grep -q "Machine:.*AArch64"
      LC_ALL=C readelf -d "${webrtc_library}" | grep -q "Shared library: \[libdatachannel.so.0.24\]"
      if LC_ALL=C readelf -d "${webrtc_library}" | grep -Eq "\((RPATH|RUNPATH)\)"; then
        echo "RTC WebRTC library unexpectedly contains RPATH/RUNPATH" >&2
        exit 1
      fi
      if find /tmp/astrabot-rtc-cross/install -name "libdatachannel.so*" -print -quit | grep -q .; then
        echo "RTC install unexpectedly bundled libdatachannel without an explicit packaging policy" >&2
        exit 1
      fi
    fi
    echo "verified ARM64 RTC node: ${node}"
    if [[ "${ASTRABOT_RTC_CROSS_ENABLE_LIBDATACHANNEL}" == "1" ]]; then
      echo "verified ARM64 backend compile/link gate; libdatachannel.so.0.24 remains an external runtime dependency"
    fi
  '
