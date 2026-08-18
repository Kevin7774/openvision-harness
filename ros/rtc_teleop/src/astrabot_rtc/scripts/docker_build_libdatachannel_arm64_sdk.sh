#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
readonly BUILD_IMAGE="registry-01-registry.ap-southeast-1.cr.aliyuncs.com/robot/nx-ubuntu2404-x86@sha256:bd313d470a6bdacee057a920a56bb698fd888bcbe6281b8a498f4677fefe8e4e"
readonly SDK_ROOT="/opt/astrabot_sdk_2404"
readonly LIBDATACHANNEL_REPOSITORY="https://github.com/paullouisageneau/libdatachannel.git"
readonly LIBDATACHANNEL_COMMIT="4e4f4892dccb2a57fe3a490d0c9d958de4244e74"
readonly JSON_COMMIT="55f93686c01528224f448c19128836e7df245f72"
readonly LIBJUICE_COMMIT="5948a4162d37bc213d6051b67ee2876ccc5a99a6"
readonly LIBSRTP_COMMIT="ee1a77c9f9dc02c42bda9901038c500c5efe4cfa"
readonly PLOG_COMMIT="94899e0b926ac1b0f4750bfbd495167b4a6ae9ef"
readonly USRSCTP_COMMIT="fec583d54493f879d2ae44a743423bf8a04371ab"
readonly BUILD_LOCK_NAME=".astrabot_rtc_sdk_build_lock"

usage() {
  echo "usage: $0 <existing-empty-output-directory>" >&2
}

fail() {
  echo "error: $*" >&2
  exit 1
}

if [[ "$#" -ne 1 ]]; then
  usage
  exit 2
fi

command -v docker >/dev/null 2>&1 || fail "docker is required"
command -v realpath >/dev/null 2>&1 || fail "realpath is required"
command -v find >/dev/null 2>&1 || fail "find is required"

output_argument="$1"
if [[ -z "${output_argument}" || ! -d "${output_argument}" || -L "${output_argument}" ]]; then
  fail "output must be an explicit existing directory and must not be a symbolic link"
fi

output_dir="$(realpath -- "${output_argument}")"
if [[ "${output_dir}" == "/" ]]; then
  fail "filesystem root cannot be used as the output directory"
fi
if [[ ! -w "${output_dir}" ]]; then
  fail "output directory is not writable: ${output_dir}"
fi
if [[ -n "$(find "${output_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  fail "output directory must be empty; existing content is never overwritten: ${output_dir}"
fi

if [[ ! -d "${SDK_ROOT}/opt/ros/jazzy" || \
      ! -f "${SDK_ROOT}/usr/include/openssl/ssl.h" || \
      ! -f "${SDK_ROOT}/usr/lib/aarch64-linux-gnu/libssl.so" || \
      ! -f "${SDK_ROOT}/usr/lib/aarch64-linux-gnu/libcrypto.so" ]]; then
  fail "THOR SDK sysroot is incomplete: ${SDK_ROOT}"
fi

lock_dir="${output_dir}/${BUILD_LOCK_NAME}"
if ! mkdir "${lock_dir}"; then
  fail "failed to acquire the output directory build lock: ${lock_dir}"
fi

release_lock() {
  rmdir "${lock_dir}" 2>/dev/null || true
}
trap release_lock EXIT

docker run --rm \
  --volume "${SDK_ROOT}:/opt/astrabot_sdk_2404:ro" \
  --volume "${output_dir}:/output" \
  --env "LIBDATACHANNEL_REPOSITORY=${LIBDATACHANNEL_REPOSITORY}" \
  --env "LIBDATACHANNEL_COMMIT=${LIBDATACHANNEL_COMMIT}" \
  --env "JSON_COMMIT=${JSON_COMMIT}" \
  --env "LIBJUICE_COMMIT=${LIBJUICE_COMMIT}" \
  --env "LIBSRTP_COMMIT=${LIBSRTP_COMMIT}" \
  --env "PLOG_COMMIT=${PLOG_COMMIT}" \
  --env "USRSCTP_COMMIT=${USRSCTP_COMMIT}" \
  --env "BUILD_LOCK_NAME=${BUILD_LOCK_NAME}" \
  "${BUILD_IMAGE}" \
  bash -lc '
    set -euo pipefail

    readonly source_root=/tmp/libdatachannel-source
    readonly build_root=/tmp/libdatachannel-build
    readonly stage_root=/tmp/libdatachannel-stage

    fail() {
      echo "error: $*" >&2
      exit 1
    }

    verify_commit() {
      local repository_path="$1"
      local expected_commit="$2"
      local actual_commit
      actual_commit="$(git -C "${repository_path}" rev-parse HEAD)"
      if [[ "${actual_commit}" != "${expected_commit}" ]]; then
        fail "unexpected commit in ${repository_path}: ${actual_commit}"
      fi
    }

    unexpected_output="$(find /output -mindepth 1 -maxdepth 1 ! -name "${BUILD_LOCK_NAME}" -print -quit)"
    if [[ -n "${unexpected_output}" ]]; then
      fail "output directory changed after validation; refusing to overwrite it"
    fi

    git -c init.defaultBranch=main init --quiet "${source_root}"
    git -C "${source_root}" remote add origin "${LIBDATACHANNEL_REPOSITORY}"
    git -C "${source_root}" fetch --quiet --depth 1 --no-tags origin "${LIBDATACHANNEL_COMMIT}"
    git -C "${source_root}" checkout --quiet --detach FETCH_HEAD
    verify_commit "${source_root}" "${LIBDATACHANNEL_COMMIT}"

    [[ "$(git -C "${source_root}" config -f .gitmodules --get submodule.deps/json.url)" == \
      "https://github.com/nlohmann/json.git" ]] || fail "unexpected json submodule URL"
    [[ "$(git -C "${source_root}" config -f .gitmodules --get submodule.deps/libjuice.url)" == \
      "https://github.com/paullouisageneau/libjuice.git" ]] || fail "unexpected libjuice submodule URL"
    [[ "$(git -C "${source_root}" config -f .gitmodules --get submodule.deps/libsrtp.url)" == \
      "https://github.com/cisco/libsrtp.git" ]] || fail "unexpected libsrtp submodule URL"
    [[ "$(git -C "${source_root}" config -f .gitmodules --get submodule.deps/plog.url)" == \
      "https://github.com/SergiusTheBest/plog.git" ]] || fail "unexpected plog submodule URL"
    [[ "$(git -C "${source_root}" config -f .gitmodules --get submodule.deps/usrsctp.url)" == \
      "https://github.com/paullouisageneau/usrsctp.git" ]] || fail "unexpected usrsctp submodule URL"

    git -C "${source_root}" submodule update --quiet --init --recursive --depth 1 --jobs 4
    verify_commit "${source_root}/deps/json" "${JSON_COMMIT}"
    verify_commit "${source_root}/deps/libjuice" "${LIBJUICE_COMMIT}"
    verify_commit "${source_root}/deps/libsrtp" "${LIBSRTP_COMMIT}"
    verify_commit "${source_root}/deps/plog" "${PLOG_COMMIT}"
    verify_commit "${source_root}/deps/usrsctp" "${USRSCTP_COMMIT}"

    export SDK_ROOT=/opt/astrabot_sdk_2404
    cmake -S "${source_root}" -B "${build_root}" \
      -DCMAKE_TOOLCHAIN_FILE=/etc/conan/xr1-toolchain.cmake \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="${stage_root}" \
      -DCMAKE_INSTALL_LIBDIR=lib \
      -DBUILD_SHARED_LIBS=ON \
      -DBUILD_SHARED_DEPS_LIBS=OFF \
      -DUSE_GNUTLS=OFF \
      -DUSE_MBEDTLS=OFF \
      -DUSE_NICE=OFF \
      -DPREFER_SYSTEM_LIB=OFF \
      -DUSE_SYSTEM_SRTP=OFF \
      -DUSE_SYSTEM_JUICE=OFF \
      -DUSE_SYSTEM_USRSCTP=OFF \
      -DUSE_SYSTEM_PLOG=OFF \
      -DUSE_SYSTEM_JSON=OFF \
      -DNO_WEBSOCKET=ON \
      -DNO_MEDIA=OFF \
      -DNO_EXAMPLES=ON \
      -DNO_TESTS=ON \
      -DWARNINGS_AS_ERRORS=OFF \
      -DRTC_UPDATE_VERSION_HEADER=OFF \
      -DOPENSSL_ROOT_DIR=/opt/astrabot_sdk_2404/usr \
      -DOPENSSL_INCLUDE_DIR=/opt/astrabot_sdk_2404/usr/include \
      -DOPENSSL_SSL_LIBRARY=/opt/astrabot_sdk_2404/usr/lib/aarch64-linux-gnu/libssl.so \
      -DOPENSSL_CRYPTO_LIBRARY=/opt/astrabot_sdk_2404/usr/lib/aarch64-linux-gnu/libcrypto.so
    cmake --build "${build_root}" --parallel 4
    cmake --install "${build_root}"

    install -D -m 0644 "${source_root}/LICENSE" \
      "${stage_root}/share/licenses/libdatachannel/LICENSE"
    install -D -m 0644 "${source_root}/deps/json/LICENSE.MIT" \
      "${stage_root}/share/licenses/libdatachannel/deps/json/LICENSE.MIT"
    install -D -m 0644 "${source_root}/deps/libjuice/LICENSE" \
      "${stage_root}/share/licenses/libdatachannel/deps/libjuice/LICENSE"
    install -D -m 0644 "${source_root}/deps/libsrtp/LICENSE" \
      "${stage_root}/share/licenses/libdatachannel/deps/libsrtp/LICENSE"
    install -D -m 0644 "${source_root}/deps/plog/LICENSE" \
      "${stage_root}/share/licenses/libdatachannel/deps/plog/LICENSE"
    install -D -m 0644 "${source_root}/deps/usrsctp/LICENSE.md" \
      "${stage_root}/share/licenses/libdatachannel/deps/usrsctp/LICENSE.md"

    manifest="${stage_root}/share/libdatachannel/source-manifest.txt"
    install -d -m 0755 "$(dirname "${manifest}")"
    {
      printf "libdatachannel_repository=%s\n" "${LIBDATACHANNEL_REPOSITORY}"
      printf "libdatachannel_version=0.24.2\n"
      printf "libdatachannel_commit=%s\n" "${LIBDATACHANNEL_COMMIT}"
      printf "json_commit=%s\n" "${JSON_COMMIT}"
      printf "libjuice_commit=%s\n" "${LIBJUICE_COMMIT}"
      printf "libsrtp_commit=%s\n" "${LIBSRTP_COMMIT}"
      printf "plog_commit=%s\n" "${PLOG_COMMIT}"
      printf "usrsctp_commit=%s\n" "${USRSCTP_COMMIT}"
      printf "target_architecture=AArch64\n"
      printf "soname=libdatachannel.so.0.24\n"
      printf "runtime_provider=external_package_required\n"
    } >"${manifest}"
    chmod 0644 "${manifest}"

    library="${stage_root}/lib/libdatachannel.so.0.24.2"
    [[ -f "${library}" ]] || fail "versioned libdatachannel library is missing"
    [[ -L "${stage_root}/lib/libdatachannel.so" ]] || fail "unversioned libdatachannel symlink is missing"
    [[ -L "${stage_root}/lib/libdatachannel.so.0.24" ]] || fail "SONAME libdatachannel symlink is missing"
    [[ "$(readlink "${stage_root}/lib/libdatachannel.so")" == "libdatachannel.so.0.24" ]] || \
      fail "unexpected unversioned libdatachannel symlink target"
    [[ "$(readlink "${stage_root}/lib/libdatachannel.so.0.24")" == "libdatachannel.so.0.24.2" ]] || \
      fail "unexpected SONAME libdatachannel symlink target"
    grep -Fqx "#define RTC_VERSION \"0.24.2\"" "${stage_root}/include/rtc/version.h" || \
      fail "libdatachannel version header does not report 0.24.2"
    LC_ALL=C readelf -h "${library}" | grep -q "Machine:.*AArch64" || \
      fail "libdatachannel output is not an AArch64 ELF"
    LC_ALL=C readelf -d "${library}" | grep -q "SONAME.*\[libdatachannel.so.0.24\]" || \
      fail "libdatachannel SONAME is not libdatachannel.so.0.24"
    if LC_ALL=C readelf -d "${library}" | grep -Eq "\((RPATH|RUNPATH)\)"; then
      fail "libdatachannel output unexpectedly contains RPATH/RUNPATH"
    fi
    [[ -s "${stage_root}/lib/cmake/LibDataChannel/LibDataChannelConfig.cmake" ]] || \
      fail "LibDataChannel CMake package is missing"
    [[ -s "${stage_root}/lib/cmake/LibDataChannel/LibDataChannelTargets-release.cmake" ]] || \
      fail "LibDataChannel release target export is missing"
    grep -Fq "libdatachannel.so.0.24.2" \
      "${stage_root}/lib/cmake/LibDataChannel/LibDataChannelTargets-release.cmake" || \
      fail "LibDataChannel CMake target does not reference version 0.24.2"
    if grep -R -Fq "${build_root}" "${stage_root}/lib/cmake/LibDataChannel"; then
      fail "LibDataChannel CMake package leaks the temporary build path"
    fi
    find "${stage_root}/share/licenses/libdatachannel" -type f -size +0c | grep -q . || \
      fail "libdatachannel license bundle is empty"
    license_count="$(find "${stage_root}/share/licenses/libdatachannel" -type f -size +0c | wc -l)"
    [[ "${license_count}" -eq 6 ]] || fail "expected 6 non-empty license files, found ${license_count}"

    unexpected_output="$(find /output -mindepth 1 -maxdepth 1 ! -name "${BUILD_LOCK_NAME}" -print -quit)"
    if [[ -n "${unexpected_output}" ]]; then
      fail "output directory changed while building; refusing to overwrite it"
    fi
    cp -a "${stage_root}/." /output/
  '

release_lock
trap - EXIT

ASTRABOT_RTC_SDK_IMAGE="${BUILD_IMAGE}" \
ASTRABOT_RTC_ARM64_SDK_ROOT="${SDK_ROOT}" \
ASTRABOT_RTC_CROSS_ENABLE_LIBDATACHANNEL=1 \
ASTRABOT_RTC_ARM64_LIBDATACHANNEL_PREFIX="${output_dir}" \
  "${SCRIPT_DIR}/docker_cross_build_arm64.sh"

echo "verified libdatachannel 0.24.2 ARM64 SDK and RTC backend link gate: ${output_dir}"
echo "note: the SDK is not installed into a deployable RTC image; package libdatachannel.so.0.24 separately before THOR deployment"
