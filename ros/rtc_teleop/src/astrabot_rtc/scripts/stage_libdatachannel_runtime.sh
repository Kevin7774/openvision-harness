#!/usr/bin/env bash
set -euo pipefail

readonly EXPECTED_VERSION="0.24.2"
readonly EXPECTED_SONAME="libdatachannel.so.0.24"
readonly EXPECTED_LIBRARY="libdatachannel.so.0.24.2"
readonly RUNTIME_LIBRARY_DIRECTORY="lib/astrabot_rtc/third_party"
readonly RUNTIME_SHARE_DIRECTORY="share/astrabot_rtc/third_party/libdatachannel"

usage() {
  echo "usage: $0 <validated-libdatachannel-sdk-prefix> <existing-empty-output-directory>" >&2
}

fail() {
  echo "error: $*" >&2
  exit 1
}

require_manifest_value() {
  local manifest="$1"
  local expected="$2"
  grep -Fqx "${expected}" "${manifest}" || fail "source manifest is missing approved value: ${expected}"
}

if [[ "$#" -ne 2 ]]; then
  usage
  exit 2
fi
if [[ ! -e /.dockerenv ]]; then
  fail "runtime staging must run inside the controlled release Docker image"
fi

for command_name in find grep install readelf readlink realpath sed sha256sum sort xargs; do
  command -v "${command_name}" >/dev/null 2>&1 || fail "required command is unavailable: ${command_name}"
done

sdk_argument="$1"
output_argument="$2"
if [[ ! -d "${sdk_argument}" || -L "${sdk_argument}" ]]; then
  fail "SDK prefix must be an explicit directory and must not be a symbolic link"
fi
if [[ ! -d "${output_argument}" || -L "${output_argument}" ]]; then
  fail "output must be an explicit existing directory and must not be a symbolic link"
fi

sdk_prefix="$(realpath -- "${sdk_argument}")"
output_root="$(realpath -- "${output_argument}")"
if [[ "${sdk_prefix}" == "/" || "${output_root}" == "/" || "${sdk_prefix}" == "${output_root}" ]]; then
  fail "filesystem root and in-place staging are not allowed"
fi
if [[ -n "$(find "${output_root}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  fail "output directory must be empty; existing content is never overwritten"
fi

source_manifest="${sdk_prefix}/share/libdatachannel/source-manifest.txt"
source_library="$(readlink -f -- "${sdk_prefix}/lib/libdatachannel.so")"
[[ -f "${source_manifest}" ]] || fail "SDK source manifest is missing"
[[ -f "${source_library}" ]] || fail "SDK shared library is missing"
[[ "$(basename -- "${source_library}")" == "${EXPECTED_LIBRARY}" ]] || \
  fail "SDK shared library version is not ${EXPECTED_VERSION}"

require_manifest_value "${source_manifest}" "libdatachannel_version=${EXPECTED_VERSION}"
require_manifest_value "${source_manifest}" \
  "libdatachannel_commit=4e4f4892dccb2a57fe3a490d0c9d958de4244e74"
require_manifest_value "${source_manifest}" "json_commit=55f93686c01528224f448c19128836e7df245f72"
require_manifest_value "${source_manifest}" "libjuice_commit=5948a4162d37bc213d6051b67ee2876ccc5a99a6"
require_manifest_value "${source_manifest}" "libsrtp_commit=ee1a77c9f9dc02c42bda9901038c500c5efe4cfa"
require_manifest_value "${source_manifest}" "plog_commit=94899e0b926ac1b0f4750bfbd495167b4a6ae9ef"
require_manifest_value "${source_manifest}" "usrsctp_commit=fec583d54493f879d2ae44a743423bf8a04371ab"
require_manifest_value "${source_manifest}" "target_architecture=AArch64"
require_manifest_value "${source_manifest}" "soname=${EXPECTED_SONAME}"

LC_ALL=C readelf -h "${source_library}" | grep -q "Machine:.*AArch64" || \
  fail "SDK shared library is not an AArch64 ELF"
LC_ALL=C readelf -d "${source_library}" | grep -q "SONAME.*\[${EXPECTED_SONAME}\]" || \
  fail "SDK shared library has an unexpected SONAME"
if LC_ALL=C readelf -d "${source_library}" | grep -Eq "\((RPATH|RUNPATH)\)"; then
  fail "SDK shared library must not contain RPATH/RUNPATH"
fi

mapfile -t runtime_dependencies < <(
  LC_ALL=C readelf -d "${source_library}" |
    sed -n 's/.*Shared library: \[\([^]]*\)\].*/\1/p' |
    LC_ALL=C sort -u
)
expected_dependencies=(
  ld-linux-aarch64.so.1
  libc.so.6
  libcrypto.so.3
  libgcc_s.so.1
  libm.so.6
  libssl.so.3
  libstdc++.so.6
)
if [[ "${runtime_dependencies[*]}" != "${expected_dependencies[*]}" ]]; then
  fail "SDK runtime dependency set does not match the approved package contract"
fi

required_license_files=(
  LICENSE
  deps/json/LICENSE.MIT
  deps/libjuice/LICENSE
  deps/libsrtp/LICENSE
  deps/plog/LICENSE
  deps/usrsctp/LICENSE.md
)
for relative_path in "${required_license_files[@]}"; do
  [[ -s "${sdk_prefix}/share/licenses/libdatachannel/${relative_path}" ]] || \
    fail "SDK license bundle is missing: ${relative_path}"
done

library_directory="${output_root}/${RUNTIME_LIBRARY_DIRECTORY}"
share_directory="${output_root}/${RUNTIME_SHARE_DIRECTORY}"
install -d -m 0755 "${library_directory}" "${share_directory}/licenses"
install -m 0755 "${source_library}" "${library_directory}/${EXPECTED_LIBRARY}"
ln -s "${EXPECTED_LIBRARY}" "${library_directory}/${EXPECTED_SONAME}"
install -m 0644 "${source_manifest}" "${share_directory}/source-manifest.txt"
for relative_path in "${required_license_files[@]}"; do
  install -D -m 0644 "${sdk_prefix}/share/licenses/libdatachannel/${relative_path}" \
    "${share_directory}/licenses/${relative_path}"
done
printf '%s\n' "${runtime_dependencies[@]}" >"${share_directory}/runtime-dependencies.txt"

manifest_path="${share_directory}/runtime-package.sha256"
(
  cd "${output_root}"
  find "${RUNTIME_LIBRARY_DIRECTORY}" "${RUNTIME_SHARE_DIRECTORY}" -type f \
    ! -path "${RUNTIME_SHARE_DIRECTORY}/runtime-package.sha256" -print0 |
    LC_ALL=C sort -z |
    xargs -0 sha256sum
) >"${manifest_path}"
chmod 0644 "${manifest_path}"

echo "staged controlled libdatachannel runtime tree: ${output_root}"
echo "loader directory: ${RUNTIME_LIBRARY_DIRECTORY}"
