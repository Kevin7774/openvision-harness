#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODULE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

mapfile -t cpp_files < <(
  find "${MODULE_ROOT}/include" "${MODULE_ROOT}/src" "${MODULE_ROOT}/test" \
    -type f \( -name '*.h' -o -name '*.hpp' -o -name '*.cpp' \) | sort
)
if [[ "${#cpp_files[@]}" -eq 0 ]]; then
  echo "no C++ files found" >&2
  exit 1
fi

clang-format --dry-run --Werror "${cpp_files[@]}"
cmake -DROOT="${MODULE_ROOT}" -P "${MODULE_ROOT}/cmake/no_exceptions_check.cmake"
