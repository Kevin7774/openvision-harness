#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
START_UP_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"

assertContains() {
    local file="$1"
    local expected="$2"

    grep -Fq -- "$expected" "$file" || {
        echo "契约缺失：$file 未包含 $expected" >&2
        return 1
    }
}

assertNotContains() {
    local file="$1"
    local unexpected="$2"

    if grep -Fq -- "$unexpected" "$file"; then
        echo "契约违规：$file 不应包含 $unexpected" >&2
        return 1
    fi
}

bash -n "$START_UP_DIR/generate_service.sh"

test_root="$(mktemp -d /tmp/astrabot-controller-rt-test.XXXXXX)"
trap 'rm -rf -- "$test_root"' EXIT

environment_file="$test_root/environment.sh"
controller_service="$test_root/Astrabot_Controller.service"
diagnostics_service="$test_root/Astrabot_Diagnostics.service"

printf '%s\n' 'THE_USER=astrabot' > "$environment_file"

# shellcheck source=/dev/null
source "$START_UP_DIR/generate_service.sh"

Generate_Service \
    "$test_root" \
    Astrabot_Controller \
    "$environment_file" \
    /bin/true \
    "$controller_service" \
    1 \
    thor

Generate_Service \
    "$test_root" \
    Astrabot_Diagnostics \
    "$environment_file" \
    /bin/true \
    "$diagnostics_service" \
    1 \
    thor

assertContains "$controller_service" "LimitRTPRIO=99"
assertContains "$controller_service" "LimitMEMLOCK=infinity"
assertNotContains "$diagnostics_service" "LimitRTPRIO="
assertNotContains "$diagnostics_service" "LimitMEMLOCK="

if command -v systemd-analyze >/dev/null 2>&1; then
    systemd-analyze verify "$controller_service" "$diagnostics_service"
fi
