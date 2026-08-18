#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
START_UP_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
THOR_AGENT_SCRIPT="$START_UP_DIR/run_script/thor/Astrabot_Log_Agent.start_script"
THOR_HUB_SCRIPT="$START_UP_DIR/run_script/thor/Astrabot_Log_Hub.start_script"
ECU_AGENT_SCRIPT="$START_UP_DIR/run_script/ecu/Astrabot_Log_Agent.start_script"

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

for script in \
    "$START_UP_DIR/wait-controller-node.sh" \
    "$START_UP_DIR/wait-and-sync-ccu-time.sh" \
    "$START_UP_DIR/wait-for-timesync.sh"; do
    bash -n "$script"
    assertNotContains "$script" "#!/bin/bash -x"
    assertNotContains "$script" "set -x"
    assertNotContains "$script" "set -o xtrace"
done

for script in "$THOR_AGENT_SCRIPT" "$THOR_HUB_SCRIPT" "$ECU_AGENT_SCRIPT"; do
    bash -n "$script"
    assertNotContains "$script" "/opt/ros/astrabot_log/config/"
done

assertContains "$THOR_AGENT_SCRIPT" \
    'config_path:="$(ros2 pkg prefix --share astrabot_log_agent)/config/agent.yaml"'
assertContains "$ECU_AGENT_SCRIPT" \
    'config_path:="$(ros2 pkg prefix --share astrabot_log_agent)/config/agent.yaml"'
assertContains "$THOR_HUB_SCRIPT" \
    'config_path:="$(ros2 pkg prefix --share astrabot_log_hub)/config/hub.yaml"'
