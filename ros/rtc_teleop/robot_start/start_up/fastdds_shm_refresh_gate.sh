#!/usr/bin/env bash
set -Eeuo pipefail

ACTION="${1:-}"
PROC_ROOT="${ASTRABOT_PROC_ROOT:-/proc}"
SHM_ROOT="${ASTRABOT_FASTDDS_SHM_DIR:-/dev/shm}"
FASTDDS_COMMAND="${ASTRABOT_FASTDDS_COMMAND:-}"
ROS_DISTRO_PREFIX="${ASTRABOT_ROS_DISTRO_PREFIX:-/opt/ros/jazzy}"
ROS_SETUP_FILE="${ASTRABOT_ROS_SETUP:-/opt/ros/astrabot/setup.bash}"
ROS_CONFIG_FILE="${ASTRABOT_ROS_CONFIG:-/opt/ros/start_up/config/ros_config.sh}"
STARTUP_ENVIRONMENT_FILE="${ASTRABOT_STARTUP_ENVIRONMENT:-/opt/ros/start_up/run/environment.sh}"
BOARD_CONFIG_FILE="${ASTRABOT_BOARD_CONFIG:-/etc/astrabot/board.yaml}"
VERIFY_TOPIC="${ASTRABOT_REFRESH_VERIFY_TOPIC:-}"
VERIFY_TIMEOUT_SEC="${ASTRABOT_REFRESH_VERIFY_TIMEOUT_SEC:-60}"
LOCAL_PROBE_TOPIC_PREFIX="${ASTRABOT_REFRESH_PROBE_TOPIC_PREFIX:-/astrabot/refresh_probe}"

declare -A ALLOWED_PIDS=()

Resolve_Fastdds_Command() {
    if [[ -n "$FASTDDS_COMMAND" ]]; then
        command -v "$FASTDDS_COMMAND" >/dev/null 2>&1
        return
    fi

    if command -v fastdds >/dev/null 2>&1; then
        FASTDDS_COMMAND="$(command -v fastdds)"
    elif [[ -x "$ROS_DISTRO_PREFIX/bin/fastdds" ]]; then
        # Debian postinst/sudo normally does not inherit /opt/ros/<distro>/bin in PATH.
        FASTDDS_COMMAND="$ROS_DISTRO_PREFIX/bin/fastdds"
    else
        return 1
    fi
}

Resolve_Verify_Topic() {
    local ros_namespace=""

    if [[ -n "$VERIFY_TOPIC" ]]; then
        return 0
    fi
    [[ -r "$BOARD_CONFIG_FILE" ]] || {
        echo "错误：Astrabot board 配置不存在或不可读：$BOARD_CONFIG_FILE" >&2
        return 1
    }
    ros_namespace="$(awk '
        /^[[:space:]]*ros_namespace:[[:space:]]*/ {
            sub(/^[[:space:]]*ros_namespace:[[:space:]]*/, "")
            gsub(/[[:space:]\047\042]/, "")
            print
            exit
        }
    ' "$BOARD_CONFIG_FILE")"
    if [[ "$ros_namespace" != /* || "$ros_namespace" == "/" || "$ros_namespace" == */ ]]; then
        echo "错误：board 配置中的 ros_namespace 无效：${ros_namespace:-<empty>}" >&2
        return 1
    fi
    VERIFY_TOPIC="${ros_namespace}/diagnostics/system"
}

Remember_Invoker_Process_Tree() {
    local pid="$$"
    local parent_pid

    while [[ "$pid" =~ ^[0-9]+$ ]] && (( pid > 1 )); do
        ALLOWED_PIDS["$pid"]=1
        if [[ ! -r "$PROC_ROOT/$pid/status" ]]; then
            break
        fi
        parent_pid="$(awk '$1 == "PPid:" {print $2}' "$PROC_ROOT/$pid/status")"
        if [[ ! "$parent_pid" =~ ^[0-9]+$ ]] || (( parent_pid == pid )); then
            break
        fi
        pid="$parent_pid"
    done
}

Process_Holds_Fastdds_Shm() {
    local process_dir="$1"
    local fd
    local target

    if [[ -d "$process_dir/fd" ]]; then
        for fd in "$process_dir"/fd/*; do
            [[ -e "$fd" || -L "$fd" ]] || continue
            target="$(readlink "$fd" 2>/dev/null || true)"
            case "$target" in
                "$SHM_ROOT"/fastrtps*) return 0 ;;
            esac
        done
    fi
    if [[ -r "$process_dir/maps" ]] && grep -Fq -- "$SHM_ROOT/fastrtps" "$process_dir/maps"; then
        return 0
    fi
    return 1
}

Process_Is_Ros_Or_Fastdds() {
    local process_dir="$1"
    local comm=""

    if [[ -r "$process_dir/comm" ]]; then
        IFS= read -r comm < "$process_dir/comm" || true
    fi
    case "$comm" in
        astrabot*|component_container*|fastdds|fastrtps*|ros2) return 0 ;;
        # Login shells and remote-editor hosts commonly inherit ROS_DISTRO from
        # /etc/bash.bashrc without creating a DDS participant. A real participant
        # in one of these processes is still caught by Process_Holds_Fastdds_Shm.
        bash|dash|node|sh|ssh|sshd|zsh) return 1 ;;
    esac
    if [[ -r "$process_dir/environ" ]] &&
       tr '\0' '\n' < "$process_dir/environ" |
           grep -Eq '^(ROS_DISTRO=|RMW_IMPLEMENTATION=.*fastrtps)'; then
        return 0
    fi
    return 1
}

List_Blocking_Processes() {
    local process_dir
    local pid
    local comm="unknown"

    for process_dir in "$PROC_ROOT"/[0-9]*; do
        [[ -d "$process_dir" ]] || continue
        pid="${process_dir##*/}"
        [[ -z "${ALLOWED_PIDS[$pid]:-}" ]] || continue
        if ! Process_Holds_Fastdds_Shm "$process_dir" && ! Process_Is_Ros_Or_Fastdds "$process_dir"; then
            continue
        fi
        if [[ -r "$process_dir/comm" ]]; then
            IFS= read -r comm < "$process_dir/comm" || comm="unknown"
        fi
        printf 'pid=%s comm=%s\n' "$pid" "$comm"
    done
}

Clean_Fastdds_Shm() {
    local -a blockers=()

    Remember_Invoker_Process_Tree
    mapfile -t blockers < <(List_Blocking_Processes)
    if (( ${#blockers[@]} > 0 )); then
        echo "错误：仍有 ROS/Fast DDS/Astrabot 进程或 SHM 映射，拒绝清理：" >&2
        printf '  %s\n' "${blockers[@]}" >&2
        return 1
    fi
    if ! Resolve_Fastdds_Command; then
        echo "错误：找不到 Fast DDS CLI；已检查 PATH 和 $ROS_DISTRO_PREFIX/bin/fastdds" >&2
        return 1
    fi

    "$FASTDDS_COMMAND" shm clean
    echo "Fast DDS SHM 清理完成。"
}

Verify_Local_Ros_Topic() {
    local deadline
    local probe_echo_pid
    local probe_topic
    local topic_type=""
    if [[ ! "$VERIFY_TIMEOUT_SEC" =~ ^[0-9]+$ ]] ||
       (( VERIFY_TIMEOUT_SEC < 1 || VERIFY_TIMEOUT_SEC > 120 )); then
        echo "错误：ASTRABOT_REFRESH_VERIFY_TIMEOUT_SEC 必须位于 1..120。" >&2
        return 1
    fi
    Resolve_Verify_Topic || return 1
    if [[ "$VERIFY_TOPIC" != /* || "$VERIFY_TOPIC" == "/" ]]; then
        echo "错误：ASTRABOT_REFRESH_VERIFY_TOPIC 必须是非根绝对 topic。" >&2
        return 1
    fi
    if [[ "$LOCAL_PROBE_TOPIC_PREFIX" != /* || "$LOCAL_PROBE_TOPIC_PREFIX" == "/" ||
          "$LOCAL_PROBE_TOPIC_PREFIX" == */ ]]; then
        echo "错误：ASTRABOT_REFRESH_PROBE_TOPIC_PREFIX 必须是非根绝对 topic 前缀。" >&2
        return 1
    fi
    [[ -r "$ROS_CONFIG_FILE" ]] || {
        echo "错误：ROS 配置不存在或不可读：$ROS_CONFIG_FILE" >&2
        return 1
    }
    [[ -r "$STARTUP_ENVIRONMENT_FILE" ]] || {
        echo "错误：Astrabot startup environment 不存在或不可读：$STARTUP_ENVIRONMENT_FILE" >&2
        return 1
    }
    [[ -r "$ROS_SETUP_FILE" ]] || {
        echo "错误：Astrabot ROS overlay 不存在或不可读：$ROS_SETUP_FILE" >&2
        return 1
    }

    set +u
    # 仅验证本机发现与通信，避免远端同名 topic 掩盖本机 SHM 分裂。
    # shellcheck disable=SC1090
    source "$STARTUP_ENVIRONMENT_FILE"
    # shellcheck disable=SC1090
    source "$ROS_CONFIG_FILE"
    # shellcheck disable=SC1090
    source "$ROS_SETUP_FILE"
    set -u
    if [[ ! "${ROS_DOMAIN_ID:-}" =~ ^[0-9]+$ ]] || (( ROS_DOMAIN_ID > 232 )); then
        echo "错误：Astrabot startup environment 中的 ROS_DOMAIN_ID 无效：${ROS_DOMAIN_ID:-<empty>}" >&2
        return 1
    fi
    command -v ros2 >/dev/null 2>&1 || {
        echo "错误：Astrabot ROS 环境中找不到 ros2。" >&2
        return 1
    }

    deadline=$((SECONDS + VERIFY_TIMEOUT_SEC))
    while (( SECONDS < deadline )); do
        topic_type="$(ros2 topic type "$VERIFY_TOPIC" --no-daemon 2>/dev/null || true)"
        [[ -n "$topic_type" ]] && break
        sleep 1
    done
    if [[ -z "$topic_type" ]]; then
        echo "错误：软件刷新后本机关键 topic 类型发现超时：$VERIFY_TOPIC" >&2
        return 1
    fi

    if ! timeout --signal=TERM "${VERIFY_TIMEOUT_SEC}s" \
        ros2 topic echo "$VERIFY_TOPIC" "$topic_type" --full-length --once \
            --no-daemon >/dev/null; then
        echo "错误：软件刷新后本机关键 topic 验证失败：$VERIFY_TOPIC" >&2
        return 1
    fi

    # 关键 topic 可能在同一 DDS domain 的其他设备上也存在。额外使用进程唯一 topic
    # 完成本机 pub/sub，确保软件刷新后本机 DDS 通信没有因残留 SHM 而分裂。
    probe_topic="${LOCAL_PROBE_TOPIC_PREFIX}_$$_${RANDOM}"
    timeout --signal=TERM "${VERIFY_TIMEOUT_SEC}s" \
        ros2 topic echo "$probe_topic" std_msgs/msg/String --once \
            --no-daemon >/dev/null &
    probe_echo_pid=$!
    sleep 1
    if ! timeout --signal=TERM "${VERIFY_TIMEOUT_SEC}s" \
        ros2 topic pub "$probe_topic" std_msgs/msg/String \
            "{data: local_fastdds_probe}" --once >/dev/null; then
        kill "$probe_echo_pid" 2>/dev/null || true
        wait "$probe_echo_pid" 2>/dev/null || true
        echo "错误：软件刷新后本机 Fast DDS 探针发布失败：$probe_topic" >&2
        return 1
    fi
    if ! wait "$probe_echo_pid"; then
        echo "错误：软件刷新后本机 Fast DDS 探针接收失败：$probe_topic" >&2
        return 1
    fi
    echo "软件刷新后本机关键 topic 验证通过：$VERIFY_TOPIC"
}

case "$ACTION" in
    clean)
        Clean_Fastdds_Shm
        ;;
    verify)
        Verify_Local_Ros_Topic
        ;;
    *)
        echo "用法：$0 clean|verify" >&2
        exit 2
        ;;
esac
