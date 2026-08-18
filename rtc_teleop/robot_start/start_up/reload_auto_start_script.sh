#!/bin/bash
set -Eeuo pipefail

echo '---------reload auto start scripts----------'

source astrabot_environment
source "${START_UP_DIR}/${FUNCTION_DIR}/generate_service.sh"

FAST_DDS_SHM_GATE_SCRIPT="${START_UP_DIR}/${FUNCTION_DIR}/fastdds_shm_refresh_gate.sh"

if [[ -z "${START_UP_DIR:-}" || "$START_UP_DIR" == "/" ]]; then
    echo "invalid START_UP_DIR: ${START_UP_DIR:-<empty>}" >&2
    return 1 2>/dev/null || exit 1
fi

DATA_COLLECTION_WAS_ENABLED=false
data_collection_state="$(systemctl is-enabled Astrabot_Data_Collection.service 2>/dev/null || true)"
case "$data_collection_state" in
    enabled|enabled-runtime|linked|linked-runtime|alias)
        DATA_COLLECTION_WAS_ENABLED=true
        ;;
    disabled|indirect|static|masked|masked-runtime|not-found|"")
        ;;
    *)
        echo "cannot determine Data Collection enable state: $data_collection_state" >&2
        return 1 2>/dev/null || exit 1
        ;;
esac

Read_Optional_Unit_Enable_State() {
    local unit="$1"
    local state

    state="$(systemctl is-enabled "$unit" 2>/dev/null || true)"
    case "$state" in
        enabled|enabled-runtime|linked|linked-runtime|alias)
            printf '%s\n' true
            ;;
        disabled|indirect|static|masked|masked-runtime|not-found|"")
            printf '%s\n' false
            ;;
        *)
            echo "cannot determine ${unit} enable state: $state" >&2
            return 1
            ;;
    esac
}

RTC_WAS_ENABLED="$(Read_Optional_Unit_Enable_State Astrabot_Rtc.service)"
TELEOP_WAS_ENABLED="$(Read_Optional_Unit_Enable_State Astrabot_Teleop.service)"
LIVEKIT_WAS_ENABLED="$(Read_Optional_Unit_Enable_State Astrabot_LiveKit.service)"
LIVEKIT_SRV_WAS_ENABLED="$(Read_Optional_Unit_Enable_State Astrabot_LiveKit_Srv.service)"

# reload 前记录所有 Astrabot 服务的 enable 状态，避免清理旧 unit 时丢失自启配置。
ENABLED_ASTRABOT_UNITS=()
mapfile -t ENABLED_ASTRABOT_UNITS < <(
    systemctl list-unit-files 'Astrabot_*.service' --no-legend 2>/dev/null \
        | awk '$2 == "enabled" || $2 == "enabled-runtime" {print $1}'
)

TAG_DIR="${START_UP_DIR}/${AUTO_START_SCRIPT_DIR}"
RELOAD_TMP_DIR="$(mktemp -d /tmp/astrabot-reload.XXXXXX)"
TMP_RUN_DIR="${RELOAD_TMP_DIR}/run"
TMP_SERVICE_DIR="${RELOAD_TMP_DIR}/service"
OLD_RUN_DIR="${RELOAD_TMP_DIR}/old-run"
OLD_SERVICE_DIR="${RELOAD_TMP_DIR}/old-service"
UNIT_BACKUP_DIR=""
DESTRUCTIVE_STARTED=false
RELOAD_COMPLETE=false
mkdir -p "$TMP_RUN_DIR" "$TMP_SERVICE_DIR" "$OLD_RUN_DIR" "$OLD_SERVICE_DIR"
cp -a "${START_UP_DIR}/${RUN_DIR}/." "$OLD_RUN_DIR/"
cp -a "${START_UP_DIR}/${SERVICE_DIR}/." "$OLD_SERVICE_DIR/"

finish_reload() {
    local result=$?
    local shm_ready=false
    trap - EXIT
    set +e

    if (( result != 0 )) && [ "$DESTRUCTIVE_STARTED" = true ] && \
       [ "$RELOAD_COMPLETE" = false ]; then
        echo "reload failed, restoring previous robot_start services" >&2
        echo "$THE_PASSWORD" | sudo -S astrabot stop || true
        if echo "$THE_PASSWORD" | sudo -S bash "$FAST_DDS_SHM_GATE_SCRIPT" clean; then
            shm_ready=true
        else
            echo "Fast DDS SHM cleanup failed during reload rollback; restored services will remain stopped." >&2
        fi
        find "${START_UP_DIR}/${RUN_DIR}" -mindepth 1 -maxdepth 1 -delete
        find "${START_UP_DIR}/${SERVICE_DIR}" -mindepth 1 -maxdepth 1 -delete
        cp -a "$OLD_RUN_DIR/." "${START_UP_DIR}/${RUN_DIR}/"
        cp -a "$OLD_SERVICE_DIR/." "${START_UP_DIR}/${SERVICE_DIR}/"

        if [[ -n "$UNIT_BACKUP_DIR" && -d "$UNIT_BACKUP_DIR" ]]; then
            echo "$THE_PASSWORD" | sudo -S find /etc/systemd/system -maxdepth 1 \
                \( -type f -o -type l \) -name 'Astrabot_*.service' -delete
            echo "$THE_PASSWORD" | sudo -S cp -a \
                "$UNIT_BACKUP_DIR/." /etc/systemd/system/
            echo "$THE_PASSWORD" | sudo -S systemctl daemon-reload

            rollback_enable_units=()
            for rollback_unit in "${ENABLED_ASTRABOT_UNITS[@]}"; do
                [[ "$rollback_unit" == "Astrabot_Data_Collection.service" ||
                   "$rollback_unit" == "Astrabot_Rtc.service" ||
                   "$rollback_unit" == "Astrabot_Teleop.service" ]] && continue
                if [ -f "/etc/systemd/system/$rollback_unit" ]; then
                    rollback_enable_units+=("$rollback_unit")
                fi
            done
            if (( ${#rollback_enable_units[@]} > 0 )); then
                echo "$THE_PASSWORD" | sudo -S systemctl enable "${rollback_enable_units[@]}"
            fi
            if [[ "$shm_ready" = true ]]; then
                echo "$THE_PASSWORD" | sudo -S astrabot init
            fi
            if [[ "$shm_ready" = true && "$DATA_COLLECTION_WAS_ENABLED" = true ]] && \
               [ -f /etc/systemd/system/Astrabot_Data_Collection.service ]; then
                echo "$THE_PASSWORD" | sudo -S systemctl enable --now \
                    Astrabot_Data_Collection.service
            fi
            if [[ "$shm_ready" = true && "$LIVEKIT_WAS_ENABLED" = true ]] && \
               [ -f /etc/systemd/system/Astrabot_LiveKit.service ]; then
                echo "$THE_PASSWORD" | sudo -S systemctl enable --now Astrabot_LiveKit.service
            elif [ -f /etc/systemd/system/Astrabot_LiveKit.service ]; then
                echo "$THE_PASSWORD" | sudo -S systemctl disable --now Astrabot_LiveKit.service
            fi
            if [[ "$shm_ready" = true && "$LIVEKIT_SRV_WAS_ENABLED" = true ]] && \
               [ -f /etc/systemd/system/Astrabot_LiveKit_Srv.service ]; then
                echo "$THE_PASSWORD" | sudo -S systemctl enable --now Astrabot_LiveKit_Srv.service
            elif [ -f /etc/systemd/system/Astrabot_LiveKit_Srv.service ]; then
                echo "$THE_PASSWORD" | sudo -S systemctl disable --now Astrabot_LiveKit_Srv.service
            fi
            if [[ "$shm_ready" = true && "$RTC_WAS_ENABLED" = true ]] && \
               [ -f /etc/systemd/system/Astrabot_Rtc.service ]; then
                echo "$THE_PASSWORD" | sudo -S systemctl enable --now Astrabot_Rtc.service
            elif [ -f /etc/systemd/system/Astrabot_Rtc.service ]; then
                echo "$THE_PASSWORD" | sudo -S systemctl disable --now Astrabot_Rtc.service
            fi
            if [[ "$shm_ready" = true && "$TELEOP_WAS_ENABLED" = true ]] && \
               [ -f /etc/systemd/system/Astrabot_Teleop.service ]; then
                echo "$THE_PASSWORD" | sudo -S systemctl enable --now Astrabot_Teleop.service
            elif [ -f /etc/systemd/system/Astrabot_Teleop.service ]; then
                echo "$THE_PASSWORD" | sudo -S systemctl disable --now Astrabot_Teleop.service
            fi
            if [[ "$shm_ready" = true ]] &&
               ! echo "$THE_PASSWORD" | sudo -S -u "$THE_USER" \
                   bash "$FAST_DDS_SHM_GATE_SCRIPT" verify; then
                echo "Restored robot_start failed local ROS topic verification." >&2
            fi
        fi
    fi

    rm -rf -- "$RELOAD_TMP_DIR"
    exit "$result"
}
trap finish_reload EXIT

backup_astrabot_units() {
    local unit_file
    UNIT_BACKUP_DIR="/etc/systemd/astrabot-backup/$(date +%Y%m%d-%H%M%S)-reload-$$"
    echo "$THE_PASSWORD" | sudo -S install -d -m 0755 "$UNIT_BACKUP_DIR"
    while IFS= read -r unit_file; do
        echo "$THE_PASSWORD" | sudo -S cp -a -- "$unit_file" "$UNIT_BACKUP_DIR/"
    done < <(find /etc/systemd/system -maxdepth 1 \
        \( -type f -o -type l \) -name 'Astrabot_*.service' -print)
    echo "Astrabot unit backup: $UNIT_BACKUP_DIR"
}

remove_astrabot_units() {
    local -a units=()
    mapfile -t units < <(
        systemctl list-unit-files 'Astrabot_*.service' --no-legend 2>/dev/null \
            | awk '{print $1}'
    )
    if (( ${#units[@]} > 0 )); then
        echo "$THE_PASSWORD" | sudo -S systemctl disable --now "${units[@]}" || true
    fi
    echo "$THE_PASSWORD" | sudo -S find /etc/systemd/system -maxdepth 1 \
        \( -type f -o -type l \) -name 'Astrabot_*.service' -delete
    echo "$THE_PASSWORD" | sudo -S systemctl daemon-reload
}

# 先在临时目录完整生成并验证，失败时不影响当前已安装版本。
cp -f "${START_UP_DIR}/${FUNCTION_DIR}/environment.sh" "$TMP_RUN_DIR/environment.sh"
cp -f "${START_UP_DIR}/${FUNCTION_DIR}/execute.sh" "$TMP_RUN_DIR/execute.sh"
cp -f "${START_UP_DIR}/${FUNCTION_DIR}/logger.sh" "$TMP_RUN_DIR/logger.sh"
cp -f "${START_UP_DIR}/${FUNCTION_DIR}/wait-for-timesync.sh" "$TMP_RUN_DIR/wait-for-timesync.sh"
cp -f "${START_UP_DIR}/${FUNCTION_DIR}/wait-controller-node.sh" "$TMP_RUN_DIR/wait-controller-node.sh"
cp -f "${START_UP_DIR}/${FUNCTION_DIR}/wait-and-sync-ccu-time.sh" "$TMP_RUN_DIR/wait-and-sync-ccu-time.sh"
cp -f "${START_UP_DIR}/${FUNCTION_DIR}/prepare-zed-usb.sh" "$TMP_RUN_DIR/prepare-zed-usb.sh"

for file_r in "$TAG_DIR"/*.start_script; do
    [[ -e "$file_r" ]] || continue
    file="$(basename "$file_r")"
    run_order="$(cat "$file_r")"
    priority_line="$(grep -m1 '^PRIORITY=' "$file_r")"
    priority="${priority_line#*=}"
    file_name="$(basename "$file" .start_script)"
    staged_run_file="${TMP_RUN_DIR}/${file_name}.sh"
    final_run_file="${START_UP_DIR}/${RUN_DIR}/${file_name}.sh"
    staged_service_file="${TMP_SERVICE_DIR}/${file_name}.service"

    echo "priority: $priority"
    Generate_Run "$run_order" "$file_name" "$staged_run_file" \
        "${START_UP_DIR}/${RUN_DIR}" astrabot_environment
    Generate_Service "${START_UP_DIR}/${RUN_DIR}" "$file_name" \
        astrabot_environment "$final_run_file" "$staged_service_file" \
        "$priority" "$BOARD_TYPE"
done

if [ -d "$TAG_DIR/supplement/service" ]; then
    echo "copy supplement service files"
    cp -rf "$TAG_DIR/supplement/service/." "$TMP_SERVICE_DIR/"
fi
if [ -d "$TAG_DIR/supplement/run" ]; then
    echo "copy supplement run files"
    cp -rf "$TAG_DIR/supplement/run/." "$TMP_RUN_DIR/"
fi

service_files=()
for service_file in "$TMP_SERVICE_DIR"/*.service; do
    [[ -e "$service_file" ]] || continue
    service_files+=("$service_file")
done
if (( ${#service_files[@]} == 0 )); then
    echo "no generated service files found" >&2
    exit 1
fi
systemd-analyze verify "${service_files[@]}"

# 验证通过后才停止旧服务并替换运行文件。
DESTRUCTIVE_STARTED=true
echo "$THE_PASSWORD" | sudo -S astrabot stop || true
backup_astrabot_units
remove_astrabot_units
echo "$THE_PASSWORD" | sudo -S bash "$FAST_DDS_SHM_GATE_SCRIPT" clean

find "${START_UP_DIR}/${RUN_DIR}" -mindepth 1 -maxdepth 1 -delete
find "${START_UP_DIR}/${SERVICE_DIR}" -mindepth 1 -maxdepth 1 -delete
cp -rf "$TMP_RUN_DIR/." "${START_UP_DIR}/${RUN_DIR}/"
cp -rf "$TMP_SERVICE_DIR/." "${START_UP_DIR}/${SERVICE_DIR}/"

echo "$THE_PASSWORD" | sudo -S cp -rf \
    "${START_UP_DIR}/${SERVICE_DIR}/." /etc/systemd/system/
echo "$THE_PASSWORD" | sudo -S systemctl daemon-reload

if [ -d "$TAG_DIR/supplement/config" ]; then
    echo "copy supplement config files"
    cp -rf "$TAG_DIR/supplement/config/." "${START_UP_DIR}/${CONFIG_DIR}/"
    rm -f "${START_UP_DIR}/${CONFIG_DIR}/data_collection/astrabot.yaml"
    rm -f "${START_UP_DIR}/${CONFIG_DIR}/livekit/livekit.env"
fi

units_to_restore=()
for unit in "${ENABLED_ASTRABOT_UNITS[@]}"; do
    [[ "$unit" == "Astrabot_Data_Collection.service" ||
       "$unit" == "Astrabot_LiveKit.service" ||
       "$unit" == "Astrabot_LiveKit_Srv.service" ||
       "$unit" == "Astrabot_Rtc.service" ||
       "$unit" == "Astrabot_Teleop.service" ]] && continue
    if [ -f "/etc/systemd/system/$unit" ]; then
        units_to_restore+=("$unit")
    fi
done
if (( ${#units_to_restore[@]} > 0 )); then
    echo "$THE_PASSWORD" | sudo -S systemctl enable "${units_to_restore[@]}"
fi

# init 启动普通 Astrabot 服务，但按约定跳过 Data Collection、RTC 和 Teleop 等 opt-in 服务。
echo "$THE_PASSWORD" | sudo -S astrabot init

# reload 只重建服务，不改变数采或 legacy Teleop rollback unit 原有的 enable 状态。
if [ -f "/etc/systemd/system/Astrabot_Data_Collection.service" ]; then
    if [ "$DATA_COLLECTION_WAS_ENABLED" = true ]; then
        echo "$THE_PASSWORD" | sudo -S systemctl enable --now Astrabot_Data_Collection.service
    else
        echo "$THE_PASSWORD" | sudo -S systemctl disable --now Astrabot_Data_Collection.service
    fi
fi
if [ -f "/etc/systemd/system/Astrabot_LiveKit.service" ]; then
    if [ "$LIVEKIT_WAS_ENABLED" = true ]; then
        echo "$THE_PASSWORD" | sudo -S systemctl enable --now Astrabot_LiveKit.service
    else
        echo "$THE_PASSWORD" | sudo -S systemctl disable --now Astrabot_LiveKit.service
    fi
fi
if [ -f "/etc/systemd/system/Astrabot_LiveKit_Srv.service" ]; then
    if [ "$LIVEKIT_SRV_WAS_ENABLED" = true ]; then
        echo "$THE_PASSWORD" | sudo -S systemctl enable --now Astrabot_LiveKit_Srv.service
    else
        echo "$THE_PASSWORD" | sudo -S systemctl disable --now Astrabot_LiveKit_Srv.service
    fi
fi

# RTC/Teleop 默认不参与整机 init/enable；reload 只恢复操作员此前显式设置的 enable 状态。
if [ -f "/etc/systemd/system/Astrabot_Rtc.service" ]; then
    if [ "$RTC_WAS_ENABLED" = true ]; then
        echo "$THE_PASSWORD" | sudo -S systemctl enable --now Astrabot_Rtc.service
    else
        echo "$THE_PASSWORD" | sudo -S systemctl disable --now Astrabot_Rtc.service
    fi
fi
if [ -f "/etc/systemd/system/Astrabot_Teleop.service" ]; then
    if [ "$TELEOP_WAS_ENABLED" = true ]; then
        echo "$THE_PASSWORD" | sudo -S systemctl enable --now Astrabot_Teleop.service
    else
        echo "$THE_PASSWORD" | sudo -S systemctl disable --now Astrabot_Teleop.service
    fi
fi

echo "$THE_PASSWORD" | sudo -S -u "$THE_USER" \
    bash "$FAST_DDS_SHM_GATE_SCRIPT" verify

RELOAD_COMPLETE=true
trap - EXIT
rm -rf -- "$RELOAD_TMP_DIR"
