#!/bin/bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

FAST_DDS_SHM_GATE_SCRIPT="${SCRIPT_DIR}/fastdds_shm_refresh_gate.sh"

install_mode="${1:-}"

#判断install_mode是否为ecu、ccu、agx、test中的一个
if [ "$install_mode" != "ecu" ] && [ "$install_mode" != "thor" ]; then
    echo "install_mode is not ecu, thor"
    exit 1
fi

Environment_Script="environment/$install_mode.sh"

source "${Environment_Script}"

if [[ -z "${START_UP_DIR:-}" || "$START_UP_DIR" == "/" ]]; then
    echo "invalid START_UP_DIR: ${START_UP_DIR:-<empty>}" >&2
    exit 1
fi

echo "Environment_Script: ${Environment_Script}"

UNIT_BACKUP_DIR=""
START_UP_BACKUP_DIR=""
PREVIOUS_DATA_COLLECTION_ENABLED=false
PREVIOUS_ENABLED_UNITS=()
mapfile -t PREVIOUS_ENABLED_UNITS < <(
    systemctl list-unit-files 'Astrabot_*.service' --no-legend 2>/dev/null \
        | awk '$2 == "enabled" || $2 == "enabled-runtime" {print $1}'
)
for previous_unit in "${PREVIOUS_ENABLED_UNITS[@]}"; do
    if [[ "$previous_unit" == "Astrabot_Data_Collection.service" ]]; then
        PREVIOUS_DATA_COLLECTION_ENABLED=true
        break
    fi
done

Rollback_Install() {
    local result=$?
    local shm_ready=false
    trap - ERR
    set +e
    echo "install failed, restoring previous robot_start installation" >&2

    if [[ -x /usr/local/bin/astrabot ]]; then
        sudo astrabot stop || true
    fi
    if sudo bash "$FAST_DDS_SHM_GATE_SCRIPT" clean; then
        shm_ready=true
    else
        echo "Fast DDS SHM cleanup failed during rollback; restored services will remain stopped." >&2
    fi

    if [[ -n "$START_UP_BACKUP_DIR" && -d "$START_UP_BACKUP_DIR" ]]; then
        sudo rm -rf -- "$START_UP_DIR"
        sudo mv -- "$START_UP_BACKUP_DIR" "$START_UP_DIR"
    fi

    if [[ -n "$UNIT_BACKUP_DIR" && -d "$UNIT_BACKUP_DIR" ]]; then
        sudo find /etc/systemd/system -maxdepth 1 \
            \( -type f -o -type l \) -name 'Astrabot_*.service' -delete
        sudo cp -a "$UNIT_BACKUP_DIR/." /etc/systemd/system/
        sudo systemctl daemon-reload

        rollback_enable_units=()
        for rollback_unit in "${PREVIOUS_ENABLED_UNITS[@]}"; do
            [[ "$rollback_unit" == "Astrabot_Data_Collection.service" ]] && continue
            if [ -f "/etc/systemd/system/$rollback_unit" ]; then
                rollback_enable_units+=("$rollback_unit")
            fi
        done
        if (( ${#rollback_enable_units[@]} > 0 )); then
            sudo systemctl enable "${rollback_enable_units[@]}"
        fi
        if [[ "$shm_ready" = true && -x /usr/local/bin/astrabot ]]; then
            sudo astrabot init
        fi
        if [[ "$shm_ready" = true && "$PREVIOUS_DATA_COLLECTION_ENABLED" = true ]] && \
           [ -f /etc/systemd/system/Astrabot_Data_Collection.service ]; then
            sudo systemctl enable --now Astrabot_Data_Collection.service
        fi
        if [[ "$shm_ready" = true ]] &&
           ! sudo -u "$THE_USER" bash "$FAST_DDS_SHM_GATE_SCRIPT" verify; then
            echo "Restored robot_start failed local ROS topic verification." >&2
        fi
    fi
    exit "$result"
}
trap Rollback_Install ERR

Mk_Dir() {
    local dir="$1"

    if [ -d "$dir" ]; then
        echo "${dir} directory exists"
    else
        echo "${dir} does not exists, so let's  mkdir it"
        sudo mkdir -p -- "$dir"
    fi

    sudo chgrp -R "$THE_USER" "$dir"
    sudo chown -R "$THE_USER" "$dir"
}

Ensure_Astrabot_Data_Dirs() {
    local -a writable_dirs=(
        /data/astrabot/file_transfer
        /data/astrabot/log/agent
        /data/astrabot/log/hub
    )

    sudo install -d -o root -g root -m 0755 /data/astrabot
    sudo install -d -o root -g "$THE_USER" -m 0750 /data/astrabot/log
    sudo install -d -o "$THE_USER" -g "$THE_USER" -m 0750 "${writable_dirs[@]}"
    for data_dir in "${writable_dirs[@]}"; do
        if ! sudo -u "$THE_USER" test -w "$data_dir"; then
            echo "Astrabot data directory is not writable by $THE_USER: $data_dir" >&2
            return 1
        fi
    done
}

Backup_Astrabot_Units() {
    UNIT_BACKUP_DIR="/etc/systemd/astrabot-backup/$(date +%Y%m%d-%H%M%S)-$$"
    sudo install -d -m 0755 "$UNIT_BACKUP_DIR"
    while IFS= read -r unit_file; do
        sudo cp -a -- "$unit_file" "$UNIT_BACKUP_DIR/"
    done < <(find /etc/systemd/system -maxdepth 1 \
        \( -type f -o -type l \) -name 'Astrabot_*.service' -print)
    echo "Astrabot unit backup: $UNIT_BACKUP_DIR"
}

Remove_Astrabot_Units() {
    local -a units=()
    mapfile -t units < <(
        systemctl list-unit-files 'Astrabot_*.service' --no-legend 2>/dev/null \
            | awk '{print $1}'
    )
    if (( ${#units[@]} > 0 )); then
        sudo systemctl disable --now "${units[@]}" || true
    fi
    sudo find /etc/systemd/system -maxdepth 1 \
        \( -type f -o -type l \) -name 'Astrabot_*.service' -delete
    sudo systemctl daemon-reload
}

Verify_Generated_Units() {
    local service_file
    local -a service_files=()
    for service_file in "${START_UP_DIR}/${SERVICE_DIR}"/*.service; do
        [[ -e "$service_file" ]] || continue
        service_files+=("$service_file")
    done
    if (( ${#service_files[@]} == 0 )); then
        echo "no generated service files found" >&2
        return 1
    fi
    systemd-analyze verify "${service_files[@]}"
}

sudo echo '---------install start----------'

if [ -x '/usr/local/bin/astrabot' ]; then
    sudo astrabot stop || true
fi
if [[ -n "$(find /etc/systemd/system -maxdepth 1 \
    \( -type f -o -type l \) -name 'Astrabot_*.service' -print -quit)" ]]; then
    Backup_Astrabot_Units
    Remove_Astrabot_Units
fi

chmod +x execute.sh

if [ -d "$START_UP_DIR" ]; then
    START_UP_BACKUP_DIR="${START_UP_DIR}.backup.$(date +%Y%m%d-%H%M%S)-$$"
    sudo mv -- "$START_UP_DIR" "$START_UP_BACKUP_DIR"
    echo "robot_start backup: $START_UP_BACKUP_DIR"
fi

Mk_Dir "${START_UP_DIR}"

# sudo cp -rf * $START_UP_DIR

sudo chgrp -R "$THE_USER" "$START_UP_DIR"
sudo chown -R "$THE_USER" "$START_UP_DIR"

Mk_Dir "${START_UP_DIR}/${SERVICE_DIR}"
Mk_Dir "${START_UP_DIR}/${LOG_DIR}"
Mk_Dir "${START_UP_DIR}/${RUN_DIR}"
Mk_Dir "${START_UP_DIR}/${CONFIG_DIR}"
Mk_Dir "${START_UP_DIR}/${TEMP_DIR}"
Mk_Dir "${START_UP_DIR}/${FUNCTION_DIR}"
Mk_Dir "${START_UP_DIR}/${AUTO_START_SCRIPT_DIR}"

source generate_service.sh

TAG_DIR="${RUN_SCRIPT_DIR}/${install_mode}"


cp "${Environment_Script}" "${START_UP_DIR}/${RUN_DIR}/environment.sh"
cp logger.sh "${START_UP_DIR}/${RUN_DIR}"
cp execute.sh "${START_UP_DIR}/${RUN_DIR}"
cp wait-for-timesync.sh "${START_UP_DIR}/${RUN_DIR}"
cp wait-controller-node.sh "${START_UP_DIR}/${RUN_DIR}"
cp wait-and-sync-ccu-time.sh "${START_UP_DIR}/${RUN_DIR}"
cp prepare-zed-usb.sh "${START_UP_DIR}/${RUN_DIR}"

cp "${Environment_Script}" "${TIME_SYN_DIR}/environment.sh"
sudo cp -rf config/. "${START_UP_DIR}/${CONFIG_DIR}/"

cp "${Environment_Script}" "${START_UP_DIR}/${FUNCTION_DIR}/environment.sh"
cp logger.sh "${START_UP_DIR}/${FUNCTION_DIR}"
cp generate_service.sh "${START_UP_DIR}/${FUNCTION_DIR}"
cp reload_auto_start_script.sh "${START_UP_DIR}/${FUNCTION_DIR}"
cp base_script.sh "${START_UP_DIR}/${FUNCTION_DIR}"
cp execute.sh "${START_UP_DIR}/${FUNCTION_DIR}"
cp wait-for-timesync.sh "${START_UP_DIR}/${FUNCTION_DIR}"
cp wait-controller-node.sh "${START_UP_DIR}/${FUNCTION_DIR}"
cp wait-and-sync-ccu-time.sh "${START_UP_DIR}/${FUNCTION_DIR}"
cp prepare-zed-usb.sh "${START_UP_DIR}/${FUNCTION_DIR}"
cp install_rtc_teleop_config.sh "${START_UP_DIR}/${FUNCTION_DIR}"
cp fastdds_shm_refresh_gate.sh "${START_UP_DIR}/${FUNCTION_DIR}"

cp -rf "./${RUN_SCRIPT_DIR}/${install_mode}/." "${START_UP_DIR}/${AUTO_START_SCRIPT_DIR}/"


echo "install time syn $install_mode"

(
    cd "$TIME_SYN_DIR"
    ./install_time_syn.sh "$install_mode"
)

sudo timedatectl set-timezone Asia/Shanghai

for file_r in "$TAG_DIR"/*.start_script; do
    [[ -e "$file_r" ]] || continue
    file="$(basename "$file_r")"
    run_order="$(cat "$file_r")"

    # 找到 PRIORITY= 后面的数字；缺失时在严格模式下终止安装。
    priority_line="$(grep -m1 '^PRIORITY=' "$file_r")"
    priority="${priority_line#*=}"
    echo "priority: $priority"

    file_name="$(basename "$file" .start_script)"
    run_file="${START_UP_DIR}/${RUN_DIR}/${file_name}.sh"
    service_file="${START_UP_DIR}/${SERVICE_DIR}/${file_name}.service"

    Generate_Run "$run_order" "$file_name" "$run_file" \
        "${START_UP_DIR}/${RUN_DIR}" "$Environment_Script"
    Generate_Service "${START_UP_DIR}/${RUN_DIR}" "$file_name" \
        "$Environment_Script" "$run_file" "$service_file" "$priority" "$BOARD_TYPE"
done

#如果run_script/${install_mode}/supplement/service文件夹存在，则将文件夹中的文件复制到service文件夹
if [ -d "run_script/${install_mode}/supplement/service" ]; then
    echo "copy supplement service file"
    cp -rf "./run_script/${install_mode}/supplement/service/." "${START_UP_DIR}/${SERVICE_DIR}/"
fi

if [ -d "run_script/${install_mode}/supplement/run" ]; then
    echo "copy supplement run file"
    cp -rf "./run_script/${install_mode}/supplement/run/." "${START_UP_DIR}/${RUN_DIR}/"
fi

if [ -d "run_script/${install_mode}/supplement/config" ]; then
    echo "copy supplement config files"
    cp -rf "./run_script/${install_mode}/supplement/config/." \
        "${START_UP_DIR}/${CONFIG_DIR}/"
    # 真实数采配置由 install_data_collection.sh 以 0660 部署到用户目录，
    # /opt/ros/start_up 中只保留不含密钥的模板。
    rm -f "${START_UP_DIR}/${CONFIG_DIR}/data_collection/astrabot.yaml"
    rm -f "${START_UP_DIR}/${CONFIG_DIR}/livekit/livekit.env"
fi

sudo chgrp -R "$THE_USER" "$START_UP_DIR"
sudo chown -R "$THE_USER" "$START_UP_DIR"

sudo rm -f /usr/local/bin/astrabot
sudo rm -f /usr/local/bin/astrabot_environment
sudo rm -f /usr/local/bin/astrabot_timesync
sudo rm -f /usr/local/bin/wait-controller-node.sh
sudo rm -f /usr/local/bin/wait-and-sync-ccu-time.sh
sudo rm -f /usr/local/bin/prepare-zed-usb.sh

sudo ln -s "${START_UP_DIR}/${RUN_DIR}/execute.sh" /usr/local/bin/astrabot
sudo ln -s "${START_UP_DIR}/${RUN_DIR}/environment.sh" /usr/local/bin/astrabot_environment
sudo ln -s "${START_UP_DIR}/${RUN_DIR}/wait-for-timesync.sh" /usr/local/bin/astrabot_timesync
sudo ln -s "${START_UP_DIR}/${RUN_DIR}/wait-controller-node.sh" /usr/local/bin/wait-controller-node.sh
sudo ln -s "${START_UP_DIR}/${RUN_DIR}/wait-and-sync-ccu-time.sh" /usr/local/bin/wait-and-sync-ccu-time.sh
sudo ln -s "${START_UP_DIR}/${RUN_DIR}/prepare-zed-usb.sh" /usr/local/bin/prepare-zed-usb.sh

Verify_Generated_Units
sudo cp -rf "${START_UP_DIR}/${SERVICE_DIR}/." /etc/systemd/system/
sudo systemctl daemon-reload

if [ "$install_mode" = "thor" ]; then
    echo "install Thor RTC/Teleop safe configuration"
    bash ./install_rtc_teleop_config.sh

    if [[ "${ASTRABOT_SKIP_DATA_COLLECTION_INSTALL:-0}" == "1" ]]; then
        echo "skip optional Thor data collection environment"
    else
        echo "install Thor data collection environment"
        ./install_data_collection.sh
    fi
fi

Ensure_Astrabot_Data_Dirs

sudo astrabot stop
sudo bash "$FAST_DDS_SHM_GATE_SCRIPT" clean
sudo astrabot init
sudo -u "$THE_USER" bash "$FAST_DDS_SHM_GATE_SCRIPT" verify
sudo astrabot enable

if [[ "$install_mode" == "thor" ]]; then
    # 每次 Thor 完整安装都将需要人工配置或受控迁移的服务恢复为停止和禁用。
    # Data Collection 配置完成后显式启动；RTC/Teleop 仅在迁移验证时按单服务启动。
    managed_opt_in_units=(
        Astrabot_Data_Collection.service
        Astrabot_LiveKit.service
        Astrabot_LiveKit_Srv.service
        Astrabot_Rtc.service
        Astrabot_Teleop.service
    )
    sudo systemctl disable --now "${managed_opt_in_units[@]}" || true
fi

trap - ERR
