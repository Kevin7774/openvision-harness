#!/bin/bash

source "environment.sh"

LOG_FILE=""

function init_log() {

    file_name=$1

    CURRENT_DAY=$(date +%m%d)
    CURRENT_DATE=$(date +%H.%M.%S)

    log_root_dir="${START_UP_DIR}/${LOG_DIR}/${file_name}"

    current_log_dir="${log_root_dir}/${CURRENT_DAY}/${CURRENT_DATE}"

    if [ ! -f "${current_log_dir}" ]; then
	    mkdir -p $current_log_dir

        chgrp -R $THE_USER ${current_log_dir}
        chown -R $THE_USER ${current_log_dir}
    fi

    NUMBER_OFFSET=$LOG_DELETE_OFFSET	# delete some days ago log file

    for file in `find "$log_root_dir" -type f,d -mtime "$NUMBER_OFFSET" -print`
    do
        echo "deleting $file"
        rm -rf $file
    done

    LOG_FILE="${current_log_dir}/${file_name}.log"

    # ================= NEW: 创建软连接 =================
    LINK_DIR="${HOME}/start_up_log"
    LINK_FILE="${LINK_DIR}/${file_name}.log"

    mkdir -p "$LINK_DIR"

    # 如果之前已经有（上一次启动），先删除
    if [ -L "$LINK_FILE" ] || [ -f "$LINK_FILE" ]; then
        rm -f "$LINK_FILE"
    fi

    ln -s "$LOG_FILE" "$LINK_FILE"

    echo "[logger] log file     : $LOG_FILE"
    echo "[logger] log symlink  : $LINK_FILE -> $LOG_FILE"
    # ==================================================

}
