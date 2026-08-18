#!/bin/bash

LOG_FILE="/opt/ros/start_up/log/wait-for-timesync.log"

# ================= 配置区 =================
WAIT_TIME=100                # 最多等待对时 100 秒
CHECK_INTERVAL=5             # 每 5 秒检查一次
MAX_OFFSET_MS=100            # 允许的最大误差
FALLBACK_TIME="2026-01-01 00:00:00"
FALLBACK_DATE_NUM=20260101
# =========================================
source astrabot_environment

log() {
    echo "[$(date '+%F %T')] $*" | tee -a "$LOG_FILE"
}

get_date_num() {
    date +%Y%m%d
}

# =========================================================
# 阶段1：如果是 1970 / 异常时间 → 立刻设置兜底时间
# =========================================================
current_date=$(get_date_num)

if [ "$current_date" -lt "$FALLBACK_DATE_NUM" ]; then
    log "System time invalid ($current_date), set fallback time: $FALLBACK_TIME"
    echo $THE_PASSWORD | sudo -S date -s "$FALLBACK_TIME"
    #echo $THE_PASSWORD | sudo -S hwclock -w
else
    log "System time already reasonable: $(date '+%F %T')"
fi

# =========================================================
# 阶段2：启动 chrony，对时（最多等 100 秒）
# =========================================================
log "Start chrony sync (timeout=${WAIT_TIME}s, max_offset=${MAX_OFFSET_MS}ms)"

start_time=$(date +%s)

while true; do
    ref_id=$(chronyc tracking | grep "Reference ID" | awk '{print $5}')
    leap_status=$(chronyc tracking | grep "Leap status" | awk '{print $4}')
    log "== $ref_id == $leap_status == $start_time =="

    # 判断 Reference ID 是否为公网源的 IP（可以根据实际源 IP 修改）
    if [[ "$ref_id" =~ "203.107.6.88" && "$leap_status" == "Normal" ]]; then
       log "Chrony synced successfully. Final time: $(date '+%F %T')"
       exit 0
    fi
    # 如果没有同步，继续等待
    log "Waiting for synchronization..."
    now=$(date +%s)
    elapsed=$((now - start_time))

    if [ "$elapsed" -ge "$WAIT_TIME" ]; then
        log "Chrony sync timeout after ${WAIT_TIME}s, keep fallback time, stop chrony to prevent time jump"
        echo $THE_PASSWORD | sudo -S systemctl stop chrony
        exit 1
    fi
    sleep "$CHECK_INTERVAL"
done
