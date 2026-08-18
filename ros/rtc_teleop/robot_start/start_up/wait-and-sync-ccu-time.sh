#!/bin/bash

# THOR 的 IP（你现在用的是 192.168.123.102）

SRC_IP="192.168.123.102"
MAX_WAIT=100
INTERVAL=5
WAITED=0
source astrabot_environment

log() {
    echo "[wait_ccu_time] $(date '+%Y-%m-%d %H:%M:%S') $1"
}

get_ccu_tracking() {
    sshpass -p "$THE_PASSWORD" ssh -o ConnectTimeout=5 ${THE_USER}@${SRC_IP} "chronyc tracking" 2>/dev/null
}

get_ccu_datetime() {
    sshpass -p "$THE_PASSWORD" ssh -o ConnectTimeout=5 ${THE_USER}@${SRC_IP} "date '+%Y-%m-%d %H:%M:%S'" 2>/dev/null
}

log "start waiting ccu time sync (max ${MAX_WAIT}s)..."

while true; do
    ccu_tracking=$(get_ccu_tracking)

    if [ -z "$ccu_tracking" ]; then
        echo "cannot get chrony status from ccu, retry..."
    else
	ref=$(echo "$ccu_tracking" | grep "Reference ID.*203.107.6.88")
        leap=$(echo "$ccu_tracking" | grep "Leap status.*Normal")
        if [ -n "$ref" ] && [ -n "$leap" ]; then
           echo "CCU chrony is synced, trigger local makestep"
           ccu_time=$(get_ccu_datetime)
           if [ -n "$ccu_time" ]; then
              echo "set local time to ccu time: $ccu_time"
              echo $THE_PASSWORD | sudo -S date -s $ccu_time
              #echo $THE_PASSWORD | sudo -S chronyc -a makestep
	      exit 0
           fi
        else
            echo "ccu not synced yet: $ref | $leap"
        fi
    fi

    sleep "$INTERVAL"
    WAITED=$((WAITED + INTERVAL))

    if [ "$WAITED" -ge "$MAX_WAIT" ]; then
        echo "timeout reached, force align to ccu system time"

        ccu_time=$(get_ccu_datetime)
        if [ -n "$ccu_time" ]; then
            echo "set local time to ccu time: $ccu_time"
            echo $THE_PASSWORD | sudo -S date -s $ccu_time
        else
            echo "failed to get ccu time, keep current time"
        fi
        exit 0
    fi
done
