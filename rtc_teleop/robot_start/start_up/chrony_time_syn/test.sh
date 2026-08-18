INTERVAL=2  # 检测间隔(秒)
THRESHOLD=0.1 # 告警阈值(秒)

while true; do
    OFFSET=$(chronyc tracking | grep "Last offset" | awk '{print $4}')
    # TIMESTAMP=$(date "+%Y-%m-%d %H:%M:%S")
    
    echo "Current offset: $OFFSET seconds"
    
    # 浮点数比较需要bc处理
    # if (( $(echo "$OFFSET > $THRESHOLD || $OFFSET < -$THRESHOLD" | bc -l) )); then
    #     echo "ALERT: Offset exceeds threshold!" >&2
    # fi
    
    sleep $INTERVAL
done