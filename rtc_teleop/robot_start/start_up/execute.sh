#!/bin/bash

echo "-------------"

command=$1

source astrabot_environment
TAG_DIR="${START_UP_DIR}/${SERVICE_DIR}"

Resolve_Service_Name() {
    local requested="$1"

    if [[ "$requested" == */* ]]; then
        echo "invalid service name: $requested" >&2
        return 1
    fi
    [[ "$requested" == *.service ]] || requested="${requested}.service"
    if [[ ! -f "${TAG_DIR}/${requested}" ]]; then
        echo "service not found: $requested" >&2
        echo "use 'astrabot list' to show available services" >&2
        return 1
    fi
    printf '%s\n' "$requested"
}

Check_Service_Dependencies() {
    local service="$1"
    local relation dependency state

    echo "dependencies of ${service}:"
    for relation in Requires Wants; do
        for dependency in $(systemctl show "$service" --property="$relation" --value); do
            [[ -n "$dependency" ]] || continue
            state=$(systemctl is-active "$dependency" 2>/dev/null || true)
            echo "  ${relation}: ${dependency} (${state:-unknown})"
        done
    done
    echo "systemd will start declared dependencies automatically."
}

Check_Active_Dependents() {
    local service="$1"
    local dependent state

    echo "active units depending on ${service}:"
    while read -r dependent; do
        dependent="${dependent##* }"
        [[ "$dependent" == *.service ]] || continue
        [[ "$dependent" == "$service" ]] && continue
        state=$(systemctl is-active "$dependent" 2>/dev/null || true)
        [[ "$state" == "active" ]] && echo "  ${dependent}"
    done < <(systemctl list-dependencies --reverse --plain --no-legend "$service" 2>/dev/null)
}

Is_Rtc_Teleop_Migration_Service() {
    local service="$1"

    [[ "$service" == "Astrabot_Rtc.service" || \
       "$service" == "Astrabot_Teleop.service" ]]
}

if [[ "$command" == "start" || "$command" == "stop" ]] && [[ -n "${2:-}" ]]; then
    if [[ -n "${3:-}" ]]; then
        echo "usage: astrabot ${command} [service_name]" >&2
        exit 2
    fi

    service=$(Resolve_Service_Name "$2") || exit 1
    echo "${command} ${service}"
    if [[ "$command" == "start" ]]; then
        Check_Service_Dependencies "$service"
    else
        Check_Active_Dependents "$service"
    fi
    if [[ "$command" == "start" ]] && \
       [[ "$service" == "Astrabot_Data_Collection.service" ]]; then
        echo "enable data collection autostart after hardware setup"
        echo "$THE_PASSWORD" | sudo -S systemctl enable --now "$service"
    else
        echo "$THE_PASSWORD" | sudo -S systemctl "$command" "$service"
    fi
    result=$?
    echo "-------------"
    exit $result
fi

if [ $command == "start" ]; then
    echo "start all of the ros node."
elif [ $command == "init" ]; then
    echo "init all of the ros node"
    if [ -f "/etc/systemd/system/wait-ccu-time.service" ]; then
       echo $THE_PASSWORD | sudo -S systemctl start wait-ccu-time.service
    fi
    sleep 3
elif [ $command == "stop" ]; then
    echo "stop all of the ros node."
elif [ $command == "enable" ]; then
    echo "enable all of the ros node."

elif [ $command == "disable" ]; then
    echo "disable all of the ros node."

elif [ $command == "log" ]; then
    echo "show the log."
    command_2=$2
    log_name=$(find $START_UP_DIR/$LOG_DIR/$command_2 -name "*.log" -mtime -1 -exec stat -c "%Y %n" {} \; | sort -nr | head -1 | awk '{print $2}')
    echo $log_name
    cat $log_name

    echo "-------------"

    exit 0
elif [ $command == "list" ]; then
    echo "list all of the ros node."
    ls $TAG_DIR | while read file; do

        file_r="${TAG_DIR}/${file}"

        if [[ -f "$file_r" &&  "$file" == *.service ]]; then

            echo "${file}"
        fi
    done

    echo "-------------"

    exit 0
elif [ $command == "reload" ]; then
    echo "reload all of the ros node."
    relaod_auto_start_script_file="${START_UP_DIR}/${FUNCTION_DIR}/reload_auto_start_script.sh"
    source $relaod_auto_start_script_file
    echo "-------------"

    exit 0
elif [ $command == "domain_id" ]; then
    echo "show the domain id of the ros node ."
    
    FILE="/home/${THE_USER}/.bashrc"

    if grep -q "^export ROS_DOMAIN_ID=" "$FILE"; then
        # 已存在 → 替换
        sed -i "s/^export ROS_DOMAIN_ID=.*/export ROS_DOMAIN_ID=$2/" "$FILE"
    else
        # 不存在 → 追加
        echo "export ROS_DOMAIN_ID=$2" >> "$FILE"
    fi
    
    exit 0
elif [ $command == "dds" ]; then
    echo "set the dds implementation of the ros to $2."

    #修改ros_config.sh中的implementation
    sed -i "s/export RMW_IMPLEMENTATION=.*/export RMW_IMPLEMENTATION=$2/" $START_UP_DIR/$CONFIG_DIR/ros_config.sh

    echo "-------------"

    exit 0
else
    echo "unknown command: $command"
    echo "-------------"
    exit 1
fi


ls $TAG_DIR | while read file; do

    file_r="${TAG_DIR}/${file}"

    if [[ -f "$file_r" &&  "$file" == *.service ]]; then

        echo "${file}"
        # 数采首次部署后还需要写入硬件信息，不能随整机安装自动启动或自启。
        # 配置完成后由用户显式执行：
        #   astrabot start Astrabot_Data_Collection
        if Is_Rtc_Teleop_Migration_Service "$file" && \
           [[ "$command" == "start" || "$command" == "init" || \
              "$command" == "enable" ]]; then
            echo "skip ${file}: migration service requires explicit single-service startup"
            continue
        fi
        if [[ "$file" == "Astrabot_Data_Collection.service" || \
              "$file" == "Astrabot_LiveKit.service" || \
              "$file" == "Astrabot_LiveKit_Srv.service" ]] && \
           [[ "$command" == "init" || "$command" == "enable" ]]; then
            echo "skip ${file}: managed by Data Collection startup"
            continue
        fi
        if [ $command == "init" ]; then
           echo $THE_PASSWORD | sudo -S systemctl start $file
        else
           echo $THE_PASSWORD | sudo -S systemctl $command $file
        fi

    fi

done

echo "-------------"
