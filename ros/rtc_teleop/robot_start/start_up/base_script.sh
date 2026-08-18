

source environment.sh
source logger.sh

init_log $RUN_FILE_NAME

source ../config/ros_config.sh
source ../config/time_syn.sh  

#timeout $WAIT_SYN_TIME chronyc waitsync -t $MAX_TIME_SYN_OFFSET &>> $LOG_FILE
#chronyc sources -v &>> $LOG_FILE

echo "START_LIST_FILE: $START_LIST_FILE"

#循环读取COMMUNICATE_PIPE_PATH，并等待其中输出包含MODEL_TYPE的行
while true; do

    sleep 2

    if [ "$MODEL_TYPE" == "First" ]; then
        echo "First"
        break
    fi

    if [ "$MODEL_TYPE" == "Init" ]; then
        echo "Init"
        break
    fi

    #if START_LIST_FILE is not exist, then continue
    if [ ! -f "$START_LIST_FILE" ]; then
        echo "$START_LIST_FILE not exist, continue"
        continue
    fi

    result=$(cat $START_LIST_FILE)
    echo "$result"

    if [[ "$result" == *"$MODEL_TYPE"* ]]; then
        echo "MODEL_TYPE found in: $result"
        break
    fi
    # fi

done


if [ "$RUN_IN_DOCKER" == "true" ]; then
    echo "run in docker"

    #创建一个.sh文件，并写入docker_command
    mkdir docker
    DOCKER_COMMAND_FILE="docker/${RUN_FILE_NAME}_Docker_Command.sh"
    touch $DOCKER_COMMAND_FILE
    chmod +x $DOCKER_COMMAND_FILE

    echo "#!/bin/bash" > $DOCKER_COMMAND_FILE
    echo "cd ${START_UP_DIR}/${RUN_DIR}" > $DOCKER_COMMAND_FILE
    echo "source ../config/ros_config.sh" >> $DOCKER_COMMAND_FILE
    echo "source ../config/time_syn.sh" >> $DOCKER_COMMAND_FILE

    for command in "${COMMAND_RUN_ONCE_IN_BEGIN[@]}"; do
    # 注意这里写入 DOCKER_COMMAND_FILE 是脚本内容，不是真执行
        echo "$command &>> $LOG_FILE" >> $DOCKER_COMMAND_FILE
    done

    if [ -n "${COMMAND_RUN_IN_LOOP}" ]; then
        echo "while true; do" >> $DOCKER_COMMAND_FILE
        for command in "${COMMAND_RUN_IN_LOOP[@]}"; do
            echo "    $command &>> $LOG_FILE " >> $DOCKER_COMMAND_FILE
        done
        echo "done" >> $DOCKER_COMMAND_FILE
    fi

    if [ -n "$(docker ps -q -f name=$DOCKER_CONTAINER_NAME)" ]; then
        echo "容器 $DOCKER_CONTAINER_NAME 正在运行" &>> $LOG_FILE
        sleep 1
    else
        echo "容器 $DOCKER_CONTAINER_NAME 未运行" &>> $LOG_FILE
        sleep 5
        exit -1
    fi

    # 执行 DOCKER_COMMAND_FILE 并日志可视化
    docker exec -u root -d $DOCKER_CONTAINER_NAME \
    nice -n ${PRIORITY} bash ${START_UP_DIR}/${RUN_DIR}/${DOCKER_COMMAND_FILE} &>> $LOG_FILE

    while true; do
        if [ -n "$(docker ps -q -f name=$DOCKER_CONTAINER_NAME)" ]; then
            echo "容器 $DOCKER_CONTAINER_NAME 正在运行" &>> $LOG_FILE
            sleep 10
        else
            echo "容器 $DOCKER_CONTAINER_NAME 未运行" &>> $LOG_FILE
            sleep 3
            exit -1
        fi
     done

else
    echo "run in host" &>> $LOG_FILE
    if [ -n "${COMMAND_RUN_ONCE_IN_BEGIN}" ]; then
        for command in "${COMMAND_RUN_ONCE_IN_BEGIN[@]}"; do
            echo "execute command: $command" &>> $LOG_FILE
            eval "$command" &>> $LOG_FILE
        done
    fi

    if [ -n "${COMMAND_RUN_IN_LOOP}" ]; then
        while true; do
            for command in "${COMMAND_RUN_IN_LOOP[@]}"; do
                echo "execute command: $command" &>> $LOG_FILE
                sleep 1
                eval "$command" &>> $LOG_FILE
            done
        done
    fi
fi

sleep 3

# 查看ros2系统中是否有slam_control服务
have_slam_control=false
have_start_slam=false
result=null

