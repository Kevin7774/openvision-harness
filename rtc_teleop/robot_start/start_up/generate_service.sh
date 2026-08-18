#!/bin/bash
###
 # @Author: Zhixun.Li
 # @Date: 2025-08-13 12:57:07
 # @LastEditors: Zhixun.Li
 # @LastEditTime: 2025-09-02 16:30:44
### 

function Generate_Run() {

    file_r=$1
    file_name=$2
    run_file=$3
    wrok_dir=$4
    environment_file=$5

    source $environment_file    

    echo "#!/bin/bash" >$run_file

    echo "$file_r" >> $run_file

    echo "" >> $run_file
    echo "" >> $run_file
    echo "" >> $run_file

    echo "RUN_FILE_NAME=$file_name" >> $run_file

    echo "cd $wrok_dir" >>$run_file
    
    cat ${START_UP_DIR}/${FUNCTION_DIR}/base_script.sh >> $run_file

    echo "generate: $file_name"

    chmod +x $run_file
}

function Generate_Service() {
    
    wrok_dir=$1
    description=$2
    environment_file=$3
    exec_start=$4
    service_file=$5
    priority=$6
    board_type=$7
    service_name="$(basename "$service_file")"
    
    source $environment_file    

    echo "" > $service_file

    echo "[Unit]" >> $service_file
    echo "Description=${description}"  >> $service_file
    # echo "After=init.service"  >> $service_file
    if [ "${board_type}" == "thor" ]; then
       echo "After=multi-user.target network-online.target time-sync.target" >> $service_file
       echo "Wants=time-sync.target" >> $service_file
       echo "Requires=multi-user.target" >> $service_file
    else
       echo "After=wait-ccu-time.service network-online.target" >> $service_file
       echo "Requires=wait-ccu-time.service" >> $service_file
    fi

    echo ""  >> $service_file
    echo "[Service]"  >> $service_file
    echo "Nice=${priority}"  >> $service_file
    echo "User=${THE_USER}"  >> $service_file
    echo "EnvironmentFile="  >> $service_file
    echo "WorkingDirectory=${wrok_dir}"  >> $service_file
    if [[ "$service_name" == "Astrabot_Controller.service" ]]; then
        echo "LimitRTPRIO=99" >> $service_file
        echo "LimitMEMLOCK=infinity" >> $service_file
    fi
    if [[ "$service_name" == "Astrabot_ZED.service" ]]; then
        echo "ExecStartPre=+/usr/local/bin/prepare-zed-usb.sh" >> $service_file
    fi
    if [[ "$service_name" == "Astrabot_Mpc.service" ]] || [[ "$service_name" == "Astrabot_Backend.service" ]]; then
	 echo "ExecStartPre=/usr/local/bin/wait-controller-node.sh" >> $service_file
    fi
    if [ "${board_type}" == "thor" ]; then
         echo "ExecStartPre=/bin/bash -c '/usr/local/bin/astrabot_timesync || { echo "[WARN] timesync failed, fallback sleep 30s"; sleep 30; }'" >> $service_file
    fi
    echo "ExecStart=${exec_start}"  >> $service_file
    echo "Restart=always"  >> $service_file
    if [[ "$service_name" == "Astrabot_ZED.service" ]]; then
        echo "KillSignal=SIGINT" >> $service_file
        echo "KillMode=control-group" >> $service_file
        echo "TimeoutStopSec=20" >> $service_file
    else
        echo "KillSignal=9"  >> $service_file
    fi

    echo ""  >> $service_file

    echo "[Install]"  >> $service_file
    echo "WantedBy=multi-user.target"  >> $service_file

}
