#!/bin/bash

THE_USER=astrabot
THE_PASSWORD=1

START_UP_DIR='/opt/ros/start_up'
ASTRABOT_EXECUTE_COMMAND_PATH='/usr/local/bin/astrabot'

Environment_File=''

CONFIG_DIR="config"
TEMP_DIR="temp"

SERVICE_DIR="service"
LOG_DIR="log"
RUN_DIR="run"
AUTO_START_SCRIPT_DIR="auto_start_script"
FUNCTION_DIR="function"

RUN_SCRIPT_DIR=run_script
TIME_SYN_DIR=chrony_time_syn

START_LIST_FILE="/tmp/astrabot_start_list"

LOG_DELETE_OFFSET=+7

BOARD_TYPE="ccu"

if [ -f "/home/${THE_USER}/.bashrc" ]; then
    ROS_DOMAIN_ID_FROM_BASHRC=$(grep -E '^export ROS_DOMAIN_ID=' "/home/${THE_USER}/.bashrc" | tail -n1 | cut -d= -f2)
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_FROM_BASHRC:-12}
