#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source astrabot_environment

if [ -f "/home/${THE_USER}/.bashrc" ]; then
    ROS_DOMAIN_ID_FROM_BASHRC=$(grep -E '^export ROS_DOMAIN_ID=' "/home/${THE_USER}/.bashrc" | tail -n1 | cut -d= -f2)
fi

ROS_DOMAIN_ID=${ROS_DOMAIN_ID_FROM_BASHRC:-12}
export CONTROLLER="astrabot_arm_forward_position_controller"

echo "[INFO] Using ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "⏳ 等待 $CONTROLLER active 中..."

for i in {1..60}; do
    if ros2 control list_controllers | grep -q "$CONTROLLER.*active"; then
        echo "✅ $CONTROLLER 激活完成"
        exit 0
    fi
    sleep 1
done

echo "❌ ERROR: $CONTROLLER 未在超时内启动"
exit 1
