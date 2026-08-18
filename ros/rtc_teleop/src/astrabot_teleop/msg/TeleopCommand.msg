# 经过端侧协议校验、session 隔离和 Teleop 安全边界后的类型化命令。
# valid_until_ns 使用机器人本机 steady clock；不得与 wall clock 比较。
string session_id
string run_id
string resource_id
string source_peer_id
string channel_label
# shadow backend 固定为 0；生产 cpp backend 必须填写 arbitration 返回的真实 owner epoch。
uint64 owner_epoch
uint64 sequence
uint64 source_timestamp_ns
uint64 receive_steady_time_ns
uint64 valid_until_ns
bool deadman
# 每个 validity 只授权对应字段；consumer 不得把数值存在视为有效。
bool right_arm_valid
bool left_arm_valid
bool right_gripper_valid
bool left_gripper_valid
bool head_valid
geometry_msgs/Pose right_arm_target
geometry_msgs/Pose left_arm_target
geometry_msgs/Pose head_target
float32 right_gripper
float32 left_gripper
geometry_msgs/Twist chassis_command
bool shadow_only
