// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#pragma once

#include "astrabot_teleop/protocol/teleop_frame_model.h"

namespace astrabot::teleop {

/** @brief 与 ROS message 解耦的已映射影子控制值。 */
struct MappedCommand {
    bool deadman{false};
    bool right_deadman{false};
    bool left_deadman{false};
    bool right_origin_captured{false};
    bool left_origin_captured{false};
    bool right_arm_valid{false};
    bool left_arm_valid{false};
    bool right_gripper_valid{false};
    bool left_gripper_valid{false};
    bool head_valid{false};
    PoseSample right_arm_target;
    PoseSample left_arm_target;
    PoseSample head_target;
    double right_gripper{0.0};
    double left_gripper{0.0};
    double chassis_linear_x{0.0};
    double chassis_angular_z{0.0};
};

/** @brief Quest tracking-space 输入到影子动作语义的映射配置。 */
struct CommandMappingConfig {
    double deadman_grip_threshold{0.5};
    double max_chassis_linear_mps{0.3};
    double max_chassis_angular_rps{0.5};
    double gripper_toggle_high_threshold{0.9};
    double gripper_toggle_low_threshold{0.1};
    double gripper_binary_threshold{0.4};
    bool require_safety_flags{false};
};

/**
 * @brief 负责 deadman、夹爪和底盘的纯函数式映射。
 *
 * 本类只完成输入语义映射；手臂相对位姿随后必须交给 ArmOriginMapper 转换到机器人 base frame。
 */
class CommandMapper {
  public:
    explicit CommandMapper(CommandMappingConfig config);

    /** @brief 将已校验 TeleopFrame 映射为 command，并更新夹爪锁存状态。 */
    MappedCommand map(const DecodedTeleopFrame &frame);

    /** @brief 新 session 或停止时清空左右夹爪锁存。 */
    void reset();

  private:
    struct GripperLatchState {
        bool hold{false};
        bool trigger{false};
    };

    double mapGripper(double input, GripperLatchState &state) const;

    CommandMappingConfig config_;
    GripperLatchState right_gripper_;
    GripperLatchState left_gripper_;
};

}  // namespace astrabot::teleop
