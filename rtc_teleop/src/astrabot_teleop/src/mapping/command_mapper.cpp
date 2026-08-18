// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include "astrabot_teleop/mapping/command_mapper.h"

#include <cmath>

namespace astrabot::teleop {
namespace {

PoseSample normalizedPose(PoseSample pose) {
    if (!pose.present) {
        return pose;
    }
    const double norm = std::sqrt(pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz + pose.qw * pose.qw);
    pose.qx /= norm;
    pose.qy /= norm;
    pose.qz /= norm;
    pose.qw /= norm;
    return pose;
}

}  // namespace

CommandMapper::CommandMapper(CommandMappingConfig config) : config_(config) {}

MappedCommand CommandMapper::map(const DecodedTeleopFrame &frame) {
    MappedCommand command;
    const bool right_safe = !config_.require_safety_flags || (frame.safety_present && frame.safety_right);
    const bool left_safe = !config_.require_safety_flags || (frame.safety_present && frame.safety_left);
    const bool head_safe = !config_.require_safety_flags || (frame.safety_present && frame.safety_head);
    command.right_arm_valid = frame.pose_valid_right && frame.action_right.present && right_safe;
    command.left_arm_valid = frame.pose_valid_left && frame.action_left.present && left_safe;
    command.head_valid = frame.pose_valid_head && frame.action_head.present && head_safe;
    command.right_arm_target = normalizedPose(frame.action_right);
    command.left_arm_target = normalizedPose(frame.action_left);
    command.head_target = normalizedPose(frame.action_head);

    if (frame.axes.present) {
        command.right_deadman = command.right_arm_valid && frame.axes.grip_right >= config_.deadman_grip_threshold;
        command.left_deadman = command.left_arm_valid && frame.axes.grip_left >= config_.deadman_grip_threshold;
        command.deadman = command.right_deadman || command.left_deadman;
        command.right_gripper_valid = !config_.require_safety_flags || command.right_deadman;
        command.left_gripper_valid = !config_.require_safety_flags || command.left_deadman;
        // 生产链路中 tracking/re-grab/deadman 任一无效时都不得推进锁存，避免重新 Armed 后执行旧 toggle。
        if (command.right_gripper_valid) {
            command.right_gripper = mapGripper(frame.axes.trigger_right, right_gripper_);
        }
        if (command.left_gripper_valid) {
            command.left_gripper = mapGripper(frame.axes.trigger_left, left_gripper_);
        }
        if (command.left_deadman) {
            command.chassis_linear_x = frame.axes.joystick_left_y * config_.max_chassis_linear_mps;
            command.chassis_angular_z = -frame.axes.joystick_left_x * config_.max_chassis_angular_rps;
        }
    }
    return command;
}

void CommandMapper::reset() {
    right_gripper_ = {};
    left_gripper_ = {};
}

double CommandMapper::mapGripper(const double input, GripperLatchState &state) const {
    if (!state.hold) {
        if (input > config_.gripper_toggle_high_threshold) {
            state.trigger = true;
        } else if (state.trigger && input < config_.gripper_toggle_low_threshold) {
            state.hold = true;
            state.trigger = false;
        }
    } else if (input > config_.gripper_toggle_high_threshold) {
        state.trigger = true;
    } else if (state.trigger && input < config_.gripper_toggle_low_threshold) {
        state.hold = false;
        state.trigger = false;
    }
    return state.hold || input > config_.gripper_binary_threshold ? 1.0 : 0.0;
}

}  // namespace astrabot::teleop
