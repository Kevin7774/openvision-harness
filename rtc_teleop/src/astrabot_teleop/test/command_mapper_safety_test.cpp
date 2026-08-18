// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include "astrabot_teleop/mapping/command_mapper.h"

namespace astrabot::teleop {
namespace {

DecodedTeleopFrame makeActiveFrame() {
    DecodedTeleopFrame frame;
    frame.pose_valid_right = true;
    frame.pose_valid_left = true;
    frame.pose_valid_head = true;
    frame.action_right.present = true;
    frame.action_left.present = true;
    frame.action_head.present = true;
    frame.axes.present = true;
    frame.axes.grip_right = 1.0;
    frame.axes.grip_left = 1.0;
    return frame;
}

TEST(CommandMapperSafetyTest, LegacyAndShadowModeKeepFrozenV1Behavior) {
    CommandMapper mapper(CommandMappingConfig{});
    const MappedCommand command = mapper.map(makeActiveFrame());

    EXPECT_TRUE(command.right_arm_valid);
    EXPECT_TRUE(command.left_arm_valid);
    EXPECT_TRUE(command.right_gripper_valid);
    EXPECT_TRUE(command.left_gripper_valid);
    EXPECT_TRUE(command.head_valid);
    EXPECT_TRUE(command.right_deadman);
    EXPECT_TRUE(command.left_deadman);
    EXPECT_TRUE(command.deadman);
}

TEST(CommandMapperSafetyTest, ProductionModeFailsClosedWhenSafetyFlagsAreMissing) {
    CommandMappingConfig config;
    config.require_safety_flags = true;
    CommandMapper mapper(config);
    const MappedCommand command = mapper.map(makeActiveFrame());

    EXPECT_FALSE(command.right_arm_valid);
    EXPECT_FALSE(command.left_arm_valid);
    EXPECT_FALSE(command.right_gripper_valid);
    EXPECT_FALSE(command.left_gripper_valid);
    EXPECT_FALSE(command.head_valid);
    EXPECT_FALSE(command.right_deadman);
    EXPECT_FALSE(command.left_deadman);
    EXPECT_FALSE(command.deadman);
}

TEST(CommandMapperSafetyTest, ProductionModeGatesEachTrackedPartIndependently) {
    CommandMappingConfig config;
    config.require_safety_flags = true;
    CommandMapper mapper(config);
    DecodedTeleopFrame frame = makeActiveFrame();
    frame.safety_present = true;
    frame.safety_right = false;
    frame.safety_left = true;
    frame.safety_head = false;

    const MappedCommand command = mapper.map(frame);

    EXPECT_FALSE(command.right_arm_valid);
    EXPECT_TRUE(command.left_arm_valid);
    EXPECT_FALSE(command.right_gripper_valid);
    EXPECT_TRUE(command.left_gripper_valid);
    EXPECT_FALSE(command.head_valid);
    EXPECT_FALSE(command.right_deadman);
    EXPECT_TRUE(command.left_deadman);
    EXPECT_TRUE(command.deadman);
}

TEST(CommandMapperSafetyTest, UnsafeSideCannotMutateGripperLatch) {
    CommandMappingConfig config;
    config.require_safety_flags = true;
    CommandMapper mapper(config);
    DecodedTeleopFrame frame = makeActiveFrame();
    frame.safety_present = true;
    frame.safety_right = false;
    frame.safety_left = true;
    frame.axes.trigger_right = 0.95;

    EXPECT_FALSE(mapper.map(frame).right_gripper_valid);
    frame.axes.trigger_right = 0.0;
    EXPECT_FALSE(mapper.map(frame).right_gripper_valid);

    frame.safety_right = true;
    const MappedCommand recovered = mapper.map(frame);
    EXPECT_TRUE(recovered.right_gripper_valid);
    EXPECT_DOUBLE_EQ(recovered.right_gripper, 0.0);
}

TEST(CommandMapperSafetyTest, ReleasedDeadmanCannotMutateGripperLatch) {
    CommandMappingConfig config;
    config.require_safety_flags = true;
    CommandMapper mapper(config);
    DecodedTeleopFrame frame = makeActiveFrame();
    frame.safety_present = true;
    frame.safety_right = true;
    frame.safety_left = true;
    frame.axes.grip_right = 0.0;
    frame.axes.trigger_right = 0.95;

    EXPECT_FALSE(mapper.map(frame).right_gripper_valid);
    frame.axes.trigger_right = 0.0;
    EXPECT_FALSE(mapper.map(frame).right_gripper_valid);

    frame.axes.grip_right = 1.0;
    const MappedCommand armed = mapper.map(frame);
    EXPECT_TRUE(armed.right_gripper_valid);
    EXPECT_DOUBLE_EQ(armed.right_gripper, 0.0);
}

}  // namespace
}  // namespace astrabot::teleop
