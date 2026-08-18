// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "astrabot_teleop/adapter/legacy_mpc_command_encoder.h"

namespace astrabot::teleop {
namespace {

PoseSample pose(const double x) {
    PoseSample value;
    value.present = true;
    value.x = x;
    value.y = 0.2;
    value.z = 0.3;
    value.qw = 1.0;
    return value;
}

TEST(LegacyMpcCommandEncoderTest, EncodesOnlyValidArmsAndControlName) {
    MappedCommand command;
    command.deadman = true;
    command.right_arm_valid = true;
    command.right_arm_target = pose(0.1);

    LegacyMpcCommandEncoder encoder;
    auto result = encoder.encodeCommand(command, "wbmpc_remote_ctrl");

    ASSERT_TRUE(result.ok()) << result.status().message();
    const auto payload = nlohmann::json::parse(result.value(), nullptr, false);
    ASSERT_FALSE(payload.is_discarded());
    EXPECT_EQ(payload["ctrl_name"], "wbmpc_remote_ctrl");
    EXPECT_TRUE(payload.contains("right_arm"));
    EXPECT_FALSE(payload.contains("left_arm"));
    EXPECT_DOUBLE_EQ(payload["right_arm"]["position"][0].get<double>(), 0.1);
}

TEST(LegacyMpcCommandEncoderTest, HoldContainsBothCurrentArmPoses) {
    LegacyMpcCommandEncoder encoder;
    auto result = encoder.encodeHold(pose(0.4), pose(-0.4), "wbmpc_remote_ctrl");

    ASSERT_TRUE(result.ok()) << result.status().message();
    const auto payload = nlohmann::json::parse(result.value(), nullptr, false);
    ASSERT_FALSE(payload.is_discarded());
    EXPECT_DOUBLE_EQ(payload["right_arm"]["position"][0].get<double>(), 0.4);
    EXPECT_DOUBLE_EQ(payload["left_arm"]["position"][0].get<double>(), -0.4);
}

TEST(LegacyMpcCommandEncoderTest, RejectsInactiveOrInvalidInput) {
    LegacyMpcCommandEncoder encoder;
    MappedCommand command;
    command.right_arm_valid = true;
    command.right_arm_target = pose(0.1);
    EXPECT_FALSE(encoder.encodeCommand(command, "wbmpc_remote_ctrl").ok());

    command.deadman = true;
    command.right_arm_target.qw = 0.0;
    EXPECT_FALSE(encoder.encodeCommand(command, "wbmpc_remote_ctrl").ok());
    EXPECT_FALSE(encoder.encodeHold(pose(0.1), pose(0.2), "").ok());
}

}  // namespace
}  // namespace astrabot::teleop
