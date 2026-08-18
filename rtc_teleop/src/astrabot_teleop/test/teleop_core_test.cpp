// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "astrabot_teleop/config/teleop_config.h"
#include "astrabot_teleop/grant/grant_expiry.h"
#include "astrabot_teleop/mapping/arm_origin_mapper.h"
#include "astrabot_teleop/mapping/command_mapper.h"
#include "astrabot_teleop/protocol/data_channel_contract.h"
#include "astrabot_teleop/runtime/latest_mailbox.h"
#include "astrabot_teleop/safety/command_limiter.h"
#include "astrabot_teleop/safety/motion_watchdog.h"
#include "astrabot_teleop/session/session_registry.h"
#include "astrabot_teleop/session/teleop_state_machine.h"

namespace astrabot::teleop {
namespace {

TEST(DataChannelContractTest, FreezesSingleContract) {
    auto contract = DataChannelContracts::find(DataChannelContracts::kLabel);
    ASSERT_TRUE(contract.ok());
    EXPECT_FALSE(contract.value().ordered);
    EXPECT_EQ(contract.value().max_packet_lifetime_ms, 20U);
    EXPECT_EQ(contract.value().max_payload_bytes, 16384U);
    EXPECT_FALSE(DataChannelContracts::find(std::string(DataChannelContracts::kLabel) + ".versioned").ok());
    EXPECT_FALSE(DataChannelContracts::find("data").ok());
}

TEST(GrantExpiryTest, ConvertsEpochExpiryToStableSteadyClockDeadline) {
    constexpr std::uint64_t kNanosecondsPerSecond = 1000000000U;
    const auto deadline =
        grantExpiryToSteadyDeadline(1060U, 1000U * kNanosecondsPerSecond + 250000000U, 500U * kNanosecondsPerSecond);
    ASSERT_TRUE(deadline.ok());
    EXPECT_EQ(deadline.value(), 559U * kNanosecondsPerSecond + 750000000U);

    // 转换完成后只保留 steady deadline；随后 wall clock 前跳或后跳都不改变授权剩余寿命。
    const std::uint64_t steady_after_ten_seconds = 510U * kNanosecondsPerSecond;
    EXPECT_EQ(deadline.value() - steady_after_ten_seconds, 49U * kNanosecondsPerSecond + 750000000U);
}

TEST(GrantExpiryTest, RejectsExpiredAndOverflowingDeadlines) {
    constexpr std::uint64_t kNanosecondsPerSecond = 1000000000U;
    EXPECT_EQ(grantExpiryToSteadyDeadline(1000U, 1000U * kNanosecondsPerSecond, 1U).status().code(),
              ErrorCode::kDeadlineExceeded);
    EXPECT_EQ(grantExpiryToSteadyDeadline(std::numeric_limits<std::uint64_t>::max(), 0U, 1U).status().code(),
              ErrorCode::kInvalidArgument);
    EXPECT_EQ(grantExpiryToSteadyDeadline(2U, 0U, std::numeric_limits<std::uint64_t>::max() - 1U).status().code(),
              ErrorCode::kInvalidArgument);
}

TEST(TeleopStateMachineTest, RequiresAuthorizationBeforeControl) {
    TeleopStateMachine state_machine;
    EXPECT_FALSE(state_machine.transition(TeleopEvent::kPeerConnected, "illegal").ok());
    ASSERT_TRUE(state_machine.transition(TeleopEvent::kAuthorize, "authorized").ok());
    ASSERT_TRUE(state_machine.transition(TeleopEvent::kPeerConnected, "connected").ok());
    EXPECT_EQ(state_machine.state(), TeleopState::kConnected);
    ASSERT_TRUE(state_machine.transition(TeleopEvent::kFrameValid, "valid").ok());
    EXPECT_EQ(state_machine.state(), TeleopState::kArmed);
    ASSERT_TRUE(state_machine.transition(TeleopEvent::kDeadmanPressed, "pressed").ok());
    EXPECT_EQ(state_machine.state(), TeleopState::kControlling);
    ASSERT_TRUE(state_machine.transition(TeleopEvent::kDeadmanReleased, "released").ok());
    EXPECT_EQ(state_machine.state(), TeleopState::kArmed);
    ASSERT_TRUE(state_machine.transition(TeleopEvent::kStopRequested, "stop").ok());
    ASSERT_TRUE(state_machine.transition(TeleopEvent::kStopCompleted, "closed").ok());
    EXPECT_EQ(state_machine.state(), TeleopState::kClosed);
}

TEST(TeleopStateMachineTest, ProductionRequiresOwnerBeforeArmed) {
    TeleopStateMachine state_machine;
    ASSERT_TRUE(state_machine.transition(TeleopEvent::kAuthorize, "authorized").ok());
    ASSERT_TRUE(state_machine.transition(TeleopEvent::kPeerConnected, "connected").ok());
    EXPECT_EQ(state_machine.state(), TeleopState::kConnected);
    EXPECT_FALSE(state_machine.transition(TeleopEvent::kDeadmanPressed, "no_owner").ok());
    EXPECT_EQ(state_machine.state(), TeleopState::kConnected);
    ASSERT_TRUE(state_machine.transition(TeleopEvent::kOwnerAcquired, "owner_acquired").ok());
    EXPECT_EQ(state_machine.state(), TeleopState::kArmed);
    ASSERT_TRUE(state_machine.transition(TeleopEvent::kDeadmanPressed, "deadman_pressed").ok());
    EXPECT_EQ(state_machine.state(), TeleopState::kControlling);
    ASSERT_TRUE(state_machine.transition(TeleopEvent::kOwnerReleased, "owner_released").ok());
    EXPECT_EQ(state_machine.state(), TeleopState::kConnected);
}

TEST(LatestMailboxTest, OverwritesUnconsumedHistoricalAction) {
    LatestMailbox<int> mailbox;
    mailbox.push(1);
    mailbox.push(2);
    EXPECT_EQ(mailbox.overwriteCount(), 1U);
    const auto latest = mailbox.take();
    ASSERT_TRUE(latest.has_value());
    EXPECT_EQ(*latest, 2);
    EXPECT_FALSE(mailbox.take().has_value());
}

TEST(SessionRegistryTest, IsolatesSingleWriterAndPacketIdentity) {
    SessionRegistry registry;
    SessionBinding first{"session-1", "peer-1",        "run-1", "thor", DataChannelContracts::kLabel,
                         1000U,       "fingerprint-1", false};
    ASSERT_TRUE(registry.authorize(first, false).ok());
    EXPECT_TRUE(registry.matchesAuthorization(first));
    EXPECT_FALSE(registry.matches("session-1", "peer-1", DataChannelContracts::kLabel));
    ASSERT_TRUE(registry.current().has_value());
    EXPECT_FALSE(registry.current()->connected);
    EXPECT_FALSE(registry
                     .authorize(SessionBinding{"session-2", "peer-2", "run-2", "thor", DataChannelContracts::kLabel,
                                               1000U, "fingerprint-2", false},
                                false)
                     .ok());
    SessionBinding same_route_different_grant = first;
    same_route_different_grant.grant_fingerprint = "fingerprint-2";
    EXPECT_FALSE(registry.matchesAuthorization(same_route_different_grant));
    EXPECT_FALSE(registry.markConnected("session-1", "peer-1", "astrabot.teleop.versioned").ok());
    EXPECT_FALSE(registry.matches("session-1", "peer-1", DataChannelContracts::kLabel));
    ASSERT_TRUE(registry.markConnected("session-1", "peer-1", DataChannelContracts::kLabel).ok());
    EXPECT_TRUE(registry.matches("session-1", "peer-1", DataChannelContracts::kLabel));
    EXPECT_FALSE(registry.matches("session-1", "peer-evil", DataChannelContracts::kLabel));
    EXPECT_FALSE(registry.close("session-2").ok());
    EXPECT_TRUE(registry.close("session-1").ok());
    EXPECT_FALSE(registry.matches("session-1", "peer-1", DataChannelContracts::kLabel));
}

TEST(SessionRegistryTest, AllowsEmptyRunOnlyWhenCallerExplicitlySelectsShadowPolicy) {
    const SessionBinding runless{"session-1", "peer-1",        "",   "thor", DataChannelContracts::kLabel,
                                 1000U,       "fingerprint-1", false};
    SessionRegistry production_registry;
    EXPECT_FALSE(production_registry.authorize(runless, false).ok());

    SessionRegistry shadow_registry;
    EXPECT_TRUE(shadow_registry.authorize(runless, true).ok());
}

TEST(TeleopConnectionLifecycleTest, AuthorizationAloneDoesNotStartConnectedWatchdog) {
    TeleopStateMachine state_machine;
    MotionWatchdog watchdog(120U);

    ASSERT_TRUE(state_machine.transition(TeleopEvent::kAuthorize, "authorized").ok());
    EXPECT_EQ(state_machine.state(), TeleopState::kAuthorized);
    EXPECT_FALSE(watchdog.expired(1000U));

    ASSERT_TRUE(state_machine.transition(TeleopEvent::kPeerConnected, "data_channel_open").ok());
    watchdog.observe(1000U);
    EXPECT_EQ(state_machine.state(), TeleopState::kConnected);
    EXPECT_FALSE(watchdog.expired(1120U));
    EXPECT_TRUE(watchdog.expired(1121U));

    ASSERT_TRUE(state_machine.transition(TeleopEvent::kStopRequested, "data_channel_closed").ok());
    ASSERT_TRUE(state_machine.transition(TeleopEvent::kStopCompleted, "data_channel_closed").ok());
    watchdog.reset();
    EXPECT_EQ(state_machine.state(), TeleopState::kClosed);
    EXPECT_FALSE(watchdog.expired(5000U));
}

TEST(MotionWatchdogTest, UsesStrictBoundedSteadyTimeout) {
    MotionWatchdog watchdog(120U);
    EXPECT_FALSE(watchdog.expired(1000U));
    watchdog.observe(1000U);
    EXPECT_FALSE(watchdog.expired(1120U));
    EXPECT_TRUE(watchdog.expired(1121U));
    watchdog.reset();
    EXPECT_FALSE(watchdog.expired(5000U));
}

TEST(CommandLimiterTest, RejectsWorkspaceAndPositionJump) {
    CommandLimiter limiter(CommandLimitConfig{-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 0.2});
    DecodedTeleopFrame frame;
    frame.pose_valid_right = true;
    frame.action_right.present = true;
    frame.action_right.qw = 1.0;
    ASSERT_TRUE(limiter.validateAndCommit(frame, 1000000000U).ok());
    frame.action_right.x = 0.21;
    EXPECT_FALSE(limiter.validateAndCommit(frame, 2000000000U).ok());
    limiter.reset();
    frame.action_right.x = 1.01;
    EXPECT_FALSE(limiter.validateAndCommit(frame, 3000000000U).ok());
}

TEST(CommandLimiterTest, EnforcesVelocityAccelerationAndMonotonicTime) {
    CommandLimitConfig config{-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 1.0};
    config.max_position_velocity_mps = 2.0;
    config.max_position_acceleration_mps2 = 5.0;
    CommandLimiter limiter(config);

    DecodedTeleopFrame frame;
    frame.pose_valid_right = true;
    frame.action_right.present = true;
    frame.action_right.qw = 1.0;
    ASSERT_TRUE(limiter.validateAndCommit(frame, 1000000000U).ok());

    frame.action_right.x = 0.1;
    ASSERT_TRUE(limiter.validateAndCommit(frame, 1100000000U).ok());
    EXPECT_FALSE(limiter.validateAndCommit(frame, 1100000000U).ok());

    frame.action_right.x = 0.26;
    EXPECT_FALSE(limiter.validateAndCommit(frame, 1200000000U).ok());

    frame.action_right.x = 0.2;
    EXPECT_TRUE(limiter.validateAndCommit(frame, 1200000000U).ok());
}

TEST(CommandLimiterTest, RejectsVelocityBeforeCommittingAnyArmHistory) {
    CommandLimitConfig config{-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 1.0};
    config.max_position_velocity_mps = 1.0;
    config.max_position_acceleration_mps2 = 100.0;
    CommandLimiter limiter(config);

    DecodedTeleopFrame frame;
    frame.pose_valid_right = true;
    frame.pose_valid_left = true;
    frame.action_right.present = true;
    frame.action_left.present = true;
    frame.action_right.qw = 1.0;
    frame.action_left.qw = 1.0;
    ASSERT_TRUE(limiter.validateAndCommit(frame, 1000000000U).ok());

    frame.action_right.x = 0.05;
    frame.action_left.x = 0.2;
    EXPECT_FALSE(limiter.validateAndCommit(frame, 1100000000U).ok());

    frame.action_left.x = 0.05;
    EXPECT_TRUE(limiter.validateAndCommit(frame, 1100000000U).ok());
}

TEST(CommandMapperTest, DerivesDeadmanAndBoundedChassisShadow) {
    CommandMapper mapper(CommandMappingConfig{0.5, 0.3, 0.5});
    DecodedTeleopFrame frame;
    frame.pose_valid_left = true;
    frame.action_left.present = true;
    frame.action_left.qw = 1.0;
    frame.axes.present = true;
    frame.axes.grip_left = 0.6;
    frame.axes.trigger_left = 0.7;
    frame.axes.joystick_left_x = 0.4;
    frame.axes.joystick_left_y = -0.5;
    const auto command = mapper.map(frame);
    EXPECT_TRUE(command.deadman);
    EXPECT_TRUE(command.left_deadman);
    EXPECT_FALSE(command.right_deadman);
    EXPECT_TRUE(command.left_gripper_valid);
    EXPECT_TRUE(command.right_gripper_valid);
    EXPECT_DOUBLE_EQ(command.left_gripper, 1.0);
    EXPECT_NEAR(command.chassis_linear_x, -0.15, 1.0e-9);
    EXPECT_NEAR(command.chassis_angular_z, -0.2, 1.0e-9);
}

TEST(CommandMapperTest, PreservesLegacyFullSqueezeReleaseGripperToggle) {
    CommandMapper mapper(CommandMappingConfig{});
    DecodedTeleopFrame frame;
    frame.axes.present = true;

    frame.axes.trigger_left = 0.95;
    EXPECT_DOUBLE_EQ(mapper.map(frame).left_gripper, 1.0);
    frame.axes.trigger_left = 0.05;
    EXPECT_DOUBLE_EQ(mapper.map(frame).left_gripper, 1.0);
    frame.axes.trigger_left = 0.0;
    EXPECT_DOUBLE_EQ(mapper.map(frame).left_gripper, 1.0);

    frame.axes.trigger_left = 0.95;
    EXPECT_DOUBLE_EQ(mapper.map(frame).left_gripper, 1.0);
    frame.axes.trigger_left = 0.05;
    EXPECT_DOUBLE_EQ(mapper.map(frame).left_gripper, 0.0);

    frame.axes.trigger_left = 0.95;
    static_cast<void>(mapper.map(frame));
    frame.axes.trigger_left = 0.05;
    EXPECT_DOUBLE_EQ(mapper.map(frame).left_gripper, 1.0);
    mapper.reset();
    frame.axes.trigger_left = 0.0;
    EXPECT_DOUBLE_EQ(mapper.map(frame).left_gripper, 0.0);
}

TEST(ArmOriginMapperTest, CapturesIndependentBaseFrameOriginAndRecapturesAfterRelease) {
    ArmOriginMapper mapper(ArmOriginMappingConfig{100U, 0.05});
    PoseSample right_robot_pose;
    right_robot_pose.present = true;
    right_robot_pose.x = 1.0;
    right_robot_pose.y = 2.0;
    right_robot_pose.z = 3.0;
    right_robot_pose.qz = std::sqrt(0.5);
    right_robot_pose.qw = std::sqrt(0.5);
    PoseSample left_robot_pose;
    left_robot_pose.present = true;
    left_robot_pose.qw = 1.0;
    ASSERT_TRUE(mapper.updateRobotPose(right_robot_pose, left_robot_pose, 1000U).ok());

    MappedCommand relative;
    relative.deadman = true;
    relative.right_deadman = true;
    relative.right_arm_valid = true;
    relative.right_gripper_valid = true;
    relative.right_arm_target.present = true;
    relative.right_arm_target.x = 1.0;
    relative.right_arm_target.qw = 1.0;
    relative.head_valid = true;
    relative.head_target.present = true;
    relative.head_target.qw = 1.0;

    auto first = mapper.map(relative, 1050U);
    ASSERT_TRUE(first.ok());
    EXPECT_TRUE(first.value().right_origin_captured);
    EXPECT_TRUE(first.value().right_arm_valid);
    EXPECT_TRUE(first.value().right_gripper_valid);
    EXPECT_FALSE(first.value().left_arm_valid);
    EXPECT_FALSE(first.value().left_gripper_valid);
    EXPECT_FALSE(first.value().head_valid);
    EXPECT_NEAR(first.value().right_arm_target.x, 1.0, 1.0e-9);
    EXPECT_NEAR(first.value().right_arm_target.y, 3.0, 1.0e-9);
    EXPECT_NEAR(first.value().right_arm_target.z, 3.0, 1.0e-9);
    EXPECT_NEAR(first.value().right_arm_target.qz, std::sqrt(0.5), 1.0e-9);
    EXPECT_NEAR(first.value().right_arm_target.qw, std::sqrt(0.5), 1.0e-9);

    right_robot_pose.x = 10.0;
    ASSERT_TRUE(mapper.updateRobotPose(right_robot_pose, left_robot_pose, 1070U).ok());
    relative.right_arm_target.x = 2.0;
    auto continued = mapper.map(relative, 1080U);
    ASSERT_TRUE(continued.ok());
    EXPECT_FALSE(continued.value().right_origin_captured);
    EXPECT_NEAR(continued.value().right_arm_target.x, 1.0, 1.0e-9);
    EXPECT_NEAR(continued.value().right_arm_target.y, 4.0, 1.0e-9);

    MappedCommand released;
    ASSERT_TRUE(mapper.map(released, 1090U).ok());
    relative.right_arm_target.x = 0.0;
    auto recaptured = mapper.map(relative, 1100U);
    ASSERT_TRUE(recaptured.ok());
    EXPECT_TRUE(recaptured.value().right_origin_captured);
    EXPECT_NEAR(recaptured.value().right_arm_target.x, 10.0, 1.0e-9);
}

TEST(ArmOriginMapperTest, FailsClosedWhenRobotPoseIsMissingStaleOrInvalid) {
    ArmOriginMapper mapper(ArmOriginMappingConfig{100U, 0.05});
    MappedCommand active;
    active.deadman = true;
    active.right_deadman = true;
    active.right_arm_valid = true;
    active.right_arm_target.present = true;
    active.right_arm_target.qw = 1.0;
    EXPECT_EQ(mapper.map(active, 1000U).status().code(), ErrorCode::kUnavailable);

    PoseSample valid_pose;
    valid_pose.present = true;
    valid_pose.qw = 1.0;
    ASSERT_TRUE(mapper.updateRobotPose(valid_pose, valid_pose, 1000U).ok());
    EXPECT_EQ(mapper.map(active, 1101U).status().code(), ErrorCode::kDeadlineExceeded);

    PoseSample invalid_pose = valid_pose;
    invalid_pose.qw = 0.0;
    EXPECT_EQ(mapper.updateRobotPose(invalid_pose, valid_pose, 1200U).code(), ErrorCode::kInvalidArgument);

    MappedCommand inactive;
    auto inactive_result = mapper.map(inactive, 5000U);
    ASSERT_TRUE(inactive_result.ok());
    EXPECT_FALSE(inactive_result.value().deadman);
}

TEST(TeleopConfigTest, KeepsShadowAndProductionOutputsMutuallyExclusive) {
    TeleopConfig config;
    config.device_id = "robot-001";
    EXPECT_TRUE(config.validate().ok());
    EXPECT_TRUE(config.shadowOutputEnabled());
    EXPECT_FALSE(config.productionOutputEnabled());
    EXPECT_FALSE(config.legacyMpcOutputEnabled());
    EXPECT_FALSE(config.controlOutputEnabled());

    config.backend = "legacy_mpc";
    EXPECT_TRUE(config.validate().ok());
    EXPECT_FALSE(config.shadowOutputEnabled());
    EXPECT_FALSE(config.productionOutputEnabled());
    EXPECT_TRUE(config.legacyMpcOutputEnabled());
    EXPECT_TRUE(config.controlOutputEnabled());

    config.legacy_reference_pose_topic.clear();
    EXPECT_FALSE(config.validate().ok());
    config.legacy_reference_pose_topic = "/reference/pose";
    config.legacy_control_name = "wbmpc_policy_ctrl";
    EXPECT_FALSE(config.validate().ok());
    config.legacy_control_name = "wbmpc_remote_ctrl";

    config.backend = "cpp";
    EXPECT_TRUE(config.validate().ok());
    EXPECT_FALSE(config.shadowOutputEnabled());
    EXPECT_TRUE(config.productionOutputEnabled());
    EXPECT_FALSE(config.legacyMpcOutputEnabled());
    EXPECT_TRUE(config.controlOutputEnabled());

    config.production_command_topic = config.shadow_command_topic;
    EXPECT_FALSE(config.validate().ok());

    config = TeleopConfig{};
    config.device_id = "robot-001";
    config.gripper_binary_threshold = config.gripper_toggle_high_threshold;
    EXPECT_FALSE(config.validate().ok());
}

TEST(TeleopConfigTest, EnforcesWatchdogAndOwnerLeaseMargins) {
    TeleopConfig config;
    config.device_id = "robot-001";
    EXPECT_EQ(config.max_frame_age_ms, 100);
    EXPECT_TRUE(config.validate().ok());

    config.max_frame_age_ms = config.command_ttl_ms + 1;
    EXPECT_FALSE(config.validate().ok());

    config.max_frame_age_ms = config.command_ttl_ms;
    config.watchdog_timeout_ms = 151;
    EXPECT_FALSE(config.validate().ok());

    config.watchdog_timeout_ms = 120;
    config.backend = "cpp";
    config.owner_renew_period_ms = 110;
    config.owner_service_timeout_ms = 40;
    EXPECT_FALSE(config.validate().ok());
}

TEST(TeleopConfigTest, BoundsNonBlockingStatusReportClient) {
    TeleopConfig config;
    config.device_id = "robot-001";
    EXPECT_TRUE(config.validate().ok());

    config.report_teleop_status_service.clear();
    EXPECT_FALSE(config.validate().ok());
    config.report_teleop_status_service = "/astrabot/data_collection/report_teleop_status";
    config.status_report_poll_period_ms = 101;
    EXPECT_FALSE(config.validate().ok());
    config.status_report_poll_period_ms = 10;
    config.status_report_retry_period_ms = 99;
    EXPECT_FALSE(config.validate().ok());

    config.status_report_retry_period_ms = 1000;
    config.max_position_velocity_mps = 0.0;
    EXPECT_FALSE(config.validate().ok());

    config.max_position_velocity_mps = 1.0;
    config.robot_pose_max_age_ms = 0;
    EXPECT_FALSE(config.validate().ok());
}

}  // namespace
}  // namespace astrabot::teleop
