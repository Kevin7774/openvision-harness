// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "TeleopFrame.pb.h"
#include "astrabot_teleop/protocol/crc32.h"
#include "astrabot_teleop/protocol/frame_validator.h"
#include "astrabot_teleop/protocol/teleop_frame_codec.h"

namespace astrabot::teleop {
namespace {

std::int64_t nowMilliseconds() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
        .count();
}

void fillPose(PoseAction *pose, const float x = 1.0F) {
    pose->set_pos_x(x);
    pose->set_pos_y(0.2F);
    pose->set_pos_z(0.3F);
    pose->set_rot_x(0.0F);
    pose->set_rot_y(0.0F);
    pose->set_rot_z(0.0F);
    pose->set_rot_w(1.0F);
}

void updateCrc(TeleopFrame *frame, const bool legacy) {
    std::string bytes;
    if (legacy) {
        TeleopFrame envelope;
        envelope.mutable_data_body()->CopyFrom(frame->data_body());
        ASSERT_TRUE(envelope.SerializeToString(&bytes));
    } else {
        ASSERT_TRUE(frame->data_body().SerializeToString(&bytes));
    }
    frame->mutable_header()->set_crc32_checksum(
        Crc32::compute(reinterpret_cast<const std::uint8_t *>(bytes.data()), bytes.size()));
}

TeleopFrame makeFrame(const std::uint64_t sequence, const bool legacy_crc = false) {
    TeleopFrame frame;
    frame.mutable_header()->set_timestamp_ms(nowMilliseconds());
    frame.mutable_header()->set_frame_index(sequence);
    frame.mutable_header()->set_pose_valid_right(true);
    frame.mutable_header()->set_pose_valid_left(true);
    fillPose(frame.mutable_data_body()->mutable_action_right());
    fillPose(frame.mutable_data_body()->mutable_action_left(), -1.0F);
    auto *axes = frame.mutable_data_body()->mutable_axes();
    axes->set_axis_trig_right(0.8F);
    axes->set_axis_trig_left(0.7F);
    axes->set_axis_grip_right(0.6F);
    axes->set_axis_grip_left(0.4F);
    axes->set_axis_js_left_x(0.2F);
    axes->set_axis_js_left_y(-0.3F);
    updateCrc(&frame, legacy_crc);
    return frame;
}

std::vector<std::uint8_t> serialize(const TeleopFrame &frame) {
    std::string bytes;
    EXPECT_TRUE(frame.SerializeToString(&bytes));
    return std::vector<std::uint8_t>(bytes.begin(), bytes.end());
}

TEST(TeleopFrameCodecTest, AcceptsFrozenDataBodyCrc) {
    TeleopFrameCodec codec(16384U);
    auto decoded = codec.decode(serialize(makeFrame(7U)));
    ASSERT_TRUE(decoded.ok()) << decoded.status().message();
    EXPECT_EQ(decoded.value().sequence, 7U);
    EXPECT_EQ(decoded.value().crc_variant, CrcVariant::kDataBody);
    EXPECT_TRUE(decoded.value().pose_valid_right);
    EXPECT_NEAR(decoded.value().axes.trigger_right, 0.8, 1.0e-6);
}

TEST(TeleopFrameCodecTest, AcceptsCurrentQuestLegacyEnvelopeCrc) {
    TeleopFrameCodec codec(16384U);
    auto decoded = codec.decode(serialize(makeFrame(8U, true)));
    ASSERT_TRUE(decoded.ok()) << decoded.status().message();
    EXPECT_EQ(decoded.value().crc_variant, CrcVariant::kQuestLegacyEnvelope);
}

TEST(TeleopFrameCodecTest, RejectsCrcMismatchAndOversize) {
    auto frame = makeFrame(9U);
    frame.mutable_header()->set_crc32_checksum(frame.header().crc32_checksum() + 1U);
    TeleopFrameCodec codec(16384U);
    EXPECT_FALSE(codec.decode(serialize(frame)).ok());

    TeleopFrameCodec tiny_codec(2U);
    EXPECT_EQ(tiny_codec.decode(serialize(makeFrame(10U))).status().code(), ErrorCode::kResourceExhausted);
}

TEST(TeleopFrameCodecTest, RejectsUnknownWireField) {
    auto payload = serialize(makeFrame(11U));
    payload.push_back(0x98U);  // field 99, varint wire type
    payload.push_back(0x06U);
    payload.push_back(0x01U);
    TeleopFrameCodec codec(16384U);
    EXPECT_FALSE(codec.decode(payload).ok());
}

TEST(FrameValidatorTest, EnforcesSequenceTimestampAndQuaternion) {
    const std::int64_t now_ms = nowMilliseconds();
    FrameValidator validator(FrameValidationConfig{250, 1000, 0.05, 26U});
    DecodedTeleopFrame frame;
    frame.sequence = 10U;
    frame.timestamp_ms = now_ms;
    frame.pose_valid_right = true;
    frame.action_right.present = true;
    frame.action_right.qw = 1.0;
    auto outcome = validator.validate(frame, now_ms);
    ASSERT_TRUE(outcome.ok());
    validator.commit(frame, outcome.value());

    EXPECT_FALSE(validator.validate(frame, now_ms).ok());
    frame.sequence = 12U;
    frame.timestamp_ms = now_ms + 1;
    outcome = validator.validate(frame, now_ms + 1);
    ASSERT_TRUE(outcome.ok());
    EXPECT_EQ(outcome.value().sequence_gap, 1U);

    frame.action_right.qw = 0.1;
    EXPECT_FALSE(validator.validate(frame, now_ms + 1).ok());
}

TEST(FrameValidatorTest, RejectsStaleNanAxisAndInvalidSkeleton) {
    const std::int64_t now_ms = nowMilliseconds();
    FrameValidator validator(FrameValidationConfig{100, 20, 0.05, 26U});
    DecodedTeleopFrame frame;
    frame.sequence = 1U;
    frame.timestamp_ms = now_ms - 101;
    EXPECT_EQ(validator.validate(frame, now_ms).status().code(), ErrorCode::kDeadlineExceeded);

    frame.timestamp_ms = now_ms;
    frame.all_values_finite = false;
    EXPECT_EQ(validator.validate(frame, now_ms).status().code(), ErrorCode::kInvalidArgument);

    frame.all_values_finite = true;
    frame.axes.present = true;
    frame.axes.trigger_right = 1.01;
    EXPECT_FALSE(validator.validate(frame, now_ms).ok());

    frame.axes = {};
    frame.hand_skeleton_valid_right = true;
    frame.hand_skeleton_right_joint_count = 25U;
    EXPECT_FALSE(validator.validate(frame, now_ms).ok());
}

TEST(TeleopFrameCodecTest, DetectsNanFromProtobufPayload) {
    auto frame = makeFrame(12U);
    frame.mutable_data_body()->mutable_action_right()->set_pos_x(std::numeric_limits<float>::quiet_NaN());
    updateCrc(&frame, false);
    TeleopFrameCodec codec(16384U);
    auto decoded = codec.decode(serialize(frame));
    ASSERT_TRUE(decoded.ok());
    EXPECT_FALSE(decoded.value().all_values_finite);
}

}  // namespace
}  // namespace astrabot::teleop
