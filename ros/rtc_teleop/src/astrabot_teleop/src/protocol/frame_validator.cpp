// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include "astrabot_teleop/protocol/frame_validator.h"

#include <cmath>
#include <limits>

namespace astrabot::teleop {
namespace {

bool quaternionIsValid(const PoseSample &pose, const double tolerance) {
    if (!pose.present) {
        return true;
    }
    const double norm_squared = pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz + pose.qw * pose.qw;
    if (!std::isfinite(norm_squared) || norm_squared <= std::numeric_limits<double>::epsilon()) {
        return false;
    }
    return std::abs(std::sqrt(norm_squared) - 1.0) <= tolerance;
}

bool inRange(const double value, const double minimum, const double maximum) {
    return value >= minimum && value <= maximum;
}

}  // namespace

FrameValidator::FrameValidator(FrameValidationConfig config) : config_(config) {}

Result<FrameValidationOutcome> FrameValidator::validate(const DecodedTeleopFrame &frame,
                                                        const std::int64_t system_now_ms) const {
    if (frame.timestamp_ms <= 0) {
        return Result<FrameValidationOutcome>::failure(
            Status::error(ErrorCode::kInvalidArgument, "teleop frame timestamp must be positive"));
    }
    if (frame.timestamp_ms > system_now_ms && frame.timestamp_ms - system_now_ms > config_.max_future_skew_ms) {
        return Result<FrameValidationOutcome>::failure(
            Status::error(ErrorCode::kDeadlineExceeded, "teleop frame timestamp is too far in the future"));
    }
    if (frame.timestamp_ms <= system_now_ms && system_now_ms - frame.timestamp_ms > config_.max_frame_age_ms) {
        return Result<FrameValidationOutcome>::failure(
            Status::error(ErrorCode::kDeadlineExceeded, "teleop frame is stale"));
    }
    if (has_previous_ && frame.sequence <= last_sequence_) {
        return Result<FrameValidationOutcome>::failure(
            Status::error(ErrorCode::kConflict, "teleop frame sequence replay or regression"));
    }
    if (has_previous_ && frame.timestamp_ms < last_timestamp_ms_) {
        return Result<FrameValidationOutcome>::failure(
            Status::error(ErrorCode::kConflict, "teleop frame timestamp regressed"));
    }
    if (!frame.all_values_finite) {
        return Result<FrameValidationOutcome>::failure(
            Status::error(ErrorCode::kInvalidArgument, "teleop frame contains NaN or infinity"));
    }
    if ((frame.pose_valid_right && !frame.action_right.present) ||
        (frame.pose_valid_left && !frame.action_left.present) ||
        (frame.pose_valid_head && !frame.action_head.present)) {
        return Result<FrameValidationOutcome>::failure(
            Status::error(ErrorCode::kInvalidArgument, "pose validity flag requires the matching pose"));
    }
    if (!quaternionIsValid(frame.action_right, config_.quaternion_norm_tolerance) ||
        !quaternionIsValid(frame.action_left, config_.quaternion_norm_tolerance) ||
        !quaternionIsValid(frame.action_head, config_.quaternion_norm_tolerance) ||
        !quaternionIsValid(frame.calibration_right, config_.quaternion_norm_tolerance) ||
        !quaternionIsValid(frame.calibration_left, config_.quaternion_norm_tolerance) ||
        !quaternionIsValid(frame.calibration_head, config_.quaternion_norm_tolerance)) {
        return Result<FrameValidationOutcome>::failure(
            Status::error(ErrorCode::kInvalidArgument, "teleop frame contains an invalid quaternion"));
    }
    if (frame.axes.present &&
        (!inRange(frame.axes.trigger_right, 0.0, 1.0) || !inRange(frame.axes.trigger_left, 0.0, 1.0) ||
         !inRange(frame.axes.grip_right, 0.0, 1.0) || !inRange(frame.axes.grip_left, 0.0, 1.0) ||
         !inRange(frame.axes.joystick_right_x, -1.0, 1.0) || !inRange(frame.axes.joystick_right_y, -1.0, 1.0) ||
         !inRange(frame.axes.joystick_left_x, -1.0, 1.0) || !inRange(frame.axes.joystick_left_y, -1.0, 1.0))) {
        return Result<FrameValidationOutcome>::failure(
            Status::error(ErrorCode::kInvalidArgument, "teleop controller axis is outside its contract range"));
    }
    if (frame.hand_skeleton_right_joint_count > config_.expected_hand_joint_count ||
        frame.hand_skeleton_left_joint_count > config_.expected_hand_joint_count ||
        (frame.hand_skeleton_valid_right &&
         frame.hand_skeleton_right_joint_count != config_.expected_hand_joint_count) ||
        (frame.hand_skeleton_valid_left && frame.hand_skeleton_left_joint_count != config_.expected_hand_joint_count)) {
        return Result<FrameValidationOutcome>::failure(
            Status::error(ErrorCode::kInvalidArgument, "teleop hand skeleton joint count is invalid"));
    }

    FrameValidationOutcome outcome;
    if (has_previous_ && frame.sequence > last_sequence_ + 1U) {
        outcome.sequence_gap = frame.sequence - last_sequence_ - 1U;
    }
    return Result<FrameValidationOutcome>::success(outcome);
}

void FrameValidator::commit(const DecodedTeleopFrame &frame, const FrameValidationOutcome &outcome) {
    has_previous_ = true;
    last_sequence_ = frame.sequence;
    last_timestamp_ms_ = frame.timestamp_ms;
    sequence_gap_count_ += outcome.sequence_gap;
}

void FrameValidator::reset() {
    has_previous_ = false;
    last_sequence_ = 0;
    last_timestamp_ms_ = 0;
    sequence_gap_count_ = 0;
}

std::uint64_t FrameValidator::lastSequence() const {
    return last_sequence_;
}

std::uint64_t FrameValidator::sequenceGapCount() const {
    return sequence_gap_count_;
}

}  // namespace astrabot::teleop
