// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include "astrabot_teleop/safety/command_limiter.h"

#include <cmath>

namespace astrabot::teleop {
namespace {

constexpr double kNanosecondsPerSecond = 1000000000.0;

}  // namespace

CommandLimiter::CommandLimiter(CommandLimitConfig config) : config_(config) {}

Result<CommandLimiter::PoseHistory>
CommandLimiter::validatedPoseHistory(const PoseSample &pose, const bool valid, const PoseHistory &previous,
                                     const std::uint64_t sample_steady_time_ns) const {
    if (!valid) {
        return Result<PoseHistory>::success(previous);
    }
    if (!pose.present || pose.x < config_.min_x || pose.x > config_.max_x || pose.y < config_.min_y ||
        pose.y > config_.max_y || pose.z < config_.min_z || pose.z > config_.max_z) {
        return Result<PoseHistory>::failure(
            Status::error(ErrorCode::kInvalidArgument, "Teleop pose is outside the configured workspace"));
    }
    if (sample_steady_time_ns == 0U) {
        return Result<PoseHistory>::failure(
            Status::error(ErrorCode::kInvalidArgument, "Teleop pose steady timestamp is zero"));
    }

    PoseHistory next = previous;
    if (!previous.has_pose) {
        next.pose = pose;
        next.steady_time_ns = sample_steady_time_ns;
        next.has_pose = true;
        next.has_velocity = false;
        return Result<PoseHistory>::success(next);
    }
    if (sample_steady_time_ns <= previous.steady_time_ns) {
        return Result<PoseHistory>::failure(
            Status::error(ErrorCode::kConflict, "Teleop pose steady timestamp did not advance"));
    }

    const double dx = pose.x - previous.pose.x;
    const double dy = pose.y - previous.pose.y;
    const double dz = pose.z - previous.pose.z;
    const double distance_m = std::hypot(dx, dy, dz);
    if (distance_m > config_.max_position_step_m) {
        return Result<PoseHistory>::failure(
            Status::error(ErrorCode::kInvalidArgument, "Teleop pose exceeds the per-frame position step limit"));
    }

    const double delta_seconds =
        static_cast<double>(sample_steady_time_ns - previous.steady_time_ns) / kNanosecondsPerSecond;
    const double velocity_x_mps = dx / delta_seconds;
    const double velocity_y_mps = dy / delta_seconds;
    const double velocity_z_mps = dz / delta_seconds;
    if (std::hypot(velocity_x_mps, velocity_y_mps, velocity_z_mps) > config_.max_position_velocity_mps) {
        return Result<PoseHistory>::failure(
            Status::error(ErrorCode::kInvalidArgument, "Teleop pose exceeds the position velocity limit"));
    }

    if (previous.has_velocity) {
        const double acceleration_x_mps2 = (velocity_x_mps - previous.velocity_x_mps) / delta_seconds;
        const double acceleration_y_mps2 = (velocity_y_mps - previous.velocity_y_mps) / delta_seconds;
        const double acceleration_z_mps2 = (velocity_z_mps - previous.velocity_z_mps) / delta_seconds;
        if (std::hypot(acceleration_x_mps2, acceleration_y_mps2, acceleration_z_mps2) >
            config_.max_position_acceleration_mps2) {
            return Result<PoseHistory>::failure(
                Status::error(ErrorCode::kInvalidArgument, "Teleop pose exceeds the position acceleration limit"));
        }
    }

    next.pose = pose;
    next.steady_time_ns = sample_steady_time_ns;
    next.velocity_x_mps = velocity_x_mps;
    next.velocity_y_mps = velocity_y_mps;
    next.velocity_z_mps = velocity_z_mps;
    next.has_pose = true;
    next.has_velocity = true;
    return Result<PoseHistory>::success(next);
}

Status CommandLimiter::validateAndCommit(const DecodedTeleopFrame &frame, const std::uint64_t sample_steady_time_ns) {
    auto right_result =
        validatedPoseHistory(frame.action_right, frame.pose_valid_right, right_history_, sample_steady_time_ns);
    if (!right_result.ok()) {
        return right_result.status();
    }
    auto left_result =
        validatedPoseHistory(frame.action_left, frame.pose_valid_left, left_history_, sample_steady_time_ns);
    if (!left_result.ok()) {
        return left_result.status();
    }
    auto head_result =
        validatedPoseHistory(frame.action_head, frame.pose_valid_head, head_history_, sample_steady_time_ns);
    if (!head_result.ok()) {
        return head_result.status();
    }

    right_history_ = right_result.takeValue();
    left_history_ = left_result.takeValue();
    head_history_ = head_result.takeValue();
    return Status::success();
}

void CommandLimiter::reset() {
    right_history_ = {};
    left_history_ = {};
    head_history_ = {};
}

void CommandLimiter::resetArmHistories(const bool right_arm, const bool left_arm) {
    if (right_arm) {
        right_history_ = {};
    }
    if (left_arm) {
        left_history_ = {};
    }
}

}  // namespace astrabot::teleop
