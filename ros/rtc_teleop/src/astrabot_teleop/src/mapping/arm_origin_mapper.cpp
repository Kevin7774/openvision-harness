// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include "astrabot_teleop/mapping/arm_origin_mapper.h"

#include <cmath>
#include <limits>
#include <string>

namespace astrabot::teleop {
namespace {

bool finitePose(const PoseSample &pose) {
    return pose.present && std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.z) &&
           std::isfinite(pose.qx) && std::isfinite(pose.qy) && std::isfinite(pose.qz) && std::isfinite(pose.qw);
}

Result<PoseSample> normalizedPose(PoseSample pose, const double tolerance, const char *label) {
    if (!finitePose(pose)) {
        return Result<PoseSample>::failure(
            Status::error(ErrorCode::kInvalidArgument, std::string(label) + " pose contains NaN, Inf or is absent"));
    }
    const double norm = std::sqrt(pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz + pose.qw * pose.qw);
    if (!std::isfinite(norm) || norm <= std::numeric_limits<double>::epsilon() || std::abs(norm - 1.0) > tolerance) {
        return Result<PoseSample>::failure(
            Status::error(ErrorCode::kInvalidArgument, std::string(label) + " quaternion is invalid"));
    }
    pose.qx /= norm;
    pose.qy /= norm;
    pose.qz /= norm;
    pose.qw /= norm;
    return Result<PoseSample>::success(pose);
}

PoseSample composePose(const PoseSample &origin, const PoseSample &relative) {
    const double tx = 2.0 * (origin.qy * relative.z - origin.qz * relative.y);
    const double ty = 2.0 * (origin.qz * relative.x - origin.qx * relative.z);
    const double tz = 2.0 * (origin.qx * relative.y - origin.qy * relative.x);

    PoseSample result;
    result.present = true;
    result.x = origin.x + relative.x + origin.qw * tx + origin.qy * tz - origin.qz * ty;
    result.y = origin.y + relative.y + origin.qw * ty + origin.qz * tx - origin.qx * tz;
    result.z = origin.z + relative.z + origin.qw * tz + origin.qx * ty - origin.qy * tx;
    result.qx = origin.qw * relative.qx + origin.qx * relative.qw + origin.qy * relative.qz - origin.qz * relative.qy;
    result.qy = origin.qw * relative.qy - origin.qx * relative.qz + origin.qy * relative.qw + origin.qz * relative.qx;
    result.qz = origin.qw * relative.qz + origin.qx * relative.qy - origin.qy * relative.qx + origin.qz * relative.qw;
    result.qw = origin.qw * relative.qw - origin.qx * relative.qx - origin.qy * relative.qy - origin.qz * relative.qz;
    const double norm =
        std::sqrt(result.qx * result.qx + result.qy * result.qy + result.qz * result.qz + result.qw * result.qw);
    result.qx /= norm;
    result.qy /= norm;
    result.qz /= norm;
    result.qw /= norm;
    return result;
}

}  // namespace

ArmOriginMapper::ArmOriginMapper(ArmOriginMappingConfig config) : config_(config) {}

Status ArmOriginMapper::updateRobotPose(const PoseSample &right_arm, const PoseSample &left_arm,
                                        const std::uint64_t receive_steady_time_ns) {
    if (receive_steady_time_ns == 0U) {
        return Status::error(ErrorCode::kInvalidArgument, "robot pose steady timestamp is zero");
    }
    auto normalized_right = normalizedPose(right_arm, config_.quaternion_norm_tolerance, "right robot end-effector");
    if (!normalized_right.ok()) {
        return normalized_right.status();
    }
    auto normalized_left = normalizedPose(left_arm, config_.quaternion_norm_tolerance, "left robot end-effector");
    if (!normalized_left.ok()) {
        return normalized_left.status();
    }
    robot_pose_.right_arm = normalized_right.takeValue();
    robot_pose_.left_arm = normalized_left.takeValue();
    robot_pose_.receive_steady_time_ns = receive_steady_time_ns;
    robot_pose_.available = true;
    return Status::success();
}

Result<MappedCommand> ArmOriginMapper::map(const MappedCommand &command, const std::uint64_t steady_now_ns) {
    MappedCommand mapped = command;
    mapped.right_origin_captured = false;
    mapped.left_origin_captured = false;

    bool next_right_active = right_active_;
    bool next_left_active = left_active_;
    PoseSample next_right_origin = right_origin_;
    PoseSample next_left_origin = left_origin_;

    auto right_result = mapArm(command.right_arm_target, command.right_deadman, &next_right_active, &next_right_origin,
                               &mapped.right_origin_captured, steady_now_ns, true);
    if (!right_result.ok()) {
        return Result<MappedCommand>::failure(right_result.status());
    }
    mapped.right_arm_target = right_result.takeValue();
    mapped.right_arm_valid = command.right_deadman;
    mapped.right_gripper_valid = command.right_gripper_valid && command.right_deadman;

    auto left_result = mapArm(command.left_arm_target, command.left_deadman, &next_left_active, &next_left_origin,
                              &mapped.left_origin_captured, steady_now_ns, false);
    if (!left_result.ok()) {
        return Result<MappedCommand>::failure(left_result.status());
    }
    mapped.left_arm_target = left_result.takeValue();
    mapped.left_arm_valid = command.left_deadman;
    mapped.left_gripper_valid = command.left_gripper_valid && command.left_deadman;

    // 头部相对位姿还没有机器人侧原点与独立使能闭环，先保持 fail closed。
    mapped.head_valid = false;
    mapped.head_target = PoseSample{};
    mapped.deadman = mapped.right_deadman || mapped.left_deadman;
    right_active_ = next_right_active;
    left_active_ = next_left_active;
    right_origin_ = next_right_origin;
    left_origin_ = next_left_origin;
    return Result<MappedCommand>::success(mapped);
}

void ArmOriginMapper::resetControlState() {
    right_origin_ = {};
    left_origin_ = {};
    right_active_ = false;
    left_active_ = false;
}

Status ArmOriginMapper::validateFreshSnapshot(const std::uint64_t steady_now_ns) const {
    if (!robot_pose_.available || robot_pose_.receive_steady_time_ns == 0U) {
        return Status::error(ErrorCode::kUnavailable, "robot end-effector pose is unavailable");
    }
    if (steady_now_ns < robot_pose_.receive_steady_time_ns ||
        steady_now_ns - robot_pose_.receive_steady_time_ns > config_.max_robot_pose_age_ns) {
        return Status::error(ErrorCode::kDeadlineExceeded, "robot end-effector pose is stale");
    }
    return Status::success();
}

Result<PoseSample> ArmOriginMapper::mapArm(const PoseSample &relative_pose, const bool deadman, bool *active,
                                           PoseSample *origin, bool *origin_captured, const std::uint64_t steady_now_ns,
                                           const bool right_arm) {
    if (!deadman) {
        *active = false;
        *origin = {};
        return Result<PoseSample>::success(PoseSample{});
    }
    auto normalized_relative = normalizedPose(relative_pose, config_.quaternion_norm_tolerance, "active relative arm");
    if (!normalized_relative.ok()) {
        return Result<PoseSample>::failure(normalized_relative.status());
    }
    if (!*active) {
        const Status freshness = validateFreshSnapshot(steady_now_ns);
        if (!freshness.ok()) {
            return Result<PoseSample>::failure(freshness);
        }
        *origin = right_arm ? robot_pose_.right_arm : robot_pose_.left_arm;
        *active = true;
        *origin_captured = true;
    }
    return Result<PoseSample>::success(composePose(*origin, normalized_relative.takeValue()));
}

}  // namespace astrabot::teleop
