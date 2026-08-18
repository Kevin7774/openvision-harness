// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include "astrabot_teleop/adapter/legacy_mpc_command_encoder.h"

#include <cmath>

#include <nlohmann/json.hpp>

namespace astrabot::teleop {
namespace {

bool validPose(const PoseSample &pose) {
    if (!pose.present || !std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.z) ||
        !std::isfinite(pose.qx) || !std::isfinite(pose.qy) || !std::isfinite(pose.qz) || !std::isfinite(pose.qw)) {
        return false;
    }
    const double squared_norm = pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz + pose.qw * pose.qw;
    return std::isfinite(squared_norm) && squared_norm > 0.5 && squared_norm < 1.5;
}

void appendPose(nlohmann::json *root, const char *name, const PoseSample &pose) {
    (*root)[name]["position"] = {pose.x, pose.y, pose.z};
    (*root)[name]["orientation"] = {pose.qx, pose.qy, pose.qz, pose.qw};
}

}  // namespace

Result<std::string> LegacyMpcCommandEncoder::encodeCommand(const MappedCommand &command,
                                                           const std::string &control_name) const {
    if (!command.deadman) {
        return Result<std::string>::failure(
            Status::error(ErrorCode::kFailedPrecondition, "legacy MPC command requires active deadman"));
    }
    const PoseSample *right_arm = command.right_arm_valid ? &command.right_arm_target : nullptr;
    const PoseSample *left_arm = command.left_arm_valid ? &command.left_arm_target : nullptr;
    return encode(right_arm, left_arm, control_name);
}

Result<std::string> LegacyMpcCommandEncoder::encodeHold(const PoseSample &right_arm, const PoseSample &left_arm,
                                                        const std::string &control_name) const {
    return encode(&right_arm, &left_arm, control_name);
}

Result<std::string> LegacyMpcCommandEncoder::encode(const PoseSample *right_arm, const PoseSample *left_arm,
                                                    const std::string &control_name) const {
    if (control_name.empty()) {
        return Result<std::string>::failure(
            Status::error(ErrorCode::kInvalidArgument, "legacy MPC control name is empty"));
    }
    if (right_arm == nullptr && left_arm == nullptr) {
        return Result<std::string>::failure(
            Status::error(ErrorCode::kInvalidArgument, "legacy MPC command has no valid arm target"));
    }
    if ((right_arm != nullptr && !validPose(*right_arm)) || (left_arm != nullptr && !validPose(*left_arm))) {
        return Result<std::string>::failure(
            Status::error(ErrorCode::kInvalidArgument, "legacy MPC command contains an invalid arm pose"));
    }

    nlohmann::json payload;
    payload["ctrl_name"] = control_name;
    if (right_arm != nullptr) {
        appendPose(&payload, "right_arm", *right_arm);
    }
    if (left_arm != nullptr) {
        appendPose(&payload, "left_arm", *left_arm);
    }
    return Result<std::string>::success(payload.dump());
}

}  // namespace astrabot::teleop
