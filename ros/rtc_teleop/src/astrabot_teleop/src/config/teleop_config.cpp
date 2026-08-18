// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include "astrabot_teleop/config/teleop_config.h"

#include <cmath>
#include <unordered_set>

namespace astrabot::teleop {

Status TeleopConfig::validate() const {
    if (backend != "disabled" && backend != "shadow" && backend != "legacy_mpc" && backend != "cpp") {
        return Status::error(ErrorCode::kInvalidArgument, "backend must be disabled, shadow, legacy_mpc or cpp");
    }
    if (backend != "disabled" && device_id.empty()) {
        return Status::error(ErrorCode::kInvalidArgument, "device_id is required; grants fail closed without it");
    }
    if (resource_id.empty()) {
        return Status::error(ErrorCode::kInvalidArgument, "resource_id is required");
    }
    if (max_frame_bytes == 0 || max_frame_bytes > 1024U * 1024U) {
        return Status::error(ErrorCode::kInvalidArgument, "max_frame_bytes is outside the allowed range");
    }
    if (watchdog_timeout_ms < 100 || watchdog_timeout_ms > 150) {
        return Status::error(ErrorCode::kInvalidArgument, "watchdog_timeout_ms must be within [100, 150]");
    }
    if (command_ttl_ms <= 0 || command_ttl_ms > watchdog_timeout_ms) {
        return Status::error(ErrorCode::kInvalidArgument,
                             "command_ttl_ms must be positive and no longer than watchdog");
    }
    if (process_period_ms <= 0 || process_period_ms > 20 || status_period_ms <= 0) {
        return Status::error(ErrorCode::kInvalidArgument, "timer periods are invalid");
    }
    if (shadow_command_topic.empty() || production_command_topic.empty() ||
        shadow_command_topic == production_command_topic) {
        return Status::error(ErrorCode::kInvalidArgument, "shadow and production command topics must be distinct");
    }
    if (productionOutputEnabled() && (run_context_topic.empty() || acquire_owner_service.empty() ||
                                      renew_owner_service.empty() || release_owner_service.empty())) {
        return Status::error(ErrorCode::kInvalidArgument, "cpp backend owner and run context endpoints are required");
    }
    if (legacyMpcOutputEnabled() && (legacy_reference_pose_topic.empty() || legacy_control_name.empty() ||
                                     legacy_left_gripper_topic.empty() || legacy_right_gripper_topic.empty())) {
        return Status::error(ErrorCode::kInvalidArgument, "legacy_mpc backend endpoints and control name are required");
    }
    if (legacyMpcOutputEnabled() && legacy_control_name != "wbmpc_remote_ctrl") {
        return Status::error(ErrorCode::kInvalidArgument,
                             "legacy_mpc backend must use wbmpc_remote_ctrl so the remote watchdog remains active");
    }
    if (report_teleop_status_service.empty()) {
        return Status::error(ErrorCode::kInvalidArgument, "Teleop status report service is required");
    }
    if (backend != "disabled" && (robot_ee_pose_topic.empty() || robot_base_frame.empty())) {
        return Status::error(ErrorCode::kInvalidArgument, "robot end-effector pose topic and base frame are required");
    }
    if (robot_pose_max_age_ms <= 0 || robot_pose_max_age_ms > 1000) {
        return Status::error(ErrorCode::kInvalidArgument, "robot_pose_max_age_ms must be within [1, 1000]");
    }
    if (owner_ttl_ms < 100 || owner_ttl_ms > 500 || owner_renew_period_ms <= 0 || owner_service_timeout_ms <= 0 ||
        owner_renew_period_ms + owner_service_timeout_ms >= owner_ttl_ms) {
        return Status::error(ErrorCode::kInvalidArgument, "owner lease timing does not preserve a fail-closed margin");
    }
    if (status_report_service_timeout_ms < 10 || status_report_service_timeout_ms > 5000 ||
        status_report_retry_period_ms < status_report_service_timeout_ms || status_report_retry_period_ms > 60000 ||
        status_report_poll_period_ms <= 0 || status_report_poll_period_ms > status_report_service_timeout_ms) {
        return Status::error(ErrorCode::kInvalidArgument, "Teleop status report timing is invalid");
    }
    if (max_frame_age_ms <= 0 || max_frame_age_ms > command_ttl_ms) {
        return Status::error(ErrorCode::kInvalidArgument, "frame age must be positive and no longer than command TTL");
    }
    if (max_future_skew_ms < 0) {
        return Status::error(ErrorCode::kInvalidArgument, "max_future_skew_ms must not be negative");
    }
    if (!std::isfinite(deadman_grip_threshold) || deadman_grip_threshold < 0.0 || deadman_grip_threshold > 1.0) {
        return Status::error(ErrorCode::kInvalidArgument, "deadman_grip_threshold must be within [0, 1]");
    }
    if (!std::isfinite(gripper_toggle_low_threshold) || !std::isfinite(gripper_binary_threshold) ||
        !std::isfinite(gripper_toggle_high_threshold) || gripper_toggle_low_threshold < 0.0 ||
        gripper_toggle_low_threshold >= gripper_binary_threshold ||
        gripper_binary_threshold >= gripper_toggle_high_threshold || gripper_toggle_high_threshold > 1.0) {
        return Status::error(ErrorCode::kInvalidArgument, "gripper toggle thresholds are invalid");
    }
    if (!std::isfinite(quaternion_norm_tolerance) || quaternion_norm_tolerance <= 0.0 ||
        quaternion_norm_tolerance >= 0.5) {
        return Status::error(ErrorCode::kInvalidArgument, "quaternion_norm_tolerance is invalid");
    }
    if (!(workspace_min_x < workspace_max_x && workspace_min_y < workspace_max_y &&
          workspace_min_z < workspace_max_z)) {
        return Status::error(ErrorCode::kInvalidArgument, "workspace minimum must be less than maximum");
    }
    if (!std::isfinite(max_position_step_m) || max_position_step_m <= 0.0 ||
        !std::isfinite(max_position_velocity_mps) || max_position_velocity_mps <= 0.0 ||
        !std::isfinite(max_position_acceleration_mps2) || max_position_acceleration_mps2 <= 0.0 ||
        !std::isfinite(max_chassis_linear_mps) || max_chassis_linear_mps < 0.0 ||
        !std::isfinite(max_chassis_angular_rps) || max_chassis_angular_rps < 0.0) {
        return Status::error(ErrorCode::kInvalidArgument, "motion limits are invalid");
    }

    std::unordered_set<std::string> key_ids;
    for (const auto &key : grant_public_keys) {
        if (key.key_id.empty() || key.public_key.empty()) {
            return Status::error(ErrorCode::kInvalidArgument, "grant key id and public key must not be empty");
        }
        if (!key_ids.insert(key.key_id).second) {
            return Status::error(ErrorCode::kConflict, "duplicate grant key id");
        }
    }
    return Status::success();
}

bool TeleopConfig::shadowOutputEnabled() const {
    return backend == "shadow";
}

bool TeleopConfig::productionOutputEnabled() const {
    return backend == "cpp";
}

bool TeleopConfig::legacyMpcOutputEnabled() const {
    return backend == "legacy_mpc";
}

bool TeleopConfig::controlOutputEnabled() const {
    return productionOutputEnabled() || legacyMpcOutputEnabled();
}

}  // namespace astrabot::teleop
