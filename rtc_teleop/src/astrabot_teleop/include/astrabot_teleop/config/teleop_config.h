// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "astrabot_teleop/common/status.h"

namespace astrabot::teleop {

/**
 * @brief Teleop 公钥配置，支持同一时间配置多把 key 进行轮换。
 */
struct GrantPublicKeyConfig {
    std::string key_id;
    std::string public_key;
};

/**
 * @brief 端侧 Teleop 运行配置。
 */
struct TeleopConfig {
    std::string backend{"shadow"};
    std::string device_id;
    std::string resource_id{"thor"};
    std::string rtc_data_topic{"/astrabot/rtc/data_channel/received"};
    std::string rtc_peer_event_topic{"/astrabot/rtc/peer_event"};
    std::string authorize_service{"/astrabot/teleop/authorize_channel"};
    std::string rtc_close_service{"/astrabot/rtc/close_peer"};
    std::string shadow_command_topic{"/astrabot/teleop/shadow_command"};
    std::string production_command_topic{"/astrabot/teleop/command"};
    std::string legacy_reference_pose_topic{"/reference/pose"};
    std::string legacy_control_name{"wbmpc_remote_ctrl"};
    std::string legacy_left_gripper_topic{"/rm_left/rm_driver/teleop_gripper_float"};
    std::string legacy_right_gripper_topic{"/rm_right/rm_driver/teleop_gripper_float"};
    std::string status_topic{"/astrabot/teleop/session_status"};
    std::string run_context_topic{"/astrabot/data_collection/run_context"};
    std::string acquire_owner_service{"/astrabot/arbitration/acquire_owner"};
    std::string renew_owner_service{"/astrabot/arbitration/renew_owner"};
    std::string release_owner_service{"/astrabot/arbitration/release_owner"};
    std::string report_teleop_status_service{"/astrabot/data_collection/report_teleop_status"};
    std::string robot_ee_pose_topic{"/ee/pose"};
    std::string robot_base_frame{"base_link"};
    std::vector<GrantPublicKeyConfig> grant_public_keys;
    std::size_t max_frame_bytes{16384};
    std::int64_t max_frame_age_ms{100};
    std::int64_t max_future_skew_ms{1000};
    std::int64_t watchdog_timeout_ms{120};
    std::int64_t command_ttl_ms{100};
    std::int64_t process_period_ms{2};
    std::int64_t status_period_ms{1000};
    std::int64_t owner_ttl_ms{150};
    std::int64_t owner_renew_period_ms{50};
    std::int64_t owner_service_timeout_ms{40};
    std::int64_t status_report_service_timeout_ms{100};
    std::int64_t status_report_retry_period_ms{1000};
    std::int64_t status_report_poll_period_ms{10};
    std::int64_t robot_pose_max_age_ms{100};
    double deadman_grip_threshold{0.5};
    double gripper_toggle_high_threshold{0.9};
    double gripper_toggle_low_threshold{0.1};
    double gripper_binary_threshold{0.4};
    double quaternion_norm_tolerance{0.05};
    double workspace_min_x{-5.0};
    double workspace_max_x{5.0};
    double workspace_min_y{-5.0};
    double workspace_max_y{5.0};
    double workspace_min_z{-5.0};
    double workspace_max_z{5.0};
    double max_position_step_m{0.25};
    double max_position_velocity_mps{1.0};
    double max_position_acceleration_mps2{5.0};
    double max_chassis_linear_mps{0.3};
    double max_chassis_angular_rps{0.5};

    /** @brief 校验安全边界和必填身份配置。 */
    Status validate() const;

    /** @brief 当前配置是否只允许发布 shadow command。 */
    bool shadowOutputEnabled() const;

    /** @brief 当前配置是否允许经过 owner lease 门控的生产 command。 */
    bool productionOutputEnabled() const;

    /** @brief 当前配置是否启用旧 `/reference/pose` 与夹爪 topic 兼容输出。 */
    bool legacyMpcOutputEnabled() const;

    /** @brief 当前配置是否会产生真实控制输出。 */
    bool controlOutputEnabled() const;
};

}  // namespace astrabot::teleop
