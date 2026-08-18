// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <cstdint>

namespace astrabot::teleop {

/** @brief 与 WebRTC SDK 和 protobuf 生成类型解耦的位姿样本。 */
struct PoseSample {
    bool present{false};
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double qx{0.0};
    double qy{0.0};
    double qz{0.0};
    double qw{1.0};
};

/** @brief Quest 控制器模拟量。 */
struct AxisSample {
    bool present{false};
    double trigger_right{0.0};
    double trigger_left{0.0};
    double grip_right{0.0};
    double grip_left{0.0};
    double joystick_right_x{0.0};
    double joystick_right_y{0.0};
    double joystick_left_x{0.0};
    double joystick_left_y{0.0};
};

/** @brief TeleopFrame 中的兼容 CRC 编码来源。 */
enum class CrcVariant {
    kDataBody,
    kQuestLegacyEnvelope,
};

/**
 * @brief 从 frozen TeleopFrame wire schema 解码出的领域值。
 */
struct DecodedTeleopFrame {
    std::uint64_t sequence{0};
    std::int64_t timestamp_ms{0};
    std::uint32_t crc32_checksum{0};
    CrcVariant crc_variant{CrcVariant::kDataBody};
    bool pose_valid_right{false};
    bool pose_valid_left{false};
    bool pose_valid_head{false};
    bool hand_skeleton_valid_right{false};
    bool hand_skeleton_valid_left{false};
    PoseSample action_right;
    PoseSample action_left;
    PoseSample action_head;
    PoseSample calibration_right;
    PoseSample calibration_left;
    PoseSample calibration_head;
    AxisSample axes;
    bool safety_present{false};
    bool safety_right{false};
    bool safety_left{false};
    bool safety_head{false};
    std::size_t hand_skeleton_right_joint_count{0};
    std::size_t hand_skeleton_left_joint_count{0};
    bool all_values_finite{true};
};

}  // namespace astrabot::teleop
