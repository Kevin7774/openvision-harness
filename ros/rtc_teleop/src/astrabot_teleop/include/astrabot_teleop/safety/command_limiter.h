// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>

#include "astrabot_teleop/common/result.hpp"
#include "astrabot_teleop/protocol/teleop_frame_model.h"

namespace astrabot::teleop {

/** @brief 机器人 base frame 命令的 workspace、步长、速度和加速度边界。 */
struct CommandLimitConfig {
    double min_x{-5.0};
    double max_x{5.0};
    double min_y{-5.0};
    double max_y{5.0};
    double min_z{-5.0};
    double max_z{5.0};
    double max_position_step_m{0.25};
    double max_position_velocity_mps{1.0};
    double max_position_acceleration_mps2{5.0};
};

/**
 * @brief 在相对位姿完成机器人原点映射后拒绝超 workspace、步长或时间型运动边界的控制位姿。
 */
class CommandLimiter {
  public:
    explicit CommandLimiter(CommandLimitConfig config);

    /**
     * @brief 校验并原子提交本帧运动基线。
     *
     * @param frame 已完成协议与数值校验的 Teleop 帧。
     * @param sample_steady_time_ns RTC 收包时的本机 steady clock 纳秒时间戳。
     */
    Status validateAndCommit(const DecodedTeleopFrame &frame, std::uint64_t sample_steady_time_ns);

    /** @brief 新 session 开始或关闭时清空位置基线。 */
    void reset();

    /** @brief deadman 重新按下并捕获新原点时，仅清空对应手臂的历史基线。 */
    void resetArmHistories(bool right_arm, bool left_arm);

  private:
    struct PoseHistory {
        PoseSample pose;
        std::uint64_t steady_time_ns{0};
        double velocity_x_mps{0.0};
        double velocity_y_mps{0.0};
        double velocity_z_mps{0.0};
        bool has_pose{false};
        bool has_velocity{false};
    };

    Result<PoseHistory> validatedPoseHistory(const PoseSample &pose, bool valid, const PoseHistory &previous,
                                             std::uint64_t sample_steady_time_ns) const;

    CommandLimitConfig config_;
    PoseHistory right_history_;
    PoseHistory left_history_;
    PoseHistory head_history_;
};

}  // namespace astrabot::teleop
