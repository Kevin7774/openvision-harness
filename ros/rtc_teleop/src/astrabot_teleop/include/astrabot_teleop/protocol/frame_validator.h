// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <cstdint>

#include "astrabot_teleop/common/result.hpp"
#include "astrabot_teleop/protocol/teleop_frame_model.h"

namespace astrabot::teleop {

/** @brief TeleopFrame 时间、序列和数学输入校验配置。 */
struct FrameValidationConfig {
    std::int64_t max_frame_age_ms{100};
    std::int64_t max_future_skew_ms{1000};
    double quaternion_norm_tolerance{0.05};
    std::size_t expected_hand_joint_count{26};
};

/** @brief 本次帧相对上一帧产生的序列统计。 */
struct FrameValidationOutcome {
    std::uint64_t sequence_gap{0};
};

/**
 * @brief 单 session 的 TeleopFrame replay、timestamp 和数值校验器。
 */
class FrameValidator {
  public:
    explicit FrameValidator(FrameValidationConfig config);

    /** @brief 在不更新 replay 状态的情况下校验一帧。 */
    Result<FrameValidationOutcome> validate(const DecodedTeleopFrame &frame, std::int64_t system_now_ms) const;

    /** @brief 在完整安全链通过后提交序列和 timestamp。 */
    void commit(const DecodedTeleopFrame &frame, const FrameValidationOutcome &outcome);

    /** @brief 新 session 开始或关闭时清空 replay 状态。 */
    void reset();

    /** @brief 返回最后已接受 sequence。 */
    std::uint64_t lastSequence() const;

    /** @brief 返回累计 sequence gap。 */
    std::uint64_t sequenceGapCount() const;

  private:
    FrameValidationConfig config_;
    bool has_previous_{false};
    std::uint64_t last_sequence_{0};
    std::int64_t last_timestamp_ms_{0};
    std::uint64_t sequence_gap_count_{0};
};

}  // namespace astrabot::teleop
