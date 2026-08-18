// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>

namespace astrabot::teleop {

/**
 * @brief 基于 steady clock 的 Teleop 运动输入 watchdog。
 */
class MotionWatchdog {
  public:
    explicit MotionWatchdog(std::uint64_t timeout_ns);

    /** @brief 记录最后一帧完整通过安全链的 steady timestamp。 */
    void observe(std::uint64_t steady_time_ns);

    /** @brief 判断当前时刻是否已超过安全 timeout。 */
    bool expired(std::uint64_t steady_now_ns) const;

    /** @brief 清空观测状态。 */
    void reset();

    /** @brief 返回最后有效帧 steady timestamp。 */
    std::uint64_t lastObservedTimeNs() const;

  private:
    std::uint64_t timeout_ns_{0};
    std::uint64_t last_observed_time_ns_{0};
    bool observed_{false};
};

}  // namespace astrabot::teleop
