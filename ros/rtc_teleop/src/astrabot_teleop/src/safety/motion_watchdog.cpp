// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include "astrabot_teleop/safety/motion_watchdog.h"

namespace astrabot::teleop {

MotionWatchdog::MotionWatchdog(const std::uint64_t timeout_ns) : timeout_ns_(timeout_ns) {}

void MotionWatchdog::observe(const std::uint64_t steady_time_ns) {
    last_observed_time_ns_ = steady_time_ns;
    observed_ = true;
}

bool MotionWatchdog::expired(const std::uint64_t steady_now_ns) const {
    return observed_ && steady_now_ns >= last_observed_time_ns_ && steady_now_ns - last_observed_time_ns_ > timeout_ns_;
}

void MotionWatchdog::reset() {
    last_observed_time_ns_ = 0;
    observed_ = false;
}

std::uint64_t MotionWatchdog::lastObservedTimeNs() const {
    return last_observed_time_ns_;
}

}  // namespace astrabot::teleop
