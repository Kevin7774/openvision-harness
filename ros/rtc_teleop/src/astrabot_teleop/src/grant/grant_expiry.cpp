// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include "astrabot_teleop/grant/grant_expiry.h"

#include <limits>

namespace astrabot::teleop {
namespace {

constexpr std::uint64_t kNanosecondsPerSecond = 1000000000U;

}  // namespace

Result<std::uint64_t> grantExpiryToSteadyDeadline(const std::uint64_t expires_at_epoch_sec,
                                                  const std::uint64_t system_now_epoch_ns,
                                                  const std::uint64_t steady_now_ns) {
    const std::uint64_t system_now_epoch_sec = system_now_epoch_ns / kNanosecondsPerSecond;
    if (expires_at_epoch_sec <= system_now_epoch_sec) {
        return Result<std::uint64_t>::failure(Status::error(ErrorCode::kDeadlineExceeded, "grant has expired"));
    }

    const std::uint64_t remaining_whole_seconds = expires_at_epoch_sec - system_now_epoch_sec;
    if (remaining_whole_seconds > std::numeric_limits<std::uint64_t>::max() / kNanosecondsPerSecond) {
        return Result<std::uint64_t>::failure(
            Status::error(ErrorCode::kInvalidArgument, "grant expiry is outside the supported clock range"));
    }
    const std::uint64_t current_fraction_ns = system_now_epoch_ns % kNanosecondsPerSecond;
    const std::uint64_t remaining_ns = remaining_whole_seconds * kNanosecondsPerSecond - current_fraction_ns;
    if (steady_now_ns > std::numeric_limits<std::uint64_t>::max() - remaining_ns) {
        return Result<std::uint64_t>::failure(
            Status::error(ErrorCode::kInvalidArgument, "grant steady deadline would overflow"));
    }
    return Result<std::uint64_t>::success(steady_now_ns + remaining_ns);
}

}  // namespace astrabot::teleop
