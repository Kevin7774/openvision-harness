// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>

#include "astrabot_teleop/common/result.hpp"

namespace astrabot::teleop {

/**
 * @brief 将 grant 的 Unix epoch 秒过期点转换为机器人本机 steady clock 纳秒截止点。
 *
 * @param expires_at_epoch_sec grant 中的绝对 Unix epoch 秒。
 * @param system_now_epoch_ns 同一次授权采样得到的 system clock Unix epoch 纳秒。
 * @param steady_now_ns 与 system_now_epoch_ns 紧邻采样的 steady clock 纳秒。
 * @return RTC 可直接比较的 steady clock 绝对截止点；过期或算术溢出时 fail closed。
 *
 * 转换只在授权边界执行一次。后续 RTC 只比较 steady clock，系统时钟跳变不会延长或缩短已授权通道。
 */
Result<std::uint64_t> grantExpiryToSteadyDeadline(std::uint64_t expires_at_epoch_sec, std::uint64_t system_now_epoch_ns,
                                                  std::uint64_t steady_now_ns);

}  // namespace astrabot::teleop
