// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <cstdint>

namespace astrabot::teleop {

/** @brief IEEE CRC32 计算器。 */
class Crc32 {
  public:
    /** @brief 计算与 Quest/平台契约一致的 IEEE CRC32。 */
    static std::uint32_t compute(const std::uint8_t *data, std::size_t size);
};

}  // namespace astrabot::teleop
