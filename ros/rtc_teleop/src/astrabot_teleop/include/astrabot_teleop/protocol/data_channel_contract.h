// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "astrabot_teleop/common/result.hpp"

namespace astrabot::teleop {

/** @brief Teleop DataChannel 的唯一传输约束。 */
struct DataChannelContract {
    std::string label;
    bool ordered{false};
    std::uint32_t max_packet_lifetime_ms{20};
    std::size_t max_payload_bytes{16384};
};

/** @brief Teleop DataChannel 唯一契约注册表。 */
class DataChannelContracts {
  public:
    static constexpr const char *kLabel = "astrabot.teleop";

    /** @brief 返回唯一正式 label 的契约；其他 label 默认拒绝。 */
    static Result<DataChannelContract> find(const std::string &label);
};

}  // namespace astrabot::teleop
