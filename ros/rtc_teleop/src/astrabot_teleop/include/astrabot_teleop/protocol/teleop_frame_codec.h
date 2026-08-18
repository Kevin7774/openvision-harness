// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "astrabot_teleop/common/result.hpp"
#include "astrabot_teleop/protocol/teleop_frame_model.h"

namespace astrabot::teleop {

/**
 * @brief Frozen TeleopFrame protobuf wire schema 的 C++ 解码边界。
 *
 * 生成的 protobuf 类型仅存在于实现文件，避免向状态机和 ROS runtime 泄漏生成代码。
 */
class TeleopFrameCodec {
  public:
    explicit TeleopFrameCodec(std::size_t max_frame_bytes);

    /** @brief 解码并校验 protobuf 结构、未知字段和 CRC。 */
    Result<DecodedTeleopFrame> decode(const std::vector<std::uint8_t> &payload) const;

  private:
    std::size_t max_frame_bytes_{0};
};

}  // namespace astrabot::teleop
