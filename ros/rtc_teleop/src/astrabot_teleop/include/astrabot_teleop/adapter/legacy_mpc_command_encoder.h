// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#pragma once

#include <string>

#include "astrabot_teleop/common/result.hpp"
#include "astrabot_teleop/mapping/command_mapper.h"

namespace astrabot::teleop {

/**
 * @brief 将已通过 Teleop 安全链的双臂目标编码为旧 MPC `/reference/pose` JSON。
 *
 * 本类只负责兼容协议转换，不管理 ROS publisher、控制权或 watchdog。底盘、头部和灵巧手在该兼容链路中默认关闭。
 */
class LegacyMpcCommandEncoder {
  public:
    /**
     * @brief 编码有效 deadman 下的双臂目标。
     *
     * @param command 已映射到机器人 base frame 的命令。
     * @param control_name 旧 arbitration 识别的控制源名称。
     * @return 至少一只手臂有效时返回 JSON，否则返回错误。
     */
    Result<std::string> encodeCommand(const MappedCommand &command, const std::string &control_name) const;

    /**
     * @brief 编码机器人当前双臂位姿，用于 deadman 释放或 session 异常时立即 hold。
     */
    Result<std::string> encodeHold(const PoseSample &right_arm, const PoseSample &left_arm,
                                   const std::string &control_name) const;

  private:
    Result<std::string> encode(const PoseSample *right_arm, const PoseSample *left_arm,
                               const std::string &control_name) const;
};

}  // namespace astrabot::teleop
