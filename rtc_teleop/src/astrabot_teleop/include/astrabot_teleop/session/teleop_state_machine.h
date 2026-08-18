// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <mutex>
#include <string>

#include "astrabot_teleop/common/status.h"

namespace astrabot::teleop {

/** @brief Teleop 显式会话状态。数值与 TeleopSessionStatus.msg 保持一致。 */
enum class TeleopState : std::uint8_t {
    kIdle = 0,
    kAuthorized = 1,
    kConnected = 2,
    kArmed = 3,
    kControlling = 4,
    kStopping = 5,
    kClosed = 6,
    kFault = 7,
};

/** @brief 驱动状态机的领域事件。 */
enum class TeleopEvent {
    kAuthorize,
    kPeerConnected,
    kFrameValid,
    kOwnerAcquired,
    kOwnerReleased,
    kDeadmanPressed,
    kDeadmanReleased,
    kStopRequested,
    kStopCompleted,
    kFault,
};

/**
 * @brief 使用显式转换表约束 Teleop 生命周期。
 *
 * 非法转换返回错误，绝不通过散落布尔值隐式改变是否可控制。
 */
class TeleopStateMachine {
  public:
    /** @brief 处理一个事件并执行合法状态转换。 */
    Status transition(TeleopEvent event, const std::string &reason_code);

    /** @brief 返回当前状态。 */
    TeleopState state() const;

    /** @brief 返回最后一次状态转换原因。 */
    std::string reasonCode() const;

    /** @brief 仅在没有活动 session 时将 Closed 恢复为 Idle。 */
    Status reset();

  private:
    mutable std::mutex mutex_;
    TeleopState state_{TeleopState::kIdle};
    std::string reason_code_{"startup"};
};

}  // namespace astrabot::teleop
