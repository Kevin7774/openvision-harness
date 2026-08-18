// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include "astrabot_teleop/common/status.h"

namespace astrabot::teleop {

/** @brief 已授权的唯一 Teleop writer 绑定。 */
struct SessionBinding {
    std::string session_id;
    std::string peer_id;
    std::string run_id;
    std::string resource_id;
    std::string channel_label;
    /** @brief RTC 授权截止点，机器人本机 steady clock 纳秒。 */
    std::uint64_t authorization_deadline_steady_ns{0};
    /** @brief 仅用于识别当前活动授权的完全相同 token 重试；不得发布或记录。 */
    std::string grant_fingerprint;
    bool connected{false};
};

/**
 * @brief 在端侧维持单 writer 的 run/session/peer/channel 隔离。
 */
class SessionRegistry {
  public:
    /**
     * @brief 注册唯一 writer；已有不同 writer 时 fail closed。
     * @param binding 待注册的完整 writer 绑定。
     * @param allow_empty_run_id 仅供不产生生产运动输出的 shadow 通用遥操使用。
     */
    Status authorize(SessionBinding binding, bool allow_empty_run_id);

    /** @brief 标记与当前 binding 完全一致的 peer 已连接。 */
    /**
     * @brief 在 RTC 确认目标 DataChannel 实际 open 后，将授权 writer 标记为已连接。
     */
    Status markConnected(const std::string &session_id, const std::string &peer_id, const std::string &channel_label);

    /** @brief 判断高频 packet 是否来自当前已连接 writer。 */
    bool matches(const std::string &session_id, const std::string &peer_id, const std::string &channel_label) const;

    /** @brief 判断候选授权是否是当前 grant 与完整 writer binding 的幂等重试。 */
    bool matchesAuthorization(const SessionBinding &binding) const;

    /** @brief 返回当前 binding 的不可变快照。 */
    std::optional<SessionBinding> current() const;

    /** @brief 关闭指定 session；不允许错误 session 清除当前 writer。 */
    Status close(const std::string &session_id);

    /** @brief 无条件清空本地 binding，仅用于进程 stop/fault 收尾。 */
    void clear();

  private:
    mutable std::mutex mutex_;
    std::optional<SessionBinding> current_;
};

}  // namespace astrabot::teleop
