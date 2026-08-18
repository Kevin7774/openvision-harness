#pragma once

#include <cstddef>
#include <deque>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "astrabot_rtc/common/status.h"
#include "astrabot_rtc/session/session_registry.h"

namespace astrabot::rtc::transport {

/**
 * @brief 记录有界 peer binding 的成功重连语义。
 *
 * 首次 Connected 不算重连。已经连接过的 binding 在 Disconnected/Failed 后再次 Connected，或在失联后销毁并以同一
 * binding 新建成功，才返回一次重连事件。调用方必须在同一锁下串行调用全部方法。
 */
class PeerReconnectTracker final {
  public:
    /**
     * @brief 创建 tracker；活动 binding 和待重连历史都以 max_bindings 为硬上限。
     */
    explicit PeerReconnectTracker(std::size_t max_bindings);

    /**
     * @brief 注册一个新 PeerConnection binding。
     */
    Status registerBinding(const std::string &binding_key);

    /**
     * @brief 注销活动 binding；只有已经观察到失联时才保留待重连资格。
     */
    void unregisterBinding(const std::string &binding_key);

    /**
     * @brief 观察连接状态；本次状态构成成功重连时返回 true。
     */
    bool observeState(const std::string &binding_key, session::PeerState state);

    /**
     * @brief 清空活动 binding 和待重连历史。
     */
    void clear();

  private:
    struct BindingState {
        bool ever_connected{false};
        bool reconnect_pending{false};
    };

    void rememberEligible(const std::string &binding_key);
    void forgetEligible(const std::string &binding_key);

    const std::size_t max_bindings_;
    std::unordered_map<std::string, BindingState> active_bindings_;
    std::unordered_set<std::string> eligible_bindings_;
    std::deque<std::string> eligible_order_;
};

}  // namespace astrabot::rtc::transport
