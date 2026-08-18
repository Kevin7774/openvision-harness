#pragma once

#include <functional>
#include <string>

#include "astrabot_rtc/common/status.h"

namespace astrabot::rtc::signaling {

/**
 * @brief Gateway 信令 ROS topic 与 RTC runtime 之间的最小抽象。
 */
class ISignalingAdapter {
  public:
    using CommandHandler = std::function<Status(const std::string &)>;

    virtual ~ISignalingAdapter() = default;

    /**
     * @brief 在 start 前设置下行信令处理器。
     */
    virtual Status setCommandHandler(CommandHandler handler) = 0;

    /**
     * @brief 启动 ROS 信令收发；重复调用成功。
     */
    virtual Status start() = 0;

    /**
     * @brief 停止 ROS 信令收发并注销 callback。
     */
    virtual void stop() = 0;

    /**
     * @brief 发布 transport 生成的原始上行信令 envelope。
     */
    virtual Status publishReport(const std::string &payload) = 0;
};

}  // namespace astrabot::rtc::signaling
