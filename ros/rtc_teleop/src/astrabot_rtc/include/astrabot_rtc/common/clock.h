#pragma once

#include <cstdint>

namespace astrabot::rtc {

/**
 * @brief 为授权有效期和测试提供可注入的单调时钟边界。
 */
class IClock {
  public:
    virtual ~IClock() = default;

    /**
     * @brief 返回本机 steady clock 纳秒时间。
     */
    virtual std::uint64_t nowNanoseconds() const = 0;
};

/**
 * @brief 使用 steady_clock 的生产实现，系统墙钟跳变不会改变已授权 channel TTL。
 */
class SteadyClock final : public IClock {
  public:
    std::uint64_t nowNanoseconds() const override;
};

}  // namespace astrabot::rtc
