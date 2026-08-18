#include "astrabot_rtc/common/clock.h"

#include <chrono>

namespace astrabot::rtc {

std::uint64_t SteadyClock::nowNanoseconds() const {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

}  // namespace astrabot::rtc
