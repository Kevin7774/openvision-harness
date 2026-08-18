#include "astrabot_rtc/runtime/media_rate_tracker.h"

#include <limits>

namespace astrabot::rtc::runtime {
namespace {

constexpr long double kNanosecondsPerSecond = 1000000000.0L;
constexpr std::uint64_t kNanosecondsPerMillisecond = 1000000U;

bool countersRegressed(const MediaCumulativeCounters &current, const MediaCumulativeCounters &previous) {
    return current.encoded_frames < previous.encoded_frames || current.encoded_bytes < previous.encoded_bytes ||
           current.peer_sends < previous.peer_sends || current.peer_bytes < previous.peer_bytes;
}

double framesPerSecond(std::uint64_t frames, std::uint64_t elapsed_ns) {
    return static_cast<double>(static_cast<long double>(frames) * kNanosecondsPerSecond /
                               static_cast<long double>(elapsed_ns));
}

std::uint64_t bitsPerSecond(std::uint64_t bytes, std::uint64_t elapsed_ns) {
    const long double rate =
        static_cast<long double>(bytes) * 8.0L * kNanosecondsPerSecond / static_cast<long double>(elapsed_ns);
    if (rate >= static_cast<long double>(std::numeric_limits<std::uint64_t>::max())) {
        return std::numeric_limits<std::uint64_t>::max();
    }
    return static_cast<std::uint64_t>(rate);
}

}  // namespace

MediaRateWindow MediaRateTracker::update(std::uint64_t steady_time_ns, const MediaCumulativeCounters &counters) {
    if (!initialized_ || steady_time_ns <= previous_steady_time_ns_ ||
        countersRegressed(counters, previous_counters_)) {
        initialized_ = true;
        previous_steady_time_ns_ = steady_time_ns;
        previous_counters_ = counters;
        return {};
    }

    const std::uint64_t elapsed_ns = steady_time_ns - previous_steady_time_ns_;
    MediaRateWindow window;
    window.available = true;
    window.duration_ms = elapsed_ns / kNanosecondsPerMillisecond;
    window.encoded_fps = framesPerSecond(counters.encoded_frames - previous_counters_.encoded_frames, elapsed_ns);
    window.encoded_bitrate_bps = bitsPerSecond(counters.encoded_bytes - previous_counters_.encoded_bytes, elapsed_ns);
    window.peer_send_fps = framesPerSecond(counters.peer_sends - previous_counters_.peer_sends, elapsed_ns);
    window.peer_send_bitrate_bps = bitsPerSecond(counters.peer_bytes - previous_counters_.peer_bytes, elapsed_ns);

    previous_steady_time_ns_ = steady_time_ns;
    previous_counters_ = counters;
    return window;
}

void MediaRateTracker::reset() {
    initialized_ = false;
    previous_steady_time_ns_ = 0U;
    previous_counters_ = {};
}

}  // namespace astrabot::rtc::runtime
