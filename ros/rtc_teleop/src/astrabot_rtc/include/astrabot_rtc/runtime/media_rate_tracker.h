#pragma once

#include <cstdint>

namespace astrabot::rtc::runtime {

/**
 * @brief RTC 媒体路径的单调累计计数器快照。
 */
struct MediaCumulativeCounters {
    std::uint64_t encoded_frames{0U};
    std::uint64_t encoded_bytes{0U};
    std::uint64_t peer_sends{0U};
    std::uint64_t peer_bytes{0U};
};

/**
 * @brief 两次 diagnostics 采样之间的应用层媒体速率。
 *
 * peer_send 指每个 peer/track 的成功发送；同一编码帧扇出给两个 peer 会计为两次发送和两份 payload 字节。
 */
struct MediaRateWindow {
    bool available{false};
    std::uint64_t duration_ms{0U};
    double encoded_fps{0.0};
    std::uint64_t encoded_bitrate_bps{0U};
    double peer_send_fps{0.0};
    std::uint64_t peer_send_bitrate_bps{0U};
};

/**
 * @brief 用单调时间和累计计数器生成最近 diagnostics 窗口速率。
 *
 * 首次采样、时间未前进或累计计数器回退时只重建基线并返回 unavailable。调用方必须串行调用。
 */
class MediaRateTracker final {
  public:
    /**
     * @brief 更新采样并返回上一采样到当前采样的速率窗口。
     */
    MediaRateWindow update(std::uint64_t steady_time_ns, const MediaCumulativeCounters &counters);

    /**
     * @brief 清除采样基线。
     */
    void reset();

  private:
    bool initialized_{false};
    std::uint64_t previous_steady_time_ns_{0U};
    MediaCumulativeCounters previous_counters_;
};

}  // namespace astrabot::rtc::runtime
