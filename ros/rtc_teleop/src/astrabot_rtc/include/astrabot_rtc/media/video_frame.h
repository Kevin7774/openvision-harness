#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace astrabot::rtc::media {

/**
 * @brief 与 ROS 和具体编码 SDK 解耦的原始视频帧快照。
 */
struct RawVideoFrame {
    std::string track_id;
    std::uint64_t capture_time_ns{0U};
    std::uint32_t width{0U};
    std::uint32_t height{0U};
    std::uint32_t row_step{0U};
    std::string encoding;
    std::shared_ptr<const std::vector<std::uint8_t>> data;
};

/**
 * @brief 可被多个 peer 共享的已编码视频帧。
 */
struct EncodedVideoFrame {
    std::string track_id;
    std::string codec;
    std::uint64_t capture_time_ns{0U};
    bool key_frame{false};
    std::shared_ptr<const std::vector<std::uint8_t>> data;
};

}  // namespace astrabot::rtc::media
