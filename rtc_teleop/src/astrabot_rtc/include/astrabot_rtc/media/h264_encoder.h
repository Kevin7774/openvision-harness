#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "astrabot_rtc/common/result.hpp"
#include "astrabot_rtc/media/video_frame.h"

namespace astrabot::rtc::media {

/**
 * @brief H.264 encoder 的稳定运行参数。
 *
 * encoder_name 只指定一个 FFmpeg encoder；实现不得在失败后换用另一 encoder。primary_frame_rate 启动失败时，
 * 只允许对同一 encoder 尝试 fallback_frame_rate。
 */
struct H264EncoderSettings {
    std::string encoder_name;
    bool require_hardware{true};
    std::uint32_t output_width{640U};
    std::uint32_t output_height{480U};
    std::uint32_t primary_frame_rate{60U};
    std::uint32_t fallback_frame_rate{30U};
    std::uint64_t bitrate_bps{4000000U};
    std::uint32_t gop_size_frames{60U};
    std::size_t max_encoded_frame_bytes{2U * 1024U * 1024U};
    std::uint32_t max_encoder_surfaces{2U};
    std::string pixel_format{"nv12"};
    std::string preset{"p1"};
    std::string tune{"ull"};
    std::string profile{"baseline"};
    std::string level{"3.1"};
};

/**
 * @brief 隔离具体编码 SDK 的同步单 track H.264 encoder 接口。
 *
 * start/stop 由控制线程串行调用，encode 只由所属 worker 线程调用。输出必须是 Annex-B access unit。
 */
class IH264Encoder {
  public:
    virtual ~IH264Encoder() = default;

    /**
     * @brief 分配 encoder 资源；重复 start 必须幂等。
     */
    virtual Status start() = 0;

    /**
     * @brief 立即释放 encoder 资源；重复 stop 必须幂等。
     */
    virtual void stop() = 0;

    /**
     * @brief 编码一帧；允许低延迟 encoder 暂时返回空集合，但内部缓存必须有界。
     */
    virtual Result<std::vector<std::shared_ptr<const EncodedVideoFrame>>> encode(const RawVideoFrame &frame) = 0;

    /**
     * @brief 返回实际打开的帧率；未启动时返回 0。
     */
    virtual std::uint32_t activeFrameRate() const = 0;

    /**
     * @brief 返回显式选择的 FFmpeg encoder 名称。
     */
    virtual const std::string &encoderName() const = 0;
};

}  // namespace astrabot::rtc::media
