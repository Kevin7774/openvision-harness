#pragma once

#include <memory>

#include "astrabot_rtc/media/h264_encoder.h"

namespace astrabot::rtc::media {

/**
 * @brief 使用 FFmpeg libavcodec/libswscale 将 ROS 原始图像转换为低延迟 Annex-B H.264。
 *
 * FFmpeg 类型全部隐藏在 Implementation 中。require_hardware=true 时只接受带 AV_CODEC_CAP_HARDWARE 的 encoder，
 * 不会隐式回退到 libx264 等软件实现。
 */
class FfmpegH264Encoder final : public IH264Encoder {
  public:
    explicit FfmpegH264Encoder(H264EncoderSettings settings);
    ~FfmpegH264Encoder() override;

    Status start() override;
    void stop() override;
    Result<std::vector<std::shared_ptr<const EncodedVideoFrame>>> encode(const RawVideoFrame &frame) override;
    std::uint32_t activeFrameRate() const override;
    const std::string &encoderName() const override;

  private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace astrabot::rtc::media
