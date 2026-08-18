#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "astrabot_rtc/media/ffmpeg_h264_encoder.h"

namespace astrabot::rtc::media {
namespace {

H264EncoderSettings softwareTestSettings() {
    H264EncoderSettings settings;
    settings.encoder_name = "libx264";
    settings.require_hardware = false;
    settings.output_width = 64U;
    settings.output_height = 48U;
    settings.primary_frame_rate = 30U;
    settings.fallback_frame_rate = 30U;
    settings.bitrate_bps = 500000U;
    settings.gop_size_frames = 30U;
    settings.max_encoded_frame_bytes = 256U * 1024U;
    settings.max_encoder_surfaces = 1U;
    settings.pixel_format = "nv12";
    settings.preset = "ultrafast";
    settings.tune = "zerolatency";
    settings.profile = "baseline";
    settings.level = "3.1";
    return settings;
}

RawVideoFrame makeBgrFrame(std::uint64_t capture_time_ns) {
    constexpr std::uint32_t kWidth = 64U;
    constexpr std::uint32_t kHeight = 48U;
    auto data = std::make_shared<std::vector<std::uint8_t>>(kWidth * kHeight * 3U);
    for (std::size_t index = 0U; index < data->size(); ++index) {
        (*data)[index] = static_cast<std::uint8_t>(index % 251U);
    }
    RawVideoFrame frame;
    frame.track_id = "right_eye";
    frame.capture_time_ns = capture_time_ns;
    frame.width = kWidth;
    frame.height = kHeight;
    frame.row_step = kWidth * 3U;
    frame.encoding = "bgr8";
    frame.data = std::move(data);
    return frame;
}

bool hasAnnexBStartCode(const std::vector<std::uint8_t> &data) {
    return (data.size() >= 4U && data[0] == 0U && data[1] == 0U && data[2] == 0U && data[3] == 1U) ||
           (data.size() >= 3U && data[0] == 0U && data[1] == 0U && data[2] == 1U);
}

TEST(FfmpegH264EncoderTest, ExplicitSoftwareEncoderProducesBoundedAnnexBFrame) {
    FfmpegH264Encoder encoder(softwareTestSettings());
    ASSERT_TRUE(encoder.start().ok());
    EXPECT_EQ(encoder.encoderName(), "libx264");
    EXPECT_EQ(encoder.activeFrameRate(), 30U);

    std::vector<std::shared_ptr<const EncodedVideoFrame>> outputs;
    for (std::uint64_t sequence = 0U; sequence < 3U && outputs.empty(); ++sequence) {
        auto result = encoder.encode(makeBgrFrame(1000U + sequence));
        ASSERT_TRUE(result.ok()) << result.status().message();
        outputs = result.takeValue();
    }

    ASSERT_FALSE(outputs.empty());
    ASSERT_TRUE(outputs.front());
    EXPECT_EQ(outputs.front()->track_id, "right_eye");
    EXPECT_EQ(outputs.front()->codec, "h264");
    EXPECT_TRUE(outputs.front()->key_frame);
    ASSERT_TRUE(outputs.front()->data);
    EXPECT_TRUE(hasAnnexBStartCode(*outputs.front()->data));
    EXPECT_LE(outputs.front()->data->size(), softwareTestSettings().max_encoded_frame_bytes);

    encoder.stop();
    encoder.stop();
    EXPECT_EQ(encoder.activeFrameRate(), 0U);
    EXPECT_EQ(encoder.encode(makeBgrFrame(2000U)).status().code(), ErrorCode::kFailedPrecondition);
}

TEST(FfmpegH264EncoderTest, HardwarePolicyRejectsLibx264WithoutFallback) {
    auto settings = softwareTestSettings();
    settings.require_hardware = true;
    FfmpegH264Encoder encoder(std::move(settings));

    const Status status = encoder.start();
    ASSERT_FALSE(status.ok());
    EXPECT_EQ(status.code(), ErrorCode::kFailedPrecondition);
    EXPECT_EQ(encoder.activeFrameRate(), 0U);
}

TEST(FfmpegH264EncoderTest, RejectsTruncatedRawFrame) {
    FfmpegH264Encoder encoder(softwareTestSettings());
    ASSERT_TRUE(encoder.start().ok());
    auto frame = makeBgrFrame(1000U);
    frame.data = std::make_shared<const std::vector<std::uint8_t>>(16U, 0U);

    auto result = encoder.encode(frame);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.status().code(), ErrorCode::kInvalidArgument);
    encoder.stop();
}

}  // namespace
}  // namespace astrabot::rtc::media
