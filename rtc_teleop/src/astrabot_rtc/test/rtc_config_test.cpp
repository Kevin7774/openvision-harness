#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "astrabot_rtc/config/rtc_config.h"

namespace astrabot::rtc::config {
namespace {

class TemporaryConfigFile {
  public:
    explicit TemporaryConfigFile(const std::string &content) {
        const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
        path_ = std::filesystem::temp_directory_path() / ("astrabot_rtc_config_" + std::to_string(suffix) + ".yaml");
        std::ofstream output(path_);
        output << content;
    }

    ~TemporaryConfigFile() {
        std::error_code error;
        std::filesystem::remove(path_, error);
    }

    std::string path() const {
        return path_.string();
    }

  private:
    std::filesystem::path path_;
};

TEST(RtcConfigLoaderTest, EmptyPathReturnsSafeDefaults) {
    RtcConfigLoader loader;
    auto result = loader.load("");

    ASSERT_TRUE(result.ok()) << result.status().message();
    EXPECT_EQ(result.value().runtime.max_viewers, 2U);
    EXPECT_EQ(result.value().runtime.max_payload_bytes, 16384U);
    EXPECT_EQ(result.value().signaling.max_payload_bytes, 65536U);
    EXPECT_EQ(result.value().transport.backend, "disabled");
    EXPECT_TRUE(result.value().transport.bind_address.empty());
    EXPECT_EQ(result.value().transport.max_media_buffered_amount_bytes, 2097152U);
    EXPECT_FALSE(result.value().media.enabled);
    EXPECT_EQ(result.value().media.max_encoded_subscribers, 2U);
    EXPECT_EQ(result.value().media.encoder.encoder_name, "h264_nvenc");
    EXPECT_TRUE(result.value().media.encoder.require_hardware);
    EXPECT_EQ(result.value().media.encoder.output_width, 640U);
    EXPECT_EQ(result.value().media.encoder.frame_rate, 60U);
    EXPECT_EQ(result.value().media.encoder.fallback_frame_rate, 30U);
    ASSERT_EQ(result.value().media.tracks.size(), 2U);
    EXPECT_EQ(result.value().media.tracks[1].track_id, "right_eye");
}

TEST(RtcConfigLoaderTest, LoadsKnownKeysAndMediaCsvTogether) {
    TemporaryConfigFile file(R"(
topics:
  gateway_command: /custom/webrtc/cmd
runtime:
  max_viewers: 1
  max_peers: 2
  max_data_channels: 4
  max_payload_bytes: 4096
media:
  enabled: true
  max_encoded_subscribers: 1
  encoder_name: libx264
  require_hardware: false
  output_width: 640
  output_height: 480
  frame_rate: 30
  fallback_frame_rate: 30
  bitrate_bps: 1000000
  gop_size_frames: 30
  max_encoded_frame_bytes: 1048576
  max_encoder_surfaces: 1
  pixel_format: nv12
  preset: ultrafast
  tune: zerolatency
  profile: baseline
  level: 3.1
  track_ids: chest
  image_topics: /camera/chest/image
  camera_info_topics: /camera/chest/camera_info
transport:
  backend: libdatachannel
  bind_address: 192.168.123.102
  max_buffered_amount_bytes: 8192
  max_media_buffered_amount_bytes: 1048576
)");
    RtcConfigLoader loader;
    auto result = loader.load(file.path());

    ASSERT_TRUE(result.ok()) << result.status().message();
    EXPECT_EQ(result.value().topics.gateway_command, "/custom/webrtc/cmd");
    EXPECT_EQ(result.value().runtime.max_payload_bytes, 4096U);
    EXPECT_EQ(result.value().transport.backend, "libdatachannel");
    EXPECT_EQ(result.value().transport.bind_address, "192.168.123.102");
    EXPECT_EQ(result.value().transport.max_buffered_amount_bytes, 8192U);
    EXPECT_EQ(result.value().transport.max_media_buffered_amount_bytes, 1048576U);
    EXPECT_EQ(result.value().media.encoder.encoder_name, "libx264");
    EXPECT_FALSE(result.value().media.encoder.require_hardware);
    EXPECT_EQ(result.value().media.encoder.frame_rate, 30U);
    ASSERT_EQ(result.value().media.tracks.size(), 1U);
    EXPECT_EQ(result.value().media.tracks.front().track_id, "chest");
}

TEST(RtcConfigLoaderTest, RejectsUnknownKeyAndUnsafeLimits) {
    TemporaryConfigFile unknown_key("runtime:\n  mystery_limit: 1\n");
    RtcConfigLoader loader;
    auto unknown_result = loader.load(unknown_key.path());
    ASSERT_FALSE(unknown_result.ok());
    EXPECT_EQ(unknown_result.status().code(), ErrorCode::kInvalidArgument);

    TemporaryConfigFile oversized("runtime:\n  max_payload_bytes: 16385\n");
    auto oversized_result = loader.load(oversized.path());
    ASSERT_FALSE(oversized_result.ok());
    EXPECT_EQ(oversized_result.status().code(), ErrorCode::kInvalidArgument);

    TemporaryConfigFile undersized_buffer(
        "runtime:\n  max_payload_bytes: 4096\ntransport:\n  max_buffered_amount_bytes: 1024\n");
    auto buffer_result = loader.load(undersized_buffer.path());
    ASSERT_FALSE(buffer_result.ok());
    EXPECT_EQ(buffer_result.status().code(), ErrorCode::kInvalidArgument);

    TemporaryConfigFile oversized_signaling("signaling:\n  max_payload_bytes: 65537\n");
    auto signaling_result = loader.load(oversized_signaling.path());
    ASSERT_FALSE(signaling_result.ok());
    EXPECT_EQ(signaling_result.status().code(), ErrorCode::kInvalidArgument);

    TemporaryConfigFile oversized_media_buffer("transport:\n  max_media_buffered_amount_bytes: 8388609\n");
    auto media_buffer_result = loader.load(oversized_media_buffer.path());
    ASSERT_FALSE(media_buffer_result.ok());
    EXPECT_EQ(media_buffer_result.status().code(), ErrorCode::kInvalidArgument);

    TemporaryConfigFile mismatched_media_limit(
        "runtime:\n  max_viewers: 1\n  max_peers: 1\nmedia:\n  max_encoded_subscribers: 2\n");
    auto media_limit_result = loader.load(mismatched_media_limit.path());
    ASSERT_FALSE(media_limit_result.ok());
    EXPECT_EQ(media_limit_result.status().code(), ErrorCode::kInvalidArgument);
}

TEST(RtcConfigLoaderTest, RejectsUnknownEmptySection) {
    TemporaryConfigFile file("mystery_section:\n");
    RtcConfigLoader loader;
    auto result = loader.load(file.path());

    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.status().code(), ErrorCode::kInvalidArgument);
}

TEST(RtcConfigLoaderTest, RejectsInvalidEncoderBoundsAndOptions) {
    RtcConfigLoader loader;

    TemporaryConfigFile odd_dimensions("media:\n  output_width: 641\n");
    auto dimensions_result = loader.load(odd_dimensions.path());
    ASSERT_FALSE(dimensions_result.ok());
    EXPECT_EQ(dimensions_result.status().code(), ErrorCode::kInvalidArgument);

    TemporaryConfigFile invalid_fallback("media:\n  frame_rate: 30\n  fallback_frame_rate: 60\n");
    auto fallback_result = loader.load(invalid_fallback.path());
    ASSERT_FALSE(fallback_result.ok());
    EXPECT_EQ(fallback_result.status().code(), ErrorCode::kInvalidArgument);

    TemporaryConfigFile unsafe_encoder("media:\n  encoder_name: 'h264_nvenc;bad'\n");
    auto encoder_result = loader.load(unsafe_encoder.path());
    ASSERT_FALSE(encoder_result.ok());
    EXPECT_EQ(encoder_result.status().code(), ErrorCode::kInvalidArgument);

    TemporaryConfigFile oversized_frame("media:\n  max_encoded_frame_bytes: 8388609\n");
    auto frame_result = loader.load(oversized_frame.path());
    ASSERT_FALSE(frame_result.ok());
    EXPECT_EQ(frame_result.status().code(), ErrorCode::kInvalidArgument);

    TemporaryConfigFile unsafe_track(
        "media:\n  track_ids: 'right_eye\tspoof'\n  image_topics: /camera/right/image\n  camera_info_topics: "
        "/camera/right/camera_info\n");
    auto track_result = loader.load(unsafe_track.path());
    ASSERT_FALSE(track_result.ok());
    EXPECT_EQ(track_result.status().code(), ErrorCode::kInvalidArgument);
}

TEST(RtcConfigLoaderTest, ExplicitMissingFileDoesNotFallback) {
    RtcConfigLoader loader;
    auto result = loader.load("/tmp/astrabot_rtc_missing_config.yaml");

    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.status().code(), ErrorCode::kNotFound);
}

}  // namespace
}  // namespace astrabot::rtc::config
