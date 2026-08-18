#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "astrabot_rtc/common/result.hpp"

namespace astrabot::rtc::config {

/**
 * @brief RTC 与其他 ROS 模块之间的稳定 endpoint 配置。
 */
struct TopicSettings {
    std::string gateway_command{"/astrabot_gateway/webrtc_signal/cmd"};
    std::string gateway_report{"/astrabot_gateway/webrtc_signal/report"};
    std::string peer_event{"/astrabot/rtc/peer_event"};
    std::string data_received{"/astrabot/rtc/data_channel/received"};
    std::string authorize_channel_service{"/astrabot/teleop/authorize_channel"};
    std::string close_peer_service{"/astrabot/rtc/close_peer"};
    std::string diagnostics{"/diagnostics"};
};

/**
 * @brief RTC 资源上限和本机调度参数。
 */
struct RuntimeSettings {
    std::size_t max_viewers{2U};
    std::size_t max_peers{4U};
    std::size_t max_data_channels{8U};
    std::size_t max_payload_bytes{16384U};
    std::int64_t dispatch_period_ms{1};
    std::int64_t authorization_timeout_ms{200};
    std::int64_t diagnostics_period_ms{1000};
};

/**
 * @brief Gateway 信令入口的独立大小边界。
 */
struct SignalingSettings {
    std::size_t max_payload_bytes{65536U};
};

/**
 * @brief WebRTC transport backend 选择。
 *
 * backend 支持 disabled；构建时启用 libdatachannel 后也可选择 libdatachannel。
 */
struct TransportSettings {
    std::string backend{"disabled"};
    // libjuice 必须绑定实际业务网卡；为空时由 SDK 自动选择，可能误选 l4tbr0/docker0。
    std::string bind_address;
    std::size_t max_buffered_amount_bytes{65536U};
    std::size_t max_media_buffered_amount_bytes{2U * 1024U * 1024U};
};

/**
 * @brief 一路相机输入与公开 track id 的绑定。
 */
struct MediaTrackSettings {
    std::string track_id;
    std::string image_topic;
    std::string camera_info_topic;
};

/**
 * @brief FFmpeg H.264 编码器的低延迟和资源上限配置。
 *
 * encoder_name 是唯一允许使用的 encoder，不会隐式尝试其他硬件或软件实现。require_hardware=true 时，
 * FFmpeg 未将该 encoder 标记为硬件实现会直接拒绝启动。
 */
struct H264EncoderSettings {
    std::string encoder_name{"h264_nvenc"};
    bool require_hardware{true};
    std::uint32_t output_width{640U};
    std::uint32_t output_height{480U};
    std::uint32_t frame_rate{60U};
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
 * @brief 原始帧接入和编码帧扇出的公共媒体配置。
 *
 * max_encoded_subscribers 是兼容既有 YAML key 的媒体 PeerConnection admission 上限，不代表存在独立帧 hub。
 */
struct MediaSettings {
    bool enabled{false};
    std::size_t max_encoded_subscribers{2U};
    H264EncoderSettings encoder;
    std::vector<MediaTrackSettings> tracks;
};

/**
 * @brief astrabot_rtc 完整运行配置。
 */
struct RtcConfig {
    TopicSettings topics;
    RuntimeSettings runtime;
    SignalingSettings signaling;
    TransportSettings transport;
    MediaSettings media;
};

/**
 * @brief 加载并严格校验 astrabot_rtc YAML 配置。
 *
 * 解析器只接受配置样例中声明的扁平标量和 CSV 列表，未知 key 会被拒绝，避免拼写错误静默生效。
 */
class RtcConfigLoader {
  public:
    /**
     * @brief 加载配置；path 为空时返回安全默认值。
     *
     * @param path 显式配置路径。非空路径不可读或内容非法时返回错误，不回退默认文件。
     */
    Result<RtcConfig> load(const std::string &path) const;

    /**
     * @brief 校验资源上限、topic 和媒体 track 一致性。
     */
    Status validate(const RtcConfig &config) const;
};

}  // namespace astrabot::rtc::config
