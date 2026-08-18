#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "astrabot_rtc/transport/webrtc/webrtc_transport.h"

namespace astrabot::rtc::transport {

/**
 * @brief libdatachannel backend 的有界运行参数。
 */
struct LibDataChannelTransportSettings {
    std::string bind_address;
    std::size_t max_peers{4U};
    std::size_t max_media_peers{2U};
    // 第一版 purpose 配额同时约束有媒体和纯 DataChannel peer，避免关闭媒体后绕过单 writer/viewer 上限。
    std::size_t max_teleop_media_peers{1U};
    std::size_t max_video_media_peers{1U};
    std::size_t max_payload_bytes{16384U};
    std::size_t max_signaling_payload_bytes{65536U};
    std::size_t max_buffered_amount_bytes{65536U};
    std::size_t max_media_buffered_amount_bytes{2U * 1024U * 1024U};
    std::vector<std::string> media_track_ids;
};

/**
 * @brief 使用 libdatachannel 0.24 C API 的真实 PeerConnection/DataChannel adapter。
 *
 * 一个 session_id + peer_id 对应一个 PeerConnection。SDK callback 通过内部 token registry 取得共享运行态，
 * stop 会先禁止新 callback、注销 SDK 资源，再等待已经进入的 callback 退出。
 */
class LibDataChannelWebRtcTransport final : public IWebRtcTransport {
  public:
    explicit LibDataChannelWebRtcTransport(LibDataChannelTransportSettings settings);
    ~LibDataChannelWebRtcTransport() override;

    Status start(WebRtcTransportCallbacks callbacks) override;
    void stop() override;
    Status handleSignaling(const std::string &payload) override;
    Status closePeer(const std::string &session_id, const std::string &peer_id, const std::string &channel_label,
                     const std::string &reason_code) override;
    Status sendDataChannelPacket(const protocol::DataChannelPacket &packet) override;
    Status sendEncodedFrame(std::shared_ptr<const media::EncodedVideoFrame> frame) override;
    WebRtcTransportCapabilities capabilities() const override;
    WebRtcTransportMetrics metrics() const override;

  private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

}  // namespace astrabot::rtc::transport
