#pragma once

#include <mutex>

#include "astrabot_rtc/transport/webrtc/webrtc_transport.h"

namespace astrabot::rtc::transport {

/**
 * @brief 明确不可用的 WebRTC stub。
 *
 * 该实现只用于建立包边界和测试生命周期；不会创建 peer、媒体 track 或 DataChannel，也不会发出伪连接事件。
 */
class DisabledWebRtcTransport final : public IWebRtcTransport {
  public:
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
    mutable std::mutex mutex_;
    bool running_{false};
};

}  // namespace astrabot::rtc::transport
