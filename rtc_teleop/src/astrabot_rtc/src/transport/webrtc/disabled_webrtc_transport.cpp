#include "astrabot_rtc/transport/webrtc/disabled_webrtc_transport.h"

namespace astrabot::rtc::transport {

Status DisabledWebRtcTransport::start(WebRtcTransportCallbacks callbacks) {
    (void)callbacks;
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = true;
    return Status::success();
}

void DisabledWebRtcTransport::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
}

Status DisabledWebRtcTransport::handleSignaling(const std::string &payload) {
    (void)payload;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
        return Status::error(ErrorCode::kFailedPrecondition, "disabled WebRTC transport is stopped");
    }
    return Status::error(ErrorCode::kUnavailable, "WebRTC transport backend is disabled");
}

Status DisabledWebRtcTransport::closePeer(const std::string &session_id, const std::string &peer_id,
                                          const std::string &channel_label, const std::string &reason_code) {
    (void)session_id;
    (void)peer_id;
    (void)channel_label;
    (void)reason_code;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
        return Status::error(ErrorCode::kFailedPrecondition, "disabled WebRTC transport is stopped");
    }
    return Status::error(ErrorCode::kUnavailable, "disabled WebRTC transport has no peer to close");
}

Status DisabledWebRtcTransport::sendDataChannelPacket(const protocol::DataChannelPacket &packet) {
    (void)packet;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
        return Status::error(ErrorCode::kFailedPrecondition, "disabled WebRTC transport is stopped");
    }
    return Status::error(ErrorCode::kUnavailable, "disabled WebRTC transport cannot send data");
}

Status DisabledWebRtcTransport::sendEncodedFrame(std::shared_ptr<const media::EncodedVideoFrame> frame) {
    (void)frame;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
        return Status::error(ErrorCode::kFailedPrecondition, "disabled WebRTC transport is stopped");
    }
    return Status::error(ErrorCode::kUnavailable, "disabled WebRTC transport cannot send media");
}

WebRtcTransportCapabilities DisabledWebRtcTransport::capabilities() const {
    return WebRtcTransportCapabilities{"disabled", false, false, false};
}

WebRtcTransportMetrics DisabledWebRtcTransport::metrics() const {
    return {};
}

}  // namespace astrabot::rtc::transport
