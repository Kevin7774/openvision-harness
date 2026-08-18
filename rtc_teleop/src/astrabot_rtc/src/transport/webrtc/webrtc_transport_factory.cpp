#include "astrabot_rtc/transport/webrtc/webrtc_transport_factory.h"

#include <memory>
#include <utility>
#include <vector>

#include "astrabot_rtc/transport/webrtc/disabled_webrtc_transport.h"

#if defined(ASTRABOT_RTC_HAS_LIBDATACHANNEL)
#include "astrabot_rtc/transport/webrtc/libdatachannel_webrtc_transport.h"
#endif

namespace astrabot::rtc::transport {

Result<std::unique_ptr<IWebRtcTransport>> WebRtcTransportFactory::create(const config::RtcConfig &config) {
    if (config.transport.backend == "disabled") {
        return Result<std::unique_ptr<IWebRtcTransport>>::success(std::make_unique<DisabledWebRtcTransport>());
    }
    if (config.transport.backend != "libdatachannel") {
        return Result<std::unique_ptr<IWebRtcTransport>>::failure(
            Status::error(ErrorCode::kInvalidArgument, "configured WebRTC backend is unsupported"));
    }

#if defined(ASTRABOT_RTC_HAS_LIBDATACHANNEL)
    std::vector<std::string> media_track_ids;
    if (config.media.enabled) {
        media_track_ids.reserve(config.media.tracks.size());
        for (const auto &track : config.media.tracks) {
            media_track_ids.push_back(track.track_id);
        }
    }
    LibDataChannelTransportSettings settings;
    settings.bind_address = config.transport.bind_address;
    settings.max_peers = config.runtime.max_peers;
    settings.max_media_peers = config.media.max_encoded_subscribers;
    settings.max_payload_bytes = config.runtime.max_payload_bytes;
    settings.max_signaling_payload_bytes = config.signaling.max_payload_bytes;
    settings.max_buffered_amount_bytes = config.transport.max_buffered_amount_bytes;
    settings.max_media_buffered_amount_bytes = config.transport.max_media_buffered_amount_bytes;
    settings.media_track_ids = std::move(media_track_ids);
    return Result<std::unique_ptr<IWebRtcTransport>>::success(
        std::make_unique<LibDataChannelWebRtcTransport>(std::move(settings)));
#else
    return Result<std::unique_ptr<IWebRtcTransport>>::failure(Status::error(
        ErrorCode::kUnavailable, "libdatachannel backend was requested but this binary was built without it"));
#endif
}

}  // namespace astrabot::rtc::transport
