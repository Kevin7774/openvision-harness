#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "astrabot_rtc/transport/webrtc/disabled_webrtc_transport.h"

namespace astrabot::rtc::transport {
namespace {

TEST(DisabledWebRtcTransportTest, NeverClaimsRtcCapabilities) {
    DisabledWebRtcTransport transport;
    const auto capabilities = transport.capabilities();
    EXPECT_EQ(capabilities.backend, "disabled");
    EXPECT_FALSE(capabilities.peer_connections);
    EXPECT_FALSE(capabilities.media_tracks);
    EXPECT_FALSE(capabilities.data_channels);
    const auto metrics = transport.metrics();
    EXPECT_EQ(metrics.media_peer_sends, 0U);
    EXPECT_EQ(metrics.media_peer_bytes, 0U);
    EXPECT_EQ(metrics.media_teleop_congestion_drops, 0U);
    EXPECT_EQ(metrics.media_viewer_congestion_drops, 0U);
    EXPECT_EQ(metrics.media_buffer_query_failures, 0U);
    EXPECT_EQ(metrics.reconnect_count, 0U);
}

TEST(DisabledWebRtcTransportTest, LifecycleIsIdempotentAndOperationsRemainUnavailable) {
    DisabledWebRtcTransport transport;
    EXPECT_TRUE(transport.start({}).ok());
    EXPECT_TRUE(transport.start({}).ok());
    EXPECT_EQ(transport.handleSignaling("{}").code(), ErrorCode::kUnavailable);

    auto frame = std::make_shared<media::EncodedVideoFrame>();
    frame->track_id = "right_eye";
    frame->data = std::make_shared<const std::vector<std::uint8_t>>(std::vector<std::uint8_t>{1U});
    EXPECT_EQ(transport.sendEncodedFrame(frame).code(), ErrorCode::kUnavailable);

    protocol::DataChannelPacket packet;
    packet.session_id = "session-1";
    packet.peer_id = "peer-1";
    packet.channel_label = "opaque.channel";
    EXPECT_EQ(transport.sendDataChannelPacket(packet).code(), ErrorCode::kUnavailable);

    transport.stop();
    transport.stop();
    EXPECT_EQ(transport.handleSignaling("{}").code(), ErrorCode::kFailedPrecondition);
}

}  // namespace
}  // namespace astrabot::rtc::transport
