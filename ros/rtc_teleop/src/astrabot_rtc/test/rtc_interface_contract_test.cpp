#include <fstream>
#include <iterator>
#include <string>

#include <gtest/gtest.h>

namespace {

std::string readFile(const std::string &relative_path) {
    std::ifstream input(std::string(RTC_SOURCE_DIR) + "/" + relative_path);
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

TEST(RtcInterfaceContractTest, MessageAndServiceSchemasRemainFrozen) {
    EXPECT_EQ(readFile("msg/RtcPeerEvent.msg"), "uint8 STATE_UNKNOWN=0\n"
                                                "uint8 STATE_CONNECTING=1\n"
                                                "uint8 STATE_CONNECTED=2\n"
                                                "uint8 STATE_DISCONNECTED=3\n"
                                                "uint8 STATE_FAILED=4\n"
                                                "uint8 STATE_CLOSED=5\n"
                                                "\n"
                                                "uint8 state\n"
                                                "string session_id\n"
                                                "string peer_id\n"
                                                "string purpose\n"
                                                "string run_id\n"
                                                "string resource_id\n"
                                                "string[] media_tracks\n"
                                                "string[] data_channels\n"
                                                "uint64 steady_time_ns\n"
                                                "string reason_code\n");

    EXPECT_EQ(readFile("msg/RtcDataPacket.msg"), "string session_id\n"
                                                 "string peer_id\n"
                                                 "string channel_label\n"
                                                 "uint64 receive_steady_time_ns\n"
                                                 "uint8[] payload\n");

    EXPECT_EQ(readFile("srv/AuthorizeDataChannel.srv"),
              "string session_id\n"
              "string peer_id\n"
              "string purpose\n"
              "string run_id\n"
              "string resource_id\n"
              "string channel_label\n"
              "string authorization_token\n"
              "---\n"
              "bool allowed\n"
              "string reason_code\n"
              "# 机器人本机 steady_clock 纳秒 deadline；allowed=true 时必须非零且位于未来。\n"
              "uint64 expires_at\n");

    EXPECT_EQ(readFile("srv/CloseRtcPeer.srv"), "string session_id\n"
                                                "string peer_id\n"
                                                "string channel_label\n"
                                                "string reason_code\n"
                                                "---\n"
                                                "bool closed\n"
                                                "string reason_code\n");
}

TEST(RtcInterfaceContractTest, DefaultGatewayAndRtcEndpointsRemainDocumentedInConfig) {
    const std::string config = readFile("config/rtc.yaml");
    EXPECT_NE(config.find("/astrabot_gateway/webrtc_signal/cmd"), std::string::npos);
    EXPECT_NE(config.find("/astrabot_gateway/webrtc_signal/report"), std::string::npos);
    EXPECT_NE(config.find("/astrabot/rtc/peer_event"), std::string::npos);
    EXPECT_NE(config.find("/astrabot/rtc/data_channel/received"), std::string::npos);
    EXPECT_NE(config.find("/astrabot/teleop/authorize_channel"), std::string::npos);
    EXPECT_NE(config.find("/astrabot/rtc/close_peer"), std::string::npos);
    EXPECT_NE(config.find("encoder_name: h264_nvenc"), std::string::npos);
    EXPECT_NE(config.find("require_hardware: true"), std::string::npos);
    EXPECT_NE(config.find("output_width: 640"), std::string::npos);
    EXPECT_NE(config.find("output_height: 480"), std::string::npos);
    EXPECT_NE(config.find("frame_rate: 60"), std::string::npos);
    EXPECT_NE(config.find("fallback_frame_rate: 30"), std::string::npos);
}

TEST(RtcInterfaceContractTest, LateAuthorizationResponsesRemainBoundToTheirAttemptGeneration) {
    const std::string header = readFile("include/astrabot_rtc/runtime/rtc_node.h");
    const std::string source = readFile("src/runtime/rtc_node.cpp");

    EXPECT_NE(header.find("std::uint64_t generation{0U};"), std::string::npos);
    EXPECT_NE(header.find("std::uint64_t next_authorization_generation_{1U};"), std::string::npos);
    EXPECT_NE(source.find("[weak_self, route_key, authorization_generation]"), std::string::npos);
    EXPECT_NE(source.find("iterator->second.generation != generation"), std::string::npos);
}

TEST(RtcInterfaceContractTest, PublishesDataChannelOpenOnlyAfterSdkOpenAndAuthorization) {
    const std::string transport_header = readFile("include/astrabot_rtc/transport/webrtc/webrtc_transport.h");
    const std::string transport_source = readFile("src/transport/webrtc/libdatachannel_webrtc_transport.cpp");
    const std::string runtime_source = readFile("src/runtime/rtc_node.cpp");

    EXPECT_NE(transport_header.find("on_data_channel_ready"), std::string::npos);
    const std::size_t sdk_open = transport_source.find("void RTC_API onDataChannelOpen");
    const std::size_t ready_callback = transport_source.find("callbacks.on_data_channel_ready", sdk_open);
    ASSERT_NE(sdk_open, std::string::npos);
    ASSERT_NE(ready_callback, std::string::npos);

    const std::size_t authorization_response = runtime_source.find("void RtcNode::handleAuthorizationResponse");
    const std::size_t authorization_publish =
        runtime_source.find("publishDataChannelReadyEvent(pending.key)", authorization_response);
    const std::size_t sdk_ready_handler = runtime_source.find("void RtcNode::handleDataChannelReady");
    const std::size_t sdk_ready_publish = runtime_source.find("publishDataChannelReadyEvent(key)", sdk_ready_handler);
    ASSERT_NE(authorization_response, std::string::npos);
    ASSERT_NE(authorization_publish, std::string::npos);
    ASSERT_NE(sdk_ready_handler, std::string::npos);
    ASSERT_NE(sdk_ready_publish, std::string::npos);
    EXPECT_NE(runtime_source.find("\"data_channel_open\""), std::string::npos);
}

}  // namespace
