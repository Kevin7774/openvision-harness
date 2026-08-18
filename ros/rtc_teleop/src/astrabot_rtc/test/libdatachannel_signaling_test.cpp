#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "astrabot_rtc/transport/webrtc/libdatachannel_signaling.h"

namespace astrabot::rtc::transport {
namespace {

TEST(LibDataChannelSignalingTest, ParsesRegisteredPeerAndTrickleShapes) {
    LibDataChannelSignalingCodec codec(65536U);
    auto registered = codec.parse(
        R"({"type":"registered","id":"robot-42","room":"room-robot-42","ice_servers":[{"urls":["stun:stun.example.com:3478","turn:turn.example.com:3478?transport=udp"],"username":"turn-user","credential":"turn-password"}]})");
    ASSERT_TRUE(registered.ok()) << registered.status().message();
    EXPECT_EQ(registered.value().type, WebRtcSignalingCommandType::kRegistered);
    EXPECT_EQ(registered.value().local_peer_id, "robot-42");
    ASSERT_EQ(registered.value().ice_servers.size(), 1U);
    EXPECT_EQ(registered.value().ice_servers.front().urls.size(), 2U);

    auto joined = codec.parse(
        R"({"type":"peer_joined","from":"quest-1","peer_id":"quest-1","room":"room-robot-42","purpose":"teleop","session_id":"session-1","run_id":"run-1","resource_id":"thor","media_tracks":["left_eye","right_eye"],"teleop_grant":"signed-grant","data_channel":{"label":"astrabot.teleop","ordered":false,"max_packet_lifetime_ms":20,"max_payload_bytes":16384}})");
    ASSERT_TRUE(joined.ok()) << joined.status().message();
    EXPECT_EQ(joined.value().type, WebRtcSignalingCommandType::kPeerJoined);
    EXPECT_EQ(joined.value().peer_join.peer_id, "quest-1");
    ASSERT_TRUE(joined.value().peer_join.data_channel.has_value());
    EXPECT_EQ(joined.value().peer_join.data_channel->label, "astrabot.teleop");
    EXPECT_EQ(joined.value().peer_join.data_channel->max_payload_bytes, 16384U);
    EXPECT_EQ(joined.value().peer_join.authorization_token, "signed-grant");

    auto joined_without_run = codec.parse(
        R"({"type":"peer_joined","from":"quest-runless","peer_id":"quest-runless","room":"room-robot-42","purpose":"teleop","session_id":"session-runless","resource_id":"thor","teleop_grant":"signed-grant","data_channel":{"label":"astrabot.teleop","ordered":false,"max_packet_lifetime_ms":20,"max_payload_bytes":16384}})");
    ASSERT_TRUE(joined_without_run.ok()) << joined_without_run.status().message();
    EXPECT_TRUE(joined_without_run.value().peer_join.run_id.empty());

    auto answer = codec.parse(
        R"({"type":"signal","from":"quest-1","to":"robot-42","room":"room-robot-42","signal":{"type":"answer","sdp":"v=0\r\n"}})");
    ASSERT_TRUE(answer.ok()) << answer.status().message();
    EXPECT_EQ(answer.value().type, WebRtcSignalingCommandType::kRemoteAnswer);
    EXPECT_EQ(answer.value().peer_id, "quest-1");

    auto candidate = codec.parse(
        R"({"type":"signal","from":"quest-1","to":"robot-42","signal":{"type":"candidate","candidate":{"candidate":"candidate:1 1 udp 1 127.0.0.1 9000 typ host","sdpMid":"0"}}})");
    ASSERT_TRUE(candidate.ok()) << candidate.status().message();
    EXPECT_EQ(candidate.value().type, WebRtcSignalingCommandType::kRemoteCandidate);
    EXPECT_EQ(candidate.value().candidate_mid, "0");
}

TEST(LibDataChannelSignalingTest, RejectsUnsafeChannelAndSecretShapes) {
    LibDataChannelSignalingCodec codec(65536U);
    const auto unsupported_channel = codec.parse(
        R"({"type":"peer_joined","from":"quest-1","room":"room-robot-42","purpose":"teleop","session_id":"session-1","run_id":"run-1","resource_id":"thor","teleop_grant":"grant","data_channel":{"label":"custom","ordered":false,"max_packet_lifetime_ms":20}})");
    EXPECT_FALSE(unsupported_channel.ok());

    const auto conflicting_token = codec.parse(
        R"({"type":"peer_joined","from":"quest-1","room":"room-robot-42","purpose":"teleop","session_id":"session-1","run_id":"run-1","resource_id":"thor","teleop_grant":"grant","authorization_token":"other","data_channel":{"label":"astrabot.teleop","ordered":false,"max_packet_lifetime_ms":20,"max_payload_bytes":16384}})");
    EXPECT_FALSE(conflicting_token.ok());

    const auto viewer_with_token = codec.parse(
        R"({"type":"peer_joined","from":"webops-1","room":"room-robot-42","purpose":"video","session_id":"session-2","teleop_grant":"unexpected"})");
    EXPECT_FALSE(viewer_with_token.ok());

    const auto unsupported_version = codec.parse(
        R"({"type":"peer_joined","version":2,"from":"webops-1","room":"room-robot-42","purpose":"video","session_id":"session-2"})");
    EXPECT_FALSE(unsupported_version.ok());

    const auto conflicting_route = codec.parse(
        R"({"type":"peer_joined","from":"quest-1","peer_id":"another-peer","room":"room-robot-42","purpose":"video","session_id":"session-2"})");
    EXPECT_FALSE(conflicting_route.ok());

    const auto target_only_route = codec.parse(
        R"({"type":"signal","to":"robot-42","room":"room-robot-42","signal":{"type":"answer","sdp":"v=0\r\n"}})");
    EXPECT_FALSE(target_only_route.ok());

    const auto peer_with_control = codec.parse(
        R"({"type":"peer_joined","from":"webops-1\nspoof","room":"room-robot-42","purpose":"video","media_tracks":["right_eye"]})");
    EXPECT_FALSE(peer_with_control.ok());

    const auto track_with_control = codec.parse(
        R"({"type":"peer_joined","from":"webops-1","room":"room-robot-42","purpose":"video","media_tracks":["right_eye\n"]})");
    EXPECT_FALSE(track_with_control.ok());

    const auto candidate_with_control = codec.parse(
        R"({"type":"signal","from":"quest-1","to":"robot-42","signal":{"type":"candidate","candidate":"candidate:1 1 udp 1 127.0.0.1 9000 typ host\nspoof"}})");
    EXPECT_FALSE(candidate_with_control.ok());

    const auto ice_url_with_control = codec.parse(
        R"({"type":"registered","id":"robot-42","room":"room-robot-42","ice_servers":[{"urls":"stun:stun.example.com:3478\nspoof"}]})");
    EXPECT_FALSE(ice_url_with_control.ok());
}

TEST(LibDataChannelSignalingTest, AcceptsPlatformVideoViewerWithoutApplicationSession) {
    LibDataChannelSignalingCodec codec(65536U);
    auto viewer = codec.parse(
        R"({"type":"peer_joined","version":1,"from":"webops-1","peer_id":"webops-1","room":"room-robot-42","purpose":"video","media_tracks":["right_eye"]})");
    ASSERT_TRUE(viewer.ok()) << viewer.status().message();
    EXPECT_EQ(viewer.value().type, WebRtcSignalingCommandType::kPeerJoined);
    EXPECT_TRUE(viewer.value().peer_join.session_id.empty());
    EXPECT_EQ(viewer.value().peer_join.peer_id, "webops-1");
    EXPECT_EQ(viewer.value().peer_join.purpose, "video");
    ASSERT_EQ(viewer.value().peer_join.media_tracks.size(), 1U);
    EXPECT_EQ(viewer.value().peer_join.media_tracks.front(), "right_eye");
    EXPECT_FALSE(viewer.value().peer_join.data_channel.has_value());

    auto offer = codec.makeDescriptionReport("robot-42", "room-robot-42", "", "webops-1", "offer", "v=0\r\n");
    ASSERT_TRUE(offer.ok()) << offer.status().message();
    const auto offer_json = nlohmann::json::parse(offer.value(), nullptr, false);
    ASSERT_TRUE(offer_json.is_object());
    EXPECT_EQ(offer_json.at("peer_id"), "webops-1");
    EXPECT_FALSE(offer_json.contains("session_id"));

    auto candidate = codec.makeCandidateReport("robot-42", "room-robot-42", "", "webops-1",
                                               "candidate:1 1 udp 1 127.0.0.1 9000 typ host", "0");
    ASSERT_TRUE(candidate.ok()) << candidate.status().message();
    const auto candidate_json = nlohmann::json::parse(candidate.value(), nullptr, false);
    EXPECT_FALSE(candidate_json.contains("session_id"));
}

TEST(LibDataChannelSignalingTest, NormalizesLocalSdpToCrLf) {
    LibDataChannelSignalingCodec codec(65536U);
    for (const std::string &input :
         {std::string("v=0\ra=mid:0\r"), std::string("v=0\na=mid:0\n"), std::string("v=0\r\na=mid:0\r\n")}) {
        const auto report =
            codec.makeDescriptionReport("robot-42", "room-robot-42", "session-1", "quest-1", "offer", input);
        ASSERT_TRUE(report.ok()) << report.status().message();
        const auto parsed = nlohmann::json::parse(report.value(), nullptr, false);
        ASSERT_TRUE(parsed.is_object());
        EXPECT_EQ(parsed.at("signal").at("sdp"), "v=0\r\na=mid:0\r\n");
    }
}

TEST(LibDataChannelSignalingTest, FreezesSingleTeleopDataChannelContract) {
    LibDataChannelSignalingCodec codec(65536U);
    const std::string prefix =
        R"({"type":"peer_joined","from":"quest-1","room":"room-robot-42","purpose":"teleop","session_id":"session-1","run_id":"run-1","resource_id":"thor","teleop_grant":"grant","data_channel":)";

    auto contract = codec.parse(
        prefix +
        R"({"label":"astrabot.teleop","ordered":false,"max_packet_lifetime_ms":20,"max_payload_bytes":16384}})");
    ASSERT_TRUE(contract.ok()) << contract.status().message();
    EXPECT_EQ(contract.value().peer_join.data_channel->max_payload_bytes, 16384U);

    const auto versioned_label = codec.parse(
        prefix +
        R"({"label":"astrabot.teleop.versioned","ordered":false,"max_packet_lifetime_ms":20,"max_payload_bytes":16384}})");
    EXPECT_FALSE(versioned_label.ok());

    const auto ordered = codec.parse(
        prefix +
        R"({"label":"astrabot.teleop","ordered":true,"max_packet_lifetime_ms":20,"max_payload_bytes":16384}})");
    EXPECT_FALSE(ordered.ok());

    const auto missing_limit =
        codec.parse(prefix + R"({"label":"astrabot.teleop","ordered":false,"max_packet_lifetime_ms":20}})");
    EXPECT_FALSE(missing_limit.ok());

    const auto wrong_limit = codec.parse(
        prefix +
        R"({"label":"astrabot.teleop","ordered":false,"max_packet_lifetime_ms":20,"max_payload_bytes":8192}})");
    EXPECT_FALSE(wrong_limit.ok());

    const auto retransmits = codec.parse(
        prefix +
        R"({"label":"astrabot.teleop","ordered":false,"max_packet_lifetime_ms":20,"max_payload_bytes":16384,"max_retransmits":0}})");
    EXPECT_FALSE(retransmits.ok());

    const auto overflowing_limit = codec.parse(
        prefix +
        R"({"label":"astrabot.teleop","ordered":false,"max_packet_lifetime_ms":20,"max_payload_bytes":18446744073709551615}})");
    EXPECT_FALSE(overflowing_limit.ok());
}

TEST(LibDataChannelSignalingTest, KeepsTeleopIdentityStrictWhenVideoSessionIsOptional) {
    LibDataChannelSignalingCodec codec(65536U);
    const auto missing_session = codec.parse(
        R"({"type":"peer_joined","from":"quest-1","room":"room-robot-42","purpose":"teleop","run_id":"run-1","resource_id":"thor","teleop_grant":"grant","data_channel":{"label":"astrabot.teleop","ordered":false,"max_packet_lifetime_ms":20,"max_payload_bytes":16384}})");
    EXPECT_FALSE(missing_session.ok());

    const auto missing_resource = codec.parse(
        R"({"type":"peer_joined","from":"quest-1","room":"room-robot-42","purpose":"teleop","session_id":"session-1","teleop_grant":"grant","data_channel":{"label":"astrabot.teleop","ordered":false,"max_packet_lifetime_ms":20,"max_payload_bytes":16384}})");
    EXPECT_FALSE(missing_resource.ok());

    const auto video_with_run = codec.parse(
        R"({"type":"peer_joined","from":"webops-1","room":"room-robot-42","purpose":"video","run_id":"run-1","media_tracks":["right_eye"]})");
    EXPECT_FALSE(video_with_run.ok());
}

TEST(LibDataChannelSignalingTest, GeneratesGatewayCompatibleReportsWithoutSecretFields) {
    LibDataChannelSignalingCodec codec(65536U);
    auto offer = codec.makeDescriptionReport("robot-42", "room-robot-42", "session-1", "quest-1", "offer", "v=0\r\n");
    ASSERT_TRUE(offer.ok()) << offer.status().message();
    const auto offer_json = nlohmann::json::parse(offer.value(), nullptr, false);
    ASSERT_TRUE(offer_json.is_object());
    EXPECT_EQ(offer_json.at("type"), "signal");
    EXPECT_EQ(offer_json.at("signal").at("type"), "offer");
    EXPECT_FALSE(offer_json.contains("teleop_grant"));
    EXPECT_FALSE(offer_json.contains("authorization_token"));

    auto candidate = codec.makeCandidateReport("robot-42", "room-robot-42", "session-1", "quest-1",
                                               "candidate:1 1 udp 1 127.0.0.1 9000 typ host", "0");
    ASSERT_TRUE(candidate.ok()) << candidate.status().message();
    const auto candidate_json = nlohmann::json::parse(candidate.value(), nullptr, false);
    EXPECT_EQ(candidate_json.at("signal").at("sdpMid"), "0");
}

TEST(LibDataChannelSignalingTest, ReplacesUntrustedCloseMessagesWithSafeReasonCodes) {
    LibDataChannelSignalingCodec codec(65536U);
    const auto remote = codec.parse(
        R"({"type":"peer_left","from":"quest-1","room":"room-robot-42","message":"authorization_token=secret"})");
    ASSERT_TRUE(remote.ok()) << remote.status().message();
    EXPECT_EQ(remote.value().reason_code, "remote_peer_left");

    auto local = codec.makePeerLeftReport("robot-42", "room-robot-42", "session-1", "quest-1", "grant signed-secret");
    ASSERT_TRUE(local.ok()) << local.status().message();
    const auto local_json = nlohmann::json::parse(local.value(), nullptr, false);
    EXPECT_EQ(local_json.at("message"), "local_close");
}

}  // namespace
}  // namespace astrabot::rtc::transport
