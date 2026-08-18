#include <gtest/gtest.h>

#include "astrabot_rtc/session/session_registry.h"

namespace astrabot::rtc::session {
namespace {

PeerDescriptor makePeer(const std::string &session_id, const std::string &peer_id, bool viewer) {
    PeerDescriptor peer;
    peer.session_id = session_id;
    peer.peer_id = peer_id;
    peer.purpose = "opaque-purpose";
    peer.state = PeerState::kConnecting;
    if (viewer) {
        peer.media_tracks = {"right_eye"};
    }
    return peer;
}

TEST(SessionRegistryTest, RejectsDuplicateAndEnforcesViewerLimit) {
    SessionRegistry registry(4U, 2U);

    EXPECT_TRUE(registry.addPeer(makePeer("session-1", "peer-1", true)).ok());
    EXPECT_TRUE(registry.addPeer(makePeer("session-2", "peer-2", true)).ok());
    const Status third_viewer = registry.addPeer(makePeer("session-3", "peer-3", true));
    EXPECT_EQ(third_viewer.code(), ErrorCode::kResourceExhausted);

    const Status duplicate = registry.addPeer(makePeer("session-1", "peer-1", false));
    EXPECT_EQ(duplicate.code(), ErrorCode::kAlreadyExists);
    EXPECT_EQ(registry.peerCount(), 2U);
    EXPECT_EQ(registry.viewerCount(), 2U);
}

TEST(SessionRegistryTest, RemovingViewerReleasesCapacity) {
    SessionRegistry registry(3U, 1U);
    ASSERT_TRUE(registry.addPeer(makePeer("session-1", "peer-1", true)).ok());
    ASSERT_TRUE(registry.removePeer("session-1", "peer-1").ok());
    EXPECT_TRUE(registry.addPeer(makePeer("session-2", "peer-2", true)).ok());
    EXPECT_EQ(registry.viewerCount(), 1U);
}

TEST(SessionRegistryTest, UpdatesStateWithoutChangingOpaqueMetadata) {
    SessionRegistry registry(2U, 1U);
    ASSERT_TRUE(registry.addPeer(makePeer("session-1", "peer-1", false)).ok());
    ASSERT_TRUE(registry.updatePeerState("session-1", "peer-1", PeerState::kConnected, 42U, "connected").ok());

    const auto peer = registry.findPeer("session-1", "peer-1");
    ASSERT_TRUE(peer.has_value());
    EXPECT_EQ(peer->state, PeerState::kConnected);
    EXPECT_EQ(peer->steady_time_ns, 42U);
    EXPECT_EQ(peer->purpose, "opaque-purpose");
}

TEST(SessionRegistryTest, UpdatesDataChannelsButRejectsImmutableBindingChanges) {
    SessionRegistry registry(2U, 1U);
    auto peer = makePeer("session-1", "peer-1", true);
    peer.data_channels = {"astrabot.teleop"};
    ASSERT_TRUE(registry.addPeer(peer).ok());

    peer.data_channels.clear();
    peer.reason_code = "data_channel_closed";
    ASSERT_TRUE(registry.updatePeer(peer).ok());
    const auto updated = registry.findPeer("session-1", "peer-1");
    ASSERT_TRUE(updated.has_value());
    EXPECT_TRUE(updated->data_channels.empty());

    peer.resource_id = "changed-resource";
    EXPECT_EQ(registry.updatePeer(peer).code(), ErrorCode::kFailedPrecondition);
}

TEST(SessionRegistryTest, AllowsOnlyVideoViewerToOmitSessionId) {
    SessionRegistry registry(2U, 2U);
    auto viewer = makePeer("", "webops-1", true);
    viewer.purpose = "video";
    ASSERT_TRUE(registry.addPeer(viewer).ok());
    EXPECT_TRUE(registry.findPeer("", "webops-1").has_value());

    auto teleop = makePeer("", "quest-1", true);
    teleop.purpose = "teleop";
    teleop.run_id = "run-1";
    teleop.resource_id = "thor";
    teleop.data_channels = {"astrabot.teleop"};
    EXPECT_EQ(registry.addPeer(teleop).code(), ErrorCode::kInvalidArgument);
}

TEST(SessionRegistryTest, PeerKeyEncodingDoesNotAliasEmbeddedSeparators) {
    SessionRegistry registry(2U, 2U);
    std::string first_peer_id{"b"};
    first_peer_id.push_back(static_cast<char>(0x1F));
    first_peer_id += "c";
    std::string second_session_id{"a"};
    second_session_id.push_back(static_cast<char>(0x1F));
    second_session_id += "b";

    EXPECT_TRUE(registry.addPeer(makePeer("a", first_peer_id, false)).ok());
    EXPECT_TRUE(registry.addPeer(makePeer(second_session_id, "c", false)).ok());
    EXPECT_EQ(registry.peerCount(), 2U);
}

}  // namespace
}  // namespace astrabot::rtc::session
