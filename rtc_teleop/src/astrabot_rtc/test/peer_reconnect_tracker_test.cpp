#include <string>

#include <gtest/gtest.h>

#include "astrabot_rtc/transport/webrtc/peer_reconnect_tracker.h"

namespace astrabot::rtc::transport {
namespace {

TEST(PeerReconnectTrackerTest, CountsOnlyConnectedAfterObservedLoss) {
    PeerReconnectTracker tracker(2U);
    ASSERT_TRUE(tracker.registerBinding("session-1/quest-1").ok());

    EXPECT_FALSE(tracker.observeState("session-1/quest-1", session::PeerState::kConnecting));
    EXPECT_FALSE(tracker.observeState("session-1/quest-1", session::PeerState::kConnected));
    EXPECT_FALSE(tracker.observeState("session-1/quest-1", session::PeerState::kDisconnected));
    EXPECT_TRUE(tracker.observeState("session-1/quest-1", session::PeerState::kConnected));
    EXPECT_FALSE(tracker.observeState("session-1/quest-1", session::PeerState::kConnected));
}

TEST(PeerReconnectTrackerTest, PreservesLostBindingAcrossPeerConnectionRecreation) {
    PeerReconnectTracker tracker(2U);
    ASSERT_TRUE(tracker.registerBinding("session-1/quest-1").ok());
    EXPECT_FALSE(tracker.observeState("session-1/quest-1", session::PeerState::kConnected));
    EXPECT_FALSE(tracker.observeState("session-1/quest-1", session::PeerState::kFailed));
    tracker.unregisterBinding("session-1/quest-1");

    ASSERT_TRUE(tracker.registerBinding("session-1/quest-1").ok());
    EXPECT_TRUE(tracker.observeState("session-1/quest-1", session::PeerState::kConnected));
}

TEST(PeerReconnectTrackerTest, CleanRecreationAndInitialFailureAreNotReconnects) {
    PeerReconnectTracker tracker(2U);
    ASSERT_TRUE(tracker.registerBinding("viewer-1").ok());
    EXPECT_FALSE(tracker.observeState("viewer-1", session::PeerState::kConnected));
    tracker.unregisterBinding("viewer-1");
    ASSERT_TRUE(tracker.registerBinding("viewer-1").ok());
    EXPECT_FALSE(tracker.observeState("viewer-1", session::PeerState::kConnected));
    tracker.unregisterBinding("viewer-1");

    ASSERT_TRUE(tracker.registerBinding("viewer-2").ok());
    EXPECT_FALSE(tracker.observeState("viewer-2", session::PeerState::kFailed));
    tracker.unregisterBinding("viewer-2");
    ASSERT_TRUE(tracker.registerBinding("viewer-2").ok());
    EXPECT_FALSE(tracker.observeState("viewer-2", session::PeerState::kConnected));
}

TEST(PeerReconnectTrackerTest, BoundsActiveAndLostBindingState) {
    PeerReconnectTracker tracker(1U);
    ASSERT_TRUE(tracker.registerBinding("peer-1").ok());
    EXPECT_EQ(tracker.registerBinding("peer-2").code(), ErrorCode::kResourceExhausted);
    EXPECT_FALSE(tracker.observeState("peer-1", session::PeerState::kConnected));
    EXPECT_FALSE(tracker.observeState("peer-1", session::PeerState::kDisconnected));
    tracker.unregisterBinding("peer-1");

    ASSERT_TRUE(tracker.registerBinding("peer-2").ok());
    EXPECT_FALSE(tracker.observeState("peer-2", session::PeerState::kConnected));
    EXPECT_FALSE(tracker.observeState("peer-2", session::PeerState::kDisconnected));
    tracker.unregisterBinding("peer-2");

    ASSERT_TRUE(tracker.registerBinding("peer-1").ok());
    EXPECT_FALSE(tracker.observeState("peer-1", session::PeerState::kConnected));
}

}  // namespace
}  // namespace astrabot::rtc::transport
