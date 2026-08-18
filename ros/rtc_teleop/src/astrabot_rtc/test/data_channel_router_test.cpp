#include <cstdint>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "astrabot_rtc/session/data_channel_router.h"

namespace astrabot::rtc::session {
namespace {

class FakeClock final : public IClock {
  public:
    std::uint64_t nowNanoseconds() const override {
        return now_ns_;
    }

    void setNow(std::uint64_t now_ns) {
        now_ns_ = now_ns;
    }

  private:
    std::uint64_t now_ns_{100U};
};

protocol::DataChannelPacket makePacket(std::uint8_t value, std::size_t payload_size = 1U) {
    protocol::DataChannelPacket packet;
    packet.session_id = "session-1";
    packet.peer_id = "peer-1";
    packet.channel_label = "opaque.channel.v1";
    packet.receive_steady_time_ns = value;
    packet.payload.assign(payload_size, value);
    return packet;
}

DataChannelKey key() {
    return DataChannelKey{"session-1", "peer-1", "opaque.channel.v1"};
}

std::string withUnitSeparator(const std::string &prefix, const std::string &suffix) {
    std::string value = prefix;
    value.push_back(static_cast<char>(0x1F));
    value += suffix;
    return value;
}

TEST(DataChannelRouterTest, RequiresStartAndAuthorizationBeforeForwarding) {
    auto clock = std::make_shared<FakeClock>();
    DataChannelRouter router(2U, 16U, clock);

    EXPECT_EQ(router.pushIncoming(makePacket(1U)).code(), ErrorCode::kFailedPrecondition);
    EXPECT_TRUE(router.start().ok());
    EXPECT_TRUE(router.start().ok());
    EXPECT_FALSE(router.isAuthorized(key()));
    EXPECT_EQ(router.pushIncoming(makePacket(2U)).code(), ErrorCode::kPermissionDenied);
    EXPECT_TRUE(router.authorize(key(), ChannelAuthorization{true, "allowed", 200U}).ok());
    EXPECT_TRUE(router.isAuthorized(key()));
    EXPECT_TRUE(router.pushIncoming(makePacket(3U)).ok());

    const auto packet = router.takeLatest(key());
    ASSERT_TRUE(packet.has_value());
    EXPECT_EQ(packet->payload.front(), 3U);
}

TEST(DataChannelRouterTest, KeepsOnlyLatestPacketAndCountsOverwrite) {
    auto clock = std::make_shared<FakeClock>();
    DataChannelRouter router(1U, 16U, clock);
    ASSERT_TRUE(router.start().ok());
    ASSERT_TRUE(router.authorize(key(), ChannelAuthorization{true, "allowed", 200U}).ok());

    ASSERT_TRUE(router.pushIncoming(makePacket(4U)).ok());
    ASSERT_TRUE(router.pushIncoming(makePacket(5U)).ok());
    const auto packets = router.takeAllLatest();
    ASSERT_EQ(packets.size(), 1U);
    EXPECT_EQ(packets.front().payload.front(), 5U);
    EXPECT_EQ(router.metrics().overwritten_packets, 1U);
}

TEST(DataChannelRouterTest, RejectsOversizedAndExpiredPackets) {
    auto clock = std::make_shared<FakeClock>();
    DataChannelRouter router(1U, 4U, clock);
    ASSERT_TRUE(router.start().ok());
    ASSERT_TRUE(router.authorize(key(), ChannelAuthorization{true, "allowed", 150U}).ok());

    EXPECT_EQ(router.pushIncoming(makePacket(6U, 5U)).code(), ErrorCode::kPayloadTooLarge);
    clock->setNow(151U);
    EXPECT_FALSE(router.isAuthorized(key()));
    EXPECT_EQ(router.pushIncoming(makePacket(7U)).code(), ErrorCode::kPermissionDenied);
    EXPECT_EQ(router.authorizedChannelCount(), 0U);
}

TEST(DataChannelRouterTest, DropsPacketThatExpiresWhileWaitingForDispatch) {
    auto clock = std::make_shared<FakeClock>();
    DataChannelRouter router(1U, 16U, clock);
    ASSERT_TRUE(router.start().ok());
    ASSERT_TRUE(router.authorize(key(), ChannelAuthorization{true, "allowed", 150U}).ok());
    ASSERT_TRUE(router.pushIncoming(makePacket(9U)).ok());

    clock->setNow(151U);
    EXPECT_FALSE(router.takeLatest(key()).has_value());
    EXPECT_EQ(router.authorizedChannelCount(), 0U);
    EXPECT_EQ(router.metrics().expired_queued_packets, 1U);

    clock->setNow(200U);
    ASSERT_TRUE(router.authorize(key(), ChannelAuthorization{true, "allowed", 250U}).ok());
    ASSERT_TRUE(router.pushIncoming(makePacket(10U)).ok());
    clock->setNow(251U);
    EXPECT_TRUE(router.takeAllLatest().empty());
    EXPECT_EQ(router.authorizedChannelCount(), 0U);
    EXPECT_EQ(router.metrics().expired_queued_packets, 2U);
}

TEST(DataChannelRouterTest, RejectsMissingAuthorizationDeadline) {
    auto clock = std::make_shared<FakeClock>();
    DataChannelRouter router(1U, 16U, clock);
    ASSERT_TRUE(router.start().ok());

    EXPECT_EQ(router.authorize(key(), ChannelAuthorization{true, "allowed", 0U}).code(), ErrorCode::kPermissionDenied);
    EXPECT_EQ(router.authorizedChannelCount(), 0U);
}

TEST(DataChannelRouterTest, StopIsIdempotentAndClearsAuthorization) {
    auto clock = std::make_shared<FakeClock>();
    DataChannelRouter router(1U, 16U, clock);
    ASSERT_TRUE(router.start().ok());
    ASSERT_TRUE(router.authorize(key(), ChannelAuthorization{true, "allowed", 200U}).ok());

    router.stop();
    router.stop();
    EXPECT_FALSE(router.isAuthorized(key()));
    EXPECT_EQ(router.authorizedChannelCount(), 0U);
    EXPECT_EQ(router.pushIncoming(makePacket(8U)).code(), ErrorCode::kFailedPrecondition);
}

TEST(DataChannelRouterTest, RouteKeyEncodingDoesNotAliasEmbeddedSeparators) {
    auto clock = std::make_shared<FakeClock>();
    DataChannelRouter router(2U, 16U, clock);
    ASSERT_TRUE(router.start().ok());

    const DataChannelKey first{"a", withUnitSeparator("b", "c"), "d"};
    const DataChannelKey second{withUnitSeparator("a", "b"), "c", "d"};
    ASSERT_TRUE(router.authorize(first, ChannelAuthorization{true, "allowed", 200U}).ok());
    ASSERT_TRUE(router.authorize(second, ChannelAuthorization{true, "allowed", 200U}).ok());

    protocol::DataChannelPacket first_packet;
    first_packet.session_id = first.session_id;
    first_packet.peer_id = first.peer_id;
    first_packet.channel_label = first.channel_label;
    first_packet.payload = {1U};
    protocol::DataChannelPacket second_packet;
    second_packet.session_id = second.session_id;
    second_packet.peer_id = second.peer_id;
    second_packet.channel_label = second.channel_label;
    second_packet.payload = {2U};
    ASSERT_TRUE(router.pushIncoming(std::move(first_packet)).ok());
    ASSERT_TRUE(router.pushIncoming(std::move(second_packet)).ok());

    const auto first_result = router.takeLatest(first);
    const auto second_result = router.takeLatest(second);
    ASSERT_TRUE(first_result.has_value());
    ASSERT_TRUE(second_result.has_value());
    EXPECT_EQ(first_result->payload.front(), 1U);
    EXPECT_EQ(second_result->payload.front(), 2U);
}

TEST(DataChannelRouterTest, RevokePeerClearsEveryQueuedChannelForThatBinding) {
    auto clock = std::make_shared<FakeClock>();
    DataChannelRouter router(3U, 16U, clock);
    ASSERT_TRUE(router.start().ok());

    const DataChannelKey first{"session-1", "peer-1", "channel-1"};
    const DataChannelKey second{"session-1", "peer-1", "channel-2"};
    const DataChannelKey other{"session-2", "peer-2", "channel-1"};
    ASSERT_TRUE(router.authorize(first, ChannelAuthorization{true, "allowed", 200U}).ok());
    ASSERT_TRUE(router.authorize(second, ChannelAuthorization{true, "allowed", 200U}).ok());
    ASSERT_TRUE(router.authorize(other, ChannelAuthorization{true, "allowed", 200U}).ok());

    protocol::DataChannelPacket first_packet;
    first_packet.session_id = first.session_id;
    first_packet.peer_id = first.peer_id;
    first_packet.channel_label = first.channel_label;
    first_packet.payload = {1U};
    protocol::DataChannelPacket second_packet;
    second_packet.session_id = second.session_id;
    second_packet.peer_id = second.peer_id;
    second_packet.channel_label = second.channel_label;
    second_packet.payload = {2U};
    protocol::DataChannelPacket other_packet;
    other_packet.session_id = other.session_id;
    other_packet.peer_id = other.peer_id;
    other_packet.channel_label = other.channel_label;
    other_packet.payload = {3U};
    ASSERT_TRUE(router.pushIncoming(std::move(first_packet)).ok());
    ASSERT_TRUE(router.pushIncoming(std::move(second_packet)).ok());
    ASSERT_TRUE(router.pushIncoming(std::move(other_packet)).ok());

    EXPECT_EQ(router.revokePeer("session-1", "peer-1"), 2U);
    EXPECT_EQ(router.authorizedChannelCount(), 1U);
    EXPECT_FALSE(router.takeLatest(first).has_value());
    EXPECT_FALSE(router.takeLatest(second).has_value());
    ASSERT_TRUE(router.takeLatest(other).has_value());
    EXPECT_EQ(router.revokePeer("session-1", "peer-1"), 0U);
}

}  // namespace
}  // namespace astrabot::rtc::session
