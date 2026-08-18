#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <rtc/rtc.h>

#include "astrabot_rtc/transport/webrtc/libdatachannel_webrtc_transport.h"

namespace astrabot::rtc::transport {
namespace {

using namespace std::chrono_literals;

class RemotePeer final {
  public:
    RemotePeer(std::string local_id, std::string remote_id, std::string room, std::string session_id)
        : local_id_(std::move(local_id)), remote_id_(std::move(remote_id)), room_(std::move(room)),
          session_id_(std::move(session_id)) {}

    ~RemotePeer() {
        stop();
    }

    bool start(LibDataChannelWebRtcTransport *transport) {
        transport_ = transport;
        rtcConfiguration configuration{};
        configuration.disableAutoNegotiation = true;
        configuration.maxMessageSize = 16384;
        pc_ = rtcCreatePeerConnection(&configuration);
        if (pc_ < 0) {
            return false;
        }
        rtcSetUserPointer(pc_, this);
        return rtcSetLocalDescriptionCallback(pc_, &RemotePeer::onLocalDescription) >= 0 &&
               rtcSetLocalCandidateCallback(pc_, &RemotePeer::onLocalCandidate) >= 0 &&
               rtcSetDataChannelCallback(pc_, &RemotePeer::onDataChannel) >= 0 &&
               rtcSetTrackCallback(pc_, &RemotePeer::onTrack) >= 0;
    }

    void stop() {
        int pc = -1;
        int dc = -1;
        std::vector<int> tracks;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            pc = pc_;
            dc = data_channel_;
            tracks = tracks_;
            pc_ = -1;
            data_channel_ = -1;
            tracks_.clear();
            transport_ = nullptr;
        }
        if (dc >= 0) {
            rtcSetOpenCallback(dc, nullptr);
            rtcSetClosedCallback(dc, nullptr);
            rtcSetMessageCallback(dc, nullptr);
            rtcSetUserPointer(dc, nullptr);
            rtcClose(dc);
            rtcDeleteDataChannel(dc);
        }
        for (const int track : tracks) {
            rtcSetMessageCallback(track, nullptr);
            rtcSetUserPointer(track, nullptr);
            rtcClose(track);
            rtcDeleteTrack(track);
        }
        if (pc >= 0) {
            rtcSetLocalDescriptionCallback(pc, nullptr);
            rtcSetLocalCandidateCallback(pc, nullptr);
            rtcSetDataChannelCallback(pc, nullptr);
            rtcSetTrackCallback(pc, nullptr);
            rtcSetUserPointer(pc, nullptr);
            rtcClosePeerConnection(pc);
            rtcDeletePeerConnection(pc);
        }
    }

    void handleDeviceSignal(const std::string &payload) {
        const auto report = nlohmann::json::parse(payload, nullptr, false);
        if (!report.is_object() || !report.contains("signal") || !report["signal"].is_object()) {
            return;
        }
        const auto &signal = report["signal"];
        if (!signal.contains("type") || !signal["type"].is_string()) {
            return;
        }
        const std::string type = signal["type"].get_ref<const std::string &>();
        if (type == "offer" && signal.contains("sdp") && signal["sdp"].is_string()) {
            handleOffer(signal["sdp"].get_ref<const std::string &>());
            return;
        }
        if (type == "candidate" && signal.contains("candidate") && signal["candidate"].is_string()) {
            handleDeviceCandidate(signal["candidate"].get_ref<const std::string &>(),
                                  signal.contains("sdpMid") && signal["sdpMid"].is_string()
                                      ? signal["sdpMid"].get_ref<const std::string &>()
                                      : std::string{});
        }
    }

    bool waitForOpen(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return condition_.wait_for(lock, timeout, [this]() { return data_channel_open_; });
    }

    bool waitForTracks(std::size_t count, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return condition_.wait_for(lock, timeout, [this, count]() { return tracks_.size() >= count; });
    }

    bool waitForMessage(const std::vector<std::uint8_t> &expected, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return condition_.wait_for(lock, timeout, [this, &expected]() { return received_payload_ == expected; });
    }

    bool send(const std::vector<std::uint8_t> &payload) {
        int dc = -1;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            dc = data_channel_;
        }
        return dc >= 0 && rtcSendMessage(dc, reinterpret_cast<const char *>(payload.data()),
                                         static_cast<int>(payload.size())) >= 0;
    }

    void closeDataChannel() {
        int dc = -1;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            dc = data_channel_;
            data_channel_ = -1;
            data_channel_open_ = false;
        }
        if (dc >= 0) {
            rtcSetOpenCallback(dc, nullptr);
            rtcSetClosedCallback(dc, nullptr);
            rtcSetMessageCallback(dc, nullptr);
            rtcSetUserPointer(dc, nullptr);
            rtcClose(dc);
            rtcDeleteDataChannel(dc);
        }
    }

    bool reliabilityMatchesContract() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return channel_label_ == "astrabot.teleop" && channel_reliability_.unordered &&
               channel_reliability_.unreliable && channel_reliability_.maxPacketLifeTime == 20U;
    }

  private:
    struct CandidateSnapshot {
        std::string candidate;
        std::string mid;
    };

    void handleOffer(const std::string &sdp) {
        int pc = -1;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            pc = pc_;
        }
        if (pc < 0 || rtcSetRemoteDescription(pc, sdp.c_str(), "offer") < 0) {
            return;
        }
        std::vector<CandidateSnapshot> pending;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            remote_description_set_ = true;
            pending.swap(pending_device_candidates_);
        }
        for (const auto &candidate : pending) {
            rtcAddRemoteCandidate(pc, candidate.candidate.c_str(),
                                  candidate.mid.empty() ? nullptr : candidate.mid.c_str());
        }
        rtcSetLocalDescription(pc, "answer");
    }

    void handleDeviceCandidate(const std::string &candidate, const std::string &mid) {
        int pc = -1;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!remote_description_set_) {
                pending_device_candidates_.push_back(CandidateSnapshot{candidate, mid});
                return;
            }
            pc = pc_;
        }
        if (pc >= 0) {
            rtcAddRemoteCandidate(pc, candidate.c_str(), mid.empty() ? nullptr : mid.c_str());
        }
    }

    void sendToDevice(nlohmann::json signal) {
        LibDataChannelWebRtcTransport *transport = nullptr;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            transport = transport_;
        }
        if (transport == nullptr) {
            return;
        }
        nlohmann::json envelope{{"type", "signal"},           {"version", 1},         {"from", local_id_},
                                {"to", remote_id_},           {"peer_id", local_id_}, {"room", room_},
                                {"signal", std::move(signal)}};
        if (!session_id_.empty()) {
            envelope["session_id"] = session_id_;
        }
        transport->handleSignaling(envelope.dump());
    }

    static void RTC_API onLocalDescription(int pc, const char *sdp, const char *type, void *pointer) {
        (void)pc;
        auto *self = static_cast<RemotePeer *>(pointer);
        if (self != nullptr && sdp != nullptr && type != nullptr) {
            self->sendToDevice(nlohmann::json{{"type", type}, {"version", 1}, {"sdp", sdp}});
        }
    }

    static void RTC_API onLocalCandidate(int pc, const char *candidate, const char *mid, void *pointer) {
        (void)pc;
        auto *self = static_cast<RemotePeer *>(pointer);
        if (self == nullptr || candidate == nullptr || candidate[0] == '\0') {
            return;
        }
        nlohmann::json signal{{"type", "candidate"}, {"version", 1}, {"candidate", candidate}};
        if (mid != nullptr && mid[0] != '\0') {
            signal["sdpMid"] = mid;
        }
        self->sendToDevice(std::move(signal));
    }

    static void RTC_API onDataChannel(int pc, int dc, void *pointer) {
        (void)pc;
        auto *self = static_cast<RemotePeer *>(pointer);
        if (self == nullptr) {
            rtcClose(dc);
            rtcDeleteDataChannel(dc);
            return;
        }
        char label[256]{};
        rtcReliability reliability{};
        if (rtcGetDataChannelLabel(dc, label, static_cast<int>(sizeof(label))) < 0 ||
            rtcGetDataChannelReliability(dc, &reliability) < 0) {
            rtcClose(dc);
            rtcDeleteDataChannel(dc);
            return;
        }
        rtcSetUserPointer(dc, self);
        rtcSetOpenCallback(dc, &RemotePeer::onChannelOpen);
        rtcSetClosedCallback(dc, &RemotePeer::onChannelClosed);
        rtcSetMessageCallback(dc, &RemotePeer::onChannelMessage);
        {
            std::lock_guard<std::mutex> lock(self->mutex_);
            self->data_channel_ = dc;
            self->channel_label_ = label;
            self->channel_reliability_ = reliability;
        }
        self->condition_.notify_all();
    }

    static void RTC_API onChannelOpen(int dc, void *pointer) {
        (void)dc;
        auto *self = static_cast<RemotePeer *>(pointer);
        if (self == nullptr) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(self->mutex_);
            self->data_channel_open_ = true;
        }
        self->condition_.notify_all();
    }

    static void RTC_API onChannelClosed(int dc, void *pointer) {
        (void)dc;
        auto *self = static_cast<RemotePeer *>(pointer);
        if (self == nullptr) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(self->mutex_);
            self->data_channel_open_ = false;
        }
        self->condition_.notify_all();
    }

    static void RTC_API onChannelMessage(int dc, const char *message, int size, void *pointer) {
        (void)dc;
        auto *self = static_cast<RemotePeer *>(pointer);
        if (self == nullptr || message == nullptr || size < 0) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(self->mutex_);
            self->received_payload_.assign(reinterpret_cast<const std::uint8_t *>(message),
                                           reinterpret_cast<const std::uint8_t *>(message) + size);
        }
        self->condition_.notify_all();
    }

    static void RTC_API onTrack(int pc, int track, void *pointer) {
        (void)pc;
        auto *self = static_cast<RemotePeer *>(pointer);
        if (self == nullptr) {
            rtcClose(track);
            rtcDeleteTrack(track);
            return;
        }
        rtcSetUserPointer(track, self);
        rtcSetMessageCallback(track, &RemotePeer::onTrackMessage);
        {
            std::lock_guard<std::mutex> lock(self->mutex_);
            self->tracks_.push_back(track);
        }
        self->condition_.notify_all();
    }

    static void RTC_API onTrackMessage(int track, const char *message, int size, void *pointer) {
        (void)track;
        (void)message;
        (void)size;
        auto *self = static_cast<RemotePeer *>(pointer);
        if (self == nullptr) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(self->mutex_);
            ++self->track_message_count_;
        }
        self->condition_.notify_all();
    }

    const std::string local_id_;
    const std::string remote_id_;
    const std::string room_;
    const std::string session_id_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    LibDataChannelWebRtcTransport *transport_{nullptr};
    int pc_{-1};
    int data_channel_{-1};
    std::vector<int> tracks_;
    bool remote_description_set_{false};
    bool data_channel_open_{false};
    std::string channel_label_;
    rtcReliability channel_reliability_{};
    std::vector<CandidateSnapshot> pending_device_candidates_;
    std::vector<std::uint8_t> received_payload_;
    std::size_t track_message_count_{0U};
};

struct TransportObservation {
    std::mutex mutex;
    std::condition_variable condition;
    bool connected{false};
    bool data_channel_ready{false};
    bool data_channel_closed{false};
    std::optional<DataChannelOpenRequest> authorization_request;
    std::optional<protocol::DataChannelPacket> incoming_packet;
};

std::shared_ptr<media::EncodedVideoFrame> makeEncodedFrame(const std::string &track_id) {
    auto data = std::make_shared<const std::vector<std::uint8_t>>(
        std::vector<std::uint8_t>{0U,    0U,    0U,    1U,    0x67U, 0x42U, 0x00U, 0x1FU, 0U,    0U,    0U,   1U,
                                  0x68U, 0xCEU, 0x3CU, 0x80U, 0U,    0U,    0U,    1U,    0x65U, 0x88U, 0x84U});
    auto frame = std::make_shared<media::EncodedVideoFrame>();
    frame->track_id = track_id;
    frame->codec = "h264";
    frame->capture_time_ns = 1000000000U;
    frame->key_frame = true;
    frame->data = std::move(data);
    return frame;
}

TEST(LibDataChannelTransportTest, ConnectsTwoPeersAndMovesDataInBothDirections) {
    LibDataChannelTransportSettings settings;
    settings.max_peers = 2U;
    settings.media_track_ids = {"left_eye", "right_eye"};
    LibDataChannelWebRtcTransport transport(settings);
    RemotePeer remote("quest-1", "robot-42", "room-robot-42", "session-1");
    ASSERT_TRUE(remote.start(&transport));

    TransportObservation observation;
    WebRtcTransportCallbacks callbacks;
    callbacks.on_peer_event = [&observation](const session::PeerDescriptor &peer) {
        if (peer.state == session::PeerState::kConnected) {
            {
                std::lock_guard<std::mutex> lock(observation.mutex);
                observation.connected = true;
            }
            observation.condition.notify_all();
        }
        if (peer.reason_code == "data_channel_closed" && peer.data_channels.empty()) {
            {
                std::lock_guard<std::mutex> lock(observation.mutex);
                observation.data_channel_closed = true;
            }
            observation.condition.notify_all();
        }
    };
    callbacks.on_data_channel_open = [&observation](DataChannelOpenRequest request) {
        {
            std::lock_guard<std::mutex> lock(observation.mutex);
            observation.authorization_request = std::move(request);
        }
        observation.condition.notify_all();
    };
    callbacks.on_data_channel_ready = [&observation](DataChannelReadyEvent event) {
        {
            std::lock_guard<std::mutex> lock(observation.mutex);
            observation.data_channel_ready = event.session_id == "session-1" && event.peer_id == "quest-1" &&
                                             event.channel_label == "astrabot.teleop";
        }
        observation.condition.notify_all();
    };
    callbacks.on_data_channel_packet = [&observation](protocol::DataChannelPacket packet) {
        {
            std::lock_guard<std::mutex> lock(observation.mutex);
            observation.incoming_packet = std::move(packet);
        }
        observation.condition.notify_all();
    };
    callbacks.on_signaling_report = [&remote](std::string payload) { remote.handleDeviceSignal(payload); };
    ASSERT_TRUE(transport.start(std::move(callbacks)).ok());
    ASSERT_TRUE(transport.handleSignaling(R"({"type":"registered","id":"robot-42","room":"room-robot-42"})").ok());
    ASSERT_TRUE(
        transport
            .handleSignaling(
                R"({"type":"peer_joined","from":"quest-1","peer_id":"quest-1","room":"room-robot-42","purpose":"teleop","session_id":"session-1","run_id":"run-1","resource_id":"thor","media_tracks":["left_eye","right_eye"],"teleop_grant":"signed-grant","data_channel":{"label":"astrabot.teleop","ordered":false,"max_packet_lifetime_ms":20,"max_payload_bytes":16384}})")
            .ok());

    {
        std::unique_lock<std::mutex> lock(observation.mutex);
        ASSERT_TRUE(observation.condition.wait_for(lock, 8s, [&observation]() {
            return observation.connected && observation.data_channel_ready &&
                   observation.authorization_request.has_value();
        }));
        ASSERT_EQ(observation.authorization_request->authorization_token, "signed-grant");
        ASSERT_EQ(observation.authorization_request->channel_label, "astrabot.teleop");
    }
    ASSERT_TRUE(remote.waitForOpen(8s));
    ASSERT_TRUE(remote.waitForTracks(2U, 8s));
    EXPECT_TRUE(remote.reliabilityMatchesContract());

    const std::vector<std::uint8_t> quest_payload{1U, 2U, 3U, 4U};
    ASSERT_TRUE(remote.send(quest_payload));
    {
        std::unique_lock<std::mutex> lock(observation.mutex);
        ASSERT_TRUE(observation.condition.wait_for(lock, 5s, [&observation, &quest_payload]() {
            return observation.incoming_packet.has_value() && observation.incoming_packet->payload == quest_payload;
        }));
        EXPECT_EQ(observation.incoming_packet->session_id, "session-1");
        EXPECT_EQ(observation.incoming_packet->peer_id, "quest-1");
    }

    protocol::DataChannelPacket device_packet;
    device_packet.session_id = "session-1";
    device_packet.peer_id = "quest-1";
    device_packet.channel_label = "astrabot.teleop";
    device_packet.payload = {9U, 8U, 7U};
    ASSERT_TRUE(transport.sendDataChannelPacket(device_packet).ok());
    ASSERT_TRUE(remote.waitForMessage(device_packet.payload, 5s));

    EXPECT_TRUE(transport.sendEncodedFrame(makeEncodedFrame("right_eye")).ok());
    const auto metrics = transport.metrics();
    EXPECT_EQ(metrics.media_peer_sends, 1U);
    EXPECT_EQ(metrics.media_peer_bytes, makeEncodedFrame("right_eye")->data->size());
    EXPECT_EQ(metrics.media_teleop_congestion_drops, 0U);
    EXPECT_EQ(metrics.media_viewer_congestion_drops, 0U);
    EXPECT_EQ(metrics.media_buffer_query_failures, 0U);
    EXPECT_EQ(metrics.reconnect_count, 0U);

    remote.closeDataChannel();
    {
        std::unique_lock<std::mutex> lock(observation.mutex);
        EXPECT_TRUE(
            observation.condition.wait_for(lock, 5s, [&observation]() { return observation.data_channel_closed; }));
    }

    EXPECT_TRUE(transport.closePeer("session-1", "quest-1", "", "test_complete").ok());
    transport.stop();
    transport.stop();
    remote.stop();
}

TEST(LibDataChannelTransportTest, DoesNotReportDataChannelReadyBeforeSdkOpen) {
    LibDataChannelTransportSettings settings;
    settings.max_peers = 1U;
    settings.max_media_peers = 1U;
    settings.media_track_ids.clear();
    LibDataChannelWebRtcTransport transport(settings);

    std::mutex observation_mutex;
    bool authorization_requested = false;
    bool data_channel_ready = false;
    WebRtcTransportCallbacks callbacks;
    callbacks.on_data_channel_open = [&](DataChannelOpenRequest) {
        std::lock_guard<std::mutex> lock(observation_mutex);
        authorization_requested = true;
    };
    callbacks.on_data_channel_ready = [&](DataChannelReadyEvent) {
        std::lock_guard<std::mutex> lock(observation_mutex);
        data_channel_ready = true;
    };
    ASSERT_TRUE(transport.start(std::move(callbacks)).ok());
    ASSERT_TRUE(transport.handleSignaling(R"({"type":"registered","id":"robot-42","room":"room-robot-42"})").ok());
    ASSERT_TRUE(
        transport
            .handleSignaling(
                R"({"type":"peer_joined","from":"quest-1","peer_id":"quest-1","room":"room-robot-42","purpose":"teleop","session_id":"session-1","run_id":"run-1","resource_id":"thor","teleop_grant":"signed-grant","data_channel":{"label":"astrabot.teleop","ordered":false,"max_packet_lifetime_ms":20,"max_payload_bytes":16384}})")
            .ok());

    {
        std::lock_guard<std::mutex> lock(observation_mutex);
        EXPECT_TRUE(authorization_requested);
        EXPECT_FALSE(data_channel_ready);
    }
    EXPECT_TRUE(transport.closePeer("session-1", "quest-1", "", "test_complete").ok());
    transport.stop();
    transport.stop();
}

TEST(LibDataChannelTransportTest, QueuesRemoteCandidateUntilAnswerArrives) {
    LibDataChannelTransportSettings settings;
    settings.max_peers = 1U;
    settings.max_media_peers = 1U;
    settings.media_track_ids.clear();
    LibDataChannelWebRtcTransport transport(settings);

    ASSERT_TRUE(transport.start({}).ok());
    ASSERT_TRUE(transport.handleSignaling(R"({"type":"registered","id":"robot-42","room":"room-robot-42"})").ok());
    ASSERT_TRUE(
        transport
            .handleSignaling(
                R"({"type":"peer_joined","from":"quest-1","peer_id":"quest-1","room":"room-robot-42","purpose":"teleop","session_id":"session-1","run_id":"run-1","resource_id":"thor","teleop_grant":"signed-grant","data_channel":{"label":"astrabot.teleop","ordered":false,"max_packet_lifetime_ms":20,"max_payload_bytes":16384}})")
            .ok());

    const Status early_candidate = transport.handleSignaling(
        R"({"type":"signal","from":"quest-1","to":"robot-42","room":"room-robot-42","session_id":"session-1","signal":{"type":"candidate","candidate":"candidate:1 1 udp 1 192.0.2.1 5000 typ host","sdpMid":"0"}})");
    EXPECT_TRUE(early_candidate.ok()) << early_candidate.message();

    EXPECT_TRUE(transport.closePeer("session-1", "quest-1", "", "test_complete").ok());
    transport.stop();
}

TEST(LibDataChannelTransportTest, RepeatedCreateAndImmediateStopDoesNotLeakCallbacks) {
    LibDataChannelTransportSettings settings;
    settings.max_peers = 1U;
    settings.max_media_peers = 1U;
    settings.media_track_ids = {"right_eye"};
    LibDataChannelWebRtcTransport transport(settings);
    for (std::size_t iteration = 0U; iteration < 100U; ++iteration) {
        ASSERT_TRUE(transport.start({}).ok());
        ASSERT_TRUE(transport.handleSignaling(R"({"type":"registered","id":"robot-42","room":"room-robot-42"})").ok());
        const std::string joined =
            R"({"type":"peer_joined","from":"viewer-1","room":"room-robot-42","purpose":"video","session_id":"session-)" +
            std::to_string(iteration) + R"("})";
        ASSERT_TRUE(transport.handleSignaling(joined).ok());
        transport.stop();
        transport.stop();
    }
}

TEST(LibDataChannelTransportTest, AdmitsOnlyOneTeleopAndOneVideoMediaPeer) {
    LibDataChannelTransportSettings settings;
    settings.max_peers = 4U;
    settings.max_media_peers = 2U;
    settings.max_teleop_media_peers = 1U;
    settings.max_video_media_peers = 1U;
    settings.media_track_ids = {"right_eye"};
    LibDataChannelWebRtcTransport transport(settings);

    ASSERT_TRUE(transport.start({}).ok());
    ASSERT_TRUE(transport.handleSignaling(R"({"type":"registered","id":"robot-42","room":"room-robot-42"})").ok());
    ASSERT_TRUE(
        transport
            .handleSignaling(
                R"({"type":"peer_joined","from":"webops-1","peer_id":"webops-1","room":"room-robot-42","purpose":"video","media_tracks":["right_eye"]})")
            .ok());

    const Status second_video = transport.handleSignaling(
        R"({"type":"peer_joined","from":"webops-2","peer_id":"webops-2","room":"room-robot-42","purpose":"video","media_tracks":["right_eye"]})");
    EXPECT_EQ(second_video.code(), ErrorCode::kResourceExhausted);

    ASSERT_TRUE(
        transport
            .handleSignaling(
                R"({"type":"peer_joined","from":"quest-1","peer_id":"quest-1","room":"room-robot-42","purpose":"teleop","session_id":"session-1","run_id":"run-1","resource_id":"thor","media_tracks":["right_eye"],"teleop_grant":"signed-grant","data_channel":{"label":"astrabot.teleop","ordered":false,"max_packet_lifetime_ms":20,"max_payload_bytes":16384}})")
            .ok());

    const Status second_teleop = transport.handleSignaling(
        R"({"type":"peer_joined","from":"quest-2","peer_id":"quest-2","room":"room-robot-42","purpose":"teleop","session_id":"session-2","run_id":"run-1","resource_id":"thor","media_tracks":["right_eye"],"teleop_grant":"signed-grant","data_channel":{"label":"astrabot.teleop","ordered":false,"max_packet_lifetime_ms":20,"max_payload_bytes":16384}})");
    EXPECT_EQ(second_teleop.code(), ErrorCode::kResourceExhausted);

    transport.stop();
}

TEST(LibDataChannelTransportTest, TeleopPurposeLimitStillAppliesWithoutMediaTracks) {
    LibDataChannelTransportSettings settings;
    settings.max_peers = 4U;
    settings.media_track_ids.clear();
    LibDataChannelWebRtcTransport transport(settings);

    ASSERT_TRUE(transport.start({}).ok());
    ASSERT_TRUE(transport.handleSignaling(R"({"type":"registered","id":"robot-42","room":"room-robot-42"})").ok());
    ASSERT_TRUE(
        transport
            .handleSignaling(
                R"({"type":"peer_joined","from":"quest-1","peer_id":"quest-1","room":"room-robot-42","purpose":"teleop","session_id":"session-1","run_id":"run-1","resource_id":"thor","teleop_grant":"signed-grant","data_channel":{"label":"astrabot.teleop","ordered":false,"max_packet_lifetime_ms":20,"max_payload_bytes":16384}})")
            .ok());

    const Status second_teleop = transport.handleSignaling(
        R"({"type":"peer_joined","from":"quest-2","peer_id":"quest-2","room":"room-robot-42","purpose":"teleop","session_id":"session-2","run_id":"run-1","resource_id":"thor","teleop_grant":"signed-grant","data_channel":{"label":"astrabot.teleop","ordered":false,"max_packet_lifetime_ms":20,"max_payload_bytes":16384}})");
    EXPECT_EQ(second_teleop.code(), ErrorCode::kResourceExhausted);

    transport.stop();
}

TEST(LibDataChannelTransportTest, SessionBoundPeerRejectsSessionlessSignalsAndClose) {
    LibDataChannelTransportSettings settings;
    settings.max_peers = 1U;
    settings.max_media_peers = 1U;
    settings.media_track_ids.clear();
    LibDataChannelWebRtcTransport transport(settings);

    ASSERT_TRUE(transport.start({}).ok());
    ASSERT_TRUE(transport.handleSignaling(R"({"type":"registered","id":"robot-42","room":"room-robot-42"})").ok());
    ASSERT_TRUE(
        transport
            .handleSignaling(
                R"({"type":"peer_joined","from":"quest-1","peer_id":"quest-1","room":"room-robot-42","purpose":"teleop","session_id":"session-1","run_id":"run-1","resource_id":"thor","teleop_grant":"signed-grant","data_channel":{"label":"astrabot.teleop","ordered":false,"max_packet_lifetime_ms":20,"max_payload_bytes":16384}})")
            .ok());

    const Status sessionless_answer = transport.handleSignaling(
        R"({"type":"signal","from":"quest-1","to":"robot-42","room":"room-robot-42","signal":{"type":"answer","sdp":"v=0\r\n"}})");
    EXPECT_EQ(sessionless_answer.code(), ErrorCode::kNotFound);

    const Status sessionless_remote_close = transport.handleSignaling(
        R"({"type":"peer_left","from":"quest-1","room":"room-robot-42","message":"remote_peer_left"})");
    EXPECT_EQ(sessionless_remote_close.code(), ErrorCode::kPermissionDenied);
    EXPECT_EQ(transport.closePeer("", "quest-1", "", "local_close").code(), ErrorCode::kPermissionDenied);
    EXPECT_TRUE(transport.closePeer("session-1", "quest-1", "", "test_complete").ok());
    transport.stop();
}

TEST(LibDataChannelTransportTest, RejectsVideoViewerWhenMediaTracksAreDisabled) {
    LibDataChannelTransportSettings settings;
    settings.max_peers = 2U;
    settings.media_track_ids.clear();
    LibDataChannelWebRtcTransport transport(settings);

    ASSERT_TRUE(transport.start({}).ok());
    ASSERT_TRUE(transport.handleSignaling(R"({"type":"registered","id":"robot-42","room":"room-robot-42"})").ok());
    const Status status = transport.handleSignaling(
        R"({"type":"peer_joined","from":"webops-1","peer_id":"webops-1","room":"room-robot-42","purpose":"video"})");
    EXPECT_EQ(status.code(), ErrorCode::kFailedPrecondition);

    transport.stop();
}

TEST(LibDataChannelTransportTest, ConnectsPlatformVideoViewerWithoutSessionId) {
    LibDataChannelTransportSettings settings;
    settings.max_peers = 1U;
    settings.max_media_peers = 1U;
    settings.media_track_ids = {"right_eye"};
    LibDataChannelWebRtcTransport transport(settings);
    RemotePeer remote("webops-1", "robot-42", "room-robot-42", "");
    ASSERT_TRUE(remote.start(&transport));

    TransportObservation observation;
    WebRtcTransportCallbacks callbacks;
    callbacks.on_peer_event = [&observation](const session::PeerDescriptor &peer) {
        if (peer.peer_id == "webops-1" && peer.session_id.empty() && peer.state == session::PeerState::kConnected) {
            {
                std::lock_guard<std::mutex> lock(observation.mutex);
                observation.connected = true;
            }
            observation.condition.notify_all();
        }
    };
    callbacks.on_signaling_report = [&remote](std::string payload) { remote.handleDeviceSignal(payload); };
    ASSERT_TRUE(transport.start(std::move(callbacks)).ok());
    ASSERT_TRUE(transport.handleSignaling(R"({"type":"registered","id":"robot-42","room":"room-robot-42"})").ok());
    ASSERT_TRUE(
        transport
            .handleSignaling(
                R"({"type":"peer_joined","version":1,"from":"webops-1","peer_id":"webops-1","room":"room-robot-42","purpose":"video","media_tracks":["right_eye"]})")
            .ok());

    {
        std::unique_lock<std::mutex> lock(observation.mutex);
        ASSERT_TRUE(observation.condition.wait_for(lock, 8s, [&observation]() { return observation.connected; }));
    }
    ASSERT_TRUE(remote.waitForTracks(1U, 8s));
    EXPECT_TRUE(transport.closePeer("", "webops-1", "", "test_complete").ok());
    transport.stop();
    remote.stop();
}

TEST(LibDataChannelTransportTest, ReleasesPeerCapacityAfterDeferredClose) {
    LibDataChannelTransportSettings settings;
    settings.max_peers = 1U;
    settings.max_media_peers = 1U;
    settings.media_track_ids = {"right_eye"};
    LibDataChannelWebRtcTransport transport(settings);
    RemotePeer remote("webops-1", "robot-42", "room-robot-42", "");
    ASSERT_TRUE(remote.start(&transport));

    std::mutex observation_mutex;
    std::condition_variable observation_condition;
    bool connected = false;
    WebRtcTransportCallbacks callbacks;
    callbacks.on_peer_event = [&](const session::PeerDescriptor &peer) {
        {
            std::lock_guard<std::mutex> lock(observation_mutex);
            connected = connected || peer.state == session::PeerState::kConnected;
        }
        observation_condition.notify_all();
    };
    callbacks.on_signaling_report = [&remote](std::string payload) { remote.handleDeviceSignal(payload); };
    ASSERT_TRUE(transport.start(std::move(callbacks)).ok());
    ASSERT_TRUE(transport.handleSignaling(R"({"type":"registered","id":"robot-42","room":"room-robot-42"})").ok());
    const std::string peer_joined =
        R"({"type":"peer_joined","version":1,"from":"webops-1","peer_id":"webops-1","room":"room-robot-42","purpose":"video","media_tracks":["right_eye"]})";
    ASSERT_TRUE(transport.handleSignaling(peer_joined).ok());
    {
        std::unique_lock<std::mutex> lock(observation_mutex);
        ASSERT_TRUE(observation_condition.wait_for(lock, 8s, [&connected]() { return connected; }));
    }

    ASSERT_TRUE(transport.closePeer("", "webops-1", "", "test_recreate").ok());
    remote.stop();

    Status recreated = Status::error(ErrorCode::kResourceExhausted, "retired peer is still being cleaned");
    for (std::size_t attempt = 0U; attempt < 100U; ++attempt) {
        recreated = transport.handleSignaling(peer_joined);
        if (recreated.ok()) {
            break;
        }
        ASSERT_EQ(recreated.code(), ErrorCode::kResourceExhausted);
        std::this_thread::sleep_for(10ms);
    }
    ASSERT_TRUE(recreated.ok()) << recreated.message();
    EXPECT_TRUE(transport.closePeer("", "webops-1", "", "test_complete").ok());
    transport.stop();
}

TEST(LibDataChannelTransportTest, DropsCongestedViewerFrameWithoutBuildingMediaBacklog) {
    LibDataChannelTransportSettings settings;
    settings.max_peers = 1U;
    settings.max_media_peers = 1U;
    settings.max_media_buffered_amount_bytes = 8U;
    settings.media_track_ids = {"right_eye"};
    LibDataChannelWebRtcTransport transport(settings);
    RemotePeer remote("webops-1", "robot-42", "room-robot-42", "");
    ASSERT_TRUE(remote.start(&transport));

    TransportObservation observation;
    WebRtcTransportCallbacks callbacks;
    callbacks.on_peer_event = [&observation](const session::PeerDescriptor &peer) {
        if (peer.peer_id == "webops-1" && peer.state == session::PeerState::kConnected) {
            {
                std::lock_guard<std::mutex> lock(observation.mutex);
                observation.connected = true;
            }
            observation.condition.notify_all();
        }
    };
    callbacks.on_signaling_report = [&remote](std::string payload) { remote.handleDeviceSignal(payload); };
    ASSERT_TRUE(transport.start(std::move(callbacks)).ok());
    ASSERT_TRUE(transport.handleSignaling(R"({"type":"registered","id":"robot-42","room":"room-robot-42"})").ok());
    ASSERT_TRUE(
        transport
            .handleSignaling(
                R"({"type":"peer_joined","version":1,"from":"webops-1","peer_id":"webops-1","room":"room-robot-42","purpose":"video","media_tracks":["right_eye"]})")
            .ok());

    {
        std::unique_lock<std::mutex> lock(observation.mutex);
        ASSERT_TRUE(observation.condition.wait_for(lock, 8s, [&observation]() { return observation.connected; }));
    }
    ASSERT_TRUE(remote.waitForTracks(1U, 8s));

    Status send_status = Status::error(ErrorCode::kUnavailable, "viewer track is not open yet");
    for (std::size_t attempt = 0U; attempt < 100U; ++attempt) {
        send_status = transport.sendEncodedFrame(makeEncodedFrame("right_eye"));
        if (transport.metrics().media_viewer_congestion_drops > 0U) {
            break;
        }
        std::this_thread::sleep_for(10ms);
    }
    EXPECT_EQ(send_status.code(), ErrorCode::kUnavailable);
    const auto metrics = transport.metrics();
    EXPECT_EQ(metrics.media_peer_sends, 0U);
    EXPECT_EQ(metrics.media_peer_bytes, 0U);
    EXPECT_EQ(metrics.media_teleop_congestion_drops, 0U);
    EXPECT_EQ(metrics.media_viewer_congestion_drops, 1U);
    EXPECT_EQ(metrics.media_buffer_query_failures, 0U);
    EXPECT_EQ(metrics.reconnect_count, 0U);

    EXPECT_TRUE(transport.closePeer("", "webops-1", "", "test_complete").ok());
    transport.stop();
    remote.stop();
}

TEST(LibDataChannelTransportTest, RejectsZeroMediaBufferLimit) {
    LibDataChannelTransportSettings settings;
    settings.max_media_buffered_amount_bytes = 0U;
    LibDataChannelWebRtcTransport transport(settings);

    const Status status = transport.start({});
    EXPECT_EQ(status.code(), ErrorCode::kInvalidArgument);
}

TEST(LibDataChannelTransportTest, RejectsMediaPeerLimitAbovePeerCapacity) {
    LibDataChannelTransportSettings settings;
    settings.max_peers = 1U;
    settings.max_media_peers = 2U;
    LibDataChannelWebRtcTransport transport(settings);

    const Status status = transport.start({});
    EXPECT_EQ(status.code(), ErrorCode::kInvalidArgument);
}

}  // namespace
}  // namespace astrabot::rtc::transport
