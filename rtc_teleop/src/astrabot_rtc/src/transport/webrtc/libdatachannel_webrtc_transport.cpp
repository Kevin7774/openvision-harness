#include "astrabot_rtc/transport/webrtc/libdatachannel_webrtc_transport.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rtc/rtc.h>

#include "astrabot_rtc/protocol/data_channel_contract.h"
#include "astrabot_rtc/transport/webrtc/libdatachannel_signaling.h"
#include "astrabot_rtc/transport/webrtc/peer_reconnect_tracker.h"

namespace astrabot::rtc::transport {
namespace {

constexpr std::uint8_t kFirstVideoPayloadType = 102U;
constexpr std::uint32_t kVideoClockRate = 90000U;
constexpr std::size_t kMaxEncodedFrameBytes = 8U * 1024U * 1024U;
constexpr std::size_t kMaxPendingRemoteCandidates = 64U;
constexpr const char *kVideoCname = "astrabot-video";

std::uint64_t steadyNowNanoseconds() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

std::string makePeerKey(const std::string &session_id, const std::string &peer_id) {
    std::string peer_key;
    peer_key.reserve(session_id.size() + peer_id.size() + 24U);
    peer_key += std::to_string(session_id.size());
    peer_key.push_back(':');
    peer_key += session_id;
    peer_key += std::to_string(peer_id.size());
    peer_key.push_back(':');
    peer_key += peer_id;
    return peer_key;
}

bool isUriUnreserved(unsigned char character) {
    return (character >= 'a' && character <= 'z') || (character >= 'A' && character <= 'Z') ||
           (character >= '0' && character <= '9') || character == '-' || character == '.' || character == '_' ||
           character == '~';
}

std::string percentEncode(const std::string &value) {
    constexpr char kHex[] = "0123456789ABCDEF";
    std::string encoded;
    encoded.reserve(value.size());
    for (const unsigned char character : value) {
        if (isUriUnreserved(character)) {
            encoded.push_back(static_cast<char>(character));
        } else {
            encoded.push_back('%');
            encoded.push_back(kHex[(character >> 4U) & 0x0FU]);
            encoded.push_back(kHex[character & 0x0FU]);
        }
    }
    return encoded;
}

Result<std::string> makeIceUrl(const std::string &url, const std::string &username, const std::string &credential) {
    if (username.empty() || url.compare(0U, 5U, "stun:") == 0 || url.compare(0U, 6U, "stuns:") == 0) {
        return Result<std::string>::success(url);
    }
    const std::size_t scheme_separator = url.find(':');
    if (scheme_separator == std::string::npos || url.find('@', scheme_separator + 1U) != std::string::npos ||
        (url.compare(0U, 5U, "turn:") != 0 && url.compare(0U, 6U, "turns:") != 0)) {
        return Result<std::string>::failure(
            Status::error(ErrorCode::kInvalidArgument, "credentialed ICE URL is not a plain TURN URL"));
    }
    return Result<std::string>::success(url.substr(0U, scheme_separator + 1U) + percentEncode(username) + ':' +
                                        percentEncode(credential) + '@' + url.substr(scheme_separator + 1U));
}

std::uint32_t stableSsrc(const std::string &peer_key, const std::string &track_id) {
    std::uint32_t hash = 2166136261U;
    for (const unsigned char character : peer_key + '\x1f' + track_id) {
        hash ^= character;
        hash *= 16777619U;
    }
    return hash == 0U ? 1U : hash;
}

session::PeerState peerState(rtcState state) {
    switch (state) {
        case RTC_NEW:
        case RTC_CONNECTING:
            return session::PeerState::kConnecting;
        case RTC_CONNECTED:
            return session::PeerState::kConnected;
        case RTC_DISCONNECTED:
            return session::PeerState::kDisconnected;
        case RTC_FAILED:
            return session::PeerState::kFailed;
        case RTC_CLOSED:
            return session::PeerState::kClosed;
    }
    return session::PeerState::kUnknown;
}

std::string peerReason(rtcState state) {
    switch (state) {
        case RTC_NEW:
            return "peer_new";
        case RTC_CONNECTING:
            return "peer_connecting";
        case RTC_CONNECTED:
            return "peer_connected";
        case RTC_DISCONNECTED:
            return "peer_disconnected";
        case RTC_FAILED:
            return "peer_failed";
        case RTC_CLOSED:
            return "peer_closed";
    }
    return "peer_unknown";
}

std::uint32_t rtpTimestamp(std::uint64_t capture_time_ns) {
    constexpr std::uint64_t kNanosecondsPerSecond = 1000000000U;
    const std::uint64_t seconds = capture_time_ns / kNanosecondsPerSecond;
    const std::uint64_t remainder = capture_time_ns % kNanosecondsPerSecond;
    return static_cast<std::uint32_t>(seconds * kVideoClockRate + remainder * kVideoClockRate / kNanosecondsPerSecond);
}

bool isH264Codec(const std::string &codec) {
    return codec == "h264" || codec == "H264" || codec == "video/h264";
}

std::string safeReasonCode(const std::string &reason_code, const char *fallback) {
    if (reason_code.empty() || reason_code.size() > 128U ||
        !std::all_of(reason_code.begin(), reason_code.end(), [](unsigned char character) {
            return std::isalnum(character) != 0 || character == '_' || character == '-' || character == '.' ||
                   character == ':';
        })) {
        return fallback;
    }
    return reason_code;
}

class CallbackStateRegistry final {
  public:
    static CallbackStateRegistry &instance() {
        static CallbackStateRegistry registry;
        return registry;
    }

    void *add(const std::shared_ptr<void> &state) {
        const std::uintptr_t token = next_token_.fetch_add(1U);
        void *key = reinterpret_cast<void *>(token);
        std::lock_guard<std::mutex> lock(mutex_);
        states_[key] = state;
        return key;
    }

    std::shared_ptr<void> find(void *key) const {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto iterator = states_.find(key);
        return iterator == states_.end() ? std::shared_ptr<void>{} : iterator->second.lock();
    }

    void remove(void *key) {
        std::lock_guard<std::mutex> lock(mutex_);
        states_.erase(key);
    }

  private:
    mutable std::mutex mutex_;
    std::unordered_map<void *, std::weak_ptr<void>> states_;
    std::atomic<std::uintptr_t> next_token_{1U};
};

}  // namespace

class LibDataChannelWebRtcTransport::Implementation final {
  public:
    explicit Implementation(LibDataChannelTransportSettings settings)
        : state_(std::make_shared<SharedState>(std::move(settings))) {}

    ~Implementation() {
        stop();
    }

    Status start(WebRtcTransportCallbacks callbacks) {
        const Status settings_status = validateSettings();
        if (!settings_status.ok()) {
            return settings_status;
        }
        // TURN credential 会以 URI 形式交给 SDK；显式关闭其全局日志，避免第三方内部输出带凭据的 ICE URL。
        rtcInitLogger(RTC_LOG_NONE, nullptr);
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            if (state_->running) {
                return Status::success();
            }
            state_->callbacks = std::move(callbacks);
            state_->cleanup_stopping = false;
            state_->running = true;
            state_->callback_token = CallbackStateRegistry::instance().add(state_);
        }
        cleanup_worker_ = std::thread([state = state_]() { cleanupLoop(std::move(state)); });
        return Status::success();
    }

    void stop() {
        std::vector<std::shared_ptr<PeerContext>> peers;
        void *callback_token = nullptr;
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            if (!state_->running && state_->callback_token == nullptr) {
                return;
            }
            state_->running = false;
            callback_token = state_->callback_token;
            state_->callback_token = nullptr;
            peers.reserve(state_->peers.size());
            for (const auto &entry : state_->peers) {
                peers.push_back(entry.second);
            }
            state_->peers.clear();
            state_->peer_keys_by_pc.clear();
            state_->peer_keys_by_remote_id.clear();
            state_->channels_by_id.clear();
            state_->reconnect_tracker.clear();
        }
        if (callback_token != nullptr) {
            CallbackStateRegistry::instance().remove(callback_token);
        }
        {
            std::unique_lock<std::mutex> lock(state_->mutex);
            state_->callbacks_drained.wait(lock, [this]() { return state_->active_callbacks == 0U; });
            state_->callbacks = {};
            state_->cleanup_stopping = true;
        }
        state_->cleanup_ready.notify_all();
        if (cleanup_worker_.joinable()) {
            cleanup_worker_.join();
        }
        for (const auto &peer : peers) {
            destroyPeer(peer);
        }
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            state_->local_peer_id.clear();
            state_->room.clear();
            state_->ice_servers.clear();
        }
    }

    Status handleSignaling(const std::string &payload) {
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            if (!state_->running) {
                return Status::error(ErrorCode::kFailedPrecondition, "libdatachannel transport is stopped");
            }
        }
        auto parsed = state_->signaling_codec->parse(payload);
        if (!parsed.ok()) {
            return parsed.status();
        }
        WebRtcSignalingCommand command = parsed.takeValue();
        const Status target_status = validateSignaledTarget(command.target_peer_id);
        if (!target_status.ok() && command.type != WebRtcSignalingCommandType::kRegistered) {
            return target_status;
        }
        switch (command.type) {
            case WebRtcSignalingCommandType::kRegistered:
                return handleRegistered(std::move(command));
            case WebRtcSignalingCommandType::kPeerJoined:
                return createPeer(std::move(command.peer_join), command.room);
            case WebRtcSignalingCommandType::kPeerLeft:
                if (!validateSignaledRoom(command.room).ok()) {
                    return Status::error(ErrorCode::kPermissionDenied,
                                         "WebRTC peer close room does not match registration");
                }
                return closePeerInternal(command.session_id, command.peer_id, "",
                                         command.reason_code.empty() ? "remote_peer_left" : command.reason_code, false);
            case WebRtcSignalingCommandType::kRemoteOffer:
                return setRemoteDescription(command, "offer");
            case WebRtcSignalingCommandType::kRemoteAnswer:
                return setRemoteDescription(command, "answer");
            case WebRtcSignalingCommandType::kRemoteCandidate:
                return addRemoteCandidate(command);
        }
        return Status::error(ErrorCode::kInvalidArgument, "WebRTC signaling command is unsupported");
    }

    Status closePeer(const std::string &session_id, const std::string &peer_id, const std::string &channel_label,
                     const std::string &reason_code) {
        return closePeerInternal(session_id, peer_id, channel_label, safeReasonCode(reason_code, "local_close"), true);
    }

    Status sendDataChannelPacket(const protocol::DataChannelPacket &packet) {
        protocol::DataChannelContract contract(state_->settings.max_payload_bytes);
        const Status contract_status = contract.validate(packet);
        if (!contract_status.ok()) {
            return contract_status;
        }
        std::shared_ptr<PeerContext> peer;
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            if (!state_->running) {
                return Status::error(ErrorCode::kFailedPrecondition, "libdatachannel transport is stopped");
            }
            const auto iterator = state_->peers.find(makePeerKey(packet.session_id, packet.peer_id));
            if (iterator == state_->peers.end()) {
                return Status::error(ErrorCode::kNotFound, "DataChannel peer was not found");
            }
            peer = iterator->second;
        }

        std::lock_guard<std::mutex> sdk_lock(peer->sdk_mutex);
        if (peer->closing || peer->retired) {
            return Status::error(ErrorCode::kFailedPrecondition, "DataChannel peer is closing");
        }
        const auto channel = peer->channels.find(packet.channel_label);
        if (channel == peer->channels.end() || !rtcIsOpen(channel->second)) {
            return Status::error(ErrorCode::kUnavailable, "DataChannel is not open");
        }
        const int buffered_amount = rtcGetBufferedAmount(channel->second);
        if (buffered_amount < 0) {
            return sdkError("get DataChannel buffered amount", buffered_amount);
        }
        const std::size_t buffered = static_cast<std::size_t>(buffered_amount);
        if (buffered > state_->settings.max_buffered_amount_bytes ||
            packet.payload.size() > state_->settings.max_buffered_amount_bytes - buffered) {
            return Status::error(ErrorCode::kResourceExhausted, "DataChannel buffered amount limit reached");
        }
        if (packet.payload.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
            return Status::error(ErrorCode::kPayloadTooLarge, "DataChannel payload cannot be represented by the SDK");
        }
        const int result = rtcSendMessage(channel->second, reinterpret_cast<const char *>(packet.payload.data()),
                                          static_cast<int>(packet.payload.size()));
        return result >= 0 ? Status::success() : sdkError("send DataChannel payload", result);
    }

    Status sendEncodedFrame(std::shared_ptr<const media::EncodedVideoFrame> frame) {
        if (!frame || frame->track_id.empty() || !frame->data || !isH264Codec(frame->codec)) {
            return Status::error(ErrorCode::kInvalidArgument, "encoded frame is not a complete H264 sample");
        }
        if (frame->data->empty() || frame->data->size() > kMaxEncodedFrameBytes ||
            frame->data->size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
            return Status::error(ErrorCode::kPayloadTooLarge, "encoded H264 frame is empty or exceeds 8 MiB");
        }

        std::vector<std::shared_ptr<PeerContext>> peers;
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            if (!state_->running) {
                return Status::error(ErrorCode::kFailedPrecondition, "libdatachannel transport is stopped");
            }
            peers.reserve(state_->peers.size());
            for (const auto &entry : state_->peers) {
                peers.push_back(entry.second);
            }
        }
        std::stable_sort(peers.begin(), peers.end(), [](const auto &lhs, const auto &rhs) {
            const bool lhs_is_teleop = lhs->descriptor.purpose == "teleop";
            const bool rhs_is_teleop = rhs->descriptor.purpose == "teleop";
            return lhs_is_teleop && !rhs_is_teleop;
        });

        const std::uint32_t frame_rtp_timestamp = rtpTimestamp(steadyNowNanoseconds());
        std::size_t sent_count = 0U;
        for (const auto &peer : peers) {
            std::unique_lock<std::mutex> sdk_lock(peer->sdk_mutex, std::defer_lock);
            if (peer->descriptor.purpose == "video") {
                if (!sdk_lock.try_lock()) {
                    state_->media_viewer_congestion_drops.fetch_add(1U, std::memory_order_relaxed);
                    continue;
                }
            } else {
                sdk_lock.lock();
            }
            if (peer->closing || peer->retired) {
                continue;
            }
            const auto track = peer->tracks.find(frame->track_id);
            if (track == peer->tracks.end() || !rtcIsOpen(track->second)) {
                continue;
            }
            const int buffered_amount = rtcGetBufferedAmount(track->second);
            if (buffered_amount < 0) {
                state_->media_buffer_query_failures.fetch_add(1U, std::memory_order_relaxed);
                continue;
            }
            const std::size_t buffered = static_cast<std::size_t>(buffered_amount);
            if (buffered > state_->settings.max_media_buffered_amount_bytes ||
                frame->data->size() > state_->settings.max_media_buffered_amount_bytes - buffered) {
                if (peer->descriptor.purpose == "teleop") {
                    state_->media_teleop_congestion_drops.fetch_add(1U, std::memory_order_relaxed);
                } else {
                    state_->media_viewer_congestion_drops.fetch_add(1U, std::memory_order_relaxed);
                }
                continue;
            }
            // ROS header 使用系统时钟且可能缺失或回跳；RTP 必须使用单调时钟，capture_time_ns 只保留给延迟观测。
            const int timestamp_result = rtcSetTrackRtpTimestamp(track->second, frame_rtp_timestamp);
            if (timestamp_result < 0) {
                continue;
            }
            const int result = rtcSendMessage(track->second, reinterpret_cast<const char *>(frame->data->data()),
                                              static_cast<int>(frame->data->size()));
            if (result >= 0) {
                ++sent_count;
                state_->media_peer_sends.fetch_add(1U, std::memory_order_relaxed);
                state_->media_peer_bytes.fetch_add(frame->data->size(), std::memory_order_relaxed);
            }
        }
        return sent_count > 0U ? Status::success()
                               : Status::error(ErrorCode::kUnavailable, "no open peer track accepted the H264 frame");
    }

    WebRtcTransportCapabilities capabilities() const {
        return WebRtcTransportCapabilities{"libdatachannel", true, !state_->settings.media_track_ids.empty(), true};
    }

    WebRtcTransportMetrics metrics() const {
        WebRtcTransportMetrics snapshot;
        snapshot.media_peer_sends = state_->media_peer_sends.load(std::memory_order_relaxed);
        snapshot.media_peer_bytes = state_->media_peer_bytes.load(std::memory_order_relaxed);
        snapshot.media_teleop_congestion_drops = state_->media_teleop_congestion_drops.load(std::memory_order_relaxed);
        snapshot.media_viewer_congestion_drops = state_->media_viewer_congestion_drops.load(std::memory_order_relaxed);
        snapshot.media_buffer_query_failures = state_->media_buffer_query_failures.load(std::memory_order_relaxed);
        snapshot.reconnect_count = state_->reconnect_count.load(std::memory_order_relaxed);
        return snapshot;
    }

  private:
    struct PeerContext {
        struct RemoteCandidate {
            std::string candidate;
            std::string mid;
        };

        mutable std::mutex sdk_mutex;
        bool closing{false};
        bool retired{false};
        bool remote_description_set{false};
        std::atomic<bool> local_description_reported{false};
        std::atomic<bool> cleanup_permitted{false};
        int pc{-1};
        session::PeerDescriptor descriptor;
        std::string local_peer_id;
        std::string room;
        std::unordered_map<std::string, int> channels;
        std::unordered_map<std::string, int> tracks;
        std::deque<RemoteCandidate> pending_remote_candidates;
    };

    struct ChannelRoute {
        std::weak_ptr<PeerContext> peer;
        std::string label;
        bool open{false};
    };

    struct SharedState {
        explicit SharedState(LibDataChannelTransportSettings configured_settings)
            : settings(std::move(configured_settings)),
              signaling_codec(std::make_shared<LibDataChannelSignalingCodec>(settings.max_signaling_payload_bytes)),
              reconnect_tracker(settings.max_peers) {}

        mutable std::mutex mutex;
        std::condition_variable callbacks_drained;
        std::condition_variable cleanup_ready;
        bool running{false};
        bool cleanup_stopping{false};
        std::size_t active_callbacks{0U};
        void *callback_token{nullptr};
        WebRtcTransportCallbacks callbacks;
        LibDataChannelTransportSettings settings;
        std::shared_ptr<const LibDataChannelSignalingCodec> signaling_codec;
        std::string local_peer_id;
        std::string room;
        std::vector<WebRtcIceServerSettings> ice_servers;
        std::unordered_map<std::string, std::shared_ptr<PeerContext>> peers;
        std::unordered_map<int, std::string> peer_keys_by_pc;
        std::unordered_map<std::string, std::string> peer_keys_by_remote_id;
        std::unordered_map<int, ChannelRoute> channels_by_id;
        std::deque<std::shared_ptr<PeerContext>> cleanup_queue;
        PeerReconnectTracker reconnect_tracker;
        std::atomic<std::uint64_t> media_peer_sends{0U};
        std::atomic<std::uint64_t> media_peer_bytes{0U};
        std::atomic<std::uint64_t> media_teleop_congestion_drops{0U};
        std::atomic<std::uint64_t> media_viewer_congestion_drops{0U};
        std::atomic<std::uint64_t> media_buffer_query_failures{0U};
        std::atomic<std::uint64_t> reconnect_count{0U};
    };

    static void retirePeerLocked(SharedState &state, const std::string &peer_key,
                                 const std::shared_ptr<PeerContext> &peer) {
        state.reconnect_tracker.unregisterBinding(peer_key);
        state.peers.erase(peer_key);
        state.peer_keys_by_pc.erase(peer->pc);
        state.peer_keys_by_remote_id.erase(peer->descriptor.peer_id);
        for (const auto &channel : peer->channels) {
            state.channels_by_id.erase(channel.second);
        }
        state.cleanup_queue.push_back(peer);
    }

    class CallbackLease final {
      public:
        explicit CallbackLease(std::shared_ptr<SharedState> state) : state_(std::move(state)) {}

        CallbackLease(CallbackLease &&other) noexcept : state_(std::move(other.state_)) {}
        CallbackLease &operator=(CallbackLease &&) = delete;
        CallbackLease(const CallbackLease &) = delete;
        CallbackLease &operator=(const CallbackLease &) = delete;

        ~CallbackLease() {
            if (!state_) {
                return;
            }
            {
                std::lock_guard<std::mutex> lock(state_->mutex);
                --state_->active_callbacks;
            }
            state_->callbacks_drained.notify_all();
            state_->cleanup_ready.notify_all();
        }

        SharedState *operator->() const {
            return state_.get();
        }

        const std::shared_ptr<SharedState> &state() const {
            return state_;
        }

      private:
        std::shared_ptr<SharedState> state_;
    };

    static std::optional<CallbackLease> acquireCallback(void *pointer) {
        auto opaque_state = CallbackStateRegistry::instance().find(pointer);
        if (!opaque_state) {
            return std::nullopt;
        }
        auto state = std::static_pointer_cast<SharedState>(std::move(opaque_state));
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            if (!state->running) {
                return std::nullopt;
            }
            ++state->active_callbacks;
        }
        return CallbackLease(std::move(state));
    }

    static Status sdkError(const char *operation, int result) {
        return Status::error(ErrorCode::kInternal, std::string("libdatachannel failed to ") + operation +
                                                       " with code " + std::to_string(result));
    }

    Status validateSettings() const {
        if (state_->settings.max_peers == 0U || state_->settings.max_payload_bytes == 0U ||
            state_->settings.max_media_peers == 0U || state_->settings.max_media_peers > state_->settings.max_peers ||
            state_->settings.max_teleop_media_peers == 0U ||
            state_->settings.max_teleop_media_peers > state_->settings.max_media_peers ||
            state_->settings.max_video_media_peers == 0U ||
            state_->settings.max_video_media_peers > state_->settings.max_media_peers ||
            state_->settings.max_media_peers >
                state_->settings.max_teleop_media_peers + state_->settings.max_video_media_peers ||
            state_->settings.max_payload_bytes > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
            state_->settings.max_signaling_payload_bytes == 0U ||
            state_->settings.max_buffered_amount_bytes < state_->settings.max_payload_bytes ||
            state_->settings.max_media_buffered_amount_bytes == 0U ||
            state_->settings.max_media_buffered_amount_bytes > kMaxEncodedFrameBytes ||
            state_->settings.media_track_ids.size() > 8U) {
            return Status::error(ErrorCode::kInvalidArgument, "libdatachannel transport settings are outside limits");
        }
        const std::set<std::string> unique_tracks(state_->settings.media_track_ids.begin(),
                                                  state_->settings.media_track_ids.end());
        if (unique_tracks.size() != state_->settings.media_track_ids.size() || unique_tracks.count("") != 0U) {
            return Status::error(ErrorCode::kInvalidArgument,
                                 "libdatachannel media track ids must be non-empty and unique");
        }
        return Status::success();
    }

    Status validatePeerAdmissionLocked(const std::string &purpose, bool has_media_tracks) const {
        if (purpose != "teleop" && purpose != "video") {
            return Status::error(ErrorCode::kInvalidArgument, "WebRTC peer purpose is unsupported");
        }

        std::size_t media_peer_count = 0U;
        std::size_t purpose_peer_count = 0U;
        for (const auto &entry : state_->peers) {
            if (!entry.second->descriptor.media_tracks.empty()) {
                ++media_peer_count;
            }
            if (entry.second->descriptor.purpose == purpose) {
                ++purpose_peer_count;
            }
        }
        for (const auto &peer : state_->cleanup_queue) {
            if (!peer->descriptor.media_tracks.empty()) {
                ++media_peer_count;
            }
            if (peer->descriptor.purpose == purpose) {
                ++purpose_peer_count;
            }
        }
        if (has_media_tracks && media_peer_count >= state_->settings.max_media_peers) {
            return Status::error(ErrorCode::kResourceExhausted, "WebRTC media peer limit reached");
        }
        const std::size_t purpose_limit =
            purpose == "teleop" ? state_->settings.max_teleop_media_peers : state_->settings.max_video_media_peers;
        if (purpose_peer_count >= purpose_limit) {
            return Status::error(ErrorCode::kResourceExhausted, "WebRTC peer purpose limit reached");
        }
        return Status::success();
    }

    Status validateSignaledRoom(const std::string &room) const {
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (!room.empty() && !state_->room.empty() && room != state_->room) {
            return Status::error(ErrorCode::kPermissionDenied, "WebRTC signaling room does not match registration");
        }
        return Status::success();
    }

    Status validateSignaledTarget(const std::string &target_peer_id) const {
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (!target_peer_id.empty() && !state_->local_peer_id.empty() && target_peer_id != state_->local_peer_id) {
            return Status::error(ErrorCode::kPermissionDenied, "WebRTC signaling target does not match registration");
        }
        return Status::success();
    }

    Status handleRegistered(WebRtcSignalingCommand command) {
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (!state_->running) {
            return Status::error(ErrorCode::kFailedPrecondition, "libdatachannel transport is stopped");
        }
        if (!state_->peers.empty() &&
            ((!state_->local_peer_id.empty() && state_->local_peer_id != command.local_peer_id) ||
             (!state_->room.empty() && state_->room != command.room))) {
            return Status::error(ErrorCode::kFailedPrecondition,
                                 "registered identity cannot change while peers are active");
        }
        state_->local_peer_id = std::move(command.local_peer_id);
        state_->room = std::move(command.room);
        state_->ice_servers = std::move(command.ice_servers);
        return Status::success();
    }

    Result<std::vector<std::string>> selectedTracks(const WebRtcPeerJoinSettings &settings) const {
        std::vector<std::string> selected = settings.media_tracks;
        if (selected.empty()) {
            if (settings.purpose == "video") {
                const auto right_eye = std::find(state_->settings.media_track_ids.begin(),
                                                 state_->settings.media_track_ids.end(), "right_eye");
                if (right_eye != state_->settings.media_track_ids.end()) {
                    selected.push_back(*right_eye);
                } else if (!state_->settings.media_track_ids.empty()) {
                    selected.push_back(state_->settings.media_track_ids.front());
                }
            } else {
                selected = state_->settings.media_track_ids;
            }
        }
        for (const auto &track_id : selected) {
            if (std::find(state_->settings.media_track_ids.begin(), state_->settings.media_track_ids.end(), track_id) ==
                state_->settings.media_track_ids.end()) {
                return Result<std::vector<std::string>>::failure(
                    Status::error(ErrorCode::kInvalidArgument, "peer requested a media track not configured on RTC"));
            }
        }
        return Result<std::vector<std::string>>::success(std::move(selected));
    }

    Result<std::vector<std::string>> iceUrls() const {
        std::vector<WebRtcIceServerSettings> configured_servers;
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            configured_servers = state_->ice_servers;
        }
        std::vector<std::string> urls;
        for (const auto &server : configured_servers) {
            for (const auto &url : server.urls) {
                auto configured_url = makeIceUrl(url, server.username, server.credential);
                if (!configured_url.ok()) {
                    return Result<std::vector<std::string>>::failure(configured_url.status());
                }
                urls.push_back(configured_url.takeValue());
            }
        }
        return Result<std::vector<std::string>>::success(std::move(urls));
    }

    Status createPeer(WebRtcPeerJoinSettings settings, const std::string &signaled_room) {
        std::string local_peer_id;
        std::string room;
        void *callback_token = nullptr;
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            if (!state_->running) {
                return Status::error(ErrorCode::kFailedPrecondition, "libdatachannel transport is stopped");
            }
            if (state_->local_peer_id.empty() || state_->room.empty()) {
                return Status::error(ErrorCode::kFailedPrecondition,
                                     "WebRTC registered identity must arrive before peer_joined");
            }
            if (!signaled_room.empty() && signaled_room != state_->room) {
                return Status::error(ErrorCode::kPermissionDenied,
                                     "WebRTC peer_joined room does not match registration");
            }
            if (state_->peers.size() + state_->cleanup_queue.size() >= state_->settings.max_peers) {
                return Status::error(ErrorCode::kResourceExhausted, "WebRTC peer limit reached");
            }
            if (state_->peers.count(makePeerKey(settings.session_id, settings.peer_id)) != 0U ||
                state_->peer_keys_by_remote_id.count(settings.peer_id) != 0U) {
                return Status::error(ErrorCode::kAlreadyExists, "WebRTC peer identity is already active");
            }
            local_peer_id = state_->local_peer_id;
            room = state_->room;
            callback_token = state_->callback_token;
        }

        auto tracks = selectedTracks(settings);
        if (!tracks.ok()) {
            return tracks.status();
        }
        if (settings.purpose == "video" && tracks.value().empty()) {
            return Status::error(ErrorCode::kFailedPrecondition,
                                 "WebRTC video viewer requires at least one configured media track");
        }
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            const Status admission_status = validatePeerAdmissionLocked(settings.purpose, !tracks.value().empty());
            if (!admission_status.ok()) {
                return admission_status;
            }
        }
        auto ice_urls_result = iceUrls();
        if (!ice_urls_result.ok()) {
            return ice_urls_result.status();
        }
        if (settings.data_channel.has_value() &&
            settings.data_channel->max_payload_bytes > state_->settings.max_payload_bytes) {
            return Status::error(ErrorCode::kInvalidArgument,
                                 "signaled DataChannel payload limit exceeds RTC transport configuration");
        }
        std::vector<std::string> ice_urls = ice_urls_result.takeValue();
        std::vector<const char *> ice_url_pointers;
        ice_url_pointers.reserve(ice_urls.size());
        for (const auto &url : ice_urls) {
            ice_url_pointers.push_back(url.c_str());
        }

        rtcConfiguration configuration{};
        configuration.iceServers = ice_url_pointers.empty() ? nullptr : ice_url_pointers.data();
        configuration.iceServersCount = static_cast<int>(ice_url_pointers.size());
        configuration.bindAddress =
            state_->settings.bind_address.empty() ? nullptr : state_->settings.bind_address.c_str();
        configuration.disableAutoNegotiation = true;
        configuration.maxMessageSize = static_cast<int>(state_->settings.max_payload_bytes);
        const int pc = rtcCreatePeerConnection(&configuration);
        if (pc < 0) {
            return sdkError("create PeerConnection", pc);
        }

        auto peer = std::make_shared<PeerContext>();
        peer->pc = pc;
        peer->local_peer_id = local_peer_id;
        peer->room = room;
        peer->descriptor.session_id = settings.session_id;
        peer->descriptor.peer_id = settings.peer_id;
        peer->descriptor.purpose = settings.purpose;
        peer->descriptor.run_id = settings.run_id;
        peer->descriptor.resource_id = settings.resource_id;
        peer->descriptor.media_tracks = tracks.takeValue();
        peer->descriptor.state = session::PeerState::kConnecting;
        peer->descriptor.steady_time_ns = steadyNowNanoseconds();
        peer->descriptor.reason_code = "peer_joined";
        if (settings.data_channel.has_value()) {
            peer->descriptor.data_channels.push_back(settings.data_channel->label);
        }

        rtcSetUserPointer(pc, callback_token);
        if (rtcSetLocalDescriptionCallback(pc, &Implementation::onLocalDescription) < 0 ||
            rtcSetLocalCandidateCallback(pc, &Implementation::onLocalCandidate) < 0 ||
            rtcSetStateChangeCallback(pc, &Implementation::onPeerState) < 0 ||
            rtcSetDataChannelCallback(pc, &Implementation::onIncomingDataChannel) < 0) {
            destroyPeer(peer);
            return Status::error(ErrorCode::kInternal, "libdatachannel failed to install PeerConnection callbacks");
        }

        const std::string peer_key = makePeerKey(settings.session_id, settings.peer_id);
        std::size_t track_index = 0U;
        for (const auto &track_id : peer->descriptor.media_tracks) {
            rtcTrackInit track_init{};
            track_init.direction = RTC_DIRECTION_SENDONLY;
            track_init.codec = RTC_CODEC_H264;
            track_init.payloadType = static_cast<int>(kFirstVideoPayloadType + track_index);
            track_init.ssrc = stableSsrc(peer_key, track_id);
            track_init.mid = track_id.c_str();
            track_init.name = kVideoCname;
            track_init.msid = "astrabot-camera";
            track_init.trackId = track_id.c_str();
            const int track = rtcAddTrackEx(pc, &track_init);
            if (track < 0) {
                destroyPeer(peer);
                return sdkError("add H264 track", track);
            }
            rtcPacketizerInit packetizer{};
            packetizer.ssrc = track_init.ssrc;
            packetizer.cname = kVideoCname;
            packetizer.payloadType = static_cast<std::uint8_t>(track_init.payloadType);
            packetizer.clockRate = kVideoClockRate;
            packetizer.nalSeparator = RTC_NAL_SEPARATOR_START_SEQUENCE;
            if (rtcSetH264Packetizer(track, &packetizer) < 0) {
                rtcDeleteTrack(track);
                destroyPeer(peer);
                return Status::error(ErrorCode::kInternal, "libdatachannel failed to configure H264 packetizer");
            }
            peer->tracks.emplace(track_id, track);
            ++track_index;
        }

        int data_channel_id = -1;
        if (settings.data_channel.has_value()) {
            rtcDataChannelInit channel_init{};
            channel_init.reliability.unordered = !settings.data_channel->ordered;
            channel_init.reliability.unreliable = true;
            channel_init.reliability.maxPacketLifeTime = settings.data_channel->max_packet_lifetime_ms;
            data_channel_id = rtcCreateDataChannelEx(pc, settings.data_channel->label.c_str(), &channel_init);
            if (data_channel_id < 0) {
                destroyPeer(peer);
                return sdkError("create DataChannel", data_channel_id);
            }
            rtcSetUserPointer(data_channel_id, callback_token);
            if (rtcSetOpenCallback(data_channel_id, &Implementation::onDataChannelOpen) < 0 ||
                rtcSetClosedCallback(data_channel_id, &Implementation::onDataChannelClosed) < 0 ||
                rtcSetErrorCallback(data_channel_id, &Implementation::onDataChannelError) < 0 ||
                rtcSetMessageCallback(data_channel_id, &Implementation::onDataChannelMessage) < 0) {
                rtcDeleteDataChannel(data_channel_id);
                destroyPeer(peer);
                return Status::error(ErrorCode::kInternal, "libdatachannel failed to install DataChannel callbacks");
            }
            peer->channels.emplace(settings.data_channel->label, data_channel_id);
        }

        bool reservation_failed = false;
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            const Status peer_admission_status =
                validatePeerAdmissionLocked(peer->descriptor.purpose, !peer->descriptor.media_tracks.empty());
            if (!state_->running || state_->peers.size() + state_->cleanup_queue.size() >= state_->settings.max_peers ||
                state_->peers.count(peer_key) != 0U || state_->peer_keys_by_remote_id.count(settings.peer_id) != 0U ||
                !peer_admission_status.ok()) {
                reservation_failed = true;
            } else {
                const Status reconnect_status = state_->reconnect_tracker.registerBinding(peer_key);
                if (!reconnect_status.ok()) {
                    reservation_failed = true;
                } else {
                    state_->peers.emplace(peer_key, peer);
                    state_->peer_keys_by_pc.emplace(pc, peer_key);
                    state_->peer_keys_by_remote_id.emplace(settings.peer_id, peer_key);
                    if (data_channel_id >= 0) {
                        state_->channels_by_id.emplace(data_channel_id,
                                                       ChannelRoute{peer, settings.data_channel->label, false});
                    }
                }
            }
        }
        if (reservation_failed) {
            destroyPeer(peer);
            return Status::error(ErrorCode::kResourceExhausted,
                                 "WebRTC peer reservation changed while PeerConnection was being created");
        }

        emitPeerEvent(peer->descriptor);
        int offer_result = RTC_ERR_NOT_AVAIL;
        {
            std::lock_guard<std::mutex> sdk_lock(peer->sdk_mutex);
            if (!peer->closing) {
                offer_result = rtcSetLocalDescription(pc, "offer");
            }
        }
        if (offer_result < 0) {
            const Status close_status =
                closePeerInternal(settings.session_id, settings.peer_id, "", "local_offer_failed", false);
            (void)close_status;
            return sdkError("set local offer", offer_result);
        }
        // libdatachannel 在指定 bindAddress 后，部分 libjuice 版本可能先回调 candidate，却不及时回调
        // localDescription。此时平台只看到 ICE 而没有 offer，远端必然超时。设置本地描述后同步读取一次作为
        // 有界兜底；原回调与此路径通过原子标志保证最多上报一次。
        reportCurrentLocalDescription(state_, peer, "offer");

        if (settings.data_channel.has_value()) {
            DataChannelOpenRequest request;
            request.session_id = settings.session_id;
            request.peer_id = settings.peer_id;
            request.purpose = settings.purpose;
            request.run_id = settings.run_id;
            request.resource_id = settings.resource_id;
            request.channel_label = settings.data_channel->label;
            request.authorization_token = std::move(settings.authorization_token);
            WebRtcTransportCallbacks callbacks;
            {
                std::lock_guard<std::mutex> lock(state_->mutex);
                callbacks = state_->callbacks;
            }
            if (callbacks.on_data_channel_open) {
                callbacks.on_data_channel_open(std::move(request));
            }
        }
        return Status::success();
    }

    std::shared_ptr<PeerContext> findPeer(const std::string &session_id, const std::string &peer_id) const {
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (!session_id.empty()) {
            const auto direct = state_->peers.find(makePeerKey(session_id, peer_id));
            return direct == state_->peers.end() ? std::shared_ptr<PeerContext>{} : direct->second;
        }
        const auto indexed = state_->peer_keys_by_remote_id.find(peer_id);
        if (indexed == state_->peer_keys_by_remote_id.end()) {
            return {};
        }
        const auto peer = state_->peers.find(indexed->second);
        if (peer == state_->peers.end() || !peer->second->descriptor.session_id.empty()) {
            return {};
        }
        return peer->second;
    }

    Status setRemoteDescription(const WebRtcSignalingCommand &command, const char *description_type) {
        const Status room_status = validateSignaledRoom(command.room);
        if (!room_status.ok()) {
            return room_status;
        }
        auto peer = findPeer(command.session_id, command.peer_id);
        if (!peer) {
            return Status::error(ErrorCode::kNotFound, "WebRTC peer for remote description was not found");
        }
        std::lock_guard<std::mutex> sdk_lock(peer->sdk_mutex);
        if (peer->closing || peer->retired) {
            return Status::error(ErrorCode::kFailedPrecondition, "WebRTC peer is closing");
        }
        int result = rtcSetRemoteDescription(peer->pc, command.sdp.c_str(), description_type);
        if (result < 0) {
            return sdkError("set remote description", result);
        }
        peer->remote_description_set = true;
        while (!peer->pending_remote_candidates.empty()) {
            const PeerContext::RemoteCandidate &candidate = peer->pending_remote_candidates.front();
            result = rtcAddRemoteCandidate(peer->pc, candidate.candidate.c_str(),
                                           candidate.mid.empty() ? nullptr : candidate.mid.c_str());
            if (result < 0) {
                return sdkError("add queued remote ICE candidate", result);
            }
            peer->pending_remote_candidates.pop_front();
        }
        if (std::string(description_type) == "offer") {
            result = rtcSetLocalDescription(peer->pc, "answer");
            if (result < 0) {
                return sdkError("set local answer", result);
            }
        }
        return Status::success();
    }

    Status addRemoteCandidate(const WebRtcSignalingCommand &command) {
        const Status room_status = validateSignaledRoom(command.room);
        if (!room_status.ok()) {
            return room_status;
        }
        auto peer = findPeer(command.session_id, command.peer_id);
        if (!peer) {
            return Status::error(ErrorCode::kNotFound, "WebRTC peer for remote candidate was not found");
        }
        std::lock_guard<std::mutex> sdk_lock(peer->sdk_mutex);
        if (peer->closing || peer->retired) {
            return Status::error(ErrorCode::kFailedPrecondition, "WebRTC peer is closing");
        }
        if (!peer->remote_description_set) {
            if (peer->pending_remote_candidates.size() >= kMaxPendingRemoteCandidates) {
                return Status::error(ErrorCode::kResourceExhausted, "WebRTC pending remote candidate limit reached");
            }
            peer->pending_remote_candidates.push_back(
                PeerContext::RemoteCandidate{command.candidate, command.candidate_mid});
            return Status::success();
        }
        const int result =
            rtcAddRemoteCandidate(peer->pc, command.candidate.c_str(),
                                  command.candidate_mid.empty() ? nullptr : command.candidate_mid.c_str());
        return result >= 0 ? Status::success() : sdkError("add remote ICE candidate", result);
    }

    Status closePeerInternal(const std::string &session_id, const std::string &peer_id,
                             const std::string &channel_label, const std::string &reason_code, bool report) {
        std::shared_ptr<PeerContext> peer;
        std::string peer_key;
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            if (!state_->running) {
                return Status::error(ErrorCode::kFailedPrecondition, "libdatachannel transport is stopped");
            }
            if (!session_id.empty()) {
                peer_key = makePeerKey(session_id, peer_id);
            } else {
                const auto indexed = state_->peer_keys_by_remote_id.find(peer_id);
                if (indexed != state_->peer_keys_by_remote_id.end()) {
                    const auto indexed_peer = state_->peers.find(indexed->second);
                    if (indexed_peer != state_->peers.end() && !indexed_peer->second->descriptor.session_id.empty()) {
                        return Status::error(ErrorCode::kPermissionDenied,
                                             "session-bound WebRTC peer close requires its session id");
                    }
                    peer_key = indexed->second;
                }
            }
            const auto peer_iterator = state_->peers.find(peer_key);
            if (peer_iterator == state_->peers.end()) {
                return Status::error(ErrorCode::kNotFound, "WebRTC peer was not found");
            }
            peer = peer_iterator->second;
            if (channel_label.empty()) {
                retirePeerLocked(*state_, peer_key, peer);
            }
        }

        if (!channel_label.empty()) {
            session::PeerDescriptor descriptor;
            {
                std::lock_guard<std::mutex> sdk_lock(peer->sdk_mutex);
                if (peer->closing || peer->retired) {
                    return Status::success();
                }
                const auto channel = peer->channels.find(channel_label);
                if (channel == peer->channels.end()) {
                    return Status::error(ErrorCode::kNotFound, "WebRTC DataChannel was not found");
                }
                const int channel_id = channel->second;
                peer->channels.erase(channel);
                {
                    std::lock_guard<std::mutex> lock(state_->mutex);
                    state_->channels_by_id.erase(channel_id);
                    auto &data_channels = peer->descriptor.data_channels;
                    data_channels.erase(std::remove(data_channels.begin(), data_channels.end(), channel_label),
                                        data_channels.end());
                    peer->descriptor.steady_time_ns = steadyNowNanoseconds();
                    peer->descriptor.reason_code = reason_code;
                    descriptor = peer->descriptor;
                }
                rtcSetOpenCallback(channel_id, nullptr);
                rtcSetClosedCallback(channel_id, nullptr);
                rtcSetErrorCallback(channel_id, nullptr);
                rtcSetMessageCallback(channel_id, nullptr);
                rtcSetUserPointer(channel_id, nullptr);
                rtcClose(channel_id);
                rtcDeleteDataChannel(channel_id);
            }
            emitPeerEvent(descriptor);
            return Status::success();
        }

        if (report) {
            auto signaling_report = state_->signaling_codec->makePeerLeftReport(
                peer->local_peer_id, peer->room, peer->descriptor.session_id, peer->descriptor.peer_id, reason_code);
            if (signaling_report.ok()) {
                emitSignalingReport(signaling_report.takeValue());
            }
        }
        session::PeerDescriptor closed = peer->descriptor;
        closed.state = session::PeerState::kClosed;
        closed.steady_time_ns = steadyNowNanoseconds();
        closed.reason_code = reason_code;
        disablePeerCallbacks(peer);
        state_->cleanup_ready.notify_all();
        emitPeerEvent(closed);
        return Status::success();
    }

    static void destroyPeer(const std::shared_ptr<PeerContext> &peer) {
        if (!peer) {
            return;
        }
        std::lock_guard<std::mutex> sdk_lock(peer->sdk_mutex);
        if (peer->closing) {
            return;
        }
        peer->closing = true;
        for (const auto &channel : peer->channels) {
            rtcSetOpenCallback(channel.second, nullptr);
            rtcSetClosedCallback(channel.second, nullptr);
            rtcSetErrorCallback(channel.second, nullptr);
            rtcSetMessageCallback(channel.second, nullptr);
            rtcSetUserPointer(channel.second, nullptr);
            rtcClose(channel.second);
            rtcDeleteDataChannel(channel.second);
        }
        peer->channels.clear();
        for (const auto &track : peer->tracks) {
            rtcClose(track.second);
            rtcDeleteTrack(track.second);
        }
        peer->tracks.clear();
        if (peer->pc >= 0) {
            rtcSetLocalDescriptionCallback(peer->pc, nullptr);
            rtcSetLocalCandidateCallback(peer->pc, nullptr);
            rtcSetStateChangeCallback(peer->pc, nullptr);
            rtcSetDataChannelCallback(peer->pc, nullptr);
            rtcSetUserPointer(peer->pc, nullptr);
            rtcClosePeerConnection(peer->pc);
            rtcDeletePeerConnection(peer->pc);
            peer->pc = -1;
        }
    }

    static void disablePeerCallbacks(const std::shared_ptr<PeerContext> &peer) {
        if (!peer) {
            return;
        }
        std::lock_guard<std::mutex> sdk_lock(peer->sdk_mutex);
        peer->retired = true;
        for (const auto &channel : peer->channels) {
            rtcSetOpenCallback(channel.second, nullptr);
            rtcSetClosedCallback(channel.second, nullptr);
            rtcSetErrorCallback(channel.second, nullptr);
            rtcSetMessageCallback(channel.second, nullptr);
            rtcSetUserPointer(channel.second, nullptr);
        }
        if (peer->pc >= 0) {
            rtcSetLocalDescriptionCallback(peer->pc, nullptr);
            rtcSetLocalCandidateCallback(peer->pc, nullptr);
            rtcSetStateChangeCallback(peer->pc, nullptr);
            rtcSetDataChannelCallback(peer->pc, nullptr);
            rtcSetUserPointer(peer->pc, nullptr);
        }
        peer->cleanup_permitted.store(true, std::memory_order_release);
    }

    static void cleanupLoop(std::shared_ptr<SharedState> state) {
        while (true) {
            std::shared_ptr<PeerContext> peer;
            {
                std::unique_lock<std::mutex> lock(state->mutex);
                state->cleanup_ready.wait(lock, [&state]() {
                    return state->cleanup_stopping ||
                           (!state->cleanup_queue.empty() && state->active_callbacks == 0U &&
                            state->cleanup_queue.front()->cleanup_permitted.load(std::memory_order_acquire));
                });
                if (state->cleanup_stopping) {
                    std::deque<std::shared_ptr<PeerContext>> remaining;
                    remaining.swap(state->cleanup_queue);
                    lock.unlock();
                    for (const auto &queued_peer : remaining) {
                        destroyPeer(queued_peer);
                    }
                    return;
                }
                peer = state->cleanup_queue.front();
            }

            destroyPeer(peer);
            {
                std::lock_guard<std::mutex> lock(state->mutex);
                if (!state->cleanup_queue.empty() && state->cleanup_queue.front() == peer) {
                    state->cleanup_queue.pop_front();
                }
            }
        }
    }

    void emitPeerEvent(const session::PeerDescriptor &descriptor) const {
        WebRtcTransportCallbacks callbacks;
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            if (!state_->running) {
                return;
            }
            callbacks = state_->callbacks;
        }
        if (callbacks.on_peer_event) {
            callbacks.on_peer_event(descriptor);
        }
    }

    void emitSignalingReport(std::string payload) const {
        WebRtcTransportCallbacks callbacks;
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            if (!state_->running) {
                return;
            }
            callbacks = state_->callbacks;
        }
        if (callbacks.on_signaling_report) {
            callbacks.on_signaling_report(std::move(payload));
        }
    }

    static void reportLocalDescription(const std::shared_ptr<SharedState> &state,
                                       const std::shared_ptr<PeerContext> &peer, const std::string &type,
                                       const std::string &sdp) {
        auto report = state->signaling_codec->makeDescriptionReport(
            peer->local_peer_id, peer->room, peer->descriptor.session_id, peer->descriptor.peer_id, type, sdp);
        if (!report.ok()) {
            return;
        }
        WebRtcTransportCallbacks callbacks;
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            if (!state->running || !state->callbacks.on_signaling_report) {
                return;
            }
            callbacks = state->callbacks;
        }
        bool expected = false;
        if (peer->local_description_reported.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
            callbacks.on_signaling_report(report.takeValue());
        }
    }

    static void reportCurrentLocalDescription(const std::shared_ptr<SharedState> &state,
                                              const std::shared_ptr<PeerContext> &peer, const char *type) {
        const int required = rtcGetLocalDescription(peer->pc, nullptr, 0);
        if (required <= 1 || static_cast<std::size_t>(required) > state->settings.max_signaling_payload_bytes) {
            return;
        }
        std::vector<char> buffer(static_cast<std::size_t>(required));
        const int copied = rtcGetLocalDescription(peer->pc, buffer.data(), required);
        if (copied <= 1) {
            return;
        }
        reportLocalDescription(state, peer, type, std::string(buffer.data()));
    }

    static void RTC_API onLocalDescription(int pc, const char *sdp, const char *type, void *pointer) {
        auto lease = acquireCallback(pointer);
        if (!lease.has_value() || sdp == nullptr || type == nullptr) {
            return;
        }
        std::shared_ptr<PeerContext> peer;
        {
            std::lock_guard<std::mutex> lock(lease->state()->mutex);
            const auto key = lease->state()->peer_keys_by_pc.find(pc);
            if (key == lease->state()->peer_keys_by_pc.end()) {
                return;
            }
            const auto peer_iterator = lease->state()->peers.find(key->second);
            if (peer_iterator == lease->state()->peers.end()) {
                return;
            }
            peer = peer_iterator->second;
        }
        reportLocalDescription(lease->state(), peer, type, sdp);
    }

    static void RTC_API onLocalCandidate(int pc, const char *candidate, const char *mid, void *pointer) {
        auto lease = acquireCallback(pointer);
        if (!lease.has_value() || candidate == nullptr || candidate[0] == '\0') {
            return;
        }
        std::shared_ptr<PeerContext> peer;
        WebRtcTransportCallbacks callbacks;
        {
            std::lock_guard<std::mutex> lock(lease->state()->mutex);
            const auto key = lease->state()->peer_keys_by_pc.find(pc);
            if (key == lease->state()->peer_keys_by_pc.end()) {
                return;
            }
            const auto peer_iterator = lease->state()->peers.find(key->second);
            if (peer_iterator == lease->state()->peers.end()) {
                return;
            }
            peer = peer_iterator->second;
            callbacks = lease->state()->callbacks;
        }
        auto report = lease->state()->signaling_codec->makeCandidateReport(
            peer->local_peer_id, peer->room, peer->descriptor.session_id, peer->descriptor.peer_id, candidate,
            mid == nullptr ? std::string{} : std::string(mid));
        if (report.ok() && callbacks.on_signaling_report) {
            callbacks.on_signaling_report(report.takeValue());
        }
    }

    static void RTC_API onPeerState(int pc, rtcState state, void *pointer) {
        auto lease = acquireCallback(pointer);
        if (!lease.has_value()) {
            return;
        }
        session::PeerDescriptor descriptor;
        std::shared_ptr<PeerContext> retired_peer;
        WebRtcTransportCallbacks callbacks;
        {
            std::lock_guard<std::mutex> lock(lease->state()->mutex);
            const auto key = lease->state()->peer_keys_by_pc.find(pc);
            if (key == lease->state()->peer_keys_by_pc.end()) {
                return;
            }
            const auto peer = lease->state()->peers.find(key->second);
            if (peer == lease->state()->peers.end()) {
                return;
            }
            const session::PeerState mapped_state = peerState(state);
            peer->second->descriptor.state = mapped_state;
            peer->second->descriptor.steady_time_ns = steadyNowNanoseconds();
            peer->second->descriptor.reason_code = peerReason(state);
            if (lease->state()->reconnect_tracker.observeState(key->second, mapped_state)) {
                lease->state()->reconnect_count.fetch_add(1U, std::memory_order_relaxed);
            }
            descriptor = peer->second->descriptor;
            callbacks = lease->state()->callbacks;
            const bool retire_peer = mapped_state == session::PeerState::kDisconnected ||
                                     mapped_state == session::PeerState::kFailed ||
                                     mapped_state == session::PeerState::kClosed;
            if (retire_peer) {
                const std::string peer_key = key->second;
                retired_peer = peer->second;
                retirePeerLocked(*lease->state(), peer_key, retired_peer);
            }
        }
        if (retired_peer) {
            // SDK callback 内只注销回调；真正 delete 延后到当前及并发 callback 全部退出之后。
            disablePeerCallbacks(retired_peer);
            lease->state()->cleanup_ready.notify_all();
        }
        if (callbacks.on_peer_event) {
            callbacks.on_peer_event(descriptor);
        }
    }

    static void RTC_API onIncomingDataChannel(int pc, int dc, void *pointer) {
        (void)pc;
        auto lease = acquireCallback(pointer);
        if (!lease.has_value()) {
            rtcClose(dc);
            rtcDeleteDataChannel(dc);
            return;
        }
        // Phase 0 固定为 device-offer/device-created channel。远端重复创建 channel 默认拒绝。
        rtcSetUserPointer(dc, nullptr);
        rtcClose(dc);
        rtcDeleteDataChannel(dc);
    }

    static void RTC_API onDataChannelOpen(int dc, void *pointer) {
        auto lease = acquireCallback(pointer);
        if (!lease.has_value()) {
            return;
        }
        DataChannelReadyEvent event;
        WebRtcTransportCallbacks callbacks;
        bool publish_ready = false;
        {
            std::lock_guard<std::mutex> lock(lease->state()->mutex);
            const auto channel = lease->state()->channels_by_id.find(dc);
            if (channel == lease->state()->channels_by_id.end() || channel->second.open) {
                return;
            }
            const auto peer = channel->second.peer.lock();
            if (!peer || peer->retired || peer->closing) {
                return;
            }
            channel->second.open = true;
            event.session_id = peer->descriptor.session_id;
            event.peer_id = peer->descriptor.peer_id;
            event.channel_label = channel->second.label;
            callbacks = lease->state()->callbacks;
            publish_ready = true;
        }
        if (publish_ready && callbacks.on_data_channel_ready) {
            callbacks.on_data_channel_ready(std::move(event));
        }
    }

    static void RTC_API onDataChannelClosed(int dc, void *pointer) {
        auto lease = acquireCallback(pointer);
        if (!lease.has_value()) {
            return;
        }
        session::PeerDescriptor descriptor;
        WebRtcTransportCallbacks callbacks;
        bool publish_event = false;
        {
            std::lock_guard<std::mutex> lock(lease->state()->mutex);
            const auto channel = lease->state()->channels_by_id.find(dc);
            if (channel == lease->state()->channels_by_id.end()) {
                return;
            }
            const ChannelRoute route = channel->second;
            lease->state()->channels_by_id.erase(channel);
            const auto peer = route.peer.lock();
            if (!peer) {
                return;
            }
            auto &data_channels = peer->descriptor.data_channels;
            data_channels.erase(std::remove(data_channels.begin(), data_channels.end(), route.label),
                                data_channels.end());
            peer->descriptor.steady_time_ns = steadyNowNanoseconds();
            peer->descriptor.reason_code = "data_channel_closed";
            descriptor = peer->descriptor;
            callbacks = lease->state()->callbacks;
            publish_event = true;
        }
        if (publish_event && callbacks.on_peer_event) {
            callbacks.on_peer_event(descriptor);
        }
    }

    static void RTC_API onDataChannelError(int dc, const char *error, void *pointer) {
        (void)error;
        onDataChannelClosed(dc, pointer);
    }

    static void RTC_API onDataChannelMessage(int dc, const char *message, int size, void *pointer) {
        auto lease = acquireCallback(pointer);
        if (!lease.has_value() || message == nullptr) {
            return;
        }
        std::string session_id;
        std::string peer_id;
        std::string label;
        WebRtcTransportCallbacks callbacks;
        std::size_t payload_size = 0U;
        {
            std::lock_guard<std::mutex> lock(lease->state()->mutex);
            const auto channel = lease->state()->channels_by_id.find(dc);
            if (channel == lease->state()->channels_by_id.end() || !channel->second.open) {
                return;
            }
            const auto peer = channel->second.peer.lock();
            if (!peer) {
                return;
            }
            session_id = peer->descriptor.session_id;
            peer_id = peer->descriptor.peer_id;
            label = channel->second.label;
            callbacks = lease->state()->callbacks;
            if (size >= 0) {
                payload_size = static_cast<std::size_t>(size);
            } else {
                while (payload_size <= lease->state()->settings.max_payload_bytes && message[payload_size] != '\0') {
                    ++payload_size;
                }
            }
            if (payload_size > lease->state()->settings.max_payload_bytes) {
                return;
            }
        }
        if (!callbacks.on_data_channel_packet) {
            return;
        }
        protocol::DataChannelPacket packet;
        packet.session_id = std::move(session_id);
        packet.peer_id = std::move(peer_id);
        packet.channel_label = std::move(label);
        packet.receive_steady_time_ns = steadyNowNanoseconds();
        packet.payload.assign(reinterpret_cast<const std::uint8_t *>(message),
                              reinterpret_cast<const std::uint8_t *>(message) + payload_size);
        callbacks.on_data_channel_packet(std::move(packet));
    }

    std::shared_ptr<SharedState> state_;
    std::thread cleanup_worker_;
};

LibDataChannelWebRtcTransport::LibDataChannelWebRtcTransport(LibDataChannelTransportSettings settings)
    : implementation_(std::make_unique<Implementation>(std::move(settings))) {}

LibDataChannelWebRtcTransport::~LibDataChannelWebRtcTransport() {
    stop();
}

Status LibDataChannelWebRtcTransport::start(WebRtcTransportCallbacks callbacks) {
    return implementation_->start(std::move(callbacks));
}

void LibDataChannelWebRtcTransport::stop() {
    if (implementation_) {
        implementation_->stop();
    }
}

Status LibDataChannelWebRtcTransport::handleSignaling(const std::string &payload) {
    return implementation_->handleSignaling(payload);
}

Status LibDataChannelWebRtcTransport::closePeer(const std::string &session_id, const std::string &peer_id,
                                                const std::string &channel_label, const std::string &reason_code) {
    return implementation_->closePeer(session_id, peer_id, channel_label, reason_code);
}

Status LibDataChannelWebRtcTransport::sendDataChannelPacket(const protocol::DataChannelPacket &packet) {
    return implementation_->sendDataChannelPacket(packet);
}

Status LibDataChannelWebRtcTransport::sendEncodedFrame(std::shared_ptr<const media::EncodedVideoFrame> frame) {
    return implementation_->sendEncodedFrame(std::move(frame));
}

WebRtcTransportCapabilities LibDataChannelWebRtcTransport::capabilities() const {
    return implementation_->capabilities();
}

WebRtcTransportMetrics LibDataChannelWebRtcTransport::metrics() const {
    return implementation_->metrics();
}

}  // namespace astrabot::rtc::transport
