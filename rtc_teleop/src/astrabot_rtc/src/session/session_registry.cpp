#include "astrabot_rtc/session/session_registry.h"

#include <string>
#include <utility>

namespace astrabot::rtc::session {
namespace {

void appendKeyPart(std::string &output, const std::string &part) {
    output += std::to_string(part.size());
    output.push_back(':');
    output += part;
}

}  // namespace

SessionRegistry::SessionRegistry(std::size_t max_peers, std::size_t max_viewers)
    : max_peers_(max_peers), max_viewers_(max_viewers) {}

Status SessionRegistry::addPeer(const PeerDescriptor &peer) {
    if (peer.peer_id.empty()) {
        return Status::error(ErrorCode::kInvalidArgument, "peer id must not be empty");
    }
    const bool video_only_without_session = peer.session_id.empty() && peer.purpose == "video" && peer.run_id.empty() &&
                                            peer.resource_id.empty() && peer.data_channels.empty();
    if (peer.session_id.empty() && !video_only_without_session) {
        return Status::error(ErrorCode::kInvalidArgument, "only a video-only peer may omit its application session id");
    }
    const std::string key = makeKey(peer.session_id, peer.peer_id);
    std::lock_guard<std::mutex> lock(mutex_);
    if (peers_.count(key) != 0U) {
        return Status::error(ErrorCode::kAlreadyExists, "peer is already registered");
    }
    if (peers_.size() >= max_peers_) {
        return Status::error(ErrorCode::kResourceExhausted, "peer registry reached its configured limit");
    }
    const bool is_viewer = !peer.media_tracks.empty();
    if (is_viewer && viewer_count_ >= max_viewers_) {
        return Status::error(ErrorCode::kResourceExhausted, "viewer registry reached its configured limit");
    }
    peers_.emplace(key, peer);
    if (is_viewer) {
        ++viewer_count_;
    }
    return Status::success();
}

Status SessionRegistry::updatePeer(const PeerDescriptor &peer) {
    const std::string key = makeKey(peer.session_id, peer.peer_id);
    std::lock_guard<std::mutex> lock(mutex_);
    const auto iterator = peers_.find(key);
    if (iterator == peers_.end()) {
        return Status::error(ErrorCode::kNotFound, "peer is not registered");
    }
    const auto &current = iterator->second;
    if (current.purpose != peer.purpose || current.run_id != peer.run_id || current.resource_id != peer.resource_id ||
        current.media_tracks != peer.media_tracks) {
        return Status::error(ErrorCode::kFailedPrecondition,
                             "peer immutable identity or media binding changed during its lifetime");
    }
    iterator->second.data_channels = peer.data_channels;
    iterator->second.state = peer.state;
    iterator->second.steady_time_ns = peer.steady_time_ns;
    iterator->second.reason_code = peer.reason_code;
    return Status::success();
}

Status SessionRegistry::updatePeerState(const std::string &session_id, const std::string &peer_id, PeerState state,
                                        std::uint64_t steady_time_ns, const std::string &reason_code) {
    const std::string key = makeKey(session_id, peer_id);
    std::lock_guard<std::mutex> lock(mutex_);
    const auto iterator = peers_.find(key);
    if (iterator == peers_.end()) {
        return Status::error(ErrorCode::kNotFound, "peer is not registered");
    }
    iterator->second.state = state;
    iterator->second.steady_time_ns = steady_time_ns;
    iterator->second.reason_code = reason_code;
    return Status::success();
}

Status SessionRegistry::removePeer(const std::string &session_id, const std::string &peer_id) {
    const std::string key = makeKey(session_id, peer_id);
    std::lock_guard<std::mutex> lock(mutex_);
    const auto iterator = peers_.find(key);
    if (iterator == peers_.end()) {
        return Status::error(ErrorCode::kNotFound, "peer is not registered");
    }
    if (!iterator->second.media_tracks.empty()) {
        --viewer_count_;
    }
    peers_.erase(iterator);
    return Status::success();
}

std::optional<PeerDescriptor> SessionRegistry::findPeer(const std::string &session_id,
                                                        const std::string &peer_id) const {
    const std::string key = makeKey(session_id, peer_id);
    std::lock_guard<std::mutex> lock(mutex_);
    const auto iterator = peers_.find(key);
    if (iterator == peers_.end()) {
        return std::nullopt;
    }
    return iterator->second;
}

std::vector<PeerDescriptor> SessionRegistry::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<PeerDescriptor> peers;
    peers.reserve(peers_.size());
    for (const auto &entry : peers_) {
        peers.push_back(entry.second);
    }
    return peers;
}

std::size_t SessionRegistry::peerCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return peers_.size();
}

std::size_t SessionRegistry::viewerCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return viewer_count_;
}

void SessionRegistry::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    peers_.clear();
    viewer_count_ = 0U;
}

std::string SessionRegistry::makeKey(const std::string &session_id, const std::string &peer_id) {
    std::string peer_key;
    peer_key.reserve(session_id.size() + peer_id.size() + 24U);
    appendKeyPart(peer_key, session_id);
    appendKeyPart(peer_key, peer_id);
    return peer_key;
}

}  // namespace astrabot::rtc::session
