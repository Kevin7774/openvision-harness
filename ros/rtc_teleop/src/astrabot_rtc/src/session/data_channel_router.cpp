#include "astrabot_rtc/session/data_channel_router.h"

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

DataChannelRouter::DataChannelRouter(std::size_t max_channels, std::size_t max_payload_bytes,
                                     std::shared_ptr<const IClock> clock)
    : max_channels_(max_channels), contract_(max_payload_bytes),
      clock_(clock ? std::move(clock) : std::make_shared<SteadyClock>()) {}

Status DataChannelRouter::start() {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = true;
    return Status::success();
}

void DataChannelRouter::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
    channels_.clear();
}

Status DataChannelRouter::authorize(const DataChannelKey &key, const ChannelAuthorization &authorization) {
    if (key.session_id.empty() || key.peer_id.empty() || key.channel_label.empty()) {
        return Status::error(ErrorCode::kInvalidArgument, "data channel authorization has an empty route identity");
    }
    if (!authorization.allowed) {
        return Status::error(ErrorCode::kPermissionDenied, authorization.reason_code.empty()
                                                               ? "data channel authorization denied"
                                                               : authorization.reason_code);
    }
    if (authorization.expires_at == 0U || authorization.expires_at <= clock_->nowNanoseconds()) {
        return Status::error(ErrorCode::kPermissionDenied,
                             "data channel authorization deadline is missing or already expired");
    }

    const std::string route_key = makeKey(key);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
        return Status::error(ErrorCode::kFailedPrecondition, "data channel router is stopped");
    }
    const auto iterator = channels_.find(route_key);
    if (iterator != channels_.end()) {
        iterator->second.expires_at = authorization.expires_at;
        iterator->second.latest.reset();
        return Status::success();
    }
    if (channels_.size() >= max_channels_) {
        return Status::error(ErrorCode::kResourceExhausted, "authorized data channel limit reached");
    }
    channels_.emplace(route_key, AuthorizedChannel{key, authorization.expires_at, std::nullopt});
    return Status::success();
}

bool DataChannelRouter::isAuthorized(const DataChannelKey &key) const {
    const std::string route_key = makeKey(key);
    std::lock_guard<std::mutex> lock(mutex_);
    const auto iterator = channels_.find(route_key);
    return running_ && iterator != channels_.end() && iterator->second.expires_at > clock_->nowNanoseconds();
}

Status DataChannelRouter::revoke(const DataChannelKey &key) {
    const std::string route_key = makeKey(key);
    std::lock_guard<std::mutex> lock(mutex_);
    if (channels_.erase(route_key) == 0U) {
        return Status::error(ErrorCode::kNotFound, "data channel authorization was not found");
    }
    return Status::success();
}

std::size_t DataChannelRouter::revokePeer(const std::string &session_id, const std::string &peer_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::size_t revoked_count = 0U;
    for (auto iterator = channels_.begin(); iterator != channels_.end();) {
        if (iterator->second.key.session_id == session_id && iterator->second.key.peer_id == peer_id) {
            iterator = channels_.erase(iterator);
            ++revoked_count;
        } else {
            ++iterator;
        }
    }
    return revoked_count;
}

Status DataChannelRouter::pushIncoming(protocol::DataChannelPacket packet) {
    const Status packet_status = contract_.validate(packet);
    if (!packet_status.ok()) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (packet_status.code() == ErrorCode::kPayloadTooLarge) {
            ++metrics_.oversized_packets;
        }
        return packet_status;
    }

    const std::string route_key = makeKey(packetKey(packet));
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
        ++metrics_.stopped_packets;
        return Status::error(ErrorCode::kFailedPrecondition, "data channel router is stopped");
    }
    const auto iterator = channels_.find(route_key);
    if (iterator == channels_.end()) {
        ++metrics_.unauthorized_packets;
        return Status::error(ErrorCode::kPermissionDenied, "data channel is not authorized");
    }
    if (iterator->second.expires_at <= clock_->nowNanoseconds()) {
        channels_.erase(iterator);
        ++metrics_.unauthorized_packets;
        return Status::error(ErrorCode::kPermissionDenied, "data channel authorization expired");
    }
    if (iterator->second.latest.has_value()) {
        ++metrics_.overwritten_packets;
    }
    iterator->second.latest = std::move(packet);
    ++metrics_.accepted_packets;
    return Status::success();
}

std::optional<protocol::DataChannelPacket> DataChannelRouter::takeLatest(const DataChannelKey &key) {
    const std::string route_key = makeKey(key);
    std::lock_guard<std::mutex> lock(mutex_);
    const auto iterator = channels_.find(route_key);
    if (iterator == channels_.end()) {
        return std::nullopt;
    }
    if (iterator->second.expires_at <= clock_->nowNanoseconds()) {
        if (iterator->second.latest.has_value()) {
            ++metrics_.expired_queued_packets;
        }
        channels_.erase(iterator);
        return std::nullopt;
    }
    if (!iterator->second.latest.has_value()) {
        return std::nullopt;
    }
    auto packet = std::move(iterator->second.latest);
    iterator->second.latest.reset();
    return packet;
}

std::vector<protocol::DataChannelPacket> DataChannelRouter::takeAllLatest() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<protocol::DataChannelPacket> packets;
    packets.reserve(channels_.size());
    const std::uint64_t now_ns = clock_->nowNanoseconds();
    for (auto iterator = channels_.begin(); iterator != channels_.end();) {
        if (iterator->second.expires_at <= now_ns) {
            if (iterator->second.latest.has_value()) {
                ++metrics_.expired_queued_packets;
            }
            iterator = channels_.erase(iterator);
            continue;
        }
        if (iterator->second.latest.has_value()) {
            packets.push_back(std::move(*iterator->second.latest));
            iterator->second.latest.reset();
        }
        ++iterator;
    }
    return packets;
}

DataChannelRouterMetrics DataChannelRouter::metrics() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return metrics_;
}

std::size_t DataChannelRouter::authorizedChannelCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return channels_.size();
}

std::string DataChannelRouter::makeKey(const DataChannelKey &key) {
    std::string route_key;
    route_key.reserve(key.session_id.size() + key.peer_id.size() + key.channel_label.size() + 32U);
    appendKeyPart(route_key, key.session_id);
    appendKeyPart(route_key, key.peer_id);
    appendKeyPart(route_key, key.channel_label);
    return route_key;
}

DataChannelKey DataChannelRouter::packetKey(const protocol::DataChannelPacket &packet) {
    return DataChannelKey{packet.session_id, packet.peer_id, packet.channel_label};
}

}  // namespace astrabot::rtc::session
