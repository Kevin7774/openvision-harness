#include "astrabot_rtc/transport/webrtc/peer_reconnect_tracker.h"

#include <algorithm>
#include <utility>

namespace astrabot::rtc::transport {

PeerReconnectTracker::PeerReconnectTracker(std::size_t max_bindings) : max_bindings_(max_bindings) {}

Status PeerReconnectTracker::registerBinding(const std::string &binding_key) {
    if (binding_key.empty() || max_bindings_ == 0U) {
        return Status::error(ErrorCode::kInvalidArgument, "peer reconnect binding or capacity is invalid");
    }
    if (active_bindings_.count(binding_key) != 0U) {
        return Status::error(ErrorCode::kAlreadyExists, "peer reconnect binding is already active");
    }
    if (active_bindings_.size() >= max_bindings_) {
        return Status::error(ErrorCode::kResourceExhausted, "peer reconnect active binding limit reached");
    }
    BindingState state;
    state.reconnect_pending = eligible_bindings_.count(binding_key) != 0U;
    active_bindings_.emplace(binding_key, state);
    return Status::success();
}

void PeerReconnectTracker::unregisterBinding(const std::string &binding_key) {
    const auto binding = active_bindings_.find(binding_key);
    if (binding == active_bindings_.end()) {
        return;
    }
    if (!binding->second.reconnect_pending) {
        forgetEligible(binding_key);
    }
    active_bindings_.erase(binding);
}

bool PeerReconnectTracker::observeState(const std::string &binding_key, session::PeerState state) {
    const auto binding = active_bindings_.find(binding_key);
    if (binding == active_bindings_.end()) {
        return false;
    }
    BindingState &binding_state = binding->second;
    if (state == session::PeerState::kConnected) {
        const bool reconnected = binding_state.reconnect_pending;
        binding_state.ever_connected = true;
        binding_state.reconnect_pending = false;
        forgetEligible(binding_key);
        return reconnected;
    }
    if ((state == session::PeerState::kDisconnected || state == session::PeerState::kFailed) &&
        binding_state.ever_connected) {
        binding_state.reconnect_pending = true;
        rememberEligible(binding_key);
    }
    return false;
}

void PeerReconnectTracker::clear() {
    active_bindings_.clear();
    eligible_bindings_.clear();
    eligible_order_.clear();
}

void PeerReconnectTracker::rememberEligible(const std::string &binding_key) {
    if (!eligible_bindings_.insert(binding_key).second) {
        return;
    }
    eligible_order_.push_back(binding_key);
    while (eligible_bindings_.size() > max_bindings_ && !eligible_order_.empty()) {
        const std::string oldest = std::move(eligible_order_.front());
        eligible_order_.pop_front();
        eligible_bindings_.erase(oldest);
    }
}

void PeerReconnectTracker::forgetEligible(const std::string &binding_key) {
    eligible_bindings_.erase(binding_key);
    eligible_order_.erase(std::remove(eligible_order_.begin(), eligible_order_.end(), binding_key),
                          eligible_order_.end());
}

}  // namespace astrabot::rtc::transport
