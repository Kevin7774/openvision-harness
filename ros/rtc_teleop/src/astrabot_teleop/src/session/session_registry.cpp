// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include "astrabot_teleop/session/session_registry.h"

#include <utility>

namespace astrabot::teleop {

Status SessionRegistry::authorize(SessionBinding binding, const bool allow_empty_run_id) {
    if (binding.session_id.empty() || binding.peer_id.empty() || (!allow_empty_run_id && binding.run_id.empty()) ||
        binding.resource_id.empty() || binding.channel_label.empty() ||
        binding.authorization_deadline_steady_ns == 0U || binding.grant_fingerprint.empty()) {
        return Status::error(ErrorCode::kInvalidArgument, "Teleop session binding is incomplete");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_.has_value()) {
        return Status::error(ErrorCode::kConflict, "another Teleop writer is already authorized");
    }
    current_ = std::move(binding);
    return Status::success();
}

Status SessionRegistry::markConnected(const std::string &session_id, const std::string &peer_id,
                                      const std::string &channel_label) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!current_.has_value() || current_->session_id != session_id || current_->peer_id != peer_id ||
        current_->channel_label != channel_label) {
        return Status::error(ErrorCode::kUnauthorized, "RTC DataChannel does not match the authorized Teleop writer");
    }
    current_->connected = true;
    return Status::success();
}

bool SessionRegistry::matches(const std::string &session_id, const std::string &peer_id,
                              const std::string &channel_label) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_.has_value() && current_->connected && current_->session_id == session_id &&
           current_->peer_id == peer_id && current_->channel_label == channel_label;
}

bool SessionRegistry::matchesAuthorization(const SessionBinding &binding) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_.has_value() && current_->session_id == binding.session_id && current_->peer_id == binding.peer_id &&
           current_->run_id == binding.run_id && current_->resource_id == binding.resource_id &&
           current_->channel_label == binding.channel_label && current_->grant_fingerprint == binding.grant_fingerprint;
}

std::optional<SessionBinding> SessionRegistry::current() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_;
}

Status SessionRegistry::close(const std::string &session_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!current_.has_value()) {
        return Status::success();
    }
    if (current_->session_id != session_id) {
        return Status::error(ErrorCode::kUnauthorized, "cannot close a different Teleop session");
    }
    current_.reset();
    return Status::success();
}

void SessionRegistry::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    current_.reset();
}

}  // namespace astrabot::teleop
