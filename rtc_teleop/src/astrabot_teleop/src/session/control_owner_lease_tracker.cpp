// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include "astrabot_teleop/session/control_owner_lease_tracker.h"

#include <limits>

namespace astrabot::teleop {
namespace {

bool validIdentity(const ControlOwnerIdentity &identity) {
    return !identity.session_id.empty() && !identity.run_id.empty() && !identity.resource_id.empty() &&
           !identity.source_id.empty();
}

bool sameIdentity(const ControlOwnerIdentity &first, const ControlOwnerIdentity &second) {
    return first.session_id == second.session_id && first.run_id == second.run_id &&
           first.resource_id == second.resource_id && first.source_id == second.source_id;
}

}  // namespace

Result<ControlOwnerOperation> ControlOwnerLeaseTracker::beginAcquire(const ControlOwnerIdentity &identity,
                                                                     const std::uint64_t steady_now_ns,
                                                                     const std::uint64_t timeout_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!validIdentity(identity) || timeout_ns == 0U) {
        return Result<ControlOwnerOperation>::failure(
            Status::error(ErrorCode::kInvalidArgument, "owner acquire identity or timeout is invalid"));
    }
    if (pending_.has_value() || lease_.has_value()) {
        return Result<ControlOwnerOperation>::failure(
            Status::error(ErrorCode::kConflict, "owner acquire conflicts with active lease or request"));
    }
    return makeOperationLocked(ControlOwnerOperationKind::kAcquire, identity, 0U, steady_now_ns, timeout_ns);
}

Status ControlOwnerLeaseTracker::completeAcquire(const ControlOwnerOperation &operation, const bool granted,
                                                 const std::uint64_t owner_epoch,
                                                 const std::uint64_t expires_at_steady_ns,
                                                 const std::uint64_t steady_now_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (operation.kind != ControlOwnerOperationKind::kAcquire || !matchesPendingLocked(operation)) {
        return Status::error(ErrorCode::kFailedPrecondition, "stale owner acquire callback");
    }
    pending_.reset();
    if (!granted) {
        return Status::error(ErrorCode::kUnauthorized, "owner acquire was rejected");
    }
    if (owner_epoch == 0U || expires_at_steady_ns <= steady_now_ns) {
        return Status::error(ErrorCode::kInvalidArgument, "owner acquire response has invalid lease");
    }
    lease_ = ControlOwnerLease{operation.identity, owner_epoch, expires_at_steady_ns};
    return Status::success();
}

Result<ControlOwnerOperation> ControlOwnerLeaseTracker::beginRenew(const std::uint64_t steady_now_ns,
                                                                   const std::uint64_t timeout_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (timeout_ns == 0U || pending_.has_value() || !lease_.has_value() ||
        lease_->expires_at_steady_ns <= steady_now_ns) {
        return Result<ControlOwnerOperation>::failure(
            Status::error(ErrorCode::kFailedPrecondition, "owner lease cannot be renewed"));
    }
    return makeOperationLocked(ControlOwnerOperationKind::kRenew, lease_->identity, lease_->owner_epoch, steady_now_ns,
                               timeout_ns);
}

Status ControlOwnerLeaseTracker::completeRenew(const ControlOwnerOperation &operation, const bool renewed,
                                               const std::uint64_t owner_epoch,
                                               const std::uint64_t expires_at_steady_ns,
                                               const std::uint64_t steady_now_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (operation.kind != ControlOwnerOperationKind::kRenew || !matchesPendingLocked(operation)) {
        return Status::error(ErrorCode::kFailedPrecondition, "stale owner renew callback");
    }
    pending_.reset();
    if (!lease_.has_value() || !sameIdentity(lease_->identity, operation.identity) ||
        lease_->owner_epoch != operation.owner_epoch || !renewed || owner_epoch != operation.owner_epoch ||
        expires_at_steady_ns <= steady_now_ns) {
        lease_.reset();
        return Status::error(ErrorCode::kDeadlineExceeded, "owner renew failed or returned an invalid lease");
    }
    lease_->expires_at_steady_ns = expires_at_steady_ns;
    return Status::success();
}

Result<ControlOwnerOperation> ControlOwnerLeaseTracker::beginRelease(const std::uint64_t steady_now_ns,
                                                                     const std::uint64_t timeout_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (timeout_ns == 0U || pending_.has_value() || !lease_.has_value()) {
        return Result<ControlOwnerOperation>::failure(
            Status::error(ErrorCode::kFailedPrecondition, "owner lease cannot be released"));
    }
    const ControlOwnerLease lease = *lease_;
    lease_.reset();
    return makeOperationLocked(ControlOwnerOperationKind::kRelease, lease.identity, lease.owner_epoch, steady_now_ns,
                               timeout_ns);
}

Status ControlOwnerLeaseTracker::completeRelease(const ControlOwnerOperation &operation, const bool released) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (operation.kind != ControlOwnerOperationKind::kRelease || !matchesPendingLocked(operation)) {
        return Status::error(ErrorCode::kFailedPrecondition, "stale owner release callback");
    }
    pending_.reset();
    if (!released) {
        return Status::error(ErrorCode::kUnavailable, "owner release was not confirmed");
    }
    return Status::success();
}

std::optional<ControlOwnerOperation> ControlOwnerLeaseTracker::cancelPending() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_.has_value()) {
        return std::nullopt;
    }
    const ControlOwnerOperation canceled = *pending_;
    pending_.reset();
    advanceGenerationLocked();
    return canceled;
}

std::optional<ControlOwnerOperation> ControlOwnerLeaseTracker::expirePending(const std::uint64_t steady_now_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_.has_value() || pending_->deadline_steady_ns > steady_now_ns) {
        return std::nullopt;
    }
    const ControlOwnerOperation expired = *pending_;
    pending_.reset();
    if (expired.kind == ControlOwnerOperationKind::kRenew) {
        lease_.reset();
    }
    advanceGenerationLocked();
    return expired;
}

std::optional<ControlOwnerLease> ControlOwnerLeaseTracker::activeLease(const std::uint64_t steady_now_ns) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!lease_.has_value() || lease_->expires_at_steady_ns <= steady_now_ns) {
        return std::nullopt;
    }
    return lease_;
}

std::optional<ControlOwnerLease> ControlOwnerLeaseTracker::leaseSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return lease_;
}

std::optional<ControlOwnerOperation> ControlOwnerLeaseTracker::pendingOperation() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return pending_;
}

void ControlOwnerLeaseTracker::invalidate() {
    std::lock_guard<std::mutex> lock(mutex_);
    lease_.reset();
    pending_.reset();
    advanceGenerationLocked();
}

Result<ControlOwnerOperation> ControlOwnerLeaseTracker::makeOperationLocked(const ControlOwnerOperationKind kind,
                                                                            const ControlOwnerIdentity &identity,
                                                                            const std::uint64_t owner_epoch,
                                                                            const std::uint64_t steady_now_ns,
                                                                            const std::uint64_t timeout_ns) {
    if (next_operation_id_ == 0U || next_operation_id_ == std::numeric_limits<std::uint64_t>::max() ||
        steady_now_ns > std::numeric_limits<std::uint64_t>::max() - timeout_ns) {
        return Result<ControlOwnerOperation>::failure(
            Status::error(ErrorCode::kResourceExhausted, "owner operation counter or deadline overflow"));
    }
    pending_ = ControlOwnerOperation{kind,     next_operation_id_++, generation_,
                                     identity, owner_epoch,          steady_now_ns + timeout_ns};
    return Result<ControlOwnerOperation>::success(*pending_);
}

bool ControlOwnerLeaseTracker::matchesPendingLocked(const ControlOwnerOperation &operation) const {
    return pending_.has_value() && pending_->kind == operation.kind &&
           pending_->operation_id == operation.operation_id && pending_->generation == operation.generation &&
           sameIdentity(pending_->identity, operation.identity) && pending_->owner_epoch == operation.owner_epoch;
}

void ControlOwnerLeaseTracker::advanceGenerationLocked() {
    ++generation_;
    if (generation_ == 0U) {
        generation_ = 1U;
    }
}

}  // namespace astrabot::teleop
