// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include "astrabot_teleop/session/teleop_status_report_scheduler.h"

#include <algorithm>
#include <limits>
#include <utility>

namespace astrabot::teleop {
namespace {

constexpr std::size_t kMaxIdentityLength = 128U;
constexpr std::size_t kMaxRequestIdPrefixLength = 96U;
constexpr std::uint8_t kReportConnected = 0U;
constexpr std::uint8_t kReportDisconnected = 1U;

bool boundedText(const std::string &value, const bool allow_empty) {
    if ((!allow_empty && value.empty()) || value.size() > kMaxIdentityLength) {
        return false;
    }
    return std::all_of(value.begin(), value.end(),
                       [](const unsigned char character) { return character >= 0x20U && character != 0x7FU; });
}

}  // namespace

Status TeleopStatusReportScheduler::initialize(std::string request_id_prefix, const std::uint64_t retry_delay_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialized_) {
        return Status::error(ErrorCode::kConflict, "Teleop status report scheduler is already initialized");
    }
    if (request_id_prefix.empty() || request_id_prefix.size() > kMaxRequestIdPrefixLength || retry_delay_ns == 0U ||
        !boundedText(request_id_prefix, false)) {
        return Status::error(ErrorCode::kInvalidArgument, "Teleop status report scheduler config is invalid");
    }
    request_id_prefix_ = std::move(request_id_prefix);
    retry_delay_ns_ = retry_delay_ns;
    initialized_ = true;
    return Status::success();
}

Result<bool> TeleopStatusReportScheduler::observe(const TeleopStatusObservation &observation,
                                                  const std::int64_t device_ts) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
        return Result<bool>::failure(
            Status::error(ErrorCode::kFailedPrecondition, "Teleop status report scheduler is not initialized"));
    }
    if (device_ts <= 0 || !validObservation(observation)) {
        return Result<bool>::failure(
            Status::error(ErrorCode::kInvalidArgument, "Teleop status observation is invalid"));
    }
    auto state = reportState(observation.state);
    if (!state.ok()) {
        if (observation.state == TeleopState::kIdle || observation.state == TeleopState::kAuthorized) {
            return Result<bool>::success(false);
        }
        return Result<bool>::failure(state.status());
    }
    if (last_observation_.has_value() && logicalEquals(*last_observation_, observation)) {
        return Result<bool>::success(false);
    }
    if (next_report_id_ == 0U || next_report_id_ == std::numeric_limits<std::uint64_t>::max()) {
        return Result<bool>::failure(
            Status::error(ErrorCode::kResourceExhausted, "Teleop status report id space is exhausted"));
    }
    const std::string request_id = request_id_prefix_ + "_" + std::to_string(next_report_id_++);
    if (request_id.size() > kMaxIdentityLength) {
        return Result<bool>::failure(
            Status::error(ErrorCode::kResourceExhausted, "Teleop status request id exceeds contract bound"));
    }

    TeleopStatusReport report{request_id,
                              observation.session_id,
                              observation.run_id,
                              observation.resource_id,
                              state.value(),
                              observation.last_sequence,
                              observation.sequence_gap_count,
                              device_ts,
                              observation.reason_code};
    if (latest_report_.has_value()) {
        ++diagnostics_.overwrite_count;
    }
    latest_report_ = std::move(report);
    last_observation_ = observation;
    next_send_steady_ns_ = 0U;
    return Result<bool>::success(true);
}

bool TeleopStatusReportScheduler::sendDue(const std::uint64_t steady_now_ns) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return initialized_ && !pending_.has_value() && latest_report_.has_value() && steady_now_ns >= next_send_steady_ns_;
}

Result<std::optional<TeleopStatusReportOperation>>
TeleopStatusReportScheduler::beginSend(const std::uint64_t steady_now_ns, const std::uint64_t timeout_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || timeout_ns == 0U) {
        return Result<std::optional<TeleopStatusReportOperation>>::failure(
            Status::error(ErrorCode::kInvalidArgument, "Teleop status send timeout or lifecycle is invalid"));
    }
    if (pending_.has_value() || !latest_report_.has_value() || steady_now_ns < next_send_steady_ns_) {
        return Result<std::optional<TeleopStatusReportOperation>>::success(std::nullopt);
    }
    if (next_operation_id_ == 0U || next_operation_id_ == std::numeric_limits<std::uint64_t>::max() ||
        steady_now_ns > std::numeric_limits<std::uint64_t>::max() - timeout_ns) {
        return Result<std::optional<TeleopStatusReportOperation>>::failure(
            Status::error(ErrorCode::kResourceExhausted, "Teleop status operation id or deadline overflow"));
    }
    pending_ =
        TeleopStatusReportOperation{next_operation_id_++, generation_, steady_now_ns + timeout_ns, *latest_report_};
    latest_report_.reset();
    return Result<std::optional<TeleopStatusReportOperation>>::success(pending_);
}

Status TeleopStatusReportScheduler::complete(const TeleopStatusReportOperation &operation, const bool accepted,
                                             const std::uint64_t steady_now_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_.has_value() || !operationMatches(*pending_, operation)) {
        return Status::error(ErrorCode::kFailedPrecondition, "stale Teleop status report callback");
    }
    const TeleopStatusReport completed_report = pending_->report;
    pending_.reset();
    if (accepted) {
        ++diagnostics_.accepted_count;
        next_send_steady_ns_ = steady_now_ns;
        return Status::success();
    }
    ++diagnostics_.rejected_count;
    retryOrDropLocked(completed_report, steady_now_ns);
    return Status::error(ErrorCode::kUnavailable, "Teleop status report was rejected");
}

std::optional<TeleopStatusReportOperation> TeleopStatusReportScheduler::expire(const std::uint64_t steady_now_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_.has_value() || pending_->deadline_steady_ns > steady_now_ns) {
        return std::nullopt;
    }
    const TeleopStatusReportOperation expired = *pending_;
    pending_.reset();
    ++diagnostics_.timeout_count;
    advanceGenerationLocked();
    retryOrDropLocked(expired.report, steady_now_ns);
    return expired;
}

void TeleopStatusReportScheduler::defer(const std::uint64_t steady_now_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || pending_.has_value() || !latest_report_.has_value()) {
        return;
    }
    next_send_steady_ns_ = boundedDeadline(steady_now_ns, retry_delay_ns_);
    ++diagnostics_.deferred_count;
}

std::optional<TeleopStatusReportOperation> TeleopStatusReportScheduler::preemptPendingForTerminal() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_.has_value() || !latest_report_.has_value() || !terminalReportState(latest_report_->state) ||
        terminalReportState(pending_->report.state)) {
        return std::nullopt;
    }
    const TeleopStatusReportOperation preempted = *pending_;
    pending_.reset();
    next_send_steady_ns_ = 0U;
    ++diagnostics_.preempt_count;
    advanceGenerationLocked();
    return preempted;
}

std::optional<TeleopStatusReportOperation> TeleopStatusReportScheduler::pendingOperation() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return pending_;
}

TeleopStatusReportDiagnostics TeleopStatusReportScheduler::diagnostics() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return diagnostics_;
}

void TeleopStatusReportScheduler::invalidate() {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.reset();
    latest_report_.reset();
    last_observation_.reset();
    next_send_steady_ns_ = 0U;
    advanceGenerationLocked();
}

Result<std::uint8_t> TeleopStatusReportScheduler::reportState(const TeleopState state) {
    switch (state) {
        case TeleopState::kIdle:
        case TeleopState::kAuthorized:
            return Result<std::uint8_t>::failure(
                Status::error(ErrorCode::kFailedPrecondition, "Teleop peer is not connected yet"));
        case TeleopState::kConnected:
        case TeleopState::kArmed:
        case TeleopState::kControlling:
            return Result<std::uint8_t>::success(kReportConnected);
        case TeleopState::kStopping:
        case TeleopState::kClosed:
        case TeleopState::kFault:
            return Result<std::uint8_t>::success(kReportDisconnected);
    }
    return Result<std::uint8_t>::failure(Status::error(ErrorCode::kInvalidArgument, "unknown Teleop state"));
}

bool TeleopStatusReportScheduler::terminalState(const TeleopState state) {
    return state == TeleopState::kStopping || state == TeleopState::kClosed || state == TeleopState::kFault;
}

bool TeleopStatusReportScheduler::logicalEquals(const TeleopStatusObservation &first,
                                                const TeleopStatusObservation &second) {
    const auto first_state = reportState(first.state);
    const auto second_state = reportState(second.state);
    return first_state.ok() && second_state.ok() && first_state.value() == second_state.value() &&
           first.session_id == second.session_id && first.run_id == second.run_id &&
           first.resource_id == second.resource_id && first.peer_id == second.peer_id &&
           first.channel_label == second.channel_label;
}

bool TeleopStatusReportScheduler::validObservation(const TeleopStatusObservation &observation) {
    return boundedText(observation.session_id, false) && boundedText(observation.run_id, false) &&
           boundedText(observation.resource_id, false) && boundedText(observation.peer_id, false) &&
           boundedText(observation.channel_label, false) && boundedText(observation.reason_code, true);
}

bool TeleopStatusReportScheduler::terminalReportState(const std::uint8_t state) {
    return state == kReportDisconnected;
}

bool TeleopStatusReportScheduler::operationMatches(const TeleopStatusReportOperation &first,
                                                   const TeleopStatusReportOperation &second) {
    return first.operation_id == second.operation_id && first.generation == second.generation &&
           first.report.request_id == second.report.request_id;
}

std::uint64_t TeleopStatusReportScheduler::boundedDeadline(const std::uint64_t steady_now_ns,
                                                           const std::uint64_t delay_ns) {
    if (steady_now_ns > std::numeric_limits<std::uint64_t>::max() - delay_ns) {
        return std::numeric_limits<std::uint64_t>::max();
    }
    return steady_now_ns + delay_ns;
}

void TeleopStatusReportScheduler::retryOrDropLocked(const TeleopStatusReport &failed_report,
                                                    const std::uint64_t steady_now_ns) {
    if (!latest_report_.has_value()) {
        latest_report_ = failed_report;
        next_send_steady_ns_ = boundedDeadline(steady_now_ns, retry_delay_ns_);
    } else {
        ++diagnostics_.overwrite_count;
        // 新事实已经取代失败请求，无需把最新状态也拖入旧请求的重试退避。
        next_send_steady_ns_ = steady_now_ns;
    }
}

void TeleopStatusReportScheduler::advanceGenerationLocked() {
    ++generation_;
    if (generation_ == 0U) {
        generation_ = 1U;
    }
}

}  // namespace astrabot::teleop
