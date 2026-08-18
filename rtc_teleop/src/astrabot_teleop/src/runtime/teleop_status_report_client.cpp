// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include "astrabot_teleop/runtime/teleop_status_report_client.h"

#include <chrono>
#include <utility>

namespace astrabot::teleop {

Status TeleopStatusReportClientConfig::validate() const {
    if (service_name.empty() || request_id_prefix.empty() || request_id_prefix.size() > 96U ||
        service_timeout_ns == 0U || retry_period_ns < service_timeout_ns || poll_period_ns == 0U ||
        poll_period_ns > service_timeout_ns) {
        return Status::error(ErrorCode::kInvalidArgument, "Teleop status report client config is invalid");
    }
    return Status::success();
}

TeleopStatusReportClient::~TeleopStatusReportClient() {
    stop();
}

Status TeleopStatusReportClient::start(const rclcpp::Node::SharedPtr &node,
                                       const TeleopStatusReportClientConfig &config) {
    if (stopped_permanently_.load()) {
        return Status::error(ErrorCode::kFailedPrecondition, "Teleop status report client cannot restart after stop");
    }
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return Status::success();
    }
    if (!node) {
        running_.store(false);
        return Status::error(ErrorCode::kInvalidArgument, "Teleop status report client requires a ROS node");
    }
    const Status config_status = config.validate();
    if (!config_status.ok()) {
        running_.store(false);
        return config_status;
    }
    const std::weak_ptr<TeleopStatusReportClient> weak_self = weak_from_this();
    if (weak_self.expired()) {
        running_.store(false);
        return Status::error(ErrorCode::kFailedPrecondition, "Teleop status report client must be owned by shared_ptr");
    }
    const Status scheduler_status = scheduler_.initialize(config.request_id_prefix, config.retry_period_ns);
    if (!scheduler_status.ok()) {
        running_.store(false);
        return scheduler_status;
    }

    config_ = config;
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    client_ = node->create_client<astrabot_data_interfaces::srv::ReportTeleopStatus>(config_.service_name);
    timer_ = node->create_wall_timer(std::chrono::nanoseconds(config_.poll_period_ns), [weak_self]() {
        if (const auto self = weak_self.lock()) {
            self->process();
        }
    });
    return Status::success();
}

void TeleopStatusReportClient::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    stopped_permanently_.store(true);
    timer_.reset();
    cancelPending();
    scheduler_.invalidate();
    client_.reset();
    clock_.reset();
}

Status TeleopStatusReportClient::observe(const TeleopStatusObservation &observation, const std::int64_t device_ts) {
    if (!running_.load()) {
        return Status::error(ErrorCode::kFailedPrecondition, "Teleop status report client is not running");
    }
    auto changed = scheduler_.observe(observation, device_ts);
    if (!changed.ok()) {
        return changed.status();
    }
    if (!changed.value()) {
        return Status::success();
    }
    if (TeleopStatusReportScheduler::terminalState(observation.state)) {
        preemptForTerminal();
    }
    pump(steadyNowNs());
    return Status::success();
}

TeleopStatusReportDiagnostics TeleopStatusReportClient::diagnostics() const {
    return scheduler_.diagnostics();
}

void TeleopStatusReportClient::process() {
    if (!running_.load()) {
        return;
    }
    const std::uint64_t now_ns = steadyNowNs();
    const auto expired = scheduler_.expire(now_ns);
    if (expired.has_value()) {
        removePendingForOperation(*expired);
        const auto diagnostics_snapshot = scheduler_.diagnostics();
        RCLCPP_WARN(logger_,
                    "Data Collection Teleop status request timed out; control continues timeout_count=%lu "
                    "overwrite_count=%lu",
                    static_cast<unsigned long>(diagnostics_snapshot.timeout_count),
                    static_cast<unsigned long>(diagnostics_snapshot.overwrite_count));
    }
    pump(now_ns);
}

void TeleopStatusReportClient::pump(const std::uint64_t steady_now_ns) {
    if (!running_.load() || !scheduler_.sendDue(steady_now_ns)) {
        return;
    }
    if (!client_ || !client_->service_is_ready()) {
        scheduler_.defer(steady_now_ns);
        const auto diagnostics_snapshot = scheduler_.diagnostics();
        if (clock_) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000,
                                 "Data Collection Teleop status service unavailable; control continues "
                                 "deferred_count=%lu overwrite_count=%lu",
                                 static_cast<unsigned long>(diagnostics_snapshot.deferred_count),
                                 static_cast<unsigned long>(diagnostics_snapshot.overwrite_count));
        }
        return;
    }

    auto operation_result = scheduler_.beginSend(steady_now_ns, config_.service_timeout_ns);
    if (!operation_result.ok()) {
        RCLCPP_WARN(logger_, "Teleop status report scheduling failed; control continues reason=%s",
                    operation_result.status().message().c_str());
        scheduler_.defer(steady_now_ns);
        return;
    }
    if (!operation_result.value().has_value()) {
        return;
    }
    const TeleopStatusReportOperation operation = *operation_result.value();
    auto request = std::make_shared<astrabot_data_interfaces::srv::ReportTeleopStatus::Request>();
    request->request_id = operation.report.request_id;
    request->session_id = operation.report.session_id;
    request->run_id = operation.report.run_id;
    request->resource_id = operation.report.resource_id;
    request->state = operation.report.state;
    request->last_sequence = operation.report.last_sequence;
    request->sequence_gap_count = operation.report.sequence_gap_count;
    request->device_ts = operation.report.device_ts;
    request->reason_code = operation.report.reason_code;

    const std::weak_ptr<TeleopStatusReportClient> weak_self = weak_from_this();
    auto future_and_id = client_->async_send_request(
        request,
        [weak_self, operation](rclcpp::Client<astrabot_data_interfaces::srv::ReportTeleopStatus>::SharedFuture future) {
            if (const auto self = weak_self.lock()) {
                self->handleResponse(operation, future);
            }
        });
    registerPending(future_and_id.request_id, operation);
}

void TeleopStatusReportClient::handleResponse(
    const TeleopStatusReportOperation &operation,
    rclcpp::Client<astrabot_data_interfaces::srv::ReportTeleopStatus>::SharedFuture future) {
    clearPending(operation);
    if (!running_.load()) {
        return;
    }
    const auto response = future.get();
    const bool accepted = response && response->accepted;
    const Status completion_status = scheduler_.complete(operation, accepted, steadyNowNs());
    if (!completion_status.ok() && completion_status.code() == ErrorCode::kFailedPrecondition) {
        return;
    }
    if (!accepted) {
        const std::string error_code = response ? response->error_code : "response_missing";
        const auto diagnostics_snapshot = scheduler_.diagnostics();
        RCLCPP_WARN(logger_,
                    "Data Collection rejected Teleop status; control continues error_code=%s rejected_count=%lu",
                    error_code.c_str(), static_cast<unsigned long>(diagnostics_snapshot.rejected_count));
    }
    pump(steadyNowNs());
}

void TeleopStatusReportClient::registerPending(const std::int64_t ros_request_id,
                                               const TeleopStatusReportOperation &operation) {
    bool already_completed = false;
    bool conflict = false;
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        const auto scheduler_pending = scheduler_.pendingOperation();
        already_completed = !scheduler_pending.has_value() || !sameOperation(*scheduler_pending, operation);
        if (!already_completed && pending_request_.has_value()) {
            conflict = true;
        } else if (!already_completed) {
            pending_request_ = PendingRequest{ros_request_id, operation};
        }
    }
    if (already_completed || conflict) {
        removeClientPending(PendingRequest{ros_request_id, operation});
    }
    if (conflict) {
        scheduler_.invalidate();
        RCLCPP_ERROR(logger_, "Teleop status report pending limit violated; reporting reset, control continues");
    }
}

void TeleopStatusReportClient::clearPending(const TeleopStatusReportOperation &operation) {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (pending_request_.has_value() && sameOperation(pending_request_->operation, operation)) {
        pending_request_.reset();
    }
}

void TeleopStatusReportClient::removePendingForOperation(const TeleopStatusReportOperation &operation) {
    std::optional<PendingRequest> pending;
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        if (pending_request_.has_value() && sameOperation(pending_request_->operation, operation)) {
            pending = pending_request_;
            pending_request_.reset();
        }
    }
    if (pending.has_value()) {
        removeClientPending(*pending);
    }
}

void TeleopStatusReportClient::cancelPending() {
    std::optional<PendingRequest> pending;
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending = pending_request_;
        pending_request_.reset();
    }
    if (pending.has_value()) {
        removeClientPending(*pending);
    }
}

void TeleopStatusReportClient::removeClientPending(const PendingRequest &pending) {
    if (client_) {
        client_->remove_pending_request(pending.ros_request_id);
    }
}

void TeleopStatusReportClient::preemptForTerminal() {
    const auto preempted = scheduler_.preemptPendingForTerminal();
    if (preempted.has_value()) {
        removePendingForOperation(*preempted);
    }
}

bool TeleopStatusReportClient::sameOperation(const TeleopStatusReportOperation &first,
                                             const TeleopStatusReportOperation &second) {
    return first.operation_id == second.operation_id && first.generation == second.generation &&
           first.report.request_id == second.report.request_id;
}

std::uint64_t TeleopStatusReportClient::steadyNowNs() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

}  // namespace astrabot::teleop
