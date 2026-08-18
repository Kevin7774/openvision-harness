// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "astrabot_data_interfaces/srv/report_teleop_status.hpp"
#include "astrabot_teleop/common/status.h"
#include "astrabot_teleop/session/teleop_status_report_scheduler.h"

namespace astrabot::teleop {

/** @brief Data Collection Teleop 状态服务的有界异步 client 配置。 */
struct TeleopStatusReportClientConfig {
    std::string service_name;
    std::string request_id_prefix;
    std::uint64_t service_timeout_ns{0U};
    std::uint64_t retry_period_ns{0U};
    std::uint64_t poll_period_ns{0U};

    /** @brief 校验 endpoint、request_id 和时间边界。 */
    Status validate() const;
};

/**
 * @brief 非阻塞上报 Teleop 逻辑状态，并隔离 Data Collection 故障与控制链。
 *
 * 对象按 one-shot 使用；standalone 模式必须在 executor callback 静默后 stop。内部最多一个 ROS pending request，
 * 新逻辑状态由 scheduler 以 latest-wins 容量 1 保存。服务不可用、拒绝或超时只记录诊断并重试，不改变 Teleop 控制状态。
 */
class TeleopStatusReportClient final : public std::enable_shared_from_this<TeleopStatusReportClient> {
  public:
    TeleopStatusReportClient() = default;
    ~TeleopStatusReportClient();

    /** @brief 创建 ROS client/timer 并启动异步上报。 */
    Status start(const rclcpp::Node::SharedPtr &node, const TeleopStatusReportClientConfig &config);

    /** @brief 幂等取消 pending、timer 和 client；stop 后不得再次 start。 */
    void stop();

    /** @brief 观察一个完整 binding 的状态事实；不会等待服务或响应。 */
    Status observe(const TeleopStatusObservation &observation, std::int64_t device_ts);

    /** @brief 返回调度器的有界诊断计数。 */
    TeleopStatusReportDiagnostics diagnostics() const;

  private:
    struct PendingRequest {
        std::int64_t ros_request_id{0};
        TeleopStatusReportOperation operation;
    };

    void process();
    void pump(std::uint64_t steady_now_ns);
    void handleResponse(const TeleopStatusReportOperation &operation,
                        rclcpp::Client<astrabot_data_interfaces::srv::ReportTeleopStatus>::SharedFuture future);
    void registerPending(std::int64_t ros_request_id, const TeleopStatusReportOperation &operation);
    void clearPending(const TeleopStatusReportOperation &operation);
    void removePendingForOperation(const TeleopStatusReportOperation &operation);
    void cancelPending();
    void removeClientPending(const PendingRequest &pending);
    void preemptForTerminal();
    static bool sameOperation(const TeleopStatusReportOperation &first, const TeleopStatusReportOperation &second);
    static std::uint64_t steadyNowNs();

    std::atomic<bool> running_{false};
    std::atomic<bool> stopped_permanently_{false};
    TeleopStatusReportClientConfig config_;
    TeleopStatusReportScheduler scheduler_;
    rclcpp::Logger logger_{rclcpp::get_logger("astrabot_teleop.status_report")};
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Client<astrabot_data_interfaces::srv::ReportTeleopStatus>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    mutable std::mutex pending_mutex_;
    std::optional<PendingRequest> pending_request_;
};

}  // namespace astrabot::teleop
