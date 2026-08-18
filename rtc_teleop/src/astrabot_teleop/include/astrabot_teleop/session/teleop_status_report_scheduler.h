// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include "astrabot_teleop/common/result.hpp"
#include "astrabot_teleop/session/teleop_state_machine.h"

namespace astrabot::teleop {

/** @brief 用于判断平台状态报告是否发生逻辑变化的 Teleop 状态快照。 */
struct TeleopStatusObservation {
    TeleopState state{TeleopState::kIdle};
    std::string session_id;
    std::string run_id;
    std::string resource_id;
    std::string peer_id;
    std::string channel_label;
    std::uint64_t last_sequence{0U};
    std::uint64_t sequence_gap_count{0U};
    std::string reason_code;
};

/** @brief 与 ReportTeleopStatus.srv Request 一一对应的稳定重试快照。 */
struct TeleopStatusReport {
    std::string request_id;
    std::string session_id;
    std::string run_id;
    std::string resource_id;
    std::uint8_t state{0U};
    std::uint64_t last_sequence{0U};
    std::uint64_t sequence_gap_count{0U};
    std::int64_t device_ts{0};
    std::string reason_code;
};

/** @brief 标识一次有界异步状态上报，用于拒绝 timeout/cancel 后的晚到 callback。 */
struct TeleopStatusReportOperation {
    std::uint64_t operation_id{0U};
    std::uint64_t generation{0U};
    std::uint64_t deadline_steady_ns{0U};
    TeleopStatusReport report;
};

/** @brief 状态报告调度器的有界诊断计数。 */
struct TeleopStatusReportDiagnostics {
    std::uint64_t accepted_count{0U};
    std::uint64_t rejected_count{0U};
    std::uint64_t timeout_count{0U};
    std::uint64_t deferred_count{0U};
    std::uint64_t overwrite_count{0U};
    std::uint64_t preempt_count{0U};
};

/**
 * @brief 维护单 pending 与单 latest-wins 后备报告，不执行 ROS IO。
 *
 * 逻辑去重只比较投影后的连接状态和完整 binding；内部 Armed/Controlling 切换、reason、last_sequence/sequence_gap
 * 变化都不会单独生成平台报告。
 * 一旦生成报告，其 request_id、device_ts 和计数快照在 timeout/reject 重试中保持不变。
 */
class TeleopStatusReportScheduler {
  public:
    /** @brief 初始化本进程唯一 request_id 前缀和失败重试间隔。 */
    Status initialize(std::string request_id_prefix, std::uint64_t retry_delay_ns);

    /** @brief 观察状态；逻辑状态未变化时返回 false。 */
    Result<bool> observe(const TeleopStatusObservation &observation, std::int64_t device_ts);

    /** @brief 返回当前是否有到期可发送的 latest 报告。 */
    bool sendDue(std::uint64_t steady_now_ns) const;

    /** @brief 开始一次发送；已有 pending、尚未到重试时间或无报告时返回空。 */
    Result<std::optional<TeleopStatusReportOperation>> beginSend(std::uint64_t steady_now_ns, std::uint64_t timeout_ns);

    /** @brief 完成响应；拒绝旧 generation 或非当前 operation。 */
    Status complete(const TeleopStatusReportOperation &operation, bool accepted, std::uint64_t steady_now_ns);

    /** @brief 超时撤销 pending，并按 latest-wins 规则安排稳定 request_id 重试。 */
    std::optional<TeleopStatusReportOperation> expire(std::uint64_t steady_now_ns);

    /** @brief 服务不可用时推迟下一次尝试，不创建 pending。 */
    void defer(std::uint64_t steady_now_ns);

    /** @brief 若 latest 是 terminal，撤销当前旧 pending，使 terminal 可立即尽力发送。 */
    std::optional<TeleopStatusReportOperation> preemptPendingForTerminal();

    /** @brief 返回当前 pending 快照。 */
    std::optional<TeleopStatusReportOperation> pendingOperation() const;

    /** @brief 返回有界诊断计数。 */
    TeleopStatusReportDiagnostics diagnostics() const;

    /** @brief 清空 pending/latest 并使所有旧 callback 失效。 */
    void invalidate();

    /**
     * @brief 将内部状态投影为平台 `connected/disconnected`；尚未连接的状态返回不可上报。
     */
    static Result<std::uint8_t> reportState(TeleopState state);

    /** @brief 判断状态是否允许在 run 结束后补报。 */
    static bool terminalState(TeleopState state);

  private:
    static bool logicalEquals(const TeleopStatusObservation &first, const TeleopStatusObservation &second);
    static bool validObservation(const TeleopStatusObservation &observation);
    static bool terminalReportState(std::uint8_t state);
    static bool operationMatches(const TeleopStatusReportOperation &first, const TeleopStatusReportOperation &second);
    static std::uint64_t boundedDeadline(std::uint64_t steady_now_ns, std::uint64_t delay_ns);
    void retryOrDropLocked(const TeleopStatusReport &failed_report, std::uint64_t steady_now_ns);
    void advanceGenerationLocked();

    mutable std::mutex mutex_;
    bool initialized_{false};
    std::string request_id_prefix_;
    std::uint64_t retry_delay_ns_{0U};
    std::uint64_t next_report_id_{1U};
    std::uint64_t next_operation_id_{1U};
    std::uint64_t generation_{1U};
    std::uint64_t next_send_steady_ns_{0U};
    std::optional<TeleopStatusObservation> last_observation_;
    std::optional<TeleopStatusReport> latest_report_;
    std::optional<TeleopStatusReportOperation> pending_;
    TeleopStatusReportDiagnostics diagnostics_;
};

}  // namespace astrabot::teleop
