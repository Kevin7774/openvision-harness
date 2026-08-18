// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include <cstdint>

#include <gtest/gtest.h>

#include "astrabot_teleop/session/teleop_status_report_scheduler.h"

namespace astrabot::teleop {
namespace {

TeleopStatusObservation makeObservation(const TeleopState state = TeleopState::kConnected,
                                        const char *reason_code = "connected") {
    return TeleopStatusObservation{state, "session-1", "run-1",    "thor", "peer-1", "astrabot.teleop",
                                   10U,   2U,          reason_code};
}

void initializeScheduler(TeleopStatusReportScheduler *scheduler) {
    ASSERT_NE(scheduler, nullptr);
    EXPECT_TRUE(scheduler->initialize("teleop_test", 100U).ok());
}

TeleopStatusReportOperation beginRequired(TeleopStatusReportScheduler *scheduler, const std::uint64_t now_ns,
                                          const std::uint64_t timeout_ns = 40U) {
    auto operation = scheduler->beginSend(now_ns, timeout_ns);
    EXPECT_TRUE(operation.ok());
    if (!operation.ok() || !operation.value().has_value()) {
        return TeleopStatusReportOperation{};
    }
    return *operation.value();
}

TEST(TeleopStatusReportSchedulerTest, ProjectsInternalStatesToPlatformConnectivity) {
    EXPECT_FALSE(TeleopStatusReportScheduler::reportState(TeleopState::kIdle).ok());
    EXPECT_FALSE(TeleopStatusReportScheduler::reportState(TeleopState::kAuthorized).ok());
    EXPECT_EQ(TeleopStatusReportScheduler::reportState(TeleopState::kConnected).value(), 0U);
    EXPECT_EQ(TeleopStatusReportScheduler::reportState(TeleopState::kArmed).value(), 0U);
    EXPECT_EQ(TeleopStatusReportScheduler::reportState(TeleopState::kControlling).value(), 0U);
    EXPECT_EQ(TeleopStatusReportScheduler::reportState(TeleopState::kStopping).value(), 1U);
    EXPECT_EQ(TeleopStatusReportScheduler::reportState(TeleopState::kClosed).value(), 1U);
    EXPECT_EQ(TeleopStatusReportScheduler::reportState(TeleopState::kFault).value(), 1U);
    EXPECT_TRUE(TeleopStatusReportScheduler::terminalState(TeleopState::kStopping));
    EXPECT_TRUE(TeleopStatusReportScheduler::terminalState(TeleopState::kClosed));
    EXPECT_TRUE(TeleopStatusReportScheduler::terminalState(TeleopState::kFault));
    EXPECT_FALSE(TeleopStatusReportScheduler::terminalState(TeleopState::kControlling));
}

TEST(TeleopStatusReportSchedulerTest, IgnoresSequenceOnlyChanges) {
    TeleopStatusReportScheduler scheduler;
    initializeScheduler(&scheduler);
    auto observation = makeObservation();
    ASSERT_TRUE(scheduler.observe(observation, 1000).value());
    observation.last_sequence = 11U;
    observation.sequence_gap_count = 3U;
    const auto unchanged = scheduler.observe(observation, 1001);
    ASSERT_TRUE(unchanged.ok());
    EXPECT_FALSE(unchanged.value());

    const auto operation = beginRequired(&scheduler, 100U);
    EXPECT_EQ(operation.report.last_sequence, 10U);
    EXPECT_EQ(operation.report.sequence_gap_count, 2U);
    EXPECT_EQ(operation.report.device_ts, 1000);
}

TEST(TeleopStatusReportSchedulerTest, DoesNotPublishBeforePeerConnectivityExists) {
    TeleopStatusReportScheduler scheduler;
    initializeScheduler(&scheduler);
    EXPECT_FALSE(scheduler.observe(makeObservation(TeleopState::kIdle, "idle"), 1000).value());
    EXPECT_FALSE(scheduler.observe(makeObservation(TeleopState::kAuthorized, "authorized"), 1001).value());
    EXPECT_FALSE(scheduler.sendDue(100U));
    EXPECT_TRUE(scheduler.observe(makeObservation(TeleopState::kConnected, "connected"), 1002).value());
    EXPECT_TRUE(scheduler.sendDue(100U));
}

TEST(TeleopStatusReportSchedulerTest, InternalStateAndReasonChangesDoNotRepublishConnectivity) {
    TeleopStatusReportScheduler scheduler;
    initializeScheduler(&scheduler);
    auto observation = makeObservation();
    ASSERT_TRUE(scheduler.observe(observation, 1000).value());
    const auto first = beginRequired(&scheduler, 100U);

    observation.state = TeleopState::kArmed;
    observation.reason_code = "owner_acquired";
    observation.last_sequence = 20U;
    ASSERT_FALSE(scheduler.observe(observation, 1001).value());
    observation.reason_code = "owner_renewed";
    observation.last_sequence = 21U;
    ASSERT_FALSE(scheduler.observe(observation, 1002).value());
    observation.peer_id = "peer-2";
    observation.last_sequence = 30U;
    ASSERT_TRUE(scheduler.observe(observation, 1003).value());
    EXPECT_EQ(scheduler.diagnostics().overwrite_count, 0U);

    ASSERT_TRUE(scheduler.complete(first, true, 110U).ok());
    const auto latest = beginRequired(&scheduler, 111U);
    EXPECT_EQ(latest.report.reason_code, "owner_renewed");
    EXPECT_EQ(latest.report.last_sequence, 30U);
    EXPECT_NE(latest.report.request_id, first.report.request_id);
}

TEST(TeleopStatusReportSchedulerTest, TimeoutRetriesExactRequestWithStableId) {
    TeleopStatusReportScheduler scheduler;
    initializeScheduler(&scheduler);
    ASSERT_TRUE(scheduler.observe(makeObservation(), 1000).value());
    const auto first = beginRequired(&scheduler, 100U);
    EXPECT_FALSE(scheduler.expire(139U).has_value());
    ASSERT_TRUE(scheduler.expire(140U).has_value());
    EXPECT_FALSE(scheduler.sendDue(239U));
    EXPECT_TRUE(scheduler.sendDue(240U));

    const auto retry = beginRequired(&scheduler, 240U);
    EXPECT_EQ(retry.report.request_id, first.report.request_id);
    EXPECT_EQ(retry.report.device_ts, first.report.device_ts);
    EXPECT_EQ(retry.report.last_sequence, first.report.last_sequence);
    EXPECT_EQ(scheduler.diagnostics().timeout_count, 1U);
    EXPECT_FALSE(scheduler.complete(first, true, 241U).ok());
}

TEST(TeleopStatusReportSchedulerTest, RejectionRetriesStableRequestUnlessNewerReportExists) {
    TeleopStatusReportScheduler scheduler;
    initializeScheduler(&scheduler);
    ASSERT_TRUE(scheduler.observe(makeObservation(), 1000).value());
    const auto first = beginRequired(&scheduler, 100U);
    EXPECT_FALSE(scheduler.complete(first, false, 110U).ok());
    const auto retry = beginRequired(&scheduler, 210U);
    EXPECT_EQ(retry.report.request_id, first.report.request_id);

    auto latest_observation = makeObservation(TeleopState::kConnected, "connected");
    latest_observation.peer_id = "peer-2";
    latest_observation.last_sequence = 20U;
    ASSERT_TRUE(scheduler.observe(latest_observation, 1002).value());
    ASSERT_TRUE(scheduler.expire(250U).has_value());
    const auto latest = beginRequired(&scheduler, 350U);
    EXPECT_NE(latest.report.request_id, retry.report.request_id);
    EXPECT_EQ(latest.report.state, 0U);
    EXPECT_EQ(latest.report.last_sequence, 20U);
}

TEST(TeleopStatusReportSchedulerTest, TerminalPreemptsOlderPendingAndInvalidatesLateCallback) {
    TeleopStatusReportScheduler scheduler;
    initializeScheduler(&scheduler);
    ASSERT_TRUE(scheduler.observe(makeObservation(), 1000).value());
    const auto connected = beginRequired(&scheduler, 100U);

    auto closed = makeObservation(TeleopState::kClosed, "watchdog_timeout");
    closed.last_sequence = 30U;
    ASSERT_TRUE(scheduler.observe(closed, 1001).value());
    const auto preempted = scheduler.preemptPendingForTerminal();
    ASSERT_TRUE(preempted.has_value());
    EXPECT_EQ(preempted->report.request_id, connected.report.request_id);
    EXPECT_FALSE(scheduler.complete(connected, true, 101U).ok());

    const auto terminal = beginRequired(&scheduler, 102U);
    EXPECT_EQ(terminal.report.state, 1U);
    EXPECT_EQ(terminal.report.reason_code, "watchdog_timeout");
    EXPECT_EQ(scheduler.diagnostics().preempt_count, 1U);
}

TEST(TeleopStatusReportSchedulerTest, CoalescesStoppingClosedAndFaultIntoOneDisconnect) {
    TeleopStatusReportScheduler scheduler;
    initializeScheduler(&scheduler);
    ASSERT_TRUE(scheduler.observe(makeObservation(TeleopState::kStopping, "watchdog_timeout"), 1000).value());
    const auto stopping = beginRequired(&scheduler, 100U);

    ASSERT_FALSE(scheduler.observe(makeObservation(TeleopState::kClosed, "watchdog_timeout"), 1001).value());
    EXPECT_FALSE(scheduler.preemptPendingForTerminal().has_value());
    ASSERT_TRUE(scheduler.complete(stopping, true, 110U).ok());
    EXPECT_FALSE(scheduler.sendDue(111U));
    EXPECT_EQ(scheduler.diagnostics().preempt_count, 0U);
}

TEST(TeleopStatusReportSchedulerTest, NewerFactIsImmediateWhenFailedPendingIsDropped) {
    TeleopStatusReportScheduler scheduler;
    initializeScheduler(&scheduler);
    ASSERT_TRUE(scheduler.observe(makeObservation(), 1000).value());
    const auto connected = beginRequired(&scheduler, 100U);

    auto newer_peer = makeObservation(TeleopState::kControlling, "controlling");
    newer_peer.peer_id = "peer-2";
    ASSERT_TRUE(scheduler.observe(newer_peer, 1001).value());
    ASSERT_TRUE(scheduler.expire(140U).has_value());
    EXPECT_TRUE(scheduler.sendDue(140U));

    const auto controlling = beginRequired(&scheduler, 140U);
    EXPECT_NE(controlling.report.request_id, connected.report.request_id);
    EXPECT_EQ(controlling.report.state, 0U);
}

TEST(TeleopStatusReportSchedulerTest, KeepsOnlyOnePendingAndDefersWithoutCreatingOperation) {
    TeleopStatusReportScheduler scheduler;
    initializeScheduler(&scheduler);
    ASSERT_TRUE(scheduler.observe(makeObservation(), 1000).value());
    scheduler.defer(100U);
    EXPECT_FALSE(scheduler.sendDue(199U));
    EXPECT_TRUE(scheduler.sendDue(200U));
    const auto pending = beginRequired(&scheduler, 200U);
    auto second = scheduler.beginSend(201U, 40U);
    ASSERT_TRUE(second.ok());
    EXPECT_FALSE(second.value().has_value());
    ASSERT_TRUE(scheduler.pendingOperation().has_value());
    EXPECT_EQ(scheduler.pendingOperation()->operation_id, pending.operation_id);
    EXPECT_EQ(scheduler.diagnostics().deferred_count, 1U);
}

TEST(TeleopStatusReportSchedulerTest, RejectsIncompleteObservationAndOldGenerationAfterInvalidate) {
    TeleopStatusReportScheduler scheduler;
    initializeScheduler(&scheduler);
    auto invalid = makeObservation();
    invalid.session_id.clear();
    EXPECT_FALSE(scheduler.observe(invalid, 1000).ok());

    ASSERT_TRUE(scheduler.observe(makeObservation(), 1000).value());
    const auto operation = beginRequired(&scheduler, 100U);
    scheduler.invalidate();
    EXPECT_FALSE(scheduler.complete(operation, true, 110U).ok());
    EXPECT_FALSE(scheduler.pendingOperation().has_value());
}

}  // namespace
}  // namespace astrabot::teleop
