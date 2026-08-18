// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "astrabot_data_interfaces/msg/data_collection_run_context.hpp"
#include "astrabot_data_interfaces/srv/report_teleop_status.hpp"
#include "astrabot_teleop/session/teleop_status_report_scheduler.h"

namespace astrabot::teleop {
namespace {

std::vector<std::string> readSchemaLines(const std::string &relative_path) {
    std::ifstream input(std::string(ASTRABOT_TELEOP_SOURCE_DIR) + "/" + relative_path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line)) {
        if (!line.empty() && line.front() != '#') {
            lines.push_back(line);
        }
    }
    return lines;
}

TEST(TeleopInterfaceContractTest, FreezesOwnerServiceSchemas) {
    EXPECT_EQ(
        readSchemaLines("srv/AcquireControlOwner.srv"),
        (std::vector<std::string>{"string source_type", "string session_id", "string run_id", "string resource_id",
                                  "string source_id", "uint64 requested_ttl_ms", "---", "bool granted",
                                  "string reason_code", "uint64 owner_epoch", "uint64 expires_at_steady_ns"}));
    EXPECT_EQ(readSchemaLines("srv/RenewControlOwner.srv"),
              (std::vector<std::string>{"string source_type", "string session_id", "string run_id",
                                        "string resource_id", "string source_id", "uint64 owner_epoch",
                                        "uint64 requested_ttl_ms", "---", "bool renewed", "string reason_code",
                                        "uint64 owner_epoch", "uint64 expires_at_steady_ns"}));
    EXPECT_EQ(readSchemaLines("srv/ReleaseControlOwner.srv"),
              (std::vector<std::string>{"string source_type", "string session_id", "string run_id",
                                        "string resource_id", "string source_id", "uint64 owner_epoch",
                                        "string reason_code", "---", "bool released", "string result_code"}));
}

TEST(TeleopInterfaceContractTest, PlacesOwnerEpochBeforeSequence) {
    const auto schema = readSchemaLines("msg/TeleopCommand.msg");
    const auto expected = std::vector<std::string>{
        "string session_id",
        "string run_id",
        "string resource_id",
        "string source_peer_id",
        "string channel_label",
        "uint64 owner_epoch",
        "uint64 sequence",
        "uint64 source_timestamp_ns",
        "uint64 receive_steady_time_ns",
        "uint64 valid_until_ns",
        "bool deadman",
        "bool right_arm_valid",
        "bool left_arm_valid",
        "bool right_gripper_valid",
        "bool left_gripper_valid",
        "bool head_valid",
        "geometry_msgs/Pose right_arm_target",
        "geometry_msgs/Pose left_arm_target",
        "geometry_msgs/Pose head_target",
        "float32 right_gripper",
        "float32 left_gripper",
        "geometry_msgs/Twist chassis_command",
        "bool shadow_only",
    };
    EXPECT_EQ(schema, expected);
}

TEST(TeleopInterfaceContractTest, MapsDataCollectionTeleopStatusConstants) {
    using Request = astrabot_data_interfaces::srv::ReportTeleopStatus::Request;
    EXPECT_FALSE(TeleopStatusReportScheduler::reportState(TeleopState::kIdle).ok());
    EXPECT_FALSE(TeleopStatusReportScheduler::reportState(TeleopState::kAuthorized).ok());
    EXPECT_EQ(TeleopStatusReportScheduler::reportState(TeleopState::kConnected).value(), Request::STATE_CONNECTED);
    EXPECT_EQ(TeleopStatusReportScheduler::reportState(TeleopState::kArmed).value(), Request::STATE_CONNECTED);
    EXPECT_EQ(TeleopStatusReportScheduler::reportState(TeleopState::kControlling).value(), Request::STATE_CONNECTED);
    EXPECT_EQ(TeleopStatusReportScheduler::reportState(TeleopState::kStopping).value(), Request::STATE_DISCONNECTED);
    EXPECT_EQ(TeleopStatusReportScheduler::reportState(TeleopState::kClosed).value(), Request::STATE_DISCONNECTED);
    EXPECT_EQ(TeleopStatusReportScheduler::reportState(TeleopState::kFault).value(), Request::STATE_DISCONNECTED);

    Request request;
    request.request_id = "teleop_contract_1";
    request.session_id = "session-1";
    request.run_id = "run-1";
    request.resource_id = "thor";
    request.state = Request::STATE_CONNECTED;
    request.last_sequence = 10U;
    request.sequence_gap_count = 2U;
    request.device_ts = 1700000000;
    request.reason_code = "connected";
    EXPECT_EQ(request.state, 0U);
}

TEST(TeleopInterfaceContractTest, RequiresExplicitRunContextPublisherGeneration) {
    astrabot_data_interfaces::msg::DataCollectionRunContext context;
    context.publisher_generation = "6d5ebdf8-7688-4c33-b795-e98fd960eda9";
    context.run_id = "run-1";
    context.resource_id = "thor";
    context.state = astrabot_data_interfaces::msg::DataCollectionRunContext::STATE_ACTIVE;
    context.context_version = 1U;
    EXPECT_FALSE(context.publisher_generation.empty());
    EXPECT_EQ(context.context_version, 1U);
}

TEST(TeleopInterfaceContractTest, ReadyPermitsAuthorizationButOnlyActivePermitsControl) {
    std::ifstream input(std::string(ASTRABOT_TELEOP_SOURCE_DIR) + "/src/runtime/teleop_node.cpp");
    ASSERT_TRUE(input.good());
    const std::string source((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    const std::size_t authorization_begin = source.find("bool runContextAllowsAuthorization");
    const std::size_t control_begin = source.find("bool runContextAllowsControl");
    const std::size_t policy_end = source.find("}  // namespace", control_begin);
    ASSERT_NE(authorization_begin, std::string::npos);
    ASSERT_NE(control_begin, std::string::npos);
    ASSERT_NE(policy_end, std::string::npos);
    const std::string authorization_policy = source.substr(authorization_begin, control_begin - authorization_begin);
    const std::string control_policy = source.substr(control_begin, policy_end - control_begin);

    EXPECT_NE(authorization_policy.find("DataCollectionRunContext::STATE_READY"), std::string::npos);
    EXPECT_NE(authorization_policy.find("DataCollectionRunContext::STATE_ACTIVE"), std::string::npos);
    EXPECT_EQ(control_policy.find("DataCollectionRunContext::STATE_READY"), std::string::npos);
    EXPECT_NE(control_policy.find("DataCollectionRunContext::STATE_ACTIVE"), std::string::npos);
}

TEST(TeleopInterfaceContractTest, ProductionAuthorizationChecksReadyOrActiveRunBeforeCommit) {
    std::ifstream input(std::string(ASTRABOT_TELEOP_SOURCE_DIR) + "/src/runtime/teleop_node.cpp");
    ASSERT_TRUE(input.good());
    const std::string source((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    const std::size_t authorize_begin = source.find("void TeleopNode::handleAuthorize");
    const std::size_t authorize_end = source.find("void TeleopNode::handlePeerEvent", authorize_begin);
    ASSERT_NE(authorize_begin, std::string::npos);
    ASSERT_NE(authorize_end, std::string::npos);
    const std::string authorize = source.substr(authorize_begin, authorize_end - authorize_begin);

    const std::size_t precheck =
        authorize.find("authorizationRunContextMatchesLocked(request->run_id, request->resource_id)");
    const std::size_t grant_consume = authorize.find("verifyAndConsume");
    const std::size_t commit_recheck = authorize.find("authorizationRunContextMatchesLocked(binding)");
    const std::size_t session_commit =
        authorize.find("session_registry_.authorize(binding, config_.shadowOutputEnabled())");
    ASSERT_NE(precheck, std::string::npos);
    ASSERT_NE(grant_consume, std::string::npos);
    ASSERT_NE(commit_recheck, std::string::npos);
    ASSERT_NE(session_commit, std::string::npos);
    EXPECT_LT(precheck, grant_consume);
    EXPECT_LT(grant_consume, commit_recheck);
    EXPECT_LT(commit_recheck, session_commit);
}

TEST(TeleopInterfaceContractTest, ConnectsOnlyAfterAuthorizedDataChannelOpenEvent) {
    std::ifstream input(std::string(ASTRABOT_TELEOP_SOURCE_DIR) + "/src/runtime/teleop_node.cpp");
    ASSERT_TRUE(input.good());
    const std::string source((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    const std::size_t authorize_begin = source.find("void TeleopNode::handleAuthorize");
    const std::size_t peer_begin = source.find("void TeleopNode::handlePeerEvent", authorize_begin);
    const std::size_t packet_begin = source.find("void TeleopNode::handleDataPacket", peer_begin);
    ASSERT_NE(authorize_begin, std::string::npos);
    ASSERT_NE(peer_begin, std::string::npos);
    ASSERT_NE(packet_begin, std::string::npos);

    const std::string authorize = source.substr(authorize_begin, peer_begin - authorize_begin);
    const std::string peer_handler = source.substr(peer_begin, packet_begin - peer_begin);
    EXPECT_EQ(authorize.find("markConnected"), std::string::npos);
    EXPECT_EQ(authorize.find("TeleopEvent::kPeerConnected"), std::string::npos);
    EXPECT_EQ(authorize.find("watchdog_->observe"), std::string::npos);
    EXPECT_NE(authorize.find("watchdog_->reset()"), std::string::npos);

    const std::size_t open_reason = peer_handler.find("data_channel_open");
    const std::size_t mark_connected = peer_handler.find("session_registry_.markConnected", open_reason);
    const std::size_t state_connected = peer_handler.find("TeleopEvent::kPeerConnected", mark_connected);
    const std::size_t watchdog_start = peer_handler.find("watchdog_->observe", state_connected);
    ASSERT_NE(open_reason, std::string::npos);
    ASSERT_NE(mark_connected, std::string::npos);
    ASSERT_NE(state_connected, std::string::npos);
    ASSERT_NE(watchdog_start, std::string::npos);
    EXPECT_LT(open_reason, mark_connected);
    EXPECT_LT(mark_connected, state_connected);
    EXPECT_LT(state_connected, watchdog_start);
    EXPECT_NE(peer_handler.find("data_channel_closed"), std::string::npos);
    EXPECT_NE(peer_handler.find("stopActiveSession(\"rtc_data_channel_closed\", false)"), std::string::npos);
}

TEST(TeleopInterfaceContractTest, RealControlBackendsRequireQuestSafetyFlags) {
    std::ifstream input(std::string(ASTRABOT_TELEOP_SOURCE_DIR) + "/src/runtime/teleop_node.cpp");
    ASSERT_TRUE(input.good());
    const std::string source((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    const std::size_t mapper_begin = source.find("std::make_unique<CommandMapper>");
    const std::size_t mapper_end = source.find("std::make_unique<ArmOriginMapper>", mapper_begin);
    ASSERT_NE(mapper_begin, std::string::npos);
    ASSERT_NE(mapper_end, std::string::npos);
    const std::string mapper_config = source.substr(mapper_begin, mapper_end - mapper_begin);
    EXPECT_NE(mapper_config.find("config_.controlOutputEnabled()"), std::string::npos);
}

TEST(TeleopInterfaceContractTest, ReadyPausesOwnerWithoutClosingRtcSession) {
    std::ifstream input(std::string(ASTRABOT_TELEOP_SOURCE_DIR) + "/src/runtime/teleop_node.cpp");
    ASSERT_TRUE(input.good());
    const std::string source((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    const std::size_t handler_begin = source.find("void TeleopNode::handleRunContext");
    const std::size_t handler_end = source.find("void TeleopNode::processLatestPacket", handler_begin);
    const std::size_t pause_begin = source.find("void TeleopNode::pauseControlForReady");
    const std::size_t pause_end = source.find("void TeleopNode::stopActiveSession", pause_begin);
    ASSERT_NE(handler_begin, std::string::npos);
    ASSERT_NE(handler_end, std::string::npos);
    ASSERT_NE(pause_begin, std::string::npos);
    ASSERT_NE(pause_end, std::string::npos);
    const std::string handler = source.substr(handler_begin, handler_end - handler_begin);
    const std::string pause = source.substr(pause_begin, pause_end - pause_begin);

    EXPECT_NE(handler.find("DataCollectionRunContext::STATE_READY"), std::string::npos);
    EXPECT_NE(handler.find("pauseControlForReady(reason_code)"), std::string::npos);
    EXPECT_NE(pause.find("TeleopEvent::kOwnerReleased"), std::string::npos);
    EXPECT_NE(pause.find("requestOwnerRelease(reason_code, false)"), std::string::npos);
    EXPECT_NE(pause.find("watchdog_->reset()"), std::string::npos);
    EXPECT_EQ(pause.find("requestRtcClose"), std::string::npos);
    EXPECT_EQ(pause.find("session_registry_.close"), std::string::npos);
}

TEST(TeleopInterfaceContractTest, ProductionPacketsRequireActiveRunBeforeDecode) {
    std::ifstream input(std::string(ASTRABOT_TELEOP_SOURCE_DIR) + "/src/runtime/teleop_node.cpp");
    ASSERT_TRUE(input.good());
    const std::string source((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    const std::size_t packet_begin = source.find("void TeleopNode::processPacket");
    const std::size_t packet_end = source.find("void TeleopNode::checkAuthorizationDeadline", packet_begin);
    ASSERT_NE(packet_begin, std::string::npos);
    ASSERT_NE(packet_end, std::string::npos);
    const std::string packet = source.substr(packet_begin, packet_end - packet_begin);

    const std::size_t active_gate = packet.find("activeRunContextMatchesLocked(*control_binding)");
    const std::size_t decode = packet.find("codec_->decode");
    ASSERT_NE(active_gate, std::string::npos);
    ASSERT_NE(decode, std::string::npos);
    EXPECT_LT(active_gate, decode);
}

TEST(TeleopInterfaceContractTest, ReadySerializesAcquireAndRenewFailureRaces) {
    std::ifstream input(std::string(ASTRABOT_TELEOP_SOURCE_DIR) + "/src/runtime/teleop_node.cpp");
    ASSERT_TRUE(input.good());
    const std::string source((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    const std::size_t run_begin = source.find("void TeleopNode::handleRunContext");
    const std::size_t run_end = source.find("void TeleopNode::processLatestPacket", run_begin);
    const std::size_t failure_begin = source.find("void TeleopNode::handleOwnerFailure");
    const std::size_t failure_end = source.find("void TeleopNode::requestRtcClose", failure_begin);
    ASSERT_NE(run_begin, std::string::npos);
    ASSERT_NE(run_end, std::string::npos);
    ASSERT_NE(failure_begin, std::string::npos);
    ASSERT_NE(failure_end, std::string::npos);
    const std::string run_handler = source.substr(run_begin, run_end - run_begin);
    const std::string owner_failure = source.substr(failure_begin, failure_end - failure_begin);

    EXPECT_NE(run_handler.find("control_transition_mutex_"), std::string::npos);
    EXPECT_NE(owner_failure.find("control_transition_mutex_"), std::string::npos);
    EXPECT_NE(owner_failure.find("stopActiveSession(reason_code, true, true, true)"), std::string::npos);
}

TEST(TeleopInterfaceContractTest, ReadyPreservesSessionForLeaseExpiryAndPendingTimeout) {
    std::ifstream input(std::string(ASTRABOT_TELEOP_SOURCE_DIR) + "/src/runtime/teleop_node.cpp");
    ASSERT_TRUE(input.good());
    const std::string source((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    const std::size_t stop_begin = source.find("void TeleopNode::stopActiveSession");
    const std::size_t stop_end = source.find("void TeleopNode::handleOwnerFailure", stop_begin);
    const std::size_t cancel_begin = source.find("void TeleopNode::quarantinePendingOwnerRequestForCleanup");
    const std::size_t cancel_end = source.find("bool TeleopNode::ownerCleanupPending", cancel_begin);
    ASSERT_NE(stop_begin, std::string::npos);
    ASSERT_NE(stop_end, std::string::npos);
    ASSERT_NE(cancel_begin, std::string::npos);
    ASSERT_NE(cancel_end, std::string::npos);
    const std::string stop = source.substr(stop_begin, stop_end - stop_begin);
    const std::string ready_quarantine = source.substr(cancel_begin, cancel_end - cancel_begin);

    const std::size_t ready_gate = stop.find("readyRunContextMatchesLocked");
    const std::size_t ready_pause = stop.find("pauseControlForReadyLocked");
    const std::size_t preserved_return = stop.find("return;", ready_pause);
    const std::size_t rtc_close = stop.find("requestRtcClose");
    ASSERT_NE(ready_gate, std::string::npos);
    ASSERT_NE(ready_pause, std::string::npos);
    ASSERT_NE(preserved_return, std::string::npos);
    ASSERT_NE(rtc_close, std::string::npos);
    EXPECT_LT(ready_gate, ready_pause);
    EXPECT_LT(ready_pause, preserved_return);
    EXPECT_LT(preserved_return, rtc_close);
    EXPECT_NE(ready_quarantine.find("ControlOwnerOperationKind::kRelease"), std::string::npos);
    EXPECT_NE(ready_quarantine.find("pending_owner_cleanup_request_"), std::string::npos);
    EXPECT_NE(ready_quarantine.find("owner_tracker_->cancelPending()"), std::string::npos);
}

TEST(TeleopInterfaceContractTest, LateAcquireAndRenewCallbacksReleaseStaleOwnerWithoutClosingCurrentPeer) {
    std::ifstream input(std::string(ASTRABOT_TELEOP_SOURCE_DIR) + "/src/runtime/teleop_node.cpp");
    ASSERT_TRUE(input.good());
    const std::string source((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    const std::size_t acquire_begin = source.find("void TeleopNode::handleOwnerAcquireResponse");
    const std::size_t renew_begin = source.find("void TeleopNode::handleOwnerRenewResponse", acquire_begin);
    const std::size_t release_begin = source.find("void TeleopNode::handleOwnerReleaseResponse", renew_begin);
    ASSERT_NE(acquire_begin, std::string::npos);
    ASSERT_NE(renew_begin, std::string::npos);
    ASSERT_NE(release_begin, std::string::npos);
    const std::string acquire = source.substr(acquire_begin, renew_begin - acquire_begin);
    const std::string renew = source.substr(renew_begin, release_begin - renew_begin);

    EXPECT_NE(acquire.find("stale_acquire_callback"), std::string::npos);
    EXPECT_NE(acquire.find("requestOwnerCleanupRelease"), std::string::npos);
    EXPECT_NE(renew.find("stale_renew_callback"), std::string::npos);
    EXPECT_NE(renew.find("requestOwnerCleanupRelease"), std::string::npos);
    EXPECT_NE(renew.find("renew_completed_after_control_pause"), std::string::npos);
    EXPECT_EQ(acquire.find("requestRtcClose"), std::string::npos);
    EXPECT_EQ(renew.find("requestRtcClose"), std::string::npos);
}

TEST(TeleopInterfaceContractTest, IdleContextAdvancesPublisherGenerationBeforeRunIdentityValidation) {
    std::ifstream input(std::string(ASTRABOT_TELEOP_SOURCE_DIR) + "/src/runtime/teleop_node.cpp");
    ASSERT_TRUE(input.good());
    const std::string source((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    const std::size_t handler_begin = source.find("void TeleopNode::handleRunContext");
    const std::size_t handler_end = source.find("void TeleopNode::processLatestPacket", handler_begin);
    ASSERT_NE(handler_begin, std::string::npos);
    ASSERT_NE(handler_end, std::string::npos);
    const std::string handler = source.substr(handler_begin, handler_end - handler_begin);

    const std::size_t generation_observe = handler.find("run_context_generation_tracker_.observe");
    const std::size_t run_identity_validation = handler.find("const bool run_identity_valid");
    const std::size_t idle_exception = handler.find("DataCollectionRunContext::STATE_IDLE", run_identity_validation);
    const std::size_t retired_rejection = handler.find("RunContextGenerationObservation::kRetired");
    ASSERT_NE(generation_observe, std::string::npos);
    ASSERT_NE(run_identity_validation, std::string::npos);
    ASSERT_NE(idle_exception, std::string::npos);
    ASSERT_NE(retired_rejection, std::string::npos);
    EXPECT_LT(generation_observe, run_identity_validation);
    EXPECT_LT(run_identity_validation, retired_rejection);
}

TEST(TeleopInterfaceContractTest, PublisherRestartPreservesSameAuthorizedRun) {
    std::ifstream input(std::string(ASTRABOT_TELEOP_SOURCE_DIR) + "/src/runtime/teleop_node.cpp");
    ASSERT_TRUE(input.good());
    const std::string source((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    const std::size_t handler_begin = source.find("void TeleopNode::handleRunContext");
    const std::size_t handler_end = source.find("void TeleopNode::processLatestPacket", handler_begin);
    ASSERT_NE(handler_begin, std::string::npos);
    ASSERT_NE(handler_end, std::string::npos);
    const std::string handler = source.substr(handler_begin, handler_end - handler_begin);

    const std::size_t changed = handler.find("RunContextGenerationObservation::kChanged");
    const std::size_t same_authorized_run = handler.find("const bool same_authorized_run", changed);
    const std::size_t conditional_stop =
        handler.find("binding.has_value() && !same_authorized_run", same_authorized_run);
    ASSERT_NE(changed, std::string::npos);
    ASSERT_NE(same_authorized_run, std::string::npos);
    ASSERT_NE(conditional_stop, std::string::npos);
    EXPECT_NE(handler.find("message->run_id == binding->run_id", same_authorized_run), std::string::npos);
    EXPECT_NE(handler.find("message->resource_id == binding->resource_id", same_authorized_run), std::string::npos);
}

}  // namespace
}  // namespace astrabot::teleop
