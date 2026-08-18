// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include "astrabot_teleop/runtime/teleop_node.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#include "astrabot_teleop/grant/grant_expiry.h"
#include "astrabot_teleop/protocol/data_channel_contract.h"

namespace astrabot::teleop {
namespace {

constexpr std::uint64_t kNanosecondsPerMillisecond = 1000000U;
constexpr std::uint64_t kNanosecondsPerSecond = 1000000000U;
constexpr std::uint64_t kCloseRequestTimeoutNs = 1000U * kNanosecondsPerMillisecond;
constexpr std::size_t kMaxRunContextGenerationLength = 128U;
constexpr std::size_t kMaxCleanupReleaseRequests = 4U;

enum class PacketAction {
    kNone,
    kPublish,
    kAcquireOwner,
    kReleaseOwner,
};

std::string reasonCodeForStatus(const Status &status) {
    switch (status.code()) {
        case ErrorCode::kInvalidArgument:
            return "invalid_input";
        case ErrorCode::kUnavailable:
            return "dependency_unavailable";
        case ErrorCode::kUnauthorized:
            return "unauthorized";
        case ErrorCode::kConflict:
            return "replay_or_conflict";
        case ErrorCode::kDataLoss:
            return "data_integrity_failed";
        case ErrorCode::kFailedPrecondition:
            return "invalid_state";
        case ErrorCode::kDeadlineExceeded:
            return "stale_or_timeout";
        case ErrorCode::kResourceExhausted:
            return "resource_limit";
        case ErrorCode::kInternal:
            return "internal_error";
        case ErrorCode::kOk:
            return "ok";
    }
    return "unknown_error";
}

void assignPose(const PoseSample &source, geometry_msgs::msg::Pose *target) {
    target->position.x = source.x;
    target->position.y = source.y;
    target->position.z = source.z;
    target->orientation.x = source.qx;
    target->orientation.y = source.qy;
    target->orientation.z = source.qz;
    target->orientation.w = source.qw;
}

PoseSample poseSample(const geometry_msgs::msg::Pose &source) {
    PoseSample pose;
    pose.present = true;
    pose.x = source.position.x;
    pose.y = source.position.y;
    pose.z = source.position.z;
    pose.qx = source.orientation.x;
    pose.qy = source.orientation.y;
    pose.qz = source.orientation.z;
    pose.qw = source.orientation.w;
    return pose;
}

Result<std::uint64_t> ownerRenewDeadline(const std::uint64_t steady_now_ns, const std::uint64_t expires_at_steady_ns,
                                         const std::uint64_t renew_period_ns, const std::uint64_t service_timeout_ns) {
    if (renew_period_ns == 0U || service_timeout_ns == 0U || expires_at_steady_ns <= steady_now_ns ||
        expires_at_steady_ns - steady_now_ns <= service_timeout_ns ||
        steady_now_ns > std::numeric_limits<std::uint64_t>::max() - renew_period_ns) {
        return Result<std::uint64_t>::failure(
            Status::error(ErrorCode::kDeadlineExceeded, "owner lease has no safe renew margin"));
    }
    const std::uint64_t desired_deadline = steady_now_ns + renew_period_ns;
    const std::uint64_t latest_safe_deadline = expires_at_steady_ns - service_timeout_ns;
    const std::uint64_t deadline = std::min(desired_deadline, latest_safe_deadline);
    if (deadline <= steady_now_ns) {
        return Result<std::uint64_t>::failure(
            Status::error(ErrorCode::kDeadlineExceeded, "owner renew deadline is already due"));
    }
    return Result<std::uint64_t>::success(deadline);
}

bool safePublisherGeneration(const std::string &value) {
    return !value.empty() && value.size() <= kMaxRunContextGenerationLength &&
           std::all_of(value.begin(), value.end(),
                       [](const unsigned char character) { return character >= 0x20U && character != 0x7FU; });
}

bool sameRunContext(const astrabot_data_interfaces::msg::DataCollectionRunContext &first,
                    const astrabot_data_interfaces::msg::DataCollectionRunContext &second) {
    return first.publisher_generation == second.publisher_generation && first.run_id == second.run_id &&
           first.resource_id == second.resource_id && first.graph_id == second.graph_id &&
           first.revision_id == second.revision_id && first.task_id == second.task_id &&
           first.controller_session_id == second.controller_session_id && first.collector == second.collector &&
           first.model_name == second.model_name && first.sampling_profile == second.sampling_profile &&
           first.state == second.state && first.context_version == second.context_version;
}

bool runContextAllowsAuthorization(const std::uint8_t state) {
    return state == astrabot_data_interfaces::msg::DataCollectionRunContext::STATE_READY ||
           state == astrabot_data_interfaces::msg::DataCollectionRunContext::STATE_ACTIVE;
}

bool runContextAllowsControl(const std::uint8_t state) {
    return state == astrabot_data_interfaces::msg::DataCollectionRunContext::STATE_ACTIVE;
}

}  // namespace

TeleopNode::TeleopNode(const rclcpp::NodeOptions &options) : rclcpp::Node("astrabot_teleop", options) {
    declare_parameter<std::string>("backend", "shadow");
    declare_parameter<std::string>("device_id", "");
    declare_parameter<std::string>("resource_id", "thor");
    declare_parameter<std::string>("rtc_data_topic", "/astrabot/rtc/data_channel/received");
    declare_parameter<std::string>("rtc_peer_event_topic", "/astrabot/rtc/peer_event");
    declare_parameter<std::string>("authorize_service", "/astrabot/teleop/authorize_channel");
    declare_parameter<std::string>("rtc_close_service", "/astrabot/rtc/close_peer");
    declare_parameter<std::string>("shadow_command_topic", "/astrabot/teleop/shadow_command");
    declare_parameter<std::string>("production_command_topic", "/astrabot/teleop/command");
    declare_parameter<std::string>("legacy_reference_pose_topic", "/reference/pose");
    declare_parameter<std::string>("legacy_control_name", "wbmpc_remote_ctrl");
    declare_parameter<std::string>("legacy_left_gripper_topic", "/rm_left/rm_driver/teleop_gripper_float");
    declare_parameter<std::string>("legacy_right_gripper_topic", "/rm_right/rm_driver/teleop_gripper_float");
    declare_parameter<std::string>("status_topic", "/astrabot/teleop/session_status");
    declare_parameter<std::string>("run_context_topic", "/astrabot/data_collection/run_context");
    declare_parameter<std::string>("acquire_owner_service", "/astrabot/arbitration/acquire_owner");
    declare_parameter<std::string>("renew_owner_service", "/astrabot/arbitration/renew_owner");
    declare_parameter<std::string>("release_owner_service", "/astrabot/arbitration/release_owner");
    declare_parameter<std::string>("report_teleop_status_service", "/astrabot/data_collection/report_teleop_status");
    declare_parameter<std::string>("robot_ee_pose_topic", "/ee/pose");
    declare_parameter<std::string>("robot_base_frame", "base_link");
    declare_parameter<std::vector<std::string>>("grant_key_ids", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("grant_public_keys", std::vector<std::string>{});
    declare_parameter<std::int64_t>("max_frame_bytes", 16384);
    declare_parameter<std::int64_t>("max_frame_age_ms", 100);
    declare_parameter<std::int64_t>("max_future_skew_ms", 1000);
    declare_parameter<std::int64_t>("watchdog_timeout_ms", 120);
    declare_parameter<std::int64_t>("command_ttl_ms", 100);
    declare_parameter<std::int64_t>("process_period_ms", 2);
    declare_parameter<std::int64_t>("status_period_ms", 1000);
    declare_parameter<std::int64_t>("owner_ttl_ms", 150);
    declare_parameter<std::int64_t>("owner_renew_period_ms", 50);
    declare_parameter<std::int64_t>("owner_service_timeout_ms", 40);
    declare_parameter<std::int64_t>("status_report_service_timeout_ms", 100);
    declare_parameter<std::int64_t>("status_report_retry_period_ms", 1000);
    declare_parameter<std::int64_t>("status_report_poll_period_ms", 10);
    declare_parameter<std::int64_t>("robot_pose_max_age_ms", 100);
    declare_parameter<double>("deadman_grip_threshold", 0.5);
    declare_parameter<double>("gripper_toggle_high_threshold", 0.9);
    declare_parameter<double>("gripper_toggle_low_threshold", 0.1);
    declare_parameter<double>("gripper_binary_threshold", 0.4);
    declare_parameter<double>("quaternion_norm_tolerance", 0.05);
    declare_parameter<double>("workspace_min_x", -5.0);
    declare_parameter<double>("workspace_max_x", 5.0);
    declare_parameter<double>("workspace_min_y", -5.0);
    declare_parameter<double>("workspace_max_y", 5.0);
    declare_parameter<double>("workspace_min_z", -5.0);
    declare_parameter<double>("workspace_max_z", 5.0);
    declare_parameter<double>("max_position_step_m", 0.25);
    declare_parameter<double>("max_position_velocity_mps", 1.0);
    declare_parameter<double>("max_position_acceleration_mps2", 5.0);
    declare_parameter<double>("max_chassis_linear_mps", 0.3);
    declare_parameter<double>("max_chassis_angular_rps", 0.5);
}

TeleopNode::~TeleopNode() {
    stop();
}

Status TeleopNode::start() {
    if (stopped_permanently_.load()) {
        return Status::error(ErrorCode::kFailedPrecondition, "TeleopNode cannot restart after stop");
    }
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return Status::success();
    }

    Status status = loadConfig();
    if (!status.ok()) {
        running_.store(false);
        return status;
    }
    status = initializeCore();
    if (!status.ok()) {
        running_.store(false);
        return status;
    }
    status = initializeRosEndpoints();
    if (!status.ok()) {
        stop();
        return status;
    }
    publishStatus();
    return Status::success();
}

void TeleopNode::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    stopped_permanently_.store(true);
    std::lock_guard<std::mutex> authorization_lock(authorization_mutex_);
    stopActiveSession("runtime_stopped", true);
    if (status_report_client_) {
        status_report_client_->stop();
    }
    process_timer_.reset();
    status_timer_.reset();
    data_subscription_.reset();
    peer_subscription_.reset();
    robot_ee_pose_subscription_.reset();
    run_context_subscription_.reset();
    authorize_service_.reset();
    cancelPendingOwnerRequest();

    std::vector<std::int64_t> pending_ids;
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_ids.reserve(pending_close_requests_.size());
        for (const auto &entry : pending_close_requests_) {
            pending_ids.push_back(entry.second.request_id);
        }
        pending_close_requests_.clear();
    }
    if (close_client_) {
        for (const auto request_id : pending_ids) {
            close_client_->remove_pending_request(request_id);
        }
    }
    close_client_.reset();
    acquire_owner_client_.reset();
    renew_owner_client_.reset();
    release_owner_client_.reset();
    resetRosEndpoints();
    mailbox_.clear();
    session_registry_.clear();
    grant_verifier_.reset();
    codec_.reset();
    frame_validator_.reset();
    command_limiter_.reset();
    command_mapper_.reset();
    arm_origin_mapper_.reset();
    legacy_mpc_encoder_.reset();
    watchdog_.reset();
    owner_tracker_.reset();
    status_report_client_.reset();
}

Status TeleopNode::loadConfig() {
    config_.backend = get_parameter("backend").as_string();
    config_.device_id = get_parameter("device_id").as_string();
    config_.resource_id = get_parameter("resource_id").as_string();
    config_.rtc_data_topic = get_parameter("rtc_data_topic").as_string();
    config_.rtc_peer_event_topic = get_parameter("rtc_peer_event_topic").as_string();
    config_.authorize_service = get_parameter("authorize_service").as_string();
    config_.rtc_close_service = get_parameter("rtc_close_service").as_string();
    config_.shadow_command_topic = get_parameter("shadow_command_topic").as_string();
    config_.production_command_topic = get_parameter("production_command_topic").as_string();
    config_.legacy_reference_pose_topic = get_parameter("legacy_reference_pose_topic").as_string();
    config_.legacy_control_name = get_parameter("legacy_control_name").as_string();
    config_.legacy_left_gripper_topic = get_parameter("legacy_left_gripper_topic").as_string();
    config_.legacy_right_gripper_topic = get_parameter("legacy_right_gripper_topic").as_string();
    config_.status_topic = get_parameter("status_topic").as_string();
    config_.run_context_topic = get_parameter("run_context_topic").as_string();
    config_.acquire_owner_service = get_parameter("acquire_owner_service").as_string();
    config_.renew_owner_service = get_parameter("renew_owner_service").as_string();
    config_.release_owner_service = get_parameter("release_owner_service").as_string();
    config_.report_teleop_status_service = get_parameter("report_teleop_status_service").as_string();
    config_.robot_ee_pose_topic = get_parameter("robot_ee_pose_topic").as_string();
    config_.robot_base_frame = get_parameter("robot_base_frame").as_string();

    const auto key_ids = get_parameter("grant_key_ids").as_string_array();
    const auto public_keys = get_parameter("grant_public_keys").as_string_array();
    if (key_ids.size() != public_keys.size()) {
        return Status::error(ErrorCode::kInvalidArgument, "grant_key_ids and grant_public_keys must have equal size");
    }
    config_.grant_public_keys.clear();
    config_.grant_public_keys.reserve(key_ids.size());
    for (std::size_t index = 0; index < key_ids.size(); ++index) {
        config_.grant_public_keys.push_back(GrantPublicKeyConfig{key_ids[index], public_keys[index]});
    }

    const auto max_frame_bytes = get_parameter("max_frame_bytes").as_int();
    config_.max_frame_bytes = max_frame_bytes > 0 ? static_cast<std::size_t>(max_frame_bytes) : 0U;
    config_.max_frame_age_ms = get_parameter("max_frame_age_ms").as_int();
    config_.max_future_skew_ms = get_parameter("max_future_skew_ms").as_int();
    config_.watchdog_timeout_ms = get_parameter("watchdog_timeout_ms").as_int();
    config_.command_ttl_ms = get_parameter("command_ttl_ms").as_int();
    config_.process_period_ms = get_parameter("process_period_ms").as_int();
    config_.status_period_ms = get_parameter("status_period_ms").as_int();
    config_.owner_ttl_ms = get_parameter("owner_ttl_ms").as_int();
    config_.owner_renew_period_ms = get_parameter("owner_renew_period_ms").as_int();
    config_.owner_service_timeout_ms = get_parameter("owner_service_timeout_ms").as_int();
    config_.status_report_service_timeout_ms = get_parameter("status_report_service_timeout_ms").as_int();
    config_.status_report_retry_period_ms = get_parameter("status_report_retry_period_ms").as_int();
    config_.status_report_poll_period_ms = get_parameter("status_report_poll_period_ms").as_int();
    config_.robot_pose_max_age_ms = get_parameter("robot_pose_max_age_ms").as_int();
    config_.deadman_grip_threshold = get_parameter("deadman_grip_threshold").as_double();
    config_.gripper_toggle_high_threshold = get_parameter("gripper_toggle_high_threshold").as_double();
    config_.gripper_toggle_low_threshold = get_parameter("gripper_toggle_low_threshold").as_double();
    config_.gripper_binary_threshold = get_parameter("gripper_binary_threshold").as_double();
    config_.quaternion_norm_tolerance = get_parameter("quaternion_norm_tolerance").as_double();
    config_.workspace_min_x = get_parameter("workspace_min_x").as_double();
    config_.workspace_max_x = get_parameter("workspace_max_x").as_double();
    config_.workspace_min_y = get_parameter("workspace_min_y").as_double();
    config_.workspace_max_y = get_parameter("workspace_max_y").as_double();
    config_.workspace_min_z = get_parameter("workspace_min_z").as_double();
    config_.workspace_max_z = get_parameter("workspace_max_z").as_double();
    config_.max_position_step_m = get_parameter("max_position_step_m").as_double();
    config_.max_position_velocity_mps = get_parameter("max_position_velocity_mps").as_double();
    config_.max_position_acceleration_mps2 = get_parameter("max_position_acceleration_mps2").as_double();
    config_.max_chassis_linear_mps = get_parameter("max_chassis_linear_mps").as_double();
    config_.max_chassis_angular_rps = get_parameter("max_chassis_angular_rps").as_double();
    return config_.validate();
}

Status TeleopNode::initializeCore() {
    grant_verifier_ = std::make_unique<GrantVerifier>();
    if (config_.backend != "disabled") {
        const Status grant_status = grant_verifier_->initialize(config_.grant_public_keys);
        if (!grant_status.ok()) {
            return grant_status;
        }
    }
    codec_ = std::make_unique<TeleopFrameCodec>(config_.max_frame_bytes);
    frame_validator_ = std::make_unique<FrameValidator>(FrameValidationConfig{
        config_.max_frame_age_ms, config_.max_future_skew_ms, config_.quaternion_norm_tolerance, 26U});
    command_limiter_ = std::make_unique<CommandLimiter>(CommandLimitConfig{
        config_.workspace_min_x, config_.workspace_max_x, config_.workspace_min_y, config_.workspace_max_y,
        config_.workspace_min_z, config_.workspace_max_z, config_.max_position_step_m,
        config_.max_position_velocity_mps, config_.max_position_acceleration_mps2});
    command_mapper_ = std::make_unique<CommandMapper>(CommandMappingConfig{
        config_.deadman_grip_threshold, config_.max_chassis_linear_mps, config_.max_chassis_angular_rps,
        config_.gripper_toggle_high_threshold, config_.gripper_toggle_low_threshold, config_.gripper_binary_threshold,
        config_.controlOutputEnabled()});
    arm_origin_mapper_ = std::make_unique<ArmOriginMapper>(
        ArmOriginMappingConfig{static_cast<std::uint64_t>(config_.robot_pose_max_age_ms) * kNanosecondsPerMillisecond,
                               config_.quaternion_norm_tolerance});
    watchdog_ = std::make_unique<MotionWatchdog>(static_cast<std::uint64_t>(config_.watchdog_timeout_ms) *
                                                 kNanosecondsPerMillisecond);
    owner_tracker_ = std::make_unique<ControlOwnerLeaseTracker>();
    if (config_.legacyMpcOutputEnabled()) {
        legacy_mpc_encoder_ = std::make_unique<LegacyMpcCommandEncoder>();
    }
    return Status::success();
}

Status TeleopNode::initializeRosEndpoints() {
    if (config_.shadowOutputEnabled()) {
        command_publisher_ = create_publisher<::astrabot_teleop::msg::TeleopCommand>(config_.shadow_command_topic,
                                                                                     rclcpp::QoS(1U).best_effort());
    } else if (config_.productionOutputEnabled()) {
        command_publisher_ = create_publisher<::astrabot_teleop::msg::TeleopCommand>(config_.production_command_topic,
                                                                                     rclcpp::QoS(1U).best_effort());
    } else if (config_.legacyMpcOutputEnabled()) {
        legacy_reference_pose_publisher_ =
            create_publisher<std_msgs::msg::String>(config_.legacy_reference_pose_topic, rclcpp::QoS(1U).reliable());
        legacy_left_gripper_publisher_ =
            create_publisher<std_msgs::msg::Float64>(config_.legacy_left_gripper_topic, rclcpp::QoS(1U).reliable());
        legacy_right_gripper_publisher_ =
            create_publisher<std_msgs::msg::Float64>(config_.legacy_right_gripper_topic, rclcpp::QoS(1U).reliable());
    }
    status_publisher_ = create_publisher<::astrabot_teleop::msg::TeleopSessionStatus>(config_.status_topic,
                                                                                      rclcpp::QoS(32U).reliable());
    data_subscription_ = create_subscription<astrabot_rtc::msg::RtcDataPacket>(
        config_.rtc_data_topic, rclcpp::QoS(1U).best_effort(),
        [this](const astrabot_rtc::msg::RtcDataPacket::ConstSharedPtr message) { handleDataPacket(message); });
    peer_subscription_ = create_subscription<astrabot_rtc::msg::RtcPeerEvent>(
        config_.rtc_peer_event_topic, rclcpp::QoS(32U).reliable(),
        [this](const astrabot_rtc::msg::RtcPeerEvent::ConstSharedPtr message) { handlePeerEvent(message); });
    if (config_.backend != "disabled") {
        robot_ee_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
            config_.robot_ee_pose_topic, rclcpp::QoS(1U).best_effort(),
            [this](const geometry_msgs::msg::PoseArray::ConstSharedPtr message) { handleRobotEePose(message); });
    }
    authorize_service_ = create_service<astrabot_rtc::srv::AuthorizeDataChannel>(
        config_.authorize_service, [this](const astrabot_rtc::srv::AuthorizeDataChannel::Request::SharedPtr request,
                                          astrabot_rtc::srv::AuthorizeDataChannel::Response::SharedPtr response) {
            handleAuthorize(request, response);
        });
    close_client_ = create_client<astrabot_rtc::srv::CloseRtcPeer>(config_.rtc_close_service);
    const auto shared_node = weak_from_this().lock();
    if (!shared_node) {
        return Status::error(ErrorCode::kFailedPrecondition, "TeleopNode must be owned by shared_ptr");
    }
    status_report_client_ = std::make_shared<TeleopStatusReportClient>();
    const std::string report_request_prefix =
        "teleop_" + std::to_string(systemNowNs()) + "_" + std::to_string(steadyNowNs());
    const Status report_status = status_report_client_->start(
        shared_node,
        TeleopStatusReportClientConfig{
            config_.report_teleop_status_service, report_request_prefix,
            static_cast<std::uint64_t>(config_.status_report_service_timeout_ms) * kNanosecondsPerMillisecond,
            static_cast<std::uint64_t>(config_.status_report_retry_period_ms) * kNanosecondsPerMillisecond,
            static_cast<std::uint64_t>(config_.status_report_poll_period_ms) * kNanosecondsPerMillisecond});
    if (!report_status.ok()) {
        return report_status;
    }
    if (config_.productionOutputEnabled()) {
        run_context_subscription_ = create_subscription<astrabot_data_interfaces::msg::DataCollectionRunContext>(
            config_.run_context_topic, rclcpp::QoS(rclcpp::KeepLast(1U)).reliable().transient_local(),
            [this](const astrabot_data_interfaces::msg::DataCollectionRunContext::ConstSharedPtr message) {
                handleRunContext(message);
            });
        acquire_owner_client_ =
            create_client<::astrabot_teleop::srv::AcquireControlOwner>(config_.acquire_owner_service);
        renew_owner_client_ = create_client<::astrabot_teleop::srv::RenewControlOwner>(config_.renew_owner_service);
        release_owner_client_ =
            create_client<::astrabot_teleop::srv::ReleaseControlOwner>(config_.release_owner_service);
    }
    process_timer_ =
        create_wall_timer(std::chrono::milliseconds(config_.process_period_ms), [this]() { processLatestPacket(); });
    status_timer_ =
        create_wall_timer(std::chrono::milliseconds(config_.status_period_ms), [this]() { publishStatus(); });
    return Status::success();
}

void TeleopNode::resetRosEndpoints() {
    command_publisher_.reset();
    status_publisher_.reset();
    legacy_reference_pose_publisher_.reset();
    legacy_left_gripper_publisher_.reset();
    legacy_right_gripper_publisher_.reset();
}

void TeleopNode::handleAuthorize(const astrabot_rtc::srv::AuthorizeDataChannel::Request::SharedPtr request,
                                 astrabot_rtc::srv::AuthorizeDataChannel::Response::SharedPtr response) {
    response->allowed = false;
    response->reason_code = "grant_rejected";
    response->expires_at = 0;
    if (!running_.load() || config_.backend == "disabled") {
        response->reason_code = "teleop_disabled";
        return;
    }
    if (request->purpose != "teleop" || request->session_id.empty() || request->peer_id.empty() ||
        (!config_.shadowOutputEnabled() && request->run_id.empty()) || request->resource_id != config_.resource_id) {
        response->reason_code = "resource_or_identity_mismatch";
        return;
    }
    const auto contract = DataChannelContracts::find(request->channel_label);
    if (!contract.ok() || request->authorization_token.empty()) {
        response->reason_code = "unsupported_channel";
        return;
    }
    std::lock_guard<std::mutex> authorization_lock(authorization_mutex_);
    if (!running_.load()) {
        response->reason_code = "runtime_stopped";
        return;
    }
    if (config_.productionOutputEnabled()) {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        if (!authorizationRunContextMatchesLocked(request->run_id, request->resource_id)) {
            response->reason_code = "run_context_not_active";
            last_reason_code_ = response->reason_code;
            return;
        }
    }

    const std::uint64_t steady_now_ns = steadyNowNs();
    auto fingerprint = GrantVerifier::tokenFingerprint(request->authorization_token);
    if (!fingerprint.ok()) {
        response->reason_code = reasonCodeForStatus(fingerprint.status());
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        last_reason_code_ = response->reason_code;
        return;
    }

    bool idempotent_retry = false;
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        const auto current = session_registry_.current();
        if (current.has_value()) {
            const SessionBinding retry_binding{request->session_id,    request->peer_id,
                                               request->run_id,        request->resource_id,
                                               request->channel_label, current->authorization_deadline_steady_ns,
                                               fingerprint.value(),    current->connected};
            if (!session_registry_.matchesAuthorization(retry_binding)) {
                response->reason_code = "writer_locked";
                last_reason_code_ = response->reason_code;
                return;
            }
            if (current->authorization_deadline_steady_ns <= steady_now_ns) {
                response->reason_code = "grant_expired";
                last_reason_code_ = response->reason_code;
                return;
            }
            response->allowed = true;
            response->reason_code = "authorized";
            response->expires_at = current->authorization_deadline_steady_ns;
            last_reason_code_ = "authorization_retry";
            idempotent_retry = true;
        }
    }
    if (idempotent_retry) {
        publishStatus();
        return;
    }

    const std::uint64_t system_now_ns = systemNowNs();
    const ExpectedGrantBinding expected{request->session_id, request->run_id, config_.device_id, request->resource_id};
    auto verified = grant_verifier_->verifyAndConsume(request->authorization_token, expected,
                                                      system_now_ns / kNanosecondsPerSecond);
    if (!verified.ok()) {
        response->reason_code = reasonCodeForStatus(verified.status());
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        last_reason_code_ = response->reason_code;
        return;
    }

    const GrantClaims claims = verified.takeValue();
    auto deadline = grantExpiryToSteadyDeadline(claims.expires_at_epoch_sec, system_now_ns, steady_now_ns);
    if (!deadline.ok()) {
        response->reason_code = reasonCodeForStatus(deadline.status());
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        last_reason_code_ = response->reason_code;
        return;
    }
    const SessionBinding binding{request->session_id,    request->peer_id, request->run_id,     request->resource_id,
                                 request->channel_label, deadline.value(), fingerprint.value(), false};
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        if (!running_.load()) {
            response->reason_code = "runtime_stopped";
            last_reason_code_ = response->reason_code;
            return;
        }
        if (config_.productionOutputEnabled() && !authorizationRunContextMatchesLocked(binding)) {
            response->reason_code = "run_context_not_active";
            last_reason_code_ = response->reason_code;
            return;
        }
        if (session_registry_.current().has_value()) {
            response->reason_code = "writer_locked";
            last_reason_code_ = response->reason_code;
            return;
        }
        if (state_machine_.state() == TeleopState::kClosed) {
            const Status reset_status = state_machine_.reset();
            if (!reset_status.ok()) {
                response->reason_code = "state_reset_failed";
                return;
            }
        }
        Status status = session_registry_.authorize(binding, config_.shadowOutputEnabled());
        if (!status.ok()) {
            response->reason_code = "writer_locked";
            last_reason_code_ = response->reason_code;
            return;
        }
        status = state_machine_.transition(TeleopEvent::kAuthorize, "grant_authorized");
        if (!status.ok()) {
            session_registry_.clear();
            response->reason_code = "state_authorize_failed";
            return;
        }
        status_binding_ = session_registry_.current();
        frame_validator_->reset();
        command_limiter_->reset();
        command_mapper_->reset();
        arm_origin_mapper_->resetControlState();
        watchdog_->reset();
        owner_tracker_->invalidate();
        deadman_active_ = false;
        deadman_requested_ = false;
        next_owner_renew_steady_ns_ = 0U;
        last_sequence_ = 0;
        sequence_gap_count_ = 0;
        last_frame_steady_time_ns_ = 0;
        last_reason_code_ = "authorized";
    }
    response->allowed = true;
    response->reason_code = "authorized";
    // 跨包契约：RTC 将 expires_at 与其本机 steady clock 纳秒直接比较，绝不能返回 Unix epoch。
    response->expires_at = deadline.value();
    publishStatus();
}

void TeleopNode::handlePeerEvent(const astrabot_rtc::msg::RtcPeerEvent::ConstSharedPtr message) {
    if (!running_.load()) {
        return;
    }
    bool connected = false;
    bool authorization_expired = false;
    bool binding_mismatch = false;
    bool channel_closed = false;
    bool peer_disconnected = false;
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        const auto binding = session_registry_.current();
        if (!binding.has_value() || binding->session_id != message->session_id ||
            binding->peer_id != message->peer_id) {
            return;
        }
        peer_disconnected = message->state == astrabot_rtc::msg::RtcPeerEvent::STATE_DISCONNECTED ||
                            message->state == astrabot_rtc::msg::RtcPeerEvent::STATE_FAILED ||
                            message->state == astrabot_rtc::msg::RtcPeerEvent::STATE_CLOSED;
        const bool channel_present = std::find(message->data_channels.begin(), message->data_channels.end(),
                                               binding->channel_label) != message->data_channels.end();
        // 一旦 RTC 快照不再包含已授权 channel，就按控制链路关闭处理；具体 transport reason 只用于诊断。
        channel_closed = !channel_present;
        // 停止动作必须在释放 runtime_mutex_ 后执行，避免停止路径重入同一把锁。
        if (!peer_disconnected && !channel_closed &&
            message->state == astrabot_rtc::msg::RtcPeerEvent::STATE_CONNECTED &&
            message->reason_code == "data_channel_open" && channel_present) {
            if (binding->connected) {
                return;
            }
            binding_mismatch = message->purpose != "teleop" || message->run_id != binding->run_id ||
                               message->resource_id != binding->resource_id;
            const std::uint64_t now_ns = steadyNowNs();
            authorization_expired = binding->authorization_deadline_steady_ns <= now_ns;
            if (!binding_mismatch && !authorization_expired) {
                const Status connect_status =
                    session_registry_.markConnected(binding->session_id, binding->peer_id, binding->channel_label);
                const Status state_status =
                    connect_status.ok()
                        ? state_machine_.transition(TeleopEvent::kPeerConnected, "rtc_data_channel_connected")
                        : connect_status;
                if (!connect_status.ok() || !state_status.ok()) {
                    binding_mismatch = true;
                } else {
                    watchdog_->reset();
                    watchdog_->observe(now_ns);
                    status_binding_ = session_registry_.current();
                    last_reason_code_ = "rtc_data_channel_connected";
                    connected = true;
                }
            }
        }
    }
    if (peer_disconnected) {
        stopActiveSession("rtc_peer_disconnected", false);
    } else if (channel_closed) {
        stopActiveSession("rtc_data_channel_closed", false);
    } else if (binding_mismatch) {
        stopActiveSession("rtc_data_channel_binding_mismatch", true, true);
    } else if (authorization_expired) {
        stopActiveSession("grant_expired", true, true);
    } else if (connected) {
        publishStatus();
    }
}

void TeleopNode::handleDataPacket(const astrabot_rtc::msg::RtcDataPacket::ConstSharedPtr message) {
    if (!running_.load()) {
        return;
    }
    if (message->payload.size() > config_.max_frame_bytes ||
        !session_registry_.matches(message->session_id, message->peer_id, message->channel_label)) {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        ++rejected_frame_count_;
        last_reason_code_ = "packet_identity_or_size_rejected";
        return;
    }
    mailbox_.push(*message);
}

void TeleopNode::handleRobotEePose(const geometry_msgs::msg::PoseArray::ConstSharedPtr message) {
    if (!running_.load() || !arm_origin_mapper_) {
        return;
    }
    if (message->header.frame_id != config_.robot_base_frame || message->poses.size() < 2U) {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        last_reason_code_ = "robot_pose_contract_rejected";
        return;
    }

    const PoseSample left_arm = poseSample(message->poses[0]);
    const PoseSample right_arm = poseSample(message->poses[1]);
    const std::uint64_t receive_steady_time_ns = steadyNowNs();
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    const Status status = arm_origin_mapper_->updateRobotPose(right_arm, left_arm, receive_steady_time_ns);
    if (!status.ok()) {
        last_reason_code_ = "robot_pose_invalid";
        return;
    }
    robot_ee_pose_ = RobotEePoseSnapshot{right_arm, left_arm, receive_steady_time_ns, true};
}

void TeleopNode::handleRunContext(
    const astrabot_data_interfaces::msg::DataCollectionRunContext::ConstSharedPtr message) {
    if (!running_.load() || !config_.productionOutputEnabled()) {
        return;
    }
    std::lock_guard<std::mutex> transition_lock(control_transition_mutex_);

    bool stop_session = false;
    bool pause_control = false;
    bool fault = false;
    bool accepted_context = false;
    std::string reason_code;
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        const bool generation_envelope_valid =
            message->context_version != 0U && safePublisherGeneration(message->publisher_generation) &&
            !message->resource_id.empty() &&
            message->state <= astrabot_data_interfaces::msg::DataCollectionRunContext::STATE_FAILED;
        const RunContextGenerationObservation generation_observation =
            generation_envelope_valid ? run_context_generation_tracker_.observe(message->publisher_generation)
                                      : RunContextGenerationObservation::kInvalid;
        const bool run_identity_valid =
            !message->run_id.empty() ||
            message->state == astrabot_data_interfaces::msg::DataCollectionRunContext::STATE_IDLE;
        if (!generation_envelope_valid || generation_observation == RunContextGenerationObservation::kInvalid) {
            stop_session = session_registry_.current().has_value();
            fault = true;
            reason_code = "invalid_run_context";
            run_context_.reset();
            last_reason_code_ = reason_code;
        } else if (generation_observation == RunContextGenerationObservation::kRetired) {
            stop_session = session_registry_.current().has_value();
            fault = true;
            reason_code = "run_context_publisher_retired";
            run_context_.reset();
            last_reason_code_ = reason_code;
        } else if (!run_identity_valid) {
            stop_session = session_registry_.current().has_value();
            fault = true;
            reason_code = "invalid_run_context";
            run_context_.reset();
            last_reason_code_ = reason_code;
        } else if (generation_observation == RunContextGenerationObservation::kInitial) {
            run_context_version_high_watermark_ = message->context_version;
            run_context_ = *message;
            accepted_context = true;
        } else if (generation_observation == RunContextGenerationObservation::kChanged) {
            const auto binding = session_registry_.current();
            const bool same_authorized_run = binding.has_value() && runContextAllowsAuthorization(message->state) &&
                                             message->run_id == binding->run_id &&
                                             message->resource_id == binding->resource_id;
            stop_session = binding.has_value() && !same_authorized_run;
            if (stop_session) {
                reason_code = "run_context_publisher_changed";
            }
            run_context_version_high_watermark_ = message->context_version;
            run_context_ = *message;
            accepted_context = true;
            if (!reason_code.empty()) {
                last_reason_code_ = reason_code;
            }
        } else if (message->context_version < run_context_version_high_watermark_) {
            stop_session = session_registry_.current().has_value();
            fault = true;
            reason_code = "run_context_version_regressed";
            run_context_.reset();
            last_reason_code_ = reason_code;
        } else if (message->context_version == run_context_version_high_watermark_) {
            if (!run_context_.has_value()) {
                stop_session = session_registry_.current().has_value();
                fault = true;
                reason_code = "run_context_recovery_requires_new_version";
                last_reason_code_ = reason_code;
            } else if (!sameRunContext(*run_context_, *message)) {
                stop_session = session_registry_.current().has_value();
                fault = true;
                reason_code = "conflicting_run_context";
                run_context_.reset();
                last_reason_code_ = reason_code;
            }
        } else {
            run_context_version_high_watermark_ = message->context_version;
            run_context_ = *message;
            accepted_context = true;
        }
        if (accepted_context && !stop_session) {
            const auto binding = session_registry_.current();
            if (binding.has_value()) {
                if (!runContextAllowsAuthorization(message->state) || message->run_id != binding->run_id ||
                    message->resource_id != binding->resource_id) {
                    stop_session = true;
                    reason_code = "run_context_ended";
                } else if (message->state == astrabot_data_interfaces::msg::DataCollectionRunContext::STATE_READY) {
                    pause_control = true;
                    reason_code = "run_context_ready";
                }
            }
        }
    }
    if (stop_session) {
        stopActiveSession(reason_code, true, fault);
    } else if (pause_control) {
        pauseControlForReady(reason_code);
    } else if (fault) {
        publishStatus();
    }
}

void TeleopNode::processLatestPacket() {
    if (!running_.load()) {
        return;
    }
    auto packet = mailbox_.take();
    if (packet.has_value()) {
        processPacket(*packet);
    }
    const std::uint64_t now_ns = steadyNowNs();
    checkAuthorizationDeadline(now_ns);
    checkOwnerLease(now_ns);
    checkWatchdog(now_ns);
    expirePendingOwnerRequest(now_ns);
    expirePendingOwnerCleanupRequests(now_ns);
    expirePendingCloseRequests(now_ns);
}

void TeleopNode::processPacket(const astrabot_rtc::msg::RtcDataPacket &packet) {
    const std::uint64_t now_ns = steadyNowNs();
    const auto authorized_binding = session_registry_.current();
    if (authorized_binding.has_value() && authorized_binding->session_id == packet.session_id &&
        authorized_binding->peer_id == packet.peer_id && authorized_binding->channel_label == packet.channel_label &&
        authorized_binding->authorization_deadline_steady_ns <= now_ns) {
        stopActiveSession("grant_expired", true, true);
        return;
    }
    PacketAction action = PacketAction::kNone;
    std::uint64_t owner_epoch = 0U;
    std::optional<SessionBinding> binding;
    std::optional<DecodedTeleopFrame> decoded;
    std::optional<MappedCommand> mapped;
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        if (!session_registry_.matches(packet.session_id, packet.peer_id, packet.channel_label)) {
            ++rejected_frame_count_;
            last_reason_code_ = "packet_session_mismatch";
            return;
        }
        if (packet.receive_steady_time_ns == 0U || packet.receive_steady_time_ns > now_ns ||
            now_ns - packet.receive_steady_time_ns >=
                static_cast<std::uint64_t>(config_.command_ttl_ms) * kNanosecondsPerMillisecond) {
            ++rejected_frame_count_;
            last_reason_code_ = "rtc_to_teleop_hop_stale";
            return;
        }
        if (config_.productionOutputEnabled()) {
            const auto control_binding = session_registry_.current();
            if (!control_binding.has_value() || !activeRunContextMatchesLocked(*control_binding)) {
                ++rejected_frame_count_;
                last_reason_code_ = "run_context_not_active";
                return;
            }
        }
        auto decode_result = codec_->decode(packet.payload);
        if (!decode_result.ok()) {
            ++rejected_frame_count_;
            last_reason_code_ = reasonCodeForStatus(decode_result.status());
            return;
        }
        decoded = decode_result.takeValue();
        auto validation = frame_validator_->validate(*decoded, systemNowMs());
        if (!validation.ok()) {
            ++rejected_frame_count_;
            last_reason_code_ = reasonCodeForStatus(validation.status());
            return;
        }
        CommandMapper command_mapper_candidate = *command_mapper_;
        ArmOriginMapper arm_origin_mapper_candidate = *arm_origin_mapper_;
        CommandLimiter command_limiter_candidate = *command_limiter_;
        const MappedCommand relative_command = command_mapper_candidate.map(*decoded);
        auto mapping_result = arm_origin_mapper_candidate.map(relative_command, now_ns);
        if (!mapping_result.ok()) {
            ++rejected_frame_count_;
            last_reason_code_ = reasonCodeForStatus(mapping_result.status());
            return;
        }
        MappedCommand mapped_command = mapping_result.takeValue();
        command_limiter_candidate.resetArmHistories(mapped_command.right_origin_captured,
                                                    mapped_command.left_origin_captured);

        DecodedTeleopFrame mapped_frame = *decoded;
        mapped_frame.pose_valid_right = mapped_command.right_arm_valid;
        mapped_frame.pose_valid_left = mapped_command.left_arm_valid;
        mapped_frame.pose_valid_head = mapped_command.head_valid;
        mapped_frame.action_right = mapped_command.right_arm_target;
        mapped_frame.action_left = mapped_command.left_arm_target;
        mapped_frame.action_head = mapped_command.head_target;
        const Status limit_status =
            command_limiter_candidate.validateAndCommit(mapped_frame, packet.receive_steady_time_ns);
        if (!limit_status.ok()) {
            ++rejected_frame_count_;
            last_reason_code_ = reasonCodeForStatus(limit_status);
            return;
        }
        *command_mapper_ = std::move(command_mapper_candidate);
        *arm_origin_mapper_ = std::move(arm_origin_mapper_candidate);
        *command_limiter_ = std::move(command_limiter_candidate);
        mapped = std::move(mapped_command);
        binding = session_registry_.current();
        if (!binding.has_value()) {
            ++rejected_frame_count_;
            last_reason_code_ = "packet_session_disappeared";
            return;
        }
        if (config_.productionOutputEnabled() && !activeRunContextMatchesLocked(*binding)) {
            ++rejected_frame_count_;
            last_reason_code_ = "run_context_not_active";
            return;
        }

        if (config_.shadowOutputEnabled() || config_.legacyMpcOutputEnabled()) {
            const bool was_deadman_active = deadman_active_;
            if (!state_machine_.transition(TeleopEvent::kFrameValid, "valid_frame").ok()) {
                ++rejected_frame_count_;
                last_reason_code_ = "frame_in_invalid_state";
                return;
            }
            const TeleopEvent deadman_event =
                mapped->deadman ? TeleopEvent::kDeadmanPressed : TeleopEvent::kDeadmanReleased;
            if (!state_machine_.transition(deadman_event, mapped->deadman ? "deadman_pressed" : "deadman_released")
                     .ok()) {
                ++rejected_frame_count_;
                last_reason_code_ = "deadman_transition_failed";
                return;
            }
            deadman_active_ = mapped->deadman;
            deadman_requested_ = mapped->deadman;
            if (config_.legacyMpcOutputEnabled()) {
                last_reason_code_ = mapped->deadman ? "controlling_legacy_mpc" : "armed_legacy_mpc";
                action = mapped->deadman || was_deadman_active ? PacketAction::kPublish : PacketAction::kNone;
            } else {
                last_reason_code_ = mapped->deadman ? "controlling_shadow" : "armed_shadow";
                action = PacketAction::kPublish;
            }
        } else if (config_.productionOutputEnabled()) {
            deadman_requested_ = mapped->deadman;
            const auto lease = owner_tracker_->activeLease(now_ns);
            const auto pending = owner_tracker_->pendingOperation();
            const bool cleanup_pending = ownerCleanupPending();
            if (mapped->deadman && lease.has_value()) {
                if (!state_machine_.transition(TeleopEvent::kFrameValid, "valid_frame").ok() ||
                    !state_machine_.transition(TeleopEvent::kDeadmanPressed, "deadman_pressed").ok()) {
                    ++rejected_frame_count_;
                    last_reason_code_ = "owner_frame_state_failed";
                    return;
                }
                deadman_active_ = true;
                owner_epoch = lease->owner_epoch;
                last_reason_code_ = "controlling_cpp";
                action = PacketAction::kPublish;
            } else if (mapped->deadman && !pending.has_value() && !cleanup_pending) {
                deadman_active_ = false;
                last_reason_code_ = "owner_acquire_pending";
                action = PacketAction::kAcquireOwner;
            } else if (!mapped->deadman && lease.has_value()) {
                const TeleopState state = state_machine_.state();
                if (state != TeleopState::kConnected &&
                    (!state_machine_.transition(TeleopEvent::kFrameValid, "valid_frame").ok() ||
                     !state_machine_.transition(TeleopEvent::kOwnerReleased, "deadman_released").ok())) {
                    ++rejected_frame_count_;
                    last_reason_code_ = "owner_release_state_failed";
                    return;
                }
                deadman_active_ = false;
                owner_epoch = lease->owner_epoch;
                last_reason_code_ = "owner_release_pending";
                action = PacketAction::kReleaseOwner;
            } else {
                deadman_active_ = false;
                last_reason_code_ = cleanup_pending       ? "owner_cleanup_pending"
                                    : pending.has_value() ? "owner_request_pending"
                                                          : "connected_no_owner";
            }
        }

        frame_validator_->commit(*decoded, validation.value());
        watchdog_->observe(now_ns);
        last_sequence_ = frame_validator_->lastSequence();
        sequence_gap_count_ = frame_validator_->sequenceGapCount();
        last_frame_steady_time_ns_ = now_ns;
    }
    if (!binding.has_value()) {
        return;
    }
    if (action == PacketAction::kPublish) {
        publishCommand(*binding, *decoded, *mapped, packet.receive_steady_time_ns, owner_epoch);
    } else if (action == PacketAction::kAcquireOwner) {
        requestOwnerAcquire(*binding);
    } else if (action == PacketAction::kReleaseOwner) {
        publishStopCommand(*binding, now_ns, owner_epoch);
        requestOwnerRelease("deadman_released", true);
    }
}

void TeleopNode::checkAuthorizationDeadline(const std::uint64_t steady_now_ns) {
    const auto binding = session_registry_.current();
    if (binding.has_value() && binding->authorization_deadline_steady_ns <= steady_now_ns) {
        stopActiveSession("grant_expired", true, true);
    }
}

void TeleopNode::checkWatchdog(const std::uint64_t steady_now_ns) {
    bool expired = false;
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        const TeleopState state = state_machine_.state();
        const auto binding = session_registry_.current();
        const bool active_control_context =
            !config_.productionOutputEnabled() || (binding.has_value() && activeRunContextMatchesLocked(*binding));
        expired =
            active_control_context &&
            (state == TeleopState::kConnected || state == TeleopState::kArmed || state == TeleopState::kControlling) &&
            watchdog_->expired(steady_now_ns);
        if (expired) {
            ++watchdog_stop_count_;
        }
    }
    if (expired) {
        stopActiveSession("motion_watchdog_timeout", true);
    }
}

void TeleopNode::checkOwnerLease(const std::uint64_t steady_now_ns) {
    if (!config_.productionOutputEnabled() || !owner_tracker_) {
        return;
    }
    const auto lease = owner_tracker_->leaseSnapshot();
    if (lease.has_value() && lease->expires_at_steady_ns <= steady_now_ns) {
        owner_tracker_->invalidate();
        handleOwnerFailure("owner_lease_expired");
        return;
    }
    if (!lease.has_value() || owner_tracker_->pendingOperation().has_value()) {
        return;
    }

    bool renew_due = false;
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        renew_due =
            deadman_requested_ && next_owner_renew_steady_ns_ != 0U && steady_now_ns >= next_owner_renew_steady_ns_;
    }
    if (renew_due) {
        requestOwnerRenew(*lease);
    }
}

void TeleopNode::requestOwnerAcquire(const SessionBinding &binding) {
    std::unique_lock<std::mutex> transition_lock(control_transition_mutex_);
    bool run_context_matches = false;
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        run_context_matches = activeRunContextMatchesLocked(binding);
        last_reason_code_ = run_context_matches ? "owner_acquire_starting" : "run_context_not_active";
    }
    if (!run_context_matches) {
        transition_lock.unlock();
        handleOwnerFailure("run_context_not_active");
        return;
    }
    if (ownerCleanupPending()) {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        last_reason_code_ = "owner_cleanup_pending";
        return;
    }
    if (!owner_tracker_ || !acquire_owner_client_ || !acquire_owner_client_->service_is_ready()) {
        transition_lock.unlock();
        handleOwnerFailure("owner_acquire_service_unavailable");
        return;
    }
    const std::uint64_t now_ns = steadyNowNs();
    const std::uint64_t timeout_ns =
        static_cast<std::uint64_t>(config_.owner_service_timeout_ms) * kNanosecondsPerMillisecond;
    auto operation = owner_tracker_->beginAcquire(ownerIdentity(binding), now_ns, timeout_ns);
    if (!operation.ok()) {
        if (operation.status().code() != ErrorCode::kConflict) {
            transition_lock.unlock();
            handleOwnerFailure("owner_acquire_state_failed");
        }
        return;
    }
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        const auto current = session_registry_.current();
        if (!current.has_value() || !activeRunContextMatchesLocked(*current) ||
            current->session_id != binding.session_id || current->peer_id != binding.peer_id ||
            current->run_id != binding.run_id || current->resource_id != binding.resource_id) {
            static_cast<void>(owner_tracker_->cancelPending());
            last_reason_code_ = "owner_acquire_canceled_before_send";
            return;
        }
    }

    auto request = std::make_shared<::astrabot_teleop::srv::AcquireControlOwner::Request>();
    request->source_type = "teleop";
    request->session_id = binding.session_id;
    request->run_id = binding.run_id;
    request->resource_id = binding.resource_id;
    request->source_id = binding.peer_id;
    request->requested_ttl_ms = static_cast<std::uint64_t>(config_.owner_ttl_ms);

    const auto shared_node = weak_from_this().lock();
    if (!shared_node) {
        owner_tracker_->invalidate();
        return;
    }
    const std::weak_ptr<TeleopNode> weak_self = std::static_pointer_cast<TeleopNode>(shared_node);
    const ControlOwnerOperation operation_value = operation.value();
    auto future_and_id = acquire_owner_client_->async_send_request(
        request,
        [weak_self, operation_value](rclcpp::Client<::astrabot_teleop::srv::AcquireControlOwner>::SharedFuture future) {
            if (const auto self = weak_self.lock()) {
                self->handleOwnerAcquireResponse(operation_value, future);
            }
        });
    if (!registerPendingOwnerRequest(future_and_id.request_id, operation_value, true)) {
        transition_lock.unlock();
        handleOwnerFailure("owner_pending_request_limit");
    }
}

void TeleopNode::requestOwnerRenew(const ControlOwnerLease &lease) {
    std::unique_lock<std::mutex> transition_lock(control_transition_mutex_);
    bool binding_matches = false;
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        const auto binding = session_registry_.current();
        if (binding.has_value() && activeRunContextMatchesLocked(*binding)) {
            const ControlOwnerIdentity identity = ownerIdentity(*binding);
            binding_matches =
                identity.session_id == lease.identity.session_id && identity.run_id == lease.identity.run_id &&
                identity.resource_id == lease.identity.resource_id && identity.source_id == lease.identity.source_id;
        }
        last_reason_code_ = binding_matches ? "owner_renew_starting" : "owner_renew_binding_mismatch";
    }
    if (!binding_matches) {
        transition_lock.unlock();
        handleOwnerFailure("owner_renew_binding_mismatch");
        return;
    }
    if (ownerCleanupPending()) {
        transition_lock.unlock();
        handleOwnerFailure("owner_cleanup_pending_during_renew");
        return;
    }
    if (!owner_tracker_ || !renew_owner_client_ || !renew_owner_client_->service_is_ready()) {
        transition_lock.unlock();
        handleOwnerFailure("owner_renew_service_unavailable");
        return;
    }
    const std::uint64_t now_ns = steadyNowNs();
    const std::uint64_t timeout_ns =
        static_cast<std::uint64_t>(config_.owner_service_timeout_ms) * kNanosecondsPerMillisecond;
    auto operation = owner_tracker_->beginRenew(now_ns, timeout_ns);
    if (!operation.ok()) {
        transition_lock.unlock();
        handleOwnerFailure("owner_renew_state_failed");
        return;
    }
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        const auto current = session_registry_.current();
        if (!current.has_value() || !activeRunContextMatchesLocked(*current) ||
            ownerIdentity(*current).session_id != lease.identity.session_id ||
            ownerIdentity(*current).run_id != lease.identity.run_id ||
            ownerIdentity(*current).resource_id != lease.identity.resource_id ||
            ownerIdentity(*current).source_id != lease.identity.source_id) {
            static_cast<void>(owner_tracker_->cancelPending());
            last_reason_code_ = "owner_renew_canceled_before_send";
            return;
        }
    }

    auto request = std::make_shared<::astrabot_teleop::srv::RenewControlOwner::Request>();
    request->source_type = "teleop";
    request->session_id = lease.identity.session_id;
    request->run_id = lease.identity.run_id;
    request->resource_id = lease.identity.resource_id;
    request->source_id = lease.identity.source_id;
    request->owner_epoch = lease.owner_epoch;
    request->requested_ttl_ms = static_cast<std::uint64_t>(config_.owner_ttl_ms);

    const auto shared_node = weak_from_this().lock();
    if (!shared_node) {
        owner_tracker_->invalidate();
        return;
    }
    const std::weak_ptr<TeleopNode> weak_self = std::static_pointer_cast<TeleopNode>(shared_node);
    const ControlOwnerOperation operation_value = operation.value();
    auto future_and_id = renew_owner_client_->async_send_request(
        request,
        [weak_self, operation_value](rclcpp::Client<::astrabot_teleop::srv::RenewControlOwner>::SharedFuture future) {
            if (const auto self = weak_self.lock()) {
                self->handleOwnerRenewResponse(operation_value, future);
            }
        });
    if (!registerPendingOwnerRequest(future_and_id.request_id, operation_value, true)) {
        transition_lock.unlock();
        handleOwnerFailure("owner_pending_request_limit");
    }
}

void TeleopNode::requestOwnerRelease(const std::string &reason_code, const bool fail_session_on_error) {
    if (!owner_tracker_) {
        return;
    }
    const auto existing_operation = owner_tracker_->pendingOperation();
    if (existing_operation.has_value() && existing_operation->kind == ControlOwnerOperationKind::kRelease) {
        return;
    }
    quarantinePendingOwnerRequestForCleanup();
    const auto lease = owner_tracker_->leaseSnapshot();
    if (!lease.has_value()) {
        return;
    }

    const std::uint64_t now_ns = steadyNowNs();
    const std::uint64_t timeout_ns =
        static_cast<std::uint64_t>(config_.owner_service_timeout_ms) * kNanosecondsPerMillisecond;
    auto operation = owner_tracker_->beginRelease(now_ns, timeout_ns);
    if (!operation.ok()) {
        owner_tracker_->invalidate();
        if (fail_session_on_error) {
            handleOwnerFailure("owner_release_state_failed");
        }
        return;
    }
    if (!release_owner_client_ || !release_owner_client_->service_is_ready()) {
        owner_tracker_->invalidate();
        if (fail_session_on_error) {
            handleOwnerFailure("owner_release_service_unavailable");
        }
        return;
    }

    auto request = std::make_shared<::astrabot_teleop::srv::ReleaseControlOwner::Request>();
    request->source_type = "teleop";
    request->session_id = lease->identity.session_id;
    request->run_id = lease->identity.run_id;
    request->resource_id = lease->identity.resource_id;
    request->source_id = lease->identity.source_id;
    request->owner_epoch = lease->owner_epoch;
    request->reason_code = reason_code;

    const auto shared_node = weak_from_this().lock();
    if (!shared_node) {
        owner_tracker_->invalidate();
        return;
    }
    const std::weak_ptr<TeleopNode> weak_self = std::static_pointer_cast<TeleopNode>(shared_node);
    const ControlOwnerOperation operation_value = operation.value();
    auto future_and_id = release_owner_client_->async_send_request(
        request, [weak_self, operation_value, fail_session_on_error](
                     rclcpp::Client<::astrabot_teleop::srv::ReleaseControlOwner>::SharedFuture future) {
            if (const auto self = weak_self.lock()) {
                self->handleOwnerReleaseResponse(operation_value, fail_session_on_error, future);
            }
        });
    if (!registerPendingOwnerRequest(future_and_id.request_id, operation_value, fail_session_on_error) &&
        fail_session_on_error) {
        handleOwnerFailure("owner_pending_request_limit");
    }
}

void TeleopNode::requestOwnerCleanupRelease(const ControlOwnerIdentity &identity, const std::uint64_t owner_epoch,
                                            const std::string &reason_code) {
    if (owner_epoch == 0U) {
        return;
    }
    const std::string route_key = ownerCleanupRouteKey(identity, owner_epoch);
    const std::uint64_t now_ns = steadyNowNs();
    const std::uint64_t grace_ns = static_cast<std::uint64_t>(config_.owner_ttl_ms + config_.owner_service_timeout_ms) *
                                   kNanosecondsPerMillisecond;
    const std::uint64_t deadline = now_ns > std::numeric_limits<std::uint64_t>::max() - grace_ns
                                       ? std::numeric_limits<std::uint64_t>::max()
                                       : now_ns + grace_ns;
    {
        std::lock_guard<std::mutex> lock(owner_pending_mutex_);
        if (pending_cleanup_release_requests_.find(route_key) != pending_cleanup_release_requests_.end() ||
            pending_cleanup_release_requests_.size() >= kMaxCleanupReleaseRequests) {
            return;
        }
        pending_cleanup_release_requests_.emplace(route_key, PendingCleanupReleaseRequest{-1, deadline});
    }
    if (!release_owner_client_ || !release_owner_client_->service_is_ready()) {
        return;
    }

    auto request = std::make_shared<::astrabot_teleop::srv::ReleaseControlOwner::Request>();
    request->source_type = "teleop";
    request->session_id = identity.session_id;
    request->run_id = identity.run_id;
    request->resource_id = identity.resource_id;
    request->source_id = identity.source_id;
    request->owner_epoch = owner_epoch;
    request->reason_code = reason_code;

    const auto shared_node = weak_from_this().lock();
    if (!shared_node) {
        std::lock_guard<std::mutex> lock(owner_pending_mutex_);
        pending_cleanup_release_requests_.erase(route_key);
        return;
    }
    const std::weak_ptr<TeleopNode> weak_self = std::static_pointer_cast<TeleopNode>(shared_node);
    auto future_and_id = release_owner_client_->async_send_request(
        request,
        [weak_self, route_key](rclcpp::Client<::astrabot_teleop::srv::ReleaseControlOwner>::SharedFuture future) {
            if (const auto self = weak_self.lock()) {
                self->handleOwnerCleanupReleaseResponse(route_key, future);
            }
        });
    bool callback_completed = false;
    {
        std::lock_guard<std::mutex> lock(owner_pending_mutex_);
        const auto iterator = pending_cleanup_release_requests_.find(route_key);
        if (iterator == pending_cleanup_release_requests_.end() || iterator->second.request_id != -1) {
            callback_completed = true;
        } else {
            iterator->second.request_id = future_and_id.request_id;
        }
    }
    if (callback_completed) {
        release_owner_client_->remove_pending_request(future_and_id.request_id);
    }
}

void TeleopNode::handleOwnerAcquireResponse(
    const ControlOwnerOperation &operation,
    rclcpp::Client<::astrabot_teleop::srv::AcquireControlOwner>::SharedFuture future) {
    clearPendingOwnerRequest(operation);
    if (!owner_tracker_) {
        return;
    }
    const auto response = future.get();
    if (!running_.load()) {
        if (response && response->granted) {
            requestOwnerCleanupRelease(operation.identity, response->owner_epoch, "stale_acquire_after_stop");
        }
        return;
    }
    const std::uint64_t now_ns = steadyNowNs();
    const Status status =
        owner_tracker_->completeAcquire(operation, response && response->granted, response ? response->owner_epoch : 0U,
                                        response ? response->expires_at_steady_ns : 0U, now_ns);
    if (!status.ok()) {
        if (status.code() == ErrorCode::kFailedPrecondition) {
            if (response && response->granted) {
                requestOwnerCleanupRelease(operation.identity, response->owner_epoch, "stale_acquire_callback");
            }
        } else {
            handleOwnerFailure(response ? "owner_acquire_rejected" : "owner_acquire_response_missing");
        }
        return;
    }
    const std::uint64_t renew_period_ns =
        static_cast<std::uint64_t>(config_.owner_renew_period_ms) * kNanosecondsPerMillisecond;
    const std::uint64_t service_timeout_ns =
        static_cast<std::uint64_t>(config_.owner_service_timeout_ms) * kNanosecondsPerMillisecond;
    const auto renew_deadline =
        ownerRenewDeadline(now_ns, response->expires_at_steady_ns, renew_period_ns, service_timeout_ns);
    if (!renew_deadline.ok()) {
        requestOwnerRelease("owner_lease_margin_invalid", false);
        handleOwnerFailure("owner_lease_margin_invalid");
        return;
    }

    bool release_immediately = false;
    bool fault = false;
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        const auto binding = session_registry_.current();
        if (!binding.has_value() || !activeRunContextMatchesLocked(*binding) ||
            binding->session_id != operation.identity.session_id || binding->run_id != operation.identity.run_id ||
            binding->resource_id != operation.identity.resource_id ||
            binding->peer_id != operation.identity.source_id) {
            release_immediately = true;
        } else if (!deadman_requested_) {
            release_immediately = true;
        } else if (!state_machine_.transition(TeleopEvent::kOwnerAcquired, "owner_acquired").ok() ||
                   !state_machine_.transition(TeleopEvent::kDeadmanPressed, "deadman_pressed").ok()) {
            release_immediately = true;
            fault = true;
        } else {
            deadman_active_ = true;
            next_owner_renew_steady_ns_ = renew_deadline.value();
            last_reason_code_ = "owner_acquired";
        }
    }
    if (release_immediately) {
        requestOwnerRelease(fault ? "owner_state_failed" : "deadman_released_before_acquire", false);
        if (fault) {
            handleOwnerFailure("owner_state_failed");
        }
        return;
    }
    publishStatus();
}

void TeleopNode::handleOwnerRenewResponse(
    const ControlOwnerOperation &operation,
    rclcpp::Client<::astrabot_teleop::srv::RenewControlOwner>::SharedFuture future) {
    clearPendingOwnerRequest(operation);
    if (!owner_tracker_) {
        return;
    }
    const auto response = future.get();
    if (!running_.load()) {
        requestOwnerCleanupRelease(operation.identity, operation.owner_epoch, "stale_renew_after_stop");
        return;
    }
    const std::uint64_t now_ns = steadyNowNs();
    const Status status =
        owner_tracker_->completeRenew(operation, response && response->renewed, response ? response->owner_epoch : 0U,
                                      response ? response->expires_at_steady_ns : 0U, now_ns);
    if (!status.ok()) {
        if (status.code() == ErrorCode::kFailedPrecondition) {
            requestOwnerCleanupRelease(operation.identity, operation.owner_epoch, "stale_renew_callback");
        } else {
            requestOwnerCleanupRelease(operation.identity, operation.owner_epoch, "renew_failed_cleanup");
            handleOwnerFailure("owner_renew_failed");
        }
        return;
    }
    const std::uint64_t renew_period_ns =
        static_cast<std::uint64_t>(config_.owner_renew_period_ms) * kNanosecondsPerMillisecond;
    const std::uint64_t service_timeout_ns =
        static_cast<std::uint64_t>(config_.owner_service_timeout_ms) * kNanosecondsPerMillisecond;
    const auto renew_deadline =
        ownerRenewDeadline(now_ns, response->expires_at_steady_ns, renew_period_ns, service_timeout_ns);
    if (!renew_deadline.ok()) {
        requestOwnerRelease("owner_lease_margin_invalid", false);
        handleOwnerFailure("owner_lease_margin_invalid");
        return;
    }
    bool renewed_binding_active = false;
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        const auto binding = session_registry_.current();
        if (binding.has_value() && activeRunContextMatchesLocked(*binding)) {
            const ControlOwnerIdentity identity = ownerIdentity(*binding);
            renewed_binding_active = identity.session_id == operation.identity.session_id &&
                                     identity.run_id == operation.identity.run_id &&
                                     identity.resource_id == operation.identity.resource_id &&
                                     identity.source_id == operation.identity.source_id && deadman_requested_;
        }
        if (renewed_binding_active) {
            next_owner_renew_steady_ns_ = renew_deadline.value();
            last_reason_code_ = "owner_renewed";
        }
    }
    if (!renewed_binding_active) {
        requestOwnerRelease("renew_completed_after_control_pause", false);
    }
}

void TeleopNode::handleOwnerReleaseResponse(
    const ControlOwnerOperation &operation, const bool fail_session_on_error,
    rclcpp::Client<::astrabot_teleop::srv::ReleaseControlOwner>::SharedFuture future) {
    clearPendingOwnerRequest(operation);
    if (!owner_tracker_) {
        return;
    }
    const auto response = future.get();
    const Status status = owner_tracker_->completeRelease(operation, response && response->released);
    if (!status.ok()) {
        if (status.code() != ErrorCode::kFailedPrecondition && fail_session_on_error && running_.load()) {
            handleOwnerFailure("owner_release_failed");
        }
        return;
    }
    if (running_.load()) {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        last_reason_code_ = "owner_released";
    }
}

void TeleopNode::handleOwnerCleanupReleaseResponse(
    const std::string &route_key, rclcpp::Client<::astrabot_teleop::srv::ReleaseControlOwner>::SharedFuture future) {
    const auto response = future.get();
    const bool released = response && response->released;
    {
        std::lock_guard<std::mutex> lock(owner_pending_mutex_);
        const auto iterator = pending_cleanup_release_requests_.find(route_key);
        if (iterator == pending_cleanup_release_requests_.end()) {
            return;
        }
        if (released) {
            pending_cleanup_release_requests_.erase(iterator);
        } else {
            iterator->second.request_id = 0;
        }
    }
    if (!released) {
        RCLCPP_WARN(get_logger(), "stale owner cleanup release was not confirmed");
    }
}

bool TeleopNode::registerPendingOwnerRequest(const std::int64_t request_id, const ControlOwnerOperation &operation,
                                             const bool fail_session_on_error) {
    bool conflict = false;
    bool already_completed = false;
    {
        std::lock_guard<std::mutex> lock(owner_pending_mutex_);
        const auto pending = owner_tracker_ ? owner_tracker_->pendingOperation() : std::nullopt;
        already_completed = !pending.has_value() || pending->operation_id != operation.operation_id ||
                            pending->generation != operation.generation;
        if (!already_completed &&
            (pending_owner_request_.has_value() ||
             (pending_owner_cleanup_request_.has_value() && operation.kind != ControlOwnerOperationKind::kRelease))) {
            conflict = true;
        } else if (!already_completed) {
            pending_owner_request_ = PendingOwnerRequest{request_id, operation, fail_session_on_error};
        }
    }
    if (already_completed || conflict) {
        removeOwnerClientPendingRequest(PendingOwnerRequest{request_id, operation, fail_session_on_error});
    }
    if (conflict) {
        owner_tracker_->invalidate();
    }
    return !conflict;
}

void TeleopNode::clearPendingOwnerRequest(const ControlOwnerOperation &operation) {
    std::lock_guard<std::mutex> lock(owner_pending_mutex_);
    if (pending_owner_request_.has_value() &&
        pending_owner_request_->operation.operation_id == operation.operation_id &&
        pending_owner_request_->operation.generation == operation.generation) {
        pending_owner_request_.reset();
    }
    if (pending_owner_cleanup_request_.has_value() &&
        pending_owner_cleanup_request_->request.operation.operation_id == operation.operation_id &&
        pending_owner_cleanup_request_->request.operation.generation == operation.generation) {
        pending_owner_cleanup_request_.reset();
    }
}

void TeleopNode::cancelPendingOwnerRequest() {
    std::optional<PendingOwnerRequest> pending;
    std::optional<PendingOwnerCleanupRequest> cleanup;
    std::vector<std::int64_t> cleanup_release_ids;
    {
        std::lock_guard<std::mutex> lock(owner_pending_mutex_);
        pending = pending_owner_request_;
        pending_owner_request_.reset();
        cleanup = pending_owner_cleanup_request_;
        pending_owner_cleanup_request_.reset();
        cleanup_release_ids.reserve(pending_cleanup_release_requests_.size());
        for (const auto &entry : pending_cleanup_release_requests_) {
            if (entry.second.request_id > 0) {
                cleanup_release_ids.push_back(entry.second.request_id);
            }
        }
        pending_cleanup_release_requests_.clear();
    }
    if (owner_tracker_) {
        static_cast<void>(owner_tracker_->cancelPending());
    }
    if (pending.has_value()) {
        removeOwnerClientPendingRequest(*pending);
    }
    if (cleanup.has_value()) {
        removeOwnerClientPendingRequest(cleanup->request);
    }
    if (release_owner_client_) {
        for (const auto request_id : cleanup_release_ids) {
            release_owner_client_->remove_pending_request(request_id);
        }
    }
}

void TeleopNode::quarantinePendingOwnerRequestForCleanup() {
    std::optional<PendingOwnerRequest> pending;
    bool stored_as_cleanup = false;
    {
        std::lock_guard<std::mutex> lock(owner_pending_mutex_);
        if (!pending_owner_request_.has_value() ||
            pending_owner_request_->operation.kind == ControlOwnerOperationKind::kRelease) {
            return;
        }
        pending = pending_owner_request_;
        pending_owner_request_.reset();
        const std::uint64_t now_ns = steadyNowNs();
        const std::uint64_t grace_ns =
            static_cast<std::uint64_t>(config_.owner_ttl_ms + config_.owner_service_timeout_ms) *
            kNanosecondsPerMillisecond;
        const std::uint64_t deadline = now_ns > std::numeric_limits<std::uint64_t>::max() - grace_ns
                                           ? std::numeric_limits<std::uint64_t>::max()
                                           : now_ns + grace_ns;
        if (!pending_owner_cleanup_request_.has_value()) {
            pending_owner_cleanup_request_ = PendingOwnerCleanupRequest{*pending, deadline};
            stored_as_cleanup = true;
        }
    }
    if (owner_tracker_) {
        static_cast<void>(owner_tracker_->cancelPending());
    }
    if (pending.has_value() && !stored_as_cleanup) {
        removeOwnerClientPendingRequest(*pending);
    }
}

bool TeleopNode::ownerCleanupPending() const {
    std::lock_guard<std::mutex> lock(owner_pending_mutex_);
    return pending_owner_cleanup_request_.has_value() || !pending_cleanup_release_requests_.empty();
}

void TeleopNode::expirePendingOwnerRequest(const std::uint64_t steady_now_ns) {
    if (!owner_tracker_) {
        return;
    }
    const auto expired = owner_tracker_->expirePending(steady_now_ns);
    if (!expired.has_value()) {
        return;
    }
    std::optional<PendingOwnerRequest> pending;
    {
        std::lock_guard<std::mutex> lock(owner_pending_mutex_);
        if (pending_owner_request_.has_value() &&
            pending_owner_request_->operation.operation_id == expired->operation_id &&
            pending_owner_request_->operation.generation == expired->generation) {
            pending = pending_owner_request_;
            pending_owner_request_.reset();
        }
    }
    bool fail_session = expired->kind != ControlOwnerOperationKind::kRelease;
    if (pending.has_value()) {
        fail_session = pending->fail_session_on_error;
        if (expired->kind == ControlOwnerOperationKind::kAcquire ||
            expired->kind == ControlOwnerOperationKind::kRenew) {
            const std::uint64_t grace_ns =
                static_cast<std::uint64_t>(config_.owner_ttl_ms + config_.owner_service_timeout_ms) *
                kNanosecondsPerMillisecond;
            const std::uint64_t deadline = steady_now_ns > std::numeric_limits<std::uint64_t>::max() - grace_ns
                                               ? std::numeric_limits<std::uint64_t>::max()
                                               : steady_now_ns + grace_ns;
            bool stored_as_cleanup = false;
            {
                std::lock_guard<std::mutex> lock(owner_pending_mutex_);
                if (!pending_owner_cleanup_request_.has_value()) {
                    pending_owner_cleanup_request_ = PendingOwnerCleanupRequest{*pending, deadline};
                    stored_as_cleanup = true;
                }
            }
            if (!stored_as_cleanup) {
                removeOwnerClientPendingRequest(*pending);
            }
        } else {
            removeOwnerClientPendingRequest(*pending);
        }
    } else if (expired->kind == ControlOwnerOperationKind::kRelease) {
        // callback-before-register 极端竞态下仍以活动 session 为准；停止路径已经清空 session，不会被误判为故障。
        fail_session = running_.load() && session_registry_.current().has_value();
    }
    if (fail_session) {
        handleOwnerFailure(expired->kind == ControlOwnerOperationKind::kAcquire ? "owner_acquire_timeout"
                           : expired->kind == ControlOwnerOperationKind::kRenew ? "owner_renew_timeout"
                                                                                : "owner_release_timeout");
    }
}

void TeleopNode::expirePendingOwnerCleanupRequests(const std::uint64_t steady_now_ns) {
    std::optional<PendingOwnerCleanupRequest> expired_owner_request;
    std::vector<std::int64_t> expired_release_ids;
    {
        std::lock_guard<std::mutex> lock(owner_pending_mutex_);
        if (pending_owner_cleanup_request_.has_value() &&
            pending_owner_cleanup_request_->deadline_steady_ns <= steady_now_ns) {
            expired_owner_request = pending_owner_cleanup_request_;
            pending_owner_cleanup_request_.reset();
        }
        for (auto iterator = pending_cleanup_release_requests_.begin();
             iterator != pending_cleanup_release_requests_.end();) {
            if (iterator->second.deadline_steady_ns <= steady_now_ns) {
                if (iterator->second.request_id > 0) {
                    expired_release_ids.push_back(iterator->second.request_id);
                }
                iterator = pending_cleanup_release_requests_.erase(iterator);
            } else {
                ++iterator;
            }
        }
    }
    if (expired_owner_request.has_value()) {
        removeOwnerClientPendingRequest(expired_owner_request->request);
    }
    if (release_owner_client_) {
        for (const auto request_id : expired_release_ids) {
            release_owner_client_->remove_pending_request(request_id);
        }
    }
}

void TeleopNode::removeOwnerClientPendingRequest(const PendingOwnerRequest &pending) {
    if (pending.operation.kind == ControlOwnerOperationKind::kAcquire && acquire_owner_client_) {
        acquire_owner_client_->remove_pending_request(pending.request_id);
    } else if (pending.operation.kind == ControlOwnerOperationKind::kRenew && renew_owner_client_) {
        renew_owner_client_->remove_pending_request(pending.request_id);
    } else if (pending.operation.kind == ControlOwnerOperationKind::kRelease && release_owner_client_) {
        release_owner_client_->remove_pending_request(pending.request_id);
    }
}

void TeleopNode::pauseControlForReady(const std::string &reason_code) {
    quarantinePendingOwnerRequestForCleanup();
    const auto owner_lease = owner_tracker_ ? owner_tracker_->leaseSnapshot() : std::nullopt;
    bool session_present = false;
    bool paused = false;
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        session_present = session_registry_.current().has_value();
        paused = session_present && pauseControlForReadyLocked(reason_code);
    }
    if (!session_present) {
        if (owner_tracker_) {
            owner_tracker_->invalidate();
        }
        return;
    }
    if (!paused) {
        stopActiveSession("ready_state_transition_failed", true, true);
        return;
    }
    if (config_.productionOutputEnabled() && owner_lease.has_value()) {
        requestOwnerRelease(reason_code, false);
    } else if (owner_tracker_) {
        const auto pending = owner_tracker_->pendingOperation();
        if (!pending.has_value() || pending->kind != ControlOwnerOperationKind::kRelease) {
            owner_tracker_->invalidate();
        }
    }
    publishStatus();
}

bool TeleopNode::pauseControlForReadyLocked(const std::string &reason_code) {
    const auto binding = session_registry_.current();
    if (!binding.has_value() || !readyRunContextMatchesLocked(*binding)) {
        return false;
    }
    const TeleopState state = state_machine_.state();
    if ((state == TeleopState::kArmed || state == TeleopState::kControlling) &&
        !state_machine_.transition(TeleopEvent::kOwnerReleased, reason_code).ok()) {
        return false;
    }
    if (state != TeleopState::kConnected && state != TeleopState::kArmed && state != TeleopState::kControlling) {
        return false;
    }
    mailbox_.clear();
    command_limiter_->reset();
    command_mapper_->reset();
    arm_origin_mapper_->resetControlState();
    watchdog_->reset();
    deadman_active_ = false;
    deadman_requested_ = false;
    next_owner_renew_steady_ns_ = 0U;
    last_frame_steady_time_ns_ = 0U;
    last_reason_code_ = reason_code;
    return true;
}

void TeleopNode::stopActiveSession(const std::string &reason_code, const bool close_rtc, const bool fault,
                                   const bool preserve_ready_context) {
    if (preserve_ready_context) {
        bool ready_binding_present = false;
        {
            std::lock_guard<std::mutex> lock(runtime_mutex_);
            const auto current = session_registry_.current();
            ready_binding_present = current.has_value() && readyRunContextMatchesLocked(*current);
        }
        if (ready_binding_present) {
            quarantinePendingOwnerRequestForCleanup();
            const auto ready_owner_lease = owner_tracker_ ? owner_tracker_->leaseSnapshot() : std::nullopt;
            bool ready_context_preserved = false;
            {
                std::lock_guard<std::mutex> lock(runtime_mutex_);
                ready_context_preserved = pauseControlForReadyLocked("run_context_ready");
            }
            if (ready_context_preserved) {
                if (config_.productionOutputEnabled() && ready_owner_lease.has_value()) {
                    requestOwnerRelease("run_context_ready", false);
                } else if (owner_tracker_) {
                    const auto pending = owner_tracker_->pendingOperation();
                    if (!pending.has_value() || pending->kind != ControlOwnerOperationKind::kRelease) {
                        owner_tracker_->invalidate();
                    }
                }
                publishStatus();
                return;
            }
        }
    }

    quarantinePendingOwnerRequestForCleanup();
    const std::uint64_t now_ns = steadyNowNs();
    const auto owner_lease = owner_tracker_ ? owner_tracker_->leaseSnapshot() : std::nullopt;
    std::optional<SessionBinding> binding;
    std::optional<TeleopStatusObservation> terminal_observation;
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        binding = session_registry_.current();
        const TeleopState state = state_machine_.state();
        if (state != TeleopState::kIdle && state != TeleopState::kClosed) {
            if (fault && state != TeleopState::kFault) {
                static_cast<void>(state_machine_.transition(TeleopEvent::kFault, reason_code));
            } else if (state != TeleopState::kFault) {
                static_cast<void>(state_machine_.transition(TeleopEvent::kStopRequested, reason_code));
            }
            const TeleopState terminal_state = state_machine_.state();
            const auto terminal_binding = session_registry_.current();
            if (terminal_binding.has_value() && TeleopStatusReportScheduler::terminalState(terminal_state)) {
                terminal_observation = TeleopStatusObservation{terminal_state,
                                                               terminal_binding->session_id,
                                                               terminal_binding->run_id,
                                                               terminal_binding->resource_id,
                                                               terminal_binding->peer_id,
                                                               terminal_binding->channel_label,
                                                               last_sequence_,
                                                               sequence_gap_count_,
                                                               reason_code};
            }
            static_cast<void>(state_machine_.transition(TeleopEvent::kStopCompleted, reason_code));
        }
        if (binding.has_value()) {
            status_binding_ = binding;
            static_cast<void>(session_registry_.close(binding->session_id));
        } else {
            session_registry_.clear();
        }
        mailbox_.clear();
        frame_validator_->reset();
        command_limiter_->reset();
        command_mapper_->reset();
        arm_origin_mapper_->resetControlState();
        watchdog_->reset();
        deadman_active_ = false;
        deadman_requested_ = false;
        next_owner_renew_steady_ns_ = 0U;
        last_reason_code_ = reason_code;
    }
    if (binding.has_value()) {
        if (config_.shadowOutputEnabled() || config_.legacyMpcOutputEnabled()) {
            publishStopCommand(*binding, now_ns, 0U);
        }
        if (close_rtc) {
            requestRtcClose(*binding, reason_code);
        }
    }
    if (config_.productionOutputEnabled() && owner_lease.has_value()) {
        requestOwnerRelease(reason_code, false);
    } else if (owner_tracker_) {
        const auto pending = owner_tracker_->pendingOperation();
        if (!pending.has_value() || pending->kind != ControlOwnerOperationKind::kRelease) {
            owner_tracker_->invalidate();
        }
    }
    if (terminal_observation.has_value() && status_report_client_) {
        const Status report_status = status_report_client_->observe(*terminal_observation, systemNowSeconds());
        if (!report_status.ok()) {
            RCLCPP_WARN(get_logger(), "terminal Teleop status could not be queued; control stop already applied: %s",
                        report_status.message().c_str());
        }
    }
    publishStatus();
}

void TeleopNode::handleOwnerFailure(const std::string &reason_code) {
    std::lock_guard<std::mutex> transition_lock(control_transition_mutex_);
    stopActiveSession(reason_code, true, true, true);
}

void TeleopNode::requestRtcClose(const SessionBinding &binding, const std::string &reason_code) {
    if (!close_client_ || !close_client_->service_is_ready()) {
        RCLCPP_WARN(get_logger(), "RTC close service unavailable for session=%s", binding.session_id.c_str());
        return;
    }
    const std::string route_key = closeRouteKey(binding);
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        if (pending_close_requests_.find(route_key) != pending_close_requests_.end() ||
            pending_close_requests_.size() >= 4U) {
            RCLCPP_WARN(get_logger(), "RTC close request limit reached for session=%s", binding.session_id.c_str());
            return;
        }
    }
    auto request = std::make_shared<astrabot_rtc::srv::CloseRtcPeer::Request>();
    request->session_id = binding.session_id;
    request->peer_id = binding.peer_id;
    request->channel_label = binding.channel_label;
    request->reason_code = reason_code;
    const auto shared_node = weak_from_this().lock();
    if (!shared_node) {
        return;
    }
    const std::weak_ptr<TeleopNode> weak_self = std::static_pointer_cast<TeleopNode>(shared_node);
    auto future_and_id = close_client_->async_send_request(
        request, [weak_self, route_key](rclcpp::Client<astrabot_rtc::srv::CloseRtcPeer>::SharedFuture future) {
            if (const auto self = weak_self.lock()) {
                self->handleCloseResponse(route_key, future);
            }
        });
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_close_requests_[route_key] =
        PendingCloseRequest{future_and_id.request_id, steadyNowNs() + kCloseRequestTimeoutNs};
}

void TeleopNode::handleCloseResponse(const std::string &route_key,
                                     rclcpp::Client<astrabot_rtc::srv::CloseRtcPeer>::SharedFuture future) {
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        if (pending_close_requests_.erase(route_key) == 0U) {
            return;
        }
    }
    const auto response = future.get();
    if (!response || !response->closed) {
        RCLCPP_WARN(get_logger(), "RTC close request was not confirmed");
    }
}

void TeleopNode::expirePendingCloseRequests(const std::uint64_t steady_now_ns) {
    if (!close_client_) {
        return;
    }
    std::vector<std::int64_t> expired_ids;
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        for (auto iterator = pending_close_requests_.begin(); iterator != pending_close_requests_.end();) {
            if (iterator->second.deadline_steady_ns <= steady_now_ns) {
                expired_ids.push_back(iterator->second.request_id);
                iterator = pending_close_requests_.erase(iterator);
            } else {
                ++iterator;
            }
        }
    }
    for (const auto request_id : expired_ids) {
        close_client_->remove_pending_request(request_id);
    }
}

bool TeleopNode::authorizationRunContextMatchesLocked(const std::string &run_id, const std::string &resource_id) const {
    return run_context_.has_value() && runContextAllowsAuthorization(run_context_->state) &&
           run_context_->run_id == run_id && run_context_->resource_id == resource_id;
}

bool TeleopNode::authorizationRunContextMatchesLocked(const SessionBinding &binding) const {
    return authorizationRunContextMatchesLocked(binding.run_id, binding.resource_id);
}

bool TeleopNode::readyRunContextMatchesLocked(const SessionBinding &binding) const {
    return run_context_.has_value() &&
           run_context_->state == astrabot_data_interfaces::msg::DataCollectionRunContext::STATE_READY &&
           run_context_->run_id == binding.run_id && run_context_->resource_id == binding.resource_id;
}

bool TeleopNode::activeRunContextMatchesLocked(const std::string &run_id, const std::string &resource_id) const {
    return run_context_.has_value() && runContextAllowsControl(run_context_->state) && run_context_->run_id == run_id &&
           run_context_->resource_id == resource_id;
}

bool TeleopNode::activeRunContextMatchesLocked(const SessionBinding &binding) const {
    return activeRunContextMatchesLocked(binding.run_id, binding.resource_id);
}

void TeleopNode::publishCommand(const SessionBinding &binding, const DecodedTeleopFrame &frame,
                                const MappedCommand &mapped, const std::uint64_t receive_steady_time_ns,
                                const std::uint64_t owner_epoch) {
    if (config_.legacyMpcOutputEnabled()) {
        if (owner_epoch == 0U) {
            publishLegacyMpcCommand(mapped, steadyNowNs());
        }
        return;
    }
    if (!command_publisher_) {
        return;
    }
    const bool shadow_output = config_.shadowOutputEnabled();
    if ((!shadow_output && (!config_.productionOutputEnabled() || owner_epoch == 0U)) ||
        (shadow_output && owner_epoch != 0U)) {
        return;
    }
    if (!shadow_output) {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        if (!session_registry_.matches(binding.session_id, binding.peer_id, binding.channel_label) ||
            !activeRunContextMatchesLocked(binding) || state_machine_.state() != TeleopState::kControlling) {
            return;
        }
    }
    const std::uint64_t command_ttl_ns =
        static_cast<std::uint64_t>(config_.command_ttl_ms) * kNanosecondsPerMillisecond;
    if (receive_steady_time_ns > std::numeric_limits<std::uint64_t>::max() - command_ttl_ns ||
        static_cast<std::uint64_t>(frame.timestamp_ms) >
            std::numeric_limits<std::uint64_t>::max() / kNanosecondsPerMillisecond) {
        handleOwnerFailure("command_timestamp_overflow");
        return;
    }
    ::astrabot_teleop::msg::TeleopCommand command;
    command.session_id = binding.session_id;
    command.run_id = binding.run_id;
    command.resource_id = binding.resource_id;
    command.source_peer_id = binding.peer_id;
    command.channel_label = binding.channel_label;
    command.owner_epoch = owner_epoch;
    command.sequence = frame.sequence;
    command.source_timestamp_ns = static_cast<std::uint64_t>(frame.timestamp_ms) * kNanosecondsPerMillisecond;
    command.receive_steady_time_ns = receive_steady_time_ns;
    command.valid_until_ns = receive_steady_time_ns + command_ttl_ns;
    command.deadman = mapped.deadman;
    command.right_arm_valid = mapped.right_arm_valid;
    command.left_arm_valid = mapped.left_arm_valid;
    command.right_gripper_valid = mapped.right_gripper_valid;
    command.left_gripper_valid = mapped.left_gripper_valid;
    command.head_valid = mapped.head_valid;
    assignPose(mapped.right_arm_target, &command.right_arm_target);
    assignPose(mapped.left_arm_target, &command.left_arm_target);
    assignPose(mapped.head_target, &command.head_target);
    command.right_gripper = static_cast<float>(mapped.right_gripper);
    command.left_gripper = static_cast<float>(mapped.left_gripper);
    command.chassis_command.linear.x = mapped.chassis_linear_x;
    command.chassis_command.angular.z = mapped.chassis_angular_z;
    command.shadow_only = shadow_output;
    command_publisher_->publish(command);
}

void TeleopNode::publishStopCommand(const SessionBinding &binding, const std::uint64_t steady_now_ns,
                                    const std::uint64_t owner_epoch) {
    if (config_.legacyMpcOutputEnabled()) {
        if (owner_epoch == 0U) {
            publishLegacyMpcHold(steady_now_ns);
        }
        return;
    }
    if (!command_publisher_ || !config_.shadowOutputEnabled() || owner_epoch != 0U) {
        return;
    }
    std::uint64_t last_sequence = 0U;
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        last_sequence = last_sequence_;
    }
    ::astrabot_teleop::msg::TeleopCommand command;
    command.session_id = binding.session_id;
    command.run_id = binding.run_id;
    command.resource_id = binding.resource_id;
    command.source_peer_id = binding.peer_id;
    command.channel_label = binding.channel_label;
    command.owner_epoch = 0U;
    command.sequence = last_sequence;
    command.receive_steady_time_ns = steady_now_ns;
    command.valid_until_ns = steady_now_ns;
    command.deadman = false;
    command.right_arm_valid = false;
    command.left_arm_valid = false;
    command.right_gripper_valid = false;
    command.left_gripper_valid = false;
    command.head_valid = false;
    command.right_arm_target.orientation.w = 1.0;
    command.left_arm_target.orientation.w = 1.0;
    command.head_target.orientation.w = 1.0;
    command.shadow_only = true;
    command_publisher_->publish(command);
}

void TeleopNode::publishLegacyMpcCommand(const MappedCommand &mapped, const std::uint64_t steady_now_ns) {
    if (!legacy_mpc_encoder_ || !legacy_reference_pose_publisher_) {
        return;
    }
    if (!mapped.deadman) {
        publishLegacyMpcHold(steady_now_ns);
        return;
    }
    auto encoded = legacy_mpc_encoder_->encodeCommand(mapped, config_.legacy_control_name);
    if (!encoded.ok()) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "legacy MPC command rejected: %s",
                              encoded.status().message().c_str());
        return;
    }
    std_msgs::msg::String reference;
    reference.data = encoded.takeValue();
    legacy_reference_pose_publisher_->publish(reference);

    if (mapped.left_gripper_valid && legacy_left_gripper_publisher_ && std::isfinite(mapped.left_gripper)) {
        std_msgs::msg::Float64 gripper;
        gripper.data = std::clamp(mapped.left_gripper, 0.0, 1.0);
        legacy_left_gripper_publisher_->publish(gripper);
    }
    if (mapped.right_gripper_valid && legacy_right_gripper_publisher_ && std::isfinite(mapped.right_gripper)) {
        std_msgs::msg::Float64 gripper;
        gripper.data = std::clamp(mapped.right_gripper, 0.0, 1.0);
        legacy_right_gripper_publisher_->publish(gripper);
    }
}

void TeleopNode::publishLegacyMpcHold(const std::uint64_t steady_now_ns) {
    if (!legacy_mpc_encoder_ || !legacy_reference_pose_publisher_) {
        return;
    }
    RobotEePoseSnapshot snapshot;
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        snapshot = robot_ee_pose_;
    }
    const std::uint64_t max_pose_age_ns =
        static_cast<std::uint64_t>(config_.robot_pose_max_age_ms) * kNanosecondsPerMillisecond;
    if (!snapshot.available || snapshot.receive_steady_time_ns == 0U ||
        steady_now_ns < snapshot.receive_steady_time_ns ||
        steady_now_ns - snapshot.receive_steady_time_ns > max_pose_age_ns) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                              "legacy MPC hold rejected because robot end-effector pose is unavailable or stale");
        return;
    }
    auto encoded = legacy_mpc_encoder_->encodeHold(snapshot.right_arm, snapshot.left_arm, config_.legacy_control_name);
    if (!encoded.ok()) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "legacy MPC hold rejected: %s",
                              encoded.status().message().c_str());
        return;
    }
    std_msgs::msg::String reference;
    reference.data = encoded.takeValue();
    legacy_reference_pose_publisher_->publish(reference);
}

void TeleopNode::publishStatus() {
    if (!status_publisher_) {
        return;
    }
    ::astrabot_teleop::msg::TeleopSessionStatus message;
    {
        std::lock_guard<std::mutex> lock(runtime_mutex_);
        const TeleopState state = state_machine_.state();
        message.state = static_cast<std::uint8_t>(state);
        const auto current_binding = session_registry_.current();
        const auto binding = current_binding.has_value() ? current_binding : status_binding_;
        if (binding.has_value()) {
            message.session_id = binding->session_id;
            message.run_id = binding->run_id;
            message.resource_id = binding->resource_id;
            message.peer_id = binding->peer_id;
            message.channel_label = binding->channel_label;
        }
        message.authorized = state == TeleopState::kAuthorized || state == TeleopState::kConnected ||
                             state == TeleopState::kArmed || state == TeleopState::kControlling ||
                             state == TeleopState::kStopping;
        message.connected = state == TeleopState::kConnected || state == TeleopState::kArmed ||
                            state == TeleopState::kControlling || state == TeleopState::kStopping;
        message.deadman_active = deadman_active_;
        message.last_sequence = last_sequence_;
        message.sequence_gap_count = sequence_gap_count_;
        message.rejected_frame_count = rejected_frame_count_;
        message.mailbox_overwrite_count = mailbox_.overwriteCount();
        message.watchdog_stop_count = watchdog_stop_count_;
        message.last_frame_steady_time_ns = last_frame_steady_time_ns_;
        message.reason_code = last_reason_code_;
    }
    status_publisher_->publish(message);
    if (status_report_client_ && !message.session_id.empty() && !message.run_id.empty() &&
        !message.resource_id.empty() && !message.peer_id.empty() && !message.channel_label.empty()) {
        const TeleopStatusObservation observation{static_cast<TeleopState>(message.state),
                                                  message.session_id,
                                                  message.run_id,
                                                  message.resource_id,
                                                  message.peer_id,
                                                  message.channel_label,
                                                  message.last_sequence,
                                                  message.sequence_gap_count,
                                                  message.reason_code};
        const Status report_status = status_report_client_->observe(observation, systemNowSeconds());
        if (!report_status.ok()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Teleop status report queue rejected an observation; control continues reason=%s",
                                 report_status.message().c_str());
        }
    }
}

std::uint64_t TeleopNode::steadyNowNs() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

std::int64_t TeleopNode::systemNowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
        .count();
}

std::int64_t TeleopNode::systemNowSeconds() {
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch())
        .count();
}

std::uint64_t TeleopNode::systemNowNs() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
            .count());
}

std::string TeleopNode::closeRouteKey(const SessionBinding &binding) {
    return binding.session_id + "\n" + binding.peer_id + "\n" + binding.channel_label;
}

std::string TeleopNode::ownerCleanupRouteKey(const ControlOwnerIdentity &identity, const std::uint64_t owner_epoch) {
    return identity.session_id + "\n" + identity.run_id + "\n" + identity.resource_id + "\n" + identity.source_id +
           "\n" + std::to_string(owner_epoch);
}

ControlOwnerIdentity TeleopNode::ownerIdentity(const SessionBinding &binding) {
    return ControlOwnerIdentity{binding.session_id, binding.run_id, binding.resource_id, binding.peer_id};
}

}  // namespace astrabot::teleop
