// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include "astrabot_data_interfaces/msg/data_collection_run_context.hpp"
#include "astrabot_rtc/msg/rtc_data_packet.hpp"
#include "astrabot_rtc/msg/rtc_peer_event.hpp"
#include "astrabot_rtc/srv/authorize_data_channel.hpp"
#include "astrabot_rtc/srv/close_rtc_peer.hpp"
#include "astrabot_teleop/adapter/legacy_mpc_command_encoder.h"
#include "astrabot_teleop/common/status.h"
#include "astrabot_teleop/config/teleop_config.h"
#include "astrabot_teleop/grant/grant_verifier.h"
#include "astrabot_teleop/mapping/arm_origin_mapper.h"
#include "astrabot_teleop/mapping/command_mapper.h"
#include "astrabot_teleop/msg/teleop_command.hpp"
#include "astrabot_teleop/msg/teleop_session_status.hpp"
#include "astrabot_teleop/protocol/frame_validator.h"
#include "astrabot_teleop/protocol/teleop_frame_codec.h"
#include "astrabot_teleop/runtime/latest_mailbox.h"
#include "astrabot_teleop/runtime/teleop_status_report_client.h"
#include "astrabot_teleop/safety/command_limiter.h"
#include "astrabot_teleop/safety/motion_watchdog.h"
#include "astrabot_teleop/session/control_owner_lease_tracker.h"
#include "astrabot_teleop/session/run_context_generation_tracker.h"
#include "astrabot_teleop/session/session_registry.h"
#include "astrabot_teleop/session/teleop_state_machine.h"
#include "astrabot_teleop/srv/acquire_control_owner.hpp"
#include "astrabot_teleop/srv/release_control_owner.hpp"
#include "astrabot_teleop/srv/renew_control_owner.hpp"

namespace astrabot::teleop {

/**
 * @brief 将通用 RTC DataChannel 接入 Teleop 纯 C++ 安全链并发布类型化命令。
 *
 * 构造函数只声明参数。对象按 one-shot 生命周期使用：start()/stop() 必须由同一控制线程串行调用，stop() 后不得对
 * 同一对象再次 start()；高频 packet 使用容量 1 mailbox，
 * 不在 RTC callback 中执行 protobuf 解析或 ROS 发布。shadow backend 不产生真实运动；legacy_mpc backend 通过旧
 * `/reference/pose` 与夹爪 topic 兼容现有机器人；cpp backend 只有在 ACTIVE run context 与 arbitration owner lease
 * 同时有效时才发布 typed 生产命令。
 */
class TeleopNode final : public rclcpp::Node {
  public:
    /**
     * @brief 创建 Teleop ROS 节点并声明参数，不启动任何订阅、服务或定时器。
     *
     * @param options ROS 节点选项；测试和组合部署可通过 parameter_overrides 注入隔离 endpoint。
     */
    explicit TeleopNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~TeleopNode() override;

    /** @brief 加载参数、公钥并启动 ROS endpoint；stop 后再次调用返回 FailedPrecondition。 */
    Status start();

    /** @brief 幂等停止 timers、pending service 和活动 session。 */
    void stop();

  private:
    struct PendingCloseRequest {
        std::int64_t request_id{0};
        std::uint64_t deadline_steady_ns{0};
    };

    struct PendingOwnerRequest {
        std::int64_t request_id{0};
        ControlOwnerOperation operation;
        bool fail_session_on_error{true};
    };

    struct PendingOwnerCleanupRequest {
        PendingOwnerRequest request;
        std::uint64_t deadline_steady_ns{0U};
    };

    struct PendingCleanupReleaseRequest {
        std::int64_t request_id{0};
        std::uint64_t deadline_steady_ns{0U};
    };

    struct RobotEePoseSnapshot {
        PoseSample right_arm;
        PoseSample left_arm;
        std::uint64_t receive_steady_time_ns{0U};
        bool available{false};
    };

    Status loadConfig();
    Status initializeCore();
    Status initializeRosEndpoints();
    void resetRosEndpoints();
    void handleAuthorize(const astrabot_rtc::srv::AuthorizeDataChannel::Request::SharedPtr request,
                         astrabot_rtc::srv::AuthorizeDataChannel::Response::SharedPtr response);
    void handlePeerEvent(const astrabot_rtc::msg::RtcPeerEvent::ConstSharedPtr message);
    void handleDataPacket(const astrabot_rtc::msg::RtcDataPacket::ConstSharedPtr message);
    void handleRobotEePose(const geometry_msgs::msg::PoseArray::ConstSharedPtr message);
    void handleRunContext(const astrabot_data_interfaces::msg::DataCollectionRunContext::ConstSharedPtr message);
    void processLatestPacket();
    void processPacket(const astrabot_rtc::msg::RtcDataPacket &packet);
    void checkAuthorizationDeadline(std::uint64_t steady_now_ns);
    void checkWatchdog(std::uint64_t steady_now_ns);
    void checkOwnerLease(std::uint64_t steady_now_ns);
    void pauseControlForReady(const std::string &reason_code);
    bool pauseControlForReadyLocked(const std::string &reason_code);
    void stopActiveSession(const std::string &reason_code, bool close_rtc, bool fault = false,
                           bool preserve_ready_context = false);
    void handleOwnerFailure(const std::string &reason_code);
    void requestRtcClose(const SessionBinding &binding, const std::string &reason_code);
    void handleCloseResponse(const std::string &route_key,
                             rclcpp::Client<astrabot_rtc::srv::CloseRtcPeer>::SharedFuture future);
    void expirePendingCloseRequests(std::uint64_t steady_now_ns);
    void requestOwnerAcquire(const SessionBinding &binding);
    void requestOwnerRenew(const ControlOwnerLease &lease);
    void requestOwnerRelease(const std::string &reason_code, bool fail_session_on_error);
    void requestOwnerCleanupRelease(const ControlOwnerIdentity &identity, std::uint64_t owner_epoch,
                                    const std::string &reason_code);
    void handleOwnerAcquireResponse(const ControlOwnerOperation &operation,
                                    rclcpp::Client<::astrabot_teleop::srv::AcquireControlOwner>::SharedFuture future);
    void handleOwnerRenewResponse(const ControlOwnerOperation &operation,
                                  rclcpp::Client<::astrabot_teleop::srv::RenewControlOwner>::SharedFuture future);
    void handleOwnerReleaseResponse(const ControlOwnerOperation &operation, bool fail_session_on_error,
                                    rclcpp::Client<::astrabot_teleop::srv::ReleaseControlOwner>::SharedFuture future);
    bool registerPendingOwnerRequest(std::int64_t request_id, const ControlOwnerOperation &operation,
                                     bool fail_session_on_error);
    void clearPendingOwnerRequest(const ControlOwnerOperation &operation);
    void cancelPendingOwnerRequest();
    void quarantinePendingOwnerRequestForCleanup();
    bool ownerCleanupPending() const;
    void expirePendingOwnerRequest(std::uint64_t steady_now_ns);
    void expirePendingOwnerCleanupRequests(std::uint64_t steady_now_ns);
    void removeOwnerClientPendingRequest(const PendingOwnerRequest &pending);
    void
    handleOwnerCleanupReleaseResponse(const std::string &route_key,
                                      rclcpp::Client<::astrabot_teleop::srv::ReleaseControlOwner>::SharedFuture future);
    bool authorizationRunContextMatchesLocked(const std::string &run_id, const std::string &resource_id) const;
    bool authorizationRunContextMatchesLocked(const SessionBinding &binding) const;
    bool readyRunContextMatchesLocked(const SessionBinding &binding) const;
    bool activeRunContextMatchesLocked(const std::string &run_id, const std::string &resource_id) const;
    bool activeRunContextMatchesLocked(const SessionBinding &binding) const;
    void publishCommand(const SessionBinding &binding, const DecodedTeleopFrame &frame, const MappedCommand &mapped,
                        std::uint64_t receive_steady_time_ns, std::uint64_t owner_epoch);
    void publishStopCommand(const SessionBinding &binding, std::uint64_t steady_now_ns, std::uint64_t owner_epoch);
    void publishLegacyMpcCommand(const MappedCommand &mapped, std::uint64_t steady_now_ns);
    void publishLegacyMpcHold(std::uint64_t steady_now_ns);
    void publishStatus();

    static std::uint64_t steadyNowNs();
    static std::int64_t systemNowMs();
    static std::int64_t systemNowSeconds();
    static std::uint64_t systemNowNs();
    static std::string closeRouteKey(const SessionBinding &binding);
    static std::string ownerCleanupRouteKey(const ControlOwnerIdentity &identity, std::uint64_t owner_epoch);
    static ControlOwnerIdentity ownerIdentity(const SessionBinding &binding);

    std::atomic<bool> running_{false};
    std::atomic<bool> stopped_permanently_{false};
    TeleopConfig config_;
    std::unique_ptr<GrantVerifier> grant_verifier_;
    std::unique_ptr<TeleopFrameCodec> codec_;
    std::unique_ptr<FrameValidator> frame_validator_;
    std::unique_ptr<CommandLimiter> command_limiter_;
    std::unique_ptr<CommandMapper> command_mapper_;
    std::unique_ptr<ArmOriginMapper> arm_origin_mapper_;
    std::unique_ptr<LegacyMpcCommandEncoder> legacy_mpc_encoder_;
    std::unique_ptr<MotionWatchdog> watchdog_;
    std::unique_ptr<ControlOwnerLeaseTracker> owner_tracker_;
    std::shared_ptr<TeleopStatusReportClient> status_report_client_;
    SessionRegistry session_registry_;
    TeleopStateMachine state_machine_;
    LatestMailbox<astrabot_rtc::msg::RtcDataPacket> mailbox_;

    rclcpp::Publisher<::astrabot_teleop::msg::TeleopCommand>::SharedPtr command_publisher_;
    rclcpp::Publisher<::astrabot_teleop::msg::TeleopSessionStatus>::SharedPtr status_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr legacy_reference_pose_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr legacy_left_gripper_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr legacy_right_gripper_publisher_;
    rclcpp::Subscription<astrabot_rtc::msg::RtcDataPacket>::SharedPtr data_subscription_;
    rclcpp::Subscription<astrabot_rtc::msg::RtcPeerEvent>::SharedPtr peer_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr robot_ee_pose_subscription_;
    rclcpp::Subscription<astrabot_data_interfaces::msg::DataCollectionRunContext>::SharedPtr run_context_subscription_;
    rclcpp::Service<astrabot_rtc::srv::AuthorizeDataChannel>::SharedPtr authorize_service_;
    rclcpp::Client<astrabot_rtc::srv::CloseRtcPeer>::SharedPtr close_client_;
    rclcpp::Client<::astrabot_teleop::srv::AcquireControlOwner>::SharedPtr acquire_owner_client_;
    rclcpp::Client<::astrabot_teleop::srv::RenewControlOwner>::SharedPtr renew_owner_client_;
    rclcpp::Client<::astrabot_teleop::srv::ReleaseControlOwner>::SharedPtr release_owner_client_;
    rclcpp::TimerBase::SharedPtr process_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    mutable std::mutex runtime_mutex_;
    mutable std::mutex authorization_mutex_;
    mutable std::mutex control_transition_mutex_;
    mutable std::mutex pending_mutex_;
    mutable std::mutex owner_pending_mutex_;
    std::optional<SessionBinding> status_binding_;
    std::unordered_map<std::string, PendingCloseRequest> pending_close_requests_;
    std::optional<PendingOwnerRequest> pending_owner_request_;
    std::optional<PendingOwnerCleanupRequest> pending_owner_cleanup_request_;
    std::unordered_map<std::string, PendingCleanupReleaseRequest> pending_cleanup_release_requests_;
    std::uint64_t rejected_frame_count_{0};
    std::uint64_t watchdog_stop_count_{0};
    std::uint64_t last_sequence_{0};
    std::uint64_t sequence_gap_count_{0};
    std::uint64_t last_frame_steady_time_ns_{0};
    std::uint64_t next_owner_renew_steady_ns_{0};
    std::optional<astrabot_data_interfaces::msg::DataCollectionRunContext> run_context_;
    RobotEePoseSnapshot robot_ee_pose_;
    RunContextGenerationTracker run_context_generation_tracker_;
    std::uint64_t run_context_version_high_watermark_{0U};
    bool deadman_active_{false};
    bool deadman_requested_{false};
    std::string last_reason_code_{"startup"};
};

}  // namespace astrabot::teleop
