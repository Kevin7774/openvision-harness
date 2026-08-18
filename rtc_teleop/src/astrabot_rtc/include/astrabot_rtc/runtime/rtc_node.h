#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "astrabot_rtc/common/status.h"
#include "astrabot_rtc/config/rtc_config.h"
#include "astrabot_rtc/media/h264_encoder_worker.h"
#include "astrabot_rtc/msg/rtc_data_packet.hpp"
#include "astrabot_rtc/msg/rtc_peer_event.hpp"
#include "astrabot_rtc/runtime/media_rate_tracker.h"
#include "astrabot_rtc/session/data_channel_router.h"
#include "astrabot_rtc/session/session_registry.h"
#include "astrabot_rtc/signaling/signaling_adapter.h"
#include "astrabot_rtc/srv/authorize_data_channel.hpp"
#include "astrabot_rtc/srv/close_rtc_peer.hpp"
#include "astrabot_rtc/transport/webrtc/webrtc_transport.h"

namespace astrabot::rtc::runtime {

/**
 * @brief 组装 ROS 契约、通用 RTC core 与 WebRTC transport adapter 的运行节点。
 *
 * 构造函数只声明参数；start/stop 必须由同一控制线程串行调用。所有 ROS callback 都会检查 running_，stop 可幂等调用。
 */
class RtcNode final : public rclcpp::Node {
  public:
    RtcNode();
    ~RtcNode() override;

    /**
     * @brief 加载配置并启动 ROS endpoint 与 transport。
     */
    Status start();

    /**
     * @brief 停止新 callback、清理 pending request 并释放全部运行资源。
     */
    void stop();

  private:
    class CallbackLease final {
      public:
        CallbackLease(CallbackLease &&other) noexcept;
        CallbackLease &operator=(CallbackLease &&other) noexcept;
        ~CallbackLease();

        CallbackLease(const CallbackLease &) = delete;
        CallbackLease &operator=(const CallbackLease &) = delete;

      private:
        friend class RtcNode;
        explicit CallbackLease(RtcNode *owner);

        RtcNode *owner_{nullptr};
    };

    struct PendingAuthorization {
        session::DataChannelKey key;
        std::int64_t request_id{0};
        std::uint64_t deadline_steady_ns{0U};
        std::uint64_t generation{0U};
    };

    struct DataChannelLifecycle {
        session::DataChannelKey key;
        bool sdk_open{false};
        bool ready_event_published{false};
    };

    Status initializeCore();
    Status initializeRosEndpoints();
    Status initializeMediaInputs();
    Status initializeAdapters();
    void resetRosEndpoints();

    Status handleSignaling(const std::string &payload);
    void handlePeerEvent(const session::PeerDescriptor &peer);
    void handleDataChannelOpen(transport::DataChannelOpenRequest request);
    void handleDataChannelReady(transport::DataChannelReadyEvent event);
    void handleDataChannelPacket(protocol::DataChannelPacket packet);
    void handleSignalingReport(std::string payload);
    void handleAuthorizationResponse(const std::string &route_key, std::uint64_t generation,
                                     rclcpp::Client<::astrabot_rtc::srv::AuthorizeDataChannel>::SharedFuture future);
    void handleClosePeerRequest(const ::astrabot_rtc::srv::CloseRtcPeer::Request::SharedPtr request,
                                ::astrabot_rtc::srv::CloseRtcPeer::Response::SharedPtr response);
    void dispatchLatestPackets();
    void expirePendingAuthorizations();
    void publishDiagnostics();
    void handleImage(const std::string &track_id, const sensor_msgs::msg::Image::ConstSharedPtr message);
    void handleCameraInfo(const std::string &track_id, const sensor_msgs::msg::CameraInfo::ConstSharedPtr message);
    void revokeAuthorizationState(const std::string &session_id, const std::string &peer_id,
                                  const std::string &channel_label);
    void closeTransportChannel(const session::DataChannelKey &key, const std::string &reason_code);
    void publishDataChannelReadyEvent(const session::DataChannelKey &key);
    void publishPeerEvent(const session::PeerDescriptor &peer);
    std::optional<CallbackLease> acquireCallbackLease();
    void releaseCallbackLease();

    static std::string makeRouteKey(const session::DataChannelKey &key);
    static std::uint64_t steadyNowNanoseconds();
    static std::uint64_t imageStampNanoseconds(const builtin_interfaces::msg::Time &stamp);

    std::atomic<bool> running_{false};
    mutable std::mutex callback_mutex_;
    std::condition_variable callbacks_drained_;
    std::size_t active_callbacks_{0U};
    config::RtcConfig config_;
    std::unique_ptr<session::SessionRegistry> session_registry_;
    std::unique_ptr<session::DataChannelRouter> data_channel_router_;
    std::unordered_map<std::string, std::unique_ptr<media::H264EncoderWorker>> encoder_workers_;
    MediaRateTracker media_rate_tracker_;
    std::unique_ptr<signaling::ISignalingAdapter> signaling_adapter_;
    std::unique_ptr<transport::IWebRtcTransport> transport_;

    rclcpp::Publisher<::astrabot_rtc::msg::RtcPeerEvent>::SharedPtr peer_event_publisher_;
    rclcpp::Publisher<::astrabot_rtc::msg::RtcDataPacket>::SharedPtr data_packet_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
    rclcpp::Client<::astrabot_rtc::srv::AuthorizeDataChannel>::SharedPtr authorization_client_;
    rclcpp::Service<::astrabot_rtc::srv::CloseRtcPeer>::SharedPtr close_peer_service_;
    rclcpp::TimerBase::SharedPtr dispatch_timer_;
    rclcpp::TimerBase::SharedPtr authorization_timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subscriptions_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_subscriptions_;

    mutable std::mutex pending_mutex_;
    std::unordered_map<std::string, PendingAuthorization> pending_authorizations_;
    std::unordered_map<std::string, DataChannelLifecycle> data_channel_lifecycles_;
    std::uint64_t next_authorization_generation_{1U};
    std::atomic<std::uint64_t> camera_frame_count_{0U};
    std::atomic<std::uint64_t> camera_frame_reject_count_{0U};
    std::atomic<std::uint64_t> camera_info_count_{0U};
    std::atomic<std::uint64_t> authorization_reject_count_{0U};
};

}  // namespace astrabot::rtc::runtime
