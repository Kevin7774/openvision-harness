#include "astrabot_rtc/runtime/rtc_node.h"

#include <algorithm>
#include <chrono>
#include <limits>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include "astrabot_rtc/media/ffmpeg_h264_encoder.h"
#include "astrabot_rtc/msg/rtc_data_packet.hpp"
#include "astrabot_rtc/msg/rtc_peer_event.hpp"
#include "astrabot_rtc/signaling/ros_signaling_adapter.h"
#include "astrabot_rtc/transport/webrtc/webrtc_transport_factory.h"

namespace astrabot::rtc::runtime {
namespace {

constexpr const char *kDefaultConfigPath = "/opt/ros/astrabot/share/astrabot_rtc/config/rtc.yaml";

diagnostic_msgs::msg::KeyValue makeDiagnosticValue(const std::string &key, const std::string &value) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    return item;
}

media::H264EncoderSettings makeEncoderSettings(const config::H264EncoderSettings &source) {
    media::H264EncoderSettings settings;
    settings.encoder_name = source.encoder_name;
    settings.require_hardware = source.require_hardware;
    settings.output_width = source.output_width;
    settings.output_height = source.output_height;
    settings.primary_frame_rate = source.frame_rate;
    settings.fallback_frame_rate = source.fallback_frame_rate;
    settings.bitrate_bps = source.bitrate_bps;
    settings.gop_size_frames = source.gop_size_frames;
    settings.max_encoded_frame_bytes = source.max_encoded_frame_bytes;
    settings.max_encoder_surfaces = source.max_encoder_surfaces;
    settings.pixel_format = source.pixel_format;
    settings.preset = source.preset;
    settings.tune = source.tune;
    settings.profile = source.profile;
    settings.level = source.level;
    return settings;
}

}  // namespace

RtcNode::RtcNode() : rclcpp::Node("astrabot_rtc") {
    declare_parameter<std::string>("rtc_config_path", kDefaultConfigPath);
}

RtcNode::CallbackLease::CallbackLease(RtcNode *owner) : owner_(owner) {}

RtcNode::CallbackLease::CallbackLease(CallbackLease &&other) noexcept : owner_(other.owner_) {
    other.owner_ = nullptr;
}

RtcNode::CallbackLease &RtcNode::CallbackLease::operator=(CallbackLease &&other) noexcept {
    if (this != &other) {
        if (owner_ != nullptr) {
            owner_->releaseCallbackLease();
        }
        owner_ = other.owner_;
        other.owner_ = nullptr;
    }
    return *this;
}

RtcNode::CallbackLease::~CallbackLease() {
    if (owner_ != nullptr) {
        owner_->releaseCallbackLease();
    }
}

RtcNode::~RtcNode() {
    stop();
}

Status RtcNode::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return Status::success();
    }

    const std::string config_path = get_parameter("rtc_config_path").as_string();
    config::RtcConfigLoader loader;
    auto loaded = loader.load(config_path);
    if (!loaded.ok()) {
        running_.store(false);
        return loaded.status();
    }
    config_ = loaded.takeValue();

    Status status = initializeCore();
    if (!status.ok()) {
        stop();
        return status;
    }
    status = initializeRosEndpoints();
    if (!status.ok()) {
        stop();
        return status;
    }
    status = initializeAdapters();
    if (!status.ok()) {
        stop();
        return status;
    }
    status = initializeMediaInputs();
    if (!status.ok()) {
        stop();
        return status;
    }

    const auto capabilities = transport_->capabilities();
    if (!capabilities.peer_connections || !capabilities.data_channels) {
        RCLCPP_WARN(get_logger(),
                    "astrabot_rtc started with backend=%s; peer_connections=%s data_channels=%s, so no real RTC "
                    "session can be established",
                    capabilities.backend.c_str(), capabilities.peer_connections ? "true" : "false",
                    capabilities.data_channels ? "true" : "false");
    } else if (!capabilities.media_tracks) {
        RCLCPP_INFO(get_logger(),
                    "astrabot_rtc started with backend=%s in data-only mode; PeerConnection/DataChannel are available "
                    "but video viewers are disabled",
                    capabilities.backend.c_str());
    }
    return Status::success();
}

void RtcNode::stop() {
    if (!running_.exchange(false)) {
        return;
    }

    dispatch_timer_.reset();
    authorization_timer_.reset();
    diagnostics_timer_.reset();
    image_subscriptions_.clear();
    camera_info_subscriptions_.clear();
    close_peer_service_.reset();

    if (signaling_adapter_) {
        signaling_adapter_->stop();
    }
    {
        std::unique_lock<std::mutex> lock(callback_mutex_);
        callbacks_drained_.wait(lock, [this]() { return active_callbacks_ == 0U; });
    }

    std::vector<std::int64_t> request_ids;
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        request_ids.reserve(pending_authorizations_.size());
        for (const auto &entry : pending_authorizations_) {
            if (entry.second.request_id >= 0) {
                request_ids.push_back(entry.second.request_id);
            }
        }
        pending_authorizations_.clear();
        data_channel_lifecycles_.clear();
    }
    if (authorization_client_) {
        for (const std::int64_t request_id : request_ids) {
            authorization_client_->remove_pending_request(request_id);
        }
    }
    authorization_client_.reset();

    for (auto &entry : encoder_workers_) {
        entry.second->stop();
    }
    encoder_workers_.clear();
    if (transport_) {
        transport_->stop();
    }
    if (data_channel_router_) {
        data_channel_router_->stop();
    }
    if (session_registry_) {
        session_registry_->clear();
    }
    media_rate_tracker_.reset();
    resetRosEndpoints();
    signaling_adapter_.reset();
    transport_.reset();
    data_channel_router_.reset();
    session_registry_.reset();
}

Status RtcNode::initializeCore() {
    session_registry_ =
        std::make_unique<session::SessionRegistry>(config_.runtime.max_peers, config_.runtime.max_viewers);
    data_channel_router_ = std::make_unique<session::DataChannelRouter>(
        config_.runtime.max_data_channels, config_.runtime.max_payload_bytes, std::make_shared<SteadyClock>());
    Status status = data_channel_router_->start();
    if (!status.ok()) {
        return status;
    }

    return Status::success();
}

Status RtcNode::initializeRosEndpoints() {
    peer_event_publisher_ =
        create_publisher<::astrabot_rtc::msg::RtcPeerEvent>(config_.topics.peer_event, rclcpp::QoS(32U).reliable());
    data_packet_publisher_ = create_publisher<::astrabot_rtc::msg::RtcDataPacket>(config_.topics.data_received,
                                                                                  rclcpp::QoS(1U).best_effort());
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(config_.topics.diagnostics,
                                                                                     rclcpp::QoS(10U).reliable());
    authorization_client_ =
        create_client<::astrabot_rtc::srv::AuthorizeDataChannel>(config_.topics.authorize_channel_service);

    const std::weak_ptr<RtcNode> weak_self = std::static_pointer_cast<RtcNode>(shared_from_this());
    close_peer_service_ = create_service<::astrabot_rtc::srv::CloseRtcPeer>(
        config_.topics.close_peer_service,
        [weak_self](const ::astrabot_rtc::srv::CloseRtcPeer::Request::SharedPtr request,
                    ::astrabot_rtc::srv::CloseRtcPeer::Response::SharedPtr response) {
            if (const auto self = weak_self.lock()) {
                self->handleClosePeerRequest(request, response);
            } else {
                response->closed = false;
                response->reason_code = "rtc_runtime_stopped";
            }
        });

    dispatch_timer_ = create_wall_timer(std::chrono::milliseconds(config_.runtime.dispatch_period_ms), [weak_self]() {
        if (const auto self = weak_self.lock()) {
            self->dispatchLatestPackets();
        }
    });
    authorization_timer_ = create_wall_timer(std::chrono::milliseconds(20), [weak_self]() {
        if (const auto self = weak_self.lock()) {
            self->expirePendingAuthorizations();
        }
    });
    diagnostics_timer_ =
        create_wall_timer(std::chrono::milliseconds(config_.runtime.diagnostics_period_ms), [weak_self]() {
            if (const auto self = weak_self.lock()) {
                self->publishDiagnostics();
            }
        });
    return Status::success();
}

Status RtcNode::initializeMediaInputs() {
    if (!config_.media.enabled) {
        return Status::success();
    }
    if (!transport_ || !transport_->capabilities().media_tracks) {
        return Status::error(ErrorCode::kFailedPrecondition,
                             "media.enabled requires a transport with real media track capability");
    }

    const media::H264EncoderSettings encoder_settings = makeEncoderSettings(config_.media.encoder);
    for (const auto &track : config_.media.tracks) {
        auto encoder = std::make_unique<media::FfmpegH264Encoder>(encoder_settings);
        auto worker = std::make_unique<media::H264EncoderWorker>(track.track_id, std::move(encoder));
        const Status status = worker->start([this](std::shared_ptr<const media::EncodedVideoFrame> frame) {
            if (!running_.load() || !transport_) {
                return Status::error(ErrorCode::kFailedPrecondition, "RTC runtime stopped before encoded frame send");
            }
            return transport_->sendEncodedFrame(std::move(frame));
        });
        if (!status.ok()) {
            return Status::error(status.code(),
                                 "failed to start H264 encoder for track " + track.track_id + ": " + status.message());
        }
        encoder_workers_.emplace(track.track_id, std::move(worker));
    }

    const std::weak_ptr<RtcNode> weak_self = std::static_pointer_cast<RtcNode>(shared_from_this());
    for (const auto &track : config_.media.tracks) {
        image_subscriptions_.push_back(create_subscription<sensor_msgs::msg::Image>(
            track.image_topic, rclcpp::SensorDataQoS().keep_last(1U),
            [weak_self, track_id = track.track_id](const sensor_msgs::msg::Image::ConstSharedPtr message) {
                if (const auto self = weak_self.lock()) {
                    self->handleImage(track_id, message);
                }
            }));
        camera_info_subscriptions_.push_back(create_subscription<sensor_msgs::msg::CameraInfo>(
            track.camera_info_topic, rclcpp::SensorDataQoS().keep_last(1U),
            [weak_self, track_id = track.track_id](const sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
                if (const auto self = weak_self.lock()) {
                    self->handleCameraInfo(track_id, message);
                }
            }));
    }
    return Status::success();
}

Status RtcNode::initializeAdapters() {
    auto transport_result = transport::WebRtcTransportFactory::create(config_);
    if (!transport_result.ok()) {
        return transport_result.status();
    }
    transport_ = transport_result.takeValue();
    signaling_adapter_ = std::make_unique<signaling::RosSignalingAdapter>(
        this, config_.topics.gateway_command, config_.topics.gateway_report, config_.signaling.max_payload_bytes);

    const std::weak_ptr<RtcNode> weak_self = std::static_pointer_cast<RtcNode>(shared_from_this());
    Status status = signaling_adapter_->setCommandHandler([weak_self](const std::string &payload) {
        const auto self = weak_self.lock();
        if (!self) {
            return Status::error(ErrorCode::kUnavailable, "rtc runtime is no longer available");
        }
        return self->handleSignaling(payload);
    });
    if (!status.ok()) {
        return status;
    }

    transport::WebRtcTransportCallbacks callbacks;
    callbacks.on_peer_event = [weak_self](const session::PeerDescriptor &peer) {
        if (const auto self = weak_self.lock()) {
            self->handlePeerEvent(peer);
        }
    };
    callbacks.on_data_channel_open = [weak_self](transport::DataChannelOpenRequest request) {
        if (const auto self = weak_self.lock()) {
            self->handleDataChannelOpen(std::move(request));
        }
    };
    callbacks.on_data_channel_ready = [weak_self](transport::DataChannelReadyEvent event) {
        if (const auto self = weak_self.lock()) {
            self->handleDataChannelReady(std::move(event));
        }
    };
    callbacks.on_data_channel_packet = [weak_self](protocol::DataChannelPacket packet) {
        if (const auto self = weak_self.lock()) {
            self->handleDataChannelPacket(std::move(packet));
        }
    };
    callbacks.on_signaling_report = [weak_self](std::string payload) {
        if (const auto self = weak_self.lock()) {
            self->handleSignalingReport(std::move(payload));
        }
    };

    status = transport_->start(std::move(callbacks));
    if (!status.ok()) {
        return status;
    }
    return signaling_adapter_->start();
}

void RtcNode::resetRosEndpoints() {
    peer_event_publisher_.reset();
    data_packet_publisher_.reset();
    diagnostics_publisher_.reset();
}

Status RtcNode::handleSignaling(const std::string &payload) {
    auto lease = acquireCallbackLease();
    if (!lease.has_value() || !transport_) {
        return Status::error(ErrorCode::kFailedPrecondition, "rtc runtime is stopped");
    }
    return transport_->handleSignaling(payload);
}

void RtcNode::handlePeerEvent(const session::PeerDescriptor &peer) {
    auto lease = acquireCallbackLease();
    if (!lease.has_value() || !session_registry_ || !transport_) {
        return;
    }

    const auto existing = session_registry_->findPeer(peer.session_id, peer.peer_id);
    Status status = Status::success();
    if (!existing.has_value()) {
        status = session_registry_->addPeer(peer);
    } else {
        status = session_registry_->updatePeer(peer);
    }
    if (!status.ok()) {
        RCLCPP_WARN(get_logger(), "reject RTC peer event: code=%d reason=%s", static_cast<int>(status.code()),
                    status.message().c_str());
        revokeAuthorizationState(peer.session_id, peer.peer_id, "");
        transport_->closePeer(peer.session_id, peer.peer_id, "", "peer_registry_rejected");
        return;
    }

    const bool terminal_state = peer.state == session::PeerState::kDisconnected ||
                                peer.state == session::PeerState::kFailed || peer.state == session::PeerState::kClosed;
    if (terminal_state) {
        // 先撤销 pending/active grant，再向应用发布断连，避免重连或复用 route 时短暂接受旧授权。
        revokeAuthorizationState(peer.session_id, peer.peer_id, "");
    } else if (existing.has_value()) {
        for (const auto &channel_label : existing->data_channels) {
            if (std::find(peer.data_channels.begin(), peer.data_channels.end(), channel_label) ==
                peer.data_channels.end()) {
                revokeAuthorizationState(peer.session_id, peer.peer_id, channel_label);
            }
        }
    }
    publishPeerEvent(peer);
    if (terminal_state) {
        session_registry_->removePeer(peer.session_id, peer.peer_id);
    }
}

void RtcNode::handleDataChannelOpen(transport::DataChannelOpenRequest request) {
    auto lease = acquireCallbackLease();
    if (!lease.has_value() || !authorization_client_ || !session_registry_ || !transport_) {
        return;
    }
    const session::DataChannelKey key{request.session_id, request.peer_id, request.channel_label};
    const auto peer = session_registry_->findPeer(request.session_id, request.peer_id);
    if (!peer.has_value() || request.channel_label.empty() || request.authorization_token.empty() ||
        peer->purpose != request.purpose || peer->run_id != request.run_id ||
        peer->resource_id != request.resource_id) {
        ++authorization_reject_count_;
        closeTransportChannel(key, "channel_identity_rejected");
        return;
    }
    if (!authorization_client_->service_is_ready()) {
        ++authorization_reject_count_;
        closeTransportChannel(key, "authorization_service_unavailable");
        return;
    }

    const auto authorization_client = authorization_client_;
    const std::string route_key = makeRouteKey(key);
    const std::uint64_t timeout_ns = static_cast<std::uint64_t>(config_.runtime.authorization_timeout_ms) * 1000000U;
    bool reservation_rejected = false;
    std::uint64_t authorization_generation = 0U;
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        if (pending_authorizations_.count(route_key) != 0U ||
            pending_authorizations_.size() >= config_.runtime.max_data_channels ||
            (data_channel_lifecycles_.count(route_key) == 0U &&
             data_channel_lifecycles_.size() >= config_.runtime.max_data_channels)) {
            reservation_rejected = true;
        } else {
            data_channel_lifecycles_.try_emplace(route_key, DataChannelLifecycle{key, false, false});
            authorization_generation = next_authorization_generation_++;
            if (next_authorization_generation_ == 0U) {
                next_authorization_generation_ = 1U;
            }
            pending_authorizations_.emplace(
                route_key,
                PendingAuthorization{key, -1, steadyNowNanoseconds() + timeout_ns, authorization_generation});
        }
    }
    if (reservation_rejected) {
        ++authorization_reject_count_;
        closeTransportChannel(key, "authorization_pending_limit");
        return;
    }

    auto service_request = std::make_shared<::astrabot_rtc::srv::AuthorizeDataChannel::Request>();
    service_request->session_id = request.session_id;
    service_request->peer_id = request.peer_id;
    service_request->purpose = request.purpose;
    service_request->run_id = request.run_id;
    service_request->resource_id = request.resource_id;
    service_request->channel_label = request.channel_label;
    service_request->authorization_token = std::move(request.authorization_token);

    const std::weak_ptr<RtcNode> weak_self = std::static_pointer_cast<RtcNode>(shared_from_this());
    auto future_and_id = authorization_client->async_send_request(
        service_request, [weak_self, route_key, authorization_generation](
                             rclcpp::Client<::astrabot_rtc::srv::AuthorizeDataChannel>::SharedFuture future) {
            if (const auto self = weak_self.lock()) {
                self->handleAuthorizationResponse(route_key, authorization_generation, std::move(future));
            }
        });
    bool request_still_pending = false;
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        const auto iterator = pending_authorizations_.find(route_key);
        if (iterator != pending_authorizations_.end()) {
            iterator->second.request_id = future_and_id.request_id;
            request_still_pending = true;
        }
    }
    if (!request_still_pending) {
        authorization_client->remove_pending_request(future_and_id.request_id);
    }
}

void RtcNode::handleDataChannelReady(transport::DataChannelReadyEvent event) {
    auto lease = acquireCallbackLease();
    if (!lease.has_value() || !session_registry_ || !data_channel_router_ || !transport_) {
        return;
    }
    const session::DataChannelKey key{event.session_id, event.peer_id, event.channel_label};
    const auto peer = session_registry_->findPeer(event.session_id, event.peer_id);
    if (!peer.has_value() || event.channel_label.empty() ||
        std::find(peer->data_channels.begin(), peer->data_channels.end(), event.channel_label) ==
            peer->data_channels.end()) {
        ++authorization_reject_count_;
        closeTransportChannel(key, "data_channel_open_identity_rejected");
        return;
    }

    bool publish_ready = false;
    bool capacity_rejected = false;
    const std::string route_key = makeRouteKey(key);
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        auto lifecycle = data_channel_lifecycles_.find(route_key);
        if (lifecycle == data_channel_lifecycles_.end()) {
            if (data_channel_lifecycles_.size() >= config_.runtime.max_data_channels) {
                capacity_rejected = true;
            } else {
                lifecycle = data_channel_lifecycles_.emplace(route_key, DataChannelLifecycle{key, true, false}).first;
            }
        } else {
            lifecycle->second.sdk_open = true;
        }
        if (!capacity_rejected && lifecycle != data_channel_lifecycles_.end() &&
            !lifecycle->second.ready_event_published && data_channel_router_->isAuthorized(key)) {
            lifecycle->second.ready_event_published = true;
            publish_ready = true;
        }
    }
    if (capacity_rejected) {
        ++authorization_reject_count_;
        closeTransportChannel(key, "data_channel_lifecycle_limit");
    } else if (publish_ready) {
        publishDataChannelReadyEvent(key);
    }
}

void RtcNode::handleDataChannelPacket(protocol::DataChannelPacket packet) {
    auto lease = acquireCallbackLease();
    if (!lease.has_value() || !data_channel_router_) {
        return;
    }
    data_channel_router_->pushIncoming(std::move(packet));
}

void RtcNode::handleSignalingReport(std::string payload) {
    auto lease = acquireCallbackLease();
    if (!lease.has_value() || !signaling_adapter_) {
        return;
    }
    const Status status = signaling_adapter_->publishReport(payload);
    if (!status.ok()) {
        RCLCPP_WARN(get_logger(), "drop RTC signaling report: code=%d reason=%s", static_cast<int>(status.code()),
                    status.message().c_str());
    }
}

void RtcNode::handleAuthorizationResponse(
    const std::string &route_key, std::uint64_t generation,
    rclcpp::Client<::astrabot_rtc::srv::AuthorizeDataChannel>::SharedFuture future) {
    auto lease = acquireCallbackLease();
    if (!lease.has_value() || !data_channel_router_) {
        return;
    }
    const auto response = future.get();
    PendingAuthorization pending;
    std::string rejection_reason;
    bool publish_ready = false;
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        const auto iterator = pending_authorizations_.find(route_key);
        if (iterator == pending_authorizations_.end() || iterator->second.generation != generation) {
            return;
        }
        pending = iterator->second;
        if (pending.deadline_steady_ns <= steadyNowNanoseconds()) {
            rejection_reason = "authorization_timeout";
        } else if (!response) {
            rejection_reason = "authorization_empty_response";
        } else if (!response->allowed) {
            rejection_reason = "authorization_denied";
        } else {
            const session::ChannelAuthorization authorization{true, response->reason_code, response->expires_at};
            const Status status = data_channel_router_->authorize(pending.key, authorization);
            if (!status.ok()) {
                rejection_reason = "authorization_result_rejected";
            } else {
                auto lifecycle = data_channel_lifecycles_.find(route_key);
                if (lifecycle == data_channel_lifecycles_.end()) {
                    if (data_channel_lifecycles_.size() >= config_.runtime.max_data_channels) {
                        rejection_reason = "data_channel_lifecycle_limit";
                        static_cast<void>(data_channel_router_->revoke(pending.key));
                    } else {
                        lifecycle =
                            data_channel_lifecycles_.emplace(route_key, DataChannelLifecycle{pending.key, false, false})
                                .first;
                    }
                }
                if (rejection_reason.empty() && lifecycle->second.sdk_open &&
                    !lifecycle->second.ready_event_published && data_channel_router_->isAuthorized(pending.key)) {
                    lifecycle->second.ready_event_published = true;
                    publish_ready = true;
                }
            }
        }
        pending_authorizations_.erase(iterator);
        if (!rejection_reason.empty()) {
            data_channel_lifecycles_.erase(route_key);
        }
    }
    if (!rejection_reason.empty()) {
        ++authorization_reject_count_;
        closeTransportChannel(pending.key, rejection_reason);
    } else if (publish_ready) {
        publishDataChannelReadyEvent(pending.key);
    }
}

void RtcNode::handleClosePeerRequest(const ::astrabot_rtc::srv::CloseRtcPeer::Request::SharedPtr request,
                                     ::astrabot_rtc::srv::CloseRtcPeer::Response::SharedPtr response) {
    auto lease = acquireCallbackLease();
    if (!lease.has_value() || !transport_ || !data_channel_router_) {
        response->closed = false;
        response->reason_code = "rtc_runtime_stopped";
        return;
    }
    revokeAuthorizationState(request->session_id, request->peer_id, request->channel_label);
    const Status status =
        transport_->closePeer(request->session_id, request->peer_id, request->channel_label, request->reason_code);
    response->closed = status.ok();
    response->reason_code = status.ok() ? "closed" : status.message();
}

void RtcNode::dispatchLatestPackets() {
    auto lease = acquireCallbackLease();
    if (!lease.has_value() || !data_channel_router_ || !data_packet_publisher_) {
        return;
    }
    auto packets = data_channel_router_->takeAllLatest();
    for (auto &packet : packets) {
        ::astrabot_rtc::msg::RtcDataPacket message;
        message.session_id = std::move(packet.session_id);
        message.peer_id = std::move(packet.peer_id);
        message.channel_label = std::move(packet.channel_label);
        message.receive_steady_time_ns = packet.receive_steady_time_ns;
        message.payload = std::move(packet.payload);
        data_packet_publisher_->publish(message);
    }
}

void RtcNode::expirePendingAuthorizations() {
    auto lease = acquireCallbackLease();
    if (!lease.has_value() || !authorization_client_) {
        return;
    }
    const std::uint64_t now_ns = steadyNowNanoseconds();
    std::vector<PendingAuthorization> expired;
    {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        for (auto iterator = pending_authorizations_.begin(); iterator != pending_authorizations_.end();) {
            if (iterator->second.deadline_steady_ns <= now_ns) {
                expired.push_back(iterator->second);
                data_channel_lifecycles_.erase(iterator->first);
                iterator = pending_authorizations_.erase(iterator);
            } else {
                ++iterator;
            }
        }
    }
    for (const auto &pending : expired) {
        if (pending.request_id >= 0) {
            authorization_client_->remove_pending_request(pending.request_id);
        }
        ++authorization_reject_count_;
        closeTransportChannel(pending.key, "authorization_timeout");
    }
}

void RtcNode::publishDiagnostics() {
    auto lease = acquireCallbackLease();
    if (!lease.has_value() || !diagnostics_publisher_ || !transport_ || !session_registry_ || !data_channel_router_) {
        return;
    }
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "astrabot_rtc/runtime";
    status.hardware_id = "thor";
    const auto capabilities = transport_->capabilities();
    const bool fully_available =
        capabilities.peer_connections && capabilities.media_tracks && capabilities.data_channels;
    status.level =
        fully_available ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = fully_available ? "rtc transport available" : "rtc transport disabled or incomplete";
    const auto router_metrics = data_channel_router_->metrics();
    const auto transport_metrics = transport_->metrics();
    status.values = {
        makeDiagnosticValue("backend", capabilities.backend),
        makeDiagnosticValue("peer_connections", capabilities.peer_connections ? "true" : "false"),
        makeDiagnosticValue("media_tracks", capabilities.media_tracks ? "true" : "false"),
        makeDiagnosticValue("data_channels", capabilities.data_channels ? "true" : "false"),
        makeDiagnosticValue("peer_count", std::to_string(session_registry_->peerCount())),
        makeDiagnosticValue("viewer_count", std::to_string(session_registry_->viewerCount())),
        makeDiagnosticValue("media_peer_count", std::to_string(session_registry_->viewerCount())),
        makeDiagnosticValue("authorized_channel_count", std::to_string(data_channel_router_->authorizedChannelCount())),
        makeDiagnosticValue("camera_frame_count", std::to_string(camera_frame_count_.load())),
        makeDiagnosticValue("camera_frame_reject_count", std::to_string(camera_frame_reject_count_.load())),
        makeDiagnosticValue("camera_info_count", std::to_string(camera_info_count_.load())),
        makeDiagnosticValue("authorization_reject_count", std::to_string(authorization_reject_count_.load())),
        makeDiagnosticValue("data_accepted", std::to_string(router_metrics.accepted_packets)),
        makeDiagnosticValue("data_overwritten", std::to_string(router_metrics.overwritten_packets)),
        makeDiagnosticValue("data_unauthorized", std::to_string(router_metrics.unauthorized_packets)),
        makeDiagnosticValue("data_oversized", std::to_string(router_metrics.oversized_packets)),
        makeDiagnosticValue("data_expired_queued", std::to_string(router_metrics.expired_queued_packets)),
        makeDiagnosticValue("media_peer_sends", std::to_string(transport_metrics.media_peer_sends)),
        makeDiagnosticValue("media_peer_bytes", std::to_string(transport_metrics.media_peer_bytes)),
        makeDiagnosticValue("media_teleop_congestion_drops",
                            std::to_string(transport_metrics.media_teleop_congestion_drops)),
        makeDiagnosticValue("media_viewer_congestion_drops",
                            std::to_string(transport_metrics.media_viewer_congestion_drops)),
        makeDiagnosticValue("media_buffer_query_failures",
                            std::to_string(transport_metrics.media_buffer_query_failures)),
        makeDiagnosticValue("reconnect_count", std::to_string(transport_metrics.reconnect_count)),
        makeDiagnosticValue("network_stats_available", "false"),
        makeDiagnosticValue("network_rtt_ms", "unsupported_libdatachannel_c_api"),
        makeDiagnosticValue("network_packet_loss_percent", "unsupported_libdatachannel_c_api"),
        makeDiagnosticValue("network_jitter_ms", "unsupported_libdatachannel_c_api"),
    };
    std::uint64_t media_submitted = 0U;
    std::uint64_t media_overwritten = 0U;
    std::uint64_t media_encoded = 0U;
    std::uint64_t media_encoded_bytes = 0U;
    std::uint64_t media_encode_failures = 0U;
    std::uint64_t media_sink_failures = 0U;
    std::uint32_t active_frame_rate = 0U;
    for (const auto &entry : encoder_workers_) {
        const auto metrics = entry.second->metrics();
        media_submitted += metrics.submitted_frames;
        media_overwritten += metrics.overwritten_frames;
        media_encoded += metrics.encoded_frames;
        media_encoded_bytes += metrics.encoded_bytes;
        media_encode_failures += metrics.encode_failures;
        media_sink_failures += metrics.sink_failures;
        const std::uint32_t worker_frame_rate = entry.second->activeFrameRate();
        active_frame_rate =
            active_frame_rate == 0U ? worker_frame_rate : std::min(active_frame_rate, worker_frame_rate);
    }
    status.values.push_back(makeDiagnosticValue("media_encoder_name", config_.media.encoder.encoder_name));
    status.values.push_back(makeDiagnosticValue("media_active_frame_rate", std::to_string(active_frame_rate)));
    status.values.push_back(makeDiagnosticValue("media_submitted", std::to_string(media_submitted)));
    status.values.push_back(makeDiagnosticValue("media_overwritten", std::to_string(media_overwritten)));
    status.values.push_back(makeDiagnosticValue("media_encoded", std::to_string(media_encoded)));
    status.values.push_back(makeDiagnosticValue("media_encoded_bytes", std::to_string(media_encoded_bytes)));
    status.values.push_back(makeDiagnosticValue("media_encode_failures", std::to_string(media_encode_failures)));
    status.values.push_back(makeDiagnosticValue("media_sink_failures", std::to_string(media_sink_failures)));
    const MediaRateWindow rate_window =
        media_rate_tracker_.update(steadyNowNanoseconds(), MediaCumulativeCounters{media_encoded, media_encoded_bytes,
                                                                                   transport_metrics.media_peer_sends,
                                                                                   transport_metrics.media_peer_bytes});
    status.values.push_back(
        makeDiagnosticValue("media_rate_window_available", rate_window.available ? "true" : "false"));
    status.values.push_back(makeDiagnosticValue("media_rate_window_ms", std::to_string(rate_window.duration_ms)));
    status.values.push_back(makeDiagnosticValue("media_encoded_fps_window", std::to_string(rate_window.encoded_fps)));
    status.values.push_back(
        makeDiagnosticValue("media_encoded_bitrate_bps_window", std::to_string(rate_window.encoded_bitrate_bps)));
    status.values.push_back(
        makeDiagnosticValue("media_peer_send_fps_window", std::to_string(rate_window.peer_send_fps)));
    status.values.push_back(
        makeDiagnosticValue("media_peer_send_bitrate_bps_window", std::to_string(rate_window.peer_send_bitrate_bps)));
    array.status.push_back(std::move(status));
    diagnostics_publisher_->publish(array);
}

void RtcNode::handleImage(const std::string &track_id, const sensor_msgs::msg::Image::ConstSharedPtr message) {
    auto lease = acquireCallbackLease();
    if (!lease.has_value() || !message) {
        return;
    }
    const auto worker = encoder_workers_.find(track_id);
    if (worker == encoder_workers_.end()) {
        ++camera_frame_reject_count_;
        return;
    }
    auto data = std::make_shared<std::vector<std::uint8_t>>(message->data.begin(), message->data.end());
    auto frame = std::make_shared<media::RawVideoFrame>();
    frame->track_id = track_id;
    frame->capture_time_ns = imageStampNanoseconds(message->header.stamp);
    frame->width = message->width;
    frame->height = message->height;
    frame->row_step = message->step;
    frame->encoding = message->encoding;
    frame->data = std::move(data);
    if (worker->second->submit(std::move(frame)).ok()) {
        ++camera_frame_count_;
    } else {
        ++camera_frame_reject_count_;
    }
}

void RtcNode::handleCameraInfo(const std::string &track_id,
                               const sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
    auto lease = acquireCallbackLease();
    if (!lease.has_value() || !message || encoder_workers_.count(track_id) == 0U) {
        return;
    }
    ++camera_info_count_;
}

void RtcNode::revokeAuthorizationState(const std::string &session_id, const std::string &peer_id,
                                       const std::string &channel_label) {
    std::vector<std::int64_t> request_ids;
    {
        // 与授权响应共用同一把锁，并在锁内撤销 router，保证“响应授权”和“peer 关闭”有确定先后顺序。
        std::lock_guard<std::mutex> lock(pending_mutex_);
        for (auto iterator = pending_authorizations_.begin(); iterator != pending_authorizations_.end();) {
            const auto &key = iterator->second.key;
            const bool route_matches = key.session_id == session_id && key.peer_id == peer_id &&
                                       (channel_label.empty() || key.channel_label == channel_label);
            if (!route_matches) {
                ++iterator;
                continue;
            }
            if (iterator->second.request_id >= 0) {
                request_ids.push_back(iterator->second.request_id);
            }
            iterator = pending_authorizations_.erase(iterator);
        }
        if (data_channel_router_) {
            if (channel_label.empty()) {
                data_channel_router_->revokePeer(session_id, peer_id);
            } else {
                const session::DataChannelKey key{session_id, peer_id, channel_label};
                data_channel_router_->revoke(key);
            }
        }
        for (auto iterator = data_channel_lifecycles_.begin(); iterator != data_channel_lifecycles_.end();) {
            const auto &key = iterator->second.key;
            const bool route_matches = key.session_id == session_id && key.peer_id == peer_id &&
                                       (channel_label.empty() || key.channel_label == channel_label);
            if (route_matches) {
                iterator = data_channel_lifecycles_.erase(iterator);
            } else {
                ++iterator;
            }
        }
    }

    const auto authorization_client = authorization_client_;
    if (authorization_client) {
        for (const std::int64_t request_id : request_ids) {
            authorization_client->remove_pending_request(request_id);
        }
    }
}

void RtcNode::closeTransportChannel(const session::DataChannelKey &key, const std::string &reason_code) {
    if (transport_) {
        transport_->closePeer(key.session_id, key.peer_id, key.channel_label, reason_code);
    }
}

void RtcNode::publishDataChannelReadyEvent(const session::DataChannelKey &key) {
    if (!session_registry_) {
        return;
    }
    const auto peer = session_registry_->findPeer(key.session_id, key.peer_id);
    if (!peer.has_value() || std::find(peer->data_channels.begin(), peer->data_channels.end(), key.channel_label) ==
                                 peer->data_channels.end()) {
        return;
    }
    const std::uint64_t now_ns = steadyNowNanoseconds();
    const Status status = session_registry_->updatePeerState(
        key.session_id, key.peer_id, session::PeerState::kConnected, now_ns, "data_channel_open");
    if (!status.ok()) {
        return;
    }
    const auto ready_peer = session_registry_->findPeer(key.session_id, key.peer_id);
    if (ready_peer.has_value()) {
        publishPeerEvent(*ready_peer);
    }
}

void RtcNode::publishPeerEvent(const session::PeerDescriptor &peer) {
    if (!peer_event_publisher_) {
        return;
    }
    ::astrabot_rtc::msg::RtcPeerEvent message;
    message.state = static_cast<std::uint8_t>(peer.state);
    message.session_id = peer.session_id;
    message.peer_id = peer.peer_id;
    message.purpose = peer.purpose;
    message.run_id = peer.run_id;
    message.resource_id = peer.resource_id;
    message.media_tracks = peer.media_tracks;
    message.data_channels = peer.data_channels;
    message.steady_time_ns = peer.steady_time_ns;
    message.reason_code = peer.reason_code;
    peer_event_publisher_->publish(message);
}

std::optional<RtcNode::CallbackLease> RtcNode::acquireCallbackLease() {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (!running_.load()) {
        return std::nullopt;
    }
    ++active_callbacks_;
    CallbackLease lease(this);
    return std::optional<CallbackLease>(std::move(lease));
}

void RtcNode::releaseCallbackLease() {
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        --active_callbacks_;
    }
    callbacks_drained_.notify_all();
}

std::string RtcNode::makeRouteKey(const session::DataChannelKey &key) {
    std::string route_key;
    route_key.reserve(key.session_id.size() + key.peer_id.size() + key.channel_label.size() + 32U);
    for (const auto *part : {&key.session_id, &key.peer_id, &key.channel_label}) {
        route_key += std::to_string(part->size());
        route_key.push_back(':');
        route_key += *part;
    }
    return route_key;
}

std::uint64_t RtcNode::steadyNowNanoseconds() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

std::uint64_t RtcNode::imageStampNanoseconds(const builtin_interfaces::msg::Time &stamp) {
    if (stamp.sec < 0) {
        return 0U;
    }
    const std::uint64_t seconds = static_cast<std::uint64_t>(stamp.sec);
    if (seconds > std::numeric_limits<std::uint64_t>::max() / 1000000000U) {
        return 0U;
    }
    return seconds * 1000000000U + static_cast<std::uint64_t>(stamp.nanosec);
}

}  // namespace astrabot::rtc::runtime
