#include "astrabot_rtc/signaling/ros_signaling_adapter.h"

#include <utility>

namespace astrabot::rtc::signaling {

RosSignalingAdapter::RosSignalingAdapter(rclcpp::Node *node, std::string command_topic, std::string report_topic,
                                         std::size_t max_payload_bytes)
    : node_(node), command_topic_(std::move(command_topic)), report_topic_(std::move(report_topic)),
      max_payload_bytes_(max_payload_bytes) {}

Status RosSignalingAdapter::setCommandHandler(CommandHandler handler) {
    if (!handler) {
        return Status::error(ErrorCode::kInvalidArgument, "signaling command handler is empty");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) {
        return Status::error(ErrorCode::kFailedPrecondition, "signaling handler cannot change while running");
    }
    command_handler_ = std::move(handler);
    return Status::success();
}

Status RosSignalingAdapter::start() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) {
        return Status::success();
    }
    if (node_ == nullptr || !command_handler_) {
        return Status::error(ErrorCode::kFailedPrecondition, "signaling adapter is missing node or handler");
    }
    report_publisher_ = node_->create_publisher<std_msgs::msg::String>(report_topic_, rclcpp::QoS(32U).reliable());
    command_subscription_ = node_->create_subscription<std_msgs::msg::String>(
        command_topic_, rclcpp::QoS(32U).reliable(),
        [this](const std_msgs::msg::String::SharedPtr message) { handleCommand(message); });
    running_ = true;
    return Status::success();
}

void RosSignalingAdapter::stop() {
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        subscription = std::move(command_subscription_);
    }
    subscription.reset();

    std::unique_lock<std::mutex> lock(mutex_);
    callbacks_drained_.wait(lock, [this]() { return active_callbacks_ == 0U; });
    report_publisher_.reset();
}

Status RosSignalingAdapter::publishReport(const std::string &payload) {
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_ || !report_publisher_) {
            return Status::error(ErrorCode::kFailedPrecondition, "signaling adapter is stopped");
        }
        if (payload.size() > max_payload_bytes_) {
            return Status::error(ErrorCode::kPayloadTooLarge, "signaling report exceeds the configured limit");
        }
        publisher = report_publisher_;
    }
    std_msgs::msg::String message;
    message.data = payload;
    publisher->publish(message);
    return Status::success();
}

void RosSignalingAdapter::handleCommand(const std_msgs::msg::String::SharedPtr message) {
    CommandHandler handler;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_ || !command_handler_ || !message) {
            return;
        }
        if (message->data.size() > max_payload_bytes_) {
            RCLCPP_WARN(node_->get_logger(), "drop oversized RTC signaling payload: bytes=%zu", message->data.size());
            return;
        }
        handler = command_handler_;
        ++active_callbacks_;
    }
    const Status status = handler(message->data);
    if (!status.ok()) {
        RCLCPP_WARN(node_->get_logger(), "RTC signaling rejected: code=%d reason=%s", static_cast<int>(status.code()),
                    status.message().c_str());
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        --active_callbacks_;
    }
    callbacks_drained_.notify_all();
}

}  // namespace astrabot::rtc::signaling
