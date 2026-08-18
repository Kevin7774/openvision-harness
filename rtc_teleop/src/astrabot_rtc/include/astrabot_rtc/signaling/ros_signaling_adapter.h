#pragma once

#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "astrabot_rtc/signaling/signaling_adapter.h"

namespace astrabot::rtc::signaling {

/**
 * @brief 使用现有 Gateway std_msgs/String topic 契约的 ROS 2 adapter。
 *
 * 本对象不拥有 node；调用方必须保证 node 生命周期覆盖本对象及 stop 调用。
 */
class RosSignalingAdapter final : public ISignalingAdapter {
  public:
    RosSignalingAdapter(rclcpp::Node *node, std::string command_topic, std::string report_topic,
                        std::size_t max_payload_bytes);

    Status setCommandHandler(CommandHandler handler) override;
    Status start() override;
    void stop() override;
    Status publishReport(const std::string &payload) override;

  private:
    void handleCommand(const std_msgs::msg::String::SharedPtr message);

    rclcpp::Node *const node_;
    const std::string command_topic_;
    const std::string report_topic_;
    const std::size_t max_payload_bytes_;
    mutable std::mutex mutex_;
    std::condition_variable callbacks_drained_;
    bool running_{false};
    std::size_t active_callbacks_{0U};
    CommandHandler command_handler_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr report_publisher_;
};

}  // namespace astrabot::rtc::signaling
