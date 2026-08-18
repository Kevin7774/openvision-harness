// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "astrabot_rtc/msg/rtc_data_packet.hpp"
#include "astrabot_rtc/srv/authorize_data_channel.hpp"
#include "astrabot_teleop/msg/teleop_command.hpp"
#include "astrabot_teleop/runtime/teleop_node.h"

namespace astrabot::teleop {
namespace {

using namespace std::chrono_literals;

class RclcppContextGuard final {
  public:
    RclcppContextGuard() {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    ~RclcppContextGuard() {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }
};

class ExecutorSpinGuard final {
  public:
    explicit ExecutorSpinGuard(rclcpp::Executor *executor)
        : executor_(executor), spin_thread_([executor]() { executor->spin(); }) {}

    ~ExecutorSpinGuard() {
        executor_->cancel();
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
    }

  private:
    rclcpp::Executor *executor_;
    std::thread spin_thread_;
};

TEST(TeleopNodeRosIntegrationTest, DisabledBackendFailsClosedAndNeverPublishesMotion) {
    RclcppContextGuard context;
    const std::string suffix = "n" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    const std::string authorize_service = "/test/teleop/authorize/" + suffix;
    const std::string data_topic = "/test/rtc/data/" + suffix;
    const std::string shadow_topic = "/test/teleop/shadow/" + suffix;
    const std::string production_topic = "/test/teleop/production/" + suffix;

    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("backend", "disabled"),
        rclcpp::Parameter("authorize_service", authorize_service),
        rclcpp::Parameter("rtc_data_topic", data_topic),
        rclcpp::Parameter("shadow_command_topic", shadow_topic),
        rclcpp::Parameter("production_command_topic", production_topic),
        rclcpp::Parameter("rtc_peer_event_topic", "/test/rtc/peer/" + suffix),
        rclcpp::Parameter("rtc_close_service", "/test/rtc/close/" + suffix),
        rclcpp::Parameter("status_topic", "/test/teleop/status/" + suffix),
        rclcpp::Parameter("report_teleop_status_service", "/test/data/status/" + suffix),
    });

    auto teleop_node = std::make_shared<TeleopNode>(options);
    ASSERT_TRUE(teleop_node->start().ok());
    EXPECT_TRUE(teleop_node->start().ok());

    auto test_node = std::make_shared<rclcpp::Node>("teleop_ros_integration_test_" + suffix);
    auto authorize_client = test_node->create_client<astrabot_rtc::srv::AuthorizeDataChannel>(authorize_service);
    auto data_publisher =
        test_node->create_publisher<astrabot_rtc::msg::RtcDataPacket>(data_topic, rclcpp::QoS(1U).best_effort());
    std::atomic<std::size_t> command_count{0U};
    auto shadow_subscription = test_node->create_subscription<::astrabot_teleop::msg::TeleopCommand>(
        shadow_topic, rclcpp::QoS(1U).best_effort(),
        [&command_count](::astrabot_teleop::msg::TeleopCommand::ConstSharedPtr) { ++command_count; });
    auto production_subscription = test_node->create_subscription<::astrabot_teleop::msg::TeleopCommand>(
        production_topic, rclcpp::QoS(1U).best_effort(),
        [&command_count](::astrabot_teleop::msg::TeleopCommand::ConstSharedPtr) { ++command_count; });

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2U);
    executor.add_node(teleop_node);
    executor.add_node(test_node);
    ExecutorSpinGuard spin_guard(&executor);

    ASSERT_TRUE(authorize_client->wait_for_service(2s));
    auto request = std::make_shared<astrabot_rtc::srv::AuthorizeDataChannel::Request>();
    request->session_id = "session-disabled";
    request->peer_id = "quest-disabled";
    request->purpose = "teleop";
    request->run_id = "run-disabled";
    request->resource_id = "thor";
    request->channel_label = "astrabot.teleop";
    request->authorization_token = "must-not-be-logged";
    auto response_future = authorize_client->async_send_request(request);
    ASSERT_EQ(response_future.wait_for(2s), std::future_status::ready);
    const auto response = response_future.get();
    EXPECT_FALSE(response->allowed);
    EXPECT_EQ(response->reason_code, "teleop_disabled");

    astrabot_rtc::msg::RtcDataPacket packet;
    packet.session_id = request->session_id;
    packet.peer_id = request->peer_id;
    packet.channel_label = request->channel_label;
    packet.receive_steady_time_ns = 1U;
    packet.payload = {1U, 2U, 3U};
    data_publisher->publish(packet);
    std::this_thread::sleep_for(100ms);
    EXPECT_EQ(command_count.load(), 0U);

    executor.remove_node(teleop_node);
    teleop_node->stop();
    teleop_node->stop();
    EXPECT_EQ(teleop_node->start().code(), ErrorCode::kFailedPrecondition);

    (void)shadow_subscription;
    (void)production_subscription;
}

}  // namespace
}  // namespace astrabot::teleop
