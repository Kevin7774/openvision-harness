// Copyright 2026 Astrabot Team
// SPDX-License-Identifier: MIT

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "astrabot_teleop/runtime/teleop_node.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<astrabot::teleop::TeleopNode>();
    const astrabot::teleop::Status status = node->start();
    if (!status.ok()) {
        RCLCPP_ERROR(node->get_logger(), "failed to start astrabot_teleop: code=%d reason=%s",
                     static_cast<int>(status.code()), status.message().c_str());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::spin(node);
    node->stop();
    rclcpp::shutdown();
    return 0;
}
