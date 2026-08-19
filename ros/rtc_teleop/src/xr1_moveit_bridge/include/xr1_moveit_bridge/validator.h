#pragma once

#include <iosfwd>

namespace xr1_moveit_bridge {

/**
 * @brief Validate one batch of named joint candidates against a MoveIt PlanningScene.
 *
 * The input and output are schema-versioned JSON streams. This function never
 * publishes a ROS command and owns no hardware resources.
 *
 * @return Zero on success; non-zero after writing a structured error.
 */
int runValidator(std::istream &input, std::ostream &output, std::ostream &error_output);

}  // namespace xr1_moveit_bridge
