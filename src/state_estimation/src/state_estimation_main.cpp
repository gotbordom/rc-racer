// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "state_estimation/state_estimation_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<state_estimation::StateEstimationNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
