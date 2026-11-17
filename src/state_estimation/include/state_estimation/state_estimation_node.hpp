// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

// StateEstimationNode (VEH.GNC.SE spec §3): thin lifecycle adapter around
// StateEstimationLogic. It reads parameters, converts messages, runs the
// predict timer and publishes /odom, TF and /diagnostics. No filter math.

#ifndef STATE_ESTIMATION__STATE_ESTIMATION_NODE_HPP_
#define STATE_ESTIMATION__STATE_ESTIMATION_NODE_HPP_

#include <Eigen/Geometry>

#include <memory>
#include <optional>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "state_estimation/state_estimation_logic.hpp"

namespace state_estimation
{

class StateEstimationNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit StateEstimationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void declare_parameters();
  StateEstimationConfig read_config();

  void on_timer();
  void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr & msg);
  void on_wheel_odom(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void on_reset(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void report_health(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /// IMU mount (TF base_footprint -> IMU frame), looked up once (static).
  struct ImuMount
  {
    Eigen::Quaterniond rotation;
    double forward_offset;  ///< m, translation x
  };
  std::optional<ImuMount> imu_mount(const std::string & imu_frame);

  void release();

  // Configured
  std::unique_ptr<StateEstimationLogic> logic_;
  double publish_rate_{50.0};
  bool publish_tf_{true};
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;

  // Active
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<rclcpp::Time> last_tick_;

  // IMU mount, cached per frame name
  std::string imu_frame_;
  std::optional<ImuMount> base_from_imu_;
  bool imu_mount_known_{true};

  // Counters at the previous diagnostics report
  int reported_resets_{0};
  int reported_rejects_{0};
};

}  // namespace state_estimation

#endif  // STATE_ESTIMATION__STATE_ESTIMATION_NODE_HPP_
