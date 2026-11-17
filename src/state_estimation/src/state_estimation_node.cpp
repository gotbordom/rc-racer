// SPDX-FileCopyrightText: 2025 Anthony Tracy
// SPDX-License-Identifier: MIT

#include "state_estimation/state_estimation_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2/exceptions.h"

#include "state_estimation/message_conversions.hpp"

namespace state_estimation
{

using std::placeholders::_1;
using std::placeholders::_2;

namespace
{

constexpr char kImuTopic[] = "/imu";
constexpr char kWheelOdomTopic[] = "/wheel/odom";
constexpr char kOdomTopic[] = "/odom";

}  // namespace

StateEstimationNode::StateEstimationNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("state_estimation", options)
{
  declare_parameters();
}

void StateEstimationNode::declare_parameters()
{
  // Names and defaults: spec §3.4. Defaults come from StateEstimationConfig
  // so there is one source for them.
  const StateEstimationConfig d;
  declare_parameter("publish_rate", 50.0);
  declare_parameter("publish_tf", true);
  declare_parameter("motion_model", d.motion_model);
  declare_parameter("fusion.method", d.fusion_method);
  declare_parameter("robot.wheelbase", d.robot.wheelbase);
  declare_parameter("robot.max_steering_angle", d.robot.max_steering_angle);
  declare_parameter("robot.rear_axle_offset", d.robot.rear_axle_offset);
  declare_parameter("inputs.imu.timeout", d.imu_timeout);
  declare_parameter("inputs.wheel_odom.timeout", d.wheel_odom_timeout);
  declare_parameter("process_noise.accel", d.accel_noise_density);
  declare_parameter("process_noise.accel_no_imu", d.accel_noise_density_no_imu);
  declare_parameter("process_noise.yaw_accel", d.yaw_accel_noise_density);
  declare_parameter("slip.threshold", d.slip_threshold);
  declare_parameter("slip.yaw_rate_margin", d.slip_yaw_rate_margin);
  declare_parameter("slip.variance_scale", d.slip_variance_scale);
  declare_parameter("initial_covariance.pose", d.initial_pose_variance);
  declare_parameter("initial_covariance.speed", d.initial_speed_variance);
  declare_parameter("initial_covariance.yaw_rate", d.initial_yaw_rate_variance);
}

StateEstimationConfig StateEstimationNode::read_config()
{
  StateEstimationConfig c;
  c.motion_model = get_parameter("motion_model").as_string();
  c.fusion_method = get_parameter("fusion.method").as_string();
  c.robot.wheelbase = get_parameter("robot.wheelbase").as_double();
  c.robot.max_steering_angle = get_parameter("robot.max_steering_angle").as_double();
  c.robot.rear_axle_offset = get_parameter("robot.rear_axle_offset").as_double();
  c.imu_timeout = get_parameter("inputs.imu.timeout").as_double();
  c.wheel_odom_timeout = get_parameter("inputs.wheel_odom.timeout").as_double();
  c.accel_noise_density = get_parameter("process_noise.accel").as_double();
  c.accel_noise_density_no_imu = get_parameter("process_noise.accel_no_imu").as_double();
  c.yaw_accel_noise_density = get_parameter("process_noise.yaw_accel").as_double();
  c.slip_threshold = get_parameter("slip.threshold").as_double();
  c.slip_yaw_rate_margin = get_parameter("slip.yaw_rate_margin").as_double();
  c.slip_variance_scale = get_parameter("slip.variance_scale").as_double();
  c.initial_pose_variance = get_parameter("initial_covariance.pose").as_double();
  c.initial_speed_variance = get_parameter("initial_covariance.speed").as_double();
  c.initial_yaw_rate_variance = get_parameter("initial_covariance.yaw_rate").as_double();
  return c;
}

StateEstimationNode::CallbackReturn StateEstimationNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  // Validate everything here, not at runtime (Conventions §7).
  try {
    publish_rate_ = get_parameter("publish_rate").as_double();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    if (!(publish_rate_ > 0.0)) {
      throw std::invalid_argument("publish_rate must be positive");
    }
    logic_ = std::make_unique<StateEstimationLogic>(read_config());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Invalid configuration: %s", e.what());
    logic_.reset();
    return CallbackReturn::FAILURE;
  }

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(kOdomTopic, rclcpp::QoS(10).reliable());
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  // Spun by this node's executor; no extra node or thread.
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
  reset_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/reset", std::bind(&StateEstimationNode::on_reset, this, _1, _2));

  diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
  diagnostics_->setHardwareID("none");
  diagnostics_->add("state_estimation", this, &StateEstimationNode::report_health);

  RCLCPP_INFO(
    get_logger(), "Configured: motion_model=%s, fusion.method=%s, %.0f Hz%s",
    logic_->config().motion_model.c_str(), logic_->config().fusion_method.c_str(),
    publish_rate_, publish_tf_ ? ", publishing TF" : "");
  return CallbackReturn::SUCCESS;
}

StateEstimationNode::CallbackReturn StateEstimationNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  // odom starts where the car is on activation (Conventions §2.1).
  logic_->reset();
  reported_resets_ = 0;
  reported_rejects_ = 0;
  last_tick_.reset();
  odom_pub_->on_activate();

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    kImuTopic, rclcpp::SensorDataQoS(), std::bind(&StateEstimationNode::on_imu, this, _1));
  wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    kWheelOdomTopic, rclcpp::SensorDataQoS(),
    std::bind(&StateEstimationNode::on_wheel_odom, this, _1));

  // Node clock, so the rate and the filter's dt follow sim time when
  // use_sim_time is set (Conventions §3).
  const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  timer_ = rclcpp::create_timer(
    this, get_clock(),
    rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(period)),
    std::bind(&StateEstimationNode::on_timer, this));
  return CallbackReturn::SUCCESS;
}

StateEstimationNode::CallbackReturn StateEstimationNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  timer_.reset();
  imu_sub_.reset();
  wheel_sub_.reset();
  odom_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

StateEstimationNode::CallbackReturn StateEstimationNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  release();
  return CallbackReturn::SUCCESS;
}

StateEstimationNode::CallbackReturn StateEstimationNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  release();
  return CallbackReturn::SUCCESS;
}

void StateEstimationNode::release()
{
  timer_.reset();
  imu_sub_.reset();
  wheel_sub_.reset();
  diagnostics_.reset();
  reset_srv_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  tf_broadcaster_.reset();
  odom_pub_.reset();
  logic_.reset();
  base_from_imu_.reset();
  imu_frame_.clear();
  imu_mount_known_ = true;
}

void StateEstimationNode::on_timer()
{
  const rclcpp::Time now = get_clock()->now();
  if (last_tick_) {
    const double dt = (now - *last_tick_).seconds();
    if (dt < 0.0) {
      // Clock went backwards (e.g. the simulator restarted): skip the step.
      RCLCPP_WARN(get_logger(), "Clock jumped back %.3f s; skipping predict", -dt);
    } else {
      logic_->predict(dt);
    }
  }
  last_tick_ = now;

  odom_pub_->publish(to_odometry(logic_->state(), now));
  if (tf_broadcaster_) {
    tf_broadcaster_->sendTransform(to_transform(logic_->state(), now));
  }
}

void StateEstimationNode::on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr & msg)
{
  const std::optional<Eigen::Quaterniond> mount = imu_mount(msg->header.frame_id);
  if (!mount) {
    return;
  }
  logic_->update_imu(to_imu_measurement(*msg, *mount));
}

void StateEstimationNode::on_wheel_odom(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  logic_->update_wheel_speed(to_wheel_speed(*msg));
}

std::optional<Eigen::Quaterniond> StateEstimationNode::imu_mount(const std::string & imu_frame)
{
  if (base_from_imu_ && imu_frame == imu_frame_) {
    return base_from_imu_;
  }
  try {
    // The mount is static (URDF), so the latest transform is the transform.
    const auto tf = tf_buffer_->lookupTransform(kBaseFrame, imu_frame, tf2::TimePointZero);
    const auto & q = tf.transform.rotation;
    base_from_imu_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
    imu_frame_ = imu_frame;
    imu_mount_known_ = true;
    return base_from_imu_;
  } catch (const tf2::TransformException & e) {
    imu_mount_known_ = false;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Ignoring /imu until TF %s -> %s is available: %s",
      kBaseFrame, imu_frame.c_str(), e.what());
    return std::nullopt;
  }
}

void StateEstimationNode::on_reset(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    response->success = false;
    response->message = "not active";
    return;
  }
  logic_->reset();
  reported_resets_ = 0;
  reported_rejects_ = 0;
  response->success = true;
  response->message = "estimate reset to the odom origin";
  RCLCPP_INFO(get_logger(), "Reset to the odom origin");
}

void StateEstimationNode::report_health(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!logic_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "not configured");
    return;
  }
  const StateEstimationStatus & s = logic_->status();

  HealthInputs in;
  in.status = s;
  in.active = get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  in.imu_mount_known = imu_mount_known_;
  in.new_numerical_reset = s.numerical_resets > reported_resets_;
  in.new_rejected_sample = s.rejected_measurements > reported_rejects_;
  reported_resets_ = s.numerical_resets;
  reported_rejects_ = s.rejected_measurements;

  const HealthReport report = summarize_health(in);
  stat.summary(report.level, report.message);
  stat.add("imu_age", s.imu_age);
  stat.add("wheel_odom_age", s.wheel_age);
  stat.add("slip_detected", s.slip_detected);
  stat.add("covariance_trace", logic_->state().covariance.trace());
  stat.add("numerical_resets", s.numerical_resets);
  stat.add("rejected_measurements", s.rejected_measurements);
  stat.add("motion_model", logic_->config().motion_model);
  stat.add("fusion_method", logic_->config().fusion_method);
}

}  // namespace state_estimation
