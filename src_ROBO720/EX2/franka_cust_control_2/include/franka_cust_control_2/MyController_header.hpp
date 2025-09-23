// Copyright (c) 2025 Your Name
//
// Licensed under the MIT License.

#pragma once
#include <string>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rclcpp/clock.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace MyController_namespace {

/**
 * My custom Franka controller based on the joint velocity example
 */
class MyController_class : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string robot_description_;
  bool is_gazebo{false};
  const int num_joints = 7;
  rclcpp::Duration elapsed_time_ = rclcpp::Duration(0, 0);
  
  // Add your custom controller variables here
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr req_velocity_subscriber_; //subscriber object
  std::array<double, 7> requested_velocities_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // the req velocities themselves
  std::array<bool, 7> requested_velocities_errState_ = {false,false,false,false,false,false,false,}; // the req velocities themselves
  std::mutex velocity_command_mutex_; // mutex is apparently needed
  float position_lim_MAX[7] = { 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973};
  float position_lim_MIN[7] = {-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973};
  // added for RLCPP throtle
  rclcpp::Clock::SharedPtr node_clock_;

};

}  // namespace MyController_namespace