// Copyright (c) 2025 Your Name
//
// Licensed under the MIT License.

#include <franka_cust_control_2/MyController_header.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

using namespace std::chrono_literals;

namespace MyController_namespace {

controller_interface::InterfaceConfiguration
MyController_class::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back("panda_joint" + std::to_string(i) + "/velocity");
    // You can also use position or effort interfaces:
    // config.names.push_back("panda_joint" + std::to_string(i) + "/position");
    // config.names.push_back("panda_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
MyController_class::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back("panda_joint" + std::to_string(i) + "/position");
    config.names.push_back("panda_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type MyController_class::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  elapsed_time_ = elapsed_time_ + period;
  
  // YOUR CUSTOM CONTROL LOGIC GOES HERE
  // This example is based on the original but you can modify it completely
  
  rclcpp::Duration time_max(3.0, 0.0);
  double omega_max = 0.4;
  double cycle = std::floor(std::pow(
      -1.0, (elapsed_time_.seconds() - std::fmod(elapsed_time_.seconds(), time_max.seconds())) /
                time_max.seconds()));
  double omega = cycle * omega_max / 2.0 *
                 (1.0 - std::cos(2.0 * M_PI / time_max.seconds() * elapsed_time_.seconds()));

  // Example: Move only joints 4 and 5 (indices 3 and 4)
  for (int i = 0; i < num_joints; i++) {
    if(i==0){
      command_interfaces_[i].set_value(omega*8);
    } 
    else if (i == 3 || i == 4) {
      command_interfaces_[i].set_value(omega);
    } else {
      command_interfaces_[i].set_value(0.0);
    }
  }

  // Example of how to read current joint states:
  // for (int i = 0; i < num_joints; i++) {
  //   double current_position = state_interfaces_[2*i].get_value();      // position
  //   double current_velocity = state_interfaces_[2*i + 1].get_value();  // velocity
  //   
  //   // Use these values in your control algorithm
  //   RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
  //                        "Joint %d: pos=%.3f, vel=%.3f", i+1, current_position, current_velocity);
  // }

  return controller_interface::return_type::OK;
}

CallbackReturn MyController_class::on_init() {
  try {
    auto_declare<bool>("gazebo", false);
    auto_declare<std::string>("robot_description", "");
    
    // Declare your custom parameters here
    // auto_declare<double>("my_custom_parameter", 1.0);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MyController_class::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  is_gazebo = get_node()->get_parameter("gazebo").as_bool();

  // Get your custom parameters here
  // custom_parameter_ = get_node()->get_parameter("my_custom_parameter").as_double();

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  RCLCPP_INFO(get_node()->get_logger(), "MyController_class configured successfully!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MyController_class::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  elapsed_time_ = rclcpp::Duration(0, 0);
  RCLCPP_INFO(get_node()->get_logger(), "MyController_class activated!");
  return CallbackReturn::SUCCESS;
}

}  // namespace MyController_namespace

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(MyController_namespace::MyController_class,
                       controller_interface::ControllerInterface)