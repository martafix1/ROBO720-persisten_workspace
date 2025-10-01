// Copyright (c) 2025 Your Name
//
// Licensed under the MIT License.

#include <franka_CC_3/MyController_header.hpp>

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
     //config.names.push_back("panda_joint" + std::to_string(i) + "/effort");
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

MyController_class::MyController_class(){
   for (int i = 0; i < num_joints; ++i) {
    position_centers[i] = (position_lim_MAX[i] + position_lim_MIN[i]) / 2.0f;
    position_ranges[i] = position_lim_MAX[i] - position_lim_MIN[i];
  }

}


controller_interface::return_type MyController_class::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  elapsed_time_ = elapsed_time_ + period;
  
  // YOUR CUSTOM CONTROL LOGIC GOES HERE
  // This example is based on the original but you can modify it completely
  
  // rclcpp::Duration time_max(3.0, 0.0);
  // double omega_max = 0.4;
  // double cycle = std::floor(std::pow(
  //     -1.0, (elapsed_time_.seconds() - std::fmod(elapsed_time_.seconds(), time_max.seconds())) /
  //               time_max.seconds()));
  // double omega = cycle * omega_max / 2.0 *
  //                (1.0 - std::cos(2.0 * M_PI / time_max.seconds() * elapsed_time_.seconds()));

  // double myWave = std::sin(2.0 * M_PI / time_max.seconds() * elapsed_time_.seconds());
 

  // this uses the requested_velocities
  std::array<double, 7> angles;
  {
    std::lock_guard<std::mutex> lock(angle_command_mutex_);
    angles = requested_angles_;
  }




  std::array<double, 7> torqe_command;
 
  updateJointStates();

  for(int i =0; i <num_joints; ++i){
    torqe_command[i] = -velocity_interface_values_[i] *2;
  }
  
  

  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(torqe_command[i]);
  }

  return controller_interface::return_type::OK;
}

CallbackReturn MyController_class::on_init() {
  try {
    auto_declare<bool>("gazebo", false);
    auto_declare<std::string>("robot_description", "");
    get_node()->set_parameter(rclcpp::Parameter("use_sim_time", true));
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


  node_clock_ = get_node()->get_clock();
  RCLCPP_INFO(get_node()->get_logger(), "\033[35m Clock type: \033[0m %d", node_clock_->get_clock_type());

  // Get your custom parameters here
  // custom_parameter_ = get_node()->get_parameter("my_custom_parameter").as_double();

  // reqested angle subscriber
  auto node = get_node();  // Shortcut
  req_angle_subscriber_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/requested_angles_CMD_INTERFACE",
      10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 7) {
          RCLCPP_WARN(this->get_node()->get_logger(), "Received angle command with wrong size: %zu", msg->data.size());
          return;
        }
        std::lock_guard<std::mutex> lock(angle_command_mutex_);
        std::copy_n(msg->data.begin(), 7, requested_angles_.begin());
      } // unlocks when it leaves this scope
  );

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

void MyController_class::updateJointStates() {
  // Pre-check array size to avoid bounds checking in loop
  if (state_interfaces_.size() != 2 * num_joints) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid number of state interfaces");
    return;
  }

  // Get the current joint positions and velocities
  auto* interfaces = state_interfaces_.data();
  for (size_t i = 0; i < num_joints; ++i) {
    // Access interfaces directly with pointer arithmetic
    const auto& position_interface = interfaces[2 * i];
    const auto& velocity_interface = interfaces[2 * i + 1];
    
    // Interface name comparison
    const auto& pos_name = position_interface.get_interface_name();
    const auto& vel_name = velocity_interface.get_interface_name();
    
    if (pos_name != "position") {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected position interface, but got %s", 
                   pos_name.c_str());
      return;
    }
    if (vel_name != "velocity") {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected velocity interface, but got %s", 
                   vel_name.c_str());
      return;
    }

    // Direct value assignment
    position_interface_values_(i) = position_interface.get_value();
    velocity_interface_values_(i) = velocity_interface.get_value();
  }
}



}  // namespace MyController_namespace

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(MyController_namespace::MyController_class,
                       controller_interface::ControllerInterface)