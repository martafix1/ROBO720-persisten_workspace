#include "franka_custom_controllers/velocity_command_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace franka_custom_controllers
{

VelocityCommandController::VelocityCommandController()
: controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration 
VelocityCommandController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/velocity");
  }
  
  return config;
}

controller_interface::InterfaceConfiguration 
VelocityCommandController::state_interface_configuration() const
{
  // We don't need state interfaces for pure velocity forwarding
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn VelocityCommandController::on_init()
{
  try {
    // Declare parameters
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VelocityCommandController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Get joint names from parameters
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joint names specified");
    return controller_interface::CallbackReturn::ERROR;
  }
  
  // For Franka, we expect 7 joints
  if (joint_names_.size() != 7) {
    RCLCPP_WARN(get_node()->get_logger(), 
      "Expected 7 joints for Franka, got %zu", joint_names_.size());
  }
  
  // Initialize command buffer
  velocity_commands_.resize(joint_names_.size(), 0.0);
  auto initial_command = std::make_shared<std::vector<double>>(joint_names_.size(), 0.0);
  rt_command_ptr_.writeFromNonRT(initial_command);
  
  // Create subscription for velocity commands
  velocity_command_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/commands",  // This creates topic at /controller_name/commands
    rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      velocity_command_callback(msg);
    }
  );
  
  RCLCPP_INFO(get_node()->get_logger(), 
    "Configured velocity controller for %zu joints", joint_names_.size());
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VelocityCommandController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset commands to zero
  velocity_commands_.assign(joint_names_.size(), 0.0);
  
  RCLCPP_INFO(get_node()->get_logger(), "Velocity controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VelocityCommandController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Send zero velocities before deactivating
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_interfaces_[i].set_value(0.0);
  }
  
  RCLCPP_INFO(get_node()->get_logger(), "Velocity controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type VelocityCommandController::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // Get the latest command
  auto command = rt_command_ptr_.readFromRT();
  
  // Apply commands to interfaces
  if (command && command->get()) {
    const auto & cmd_values = *(*command);
    
    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
      if (i < cmd_values.size()) {
        command_interfaces_[i].set_value(cmd_values[i]);
      }
    }
  }
  
  return controller_interface::return_type::OK;
}

void VelocityCommandController::velocity_command_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() != joint_names_.size()) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,  // Throttle to once per second
      "Received %zu velocity commands but have %zu joints",
      msg->data.size(), joint_names_.size()
    );
    return;
  }
  
  // Write to realtime buffer
  auto command = std::make_shared<std::vector<double>>(msg->data.begin(), msg->data.end());
  rt_command_ptr_.writeFromNonRT(command);
}

}  // namespace franka_custom_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  franka_custom_controllers::VelocityCommandController,
  controller_interface::ControllerInterface)