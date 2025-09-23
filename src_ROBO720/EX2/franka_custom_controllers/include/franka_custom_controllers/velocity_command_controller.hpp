#ifndef FRANKA_CUSTOM_CONTROLLERS__VELOCITY_COMMAND_CONTROLLER_HPP_
#define FRANKA_CUSTOM_CONTROLLERS__VELOCITY_COMMAND_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace franka_custom_controllers
{

class VelocityCommandController : public controller_interface::ControllerInterface
{
public:
  VelocityCommandController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  std::vector<std::string> joint_names_;
  std::vector<double> velocity_commands_;
  
  realtime_tools::RealtimeBuffer<std::shared_ptr<std::vector<double>>> rt_command_ptr_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_command_subscription_;
  
  void velocity_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

}  // namespace franka_custom_controllers

#endif  // FRANKA_CUSTOM_CONTROLLERS__VELOCITY_COMMAND_CONTROLLER_HPP_