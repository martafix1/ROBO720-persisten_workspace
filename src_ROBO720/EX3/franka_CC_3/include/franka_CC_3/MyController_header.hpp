// Copyright (c) 2025 Your Name
//
// Licensed under the MIT License.

#pragma once
#include <string>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rclcpp/clock.hpp>

#include <Eigen/Eigen>

// URDF model parsing
#include <urdf/model.h>

// KDL libraries
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

// Memory management (use standard C++ smart pointers)
#include <memory>




using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace MyController_namespace {

/**
 * My custom Franka controller based on the joint velocity example
 */
class MyController_class : public controller_interface::ControllerInterface {
 public:
 using Vector7d = Eigen::Matrix<double, 7, 1>;
    MyController_class();
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

  Vector7d position_interface_values_;
  Vector7d velocity_interface_values_;
  
  // KDL
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;

  // kdl variables
  KDL::JntSpaceInertiaMatrix M_; // inertia matrix
  KDL::JntArray C_; // coriolis and centrifugal forces
  KDL::JntArray G_; // gravity forces
  KDL::Vector gravity_;

  // kdl solver (solver to compute the inverse dynamics)
  std::unique_ptr<KDL::ChainDynParam> id_solver_;

  //Joint space state
  KDL::JntArray qd_, qd_dot_, qd_ddot_;
  KDL::JntArray q_, qdot_;
  KDL::JntArray e_, e_dot_, e_int_;

  // input
  KDL::JntArray aux_d_;
  KDL::JntArray comp_d_;
  KDL::JntArray tau_d_;
    
  // gains
  KDL::JntArray Kp_, Ki_, Kd_;

  // joint handles for URDF
  std::vector<std::string> joint_names_;  // joint names
  std::string root_name, tip_name;  //this coz why not hardcode it
  // std::vector<hardware_interface::JointHandle> joints_;  // joint handles
  std::vector<urdf::JointConstSharedPtr> joint_urdfs_;  // joint urdfs


  // Add your custom controller variables here
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr req_angle_subscriber_; //subscriber object
  std::array<double, 7> requested_angles_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // the req velocities themselves
  std::array<bool, 7> requested_velocities_errState_ = {false,false,false,false,false,false,false,}; // the req velocities themselves
  std::mutex angle_command_mutex_; // mutex is apparently needed
  float position_lim_MAX[7] = { 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973};
  float position_lim_MIN[7] = {-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973};
  float position_centers[7] = {0};
  float position_ranges[7] = {0};
  float PIreg_I[7] = {0};
  // added for RLCPP throtle
  rclcpp::Clock::SharedPtr node_clock_;
  
  void updateJointStates();


};

}  // namespace MyController_namespace