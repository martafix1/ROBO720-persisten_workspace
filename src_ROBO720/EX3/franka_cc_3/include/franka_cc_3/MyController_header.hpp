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
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <chainjnttojacdotsolver.hpp>


// Trajectory point
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

// Taskspace objective 
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp"

// Memory management (use standard C++ smart pointers)
#include <memory>

// Service
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "franka_cc_3/srv/compute_ik.hpp"
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2/LinearMath/Transform.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

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
  Vector7d effort_interface_values_;
  
  // service
  rclcpp::Service<franka_cc_3::srv::ComputeIK>::SharedPtr ik_service_;
  void computeIKCallback(
    const std::shared_ptr<franka_cc_3::srv::ComputeIK::Request> request,
    std::shared_ptr<franka_cc_3::srv::ComputeIK::Response> response);
  

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
  // std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;

  // for IK:
  // Joint limits
  KDL::JntArray joint_min_limits_;
  KDL::JntArray joint_max_limits_;
  KDL::JntArray joint_center_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverVel_wdls> ik_vel_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solver_;
  
  std::unique_ptr<KDL::ChainJntToJacSolver>  jac_solver_;
  std::unique_ptr<KDL::ChainJntToJacDotSolver>  jacdot_solver_;
  



  // control checks
  bool x_d_first = true;
  bool xdot_d_first = true;

  //Joint space state
  KDL::JntArray qd_, qd_dot_, qd_ddot_;
  KDL::JntArray exertedEffort_;
  KDL::JntArray q_, qdot_;
  KDL::JntArray e_, e_dot_, e_int_;

  // Cartesian Velocity
  Eigen::VectorXd xdot;
  Eigen::VectorXd xdot_d;
  Eigen::VectorXd x_ddot_d;
  Eigen::VectorXd e_x;
  Eigen::Vector3d e_rot;

  // torqe Controller
  KDL::JntArray aux_d_;
  KDL::JntArray comp_d_;
  KDL::JntArray tau_d_;
    
  // gains
  KDL::JntArray Kp_, Ki_, Kd_;


  // for path planning
  int pathSteps_1 = 0;
  int maxpathSteps_1 = 40;
  KDL::Frame lastTarget = KDL::Frame::Identity();
    
  // joint handles for URDF
  std::vector<std::string> joint_names_;  // joint names
  std::string root_name, tip_name;  //this coz why not hardcode it
  // std::vector<hardware_interface::JointHandle> joints_;  // joint handles
  std::vector<urdf::JointConstSharedPtr> joint_urdfs_;  // joint urdfs
  

  #define MISCDATAMAX 69
  double miscData[MISCDATAMAX] = {0};

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_qd_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_q_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_e_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_tau_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_EE_pos;
  

  // messages
  std_msgs::msg::Float64MultiArray msg_qd_;
  std_msgs::msg::Float64MultiArray msg_q_;  
  std_msgs::msg::Float64MultiArray msg_e_;
  std_msgs::msg::Float64MultiArray msg_tau_;


  // Add your custom controller variables here
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr req_traj_point_subscriber_; //subscriber object
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr taskspace_objective_subscriber; // subscriber for taskspace objective

  trajectory_msgs::msg::MultiDOFJointTrajectoryPoint taskspace_objective_point;

  std::array<double, 7> req_pos = {0.0};
  std::array<double, 7> req_vel = {0.0};
  std::array<double, 7> req_acc = {0.0};
  std::array<bool, 7> requested_velocities_errState_ = {false,false,false,false,false,false,false,}; // the req velocities themselves
  std::mutex req_traj_point_mutex_; // mutex is apparently needed
  float position_lim_MAX[7] = { 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973};
  float position_lim_MIN[7] = {-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973};
  float position_centers[7] = {0};
  float position_ranges[7] = {0};
  float PIreg_I[7] = {0};
  // added for RLCPP throtle
  rclcpp::Clock::SharedPtr node_clock_;
  
  void updateJointStates();
  
  void ex3_smarterControllers(int controllerType);
  void TaskSpacePathPlanner(KDL::Frame target, KDL::Frame current, KDL::Frame &nextStep, int currentStep, int maxSteps);
  int InverseK(KDL::Frame target, KDL::JntArray &result);
  double compareFrames(KDL::Frame a, KDL::Frame b);

  int rateLimiter_100 = 0;
  int rateLimiter_10_1 = 0;
  int rateLimiter_10_2 = 0;

};

}  // namespace MyController_namespace