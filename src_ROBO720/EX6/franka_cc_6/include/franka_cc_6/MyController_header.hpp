// Copyright (c) 2025 Your Name
//
// Licensed under the MIT License.


// #define DEBUG_IK
// #define DEBUG_EX3
// #define DEBUG_INPUT
#define DEBUG_EX5_POINTS
//#define DO_REPULSE

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
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2/LinearMath/Transform.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainjnttojacsolver.hpp>

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
  

  //Joint space state
  KDL::JntArray qd_, qd_dot_, qd_ddot_;
  KDL::JntArray exertedEffort_;
  KDL::JntArray q_, qdot_;
  KDL::JntArray e_, e_dot_, e_int_;

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


  // for taskspace repulsion
  struct pointRep{
    Eigen::Vector3d x;
    double dist0;
    double dist1;
    double dist10; // read only 
    double k;
    double a;
    double b;

    pointRep(const Eigen::Vector3d& x_, double d0, double d1,double k)
        : x(x_), dist0(d0), dist1(d1), dist10(0), k(k), a(0), b(0)
    {
        MyController_class::taskSpaceRepulse_calcRepulseCoefs(*this);
    }
  };

  std::vector<pointRep> repulsionPoints;

  struct planeRep_finite{
    pointRep centerPoint; //has all the repulsion stuff, acts as a plane cetre
    Eigen::Vector3d direction;
    Eigen::Vector3d planeX; // will be projected, just should not be parralel 
    Eigen::Vector3d planeY; // auto calculated
    Eigen::Vector2d l1;
    Eigen::Vector2d l2;
    // Eigen::Vector2d l3;
    // Eigen::Vector2d l4;

    planeRep_finite(const pointRep centerPoint_, const Eigen::Vector3d direction_, const Eigen::Vector3d planeX_,
       const Eigen::Vector2d l1_, const Eigen::Vector2d l2_)
      :  centerPoint(centerPoint_), direction(direction_), planeX(planeX_), l1(l1_), l2(l2_)/*, l3(l3_), l4(l4_)*/
      {
        direction.normalize();
        planeX = (planeX - (planeX.dot(direction)) * direction);
        double normX = planeX.norm();
        if (normX < 1e-8) {
            std::cerr << "Warning: planeX is parallel (or almost parallel) to the plane normal!" << std::endl;
            throw std::runtime_error("planeRep_finite: planeX is parallel (or almost parallel) to the plane normal.");
            //planeX = Eigen::Vector3d(1, 0, 0); // fallback to avoid NaNs
        } else {
            planeX.normalize();
        }

        planeY = planeX.cross(direction);

        

      }
  };
    
  std::vector<planeRep_finite> repulsionPlanes;


  //jacobian solver
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver;


  // joint handles for URDF
  std::vector<std::string> joint_names_;  // joint names
  std::string root_name, tip_name;  //this coz why not hardcode it
  // std::vector<hardware_interface::JointHandle> joints_;  // joint handles
  std::vector<urdf::JointConstSharedPtr> joint_urdfs_;  // joint urdfs

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_qd_dot_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_qdot_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_qd_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_q_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_e_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_tau_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_EE_pos; 
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_misc;
  

  // messages

  std_msgs::msg::Float64MultiArray msg_qd_dot;
  std_msgs::msg::Float64MultiArray msg_q_dot;  
  std_msgs::msg::Float64MultiArray msg_qd_;
  std_msgs::msg::Float64MultiArray msg_q_;  
  std_msgs::msg::Float64MultiArray msg_e_;
  std_msgs::msg::Float64MultiArray msg_tau_;
  std_msgs::msg::Float64MultiArray msg_miscData_;
  
  #define MISCDATAMAX 69
  double miscData[MISCDATAMAX] = {0};

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

  float velocity_lim_MIN[7] = {-2.1750,-2.1750,-3.15,-3.15,-3.15,-3.15,-3.15};
  float velocity_lim_MAX[7] = {2.1750,2.1750,3.15,3.15,3.15,3.15,3.15 };

  // arbitraly set, not actually in the robot (neither torqe nor acc) feel free to change
  float acc_lim_MIN[7] = {-2.1750,-2.1750,-3.15,-3.15,-3.15,-3.15,-3.15};
  float acc_lim_MAX[7] = {2.1750,2.1750,3.15,3.15,3.15,3.15,3.15};


  float PIreg_I[7] = {0};
  // added for RLCPP throtle
  rclcpp::Clock::SharedPtr node_clock_;
  
  void updateJointStates();
  void ex3_smarterControllers(int controllerType);
  int InverseK(KDL::Frame target, KDL::JntArray &result);
  void TaskSpacePathPlanner(KDL::Frame target, KDL::Frame current, KDL::Frame &nextStep, int currentStep, int maxSteps);
  double compareFrames(KDL::Frame a, KDL::Frame b);
  int rateLimiter_10_1 = 0;
  int rateLimiter_10_2 = 0;


  void ex5_potentialFields();
  
  void jointLimitRepulse();
  void taskSpaceRepulse();
  static void taskSpaceRepulse_calcRepulseCoefs(pointRep &point);
  void taskSpaceRepulse_initHardcodedStuff();
  bool useJointSpaceInputs = false;



};

}  // namespace MyController_namespace