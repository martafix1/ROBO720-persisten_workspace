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
  joint_names_ = {
    "panda_joint1","panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7",
  };
   tip_name= "panda_link7";
   root_name= "base";

   std::cout << "\033[35m ItDidWork: \033[0m constructor" << std::endl;


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
  
  //switcheroo as they cannot be assigned. 
  for (int i = 0; i < num_joints; i++)
  {
      q_(i) = position_interface_values_(i);
      qdot_(i) = velocity_interface_values_(i);
  }

  
    // Compute model(M,C,G) 
  id_solver_->JntToMass(q_, M_);
  id_solver_->JntToCoriolis(q_, qdot_, C_);
  id_solver_->JntToGravity(q_, G_); 


    //switch from kdl JntArray to eigen for matrix operations
  Eigen::VectorXd qdot_eigen = qdot_.data;

  
   tau_d_.data = (G_.data*1.5) ;//- qdot_.data;
  //Eigen::VectorXd tau = G_.data - qdot_.data;
  

  for(int i =0; i <num_joints; ++i){
    torqe_command[i] =  tau_d_(i); //not needed rn likely
    //torqe_command[i] =  tau(i); //not needed rn likely
    
  }
  
  //RCLCPP_INFO(get_node()->get_logger(), "\033[35m ItDidWork: \033[0m %d", 4);
  

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
    return CallbackReturn::FAILURE;
  }

   // Get the joint names from the parameter server
  // joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_); //this shit needs working yaml, but there is enough hard coded shit anyway so why do it this way
  if (joint_names_.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "joint_names_ not set");
    return CallbackReturn::FAILURE;
  }
  // Check if there are the correct number of joint names
  if (joint_names_.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "joint_names_ should be of size %d but is of size %ld",
                 num_joints, joint_names_.size());
    return CallbackReturn::FAILURE;
  }
  // Get the URDF model and the joint URDF objects
  urdf::Model urdf;
  if (!urdf.initString(robot_description_))
  {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse urdf file");
      return CallbackReturn::ERROR;
  }
  else
  {
      RCLCPP_INFO(get_node()->get_logger(), "Found robot_description");
  }

  // Get the joint URDF objects
  for (int i = 0; i < num_joints; i++)
  {
    urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
    if (!joint_urdf)
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Could not find joint '%s' in urdf", joint_names_[i].c_str());
        return CallbackReturn::ERROR;
    }
    joint_urdfs_.push_back(joint_urdf);
  }

  // Get the KDL tree from the robot description
  if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to construct kdl tree");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(get_node()->get_logger(), "Constructed kdl tree");
  }

  // Get the root and tip link names from the parameter server
  // If the parameter is not found, return an error
  
  //  std::string root_name, tip_name; // more yaml parameters begone 
  // if (get_node()->has_parameter("root_link"))
  // {
  //   root_name = get_node()->get_parameter("root_link").as_string();
  //   RCLCPP_INFO(get_node()->get_logger(), "Found root link name form yaml: %s", root_name.c_str());
  // }
  // else
  // {
  //   RCLCPP_ERROR(get_node()->get_logger(), "Could not find root link name");
  //   return CallbackReturn::ERROR;
  // }
  // if (get_node()->has_parameter("tip_link"))
  // {
  //   tip_name = get_node()->get_parameter("tip_link").as_string();
  //   RCLCPP_INFO(get_node()->get_logger(), "Found tip link name form yaml: %s", tip_name.c_str());
  // }
  // else
  // {
  //   RCLCPP_ERROR(get_node()->get_logger(), "Could not find tip link name");
  //   return CallbackReturn::ERROR;
  // }

  // Get the KDL chain from the KDL tree
  // if kdl tree has no chain from root to tip, return error
  if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to get KDL chain from tree: ");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  " << root_name << " --> " << tip_name);
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  The segments are:");

    KDL::SegmentMap segment_map = kdl_tree_.getSegments();
    KDL::SegmentMap::iterator it;

    for (it = segment_map.begin(); it != segment_map.end(); it++)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "    %s", std::string((*it).first).c_str());
    }

    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(get_node()->get_logger(), "Got kdl chain");

    // debug: print kdl tree and kdl chain
    RCLCPP_INFO(get_node()->get_logger(), "  %s --> %s", root_name.c_str(), tip_name.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "  Tree has %d joints", kdl_tree_.getNrOfJoints());
    RCLCPP_INFO(get_node()->get_logger(), "  Tree has %d segments", kdl_tree_.getNrOfSegments());
    RCLCPP_INFO(get_node()->get_logger(), "  The kdl_tree_ segments are:");

    // Print the segments of the KDL tree
    KDL::SegmentMap segment_map = kdl_tree_.getSegments();
    KDL::SegmentMap::iterator it;
    for (it = segment_map.begin(); it != segment_map.end(); it++)
    {
      RCLCPP_INFO(get_node()->get_logger(), "    %s", std::string((*it).first).c_str());
    }
    RCLCPP_INFO(get_node()->get_logger(), "  Chain has %d joints", kdl_chain_.getNrOfJoints());
    RCLCPP_INFO(get_node()->get_logger(), "  Chain has %d segments", kdl_chain_.getNrOfSegments());
    RCLCPP_INFO(get_node()->get_logger(), "  The kdl_chain_ segments are:");
    for (unsigned int i = 0; i < kdl_chain_.getNrOfSegments(); i++) {
        const KDL::Segment& segment = kdl_chain_.getSegment(i);
        RCLCPP_INFO(get_node()->get_logger(), "    %s", segment.getName().c_str());
    }
  }

  // Create the KDL chain dyn param solver
  id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

  M_.resize(kdl_chain_.getNrOfJoints());
  C_.resize(kdl_chain_.getNrOfJoints());
  G_.resize(kdl_chain_.getNrOfJoints());

  // print kdltree, kdlchain, jointnames, jointurdfs for learning purposes
  fprintf(stderr, "Number of segments in kdl_tree_: %d\n", kdl_tree_.getNrOfSegments());
  fprintf(stderr, "Number of joints in kdl_chain_: %d\n", kdl_chain_.getNrOfJoints());
  fprintf(stderr, "Joint names in joint_names_: ");
  for (int i = 0; i < num_joints; i++)
  {
    fprintf(stderr, "%s ", joint_names_[i].c_str());
  }
  fprintf(stderr, "\n");

  RCLCPP_INFO(get_node()->get_logger(), "MyController_class configured successfully!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MyController_class::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  elapsed_time_ = rclcpp::Duration(0, 0);


// Initialize the joint states
  updateJointStates();

  // Initialize the KDL variables
  M_.data.setZero();
  C_.data.setZero();
  G_.data.setZero();

  // t = 0.0;  // Initialize the simulation time variable

  // // Initialize the variables
  // qd_.resize(num_joints);
  // qd_dot_.resize(num_joints);
  // qd_ddot_.resize(num_joints);
   q_.resize(num_joints);
   qdot_.resize(num_joints);
  // e_.resize(num_joints);
  // e_dot_.resize(num_joints);
  // e_int_.resize(num_joints);

  // aux_d_.resize(num_joints);
  // comp_d_.resize(num_joints);
   tau_d_.resize(num_joints);

  // Kp_.resize(num_joints);
  // Ki_.resize(num_joints);
  // Kd_.resize(num_joints);

  // for (int i = 0; i < SaveDataMax; i++) {
  //   SaveData_[i] = 0.0;
  // }

  // // Activate the publishers
  // pub_qd_->on_activate();
  // pub_q_->on_activate();
  // pub_e_->on_activate();
  // pub_SaveData_->on_activate();

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