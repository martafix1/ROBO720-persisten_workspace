// Copyright (c) 2025 Your Name
//
// Licensed under the MIT License.

#include <franka_cc_6/MyController_header.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

using namespace std::chrono_literals;

namespace MyController_namespace
{

  controller_interface::InterfaceConfiguration
  MyController_class::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (int i = 1; i <= num_joints; ++i)
    {
      // config.names.push_back("panda_joint" + std::to_string(i) + "/velocity");
      //  You can also use position or effort interfaces:
      //  config.names.push_back("panda_joint" + std::to_string(i) + "/position");
      config.names.push_back("panda_joint" + std::to_string(i) + "/effort");
    }

    return config;
  }

  controller_interface::InterfaceConfiguration
  MyController_class::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i = 1; i <= num_joints; ++i)
    {
      config.names.push_back("panda_joint" + std::to_string(i) + "/position");
      config.names.push_back("panda_joint" + std::to_string(i) + "/velocity");
      config.names.push_back("panda_joint" + std::to_string(i) + "/effort");
    }
    return config;
  }

  MyController_class::MyController_class()
  {
    for (int i = 0; i < num_joints; ++i)
    {
      position_centers[i] = (position_lim_MAX[i] + position_lim_MIN[i]) / 2.0f;
      position_ranges[i] = position_lim_MAX[i] - position_lim_MIN[i];
    }
    joint_names_ = {
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    };
    tip_name = "panda_link7";
    root_name = "base";


    std::cout << "\033[35m ItDidWork: \033[0m constructor" << std::endl;
  }

  controller_interface::return_type MyController_class::update(
      const rclcpp::Time & /*time*/,
      const rclcpp::Duration &period)
  {
    elapsed_time_ = elapsed_time_ + period;

    std::array<double, 7> torqe_command;

    updateJointStates();

    // switcheroo as they cannot be assigned.
    for (int i = 0; i < num_joints; i++)
    {
      q_(i) = position_interface_values_(i);
      qdot_(i) = velocity_interface_values_(i);
      exertedEffort_(i) = effort_interface_values_(i);

      if(useJointSpaceInputs){
        qd_(i) = req_pos[i];
        qd_dot_(i) = req_vel[i];
        qd_ddot_(i) = req_acc[i];

        double w = 2; //rad/s
        double damp = 1; // relative

        Kp_.data = (w*w) * (Eigen::VectorXd(7) << 2.2,2.0,1.8,1.6,1.4,1.2,1.0).finished();
        Kd_.data = (w*damp*2) * (Eigen::VectorXd(7) << 2.2,2.0,1.8,1.6,1.4,1.2,1.0).finished();

      }

    }

    // required for potential fields:
    qd_dot_.data.setZero();


    // rate limiter
    if (rateLimiter_10_1 < 10)
    {
      rateLimiter_10_1++;
    }
    else
    {
      rateLimiter_10_1 = 0;
      if(!useJointSpaceInputs){
        ex3_smarterControllers(3); // 100Hz
      } 
    }

    #ifdef DO_REPULSE
      ex5_potentialFields();
    #endif

    e_.data = qd_.data - q_.data;
    e_dot_.data = qd_dot_.data - qdot_.data;
    // e_int_.data = qd_.data - q_.data; //wtf is dis for

    // Compute model(M,C,G)
    id_solver_->JntToMass(q_, M_);
    id_solver_->JntToCoriolis(q_, qdot_, C_);
    id_solver_->JntToGravity(q_, G_);

    // switch from kdl JntArray to eigen for matrix operations
    // Eigen::VectorXd qdot_eigen = qdot_.data;

    aux_d_.data = M_.data * (qd_ddot_.data + Kp_.data.cwiseProduct(e_.data) + Kd_.data.cwiseProduct(e_dot_.data));
    comp_d_.data = C_.data + G_.data;
    tau_d_.data = aux_d_.data + comp_d_.data;

    // tau_d_.data = (G_.data) - qdot_.data * 0.5;
    // Eigen::VectorXd tau = G_.data - qdot_.data;

    for (int i = 0; i < num_joints; ++i)
    {
      torqe_command[i] = tau_d_(i); // not needed rn likely
      // torqe_command[i] =  tau(i); //not needed rn likely
    }

    // RCLCPP_INFO(get_node()->get_logger(), "\033[35m ItDidWork: \033[0m %d", 4);

    for (int i = 0; i < num_joints; ++i)
    {
      command_interfaces_[i].set_value(torqe_command[i]);
    }

    // Clear the data arrays
    msg_qd_dot.data.clear();
    msg_q_dot.data.clear();  
    msg_qd_.data.clear();
    msg_q_.data.clear();
    msg_e_.data.clear();
    msg_tau_.data.clear();
    msg_miscData_.data.clear();
    // Fill the data arrays with the calculated values
    for (int i = 0; i < num_joints; i++)  
    {
      msg_qd_dot.data.push_back(qd_dot_(i));
      msg_q_dot.data.push_back(qdot_(i));
      msg_qd_.data.push_back(qd_(i));
      msg_q_.data.push_back(q_(i));
      msg_e_.data.push_back(exertedEffort_(i));
      msg_tau_.data.push_back(tau_d_(i));
    }

    for (int i = 0; i < MISCDATAMAX; i++)
    {
      msg_miscData_.data.push_back(miscData[i]);
    }

    // Publish data to topics
    pub_qd_dot_ ->publish(msg_qd_dot);
    pub_qdot_ ->publish(msg_q_dot);

    pub_qd_->publish(msg_qd_);
    pub_q_->publish(msg_q_);
    pub_e_->publish(msg_e_);
    pub_tau_->publish(msg_tau_);
    pub_misc->publish(msg_miscData_);

    KDL::Frame ee_frame;
    int fk_result = fk_solver_->JntToCart(q_, ee_frame);
    if (fk_result >= 0)
    {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.stamp = node_clock_->now();
      msg.header.frame_id = root_name; // or "base_link" etc.

      msg.pose.position.x = ee_frame.p.x();
      msg.pose.position.y = ee_frame.p.y();
      msg.pose.position.z = ee_frame.p.z();

      double x, y, z, w;
      ee_frame.M.GetQuaternion(x, y, z, w);
      msg.pose.orientation.x = x;
      msg.pose.orientation.y = y;
      msg.pose.orientation.z = z;
      msg.pose.orientation.w = w;

      pub_EE_pos->publish(msg);
    }
    else
    {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to compute FK");
    }

    

    return controller_interface::return_type::OK;
  }

  CallbackReturn MyController_class::on_init()
  {
    try
    {
      auto_declare<bool>("gazebo", false);
      auto_declare<std::string>("robot_description", "");
      get_node()->set_parameter(rclcpp::Parameter("use_sim_time", true));
      // Declare your custom parameters here
      // auto_declare<double>("my_custom_parameter", 1.0);

      // Create publishers for the desired and current joint positions, velocities, and accelerations
      pub_qd_dot_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("qd_dot", 1000);
      pub_qdot_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("q_dot", 1000);

      pub_qd_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("qd", 1000);
      pub_q_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("q", 1000);
      pub_e_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("e", 1000);
      pub_tau_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("tau", 1000);
      pub_EE_pos = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("EE_pos", 1000);
      pub_misc = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("misc", 1000);
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn MyController_class::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    is_gazebo = get_node()->get_parameter("gazebo").as_bool();

    node_clock_ = get_node()->get_clock();
    RCLCPP_INFO(get_node()->get_logger(), "\033[35m Clock type: \033[0m %d", node_clock_->get_clock_type());

    // Get your custom parameters here
    // custom_parameter_ = get_node()->get_parameter("my_custom_parameter").as_double();

    // reqested angle subscriber
    auto node = get_node(); // Shortcut
    req_traj_point_subscriber_ = node->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
        "/requested_traj_point",
        10,
        [this](const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg)
        {
          if (msg->positions.size() != num_joints ||
              msg->velocities.size() != num_joints ||
              msg->accelerations.size() != num_joints)
          {
            
            RCLCPP_WARN(get_node()->get_logger(), "Received trajectory point with wrong sizes");
            
            return;
          }

          std::lock_guard<std::mutex> lock(req_traj_point_mutex_);
          std::copy_n(msg->positions.begin(), num_joints, req_pos.begin());
          std::copy_n(msg->velocities.begin(), num_joints, req_vel.begin());
          std::copy_n(msg->accelerations.begin(), num_joints, req_acc.begin());
        });

    // Taskspace objective subscriber
    taskspace_objective_subscriber = node->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
        "/taskspace_objective",
        10,
        [this](const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg)
        {
          if (msg->points.empty())
          {
            RCLCPP_WARN(get_node()->get_logger(), "Received message with no points");
            return;
          }

          taskspace_objective_point = msg->points[0];
          #ifdef DEBUG_INPUT
          RCLCPP_INFO(get_node()->get_logger(), "Received taskspace objective");
          #endif
          

        });

    auto parameters_client =
        std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
    parameters_client->wait_for_service();

    auto future = parameters_client->get_parameters({"robot_description"});
    auto result = future.get();
    if (!result.empty())
    {
      robot_description_ = result[0].value_to_string();
    }
    else
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
      return CallbackReturn::FAILURE;
    }

    // Get the joint names from the parameter server
    // joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_); //this shit needs working yaml, but there is enough hard coded shit anyway so why do it this way
    if (joint_names_.empty())
    {
      RCLCPP_FATAL(get_node()->get_logger(), "joint_names_ not set");
      return CallbackReturn::FAILURE;
    }
    // Check if there are the correct number of joint names
    if (joint_names_.size() != static_cast<uint>(num_joints))
    {
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
    // Initialize joint limits
    joint_min_limits_ = KDL::JntArray(num_joints);
    joint_max_limits_ = KDL::JntArray(num_joints);
    joint_center_ = KDL::JntArray(num_joints);

    for (size_t i = 0; i < num_joints; ++i)
    {
      const auto &joint_urdf = joint_urdfs_[i];

      if (!joint_urdf->limits)
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' has no limits defined!", joint_names_[i].c_str());
        return CallbackReturn::ERROR;
      }

      joint_min_limits_(i) = joint_urdf->limits->lower;
      joint_max_limits_(i) = joint_urdf->limits->upper;
      joint_center_(i) = (joint_max_limits_(i) - joint_min_limits_(i)) / 2 + joint_min_limits_(i);
      RCLCPP_INFO(get_node()->get_logger(), "Joint '%s' limits: [%f, %f]",
                  joint_names_[i].c_str(), joint_min_limits_(i), joint_max_limits_(i));
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
      for (unsigned int i = 0; i < kdl_chain_.getNrOfSegments(); i++)
      {
        const KDL::Segment &segment = kdl_chain_.getSegment(i);
        RCLCPP_INFO(get_node()->get_logger(), "    %s", segment.getName().c_str());
      }
    }

    // Set the gravity vector
    gravity_ = KDL::Vector::Zero();
    gravity_(2) = -9.81;
    // Create the KDL chain dyn param solver
    id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
    // ik_solver_.reset(new KDL::ChainIkSolverPos_LMA(kdl_chain_));

    // set up solvers
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
    // ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(kdl_chain_);
    ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_wdls>(kdl_chain_);
    ik_vel_solver_->setLambda(0.01);

    ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
        kdl_chain_,
        joint_min_limits_,
        joint_max_limits_,
        *fk_solver_,
        *ik_vel_solver_,
        1600, // Max iterations  default: 400
        1e-4  // Tolerance default: 1e-5
    );

    jac_solver = std::make_unique<KDL::ChainJntToJacSolver>(kdl_chain_);

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
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    elapsed_time_ = rclcpp::Duration(0, 0);

    // Initialize the joint states
    updateJointStates();

    // Initialize the KDL variables
    M_.data.setZero();
    C_.data.setZero();
    G_.data.setZero();

    // t = 0.0;  // Initialize the simulation time variable

    // Initialize the variables
    qd_.resize(num_joints);
    qd_dot_.resize(num_joints);
    qd_ddot_.resize(num_joints);
    q_.resize(num_joints);
    qdot_.resize(num_joints);
    e_.resize(num_joints);
    e_dot_.resize(num_joints);
    e_int_.resize(num_joints);

    aux_d_.resize(num_joints);
    comp_d_.resize(num_joints);
    tau_d_.resize(num_joints);

    exertedEffort_.resize(num_joints);

    Kp_.resize(num_joints);
    Ki_.resize(num_joints);
    Kd_.resize(num_joints);

    Kp_.data.setConstant(2);
    Ki_.data.setConstant(0);
    Kd_.data.setConstant(1);

    // for (int i = 0; i < SaveDataMax; i++) {
    //   SaveData_[i] = 0.0;
    // }

    // Activate the publishers
    pub_qd_dot_->on_activate();
    pub_qdot_->on_activate();

    pub_qd_->on_activate();
    pub_q_->on_activate();
    pub_e_->on_activate();
    pub_tau_->on_activate();
    pub_EE_pos->on_activate();
    pub_misc->on_activate();


    RCLCPP_INFO(get_node()->get_logger(), "MyController_class activated!");
    if(useJointSpaceInputs)
    {
      RCLCPP_WARN(get_node()->get_logger(), " \033[44m controller expects \033[0m jointspace inputs");
    }
    if(!useJointSpaceInputs)
    {
      RCLCPP_WARN(get_node()->get_logger(), " \033[41m controller expects \033[0m taskspace inputs");
    }

    #ifdef DO_REPULSE
    taskSpaceRepulse_initHardcodedStuff();
    // cannot be in the static function or in the constructor (no logger yet).
    #ifdef DEBUG_EX5_POINTS
    for (size_t i = 0; i < repulsionPoints.size(); ++i) {
      const auto& point = repulsionPoints[i];
      RCLCPP_INFO(get_node()->get_logger(), "added point (%4.2f %4.2f %4.2f), d10 = %4.2f",point.x.x(),point.x.y(),point.x.z(),point.dist10);
    }   
    #endif
    #endif


    return CallbackReturn::SUCCESS;
  }

  void MyController_class::updateJointStates()
  {
    // Pre-check array size to avoid bounds checking in loop
    if (state_interfaces_.size() != 3 * num_joints)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid number of state interfaces");
      return;
    }

    // Get the current joint positions and velocities
    auto *interfaces = state_interfaces_.data();
    for (size_t i = 0; i < num_joints; ++i)
    {
      // Access interfaces directly with pointer arithmetic
      const auto &position_interface = interfaces[3 * i];
      const auto &velocity_interface = interfaces[3 * i + 1];
      const auto &effort_interface = interfaces[3 * i + 2];

      // Interface name comparison
      const auto &pos_name = position_interface.get_interface_name();
      const auto &vel_name = velocity_interface.get_interface_name();
      const auto &eff_name = effort_interface.get_interface_name();

      if (pos_name != "position")
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Expected position interface, but got %s",
                     pos_name.c_str());
        return;
      }
      if (vel_name != "velocity")
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Expected velocity interface, but got %s",
                     vel_name.c_str());
        return;
      }
      if (eff_name != "effort")
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Expected effort interface, but got %s",
                     eff_name.c_str());
        return;
      }

      // Direct value assignment
      position_interface_values_(i) = position_interface.get_value();
      velocity_interface_values_(i) = velocity_interface.get_value();
      effort_interface_values_(i) = effort_interface.get_value();
    }
  }


  void MyController_class::TaskSpacePathPlanner(KDL::Frame target, KDL::Frame current, KDL::Frame &nextStep, int currentStep, int maxSteps)
  {

    // total linear interpolation
    if (true)
    {

      double completness = (double)currentStep / (double)maxSteps; // ratio of how much of the movement is supposed to be completed
      if (completness > 1) completness = 1;
      if (completness < 0) completness = 0;

      double xd = target.p.x();
      double yd = target.p.y();
      double zd = target.p.z();
      double rd, pd, ad;
      target.M.GetRPY(rd, pd, ad);

      double xc = current.p.x();
      double yc = current.p.y();
      double zc = current.p.z();
      double rc, pc, ac;
      target.M.GetRPY(rc, pc, ac);

      double xn = (xd - xc) * completness + xc;
      double yn = (yd - yc) * completness + yc;
      double zn = (zd - zc) * completness + zc;
      
      // angles probs should be interpolated over spherical coords but hey this will work.
      double rn = (rd - rc) * completness + rc;
      double pn = (pd - pc) * completness + pc;
      double an = (ad - ac) * completness + ac;

      RCLCPP_INFO(get_node()->get_logger(), "\033[34m PathPlanning:\033[0m comp: %.3f| xn=%.3f, yn=%.3f, zn=%.3f, rn=%.3f, pn=%.3f, an=%.3f",completness, xn,yn,zn, rn,pn,an );

      nextStep = KDL::Frame((KDL::Rotation::RotZ(an) * KDL::Rotation::RotY(pn) * KDL::Rotation::RotX(rn)), KDL::Vector(xn, yn, zn));
    }
    else
    {
    } // feel free to implement something better
  }

  int MyController_class::InverseK(KDL::Frame target, KDL::JntArray &result)
  {

    KDL::JntArray initial_guess(num_joints);
    initial_guess = q_;
    //initial_guess = joint_center_;


    #ifdef DEBUG_IK
    std::string output;
    for (int i = 0; i < num_joints; ++i)
      {output += "J" + std::to_string(i) + ": " + std::to_string(initial_guess(i)) + ", ";}
    RCLCPP_INFO(get_node()->get_logger(), "Initial guess: %s", output.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "Target: x: %f, y: %f, z: %f", target.p.x(), target.p.y(), target.p.z());
    double roll, pitch, yaw;
    target.M.GetRPY(roll, pitch, yaw);
    RCLCPP_INFO(get_node()->get_logger(), "Target: roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);
    #endif


    KDL::Frame ee_frame;
    int fk_result = fk_solver_->JntToCart(q_, ee_frame);

    if (fk_result >= 0)
    {
      // Success — ee_frame now contains EE pose
      double x = ee_frame.p.x();
      double y = ee_frame.p.y();
      double z = ee_frame.p.z();

    #ifdef DEBUG_IK
     double roll, pitch, yaw;
      ee_frame.M.GetRPY(roll, pitch, yaw);
      RCLCPP_INFO(get_node()->get_logger(), "EE pose: x=%.3f y=%.3f z=%.3f roll=%.3f pitch=%.3f yaw=%.3f",
                  x, y, z, roll, pitch, yaw);
    #endif

    } 
    else
    {
      RCLCPP_ERROR(get_node()->get_logger(), "FK solver failed with error code %d", fk_result);
      ee_frame = KDL::Frame::Identity();
    }

    // EE dist heuristics
    double frameDiff = compareFrames(ee_frame,target); 
    double tolerance = std::min((frameDiff/5)*(frameDiff/5),frameDiff/50);
    #ifdef DEBUG_IK
      RCLCPP_INFO(get_node()->get_logger(), "\033[31m IK: \033[0m selected IK tolerance %e", tolerance);
    #endif
    miscData[5] = frameDiff;
    miscData[6] = tolerance;

      ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
        kdl_chain_,
        joint_min_limits_,
        joint_max_limits_,
        *fk_solver_,
        *ik_vel_solver_,
        1600, // Max iterations  default: 400
        tolerance  // Tolerance default: 1e-5 1e-2
      );
    int ret = ik_solver_->CartToJnt(initial_guess, target, result);
    miscData[8] = 0; 
    //RCLCPP_INFO(get_node()->get_logger(), "IK: CartToJnt returned %d", ret);
    if(ret != 0){
      
      initial_guess = joint_center_;
      
      #ifdef DEBUG_IK
        RCLCPP_WARN(get_node()->get_logger(), "\033[31m IK: \033[33m joint position as initial guess failed with code \033[0m %d, trying mid joint position", ret);
        output = "";
        for (int i = 0; i < num_joints; ++i)
          {output += "J" + std::to_string(i) + ": " + std::to_string(initial_guess(i)) + ", ";}
        RCLCPP_INFO(get_node()->get_logger(), "New initial guess: %s", output.c_str());
      #endif
      
      ret = ik_solver_->CartToJnt(initial_guess, target, result);
      miscData[8] = 1;
      if(ret != 0){
        #ifdef DEBUG_IK
        RCLCPP_WARN(get_node()->get_logger(), "\033[31m IK: \033[31m ik fallback failed with \033[0m %d returing", ret);
        #endif
        miscData[8] = 2;
      }

    }


    KDL::Frame ik_ee_result;
    fk_result = fk_solver_->JntToCart(result, ik_ee_result);
    if(fk_result != 0){
      ik_ee_result = KDL::Frame::Identity();
    }
    double ik_ee_target_diff = compareFrames(ik_ee_result,target); 
    miscData[7] = ik_ee_target_diff;


    return ret;
  }

  double MyController_class::compareFrames(KDL::Frame a, KDL::Frame b)
  {
    double posDiff = (a.p - b.p).Norm();
    KDL::Rotation R_diff = a.M.Inverse() * b.M;
    KDL::Vector axis;
    double angle = R_diff.GetRotAngle(axis);  // This returns the angle, and sets the axis
    double rotDiff = std::abs(angle);

    return posDiff + rotDiff;

  }


  void MyController_class::ex3_smarterControllers(int controllerType)
  {

    KDL::Frame tsop_kdlFrame;
    if (!taskspace_objective_point.transforms.empty())
    {
      KDL::Vector position(
          taskspace_objective_point.transforms[0].translation.x,
          taskspace_objective_point.transforms[0].translation.y,
          taskspace_objective_point.transforms[0].translation.z);
      KDL::Rotation orientation = KDL::Rotation::Quaternion(
          taskspace_objective_point.transforms[0].rotation.x,
          taskspace_objective_point.transforms[0].rotation.y,
          taskspace_objective_point.transforms[0].rotation.z,
          taskspace_objective_point.transforms[0].rotation.w);
      tsop_kdlFrame = KDL::Frame(orientation, position);
        miscData[1] = position.data[0];
        miscData[2] = position.data[1];
        miscData[3] = position.data[2];

    }
    else
    {
      RCLCPP_WARN(get_node()->get_logger(), "No transform received yet — skipping frame conversion");
      tsop_kdlFrame = KDL::Frame(
          KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0), // Identity orientation
          KDL::Vector(1.0, 1.0, 1.0)                     // Position (x=1, y=2, z=3)
      );
      // return;
    }

    switch (controllerType)
    {

    case 3: // jointSpaceController
    {
      KDL::Frame target = tsop_kdlFrame;
      
      //setup necesities for torque controller
      double w = 2; //rad/s
      double damp = 1; // relative
      // Kp_.data = (w*w) * (Eigen::VectorXd(7) << 1.0,1.2,1.4,1.6,1.8,2.0,2.2).finished();
      // Kd_.data = (w*damp*2) * (Eigen::VectorXd(7) << 1.0,1.2,1.4,1.6,1.8,2.0,2.2).finished();
      Kp_.data = (w*w) * (Eigen::VectorXd(7) << 2.2,2.0,1.8,1.6,1.4,1.2,1.0).finished();
      Kd_.data = (w*damp*2) * (Eigen::VectorXd(7) << 2.2,2.0,1.8,1.6,1.4,1.2,1.0).finished();
      // Kp_.data.setConstant(w*w);
      // Ki_.data.setConstant(0);
      // Kd_.data.setConstant(w*damp*2);
      qd_dot_.data.setConstant(0);
      
      
      
      double TaskSpaceError =  compareFrames(lastTarget,target);
      miscData[0] = TaskSpaceError;
      lastTarget = target;
      if(TaskSpaceError > 1e-3) //run IK only when the objective point changes
      {
        KDL::JntArray result(num_joints);
        int ret = InverseK(target, result);
        if (ret != 0)
        {
          //#ifdef DEBUG_EX3
            RCLCPP_WARN(get_node()->get_logger(), "ex3_smarterControllers.3: IK failed, breaking");
          //#endif
          break;
        }
        for (int i = 0; i < num_joints; i++)
        {
          #ifdef DEBUG_EX3
            RCLCPP_INFO(get_node()->get_logger(), "ex3_smarterControllers.3: IK success, joint %d = %f", i + 1, result(i));
          #endif
          qd_(i) = result(i);
        }
      }
      
      

    }
    break;

    default:
      for (int i = 0; i < num_joints; i++)
      {
        qd_(i) = qd_(i);
        qd_dot_(i) = qd_dot_(i);
        qd_ddot_(i) = qd_ddot_(i);
      }
      break;
    }
  }


  // adds to qd_dot as output
  // if future position is predicted to be between limit position and safety limit, the desiered speed is modified such that the future position should be half of it predicted.
  void MyController_class::jointLimitRepulse()
  {
    
    //KDL::JntArray q_z(num_joints);
    double safetyLimit_deg =  10.0;//10;
    double safetyLimit_rad = safetyLimit_deg*(3.14/180.0);


    double viscoseConstant = 10;

    for(int i = 0; i < num_joints; i++)
    {

    

      // double dist_max = std::abs(position_lim_MAX[i] -q_z(i));
      // double dist_min = std::abs(position_lim_MIN[i] -q_z(i));
      double distLim, viscosity, repulse,direction;
      if (qdot_(i) > 0) {
            direction = 1.0;
            distLim = q_(i) - (position_lim_MAX[i] - safetyLimit_rad);
      } 
      else if (qdot_(i) < 0) {
            direction = -1.0;
            distLim = -q_(i) + (position_lim_MIN[i] + safetyLimit_rad);
      } 
      else {
            direction = 0.0;
            distLim = 0.0;
      }

      // distLim being positive means the field is active
      if (distLim > 0) {
          viscosity = (distLim * viscoseConstant);
      } 
      else {
          viscosity = 0.0;
      }

      if (qdot_(i) * direction > 0) {  // heading into the limit
          repulse = (-1)*((qdot_(i) * qdot_(i))*direction + qdot_(i))   * viscosity;
      }

      qd_dot_(i) = repulse;

      if(i == 0){
        miscData[12] = distLim;
        miscData[13] = direction;
        miscData[14] = viscosity;
        miscData[15] = repulse;

      }

    }
    // miscData[12] = q_z(0);
    // miscData[13] = qd_dot_(0);
    

  }

  void MyController_class::taskSpaceRepulse(){
    KDL::Frame ee_frame;
    int fk_result = fk_solver_->JntToCart(q_, ee_frame);
    if (fk_result != 0)
    {
      RCLCPP_WARN(get_node()->get_logger(), "taskSpaceRepulse: FK failed for some reason");
      return;
    }

    Eigen::Vector3d EE(ee_frame.p.x(),ee_frame.p.y(),ee_frame.p.z());

    miscData[18] = ee_frame.p.x();
    miscData[19] = ee_frame.p.y();
    miscData[20] = ee_frame.p.z();




    Eigen::Vector3d totalRepulse(0,0,0);

    for (size_t i = 0; i < repulsionPoints.size(); ++i) {
      const auto& rp = repulsionPoints[i];

      //dist
      double dist = (EE-rp.x).norm();

      Eigen::Vector3d repulseVector(0,0,0);

      if(dist > rp.dist0)
      {
        // turn off for debuggins
        //continue;
        
      }
      else{
        double U = 0.5*rp.b *(1.0/dist-1.0/rp.a)*rp.k ; 
        
        if((EE-rp.x).squaredNorm()<= 0)
        {
          RCLCPP_WARN(get_node()->get_logger(), "taskSpaceRepulse: distance to point %d is zero, cannot repulse.",i);
          continue;
        }
         repulseVector = (EE-rp.x).normalized(); // should point to EE from center of repulse point (= correct repulse direction (hopefuly))
        
        repulseVector *= U;
        totalRepulse +=repulseVector;

      }


      int pointOfInterest = 2;
      if(i == pointOfInterest){
        miscData[24] = i;
        miscData[25] = dist;

        miscData[26] = rp.x.x();
        miscData[27] = rp.x.y();
        miscData[28] = rp.x.z();

        miscData[29] = repulseVector.x();
        miscData[30] = repulseVector.y();
        miscData[31] = repulseVector.z();

        miscData[32] = repulseVector.norm();
      }


    }

    for (size_t i = 0; i < repulsionPlanes.size(); ++i) {
      const auto& rplane = repulsionPlanes[i];
      ////RCLCPP_WARN(get_node()->get_logger(), "taskSpaceRepulse: repulsionPlanes.size %d ",repulsionPlanes.size());
      double dist = (EE - rplane.centerPoint.x).dot(rplane.direction);
      Eigen::Vector3d repulseVector(0,0,0);

      Eigen::Vector3d Qprojected(0,0,0);
      Eigen::Vector3d r(0,0,0);

      Eigen::Vector2d planeCordsProjected(0,0);

      if (dist > rplane.centerPoint.dist0)
      {
        // turn off for debuggins
        //continue;

      }else{
        // check for limits tho.
        // TODO
        Qprojected = EE - (dist * rplane.direction);
        r = Qprojected - rplane.centerPoint.x;

        
        planeCordsProjected(0) = r.dot(rplane.planeX);
        planeCordsProjected(1) = r.dot(rplane.planeY);
        
        if( std::min(rplane.l1.x(),rplane.l2.x()) < planeCordsProjected(0) && planeCordsProjected(0) < std::max(rplane.l1.x(),rplane.l2.x())   &&
            std::min(rplane.l1.y(),rplane.l2.y()) < planeCordsProjected(1) && planeCordsProjected(1) < std::max(rplane.l1.y(),rplane.l2.y())   )
        {
          // not needed for linear repulsion
          if(dist < 1e-3){
            dist = 1e-3;
          }

          double U = 0.5*rplane.centerPoint.b *(1.0/dist-1.0/rplane.centerPoint.a)*rplane.centerPoint.k; 
          
          // linear
          //double U = (rplane.centerPoint.dist0-dist)/rplane.centerPoint.dist1*rplane.centerPoint.k;

          repulseVector = rplane.direction;
          repulseVector *= U;
          totalRepulse +=repulseVector;

          



        }
        else{} // did not lie in the rect

        

      }

      int planeOfInterest = 0;
      if(i == planeOfInterest){
        miscData[34] = i;
        miscData[35] = dist;

        miscData[36] = Qprojected.x();
        miscData[37] = Qprojected.y();
        miscData[38] = Qprojected.z();

        miscData[39] = repulseVector.x();
        miscData[40] = repulseVector.y();
        miscData[41] = repulseVector.z();


        miscData[42] = repulseVector.norm();

        miscData[43] = planeCordsProjected(0);
        miscData[44] = planeCordsProjected(1);
        miscData[45] = r.norm();

        miscData[46] = rplane.centerPoint.dist0;
        miscData[47] = rplane.centerPoint.dist1;
      }


    
    }

    KDL::Jacobian J(num_joints);
    jac_solver->JntToJac(q_, J);
    Eigen::MatrixXd J_eigen = J.data;

    Eigen::Matrix<double,6,1> wrench;
    wrench.setZero();
    wrench.head<3>() = totalRepulse; // linear force only
    
    Eigen::VectorXd JtF = J_eigen.transpose() * wrench;

    qd_dot_.data += JtF;

    miscData[21] = totalRepulse.x();
    miscData[22] = totalRepulse.y();
    miscData[23] = totalRepulse.z();


  }

  // it is static but it cannot be said such
  void MyController_class::taskSpaceRepulse_calcRepulseCoefs(pointRep &point){
    // assuming the linear version : U = 1/2 *b1*(1./d_-1/a1); 
    //a1= x0
    //b1= (x0*x1)/(x0 - x1)
    //x1_10 = 1/(1/a1 + 10/b1)
    point.a = point.dist0;
    point.b = (point.dist0*point.dist1)/((point.dist0-point.dist1));
    point.dist10 = 1/(1/point.a + 10/point.b);



  }
  // in on_activate
  void MyController_class::taskSpaceRepulse_initHardcodedStuff(){
    repulsionPoints.emplace_back(Eigen::Vector3d(0.3, 0, 1.8), 0.4, 0.2, 0);
    repulsionPoints.emplace_back(Eigen::Vector3d(-0.3, 0, 1.8), 0.4, 0.2, 0);
    repulsionPoints.emplace_back(Eigen::Vector3d(-0.3, 0.3, 1.8), 0.4, 0.2, 0);

    pointRep rp = pointRep(Eigen::Vector3d(0.2, 0, 1.8), 0.1, 0.05, 15);
    repulsionPlanes.emplace_back(rp,Eigen::Vector3d(-1, 0, 0),Eigen::Vector3d(0, 1, 0), Eigen::Vector2d(-1,-1), Eigen::Vector2d(1,1));



    //debug repulsionPlanes

    for(int i = 0; i< repulsionPlanes.size();i++)
    {
      const auto& rPl = repulsionPlanes[i];
      RCLCPP_INFO(get_node()->get_logger(), "\033[31m repulsion plane\033[0m %d: \033[32m dist0:\033[0m %3.2f, \033[32m dist1:\033[0m %3.2f, \033[32m dist10:\033[0m %3.2f,"
      "\033[32m a:\033[0m%3.2f, \033[32m b:\033[0m%3.2f, \033[32m k:\033[0m%3.2f, "
      "\033[31m planeX vector:\033[0m[ %3.3f; %3.3f; %3.3f ], \033[31m planeY vector:\033[0m[ %3.3f; %3.3f; %3.3f ] ",
        i , rPl.centerPoint.dist0,rPl.centerPoint.dist1,rPl.centerPoint.dist10,
        rPl.centerPoint.a,rPl.centerPoint.b,rPl.centerPoint.k ,
        rPl.planeX.x(),rPl.planeX.y(),rPl.planeX.z(), rPl.planeY.x(),rPl.planeY.y(),rPl.planeY.z()
      );

    }

  }

  void MyController_class::ex5_potentialFields(){
    jointLimitRepulse(); //assumes 1kHz refresh rate
    taskSpaceRepulse();


  }

} // namespace MyController_namespace

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(MyController_namespace::MyController_class,
                       controller_interface::ControllerInterface)