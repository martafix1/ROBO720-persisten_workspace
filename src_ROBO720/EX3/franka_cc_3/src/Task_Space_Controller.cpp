// Task_Space_Controller.cpp
// Implements an operational-space (task-space) torque controller

#include "task_space_controller.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <kdl/frames_io.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/frames.hpp>

#include <rclcpp/parameter_client.hpp>
#include <rclcpp/qos.hpp>

#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace arm_controllers
{

  using std::placeholders::_1;


  controller_interface::InterfaceConfiguration
  Task_Space_Controller::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto &name : joint_names_)
    {
      config.names.push_back(name + "/effort");
    }
    return config;
  }

  controller_interface::InterfaceConfiguration
  Task_Space_Controller::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto &name : joint_names_)
    {
      config.names.push_back(name + "/position");
      config.names.push_back(name + "/velocity");
    }
    return config;
  }

  // on_init
  CallbackReturn Task_Space_Controller::on_init()
  {
    RCLCPP_INFO(get_node()->get_logger(), "Task_Space_Controller::on_init()");

    // Default joint names (you can override via parameter later)
    joint_names_.clear();
    joint_names_.push_back("panda_joint1");
    joint_names_.push_back("panda_joint2");
    joint_names_.push_back("panda_joint3");
    joint_names_.push_back("panda_joint4");
    joint_names_.push_back("panda_joint5");
    joint_names_.push_back("panda_joint6");
    joint_names_.push_back("panda_joint7");

    // Initialize KDL arrays
    q_.resize(num_joints);
    qdot_.resize(num_joints);
    qd_.resize(num_joints);
    qd_dot_.resize(num_joints);
    qd_ddot_.resize(num_joints);
    e_.resize(num_joints);
    e_dot_.resize(num_joints);
    e_int_.resize(num_joints);

    aux_d_.resize(num_joints);
    comp_d_.resize(num_joints);
    tau_d_.resize(num_joints);

    Kp_.resize(num_joints);
    Ki_.resize(num_joints);
    Kd_.resize(num_joints);

    Kp_task_.resize(6);
    Kd_task_.resize(6);

    // sensible defaults for task gains (can be replaced by parameters)
    for (int i = 0; i < 6; ++i)
    {
      Kp_task_(i) = 200.0;
      Kd_task_(i) = 40.0;
    }

    // Save data
    t = 0.0;
    have_jdot = false;

    // Initialize message containers
    msg_qd_.data.reserve(num_joints);
    msg_q_.data.reserve(num_joints);
    msg_e_.data.reserve(num_joints);
    msg_SaveData_.data.reserve(SaveDataMax);

    RCLCPP_INFO(get_node()->get_logger(), "Task_Space_Controller::on_init() done");
    return CallbackReturn::SUCCESS;
  }

  // on_configure

  CallbackReturn Task_Space_Controller::on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Task_Space_Controller::on_configure()");

    // Optionally read joint names from parameter server (if set)
    if (get_node()->has_parameter("joints"))
    {
      auto param = get_node()->get_parameter("joints");
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
      {
        joint_names_ = param.as_string_array();
        if (static_cast<int>(joint_names_.size()) != num_joints)
        {
          RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' must have %d entries", num_joints);
          return CallbackReturn::ERROR;
        }
      }
    }

    // Robot description (URDF) retrieval using AsyncParametersClient to robot_state_publisher
    try
    {
      auto parameters_client =
          std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
      if (!parameters_client->wait_for_service(std::chrono::milliseconds(500)))
      {
        RCLCPP_WARN(get_node()->get_logger(),
                    "robot_state_publisher parameter service not available; trying direct param on this node");
      }
      // Try both places: robot_state_publisher param first, then local param
      std::vector<rclcpp::Parameter> params;
      if (parameters_client->service_is_ready())
      {
        auto fut = parameters_client->get_parameters({"robot_description"});
        auto res = fut.get();
        if (!res.empty())
        {
          robot_description_ = res[0].value_to_string();
        }
      }

      if (robot_description_.empty() && get_node()->has_parameter("robot_description"))
      {
        robot_description_ = get_node()->get_parameter("robot_description").as_string();
      }
    }
    catch (const std::exception &ex)
    {
      (void)ex;
      RCLCPP_WARN(get_node()->get_logger(), "Could not fetch robot_description parameter: %s", ex.what());
    }

    if (robot_description_.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "robot_description is empty. You must set robot_description.");
      return CallbackReturn::ERROR;
    }

    // Build KDL tree from URDF
    if (!kdl_parser::treeFromString(robot_description_, kdl_tree_))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to construct KDL tree from URDF");
      return CallbackReturn::ERROR;
    }

    // Read root and tip link names from parameters or fallback to common Panda links
    std::string root_name = "panda_link0";
    std::string tip_name = "panda_link8";
    if (get_node()->has_parameter("root_link"))
      root_name = get_node()->get_parameter("root_link").as_string();
    if (get_node()->has_parameter("tip_link"))
      tip_name = get_node()->get_parameter("tip_link").as_string();

    // Get chain
    if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
    {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                          "Failed to get KDL chain from '" << root_name << "' to '" << tip_name << "'");
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_node()->get_logger(), "KDL chain: %s -> %s (joints: %d)",
                root_name.c_str(), tip_name.c_str(), kdl_chain_.getNrOfJoints());

    // Gravity vector
    gravity_ = KDL::Vector::Zero();
    gravity_(2) = -9.81;

    // KDL solvers
    try
    {
      id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
      fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
      jac_solver = std::make_shared<KDL::ChainJntToJacSolver>(kdl_chain_);
      jacdot_solver = std::make_shared<KDL::ChainJntToJacDotSolver>(kdl_chain_);
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create KDL solvers: %s", ex.what());
      return CallbackReturn::ERROR;
    }

    // Resize dynamics arrays
    M_.resize(num_joints);
    C_.resize(num_joints);
    G_.resize(num_joints);

    // Publishers (lifecycle publishers — create and activate in on_activate)
    pub_qd_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("qd", 10);
    pub_q_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("q", 10);
    pub_e_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("e", 10);
    pub_SaveData_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("SaveData", 10);

    // Subscription to task-space objective: MultiDOFJointTrajectory expected,
    // we take the first transform of the first point (if exists)
    taskspace_objective_subscriber = get_node()->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
        "/taskspace_objective",
        10,
        [this](const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg)
        {
          if (!msg || msg->points.empty() || msg->points[0].transforms.empty())
          {
            RCLCPP_WARN(get_node()->get_logger(), "Received empty taskspace objective");
            return;
          }
          const auto &tf = msg->points[0].transforms[0];
          KDL::Vector pos(tf.translation.x, tf.translation.y, tf.translation.z);
          KDL::Rotation rot = KDL::Rotation::Quaternion(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w);
          std::lock_guard<std::mutex> guard(req_traj_point_mutex_);
          Fd_ = KDL::Frame(rot, pos);
          // optionally reset path planner counters etc.
        });

    RCLCPP_INFO(get_node()->get_logger(), "Task_Space_Controller::on_configure() done");
    return CallbackReturn::SUCCESS;
  }

  // on_activate

  CallbackReturn Task_Space_Controller::on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Task_Space_Controller::on_activate()");

    // If lifecycle publishers were created as lifecycle ones by the node, activate them
    try
    {
      pub_qd_->on_activate();
      pub_q_->on_activate();
      pub_e_->on_activate();
      pub_SaveData_->on_activate();
    }
    catch (...)
    {
      // In some environments the publisher returned is not a lifecycle publisher;
      // ignore exceptions from on_activate to keep compatibility.
    }

    // initialize previous Jacobian for numerical Jdot fallback
    static bool Jprev_initialized = false;
    if (!Jprev_initialized)
    {
      KDL::Jacobian J(num_joints);
      jac_solver->JntToJac(q_, J); // q_ is probably zero initially
      J_prev = J;
      Jprev_initialized = true;
    }

    // zero integrator
    for (int i = 0; i < num_joints; ++i)
      e_int_(i) = 0.0;

    t = 0.0;
    have_jdot = false;

    RCLCPP_INFO(get_node()->get_logger(), "Task_Space_Controller activated");
    return CallbackReturn::SUCCESS;
  }

  // updateJointStates
  void Task_Space_Controller::updateJointStates()
  {
    // state_interfaces_ contains [pos0, vel0, pos1, vel1, ...] because state_interface_configuration
    if (state_interfaces_.size() != static_cast<size_t>(2 * num_joints))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Unexpected number of state interfaces: %zu (expected %d)",
                   state_interfaces_.size(), 2 * num_joints);
      return;
    }

    for (size_t i = 0; i < static_cast<size_t>(num_joints); ++i)
    {
      const auto &pos_if = state_interfaces_[2 * i];
      const auto &vel_if = state_interfaces_[2 * i + 1];

      q_(i) = pos_if.get_value();
      qdot_(i) = vel_if.get_value();
    }
  }

  // update (control loop)
  controller_interface::return_type Task_Space_Controller::update(
      const rclcpp::Time & /*time*/,
      const rclcpp::Duration &period)
  {
    // Basic timing
    double dt = period.seconds();
    if (dt <= 0.0)
      dt = 1e-3;
    t += dt;

    // Ensure correct number of command interfaces
    if (command_interfaces_.size() != static_cast<size_t>(num_joints))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Unexpected number of command interfaces: %zu (expected %d)",
                   command_interfaces_.size(), num_joints);
      return controller_interface::return_type::ERROR;
    }

    // 1) Read joint states
    updateJointStates();

    // 2) FK: current end-effector pose
    if (fk_solver_->JntToCart(q_, F_) < 0)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "FK solver failed");
      return controller_interface::return_type::ERROR;
    }

    // 3) Jacobian
    KDL::Jacobian J(num_joints);
    if (jac_solver->JntToJac(q_, J) < 0)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Jacobian computation failed");
      return controller_interface::return_type::ERROR;
    }

    // 4) Compute Jdot (try analytic solver; fallback to numerical)
    KDL::Jacobian Jdot(num_joints);
    have_jdot = false;
    if (jacdot_solver)
    {
      if (jacdot_solver->JntToJacDot(q_, qdot_, Jdot) == 0)
      {
        have_jdot = true;
      }
    }

    if (!have_jdot)
    {
      // numerical differentiation (using previous J_prev)
      for (unsigned r = 0; r < 6; ++r)
      {
        for (unsigned c = 0; c < static_cast<unsigned>(num_joints); ++c)
        {
          Jdot(r, c) = (J(r, c) - J_prev(r, c)) / dt;
        }
      }
    }
    J_prev = J; // save for next iter

    // 5) Task error (pose)
    KDL::Twist Xerr = KDL::diff(F_, Fd_); // 6D: (vx vy vz wx wy wz) small-error twist

    // 6) Current task velocity dx = J * qdot
    KDL::Twist dx = KDL::Twist::Zero();
    for (unsigned i = 0; i < 6; ++i)
    {
      for (unsigned j = 0; j < static_cast<unsigned>(num_joints); ++j)
      {
        dx[i] += J(i, j) * qdot_(j);
      }
    }

    // 7) Desired task velocity and accel (assuming zero feedforward)
    KDL::Twist Vd = KDL::Twist::Zero();
    KDL::Twist xd_ddot = KDL::Twist::Zero();

    // 8) Task-space PD: xr_ddot = xd_ddot + Kd*(Vd - dx) + Kp*(Xerr)
    KDL::Twist xr_ddot = KDL::Twist::Zero();
    for (int i = 0; i < 6; ++i)
    {
      double kp = Kp_task_(i);
      double kd = Kd_task_(i);
      xr_ddot[i] = xd_ddot[i] + kd * (Vd[i] - dx[i]) + kp * (Xerr[i]);
    }

    // 9) Compute Jdot*qdot
    KDL::Twist Jdot_qdot = KDL::Twist::Zero();
    for (unsigned i = 0; i < 6; ++i)
      for (unsigned j = 0; j < static_cast<unsigned>(num_joints); ++j)
        Jdot_qdot[i] += Jdot(i, j) * qdot_(j);

    // rhs = xr_ddot - Jdot*qdot
    Eigen::VectorXd rhs_e(6);
    for (int i = 0; i < 6; ++i)
      rhs_e(i) = xr_ddot[i] - Jdot_qdot[i];

    // 10) Convert Jacobian to Eigen
    Eigen::MatrixXd J_e = J.data;           // 6 x n
    Eigen::MatrixXd Jt_e = J_e.transpose(); // n x 6

    // Damped least squares inverse: ddq_r = J^T * (J * J^T + lambda I)^-1 * rhs
    double lambda = 1e-6;
    Eigen::MatrixXd JJt = J_e * Jt_e; // 6x6
    Eigen::MatrixXd reg = JJt + lambda * Eigen::MatrixXd::Identity(6, 6);

    Eigen::VectorXd sol6;
    // Solve regularized system (6x6)
    sol6 = reg.ldlt().solve(rhs_e);        // 6x1
    Eigen::VectorXd ddq_r_e = Jt_e * sol6; // n x 1

    // 11) Dynamics: M, C, G
    id_solver_->JntToMass(q_, M_);
    id_solver_->JntToCoriolis(q_, qdot_, C_);
    id_solver_->JntToGravity(q_, G_);

    Eigen::MatrixXd M_e = M_.data; // n x n
    Eigen::VectorXd C_e = C_.data; // n
    Eigen::VectorXd G_e = G_.data; // n

    // 12) Computed torque
    Eigen::VectorXd tau_e = M_e * ddq_r_e + C_e + G_e;

    // 13) Send torques to command interfaces
    for (size_t i = 0; i < static_cast<size_t>(num_joints); ++i)
    {
      command_interfaces_[i].set_value(tau_e(static_cast<int>(i)));
    }

    // 14) Publish debugging arrays
    msg_qd_.data.clear();
    msg_q_.data.clear();
    msg_e_.data.clear();
    msg_SaveData_.data.clear();

    for (int i = 0; i < num_joints; ++i)
    {
      msg_qd_.data.push_back(qd_(i)); // in our design qd_ is unused here, but keep for visibility
      msg_q_.data.push_back(q_(i));
      msg_e_.data.push_back((double)Xerr[i < 3 ? i : (i - 3)]); // approximate 6->3/rot split, keep size manageable
    }

    // Save some data into SaveData_ (you can extend as needed)
    SaveData_[0] = t;
    for (int i = 0; i < num_joints && i + 1 < SaveDataMax; ++i)
      SaveData_[i + 1] = q_(i);

    for (int i = 0; i < SaveDataMax; ++i)
      msg_SaveData_.data.push_back(SaveData_[i]);

    // publish (lifecycle publishers may need activation; safe to call publish)
    pub_qd_->publish(msg_qd_);
    pub_q_->publish(msg_q_);
    pub_e_->publish(msg_e_);
    pub_SaveData_->publish(msg_SaveData_);

    return controller_interface::return_type::OK;
  }

} // namespace arm_controllers

PLUGINLIB_EXPORT_CLASS(arm_controllers::Task_Space_Controller,
                       controller_interface::ControllerInterface)
