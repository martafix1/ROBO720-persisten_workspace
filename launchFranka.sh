#!/bin/bash
#CONTROLER_NAME="computed_torque_controller"
# CONTROLER_NAME=time_delay_controller
CONTROLER_NAME="joint_velocity_example_controller"

echo -e "\033[0;31m make sure to have the \033[1;33m xhost +local:docker \033[0;31m done \033[0m"

cd /edu-franka_simulation_ws/
colcon build
source install/setup.bash
ros2 launch franka_gazebo $CONTROLER_NAME.launch.py