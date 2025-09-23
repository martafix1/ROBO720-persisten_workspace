#!/bin/bash

clear

colcon build --packages-skip franka_custom_controllers KinematicsEX1
source install/setup.bash
ros2 launch franka_cust_control_2 controller.launch.py