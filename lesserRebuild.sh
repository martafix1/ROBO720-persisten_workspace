#!/bin/bash

clear

colcon build --packages-select franka_cust_control_2 fuck_it_we_move_py
source install/setup.bash
ros2 launch franka_cust_control_2 controller.launch.py