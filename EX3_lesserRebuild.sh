#!/bin/bash

set -e  # Exit immediately if any command fails

clear

colcon build --packages-select franka_cc_3 pathing_py
source install/setup.bash
ros2 launch ./src_ROBO720/EX3/franka_cc_3/launch/controller.launch.py 