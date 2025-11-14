#!/bin/bash

set -e  # Exit immediately if any command fails

clear

colcon build --packages-select franka_cc_6 pathing_py6
source install/setup.bash

#konsole --hold -e "ros2 run pathing_py node3"

ros2 launch ./src_ROBO720/EX6/franka_cc_6/launch/controller.launch.py 