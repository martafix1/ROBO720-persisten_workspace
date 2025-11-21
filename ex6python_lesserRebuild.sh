#!/bin/bash

set -e  # Exit immediately if any command fails

clear

colcon build --packages-select pathing_py6
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run pathing_py6 cutPizza