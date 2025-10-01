#!/bin/bash

clear

colcon build --packages-select franka_CC_3
source install/setup.bash
ros2 launch ./src_ROBO720/EX3/franka_CC_3/launch/controller.launch.py 