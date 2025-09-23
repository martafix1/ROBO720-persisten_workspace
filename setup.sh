#!/bin/bash

# Source the pre-built franka simulation workspace
source /edu-franka_simulation_ws/install/setup.bash

# Source your custom workspace (if it's been built)
if [ -f "/persistent_workspace/install/setup.bash" ]; then
    source /persistent_workspace/install/setup.bash
    echo "✓ Both franka and custom workspaces sourced"
else
    echo "✓ Franka workspace sourced (run 'colcon build' to build your custom packages)"
fi