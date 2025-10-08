#!/bin/bash

# Remove build artifacts
rm -rf build/ install/ log/

# Build the package
colcon build --packages-select scissor_lift_description

# Source the workspace
source install/setup.bash

# Choose launch file based on argument
if [ "$1" = "gazebo" ]; then
    echo "Launching with Gazebo..."
    ros2 launch scissor_lift_description gazebo.launch.py
else
    echo "Launching with RViz only..."
    ros2 launch scissor_lift_description display.launch.py
fi
