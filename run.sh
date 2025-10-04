sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-ros2-control ros-humble-ros2-controllers \
                 ros-humble-diff-drive-controller ros-humble-joint-trajectory-controller \
                 ros-humble-position-controllers

# Build
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

ros2 launch mobile_lift_robot sim.launch.py
