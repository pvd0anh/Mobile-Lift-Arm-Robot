# sudo apt update
# sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-ros2-control ros-humble-ros2-controllers \
#                  ros-humble-diff-drive-controller ros-humble-joint-trajectory-controller \
#                  ros-humble-position-controllers

# Build
# cd ~/ros2_ws
killall gzserver
killall gzclient
rm -rf build install log
colcon build --symlink-install
source install/setup.bash

xacro $(ros2 pkg prefix mobile_lift_arm)/share/mobile_lift_arm/urdf/mobile_lift_arm.urdf.xacro > /tmp/robot.urdf


ros2 launch mobile_lift_arm sim.launch.py
