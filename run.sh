rm -rf ./build/ ./install/ ./log/
colcon build
source install/setup.bash
ros2 launch scissor_lift_description display.launch.py
