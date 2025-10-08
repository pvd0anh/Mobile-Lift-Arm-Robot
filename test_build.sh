#!/bin/bash

echo "=== Testing Mobile Lift Arm Robot Build ==="
echo "Cleaning previous build..."
rm -rf build install log

echo "Building workspace..."
colcon build --symlink-install

if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    echo "Sourcing workspace..."
    source install/setup.bash
    
    echo "Testing URDF conversion..."
    ros2 run xacro xacro src/mobile_lift_arm/urdf/mobile_lift_arm.urdf.xacro > /tmp/test_robot.urdf
    
    if [ $? -eq 0 ]; then
        echo "✅ URDF generation successful!"
        echo "You can now run: ros2 launch mobile_lift_arm sim.launch.py"
    else
        echo "❌ URDF generation failed!"
        exit 1
    fi
else
    echo "❌ Build failed!"
    exit 1
fi

echo "=== Testing Complete ==="