#!/bin/bash

echo "=== Mobile Lift Arm Robot Control ==="
echo "1. Tricycle Base Control"
echo "2. Lift Control" 
echo "3. Arm Control"
echo "4. Combined Movements"

case $1 in
  "base")
    echo "Moving base forward..."
    ros2 topic pub /tricycle_drive_controller/cmd_vel geometry_msgs/Twist \
    "{linear: {x: 0.5}, angular: {z: 0.0}}" --once
    ;;
  "steer")  
    echo "Steering left..."
    ros2 topic pub /tricycle_drive_controller/cmd_vel geometry_msgs/Twist \
    "{linear: {x: 0.3}, angular: {z: 0.5}}" --once
    ;;
  "lift_up")
    echo "Lifting to 0.8m..."
    ros2 topic pub /lift_position_controller/commands std_msgs/Float64MultiArray \
    "data: [0.8]" --once
    ;;
  "lift_down")
    echo "Lowering to 0.1m..."
    ros2 topic pub /lift_position_controller/commands std_msgs/Float64MultiArray \
    "data: [0.1]" --once
    ;;
  "arm_home")
    echo "Moving arm to home position..."
    ros2 action send_goal /arm_trajectory_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory \
    '{trajectory: {
        joint_names: ["arm_shoulder_yaw_joint","arm_shoulder_pitch_joint","arm_shoulder_roll_joint",
                      "arm_elbow_pitch_joint","arm_wrist_pitch_joint","arm_wrist_roll_joint"],
        points: [
          {positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}
        ]}}'
    ;;
  "demo")
    echo "Running demo sequence..."
    ./control.sh lift_up
    sleep 4
    ./control.sh arm_home  
    sleep 4
    ./control.sh base
    ;;
  *)
    echo "Usage: ./control.sh [base|steer|lift_up|lift_down|arm_home|demo]"
    ;;
esac
