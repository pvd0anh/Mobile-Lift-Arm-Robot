# Twist
ros2 topic pub /diff_drive_base_controller/cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"

# prismatic
# Set position (m) cho lift_joint qua group position controller
ros2 topic pub /lift_position_controller/commands std_msgs/Float64MultiArray "data: [0.6]"

ros2 topic pub /lift_position_controller/commands std_msgs/Float64MultiArray "data: [0.1]"


# trajectory arm
ros2 action send_goal /arm_trajectory_controller/follow_joint_trajectory \
control_msgs/action/FollowJointTrajectory \
'{trajectory: {joint_names: ["arm_shoulder_yaw_joint","arm_shoulder_pitch_joint","arm_elbow_joint"],
points: [{positions: [0.0, 0.7, -0.5], time_from_start: {sec: 2}},
         {positions: [1.2, 0.2, 0.8],  time_from_start: {sec: 5}}]}}'
