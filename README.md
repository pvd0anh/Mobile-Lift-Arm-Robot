# Mobile Lift Arm Robot

A ROS 2 simulation project featuring a mobile robot integrated with a scissor lift system and a 6-DOF robotic arm in Gazebo environment using ROS 2 Humble.

## Overview

The Mobile Lift Arm Robot consists of three main components:
- **4-wheel mobile base** with differential drive system
- **Scissor lift mechanism** capable of vertical movement from 0-1.2m
- **6-DOF robotic arm** mounted on the lifting platform

## Project Structure

```
Mobile-Lift-Arm-Robot/
├── src/mobile_lift_arm/           # Main ROS2 package
│   ├── urdf/                      # Robot URDF/Xacro definitions
│   │   ├── mobile_lift_arm.urdf.xacro  # Main URDF file
│   │   ├── base_vehicle.xacro          # Mobile base
│   │   ├── scissor_lift.xacro          # Scissor lift system
│   │   └── robot_arm.xacro             # 6-DOF robot arm
│   ├── launch/                    # Launch files
│   │   └── sim.launch.py         # Simulation launcher
│   ├── config/                    # Controller configurations
│   ├── meshes/                    # 3D models
│   └── backup/                    # Backup files
├── run.sh                         # Simulation startup script
└── control.sh                     # Robot control script
```

## System Requirements

- **ROS 2 Humble**
- **Gazebo Classic**
- **Ubuntu 22.04**

### ROS 2 Dependencies

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-ros2-control ros-humble-ros2-controllers \
                 ros-humble-diff-drive-controller ros-humble-joint-trajectory-controller \
                 ros-humble-position-controllers
```

## Installation and Usage

### 1. Clone Repository

```bash
git clone https://github.com/pvd0anh/Mobile-Lift-Arm-Robot.git
cd Mobile-Lift-Arm-Robot
```

### 2. Build Workspace

```bash
chmod +x run.sh
./run.sh
```

Or build manually:

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch Simulation

```bash
ros2 launch mobile_lift_arm sim.launch.py
```

## Robot Control

Use the `control.sh` script or direct ROS 2 commands:

### Mobile Base Movement

```bash
ros2 topic pub /tricycle_drive_controller/cmd_vel geometry_msgs/Twist \
"{linear: {x: 0.5}, angular: {z: 0.3}}"
```

### Lift System Control

Raise to 0.6m:
```bash
ros2 topic pub /lift_position_controller/commands std_msgs/Float64MultiArray "data: [0.6]"
```

Lower to 0.1m:
```bash
ros2 topic pub /lift_position_controller/commands std_msgs/Float64MultiArray "data: [0.1]"
```

### Robot Arm Control

```bash
ros2 action send_goal /arm_trajectory_controller/follow_joint_trajectory \
control_msgs/action/FollowJointTrajectory \
'{trajectory: {
    joint_names: ["arm_shoulder_yaw_joint","arm_shoulder_pitch_joint","arm_shoulder_roll_joint",
                  "arm_elbow_pitch_joint","arm_wrist_pitch_joint","arm_wrist_roll_joint"],
    points: [
      {positions: [0.0, 0.6, 0.0,  -0.8, 0.4, 0.0], time_from_start: {sec: 2}},
      {positions: [1.2, 0.2, -0.4,  0.6, 0.2, 1.0], time_from_start: {sec: 5}}
    ]}}'
```

## Technical Specifications

### Mobile Base
- **Type**: Differential drive with 4 wheels
- **Controller**: `diff_drive_controller`
- **Velocity limits**: ±10 rad/s per wheel

### Lift System
- **Type**: Prismatic joint (scissor mechanism)
- **Travel range**: 0.0 - 1.2m
- **Controller**: `position_controllers`

### Robot Arm
- **DOF**: 6 degrees of freedom
- **Joints**:
  - `arm_shoulder_yaw_joint`: ±6.28 rad (360°)
  - `arm_shoulder_pitch_joint`: ±1.57 rad (90°)
  - `arm_shoulder_roll_joint`: ±1.57 rad (90°)
  - `arm_elbow_pitch_joint`: ±1.57 rad (90°)
  - `arm_wrist_pitch_joint`: ±3.14 rad (180°)
  - `arm_wrist_roll_joint`: ±6.28 rad (360°)
- **Controller**: `joint_trajectory_controller`

## ROS 2 Controllers

The project utilizes the following controllers:
- `joint_state_broadcaster`: Publishes joint states
- `diff_drive_controller`: Controls mobile base movement
- `lift_position_controller`: Controls lift position
- `arm_trajectory_controller`: Controls arm trajectory execution

## Features

- **Modular design** with separate URDF/Xacro files for each component
- **ROS 2 Control integration** for real-time robot control
- **Gazebo simulation** with physics-based modeling
- **Multi-DOF coordination** between base, lift, and arm systems
- **Position and trajectory control** capabilities

## Development Environment

- Ubuntu 22.04.5
- ROS 2 Humble
- Gazebo Fortress

## Author

**Maintainer**: [pvdoanh.ce@gmail.com](mailto:pvdoanh.ce@gmail.com)
