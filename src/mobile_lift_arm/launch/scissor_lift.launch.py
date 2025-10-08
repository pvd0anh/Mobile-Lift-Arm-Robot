"""Launch file for the scissor lift simulation.

This launch description brings up a Gazebo simulation of the scissor
lift along with the ROS2 control node and the necessary controllers.
It has been refactored to remove incorrect indentation present in
the original repository.  Paths are resolved using
``ament_index_python`` so that the package can be installed and
relocated cleanly.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command


def generate_launch_description() -> LaunchDescription:
    # Resolve package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_scissor_lift = get_package_share_directory('scissor_lift_control')

    # URDF file path (xacro)
    urdf_file = os.path.join(pkg_scissor_lift, 'urdf', 'scissor_lift.urdf.xacro')

    # Build the robot_description parameter by running xacro
    robot_description = Command(['xacro ', urdf_file])

    # Node: publish robot state
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    # Optional joint state publisher GUI for interactive viewing
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    # Include the standard Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': 'empty.world'}.items(),
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'scissor_lift'],
        output='screen',
    )

    # Start the controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[os.path.join(pkg_scissor_lift, 'config', 'scissor_lift_controllers.yaml')],
        output='screen',
    )

    # Load controllers (these spawner nodes will connect to the manager)
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    load_hydraulic_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hydraulic_position_controller'],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        gazebo,
        spawn_entity,
        controller_manager,
        load_joint_state_broadcaster,
        load_hydraulic_controller,
    ])