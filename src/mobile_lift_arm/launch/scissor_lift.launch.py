import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_scissor_lift = get_package_share_directory('scissor_lift_description')

    # URDF file path
    urdf_file = os.path.join(pkg_scissor_lift, 'urdf', 'scissor_lift.urdf.xacro')

    # Robot description
    robot_description = Command(['xacro ', urdf_file])

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint state publisher GUI (optional)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': 'empty.world'}.items()
    )

    # Spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'scissor_lift'],
        output='screen'
    )

    # Load controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[os.path.join(pkg_scissor_lift, 'config', 'scissor_lift_controllers.yaml')],
        output='screen'
    )

    # Load controllers
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    load_hydraulic_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hydraulic_position_controller'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        gazebo,
        spawn_entity,
        controller_manager,
        load_joint_state_broadcaster,
        load_hydraulic_controller
    ])
