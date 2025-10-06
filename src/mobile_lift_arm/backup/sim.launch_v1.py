from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('mobile_lift_arm')
    urdf_path = os.path.join(pkg, 'urdf', 'mobile_lift_arm.urdf.xacro')
    controllers_yaml = os.path.join(pkg, 'config', 'ros2_controllers.yaml')

    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world', default_value='empty', description='Gazebo world name'
    )

    # Launch Gazebo Classic (server + client)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'verbose': 'true',
                          'pause': 'false',
                          'world': world}.items()
    )

    # Robot State Publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_path]
    )

    # Spawn the entity into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'mobile_lift_arm', '-file', urdf_path, '-z', '0.02'],
        output='screen'
    )

    # Start controller manager spawners
    jsp = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    diff = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_base_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    lift = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['lift_position_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gazebo_launch,
        robot_state_pub,
        spawn_entity,
        jsp, diff, lift, arm
    ])
