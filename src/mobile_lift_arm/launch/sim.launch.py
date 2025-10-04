from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'mobile_lift_arm'  # tên gói của bạn
    pkg_share = FindPackageShare(pkg_name)

    urdf_xacro = PathJoinSubstitution([pkg_share, 'urdf', 'mobile_lift_arm.urdf.xacro'])

    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument('world', default_value='empty.world')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={'world': world}.items()
    )

    robot_state_pub = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'use_sim_time': True,
                     'robot_description': Command(['xacro ', urdf_xacro])}]
    )

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'mobile_lift_arm', '-topic', 'robot_description'],
        output='screen'
    )

    # spawner controllers
    cm = '/mobile_lift_arm/controller_manager'  # trùng tên -entity khi spawn

    jsp = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', cm],
        output='screen'
    )
    diff = Node(sửa
        package='controller_manager', executable='spawner',
        arguments=['diff_drive_base_controller', '--controller-manager', cm],
        output='screen'
    )
    lift = Node(
        package='controller_manager', executable='spawner',
        arguments=['lift_position_controller', '--controller-manager', cm],
        output='screen'
    )
    arm = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_trajectory_controller', '--controller-manager', cm],
        output='screen'
    )

    return LaunchDescription([world_arg, gazebo_launch, robot_state_pub, spawn_entity, jsp, diff, lift, arm])
