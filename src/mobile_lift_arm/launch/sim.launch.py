from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_name = 'mobile_lift_arm'
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
    
    _robot_description_content = Command(['xacro ', urdf_xacro])
    _robot_description = ParameterValue(_robot_description_content, value_type=str)
    
    robot_state_pub = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'use_sim_time': True,
                     'robot_description': _robot_description}]
    )

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'mobile_lift_arm', '-topic', 'robot_description'],
        output='screen'
    )

    # Controller manager namespace
    cm = '/mobile_lift_arm/controller_manager'

    # Spawn controllers với delay để tránh race condition
    jsp = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', cm],
        output='screen'
    )
    
    # SỬA TÊN CONTROLLER CHO ĐÚNG
    tricycle = Node(
        package='controller_manager', executable='spawner',
        arguments=['tricycle_drive_controller', '--controller-manager', cm],
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

    # Add delay between spawning controllers
    delayed_tricycle = TimerAction(period=3.0, actions=[tricycle])
    delayed_lift = TimerAction(period=4.0, actions=[lift])  
    delayed_arm = TimerAction(period=5.0, actions=[arm])

    return LaunchDescription([
        world_arg, 
        gazebo_launch, 
        robot_state_pub, 
        spawn_entity, 
        jsp,
        delayed_tricycle,
        delayed_lift, 
        delayed_arm
    ])
