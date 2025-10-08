import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use sim time if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_world",
            default_value="",
            description="World file to load in Gazebo",
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    gazebo_world = LaunchConfiguration("gazebo_world")

    # Package paths
    pkg_path = get_package_share_directory('scissor_lift_description')
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("scissor_lift_description"), "urdf", "scissor_lift.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {"use_sim_time": use_sim_time}]
    )

    # Launch Gazebo with empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            ])
        ]),
        launch_arguments={
            "world": gazebo_world,
            "verbose": "true"
        }.items()
    )

    # Spawn Entity Node
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'scissor_lift', '-z', '1.0'],
        output='screen'
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Hydraulic Controller Spawner (for position control)
    hydraulic_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hydraulic_position_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    # RViz Node
    rviz_config_file = os.path.join(pkg_path, 'config', 'display.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{"use_sim_time": use_sim_time}]
    )
    
    # Event handlers for proper startup sequence
    delay_joint_state_broadcaster_spawner_after_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_hydraulic_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[hydraulic_controller_spawner],
        )
    )

    nodes = [
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        rviz_node,
        delay_joint_state_broadcaster_spawner_after_spawn_entity,
        delay_hydraulic_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)