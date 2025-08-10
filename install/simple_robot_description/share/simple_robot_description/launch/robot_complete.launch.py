"""Main launch file for the complete robot setup with multiple operation modes."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('simple_robot_description')

    # Define paths to URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'simple_robot_gazebo.urdf')

    # Create launch configuration variables
    use_gazebo = LaunchConfiguration('use_gazebo', default='true')
    use_teleop = LaunchConfiguration('use_teleop', default='true')
    world_file = os.path.join(pkg_dir, 'worlds', 'diff_drive', 'diff_drive.sdf')

    # Declare launch arguments
    declare_use_gazebo = DeclareLaunchArgument(
        'use_gazebo',
        default_value='true',
        description='Launch Gazebo if true'
    )

    declare_use_teleop = DeclareLaunchArgument(
        'use_teleop',
        default_value='true',
        description='Launch teleop if true'
    )

    # Read URDF file content
    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    # ------------------------------------------------------
    # Nodes required for all modes
    # ------------------------------------------------------

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # ------------------------------------------------------
    # RViz nodes with different configurations
    # ------------------------------------------------------

    # RViz node for Gazebo mode (with camera)
    rviz_node_gazebo = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'urdf_config.rviz')],
        condition=IfCondition(use_gazebo)
    )

    # RViz node for visualisation-only mode (without camera)
    rviz_node_viz_only = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'urdf_config_nogazebo.rviz')],
        condition=UnlessCondition(use_gazebo)
    )

    # ------------------------------------------------------
    # Nodes required for ONLY the non-Gazebo visualisation mode
    # ------------------------------------------------------

    # Joint state publisher (used when Gazebo is not running)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(use_gazebo)
    )

    # Joint state publisher GUI (used when Gazebo is not running)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=UnlessCondition(use_gazebo)
    )

    # ------------------------------------------------------
    # Nodes required for ONLY the Gazebo simulation mode
    # ------------------------------------------------------

    # Gazebo node
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        condition=IfCondition(use_gazebo)
    )

    # Spawn entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'simple_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen',
        condition=IfCondition(use_gazebo)
    )

    # ROS-Gazebo bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p', f'config_file:={os.path.join(pkg_dir, "config", "gz_bridge.yaml")}',
        ],
        condition=IfCondition(use_gazebo)
    )

    # Camera bridge
    camera_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"],
        condition=IfCondition(use_gazebo)
    )

    # ------------------------------------------------------
    # Teleop (controlled by use_teleop flag AND use_gazebo flag)
    # ------------------------------------------------------

    # Teleop node - only active when both use_teleop AND use_gazebo are true
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[('/cmd_vel', '/cmd_vel')],
        condition=IfCondition(
            PythonExpression([
                '"', use_teleop, '" == "true" and "', use_gazebo, '" == "true"'
            ])
        )
    )

    # Return the launch description
    return LaunchDescription([
        # Launch arguments
        declare_use_gazebo,
        declare_use_teleop,

        # Nodes for all modes
        robot_state_publisher,

        # RViz nodes (conditionally loaded based on mode)
        rviz_node_gazebo,
        rviz_node_viz_only,

        # Visualisation-only nodes
        joint_state_publisher,
        joint_state_publisher_gui,

        # Gazebo simulation nodes
        gazebo,
        spawn_entity,
        bridge,
        camera_bridge,

        # Teleop
        teleop
    ])