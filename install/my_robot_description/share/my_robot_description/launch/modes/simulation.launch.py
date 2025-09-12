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
    pkg_dir = get_package_share_directory('my_robot_description')

    # Define paths to URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'my_robot_gazebo.urdf')

    # Create launch configuration variables
    world_file = os.path.join(pkg_dir, 'worlds', 'diff_drive', 'diff_drive.sdf')

    # Read URDF file content
    with open(urdf_file, 'r') as file:
        robot_description = file.read()


    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # RViz node for Gazebo mode (with camera)
    rviz_node_gazebo = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'urdf_config.rviz')]
    )
    # Gazebo node
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
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
        output='screen'
    )

    # ROS-Gazebo bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p', f'config_file:={os.path.join(pkg_dir, "config", "gz_bridge.yaml")}',
        ]
    )

    # Camera bridges
    right_camera_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/right_camera/image_raw"],
    )

    left_camera_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/left_camera/image_raw"],
    )

    # Return the launch description
    return LaunchDescription([

        # Nodes for all modes
        robot_state_publisher,

        # RViz nodes (conditionally loaded based on mode)
        rviz_node_gazebo,

        # Gazebo simulation nodes
        gazebo,
        spawn_entity,
        bridge,
        right_camera_bridge,
        left_camera_bridge,

    ])