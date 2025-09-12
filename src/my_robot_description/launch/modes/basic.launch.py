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

    # RViz node for visualisation-only mode (without camera)
    rviz_node_viz_only = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'urdf_config_nogazebo.rviz')]
    )

    # Joint state publisher (used when Gazebo is not running)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Joint state publisher GUI (used when Gazebo is not running)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )


    # Return the launch description
    return LaunchDescription([

        # Nodes for all modes
        robot_state_publisher,

        # RViz nodes 
        rviz_node_viz_only,

        # Visualisation-only nodes
        joint_state_publisher,
        joint_state_publisher_gui
    ])