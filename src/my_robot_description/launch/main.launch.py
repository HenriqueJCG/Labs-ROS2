"""
Main launch file for the robot system with multiple operational modes.
Supports: basic, simulation, direct_control, teleop, and custom modes.
"""

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('my_robot_description')  # Replace with your package name
    launch_dir = os.path.join(pkg_dir, 'launch')

    # Define paths
    components_dir = os.path.join(launch_dir, 'components')
    modes_dir = os.path.join(launch_dir, 'modes')

    # Create launch configuration variables
    operation_mode = LaunchConfiguration('mode', default='basic')

    # TODO: Add more launch configuration variables

    # Declare launch arguments
    declare_mode = DeclareLaunchArgument(
        'mode',
        default_value='basic',
        description='Operation mode: basic, simulation, direct_control, teleop, or custom'
    )

    # TODO: Declare more launch arguments

    # TODO: Create includes for component launch files

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(components_dir, 'robot_description.launch.py')
        )
    )

    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(components_dir, 'visualization.launch.py')
        )
    )

    simulation_component = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(components_dir, 'simulation.launch.py')
        )
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(components_dir, 'control.launch.py')
        )
    )
    
    # TODO: Create conditional includes for mode-specific launch files

    basic_mode = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(modes_dir, 'basic.launch.py')
        ),
        condition=IfCondition(operation_mode.perform(None) == 'basic')
    )

    simulation_mode = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(modes_dir, 'simulation.launch.py')
        ),
        condition=IfCondition(operation_mode.perform(None) == 'simulation')
    )

    teleop_mode = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(modes_dir, 'teleop.launch.py')
        ),
        condition=IfCondition(operation_mode.perform(None) == 'teleop')
    )

    # Return the launch description
    return LaunchDescription([
        declare_mode,

        # TODO: Add all components and conditional mode launches
    ])