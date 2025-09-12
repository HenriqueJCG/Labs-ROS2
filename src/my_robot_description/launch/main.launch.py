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
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('my_robot_description')  # Replace with your package name
    launch_dir = os. path.join(pkg_dir, 'launch')

    # Define paths
    components_dir = os.path.join(launch_dir, 'components')
    modes_dir = os.path.join(launch_dir, 'modes')

    # Create launch configuration variables
    operation_mode = LaunchConfiguration('mode', default=EnvironmentVariable('ROBOT_ENV', default_value='basic'))

    # Add more launch configuration variables
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_gazebo = LaunchConfiguration('use_gazebo', default='true')
    use_teleop = LaunchConfiguration('use_teleop', default='true')
    # Declare launch arguments
    declare_mode = DeclareLaunchArgument(
        'mode',
        default_value=EnvironmentVariable('ROBOT_ENV', default_value='basic'),
        description='Operation mode: basic, simulation, direct_control, teleop, or custom'
    )

    # Declare more launch arguments

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch rviz if true'
    )

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
     
    # Create conditional includes for mode-specific launch files

    #Launches rviz only
    basic_mode = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(modes_dir, 'basic.launch.py')
        ),
        condition=IfCondition(
            PythonExpression([
            "'", operation_mode, "' == 'basic' and '", use_rviz, "' == 'true'"
            ])
        )
    )

    #Launches Gazebo, rviz 
    simulation_mode = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(modes_dir, 'simulation.launch.py')
        ),
        condition=IfCondition(
            PythonExpression([
            "'", operation_mode, "' == 'simulation' and '", use_gazebo, "' == 'true' and '", use_rviz, "' == 'true'"
            ])
        )
    )

    #Launches Gazebo, rviz and teleop
    teleop_mode = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(modes_dir, 'teleop.launch.py')
        ),
        condition=IfCondition(
            PythonExpression([
            "'", operation_mode, "' == 'teleop' and '", use_teleop, "' == 'true' and '", use_gazebo, "' == 'true' and '", use_rviz, "' == 'true'"
            ])
        )
    )

    # Declare environment variable
    declare_robot_env = DeclareLaunchArgument(
        'robot_env',
        default_value=EnvironmentVariable('ROBOT_ENV', default_value='basic'),
        description='Launch mode type '
    )

    # Set environment variable 
    set_env_var = SetEnvironmentVariable(
        name='ROBOT_ENV',
        value=operation_mode
    )


    # Return the launch description
    return LaunchDescription([
        declare_mode,

        #Add all components and conditional mode launches
        #Arguments
        declare_use_rviz,
        declare_use_gazebo,
        declare_use_teleop,

        #Modes
        basic_mode,
        simulation_mode,
        teleop_mode,

        declare_robot_env,
        set_env_var,
    ])