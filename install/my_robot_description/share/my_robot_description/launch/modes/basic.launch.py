"""Basic mode launch file that uses the components launch files"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_description')
    include_dir = os.path.join(pkg_dir, 'launch/components')

    #Forwarded param_file from main launch file
    param_file = LaunchConfiguration('param_file')
    # Include the robot_description and rviz launch files from the modular components
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(include_dir, 'robot_description.launch.py')),
        launch_arguments={'param_file': param_file}.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(include_dir, 'visualization.launch.py')),
        launch_arguments={'param_file': param_file}.items()
    )

    return LaunchDescription([
        robot_description,
        rviz
    ])