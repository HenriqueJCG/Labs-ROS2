"""Main launch file for the complete robot setup."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('simple_robot_description')
    launch_dir = os.path.join(pkg_dir, 'launch')
    include_dir = os.path.join(launch_dir, 'include')

    # Create launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui', default='true')
    use_teleop = LaunchConfiguration('use_teleop', default='true')
    urdf_file = LaunchConfiguration('urdf_file', default='simple_robot_gazebo.urdf')
    world_file = LaunchConfiguration('world_file', default=os.path.join(pkg_dir, 'worlds', 'diff_drive', 'diff_drive.sdf'))
    robot_name = LaunchConfiguration('robot_name', default='simple_robot')
    x_pos = LaunchConfiguration('x_pos', default='0.0')
    y_pos = LaunchConfiguration('y_pos', default='0.0')
    z_pos = LaunchConfiguration('z_pos', default='0.1')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic', default='/cmd_vel')

    # Declare all launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz if true'
    )

    declare_use_gazebo_gui = DeclareLaunchArgument(
        'use_gazebo_gui',
        default_value='true',
        description='Use Gazebo GUI if true'
    )

    declare_use_teleop = DeclareLaunchArgument(
        'use_teleop',
        default_value='true',
        description='Launch teleop if true'
    )

    declare_urdf_file = DeclareLaunchArgument(
        'urdf_file',
        default_value='simple_robot_gazebo.urdf',
        description='URDF file name (within the urdf directory)'
    )

    declare_world_file = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(pkg_dir, 'worlds', 'diff_drive', 'diff_drive.sdf'),
        description='Path to world file'
    )

    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='simple_robot',
        description='Name of the robot'
    )

    declare_x_pos = DeclareLaunchArgument(
        'x_pos',
        default_value='0.0',
        description='X position for robot spawning'
    )

    declare_y_pos = DeclareLaunchArgument(
        'y_pos',
        default_value='0.0',
        description='Y position for robot spawning'
    )

    declare_z_pos = DeclareLaunchArgument(
        'z_pos',
        default_value='0.1',
        description='Z position for robot spawning'
    )

    declare_cmd_vel_topic = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Topic for velocity commands'
    )

    # Include robot state publisher launch file
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(include_dir, 'robot_state_publisher.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'urdf_file': urdf_file
        }.items()
    )

    # Include RViz launch file
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(include_dir, 'rviz.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz
        }.items()
    )

    # Include Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(include_dir, 'gazebo.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_gazebo_gui': use_gazebo_gui,
            'world_file': world_file,
            'robot_name': robot_name,
            'x_pos': x_pos,
            'y_pos': y_pos,
            'z_pos': z_pos
        }.items()
    )

    # Include teleop launch file
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(include_dir, 'teleop.launch.py')),
        launch_arguments={
            'cmd_vel_topic': cmd_vel_topic
        }.items()
    )

    # Return the complete launch description
    return LaunchDescription([
        # All declare arguments
        declare_use_sim_time,
        declare_use_rviz,
        declare_use_gazebo_gui,
        declare_use_teleop,
        declare_urdf_file,
        declare_world_file,
        declare_robot_name,
        declare_x_pos,
        declare_y_pos,
        declare_z_pos,
        declare_cmd_vel_topic,

        # All launch includes
        robot_state_publisher_launch,
        rviz_launch,
        gazebo_launch,
        teleop_launch
    ])