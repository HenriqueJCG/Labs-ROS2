import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('my_robot_description')

    # Paths to files
    urdf_file = os.path.join(pkg_dir, 'urdf', 'my_robot_gazebo.urdf')
    world_file = os.path.join(pkg_dir, 'worlds', 'diff_drive', 'diff_drive.sdf')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=world_file)

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Full path to the world model file to load'
    )

    # Read the URDF file
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
            'qos_overrides./robot_description.publisher.durability': 'transient_local',
            'qos_overrides./robot_description.publisher.reliability': 'reliable'
        }]
    )

    # Start Gazebo Harmonic with the world (headless mode)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # Spawn entity using ros_gz_sim
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'simple_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    # Bridge configuration
    bridge_params = os.path.join(pkg_dir, 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    # Right Camera bridge (example)
    bridge_right_camera_image = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/right_camera/image_raw"]
    )

    # Left Camera bridge (example)
    bridge_left_camera_image = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/left_camera/image_raw"]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d',os.path.join(pkg_dir, 'rviz', 'urdf_config.rviz')],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        robot_state_publisher_node,
        gz_sim,
        spawn_entity,
        bridge_right_camera_image,
        bridge_left_camera_image,
        ros_gz_bridge,
        rviz_node
    ])
