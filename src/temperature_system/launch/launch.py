from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments for configurable parameters

    minimum_temp_arg = DeclareLaunchArgument(
        'minimum_temp',
        default_value='15.0',
        description='Minimum temperature value in Celsius'
    )

    maximum_temp_arg = DeclareLaunchArgument(
        'maximum_temp',
        default_value='35.0',
        description='Maximum temperature value in Celsius'
    )

    publishing_rate_arg = DeclareLaunchArgument(
        'publishing_rate',
        default_value='1',
        description='Publishing rate in Hz'
    )

    log_frequency_arg = DeclareLaunchArgument(
        'log_frequency',
        default_value='5',
        description='Frequency of logs stored in file'
    )

    warning_temperature_threshold_arg = DeclareLaunchArgument(
        'warning_temperature_threshold',
        default_value='25.0',
        description='Warning threshold value'
    )

    critical_temperature_threshold_arg = DeclareLaunchArgument(
        'critical_temperature_threshold',
        default_value='30.0',
        description='Critical threshold value'
    )


    # Create node actions with parameter mappings

    enhanced_publisher_node = Node(
        package='temperature_system',  
        executable='enhanced_publisher_node',  
        name='enhanced_tpublisher_node',
        prefix='xterm -e', 
        parameters=[{
            'minimum_temp': LaunchConfiguration('minimum_temp'),
            'maximum_temp': LaunchConfiguration('maximum_temp'),
            'publishing_rate': LaunchConfiguration('publishing_rate')
        }]
    )

    advanced_monitor_node = Node(
        package='temperature_system',  
        executable='advanced_monitor_node',  
        name='advanced_monitor_node',
        prefix='xterm -e', 
        parameters=[{
            'warning_temperature_threshold': LaunchConfiguration('warning_temperature_threshold'),
            'critical_temperature_threshold': LaunchConfiguration('critical_temperature_threshold')
        }]
    )

    logger_node = Node(
        package='temperature_system',  
        executable='logger_node',  
        name='enhanced_temperature_sensor',
        prefix='xterm -e', 
        parameters=[{
            'log_frequency': LaunchConfiguration('log_frequency')
        }]
    )

    # Return LaunchDescription with all nodes and arguments
    return LaunchDescription([
        # Your launch arguments and nodes will go here
        warning_temperature_threshold_arg,
        critical_temperature_threshold_arg,
        log_frequency_arg,
        minimum_temp_arg,
        maximum_temp_arg,
        publishing_rate_arg,
        enhanced_publisher_node,
        logger_node,
        advanced_monitor_node
    ])