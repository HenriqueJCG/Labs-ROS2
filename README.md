# Labs-ROS2
Repository for labs of ROS2 training, using Ubuntu 24.04, ROS2 Jazzy LTS and Gazebo Harmonic.

### Lab1 - src/temperature_monitor | src/temperature_system

**Challenge:**

- Nodes: *enhanced_publisher_node*, *advanced_monitor_node*, *logger_node*
- Parameters: *publishing_rate*, *minimum_temp*, *maximum_temp*, *warning_temperature_threshold*, *critical_temperature_threshold*, *log_frequency*

*enhanced_publisher_node* publishes temperatures for the subscribers *advanced_monitor_node* and *logger_node*. It uses *publishing_rate* to control the frequency of publishes, and *maximum_temp* and *minimum_temp* to set the amplitude of possible temperatures.

*warning_temperature_threshold* and *critical_temperature_threshold* control the thresholds for the warnings.

Every *log_frequency* readings, the temperature value and timestamp is logged in a file.

### Lab2 - src/simple_robot_description

**Challenge:**
