# Labs-ROS2
Repository for labs of ROS2 training, using Ubuntu 24.04, ROS2 Jazzy LTS and Gazebo Harmonic.

### Lab1 - src/temperature_monitor and src/temperature_system

**Challenge:**

- **Nodes:** *enhanced_publisher_node*, *advanced_monitor_node*, *logger_node*
- **Parameters:** *publishing_rate*, *minimum_temp*, *maximum_temp*, *warning_temperature_threshold*, *critical_temperature_threshold*, *log_frequency*

*enhanced_publisher_node* publishes temperatures for the subscribers *advanced_monitor_node* and *logger_node*. It uses *publishing_rate* to control the frequency of publishes, and *maximum_temp* and *minimum_temp* to set the amplitude of possible temperatures.

*warning_temperature_threshold* and *critical_temperature_threshold* control the thresholds for the warnings. It calculates a simple moving average filter to reduce false alarms, and compares the resulting value with the thresholds.

Every *log_frequency* readings, the temperature value and timestamp is logged in a file.

Use ros2 launch temperature_system launch.py to run, needs xterm installed to run.

**Youtube Video:** [https://www.youtube.com/watch?v=uT9rRpTyGzo&feature=youtu.be](https://www.youtube.com/watch?v=uT9rRpTyGzo&feature=youtu.be)

---

### Lab2 - src/simple_robot_description and src/my_robot_description

**Challenge:**

To run with Gazebo use *ros2 launch my_robot_description gazebo.launch.py* and to run the display only on rviz2 use *ros2 launch my_robot_description display.launch.py*.


Changes to the Gazebo world are made on the *diff_drive.sdf* file and changes to the robot are made on *my_robot_gazebo.urdf* and *my_robot.urdf*. The length of the robot increased and it was added a cillinder to the top of the robot. The gazebo map includes 4 objects and a smaller walled area.

Updated the *gazebo.launch.py* file and *gz_bridge.yaml* to include the bridge for both cameras.

**Youtube Video:** 

---

### Lab3 - /launch/include/ and /launch/robot_complete.launch.py

**Challenge:**



**Youtube Video:** 