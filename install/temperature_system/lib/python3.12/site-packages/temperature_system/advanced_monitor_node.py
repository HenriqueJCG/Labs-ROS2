#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
from datetime import datetime

class AdvancedTemperatureMonitorNode(Node):
    def __init__(self):
        super().__init__('advanced_temperature_monitor')

        # Declare parameters for warning and critical thresholds
        self.warning_temperature_threshold = 25.0
        self.critical_temperature_threshold = 30.0

        self.declare_parameter('warning_temperature_threshold', self.warning_temperature_threshold)
        self.declare_parameter('critical_temperature_threshold', self.critical_temperature_threshold)


        # TODO: Create a subscriber with appropriate QoS settings
        self.subscription = self.create_subscription(
            String,                     # Message type
            'temperature',               # Topic name
            self.temperature_callback,   # Callback function
            10                           # QoS profile (queue size)
        )

        # Get the parameter value
        self.warning_temperature_threshold = self.get_parameter(
            'warning_temperature_threshold'
        ).get_parameter_value().double_value

        self.critical_temperature_threshold = self.get_parameter(
            'critical_temperature_threshold'
        ).get_parameter_value().double_value


        # Initialise counters for different alert levels

        self.critical_count = 0
        self.warning_count = 0


        # Set up moving average window

        self.temperatures = []
        self.moving_avg_window = 3

    def temperature_callback(self, msg):
        # Parse JSON temperature data
        parsed_data = json.loads(msg.data)
        temperature = parsed_data["value"]
        location = parsed_data["location"]

        # Apply moving average filter
        sum_temperature = 0
        self.temperatures.append(temperature)
        
        sum_temperature = sum(self.temperatures[-self.moving_avg_window:])
        temperature = round(sum_temperature/self.moving_avg_window, 1)

        # Log the received temperature
        #self.get_logger().info(f'Temperature: {temperature:.1f}°C')
        #self.get_logger().info(f'Location: {location}')

        os.system('clear')

        print(f'Location: {location}')
        print(f'Time: {datetime.now().isoformat()}')
        print(f' {temperature:.1f}C')
        


        # Check against thresholds and issue appropriate alerts
        if temperature > self.warning_temperature_threshold and temperature < self.critical_temperature_threshold:
            self.warning_count = self.warning_count + 1 #Update warning counter

            self.get_logger().warn(
                f'WARNING!!\nHIGH TEMPERATURE ALERT: {temperature:.1f}°C exceeds threshold of {self.warning_temperature_threshold}°C'
            )
        elif temperature > self.critical_temperature_threshold:
            self.critical_count = self.critical_count + 1 #Update critical counter
            
            self.get_logger().warn(
                f'!!CRITICAL!!\nHIGH TEMPERATURE ALERT: {temperature:.1f}°C exceeds threshold of {self.critical_temperature_threshold}°C'
            )


def main(args=None):
    # Initialise node and handle lifecycle

    rclpy.init(args=args)
    node = AdvancedTemperatureMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()