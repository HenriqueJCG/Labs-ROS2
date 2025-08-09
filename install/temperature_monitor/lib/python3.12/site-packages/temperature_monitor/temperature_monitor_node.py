#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TemperatureMonitorNode(Node):
    """Node that monitors temperature and issues warnings."""

    def __init__(self):
        super().__init__('temperature_monitor')

        # Create a subscriber for temperature readings
        self.subscription = self.create_subscription(
            Float32,                     # Message type
            'temperature',               # Topic name
            self.temperature_callback,   # Callback function
            10                           # QoS profile (queue size)
        )

        # Default temperature threshold
        self.temperature_threshold = 30.0

        # Declare a parameter for the temperature threshold
        self.declare_parameter('temperature_threshold', self.temperature_threshold)

        # Get the parameter value
        self.temperature_threshold = self.get_parameter(
            'temperature_threshold'
        ).get_parameter_value().double_value

        self.get_logger().info('Temperature monitor node initialised')
        self.get_logger().info(f'Temperature threshold set to: {self.temperature_threshold}°C')

    def temperature_callback(self, msg):
        """Process incoming temperature readings."""
        temperature = msg.data

        # Log the received temperature
        self.get_logger().info(f'Temperature received: {temperature:.1f}°C')

        # Check if temperature exceeds threshold
        if temperature > self.temperature_threshold:
            self.get_logger().warn(
                f'HIGH TEMPERATURE ALERT: {temperature:.1f}°C exceeds threshold of {self.temperature_threshold}°C'
            )


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()