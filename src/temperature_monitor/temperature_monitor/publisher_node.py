#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TemperatureSensorNode(Node):
    """Node that simulates a temperature sensor."""

    def __init__(self):
        super().__init__('temperature_sensor')

        # Create a publisher for temperature readings
        self.publisher = self.create_publisher(
            Float32,                 # Message type
            'temperature',           # Topic name
            10                       # QoS profile (queue size)
        )

        # Create a timer to publish readings at 1 Hz
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(
            self.timer_period,
            self.timer_callback
        )

        self.get_logger().info('Temperature sensor node initialised')

    def timer_callback(self):
        """Generate and publish a random temperature reading."""
        # Generate a random temperature between 15-35°C
        temp = Float32()
        temp.data = random.uniform(15.0, 35.0)

        # Publish the temperature
        self.publisher.publish(temp)

        # Log the reading
        self.get_logger().info(f'Publishing temperature: {temp.data:.1f}°C')


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()