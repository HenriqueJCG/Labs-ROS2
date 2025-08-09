#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import json

class EnhancedTemperatureSensorNode(Node):
    def __init__(self):
        super().__init__('enhanced_temperature_sensor')

        # Declare parameters for temperature range and publishing rate

        self.minimum_temp = 15.0
        self.maximum_temp = 35.0

        self.publishing_rate = 1

        self.declare_parameter('minimum_temp', self.minimum_temp)
        self.declare_parameter('maximum_temp', self.maximum_temp)
        self.declare_parameter('publishing_rate', self.publishing_rate)


        self.minimum_temp = self.get_parameter(
            'minimum_temp'
        ).get_parameter_value().double_value

        self.maximum_temp = self.get_parameter(
            'maximum_temp'
        ).get_parameter_value().double_value

        self.publishing_rate = self.get_parameter(
            'publishing_rate'
        ).get_parameter_value().integer_value

 
        # Create a publisher with appropriate QoS settings

        self.publisher = self.create_publisher(
            String,                 # Message type
            'temperature',           # Topic name
            10                       # QoS profile (queue size)
        )

        # Create a timer based on the configured rate

        self.timer_period = 1.0 / self.publishing_rate  # seconds
        self.timer = self.create_timer(
            self.timer_period,
            self.timer_callback
        )

        # Initialise sensor location and other metadata

        self.location = "living_room"
        self.unit = "celsius"

    def timer_callback(self):
        # Generate temperature reading with metadata

        temp = random.uniform(self.minimum_temp, self.maximum_temp)

        # Publish reading as a JSON string

        self.get_logger().info(f'Publishing temperature: {temp:.1f}°C')


        data = {
            "location": self.location,
            "value": temp,
            "unit": self.unit
        }

        msg = String()
        msg.data = json.dumps(data)
        self.publisher.publish(msg)
        pass

def main(args=None):
    # Initialise node and handle lifecycle
    
    rclpy.init(args=args)
    node = EnhancedTemperatureSensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()