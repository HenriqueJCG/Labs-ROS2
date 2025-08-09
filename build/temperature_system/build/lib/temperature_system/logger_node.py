#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import datetime

class TemperatureLoggerNode(Node):
    def __init__(self):
        super().__init__('temperature_logger')

        # Declare parameters for log file path and logging frequency

        self.file_path = 'temperature_log.txt'
        self.log_frequency = 3

        self.declare_parameter('log_frequency', self.log_frequency)
        
        self.log_frequency = self.get_parameter(
            'log_frequency'
        ).get_parameter_value().integer_value


        # Create a subscriber with appropriate QoS settings

        self.subscription = self.create_subscription(
            String,                     # Message type
            'temperature',               # Topic name
            self.logging_callback,   # Callback function
            10                           # QoS profile (queue size)
        )

        # Initialise logging counter and open log file

        self.log_counter = 0

        self.open_file = open(self.file_path, 'a')

        

    def logging_callback(self, msg):
        # Parse temperature data
        parsed_data = json.loads(msg)
        temperature = parsed_data["value"]

        self.log_counter = self.log_counter + 1

        # Check if this reading should be logged (based on frequency)
        if self.log_counter % self.log_frequency == 0:
            # Write to log file with timestamp

            timestamp = datetime.now().isoformat()

            line = f"{timestamp} - Temperature: {temperature:.2f}°C\n"
            open_file.write(line)

        
        pass

def main(args=None):
    # Initialise node and handle lifecycle

    rclpy.init(args=args)
    node = TemperatureLoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        open_file.close()
        node.destroy_node()
        rclpy.shutdown()

    # Ensure proper file cleanup on shutdown
    pass

if __name__ == '__main__':
    main()