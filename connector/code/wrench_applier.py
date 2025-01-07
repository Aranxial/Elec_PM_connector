#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os

class ForcePublisher(Node):
    def __init__(self):
        super().__init__('force_publisher')
        self.subscription = self.create_subscription(
            String,
            'connector_pos_list',
            self.listener_callback,
            10
        )
        self.get_logger().info('ForcePublisher is initialized and listening...')

    def listener_callback(self, msg):
        """Callback to process the incoming connector_pos_list message."""
        position_data = msg.data.split('; ')
        for entry in position_data:
            model_name, _ = entry.split(':', 1)  # Extract model name
            self.apply_wrench_to_model(model_name)

    def apply_wrench_to_model(self, model_name):
        """Apply wrench to the specific model's connector_link."""
        command = f"ign topic -t \"/world/moving_cube/wrench\" " \
                  f"-m ignition.msgs.EntityWrench " \
                  f"-p \"entity: {{name: '{model_name}::my world', type: LINK}}, " \
                  f"wrench: {{force: {{x: 100.0, y: 0.0, z: 0.0}}, torque: {{x: 0.0, y: 0.0, z: 1.0}}}}\""
        self.get_logger().info(f'Applying wrench to {model_name}')
        os.system(command)  # Execute the command

def main(args=None):
    rclpy.init(args=args)
    node = ForcePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

