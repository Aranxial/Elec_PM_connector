#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import math
from threading import Thread

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
        
        # Define the threshold distance d0 (can be adjusted based on requirements)
        self.d0 = 6.0  # Adjust this value as needed

    def listener_callback(self, msg):
        """Callback to process the incoming connector_pos_list message."""
        position_data = [entry.split(' : ') for entry in msg.data.split('; ')]
        
        for i in range(len(position_data)):
            for j in range(i + 1, len(position_data)):
                model_1, pos_1 = position_data[i]
                model_2, pos_2 = position_data[j]

                # Convert positions to float
                pos_1 = [float(coord) for coord in pos_1.split(',')]
                pos_2 = [float(coord) for coord in pos_2.split(',')]

                # Calculate the distance between the two models
                distance = self.calculate_distance(pos_1, pos_2)

                # Apply force only if distance is less than d0
  
                distance = self.calculate_distance(pos_1, pos_2)
                force_vector = self.calculate_force(pos_1, pos_2, distance)
                force_vector_ = self.calculate_force(pos_2, pos_1, distance)
                #force_vector = Thread(target = self.calculate_force, args=(pos_1, pos_2, distance, )).start()
                #force_vector_ = Thread(target = self.calculate_force, args=(pos_2, pos_1, distance, )).start()
                Thread(target = self.apply_wrench_to_model, args=(model_1, force_vector,)).start()
                Thread(target = self.apply_wrench_to_model, args=(model_2, force_vector_,)).start()


    def calculate_distance(self, pos_1, pos_2):
        """Calculate the distance between two positions."""
        dist = math.sqrt(sum([(pos_2[i] - pos_1[i]) ** 2 for i in range(3)]))
        if dist <= 2 :
              dist = 10
        if dist <=0.2 :
              dist = 10000
        return dist

    def calculate_force(self, pos_1, pos_2, distance):
        """Calculate the force direction and magnitude between two positions using inverse square law."""
        # Calculate the vector from model_1 to model_2
        vector = [pos_2[i] - pos_1[i] for i in range(3)]

        # Normalize the vector to get the unit direction
        unit_vector = [v / distance for v in vector]

        # Define the constant k (can be adjusted based on requirements)
        k = 2000.0  # Adjust this constant to control force magnitude

        # Calculate the force magnitude based on inverse square law: F = k / d^2
        force_magnitude = k / (distance)

        # Scale the unit vector by the force magnitude
        force_vector = [force_magnitude * u for u in unit_vector]

        return force_vector

    def apply_wrench_to_model(self, model_name, force_vector):
        """Apply wrench to the specific model's connector_link."""
        a = f'ign topic -t "/world/moving_cube/wrench" '
        b = f'-m ignition.msgs.EntityWrench '
        c = f'-p "entity: {{name: \'{model_name}::connector_link\', type: LINK}}, '
        d = f'wrench: {{force: {{x: {force_vector[0]}, y: {force_vector[1]}, z: {force_vector[2]}}}, '
        e = f'torque: {{x: 0.0, y: 0.0, z: 1.0}}}}"'
        command = a + b + c + d + e
        print(command)

        self.get_logger().info(f'Applying wrench to {model_name} with force vector {force_vector}')
        os.system(command)  # Execute the command

def main(args=None):
    rclpy.init(args=args)
    node = ForcePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

