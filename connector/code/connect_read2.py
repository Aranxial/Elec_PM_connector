#!/usr/bin/env python3

import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class ModelPositionTracker(Node):
    def __init__(self):
        super().__init__('model_position_tracker')
        self.publisher = self.create_publisher(String, 'connector_pos_list', 10)
        self.frame_rate = 1  # 1 Hz, adjust as needed
        self.track_models()

    def get_model_names(self):
        """Get all model names from Ignition Gazebo."""
        result = subprocess.run(['ign', 'model', '--list'], stdout=subprocess.PIPE, text=True)
        models = []
        lines = result.stdout.splitlines()
        for line in lines:
            if line.startswith("    - "):
                model_name = line.split('- ')[1].strip()
                models.append(model_name)
        return models

    def filter_connect_models(self, models):
        """Filter models that start with 'connect_'."""
        return [model for model in models if model.startswith('robot')]

    def get_model_pose(self, model_name,link_id):
        """Get pose information for a specific model."""
        
        result = subprocess.run(['ign', 'model', '-m', model_name,'-l',f'connector_{model_name[-1]}{link_id}'], stdout=subprocess.PIPE, text=True)
        return result.stdout

    def extract_xyz(self, pose_info):
        """Extract the second set of XYZ coordinates from the pose information."""
        lines = pose_info.splitlines()
        xyz = None
        pose_count = 0  # To track the number of pose entries found
    
        for i, line in enumerate(lines):
            if "Pose [ XYZ (m) ] [ RPY (rad) ]:" in line:
                pose_count += 1
            
                # Check if it's the second occurrence
                if pose_count == 2:
                # Extract the XYZ line after this
                    xyz_line = lines[i + 1].strip()
                    xyz_values = xyz_line.strip('[]').split()
                    xyz = {
                        'x': float(xyz_values[0]),
                        'y': float(xyz_values[1]),
                        'z': float(xyz_values[2])
                    }
                    break
    
        return xyz


    def track_models(self):
        """Continuously track and publish the positions of models."""
        while rclpy.ok():
            models = self.get_model_names()
            connect_models = self.filter_connect_models(models)
            
            position_data = []
            for model in connect_models:
                for i in range(1,7):
                    pose_info = self.get_model_pose(model,i)
                    xyz = self.extract_xyz(pose_info)
                    if xyz:
                        position_data.append(f"connector_{model[-1]}{i} : {xyz['x']}, {xyz['y']}, {xyz['z']}")
                    else:
                        self.get_logger().warning(f"Could not retrieve XYZ data for model: {model}")

            if position_data:
                # Publish the position data as a string
                message = String()
                message.data = "; ".join(position_data)
                self.publisher.publish(message)

            time.sleep(1.0 / self.frame_rate)  # Sleep to maintain frame rate

def main(args=None):
    rclpy.init(args=args)
    tracker = ModelPositionTracker()
    rclpy.spin(tracker)
    tracker.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

