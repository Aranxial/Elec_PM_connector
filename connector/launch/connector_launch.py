from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    package_share_directory = get_package_share_directory('connector')
    
    # Define the path to the Python script
    script_path = os.path.join(package_share_directory, 'code', 'connect_read.py')
    
    # Construct the LaunchDescription
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', script_path],
            output='screen',
            name='connect_read_node'
        ),
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            arguments=['/world/moving_cube/pose/info@tf2_msgs/TFMessage@ignition.msgs.Pose_V'],
            output='screen',
            name='parameter_bridge'
        ),
    ])

