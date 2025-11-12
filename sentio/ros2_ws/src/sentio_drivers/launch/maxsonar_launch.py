"""Launch file for MaxSonar ultrasonic sensor driver."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for MaxSonar node."""
    
    package_dir = get_package_share_directory('sentio_drivers')
    config_file = os.path.join(package_dir, 'config', 'maxsonar_config.yaml')
    
    maxsonar_node = Node(
        package='sentio_drivers',
        executable='maxsonar_node',
        name='maxsonar_node',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args',
                   '--log-level', 'info']
    )
    
    return LaunchDescription([maxsonar_node])
