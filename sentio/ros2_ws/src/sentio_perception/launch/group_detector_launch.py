"""Launch file for group detection."""

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description."""
    
    package_dir = get_package_share_directory('sentio_perception')
    config_file = os.path.join(package_dir, 'config', 'group_detector_config.yaml')
    
    node = Node(
        package='sentio_perception',
        executable='group_detector_node',
        name='group_detector_node',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([node])
