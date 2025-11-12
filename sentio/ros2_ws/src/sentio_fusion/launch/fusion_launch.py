"""Launch file for AETHER fusion node."""

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description."""
    
    package_dir = get_package_share_directory('sentio_fusion')
    config_file = os.path.join(package_dir, 'config', 'fusion_node_config.yaml')
    
    node = Node(
        package='sentio_fusion',
        executable='fusion_node',
        name='fusion_node',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([node])
