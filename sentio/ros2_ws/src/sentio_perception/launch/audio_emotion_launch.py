"""Launch file for audio emotion detection."""

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description."""
    
    package_dir = get_package_share_directory('sentio_perception')
    config_file = os.path.join(package_dir, 'config', 'audio_emotion_config.yaml')
    
    node = Node(
        package='sentio_perception',
        executable='audio_emotion_node',
        name='audio_emotion_node',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([node])
