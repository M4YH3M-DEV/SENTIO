"""Launch all perception nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for all perception."""
    
    package_dir = get_package_share_directory('sentio_perception')
    
    config_files = {
        'face': os.path.join(package_dir, 'config', 'face_emotion_config.yaml'),
        'audio': os.path.join(package_dir, 'config', 'audio_emotion_config.yaml'),
        'group': os.path.join(package_dir, 'config', 'group_detector_config.yaml'),
    }
    
    nodes = [
        Node(
            package='sentio_perception',
            executable='face_emotion_node',
            name='face_emotion_node',
            output='screen',
            parameters=[config_files['face']],
        ),
        Node(
            package='sentio_perception',
            executable='audio_emotion_node',
            name='audio_emotion_node',
            output='screen',
            parameters=[config_files['audio']],
        ),
        Node(
            package='sentio_perception',
            executable='group_detector_node',
            name='group_detector_node',
            output='screen',
            parameters=[config_files['group']],
        ),
    ]
    
    return LaunchDescription(nodes)
