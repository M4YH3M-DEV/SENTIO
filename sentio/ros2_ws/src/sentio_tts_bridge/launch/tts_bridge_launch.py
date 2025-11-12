"""Launch file for SENTIO TTS Bridge."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for TTS Bridge."""
    
    package_dir = get_package_share_directory('sentio_tts_bridge')
    config_file = os.path.join(package_dir, 'config', 'tts_config.yaml')
    
    # Launch arguments
    piper_url_arg = DeclareLaunchArgument(
        'piper_url',
        default_value='http://localhost:5000',
        description='Piper TTS server URL'
    )
    
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Enable simulation mode'
    )
    
    # TTS Bridge node
    tts_node = Node(
        package='sentio_tts_bridge',
        executable='sentio_tts_bridge_node',
        name='sentio_tts_bridge_node',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        piper_url_arg,
        sim_mode_arg,
        tts_node,
    ])
