"""Launch file for demo runner."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description."""
    
    package_dir = get_package_share_directory('sentio_demo')
    config_file = os.path.join(package_dir, 'config', 'demo_config.yaml')
    choreography_dir = os.path.join(package_dir, 'choreography')
    
    # Launch arguments
    sequence_arg = DeclareLaunchArgument(
        'sequence',
        default_value='greet_sequence',
        description='Demo sequence to run'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Auto-start demo on launch'
    )
    
    # Demo runner node
    demo_node = Node(
        package='sentio_demo',
        executable='demo_runner_node',
        name='demo_runner_node',
        output='screen',
        parameters=[{
            'default_sequence': LaunchConfiguration('sequence'),
            'auto_start': LaunchConfiguration('auto_start'),
            'choreography_dir': choreography_dir,
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        sequence_arg,
        auto_start_arg,
        demo_node,
    ])
