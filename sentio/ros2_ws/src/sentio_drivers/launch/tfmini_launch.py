"""Launch file for TFMini-S LiDAR driver."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for TFMini node."""
    
    package_dir = get_package_share_directory('sentio_drivers')
    config_file = os.path.join(package_dir, 'config', 'tfmini_config.yaml')
    
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for TFMini'
    )
    
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Enable simulation mode'
    )
    
    # TFMini node
    tfmini_node = Node(
        package='sentio_drivers',
        executable='tfmini_node',
        name='tfmini_node',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args',
                   '--log-level', 'info']
    )
    
    return LaunchDescription([
        port_arg,
        sim_mode_arg,
        tfmini_node,
    ])
