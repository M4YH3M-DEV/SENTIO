"""Launch file for USB camera driver."""

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for camera node."""
    
    package_dir = get_package_share_directory('sentio_drivers')
    config_file = os.path.join(package_dir, 'config', 'camera_config.yaml')
    
    camera_node = Node(
        package='sentio_drivers',
        executable='uvc_camera_node',
        name='uvc_camera_node',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args',
                   '--log-level', 'info']
    )
    
    return LaunchDescription([camera_node])
