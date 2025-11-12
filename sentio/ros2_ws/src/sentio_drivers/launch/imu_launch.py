"""Launch file for IMU driver."""

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for IMU node."""
    
    package_dir = get_package_share_directory('sentio_drivers')
    config_file = os.path.join(package_dir, 'config', 'imu_config.yaml')
    
    imu_node = Node(
        package='sentio_drivers',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args',
                   '--log-level', 'info']
    )
    
    return LaunchDescription([imu_node])
