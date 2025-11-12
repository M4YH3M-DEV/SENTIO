"""Launch file for all SENTIO drivers."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for all drivers."""
    
    package_dir = get_package_share_directory('sentio_drivers')
    
    # Config files
    tfmini_config = os.path.join(package_dir, 'config', 'tfmini_config.yaml')
    maxsonar_config = os.path.join(package_dir, 'config', 'maxsonar_config.yaml')
    imu_config = os.path.join(package_dir, 'config', 'imu_config.yaml')
    camera_config = os.path.join(package_dir, 'config', 'camera_config.yaml')
    
    # Declare launch arguments
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Enable simulation mode for all drivers'
    )
    
    # Individual driver nodes
    tfmini_node = Node(
        package='sentio_drivers',
        executable='tfmini_node',
        name='tfmini_node',
        output='screen',
        parameters=[tfmini_config],
    )
    
    maxsonar_node = Node(
        package='sentio_drivers',
        executable='maxsonar_node',
        name='maxsonar_node',
        output='screen',
        parameters=[maxsonar_config],
    )
    
    imu_node = Node(
        package='sentio_drivers',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[imu_config],
    )
    
    camera_node = Node(
        package='sentio_drivers',
        executable='uvc_camera_node',
        name='uvc_camera_node',
        output='screen',
        parameters=[camera_config],
    )
    
    return LaunchDescription([
        sim_mode_arg,
        tfmini_node,
        maxsonar_node,
        imu_node,
        camera_node,
    ])
