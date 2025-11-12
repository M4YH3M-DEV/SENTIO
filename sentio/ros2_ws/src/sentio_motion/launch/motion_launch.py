"""Launch file for motion control."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description."""
    
    package_dir = get_package_share_directory('sentio_motion')
    kinematics_file = os.path.join(package_dir, 'config', 'kinematics.yaml')
    servo_map_file = os.path.join(package_dir, 'config', 'servo_map.yaml')
    safety_file = os.path.join(package_dir, 'config', 'safety_limits.yaml')
    
    # Launch arguments
    sim_arg = DeclareLaunchArgument(
        'simulate',
        default_value='false',
        description='Run in simulation mode'
    )
    
    port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for ESP32'
    )
    
    # Motion node
    motion_node = Node(
        package='sentio_motion',
        executable='servo_bridge_node',
        name='servo_bridge_node',
        output='screen',
        parameters=[{
            'kinematics_config': kinematics_file,
            'servo_map_config': servo_map_file,
            'safety_limits_config': safety_file,
            'serial_port': LaunchConfiguration('serial_port'),
            'simulate_hardware': LaunchConfiguration('simulate'),
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        sim_arg,
        port_arg,
        motion_node,
    ])
