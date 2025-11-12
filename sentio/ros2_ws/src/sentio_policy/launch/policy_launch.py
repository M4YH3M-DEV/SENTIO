"""Launch file for policy engine."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description."""
    
    package_dir = get_package_share_directory('sentio_policy')
    config_file = os.path.join(package_dir, 'config', 'default_policy.yaml')
    schema_file = os.path.join(package_dir, 'schema', 'behavior_schema.json')
    
    # Launch arguments
    profile_arg = DeclareLaunchArgument(
        'profile',
        default_value='balanced',
        description='Policy profile (balanced/expressive/cautious)'
    )
    
    safety_arg = DeclareLaunchArgument(
        'safety_enabled',
        default_value='true',
        description='Enable safety checks'
    )
    
    sim_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Enable simulation mode'
    )
    
    # Policy node
    policy_node = Node(
        package='sentio_policy',
        executable='policy_engine_node',
        name='policy_engine_node',
        output='screen',
        parameters=[{
            'policy_config_file': config_file,
            'schema_file': schema_file,
            'default_profile': LaunchConfiguration('profile'),
            'safety_enabled': LaunchConfiguration('safety_enabled'),
            'sim_mode': LaunchConfiguration('sim_mode'),
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        profile_arg,
        safety_arg,
        sim_arg,
        policy_node,
    ])
