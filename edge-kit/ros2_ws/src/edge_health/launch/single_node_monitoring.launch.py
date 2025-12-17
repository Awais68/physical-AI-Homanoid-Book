"""Single node monitoring launch file for edge_health package."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for single node monitoring."""
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='Path to the configuration file'
        ),

        # Health Monitor Node
        Node(
            package='edge_health',
            executable='health_monitor',
            name='health_monitor',
            parameters=[
                LaunchConfiguration('config_file')
            ],
            output='screen',
            emulate_tty=True
        )
    ])