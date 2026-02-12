"""Multi-node monitoring launch file for edge_health package."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for multi-node monitoring."""
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'edge_brain_config',
            default_value='/path/to/edge_brain_config.yaml',
            description='Path to the edge brain configuration file'
        ),
        DeclareLaunchArgument(
            'sensor_node_config',
            default_value='/path/to/sensor_node_config.yaml',
            description='Path to the sensor node configuration file'
        ),
        DeclareLaunchArgument(
            'actuator_node_config',
            default_value='/path/to/actuator_node_config.yaml',
            description='Path to the actuator node configuration file'
        ),

        # Edge Brain Health Monitor Node
        Node(
            package='edge_health',
            executable='health_monitor',
            name='edge_brain_health_monitor',
            namespace='edge_brain',
            parameters=[
                LaunchConfiguration('edge_brain_config')
            ],
            output='screen',
            emulate_tty=True
        ),

        # Sensor Node Health Monitor
        Node(
            package='edge_health',
            executable='health_monitor',
            name='sensor_node_health_monitor',
            namespace='sensor_node',
            parameters=[
                LaunchConfiguration('sensor_node_config')
            ],
            output='screen',
            emulate_tty=True
        ),

        # Actuator Node Health Monitor
        Node(
            package='edge_health',
            executable='health_monitor',
            name='actuator_node_health_monitor',
            namespace='actuator_node',
            parameters=[
                LaunchConfiguration('actuator_node_config')
            ],
            output='screen',
            emulate_tty=True
        ),

        # Edge Brain Alert Publisher
        Node(
            package='edge_health',
            executable='alert_publisher',
            name='edge_brain_alert_publisher',
            namespace='edge_brain',
            parameters=[
                LaunchConfiguration('edge_brain_config')
            ],
            output='screen',
            emulate_tty=True
        ),

        # Edge Brain Safe Shutdown Service
        Node(
            package='edge_health',
            executable='safe_shutdown_service',
            name='edge_brain_safe_shutdown_service',
            namespace='edge_brain',
            parameters=[
                LaunchConfiguration('edge_brain_config')
            ],
            output='screen',
            emulate_tty=True
        )
    ])