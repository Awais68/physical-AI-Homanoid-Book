#!/usr/bin/env python3
"""HealthMonitor ROS2 node for Physical AI System Monitoring and Control."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from builtin_interfaces.msg import Time

# Import our custom message type
# Note: This import will work once the package is built
# For now, we'll import the generated message
try:
    from edge_health_interfaces.msg import HealthStatus as HealthStatusMsg
except ImportError:
    # Fallback for development
    from edge_health.msg import HealthStatus as HealthStatusMsg

from .health_status import HealthStatus
from .monitoring_config import MonitoringConfiguration
from .system_metrics import get_all_metrics, check_sensors_available
from .logging_utils import setup_component_logger


class HealthMonitor(Node):
    """ROS2 node that monitors system health metrics and publishes them."""

    def __init__(self, config_path: str = None):
        """Initialize the HealthMonitor node."""
        super().__init__('health_monitor')

        # Load configuration
        if config_path:
            self.config = MonitoringConfiguration.from_yaml(config_path)
        else:
            self.config = MonitoringConfiguration.create_default(node_name=self.get_name())

        # Set up logging
        self.logger = setup_component_logger('health_monitor', self.config)

        # Initialize sensor availability check
        self.sensors_available = check_sensors_available()

        # Create publisher for health status
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(HealthStatusMsg, '/health_status', qos_profile)

        # Create timer for periodic health checks
        self.timer = self.create_timer(
            self.config.publish_frequency,
            self.timer_callback
        )

        self.logger.info(f"HealthMonitor initialized with frequency {self.config.publish_frequency}s")
        self.logger.info(f"Enabled metrics: {self.config.enabled_metrics}")

    def timer_callback(self):
        """Timer callback to collect and publish health metrics."""
        try:
            # Collect system metrics
            raw_metrics = get_all_metrics()

            # Filter metrics based on configuration
            filtered_metrics = {}
            for metric_name in self.config.enabled_metrics:
                if metric_name in raw_metrics:
                    filtered_metrics[metric_name] = raw_metrics[metric_name]

            # Determine overall health status based on thresholds
            status = self._determine_health_status(raw_metrics)

            # Create HealthStatus instance
            health_status = HealthStatus(
                node_name=self.config.node_name,
                timestamp=self.get_clock().now().nanoseconds / 1e9,  # Convert to seconds
                cpu_percent=raw_metrics.get('cpu_percent', -1.0),
                memory_percent=raw_metrics.get('memory_percent', -1.0),
                disk_percent=raw_metrics.get('disk_percent', -1.0),
                temperature=raw_metrics.get('temperature', None),
                status=status,
                metrics=filtered_metrics
            )

            # Create and publish ROS2 message
            msg = self._create_health_status_msg(health_status)
            self.publisher.publish(msg)

            # Log the published metrics
            self.logger.debug(f"Published health status: {health_status}")

        except Exception as e:
            self.logger.error(f"Error in timer callback: {e}")
            # Still try to publish a status indicating the error
            try:
                error_msg = HealthStatusMsg()
                error_msg.node_name = self.config.node_name
                error_msg.timestamp = self.get_clock().now().nanoseconds / 1e9
                error_msg.cpu_percent = -1.0
                error_msg.memory_percent = -1.0
                error_msg.disk_percent = -1.0
                error_msg.temperature = -1.0
                error_msg.status = "CRITICAL"
                error_msg.header.stamp = self.get_clock().now().to_msg()
                self.publisher.publish(error_msg)
            except Exception as pub_error:
                self.logger.error(f"Failed to publish error status: {pub_error}")

    def _determine_health_status(self, metrics: dict) -> str:
        """Determine overall health status based on metrics and thresholds."""
        try:
            # Check each enabled metric against its thresholds
            for metric_name in self.config.enabled_metrics:
                if metric_name not in metrics:
                    continue

                value = metrics[metric_name]
                if value < 0:  # Skip invalid metrics
                    continue

                # Get thresholds for this metric
                try:
                    warning_threshold = self.config.get_threshold(metric_name, 'warning')
                    critical_threshold = self.config.get_threshold(metric_name, 'critical')
                except ValueError:
                    # If thresholds don't exist for this metric, skip checking
                    continue

                # Check for critical threshold
                if value >= critical_threshold:
                    return "CRITICAL"

                # Check for warning threshold
                if value >= warning_threshold:
                    return "WARNING"

            # If no thresholds were exceeded, status is OK
            return "OK"

        except Exception as e:
            self.logger.error(f"Error determining health status: {e}")
            return "CRITICAL"  # Default to critical if there's an error

    def _create_health_status_msg(self, health_status: HealthStatus) -> HealthStatusMsg:
        """Create a ROS2 HealthStatus message from a HealthStatus instance."""
        msg = HealthStatusMsg()
        msg.node_name = health_status.node_name
        msg.timestamp = health_status.timestamp
        msg.cpu_percent = health_status.cpu_percent
        msg.memory_percent = health_status.memory_percent
        msg.disk_percent = health_status.disk_percent
        msg.temperature = health_status.temperature if health_status.temperature is not None else -1.0
        msg.status = health_status.status
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg


def main(args=None):
    """Main function to run the HealthMonitor node."""
    rclpy.init(args=args)

    # Get config path from command line arguments if provided
    import sys
    config_path = None
    if len(sys.argv) > 1:
        config_path = sys.argv[1]

    health_monitor = HealthMonitor(config_path=config_path)

    try:
        rclpy.spin(health_monitor)
    except KeyboardInterrupt:
        health_monitor.get_logger().info("Interrupted by user")
    finally:
        health_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()