#!/usr/bin/env python3
"""AlertPublisher ROS2 node for Physical AI System Monitoring and Control."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

# Import our custom message types
try:
    from edge_health_interfaces.msg import Alert as AlertMsg
    from edge_health_interfaces.msg import HealthStatus as HealthStatusMsg
except ImportError:
    # Fallback for development
    from edge_health.msg import Alert as AlertMsg
    from edge_health.msg import HealthStatus as HealthStatusMsg

from .threshold_monitor import ThresholdMonitor
from .health_status import HealthStatus
from .monitoring_config import MonitoringConfiguration
from .logging_utils import setup_component_logger


class AlertPublisher(Node):
    """ROS2 node that publishes alerts when thresholds are exceeded."""

    def __init__(self, config_path: str = None):
        """Initialize the AlertPublisher node."""
        super().__init__('alert_publisher')

        # Load configuration
        if config_path:
            self.config = MonitoringConfiguration.from_yaml(config_path)
        else:
            self.config = MonitoringConfiguration.create_default(node_name=self.get_name())

        # Set up logging
        self.logger = setup_component_logger('alert_publisher', self.config)

        # Create threshold monitor
        self.threshold_monitor = ThresholdMonitor(self.config)

        # Create publisher for alerts
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(AlertMsg, '/alerts', qos_profile)

        # Create subscription to health status topic
        self.health_subscription = self.create_subscription(
            HealthStatusMsg,
            '/health_status',
            self.health_status_callback,
            qos_profile
        )

        self.logger.info("AlertPublisher initialized and subscribed to /health_status")

    def health_status_callback(self, msg):
        """
        Callback function for health status messages.

        Args:
            msg: HealthStatus message from the health monitor
        """
        try:
            # Convert ROS2 message to HealthStatus object
            health_status = HealthStatus(
                node_name=msg.node_name,
                timestamp=msg.timestamp,
                cpu_percent=msg.cpu_percent,
                memory_percent=msg.memory_percent,
                disk_percent=msg.disk_percent,
                temperature=msg.temperature if msg.temperature >= 0 else None,
                status=msg.status,
                metrics={}  # We can populate this if needed
            )

            # Check thresholds and get alerts
            alerts = self.threshold_monitor.check_thresholds(health_status)

            # Publish any generated alerts
            for alert in alerts:
                alert_msg = self._create_alert_msg(alert)
                self.publisher.publish(alert_msg)
                self.logger.info(f"Published alert: {alert.severity} - {alert.message}")

        except Exception as e:
            self.logger.error(f"Error processing health status: {e}")

    def _create_alert_msg(self, alert) -> AlertMsg:
        """
        Create a ROS2 Alert message from an Alert instance.

        Args:
            alert: Alert instance to convert

        Returns:
            AlertMsg ROS2 message
        """
        msg = AlertMsg()
        msg.alert_id = alert.alert_id
        msg.timestamp = alert.timestamp
        msg.severity = alert.severity
        msg.component = alert.component
        msg.metric_name = alert.metric_name
        msg.current_value = alert.current_value
        msg.threshold_value = alert.threshold_value
        msg.message = alert.message
        msg.node_name = alert.node_name
        msg.alert_type = alert.alert_type
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg


def main(args=None):
    """Main function to run the AlertPublisher node."""
    rclpy.init(args=args)

    # Get config path from command line arguments if provided
    import sys
    config_path = None
    if len(sys.argv) > 1:
        config_path = sys.argv[1]

    alert_publisher = AlertPublisher(config_path=config_path)

    try:
        rclpy.spin(alert_publisher)
    except KeyboardInterrupt:
        alert_publisher.get_logger().info("Interrupted by user")
    finally:
        alert_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()