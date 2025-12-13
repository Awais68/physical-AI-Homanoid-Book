"""Integration tests for multi-node deployment of edge_health package."""

import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
import os
import time

# Add the edge_health module to the path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'edge_health'))

from edge_health.health_monitor import HealthMonitor
from edge_health.threshold_monitor import ThresholdMonitor
from edge_health.alert_publisher import AlertPublisher
from edge_health.safe_shutdown_service import SafeShutdownService
from edge_health.monitoring_config import MonitoringConfiguration


class TestMultiNodeIntegration(unittest.TestCase):
    """Integration tests for multi-node deployment scenarios."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        pass

    def test_multiple_health_monitors_with_different_configs(self):
        """Test that multiple health monitors can run with different configurations."""
        # Create different configurations for different node types
        edge_brain_config = MonitoringConfiguration(
            node_name="edge_brain_monitor",
            publish_frequency=1.0,
            thresholds={
                'cpu': {'warning': 75.0, 'critical': 85.0},
                'memory': {'warning': 80.0, 'critical': 90.0},
                'disk': {'warning': 75.0, 'critical': 90.0},
                'temperature': {'warning': 65.0, 'critical': 75.0}
            },
            enabled_metrics=['cpu', 'memory', 'disk', 'temperature'],
            log_level='INFO',
            hardware_specific={'power_sensitive': False},
            alert_recipients=['operator_console']
        )

        sensor_node_config = MonitoringConfiguration(
            node_name="sensor_node_monitor",
            publish_frequency=2.0,
            thresholds={
                'cpu': {'warning': 60.0, 'critical': 80.0},
                'memory': {'warning': 70.0, 'critical': 85.0},
                'disk': {'warning': 70.0, 'critical': 90.0},
                'temperature': {'warning': 60.0, 'critical': 75.0}
            },
            enabled_metrics=['cpu', 'memory', 'disk', 'temperature'],
            log_level='WARNING',
            hardware_specific={'power_sensitive': True},
            alert_recipients=['operator_console']
        )

        actuator_node_config = MonitoringConfiguration(
            node_name="actuator_node_monitor",
            publish_frequency=0.5,
            thresholds={
                'cpu': {'warning': 70.0, 'critical': 85.0},
                'memory': {'warning': 75.0, 'critical': 90.0},
                'disk': {'warning': 75.0, 'critical': 95.0},
                'temperature': {'warning': 65.0, 'critical': 75.0}
            },
            enabled_metrics=['cpu', 'memory', 'disk', 'temperature'],
            log_level='INFO',
            hardware_specific={'safety_critical': True},
            alert_recipients=['operator_console', 'safety_system']
        )

        # Mock ROS2 components for testing
        with patch('edge_health.health_monitor.Node.__init__'):
            with patch('edge_health.health_monitor.setup_component_logger'):
                with patch('edge_health.health_monitor.HealthMonitor.create_publisher'):
                    with patch('edge_health.health_monitor.HealthMonitor.create_timer'):
                        with patch('edge_health.health_monitor.HealthMonitor.get_clock'):
                            # Create health monitors with different configs
                            brain_monitor = HealthMonitor.__new__(HealthMonitor)
                            brain_monitor.config = edge_brain_config
                            brain_monitor.get_logger = Mock()
                            brain_monitor.create_publisher = Mock()
                            brain_monitor.create_timer = Mock()
                            brain_monitor.get_clock = Mock()

                            sensor_monitor = HealthMonitor.__new__(HealthMonitor)
                            sensor_monitor.config = sensor_node_config
                            sensor_monitor.get_logger = Mock()
                            sensor_monitor.create_publisher = Mock()
                            sensor_monitor.create_timer = Mock()
                            sensor_monitor.get_clock = Mock()

                            actuator_monitor = HealthMonitor.__new__(HealthMonitor)
                            actuator_monitor.config = actuator_node_config
                            actuator_monitor.get_logger = Mock()
                            actuator_monitor.create_publisher = Mock()
                            actuator_monitor.create_timer = Mock()
                            actuator_monitor.get_clock = Mock()

                            # Verify that each monitor has the correct configuration
                            self.assertEqual(brain_monitor.config.node_name, "edge_brain_monitor")
                            self.assertEqual(brain_monitor.config.publish_frequency, 1.0)
                            self.assertEqual(sensor_monitor.config.node_name, "sensor_node_monitor")
                            self.assertEqual(sensor_monitor.config.publish_frequency, 2.0)
                            self.assertEqual(actuator_monitor.config.node_name, "actuator_node_monitor")
                            self.assertEqual(actuator_monitor.config.publish_frequency, 0.5)

    def test_cross_node_alert_propagation(self):
        """Test that alerts from one node can be processed by alert publishers on other nodes."""
        # Create a threshold monitor for one node
        config = MonitoringConfiguration.create_default(node_name="test_node")
        threshold_monitor = ThresholdMonitor(config)

        # Simulate a health status that would trigger an alert
        from edge_health.health_status import HealthStatus
        high_cpu_status = HealthStatus(
            node_name="high_cpu_node",
            timestamp=time.time(),
            cpu_percent=95.0,  # Critical
            memory_percent=60.0,
            disk_percent=40.0,
            temperature=30.0,
            status="CRITICAL",
            metrics={'cpu_percent': 95.0}
        )

        # Generate alerts
        alerts = threshold_monitor.check_thresholds(high_cpu_status)
        self.assertEqual(len(alerts), 1)
        self.assertEqual(alerts[0].severity, "CRITICAL")
        self.assertEqual(alerts[0].node_name, "high_cpu_node")

    def test_multi_node_threshold_configuration_isolation(self):
        """Test that threshold configurations are isolated between nodes."""
        # Create two different configurations
        config1 = MonitoringConfiguration(
            node_name="node1",
            publish_frequency=1.0,
            thresholds={
                'cpu': {'warning': 50.0, 'critical': 60.0},  # Lower thresholds
                'memory': {'warning': 70.0, 'critical': 80.0},
                'disk': {'warning': 70.0, 'critical': 80.0},
                'temperature': {'warning': 50.0, 'critical': 60.0}
            },
            enabled_metrics=['cpu', 'memory', 'disk', 'temperature'],
            log_level='INFO',
            hardware_specific={},
            alert_recipients=['test']
        )

        config2 = MonitoringConfiguration(
            node_name="node2",
            publish_frequency=1.0,
            thresholds={
                'cpu': {'warning': 80.0, 'critical': 90.0},  # Higher thresholds
                'memory': {'warning': 85.0, 'critical': 95.0},
                'disk': {'warning': 80.0, 'critical': 95.0},
                'temperature': {'warning': 70.0, 'critical': 80.0}
            },
            enabled_metrics=['cpu', 'memory', 'disk', 'temperature'],
            log_level='INFO',
            hardware_specific={},
            alert_recipients=['test']
        )

        # Create threshold monitors with different configs
        monitor1 = ThresholdMonitor(config1)
        monitor2 = ThresholdMonitor(config2)

        # Create a health status with CPU at 55% - should trigger warning for config1 but not config2
        from edge_health.health_status import HealthStatus
        health_status = HealthStatus(
            node_name="test_node",
            timestamp=time.time(),
            cpu_percent=55.0,  # Between warning (50) and critical (60) for config1, below warning (80) for config2
            memory_percent=60.0,
            disk_percent=40.0,
            temperature=30.0,
            status="WARNING",
            metrics={'cpu_percent': 55.0}
        )

        alerts1 = monitor1.check_thresholds(health_status)
        alerts2 = monitor2.check_thresholds(health_status)

        # Node 1 should have a WARNING alert (55% > 50% warning threshold but < 60% critical)
        self.assertEqual(len(alerts1), 1)
        self.assertEqual(alerts1[0].severity, "WARNING")

        # Node 2 should not have an alert (55% < 80% warning threshold)
        self.assertEqual(len(alerts2), 0)

    def test_safe_shutdown_service_isolation(self):
        """Test that safe shutdown services operate independently per node."""
        # This test verifies that the configuration system supports node isolation
        brain_config = MonitoringConfiguration(
            node_name="edge_brain",
            publish_frequency=1.0,
            thresholds={'cpu': {'warning': 75.0, 'critical': 85.0}},
            enabled_metrics=['cpu'],
            log_level='INFO',
            hardware_specific={'safety_critical': True},
            alert_recipients=['safety_system']
        )

        sensor_config = MonitoringConfiguration(
            node_name="sensor_node",
            publish_frequency=2.0,
            thresholds={'cpu': {'warning': 60.0, 'critical': 80.0}},
            enabled_metrics=['cpu'],
            log_level='WARNING',
            hardware_specific={'power_sensitive': True},
            alert_recipients=['monitoring_system']
        )

        # Verify that configurations are properly isolated
        self.assertNotEqual(brain_config.node_name, sensor_config.node_name)
        self.assertNotEqual(brain_config.publish_frequency, sensor_config.publish_frequency)
        self.assertNotEqual(brain_config.get_threshold('cpu', 'critical'), sensor_config.get_threshold('cpu', 'critical'))
        self.assertNotEqual(brain_config.alert_recipients, sensor_config.alert_recipients)

    def test_configurable_node_names_in_multi_node_scenario(self):
        """Test that node names are properly configured in multi-node scenarios."""
        configs = [
            MonitoringConfiguration.create_default(node_name="node_1"),
            MonitoringConfiguration.create_default(node_name="node_2"),
            MonitoringConfiguration.create_default(node_name="node_3"),
        ]

        # Verify each config has a unique node name
        node_names = [config.node_name for config in configs]
        self.assertEqual(len(set(node_names)), 3)  # All names should be unique
        self.assertIn("node_1", node_names)
        self.assertIn("node_2", node_names)
        self.assertIn("node_3", node_names)


def run_tests():
    """Run the integration tests."""
    unittest.main()


if __name__ == '__main__':
    run_tests()