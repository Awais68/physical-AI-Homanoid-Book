"""End-to-end tests for the complete monitoring workflow."""

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
from edge_health.health_status import HealthStatus
from edge_health.alert import Alert
from edge_health.safe_shutdown_request import SafeShutdownRequest
from edge_health.monitoring_config import MonitoringConfiguration


class TestEndToEndWorkflow(unittest.TestCase):
    """End-to-end tests for the complete monitoring workflow."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        pass

    def test_complete_monitoring_workflow_normal_conditions(self):
        """Test the complete monitoring workflow under normal conditions."""
        # Create configuration
        config = MonitoringConfiguration.create_default(node_name="e2e_test_node")

        # Create components
        threshold_monitor = ThresholdMonitor(config)

        # Simulate normal health status
        normal_status = HealthStatus(
            node_name="e2e_test_node",
            timestamp=time.time(),
            cpu_percent=50.0,
            memory_percent=60.0,
            disk_percent=40.0,
            temperature=35.0,
            status="OK",
            metrics={'cpu_percent': 50.0, 'memory_percent': 60.0}
        )

        # Process through threshold monitor
        alerts = threshold_monitor.check_thresholds(normal_status)

        # Under normal conditions, no alerts should be generated
        self.assertEqual(len(alerts), 0, "No alerts should be generated under normal conditions")

    def test_complete_monitoring_workflow_critical_conditions(self):
        """Test the complete monitoring workflow under critical conditions."""
        # Create configuration
        config = MonitoringConfiguration.create_default(node_name="e2e_critical_test")

        # Create threshold monitor
        threshold_monitor = ThresholdMonitor(config)

        # Simulate critical health status
        critical_status = HealthStatus(
            node_name="e2e_critical_test",
            timestamp=time.time(),
            cpu_percent=98.0,  # Critical
            memory_percent=95.0,  # Critical
            disk_percent=96.0,  # Critical
            temperature=85.0,  # Critical
            status="CRITICAL",
            metrics={'cpu_percent': 98.0, 'memory_percent': 95.0, 'disk_percent': 96.0, 'temperature': 85.0}
        )

        # Process through threshold monitor
        alerts = threshold_monitor.check_thresholds(critical_status)

        # Should generate multiple critical alerts
        self.assertGreaterEqual(len(alerts), 1, "Should generate at least one alert for critical conditions")

        # Check that we have critical alerts
        critical_alerts = [alert for alert in alerts if alert.is_critical()]
        self.assertGreaterEqual(len(critical_alerts), 1, "Should have at least one critical alert")

    def test_workflow_from_health_to_alert_to_shutdown_request(self):
        """Test the complete workflow from health monitoring to alert generation to potential shutdown."""
        # This test simulates the complete chain: HealthStatus -> ThresholdMonitor -> Alerts -> Potential Shutdown Decision

        config = MonitoringConfiguration.create_default(node_name="workflow_test")

        # Create threshold monitor
        threshold_monitor = ThresholdMonitor(config)

        # Simulate a health status that triggers multiple critical alerts
        problematic_status = HealthStatus(
            node_name="workflow_test",
            timestamp=time.time(),
            cpu_percent=98.0,
            memory_percent=88.0,  # Warning level
            disk_percent=97.0,
            temperature=82.0,  # Warning level
            status="CRITICAL",
            metrics={'cpu_percent': 98.0, 'memory_percent': 88.0, 'disk_percent': 97.0, 'temperature': 82.0}
        )

        # Generate alerts from health status
        alerts = threshold_monitor.check_thresholds(problematic_status)

        # Verify we have both critical and warning alerts
        critical_alerts = [a for a in alerts if a.is_critical()]
        warning_alerts = [a for a in alerts if a.is_warning()]

        self.assertGreaterEqual(len(critical_alerts), 1, "Should have critical alerts")
        self.assertGreaterEqual(len(warning_alerts), 1, "Should have warning alerts")

        # Simulate a decision process that might lead to shutdown based on alerts
        # In a real system, this would be handled by a separate component
        has_multiple_critical = len(critical_alerts) >= 2
        has_overtemp_critical = any(a.metric_name == "temperature" and a.is_critical() for a in alerts)

        # Based on the alerts, a shutdown might be warranted
        if has_multiple_critical or has_overtemp_critical:
            # Create a shutdown request based on the critical alerts
            shutdown_request = SafeShutdownRequest.create_emergency_shutdown(
                trigger_reason="MULTIPLE_CRITICAL_ALERTS",
                requested_by="alert_system"
            )

            # Verify the shutdown request
            self.assertIsNotNone(shutdown_request)
            self.assertEqual(shutdown_request.trigger_reason, "MULTIPLE_CRITICAL_ALERTS")
            self.assertIn("save_critical_data", shutdown_request.shutdown_sequence)

    def test_multi_component_integration_scenario(self):
        """Test integration of all components in a realistic scenario."""
        # Create configuration for the test
        config = MonitoringConfiguration(
            node_name="integration_test_node",
            publish_frequency=1.0,
            thresholds={
                'cpu': {'warning': 70.0, 'critical': 85.0},
                'memory': {'warning': 75.0, 'critical': 90.0},
                'disk': {'warning': 80.0, 'critical': 95.0},
                'temperature': {'warning': 65.0, 'critical': 75.0}
            },
            enabled_metrics=['cpu', 'memory', 'disk', 'temperature'],
            log_level='INFO',
            hardware_specific={},
            alert_recipients=['test_system']
        )

        # Initialize components
        threshold_monitor = ThresholdMonitor(config)

        # Simulate a sequence of health statuses over time
        health_sequence = [
            # Normal conditions
            HealthStatus(
                node_name="integration_test_node",
                timestamp=time.time(),
                cpu_percent=40.0,
                memory_percent=50.0,
                disk_percent=30.0,
                temperature=30.0,
                status="OK",
                metrics={'cpu_percent': 40.0, 'memory_percent': 50.0}
            ),
            # Gradually increasing stress
            HealthStatus(
                node_name="integration_test_node",
                timestamp=time.time() + 1,
                cpu_percent=75.0,  # Warning level
                memory_percent=78.0,  # Warning level
                disk_percent=40.0,
                temperature=68.0,  # Warning level
                status="WARNING",
                metrics={'cpu_percent': 75.0, 'memory_percent': 78.0}
            ),
            # Critical conditions
            HealthStatus(
                node_name="integration_test_node",
                timestamp=time.time() + 2,
                cpu_percent=92.0,  # Critical level
                memory_percent=92.0,  # Critical level
                disk_percent=96.0,  # Critical level
                temperature=78.0,  # Critical level
                status="CRITICAL",
                metrics={'cpu_percent': 92.0, 'memory_percent': 92.0}
            )
        ]

        # Process the sequence and collect alerts
        all_alerts = []
        for health_status in health_sequence:
            alerts = threshold_monitor.check_thresholds(health_status)
            all_alerts.extend(alerts)

        # Analyze the collected alerts
        critical_alerts = [a for a in all_alerts if a.is_critical()]
        warning_alerts = [a for a in all_alerts if a.is_warning()]

        # Verify that we got appropriate alerts for the conditions
        self.assertGreaterEqual(len(critical_alerts), 1, "Should have critical alerts for critical conditions")
        self.assertGreaterEqual(len(warning_alerts), 1, "Should have warning alerts for warning conditions")

        # Verify alert diversity
        alert_metrics = {a.metric_name for a in all_alerts}
        expected_metrics = {'cpu', 'memory', 'disk', 'temperature'}
        self.assertTrue(alert_metrics.issubset(expected_metrics), "Alerts should only be for monitored metrics")

    def test_configuration_consistency_across_workflow(self):
        """Test that configuration is consistently applied across the workflow."""
        # Create a specific configuration
        test_config = MonitoringConfiguration(
            node_name="config_consistency_test",
            publish_frequency=2.0,  # Different from default
            thresholds={
                'cpu': {'warning': 60.0, 'critical': 70.0},  # More sensitive than default
                'memory': {'warning': 70.0, 'critical': 80.0},
                'disk': {'warning': 70.0, 'critical': 85.0},
                'temperature': {'warning': 55.0, 'critical': 65.0}  # More sensitive
            },
            enabled_metrics=['cpu', 'memory'],
            log_level='DEBUG',
            hardware_specific={'test_mode': True},
            alert_recipients=['test_system']
        )

        # Create threshold monitor with this config
        threshold_monitor = ThresholdMonitor(test_config)

        # Test with values that would be critical under this config but not under default
        sensitive_status = HealthStatus(
            node_name="config_consistency_test",
            timestamp=time.time(),
            cpu_percent=65.0,  # Would be warning with this config (60 warning threshold), but OK with default (80 threshold)
            memory_percent=75.0,  # Would be warning with this config (70 warning threshold), but OK with default (85 threshold)
            disk_percent=50.0,  # Not enabled in this config
            temperature=60.0,  # Would be warning with this config (55 threshold), but OK with default (70 threshold)
            status="WARNING",
            metrics={'cpu_percent': 65.0, 'memory_percent': 75.0}
        )

        alerts = threshold_monitor.check_thresholds(sensitive_status)

        # Should have alerts because values exceed the custom thresholds
        # But only for enabled metrics (cpu and memory, not disk or temperature)
        self.assertGreaterEqual(len(alerts), 1, "Should have alerts based on custom configuration")

        # Verify that only enabled metrics generated alerts
        alert_metrics = {a.metric_name for a in alerts}
        enabled_metrics = {m.replace('_percent', '') for m in test_config.enabled_metrics}
        self.assertTrue(alert_metrics.issubset(enabled_metrics), "Only enabled metrics should generate alerts")

    def test_workflow_resilience_to_invalid_data(self):
        """Test that the workflow handles edge case data gracefully."""
        config = MonitoringConfiguration.create_default(node_name="resilience_test")
        threshold_monitor = ThresholdMonitor(config)

        # Test with edge case data scenarios that are valid but edge cases
        edge_case_statuses = [
            # Status with boundary values (valid but edge cases)
            HealthStatus(
                node_name="resilience_test",
                timestamp=time.time(),
                cpu_percent=0.0,  # Minimum valid value
                memory_percent=100.0,  # Maximum valid value
                disk_percent=0.0,
                temperature=0.0,
                status="OK",
                metrics={'cpu_percent': 0.0}
            ),
            # Status with None temperature (valid edge case)
            HealthStatus(
                node_name="resilience_test",
                timestamp=time.time() + 1,
                cpu_percent=50.0,
                memory_percent=50.0,
                disk_percent=50.0,
                temperature=None,  # No temperature sensor
                status="OK",
                metrics={'cpu_percent': 50.0, 'memory_percent': 50.0}
            ),
        ]

        # The system should handle edge case data gracefully
        for edge_status in edge_case_statuses:
            try:
                alerts = threshold_monitor.check_thresholds(edge_status)
                # Should not crash and should handle edge case metrics appropriately
                self.assertIsInstance(alerts, list, "Should return a list even with edge case data")
            except Exception as e:
                self.fail(f"Workflow should handle edge case data gracefully, but raised: {e}")

    def test_alert_recovery_workflow(self):
        """Test the complete workflow including recovery from alert states."""
        config = MonitoringConfiguration.create_default(node_name="recovery_test")
        threshold_monitor = ThresholdMonitor(config)

        # First, create a critical situation
        critical_status = HealthStatus(
            node_name="recovery_test",
            timestamp=time.time(),
            cpu_percent=95.0,
            memory_percent=60.0,
            disk_percent=40.0,
            temperature=30.0,
            status="CRITICAL",
            metrics={'cpu_percent': 95.0}
        )

        initial_alerts = threshold_monitor.check_thresholds(critical_status)
        self.assertGreaterEqual(len(initial_alerts), 1, "Should generate initial alerts")

        # Then, return to normal conditions (this should eventually trigger recovery)
        normal_status = HealthStatus(
            node_name="recovery_test",
            timestamp=time.time() + 10,  # Later timestamp
            cpu_percent=50.0,
            memory_percent=60.0,
            disk_percent=40.0,
            temperature=30.0,
            status="OK",
            metrics={'cpu_percent': 50.0}
        )

        recovery_alerts = threshold_monitor.check_thresholds(normal_status)
        # Depending on implementation, this might generate recovery alerts


def run_tests():
    """Run the end-to-end tests."""
    unittest.main()


if __name__ == '__main__':
    run_tests()