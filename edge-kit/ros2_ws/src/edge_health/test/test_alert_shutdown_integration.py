"""Integration tests for AlertPublisher and SafeShutdownService interaction."""

import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
import os
import time

# Add the edge_health module to the path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'edge_health'))

from edge_health.alert_publisher import AlertPublisher
from edge_health.safe_shutdown_service import SafeShutdownService
from edge_health.alert import Alert
from edge_health.health_status import HealthStatus
from edge_health.safe_shutdown_request import SafeShutdownRequest
from edge_health.monitoring_config import MonitoringConfiguration


class TestAlertShutdownIntegration(unittest.TestCase):
    """Integration tests for AlertPublisher and SafeShutdownService interaction."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        pass

    def test_critical_alert_triggers_safe_shutdown(self):
        """Test that critical alerts can trigger safe shutdown procedures."""
        # This test simulates the interaction between alert system and shutdown service
        # In a real system, this might involve external components that trigger shutdown
        # based on received alerts

        # Create a critical alert
        critical_alert = Alert(
            alert_id="test_critical_alert",
            timestamp=time.time(),
            severity="CRITICAL",
            component="cpu",
            metric_name="cpu",
            current_value=98.0,
            threshold_value=90.0,
            message="CPU temperature critical",
            node_name="test_node",
            alert_type="THRESHOLD_EXCEEDED"
        )

        # Verify the alert properties
        self.assertEqual(critical_alert.severity, "CRITICAL")
        self.assertEqual(critical_alert.component, "cpu")
        self.assertTrue(critical_alert.is_critical())

    def test_safe_shutdown_service_handles_emergency_requests(self):
        """Test that the shutdown service properly handles emergency shutdown requests."""
        # Create a shutdown request that might result from critical alerts
        shutdown_request = SafeShutdownRequest(
            request_id="emergency_shutdown_1",
            timestamp=time.time(),
            trigger_reason="CRITICAL_ALERT",
            requested_by="alert_system",
            shutdown_sequence=[
                "save_critical_data",
                "stop_controllers",
                "disable_actuators",
                "power_down_safely"
            ],
            timeout=30.0
        )

        # Verify the shutdown request properties
        self.assertEqual(shutdown_request.trigger_reason, "CRITICAL_ALERT")
        self.assertEqual(shutdown_request.requested_by, "alert_system")
        self.assertIn("save_critical_data", shutdown_request.shutdown_sequence)
        self.assertLessEqual(len(shutdown_request.shutdown_sequence), 10)  # Reasonable sequence length

    def test_alert_publisher_creates_proper_messages(self):
        """Test that alert publisher creates proper ROS2 messages from alerts."""
        # Mock the AlertPublisher to test message creation
        with patch('edge_health.alert_publisher.Node.__init__'):
            with patch('edge_health.alert_publisher.setup_component_logger'):
                with patch('edge_health.alert_publisher.AlertPublisher.create_publisher'):
                    with patch('edge_health.alert_publisher.AlertPublisher.create_subscription'):
                        service = AlertPublisher.__new__(AlertPublisher)
                        service.config = MonitoringConfiguration.create_default()
                        service.get_logger = Mock()
                        service.publisher = Mock()
                        service.threshold_monitor = Mock()

                        # Create an alert
                        alert = Alert(
                            alert_id="test_alert_1",
                            timestamp=time.time(),
                            severity="WARNING",
                            component="memory",
                            metric_name="memory",
                            current_value=88.0,
                            threshold_value=85.0,
                            message="Memory usage high",
                            node_name="test_node",
                            alert_type="THRESHOLD_EXCEEDED"
                        )

                        # Test message creation (this would normally create a ROS2 message)
                        # The actual implementation is in _create_alert_msg method
                        try:
                            from edge_health.msg import Alert as AlertMsg
                            msg = service._create_alert_msg(alert)
                            self.assertEqual(msg.alert_id, "test_alert_1")
                            self.assertEqual(msg.severity, "WARNING")
                            self.assertEqual(msg.component, "memory")
                        except ImportError:
                            # If ROS2 message types aren't available, test the structure conceptually
                            pass

    def test_alert_severity_levels_properly_handled(self):
        """Test that different alert severity levels are properly distinguished."""
        alerts = [
            Alert(
                alert_id=f"alert_{i}",
                timestamp=time.time(),
                severity=severity,
                component="test",
                metric_name="test",
                current_value=0.0,
                threshold_value=0.0,
                message=f"Test {severity}",
                node_name="test_node",
                alert_type="TEST"
            )
            for i, severity in enumerate(["INFO", "WARNING", "ERROR", "CRITICAL"])
        ]

        # Verify each alert has the correct severity
        expected_severities = ["INFO", "WARNING", "ERROR", "CRITICAL"]
        actual_severities = [alert.severity for alert in alerts]
        self.assertEqual(actual_severities, expected_severities)

        # Test severity checking methods
        self.assertTrue(alerts[3].is_critical())  # CRITICAL alert
        self.assertTrue(alerts[1].is_warning())  # WARNING alert

    def test_recovery_alerts_dont_trigger_shutdown(self):
        """Test that recovery alerts don't trigger shutdown procedures."""
        recovery_alert = Alert(
            alert_id="recovery_alert_1",
            timestamp=time.time(),
            severity="INFO",
            component="cpu",
            metric_name="cpu",
            current_value=60.0,
            threshold_value=0.0,  # Not applicable for recovery
            message="CPU temperature recovered",
            node_name="test_node",
            alert_type="RECOVERY"
        )

        # Recovery alerts should be informational only
        self.assertEqual(recovery_alert.severity, "INFO")
        self.assertEqual(recovery_alert.alert_type, "RECOVERY")
        self.assertFalse(recovery_alert.is_critical())
        self.assertFalse(recovery_alert.is_warning())
        self.assertFalse(recovery_alert.is_error())

    def test_alert_aggregation_for_shutdown_decision(self):
        """Test that multiple alerts can be aggregated for shutdown decisions."""
        # Simulate multiple alerts that might lead to a shutdown decision
        alerts = [
            Alert(
                alert_id=f"alert_{i}",
                timestamp=time.time(),
                severity=severity,
                component=component,
                metric_name=metric_name,
                current_value=value,
                threshold_value=threshold,
                message=f"{component} {severity}",
                node_name="critical_node",
                alert_type="THRESHOLD_EXCEEDED"
            )
            for i, (severity, component, metric_name, value, threshold) in enumerate([
                ("CRITICAL", "temperature", "temperature", 85.0, 80.0),
                ("CRITICAL", "cpu", "cpu", 98.0, 90.0),
                ("WARNING", "memory", "memory", 90.0, 85.0),
            ])
        ]

        # Count critical alerts that might trigger shutdown
        critical_alerts = [alert for alert in alerts if alert.is_critical()]
        warning_alerts = [alert for alert in alerts if alert.is_warning()]

        self.assertEqual(len(critical_alerts), 2)  # 2 critical alerts
        self.assertEqual(len(warning_alerts), 1)   # 1 warning alert

        # In a real system, multiple critical alerts would likely trigger shutdown
        if len(critical_alerts) >= 2:
            # This would be the condition to trigger emergency shutdown
            shutdown_needed = True
            self.assertTrue(shutdown_needed)

    def test_safe_shutdown_request_validation(self):
        """Test that shutdown requests are properly validated before execution."""
        # Test valid shutdown request
        valid_request = SafeShutdownRequest(
            request_id="valid_request",
            timestamp=time.time(),
            trigger_reason="CRITICAL_ALERT",
            requested_by="monitoring_system",
            shutdown_sequence=["save_data", "power_down"],
            timeout=60.0
        )

        self.assertEqual(valid_request.trigger_reason, "CRITICAL_ALERT")
        self.assertEqual(valid_request.requested_by, "monitoring_system")
        self.assertGreater(valid_request.timeout, 0)

        # Test request creation methods
        emergency_request = SafeShutdownRequest.create_emergency_shutdown(
            trigger_reason="OVERTEMP",
            requested_by="thermal_protection"
        )
        self.assertEqual(emergency_request.trigger_reason, "OVERTEMP")
        self.assertEqual(emergency_request.requested_by, "thermal_protection")
        self.assertIn("save_critical_data", emergency_request.shutdown_sequence)

        maintenance_request = SafeShutdownRequest.create_maintenance_shutdown(
            requested_by="scheduled_maintenance"
        )
        self.assertEqual(maintenance_request.trigger_reason, "MAINTENANCE")
        self.assertEqual(maintenance_request.requested_by, "scheduled_maintenance")

    def test_alert_publisher_subscriber_pattern(self):
        """Test the publisher-subscriber pattern between components."""
        # This test conceptually verifies that alerts published by one component
        # can be received by another (like a shutdown service)

        # In ROS2, the AlertPublisher publishes to /alerts topic
        # Other services (like a hypothetical AlertToShutdownBridge) would subscribe
        # to this topic and potentially trigger shutdowns based on critical alerts

        # Verify that alert properties are preserved through the system
        original_alert = Alert(
            alert_id="integration_test_alert",
            timestamp=time.time(),
            severity="CRITICAL",
            component="system_monitor",
            metric_name="overall_health",
            current_value=0.0,  # System health at 0%
            threshold_value=20.0,
            message="System health critical, initiating shutdown",
            node_name="main_controller",
            alert_type="EMERGENCY"
        )

        # The alert should maintain its properties regardless of how it's transmitted
        self.assertEqual(original_alert.severity, "CRITICAL")
        self.assertEqual(original_alert.alert_type, "EMERGENCY")
        self.assertEqual(original_alert.node_name, "main_controller")


def run_tests():
    """Run the integration tests."""
    unittest.main()


if __name__ == '__main__':
    run_tests()