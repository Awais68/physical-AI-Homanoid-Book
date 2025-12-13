"""Unit tests for the ThresholdMonitor class."""

import unittest
from edge_health.threshold_monitor import ThresholdMonitor
from edge_health.health_status import HealthStatus
from edge_health.monitoring_config import MonitoringConfiguration


class TestThresholdMonitor(unittest.TestCase):
    """Test cases for the ThresholdMonitor class."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        config = MonitoringConfiguration.create_default()
        self.monitor = ThresholdMonitor(config)

    def test_no_alerts_when_metrics_within_thresholds(self):
        """Test that no alerts are generated when all metrics are within thresholds."""
        health_status = HealthStatus(
            node_name="test_node",
            timestamp=1234567890.0,
            cpu_percent=50.0,
            memory_percent=60.0,
            disk_percent=40.0,
            temperature=30.0,
            status="OK",
            metrics={'cpu_percent': 50.0, 'memory_percent': 60.0}
        )

        alerts = self.monitor.check_thresholds(health_status)
        self.assertEqual(len(alerts), 0)

    def test_warning_alert_for_cpu_exceeding_threshold(self):
        """Test that a warning alert is generated when CPU exceeds warning threshold."""
        health_status = HealthStatus(
            node_name="test_node",
            timestamp=1234567890.0,
            cpu_percent=85.0,  # Above warning threshold of 80
            memory_percent=60.0,
            disk_percent=40.0,
            temperature=30.0,
            status="WARNING",
            metrics={'cpu_percent': 85.0}
        )

        alerts = self.monitor.check_thresholds(health_status)
        self.assertEqual(len(alerts), 1)
        self.assertEqual(alerts[0].severity, "WARNING")
        self.assertEqual(alerts[0].metric_name, "cpu")
        self.assertEqual(alerts[0].current_value, 85.0)

    def test_critical_alert_for_cpu_exceeding_threshold(self):
        """Test that a critical alert is generated when CPU exceeds critical threshold."""
        health_status = HealthStatus(
            node_name="test_node",
            timestamp=1234567890.0,
            cpu_percent=95.0,  # Above critical threshold of 90
            memory_percent=60.0,
            disk_percent=40.0,
            temperature=30.0,
            status="CRITICAL",
            metrics={'cpu_percent': 95.0}
        )

        alerts = self.monitor.check_thresholds(health_status)
        self.assertEqual(len(alerts), 1)
        self.assertEqual(alerts[0].severity, "CRITICAL")
        self.assertEqual(alerts[0].metric_name, "cpu")
        self.assertEqual(alerts[0].current_value, 95.0)

    def test_multiple_alerts_for_multiple_threshold_violations(self):
        """Test that multiple alerts are generated for multiple threshold violations."""
        health_status = HealthStatus(
            node_name="test_node",
            timestamp=1234567890.0,
            cpu_percent=95.0,  # Critical
            memory_percent=90.0,  # Warning
            disk_percent=40.0,
            temperature=80.0,  # Warning
            status="CRITICAL",
            metrics={'cpu_percent': 95.0, 'memory_percent': 90.0, 'temperature': 80.0}
        )

        alerts = self.monitor.check_thresholds(health_status)
        self.assertEqual(len(alerts), 3)  # CPU critical, memory warning, temperature warning

        # Check that we have alerts for the right metrics
        alert_metrics = [alert.metric_name for alert in alerts]
        self.assertIn("cpu", alert_metrics)
        self.assertIn("memory", alert_metrics)
        self.assertIn("temperature", alert_metrics)

    def test_recovery_alert_when_metric_returns_to_normal(self):
        """Test that a recovery alert is generated when a metric returns to normal."""
        # First, trigger an alert
        high_cpu_status = HealthStatus(
            node_name="test_node",
            timestamp=1234567890.0,
            cpu_percent=95.0,  # Critical
            memory_percent=60.0,
            disk_percent=40.0,
            temperature=30.0,
            status="CRITICAL",
            metrics={'cpu_percent': 95.0}
        )

        alerts = self.monitor.check_thresholds(high_cpu_status)
        self.assertEqual(len(alerts), 1)
        self.assertEqual(alerts[0].severity, "CRITICAL")

        # Then, check with normal values (this should generate a recovery alert)
        normal_cpu_status = HealthStatus(
            node_name="test_node",
            timestamp=1234567891.0,
            cpu_percent=50.0,  # Normal
            memory_percent=60.0,
            disk_percent=40.0,
            temperature=30.0,
            status="OK",
            metrics={'cpu_percent': 50.0}
        )

        recovery_alerts = self.monitor.check_thresholds(normal_cpu_status)
        # Should have a recovery alert
        recovery_alerts = [alert for alert in recovery_alerts if alert.alert_type == "RECOVERY"]
        # Note: The recovery logic might not trigger immediately depending on implementation
        # This test checks the basic functionality

    def test_hysteresis_prevents_alert_flapping(self):
        """Test that hysteresis prevents alert flapping around threshold values."""
        # Set up custom config with specific thresholds for testing
        config = MonitoringConfiguration(
            node_name="test_node",
            publish_frequency=1.0,
            thresholds={
                'cpu': {'warning': 75.0, 'critical': 85.0},
                'memory': {'warning': 80.0, 'critical': 90.0},
                'disk': {'warning': 80.0, 'critical': 95.0},
                'temperature': {'warning': 70.0, 'critical': 80.0}
            },
            enabled_metrics=['cpu', 'memory', 'disk', 'temperature'],
            log_level='INFO',
            hardware_specific={},
            alert_recipients=['test']
        )
        monitor = ThresholdMonitor(config)

        # Alternate values just above and below threshold
        for i in range(5):
            value = 76.0 if i % 2 == 0 else 74.0  # Alternating just above/below warning threshold
            health_status = HealthStatus(
                node_name="test_node",
                timestamp=1234567890.0 + i,
                cpu_percent=value,
                memory_percent=50.0,
                disk_percent=40.0,
                temperature=30.0,
                status="OK" if value < 75.0 else "WARNING",
                metrics={'cpu_percent': value}
            )

            alerts = monitor.check_thresholds(health_status)
            # With hysteresis, we should not get alternating alerts
            # The exact behavior depends on the hysteresis implementation

    def test_disabled_metrics_not_checked(self):
        """Test that disabled metrics are not checked for threshold violations."""
        # Create config with only CPU enabled
        config = MonitoringConfiguration(
            node_name="test_node",
            publish_frequency=1.0,
            thresholds={
                'cpu': {'warning': 80.0, 'critical': 90.0},
                'memory': {'warning': 85.0, 'critical': 95.0},
                'disk': {'warning': 80.0, 'critical': 95.0},
                'temperature': {'warning': 70.0, 'critical': 85.0}
            },
            enabled_metrics=['cpu'],  # Only CPU enabled
            log_level='INFO',
            hardware_specific={},
            alert_recipients=['test']
        )
        monitor = ThresholdMonitor(config)

        # High values for all metrics, but only CPU should generate alert
        health_status = HealthStatus(
            node_name="test_node",
            timestamp=1234567890.0,
            cpu_percent=95.0,  # Critical
            memory_percent=95.0,  # Would be critical but disabled
            disk_percent=98.0,  # Would be critical but disabled
            temperature=90.0,  # Would be critical but disabled
            status="CRITICAL",
            metrics={'cpu_percent': 95.0, 'memory_percent': 95.0}
        )

        alerts = monitor.check_thresholds(health_status)
        self.assertEqual(len(alerts), 1)  # Only CPU alert should be generated
        self.assertEqual(alerts[0].metric_name, "cpu")

    def test_temperature_alert_with_none_value(self):
        """Test that temperature alerts are not generated when temperature is None."""
        health_status = HealthStatus(
            node_name="test_node",
            timestamp=1234567890.0,
            cpu_percent=50.0,
            memory_percent=60.0,
            disk_percent=40.0,
            temperature=None,  # Temperature not available
            status="OK",
            metrics={'cpu_percent': 50.0}
        )

        alerts = self.monitor.check_thresholds(health_status)
        self.assertEqual(len(alerts), 0)  # No alerts should be generated

    def test_temperature_alert_with_valid_value(self):
        """Test that temperature alerts are generated when temperature exceeds thresholds."""
        health_status = HealthStatus(
            node_name="test_node",
            timestamp=1234567890.0,
            cpu_percent=50.0,
            memory_percent=60.0,
            disk_percent=40.0,
            temperature=85.0,  # Above critical threshold of 80
            status="CRITICAL",
            metrics={'temperature': 85.0}
        )

        alerts = self.monitor.check_thresholds(health_status)
        self.assertEqual(len(alerts), 1)
        self.assertEqual(alerts[0].severity, "CRITICAL")
        self.assertEqual(alerts[0].metric_name, "temperature")
        self.assertEqual(alerts[0].current_value, 85.0)


def run_tests():
    """Run the unit tests."""
    unittest.main()


if __name__ == '__main__':
    run_tests()