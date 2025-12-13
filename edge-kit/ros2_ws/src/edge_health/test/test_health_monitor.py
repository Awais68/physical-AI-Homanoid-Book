"""Unit tests for the HealthMonitor class."""

import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add the edge_health module to the path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'edge_health'))

from edge_health.health_monitor import HealthMonitor
from edge_health.monitoring_config import MonitoringConfiguration


class TestHealthMonitor(unittest.TestCase):
    """Test cases for the HealthMonitor class."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        # Mock the ROS2 node initialization
        with patch('edge_health.health_monitor.Node.__init__'):
            with patch('edge_health.health_monitor.setup_component_logger') as mock_logger_setup:
                # Create a mock logger
                mock_logger = Mock()
                mock_logger.info = Mock()
                mock_logger.warning = Mock()
                mock_logger.error = Mock()
                mock_logger.debug = Mock()
                mock_logger_setup.return_value = mock_logger

                self.monitor = HealthMonitor.__new__(HealthMonitor)
                self.monitor.config = MonitoringConfiguration.create_default()
                self.monitor.get_logger = Mock(return_value=mock_logger)
                self.monitor.create_publisher = Mock()
                self.monitor.create_timer = Mock()
                self.monitor.get_clock = Mock()
                self.monitor.publisher = Mock()
                self.monitor.logger = mock_logger  # Set the logger attribute directly

    def test_initialization(self):
        """Test that HealthMonitor initializes correctly."""
        # This test would require more complex mocking of ROS2 components
        # For now, we'll just verify the basic structure
        self.assertIsNotNone(self.monitor.config)
        self.assertIsInstance(self.monitor.config, MonitoringConfiguration)

    @patch('edge_health.health_monitor.get_all_metrics')
    def test_determine_health_status_ok(self, mock_get_metrics):
        """Test that health status is OK when all metrics are within thresholds."""
        mock_get_metrics.return_value = {
            'cpu_percent': 50.0,
            'memory_percent': 60.0,
            'disk_percent': 40.0,
            'temperature': 30.0
        }

        self.monitor.config = MonitoringConfiguration.create_default()
        result = self.monitor._determine_health_status(mock_get_metrics.return_value)
        self.assertEqual(result, "OK")

    @patch('edge_health.health_monitor.get_all_metrics')
    def test_determine_health_status_warning(self, mock_get_metrics):
        """Test that health status is WARNING when metrics exceed warning thresholds."""
        # enabled_metrics uses 'cpu', 'memory', 'disk', 'temperature' as keys
        mock_get_metrics.return_value = {
            'cpu': 85.0,  # Above warning threshold of 80
            'memory': 60.0,
            'disk': 40.0,
            'temperature': 30.0
        }

        self.monitor.config = MonitoringConfiguration.create_default()
        result = self.monitor._determine_health_status(mock_get_metrics.return_value)
        self.assertEqual(result, "WARNING")

    @patch('edge_health.health_monitor.get_all_metrics')
    def test_determine_health_status_critical(self, mock_get_metrics):
        """Test that health status is CRITICAL when metrics exceed critical thresholds."""
        # enabled_metrics uses 'cpu', 'memory', 'disk', 'temperature' as keys
        mock_get_metrics.return_value = {
            'cpu': 95.0,  # Above critical threshold of 90
            'memory': 60.0,
            'disk': 40.0,
            'temperature': 30.0
        }

        self.monitor.config = MonitoringConfiguration.create_default()
        result = self.monitor._determine_health_status(mock_get_metrics.return_value)
        self.assertEqual(result, "CRITICAL")

    def test_determine_health_status_invalid_metrics(self):
        """Test that invalid metrics (negative values) are handled properly."""
        metrics = {
            'cpu_percent': -1.0,  # Invalid value
            'memory_percent': -1.0,  # Invalid value
        }

        self.monitor.config = MonitoringConfiguration.create_default()
        result = self.monitor._determine_health_status(metrics)
        self.assertEqual(result, "OK")  # Should be OK since invalid metrics are skipped

    def test_create_health_status_msg(self):
        """Test that HealthStatus message is created correctly."""
        from edge_health.health_status import HealthStatus

        health_status = HealthStatus(
            node_name="test_node",
            timestamp=1234567890.0,
            cpu_percent=50.0,
            memory_percent=60.0,
            disk_percent=40.0,
            temperature=35.0,
            status="OK",
            metrics={'cpu_percent': 50.0}
        )

        # Mock the message creation
        with patch('edge_health.health_monitor.HealthStatusMsg') as mock_msg:
            mock_msg_instance = Mock()
            mock_msg.return_value = mock_msg_instance

            # Mock the clock
            mock_clock = Mock()
            mock_time_msg = Mock()
            mock_clock.to_msg.return_value = mock_time_msg
            self.monitor.get_clock = Mock(return_value=mock_clock)

            # Call the method
            result = self.monitor._create_health_status_msg(health_status)

            # Verify the message was configured correctly
            self.assertEqual(mock_msg_instance.node_name, "test_node")
            self.assertEqual(mock_msg_instance.cpu_percent, 50.0)


def run_tests():
    """Run the unit tests."""
    unittest.main()


if __name__ == '__main__':
    run_tests()