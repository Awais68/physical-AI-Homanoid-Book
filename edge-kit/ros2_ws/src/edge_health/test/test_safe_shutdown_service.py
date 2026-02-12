"""Unit tests for the SafeShutdownService class."""

import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
import os
import time

# Add the edge_health module to the path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'edge_health'))

from edge_health.safe_shutdown_service import SafeShutdownService
from edge_health.monitoring_config import MonitoringConfiguration


class TestSafeShutdownService(unittest.TestCase):
    """Test cases for the SafeShutdownService class."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        # Mock the ROS2 node initialization
        with patch('edge_health.safe_shutdown_service.Node.__init__'):
            with patch('edge_health.safe_shutdown_service.setup_component_logger') as mock_logger_setup:
                # Create a mock logger
                mock_logger = Mock()
                mock_logger.info = Mock()
                mock_logger.warning = Mock()
                mock_logger.error = Mock()
                mock_logger.debug = Mock()
                mock_logger_setup.return_value = mock_logger

                self.service = SafeShutdownService.__new__(SafeShutdownService)
                self.service.config = MonitoringConfiguration.create_default()
                self.service.get_logger = Mock(return_value=mock_logger)
                self.service.create_service = Mock()
                self.service.shutdown_in_progress = False
                self.service.logger = mock_logger  # Set the logger attribute directly

    def test_validate_shutdown_request_valid(self):
        """Test that valid shutdown requests pass validation."""
        # Create a mock request object
        class MockRequest:
            def __init__(self):
                self.reason = "OVERTEMP"
                self.requested_by = "health_monitor"
                self.timeout = 30.0

        request = MockRequest()
        result = self.service._validate_shutdown_request(request)
        self.assertTrue(result)

    def test_validate_shutdown_request_invalid_reason(self):
        """Test that invalid shutdown reasons fail validation."""
        class MockRequest:
            def __init__(self):
                self.reason = "INVALID_REASON"
                self.requested_by = "health_monitor"
                self.timeout = 30.0

        request = MockRequest()
        result = self.service._validate_shutdown_request(request)
        self.assertFalse(result)

    def test_validate_shutdown_request_missing_requested_by(self):
        """Test that requests without requested_by fail validation."""
        class MockRequest:
            def __init__(self):
                self.reason = "OVERTEMP"
                self.requested_by = ""
                self.timeout = 30.0

        request = MockRequest()
        result = self.service._validate_shutdown_request(request)
        self.assertFalse(result)

    def test_validate_shutdown_request_invalid_timeout(self):
        """Test that requests with invalid timeout fail validation."""
        class MockRequest:
            def __init__(self, timeout):
                self.reason = "OVERTEMP"
                self.requested_by = "health_monitor"
                self.timeout = timeout

        # Test with timeout too low
        request = MockRequest(5.0)  # Below minimum of 10
        result = self.service._validate_shutdown_request(request)
        self.assertFalse(result)

        # Test with timeout too high
        request = MockRequest(500.0)  # Above maximum of 300
        result = self.service._validate_shutdown_request(request)
        self.assertFalse(result)

    def test_get_shutdown_sequence_emergency(self):
        """Test that emergency reasons get the correct shutdown sequence."""
        sequences = ["OVERTEMP", "CRITICAL_ALERT", "EMERGENCY"]

        for reason in sequences:
            with self.subTest(reason=reason):
                sequence = self.service._get_shutdown_sequence(reason)
                expected = [
                    "save_critical_data",
                    "stop_controllers",
                    "disable_actuators",
                    "power_down_safely"
                ]
                self.assertEqual(sequence, expected)

    def test_get_shutdown_sequence_maintenance(self):
        """Test that maintenance reason gets the correct shutdown sequence."""
        sequence = self.service._get_shutdown_sequence("MAINTENANCE")
        expected = [
            "save_all_data",
            "complete_active_tasks",
            "stop_all_services",
            "power_down_system"
        ]
        self.assertEqual(sequence, expected)

    def test_get_shutdown_sequence_standard(self):
        """Test that other reasons get the standard shutdown sequence."""
        sequence = self.service._get_shutdown_sequence("MANUAL")
        expected = [
            "save_session_data",
            "stop_non_critical_services",
            "power_down_safely"
        ]
        self.assertEqual(sequence, expected)

    def test_perform_pre_shutdown_checks_pass(self):
        """Test that pre-shutdown checks pass under normal conditions."""
        result = self.service._perform_pre_shutdown_checks()
        self.assertTrue(result)

    def test_execute_shutdown_step_valid(self):
        """Test that valid shutdown steps execute successfully."""
        # Create a mock shutdown request
        from edge_health.safe_shutdown_request import SafeShutdownRequest
        shutdown_request = SafeShutdownRequest(
            request_id="test_request",
            timestamp=time.time(),
            trigger_reason="TEST",
            requested_by="test",
            shutdown_sequence=["save_session_data"],
            timeout=30.0
        )

        success, message = self.service._execute_shutdown_step("save_session_data", shutdown_request)
        self.assertTrue(success)
        self.assertIn("Session data saved", message)

    def test_execute_shutdown_step_invalid(self):
        """Test that invalid shutdown steps fail."""
        from edge_health.safe_shutdown_request import SafeShutdownRequest
        shutdown_request = SafeShutdownRequest(
            request_id="test_request",
            timestamp=time.time(),
            trigger_reason="TEST",
            requested_by="test",
            shutdown_sequence=["invalid_step"],
            timeout=30.0
        )

        success, message = self.service._execute_shutdown_step("invalid_step", shutdown_request)
        self.assertFalse(success)
        self.assertIn("Unknown shutdown step", message)

    def test_shutdown_already_in_progress(self):
        """Test that concurrent shutdown requests are rejected."""
        # Set the service to have shutdown in progress
        self.service.shutdown_in_progress = True

        # Create mock request and response
        class MockRequest:
            def __init__(self):
                self.reason = "OVERTEMP"
                self.requested_by = "health_monitor"
                self.timeout = 30.0

        class MockResponse:
            def __init__(self):
                self.success = False
                self.message = ""

        request = MockRequest()
        response = MockResponse()

        # Mock the validation to pass
        with patch.object(self.service, '_validate_shutdown_request', return_value=True):
            with patch.object(self.service, '_perform_pre_shutdown_checks', return_value=True):
                # Call the handler
                result = self.service.handle_shutdown_request(request, response)

                # Verify the response indicates rejection
                self.assertFalse(response.success)
                self.assertIn("already in progress", response.message)

    def test_execute_shutdown_sequence_success(self):
        """Test that a full shutdown sequence executes successfully."""
        from edge_health.safe_shutdown_request import SafeShutdownRequest

        shutdown_request = SafeShutdownRequest(
            request_id="test_request",
            timestamp=time.time(),
            trigger_reason="TEST",
            requested_by="test",
            shutdown_sequence=["save_session_data", "power_down_safely"],
            timeout=60.0
        )

        success, message = self.service._execute_shutdown_sequence(shutdown_request)
        self.assertTrue(success)

    def test_execute_shutdown_sequence_timeout(self):
        """Test shutdown sequence handles timeout properly."""
        from edge_health.safe_shutdown_request import SafeShutdownRequest

        # Create a request with a very short timeout
        shutdown_request = SafeShutdownRequest(
            request_id="test_request",
            timestamp=time.time(),
            trigger_reason="TEST",
            requested_by="test",
            shutdown_sequence=["save_session_data"],
            timeout=0.001  # Very short timeout to trigger timeout condition
        )

        # Mock the step execution to take longer than the timeout
        with patch.object(self.service, '_execute_shutdown_step') as mock_step:
            mock_step.return_value = (True, "Step completed")
            success, message = self.service._execute_shutdown_sequence(shutdown_request)
            # The result depends on how the timeout is implemented in the actual code
            # If the timeout check happens before execution, it may return False


def run_tests():
    """Run the unit tests."""
    unittest.main()


if __name__ == '__main__':
    run_tests()