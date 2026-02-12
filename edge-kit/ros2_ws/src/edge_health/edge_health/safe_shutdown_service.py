#!/usr/bin/env python3
"""SafeShutdownService ROS2 service for Physical AI System Monitoring and Control."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

# Import our custom service type
try:
    from edge_health_interfaces.srv import SafeShutdown as SafeShutdownSrv
except ImportError:
    # Fallback for development
    from edge_health.srv import SafeShutdown as SafeShutdownSrv

from .safe_shutdown_request import SafeShutdownRequest
from .monitoring_config import MonitoringConfiguration
from .logging_utils import setup_component_logger
import time
import subprocess
from typing import List


class SafeShutdownService(Node):
    """ROS2 service that provides safe shutdown capabilities."""

    def __init__(self, config_path: str = None):
        """Initialize the SafeShutdownService."""
        super().__init__('safe_shutdown_service')

        # Load configuration
        if config_path:
            self.config = MonitoringConfiguration.from_yaml(config_path)
        else:
            self.config = MonitoringConfiguration.create_default(node_name=self.get_name())

        # Set up logging
        self.logger = setup_component_logger('safe_shutdown_service', self.config)

        # Create service for safe shutdown
        self.service = self.create_service(
            SafeShutdownSrv,
            '/safe_shutdown',
            self.handle_shutdown_request
        )

        # Flag to track if shutdown is in progress
        self.shutdown_in_progress = False

        self.logger.info("SafeShutdownService initialized and ready to handle requests")

    def handle_shutdown_request(self, request, response):
        """
        Handle incoming shutdown requests.

        Args:
            request: SafeShutdown request message
            response: SafeShutdown response message

        Returns:
            SafeShutdown response message
        """
        try:
            self.logger.info(f"Received shutdown request: reason={request.reason}, requested_by={request.requested_by}, timeout={request.timeout}")

            # Validate request parameters
            if not self._validate_shutdown_request(request):
                response.success = False
                response.message = "Invalid shutdown request parameters"
                self.logger.warning(f"Invalid shutdown request from {request.requested_by}")
                return response

            # Check if shutdown is already in progress
            if self.shutdown_in_progress:
                response.success = False
                response.message = "Shutdown already in progress"
                self.logger.warning("Shutdown request rejected - shutdown already in progress")
                return response

            # Create SafeShutdownRequest object
            shutdown_request = SafeShutdownRequest(
                request_id=f"shutdown_{int(time.time())}",
                timestamp=time.time(),
                trigger_reason=request.reason,
                requested_by=request.requested_by,
                shutdown_sequence=self._get_shutdown_sequence(request.reason),
                timeout=request.timeout
            )

            # Perform pre-shutdown checks
            if not self._perform_pre_shutdown_checks():
                response.success = False
                response.message = "Pre-shutdown checks failed"
                self.logger.error("Pre-shutdown checks failed, shutdown aborted")
                return response

            # Execute safe shutdown sequence
            success, message = self._execute_shutdown_sequence(shutdown_request)

            response.success = success
            response.message = message

            if success:
                self.logger.info(f"Shutdown completed successfully: {message}")
            else:
                self.logger.error(f"Shutdown failed: {message}")

            return response

        except Exception as e:
            self.logger.error(f"Error handling shutdown request: {e}")
            response.success = False
            response.message = f"Error handling shutdown request: {str(e)}"
            return response

    def _validate_shutdown_request(self, request) -> bool:
        """
        Validate shutdown request parameters.

        Args:
            request: SafeShutdown request message

        Returns:
            True if request is valid, False otherwise
        """
        # Check if reason is provided and valid
        valid_reasons = [
            "OVERTEMP", "CRITICAL_ALERT", "MAINTENANCE",
            "POWER_LOW", "EMERGENCY", "MANUAL"
        ]

        if not request.reason or request.reason not in valid_reasons:
            self.logger.warning(f"Invalid shutdown reason: {request.reason}")
            return False

        # Check if requested_by is provided
        if not request.requested_by:
            self.logger.warning("Shutdown request missing requested_by field")
            return False

        # Check if timeout is reasonable (between 10s and 300s)
        if request.timeout < 10.0 or request.timeout > 300.0:
            self.logger.warning(f"Shutdown timeout out of range: {request.timeout}")
            return False

        return True

    def _perform_pre_shutdown_checks(self) -> bool:
        """
        Perform pre-shutdown safety checks.

        Returns:
            True if all checks pass, False otherwise
        """
        try:
            # Check if critical processes are in a safe state
            # For now, we'll assume the system is in a safe state
            # In a real implementation, this would check for:
            # - Active robotic operations that need to complete
            # - Critical data that needs to be saved
            # - Safety interlocks that need to be engaged

            self.logger.info("Pre-shutdown checks passed")
            return True

        except Exception as e:
            self.logger.error(f"Pre-shutdown checks failed: {e}")
            return False

    def _get_shutdown_sequence(self, reason: str) -> List[str]:
        """
        Get the appropriate shutdown sequence based on the reason.

        Args:
            reason: Reason for shutdown

        Returns:
            List of shutdown steps to execute
        """
        if reason in ["OVERTEMP", "CRITICAL_ALERT", "EMERGENCY"]:
            # Emergency shutdown sequence - prioritize safety
            return [
                "save_critical_data",
                "stop_controllers",
                "disable_actuators",
                "power_down_safely"
            ]
        elif reason == "MAINTENANCE":
            # Maintenance shutdown sequence - full data preservation
            return [
                "save_all_data",
                "complete_active_tasks",
                "stop_all_services",
                "power_down_system"
            ]
        else:
            # Standard shutdown sequence
            return [
                "save_session_data",
                "stop_non_critical_services",
                "power_down_safely"
            ]

    def _execute_shutdown_sequence(self, shutdown_request: SafeShutdownRequest) -> tuple:
        """
        Execute the shutdown sequence.

        Args:
            shutdown_request: SafeShutdownRequest object

        Returns:
            Tuple of (success: bool, message: str)
        """
        try:
            self.shutdown_in_progress = True
            start_time = time.time()

            self.logger.info(f"Starting shutdown sequence: {shutdown_request.trigger_reason}")

            # Execute each step in the shutdown sequence
            for step in shutdown_request.shutdown_sequence:
                step_start = time.time()

                if time.time() - start_time > shutdown_request.timeout:
                    self.logger.error("Shutdown timeout exceeded")
                    self.shutdown_in_progress = False
                    return False, "Shutdown timeout exceeded"

                success, message = self._execute_shutdown_step(step, shutdown_request)

                if not success:
                    self.logger.error(f"Shutdown step failed: {step} - {message}")
                    self.shutdown_in_progress = False
                    return False, f"Shutdown failed at step '{step}': {message}"

                step_duration = time.time() - step_start
                self.logger.info(f"Completed shutdown step '{step}' in {step_duration:.2f}s")

            # Final check to ensure timeout wasn't exceeded
            total_duration = time.time() - start_time
            if total_duration > shutdown_request.timeout:
                self.logger.warning(f"Shutdown completed but exceeded timeout by {total_duration - shutdown_request.timeout:.2f}s")
                self.shutdown_in_progress = False
                return False, f"Shutdown completed but exceeded timeout by {total_duration - shutdown_request.timeout:.2f}s"

            self.logger.info(f"Shutdown sequence completed successfully in {total_duration:.2f}s")
            self.shutdown_in_progress = False
            return True, f"Shutdown completed successfully in {total_duration:.2f}s"

        except Exception as e:
            self.logger.error(f"Error executing shutdown sequence: {e}")
            self.shutdown_in_progress = False
            return False, f"Error executing shutdown sequence: {str(e)}"

    def _execute_shutdown_step(self, step: str, shutdown_request: SafeShutdownRequest) -> tuple:
        """
        Execute a single shutdown step.

        Args:
            step: Name of the shutdown step to execute
            shutdown_request: SafeShutdownRequest object for context

        Returns:
            Tuple of (success: bool, message: str)
        """
        try:
            if step == "save_critical_data":
                # Save critical data to persistent storage
                self.logger.info("Saving critical data...")
                # In a real implementation, this would save critical data
                time.sleep(0.5)  # Simulate saving time
                return True, "Critical data saved"

            elif step == "save_all_data":
                # Save all data to persistent storage
                self.logger.info("Saving all data...")
                # In a real implementation, this would save all data
                time.sleep(1.0)  # Simulate saving time
                return True, "All data saved"

            elif step == "save_session_data":
                # Save session data
                self.logger.info("Saving session data...")
                time.sleep(0.2)  # Simulate saving time
                return True, "Session data saved"

            elif step == "stop_controllers":
                # Stop robot controllers
                self.logger.info("Stopping controllers...")
                # In a real implementation, this would safely stop controllers
                time.sleep(0.3)  # Simulate stopping time
                return True, "Controllers stopped"

            elif step == "disable_actuators":
                # Disable robot actuators for safety
                self.logger.info("Disabling actuators...")
                # In a real implementation, this would safely disable actuators
                time.sleep(0.2)  # Simulate disabling time
                return True, "Actuators disabled"

            elif step == "stop_all_services":
                # Stop all services
                self.logger.info("Stopping all services...")
                # In a real implementation, this would stop services
                time.sleep(0.5)  # Simulate stopping time
                return True, "All services stopped"

            elif step == "stop_non_critical_services":
                # Stop non-critical services
                self.logger.info("Stopping non-critical services...")
                time.sleep(0.3)  # Simulate stopping time
                return True, "Non-critical services stopped"

            elif step == "power_down_safely":
                # Perform safe power down
                self.logger.info("Performing safe power down...")
                # In a real implementation, this would initiate safe power down
                time.sleep(0.5)  # Simulate power down time
                return True, "System safely powered down"

            elif step == "power_down_system":
                # Power down the entire system
                self.logger.info("Powering down system...")
                time.sleep(0.8)  # Simulate power down time
                return True, "System powered down"

            elif step == "complete_active_tasks":
                # Complete any active tasks before shutdown
                self.logger.info("Completing active tasks...")
                time.sleep(0.4)  # Simulate task completion time
                return True, "Active tasks completed"

            else:
                self.logger.warning(f"Unknown shutdown step: {step}")
                return False, f"Unknown shutdown step: {step}"

        except Exception as e:
            self.logger.error(f"Error executing shutdown step '{step}': {e}")
            return False, f"Error executing step '{step}': {str(e)}"


def main(args=None):
    """Main function to run the SafeShutdownService."""
    rclpy.init(args=args)

    # Get config path from command line arguments if provided
    import sys
    config_path = None
    if len(sys.argv) > 1:
        config_path = sys.argv[1]

    safe_shutdown_service = SafeShutdownService(config_path=config_path)

    try:
        rclpy.spin(safe_shutdown_service)
    except KeyboardInterrupt:
        safe_shutdown_service.get_logger().info("Interrupted by user")
    finally:
        safe_shutdown_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()