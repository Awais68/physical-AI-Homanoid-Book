"""SafeShutdownRequest model for Physical AI System Monitoring and Control."""

from dataclasses import dataclass
from typing import List
import time
import uuid


@dataclass
class SafeShutdownRequest:
    """Represents a request to execute safe shutdown procedures."""

    request_id: str             # Unique identifier for the shutdown request
    timestamp: float            # Unix timestamp of request
    trigger_reason: str         # Reason for shutdown (OVERTEMP, CRITICAL_ALERT, etc.)
    requested_by: str           # Node or service that requested shutdown
    shutdown_sequence: List[str] # Sequence of shutdown steps to execute
    timeout: float              # Maximum time allowed for shutdown (seconds)

    def __post_init__(self):
        """Validate the SafeShutdownRequest instance after initialization."""
        self._validate()

    def _validate(self):
        """Validate the shutdown request fields."""
        if not self.request_id or not isinstance(self.request_id, str):
            raise ValueError("request_id must be a non-empty string")

        if not isinstance(self.timestamp, (int, float)) or self.timestamp <= 0:
            raise ValueError("timestamp must be a positive number")

        if not self.trigger_reason or not isinstance(self.trigger_reason, str):
            raise ValueError("trigger_reason must be a non-empty string")

        if not self.requested_by or not isinstance(self.requested_by, str):
            raise ValueError("requested_by must be a non-empty string")

        if not isinstance(self.shutdown_sequence, list):
            raise ValueError("shutdown_sequence must be a list")

        if not isinstance(self.timeout, (int, float)) or self.timeout <= 0:
            raise ValueError("timeout must be a positive number")

    @classmethod
    def create_emergency_shutdown(
        cls,
        trigger_reason: str,
        requested_by: str,
        additional_steps: List[str] = None
    ) -> 'SafeShutdownRequest':
        """Create an emergency shutdown request."""
        default_sequence = [
            "save_critical_data",
            "stop_controllers",
            "disable_actuators",
            "power_down_safely"
        ]

        if additional_steps:
            default_sequence.extend(additional_steps)

        return cls(
            request_id=str(uuid.uuid4()),
            timestamp=time.time(),
            trigger_reason=trigger_reason,
            requested_by=requested_by,
            shutdown_sequence=default_sequence,
            timeout=30.0  # Default 30 second timeout
        )

    @classmethod
    def create_maintenance_shutdown(
        cls,
        requested_by: str,
        timeout: float = 60.0
    ) -> 'SafeShutdownRequest':
        """Create a maintenance shutdown request."""
        sequence = [
            "save_all_data",
            "complete_active_tasks",
            "stop_all_services",
            "power_down_system"
        ]

        return cls(
            request_id=str(uuid.uuid4()),
            timestamp=time.time(),
            trigger_reason="MAINTENANCE",
            requested_by=requested_by,
            shutdown_sequence=sequence,
            timeout=timeout
        )

    def is_overdue(self) -> bool:
        """Check if the shutdown request is overdue."""
        return (time.time() - self.timestamp) > self.timeout