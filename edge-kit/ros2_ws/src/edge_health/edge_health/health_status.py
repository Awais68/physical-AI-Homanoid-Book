"""HealthStatus model for Physical AI System Monitoring and Control."""

from dataclasses import dataclass
from typing import Optional, Dict, Any
import time


@dataclass
class HealthStatus:
    """Represents the current health metrics of a Physical AI system component."""

    node_name: str              # Name of the monitoring node
    timestamp: float            # Unix timestamp of measurement
    cpu_percent: float          # CPU usage percentage (0-100)
    memory_percent: float       # Memory usage percentage (0-100)
    disk_percent: float         # Disk usage percentage (0-100)
    temperature: Optional[float] # Temperature in Celsius (if available)
    status: str                 # Overall health status (OK, WARNING, CRITICAL)
    metrics: Dict[str, Any]     # Additional metrics specific to hardware

    def __post_init__(self):
        """Validate the HealthStatus instance after initialization."""
        self._validate()

    def _validate(self):
        """Validate the health status fields."""
        if not self.node_name or not isinstance(self.node_name, str):
            raise ValueError("node_name must be a non-empty string")

        if not isinstance(self.timestamp, (int, float)) or self.timestamp <= 0:
            raise ValueError("timestamp must be a positive number")

        if not (0 <= self.cpu_percent <= 100):
            raise ValueError("cpu_percent must be between 0 and 100")

        if not (0 <= self.memory_percent <= 100):
            raise ValueError("memory_percent must be between 0 and 100")

        if not (0 <= self.disk_percent <= 100):
            raise ValueError("disk_percent must be between 0 and 100")

        if self.temperature is not None and self.temperature < -273.15:
            raise ValueError("temperature must be above absolute zero (-273.15°C)")

        if self.status not in ["OK", "WARNING", "CRITICAL"]:
            raise ValueError("status must be one of: OK, WARNING, CRITICAL")

        if not isinstance(self.metrics, dict):
            raise ValueError("metrics must be a dictionary")

    @classmethod
    def create_default(cls, node_name: str) -> 'HealthStatus':
        """Create a default HealthStatus instance with current timestamp."""
        return cls(
            node_name=node_name,
            timestamp=time.time(),
            cpu_percent=0.0,
            memory_percent=0.0,
            disk_percent=0.0,
            temperature=None,
            status="OK",
            metrics={}
        )

    def is_healthy(self) -> bool:
        """Check if the system is in a healthy state."""
        return self.status == "OK"

    def is_warning(self) -> bool:
        """Check if the system is in a warning state."""
        return self.status == "WARNING"

    def is_critical(self) -> bool:
        """Check if the system is in a critical state."""
        return self.status == "CRITICAL"