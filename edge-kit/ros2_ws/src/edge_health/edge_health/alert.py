"""Alert model for Physical AI System Monitoring and Control."""

from dataclasses import dataclass
from typing import Dict, Any
import time
import uuid


@dataclass
class Alert:
    """Represents a threshold violation or system alert."""

    alert_id: str               # Unique identifier for the alert
    timestamp: float            # Unix timestamp of alert generation
    severity: str               # Alert severity (INFO, WARNING, ERROR, CRITICAL)
    component: str              # Affected component name
    metric_name: str            # Name of the metric that triggered the alert
    current_value: float        # Current value that triggered the alert
    threshold_value: float      # Threshold that was exceeded
    message: str                # Human-readable alert message
    node_name: str              # Name of the node that generated the alert
    alert_type: str             # Type of alert (THRESHOLD_EXCEEDED, RECOVERY, etc.)

    def __post_init__(self):
        """Validate the Alert instance after initialization."""
        self._validate()

    def _validate(self):
        """Validate the alert fields."""
        if not self.alert_id or not isinstance(self.alert_id, str):
            raise ValueError("alert_id must be a non-empty string")

        if not isinstance(self.timestamp, (int, float)) or self.timestamp <= 0:
            raise ValueError("timestamp must be a positive number")

        valid_severities = ["INFO", "WARNING", "ERROR", "CRITICAL"]
        if self.severity not in valid_severities:
            raise ValueError(f"severity must be one of: {valid_severities}")

        if not self.component or not isinstance(self.component, str):
            raise ValueError("component must be a non-empty string")

        if not self.metric_name or not isinstance(self.metric_name, str):
            raise ValueError("metric_name must be a non-empty string")

        if not isinstance(self.current_value, (int, float)):
            raise ValueError("current_value must be a number")

        if not isinstance(self.threshold_value, (int, float)):
            raise ValueError("threshold_value must be a number")

        if not self.message or not isinstance(self.message, str):
            raise ValueError("message must be a non-empty string")

        if not self.node_name or not isinstance(self.node_name, str):
            raise ValueError("node_name must be a non-empty string")

        if not self.alert_type or not isinstance(self.alert_type, str):
            raise ValueError("alert_type must be a non-empty string")

    @classmethod
    def create_threshold_alert(
        cls,
        component: str,
        metric_name: str,
        current_value: float,
        threshold_value: float,
        severity: str,
        node_name: str,
        message: str = ""
    ) -> 'Alert':
        """Create a threshold-based alert."""
        if not message:
            message = f"{metric_name.upper()} threshold exceeded: {current_value} > {threshold_value}"

        return cls(
            alert_id=str(uuid.uuid4()),
            timestamp=time.time(),
            severity=severity,
            component=component,
            metric_name=metric_name,
            current_value=current_value,
            threshold_value=threshold_value,
            message=message,
            node_name=node_name,
            alert_type="THRESHOLD_EXCEEDED"
        )

    @classmethod
    def create_recovery_alert(
        cls,
        component: str,
        metric_name: str,
        current_value: float,
        node_name: str,
        message: str = ""
    ) -> 'Alert':
        """Create a recovery alert when metrics return to normal."""
        if not message:
            message = f"{metric_name.upper()} recovered: {current_value} <= threshold"

        return cls(
            alert_id=str(uuid.uuid4()),
            timestamp=time.time(),
            severity="INFO",
            component=component,
            metric_name=metric_name,
            current_value=current_value,
            threshold_value=0.0,  # Not applicable for recovery
            message=message,
            node_name=node_name,
            alert_type="RECOVERY"
        )

    def is_critical(self) -> bool:
        """Check if the alert is critical."""
        return self.severity == "CRITICAL"

    def is_warning(self) -> bool:
        """Check if the alert is a warning."""
        return self.severity == "WARNING"

    def is_error(self) -> bool:
        """Check if the alert is an error."""
        return self.severity == "ERROR"