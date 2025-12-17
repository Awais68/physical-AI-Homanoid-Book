"""MonitoringConfiguration model for Physical AI System Monitoring and Control."""

from dataclasses import dataclass
from typing import Dict, List, Any
import yaml
from pathlib import Path


@dataclass
class MonitoringConfiguration:
    """Contains threshold values and monitoring parameters."""

    node_name: str                          # Name of this monitoring node
    publish_frequency: float               # Frequency of health status publishing (seconds)
    thresholds: Dict[str, Dict[str, float]] # Threshold values by metric and level
    enabled_metrics: List[str]             # List of metrics to monitor
    log_level: str                         # Logging level (DEBUG, INFO, WARNING, ERROR)
    hardware_specific: Dict[str, Any]      # Hardware-specific configuration
    alert_recipients: List[str]            # List of alert recipient identifiers

    def __post_init__(self):
        """Validate the MonitoringConfiguration instance after initialization."""
        self._validate()

    def _validate(self):
        """Validate the monitoring configuration fields."""
        if not self.node_name or not isinstance(self.node_name, str):
            raise ValueError("node_name must be a non-empty string")

        if not isinstance(self.publish_frequency, (int, float)) or self.publish_frequency <= 0:
            raise ValueError("publish_frequency must be a positive number")

        if not isinstance(self.thresholds, dict):
            raise ValueError("thresholds must be a dictionary")

        if not isinstance(self.enabled_metrics, list):
            raise ValueError("enabled_metrics must be a list")

        valid_log_levels = ["DEBUG", "INFO", "WARNING", "ERROR"]
        if self.log_level not in valid_log_levels:
            raise ValueError(f"log_level must be one of: {valid_log_levels}")

        if not isinstance(self.hardware_specific, dict):
            raise ValueError("hardware_specific must be a dictionary")

        if not isinstance(self.alert_recipients, list):
            raise ValueError("alert_recipients must be a list")

    @classmethod
    def from_yaml(cls, config_path: str) -> 'MonitoringConfiguration':
        """Load configuration from a YAML file."""
        with open(config_path, 'r') as file:
            data = yaml.safe_load(file)

        return cls(
            node_name=data.get('node_name', 'health_monitor'),
            publish_frequency=data.get('publish_frequency', 1.0),
            thresholds=data.get('thresholds', {
                'cpu': {'warning': 80.0, 'critical': 90.0},
                'memory': {'warning': 85.0, 'critical': 95.0},
                'disk': {'warning': 80.0, 'critical': 95.0},
                'temperature': {'warning': 70.0, 'critical': 85.0}
            }),
            enabled_metrics=data.get('enabled_metrics', ['cpu', 'memory', 'disk', 'temperature']),
            log_level=data.get('log_level', 'INFO'),
            hardware_specific=data.get('hardware_specific', {}),
            alert_recipients=data.get('alert_recipients', ['operator_console', 'dashboard_service'])
        )

    @classmethod
    def create_default(cls, node_name: str = "health_monitor") -> 'MonitoringConfiguration':
        """Create a default configuration."""
        return cls(
            node_name=node_name,
            publish_frequency=1.0,
            thresholds={
                'cpu': {'warning': 80.0, 'critical': 90.0},
                'memory': {'warning': 85.0, 'critical': 95.0},
                'disk': {'warning': 80.0, 'critical': 95.0},
                'temperature': {'warning': 70.0, 'critical': 85.0}
            },
            enabled_metrics=['cpu', 'memory', 'disk', 'temperature'],
            log_level='INFO',
            hardware_specific={},
            alert_recipients=['operator_console', 'dashboard_service']
        )

    def is_metric_enabled(self, metric_name: str) -> bool:
        """Check if a specific metric is enabled for monitoring."""
        return metric_name in self.enabled_metrics

    def get_threshold(self, metric: str, level: str) -> float:
        """Get the threshold value for a specific metric and level."""
        if metric in self.thresholds and level in self.thresholds[metric]:
            return self.thresholds[metric][level]
        raise ValueError(f"Threshold not found for metric '{metric}' and level '{level}'")