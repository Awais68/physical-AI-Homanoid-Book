"""ThresholdMonitor class for Physical AI System Monitoring and Control."""

from typing import Dict, Optional, Tuple
import time
from .health_status import HealthStatus
from .alert import Alert
from .monitoring_config import MonitoringConfiguration


class ThresholdMonitor:
    """Monitors health metrics against configurable thresholds and generates alerts."""

    def __init__(self, config: MonitoringConfiguration):
        """
        Initialize the ThresholdMonitor.

        Args:
            config: MonitoringConfiguration instance with threshold values
        """
        self.config = config
        # Track previous alert states to implement hysteresis
        self._previous_alert_states: Dict[str, Tuple[float, str]] = {}
        # Track recovery states to prevent immediate re-alerting
        self._recovery_timestamps: Dict[str, float] = {}

    def check_thresholds(self, health_status: HealthStatus) -> list:
        """
        Check health metrics against thresholds and return any alerts.

        Args:
            health_status: HealthStatus instance to check against thresholds

        Returns:
            List of Alert instances for any threshold violations
        """
        alerts = []

        # Check each enabled metric against its thresholds
        for metric_name in self.config.enabled_metrics:
            if not hasattr(health_status, metric_name + '_percent') and metric_name != 'temperature':
                continue

            # Get the metric value from health status
            if metric_name == 'temperature':
                value = health_status.temperature
                if value is None or value < 0:
                    continue  # Skip if temperature not available
            elif hasattr(health_status, metric_name + '_percent'):
                value = getattr(health_status, metric_name + '_percent')
                if value < 0:  # Skip if metric not available
                    continue
            else:
                continue

            # Get thresholds for this metric
            try:
                warning_threshold = self.config.get_threshold(metric_name, 'warning')
                critical_threshold = self.config.get_threshold(metric_name, 'critical')
            except ValueError:
                # If thresholds don't exist for this metric, skip checking
                continue

            # Check for threshold violations
            alert = self._check_single_metric(
                metric_name, value, warning_threshold, critical_threshold, health_status
            )

            if alert:
                alerts.append(alert)

        return alerts

    def _check_single_metric(
        self, metric_name: str, value: float, warning_threshold: float, critical_threshold: float, health_status: HealthStatus
    ) -> Optional[Alert]:
        """
        Check a single metric against its thresholds.

        Args:
            metric_name: Name of the metric to check
            value: Current value of the metric
            warning_threshold: Warning threshold value
            critical_threshold: Critical threshold value
            health_status: HealthStatus instance for context

        Returns:
            Alert instance if threshold is violated, None otherwise
        """
        # Check if we need hysteresis (to prevent alert flapping)
        if metric_name in self._previous_alert_states:
            prev_value, prev_state = self._previous_alert_states[metric_name]

            # If the metric value has improved significantly, allow recovery
            if prev_state == "CRITICAL" and value < (critical_threshold * 0.9):
                # Allow recovery from critical state
                pass
            elif prev_state == "WARNING" and value < (warning_threshold * 0.9):
                # Allow recovery from warning state
                pass
            elif abs(value - prev_value) < (warning_threshold * 0.05):  # 5% hysteresis
                # Don't trigger new alert if value hasn't changed significantly
                return None

        # Determine alert severity based on thresholds
        if value >= critical_threshold:
            severity = "CRITICAL"
            threshold_type = "critical"
        elif value >= warning_threshold:
            severity = "WARNING"
            threshold_type = "warning"
        else:
            # Metric is within normal range
            # Check if we need to send a recovery alert
            recovery_alert = self._check_recovery(metric_name, value, warning_threshold, health_status)
            if recovery_alert:
                # Update previous state to reflect recovery
                self._previous_alert_states[metric_name] = (value, "OK")
                return recovery_alert
            else:
                # Update previous state to current normal state
                self._previous_alert_states[metric_name] = (value, "OK")
                return None

        # Create alert for threshold violation
        alert = Alert.create_threshold_alert(
            component=metric_name,
            metric_name=metric_name,
            current_value=value,
            threshold_value=critical_threshold if severity == "CRITICAL" else warning_threshold,
            severity=severity,
            node_name=health_status.node_name,
            message=f"{metric_name.upper()} threshold exceeded: {value:.2f} >= {critical_threshold if severity == 'CRITICAL' else warning_threshold:.2f}"
        )

        # Update previous state
        self._previous_alert_states[metric_name] = (value, severity)

        return alert

    def _check_recovery(
        self, metric_name: str, current_value: float, warning_threshold: float, health_status: HealthStatus
    ) -> Optional[Alert]:
        """
        Check if a metric has recovered from a threshold violation.

        Args:
            metric_name: Name of the metric to check
            current_value: Current value of the metric
            warning_threshold: Warning threshold value
            health_status: HealthStatus instance for context

        Returns:
            Recovery Alert instance if metric has recovered, None otherwise
        """
        # Check if this metric was previously in an alert state
        if metric_name in self._previous_alert_states:
            prev_value, prev_state = self._previous_alert_states[metric_name]

            # Only generate recovery alert if we were previously in an alert state
            if prev_state in ["WARNING", "CRITICAL"] and current_value <= warning_threshold * 0.9:
                # Check if enough time has passed since the last alert to avoid flapping
                current_time = time.time()
                recovery_timeout = 5.0  # 5 seconds minimum between recovery and re-alert

                if metric_name not in self._recovery_timestamps or \
                   (current_time - self._recovery_timestamps[metric_name]) > recovery_timeout:
                    # Create recovery alert
                    recovery_alert = Alert.create_recovery_alert(
                        component=metric_name,
                        metric_name=metric_name,
                        current_value=current_value,
                        node_name=health_status.node_name,
                        message=f"{metric_name.upper()} recovered: {current_value:.2f} <= {warning_threshold * 0.9:.2f}"
                    )

                    # Update recovery timestamp
                    self._recovery_timestamps[metric_name] = current_time
                    return recovery_alert

        return None

    def reset_alert_state(self, metric_name: str):
        """
        Reset the alert state for a specific metric.

        Args:
            metric_name: Name of the metric to reset
        """
        if metric_name in self._previous_alert_states:
            del self._previous_alert_states[metric_name]
        if metric_name in self._recovery_timestamps:
            del self._recovery_timestamps[metric_name]

    def get_alert_state(self, metric_name: str) -> Optional[Tuple[float, str]]:
        """
        Get the current alert state for a specific metric.

        Args:
            metric_name: Name of the metric to check

        Returns:
            Tuple of (previous_value, previous_state) or None if not tracked
        """
        return self._previous_alert_states.get(metric_name, None)

    def clear_all_states(self):
        """Clear all stored alert states."""
        self._previous_alert_states.clear()
        self._recovery_timestamps.clear()