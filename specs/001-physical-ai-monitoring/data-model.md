# Data Model: Physical AI System Monitoring and Control

## Overview
This document defines the key data structures, message formats, and data relationships for the Physical AI System Monitoring and Control feature.

## Core Entities

### HealthStatus
Represents the current health metrics of a Physical AI system component.

```python
class HealthStatus:
    node_name: str              # Name of the monitoring node
    timestamp: float            # Unix timestamp of measurement
    cpu_percent: float          # CPU usage percentage (0-100)
    memory_percent: float       # Memory usage percentage (0-100)
    disk_percent: float         # Disk usage percentage (0-100)
    temperature: Optional[float] # Temperature in Celsius (if available)
    status: str                 # Overall health status (OK, WARNING, CRITICAL)
    metrics: Dict[str, Any]     # Additional metrics specific to hardware
```

**Validation Rules:**
- cpu_percent must be between 0 and 100
- memory_percent must be between 0 and 100
- disk_percent must be between 0 and 100
- timestamp must be current or recent (within 5 seconds)

### Alert
Represents a threshold violation or system alert.

```python
class Alert:
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
```

**Validation Rules:**
- severity must be one of: INFO, WARNING, ERROR, CRITICAL
- timestamp must be current
- alert_id must be unique within the system

### MonitoringConfiguration
Contains threshold values and monitoring parameters.

```python
class MonitoringConfiguration:
    node_name: str                          # Name of this monitoring node
    publish_frequency: float               # Frequency of health status publishing (seconds)
    thresholds: Dict[str, Dict[str, float]] # Threshold values by metric and level
    enabled_metrics: List[str]             # List of metrics to monitor
    log_level: str                         # Logging level (DEBUG, INFO, WARNING, ERROR)
    hardware_specific: Dict[str, Any]      # Hardware-specific configuration
    alert_recipients: List[str]            # List of alert recipient identifiers
```

**Validation Rules:**
- publish_frequency must be greater than 0
- thresholds must have valid metric names and threshold values
- log_level must be one of: DEBUG, INFO, WARNING, ERROR

### SafeShutdownRequest
Represents a request to execute safe shutdown procedures.

```python
class SafeShutdownRequest:
    request_id: str             # Unique identifier for the shutdown request
    timestamp: float            # Unix timestamp of request
    trigger_reason: str         # Reason for shutdown (OVERTEMP, CRITICAL_ALERT, etc.)
    requested_by: str           # Node or service that requested shutdown
    shutdown_sequence: List[str] # Sequence of shutdown steps to execute
    timeout: float              # Maximum time allowed for shutdown (seconds)
```

**Validation Rules:**
- request_id must be unique
- timestamp must be current
- trigger_reason must be a valid shutdown reason

## ROS2 Message Definitions

### HealthStatus.msg
```
string node_name
float64 timestamp
float64 cpu_percent
float64 memory_percent
float64 disk_percent
float64 temperature
string status
builtin_interfaces/Time header
```

### Alert.msg
```
string alert_id
float64 timestamp
string severity
string component
string metric_name
float64 current_value
float64 threshold_value
string message
string node_name
string alert_type
builtin_interfaces/Time header
```

### SafeShutdown.srv
```
# Request
string reason
string requested_by
float64 timeout

# Response
bool success
string message
```

## Configuration Schema

### edge_health_config.yaml
```yaml
# Node configuration
node_name: "health_monitor"
publish_frequency: 1.0  # seconds between health status publications

# Thresholds configuration
thresholds:
  cpu:
    warning: 80.0      # Percentage
    critical: 90.0     # Percentage
  memory:
    warning: 85.0      # Percentage
    critical: 95.0     # Percentage
  disk:
    warning: 80.0      # Percentage
    critical: 95.0     # Percentage
  temperature:
    warning: 70.0      # Celsius
    critical: 85.0     # Celsius

# Metrics to monitor
enabled_metrics:
  - cpu
  - memory
  - disk
  - temperature

# Logging configuration
log_level: "INFO"

# Hardware-specific settings
hardware_specific:
  jetson:
    enable_gpu_monitoring: true
    enable_power_monitoring: false

# Alert configuration
alert_recipients:
  - "operator_console"
  - "dashboard_service"
```

## State Transitions

### Health Status States
- **HEALTHY**: All metrics within normal ranges
- **WARNING**: One or more metrics in warning range
- **CRITICAL**: One or more metrics in critical range
- **UNAVAILABLE**: Monitoring node is not responding

### Alert States
- **ACTIVE**: Alert has been generated and not yet resolved
- **RESOLVED**: Alert condition has been resolved
- **ACKNOWLEDGED**: Alert has been acknowledged by operator

## Relationships

### HealthStatus and Alert
- One HealthStatus may trigger zero or more Alerts when thresholds are exceeded
- Alerts reference the HealthStatus that triggered them
- Historical HealthStatus data provides context for Alerts

### MonitoringConfiguration and HealthStatus
- MonitoringConfiguration determines which metrics are collected for HealthStatus
- Threshold values in MonitoringConfiguration determine Alert generation from HealthStatus

### SafeShutdownRequest and Alert
- Critical Alerts may trigger SafeShutdownRequests
- SafeShutdownRequests reference the Alert that triggered them

## Data Flow

1. **Collection**: HealthStatus objects are created by HealthMonitor at configured intervals
2. **Evaluation**: ThresholdMonitor evaluates HealthStatus against MonitoringConfiguration
3. **Alerting**: When thresholds are exceeded, Alert objects are created and published
4. **Response**: Critical Alerts may trigger SafeShutdownRequest objects
5. **Storage**: All data is logged locally for dashboard consumption