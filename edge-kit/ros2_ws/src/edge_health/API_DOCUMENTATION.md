# ROS2 API Documentation for Edge Health Package

## Overview
This document provides detailed API documentation for all ROS2 interfaces in the edge_health package, including topics, services, and messages.

## Message Types

### HealthStatus.msg
Publishes real-time health metrics for Physical AI hardware components.

#### Fields
| Field | Type | Description |
|-------|------|-------------|
| node_name | string | Name of the monitoring node |
| timestamp | float64 | Unix timestamp of measurement |
| cpu_percent | float64 | CPU usage percentage (0-100) |
| memory_percent | float64 | Memory usage percentage (0-100) |
| disk_percent | float64 | Disk usage percentage (0-100) |
| temperature | float64 | Temperature in Celsius (-1.0 if unavailable) |
| status | string | Overall health status ("OK", "WARNING", "CRITICAL") |
| header | builtin_interfaces/Time | Standard ROS2 header |

#### Example Message
```
node_name: "jetson_nano_01"
timestamp: 1702285200.123
cpu_percent: 45.2
memory_percent: 62.8
disk_percent: 23.1
temperature: 42.5
status: "OK"
header:
  stamp:
    sec: 1702285200
    nanosec: 123000000
  frame_id: ""
```

### Alert.msg
Represents a threshold violation or system alert.

#### Fields
| Field | Type | Description |
|-------|------|-------------|
| alert_id | string | Unique identifier for the alert |
| timestamp | float64 | Unix timestamp of alert generation |
| severity | string | Alert severity ("INFO", "WARNING", "ERROR", "CRITICAL") |
| component | string | Affected component name |
| metric_name | string | Name of the metric that triggered the alert |
| current_value | float64 | Current value that triggered the alert |
| threshold_value | float64 | Threshold that was exceeded |
| message | string | Human-readable alert message |
| node_name | string | Name of the node that generated the alert |
| alert_type | string | Type of alert ("THRESHOLD_EXCEEDED", "RECOVERY", etc.) |
| header | builtin_interfaces/Time | Standard ROS2 header |

#### Example Message
```
alert_id: "alert_12345"
timestamp: 1702285300.456
severity: "WARNING"
component: "cpu"
metric_name: "cpu"
current_value: 85.5
threshold_value: 80.0
message: "CPU usage threshold exceeded: 85.5 > 80.0"
node_name: "jetson_nano_01"
alert_type: "THRESHOLD_EXCEEDED"
header:
  stamp:
    sec: 1702285300
    nanosec: 456000000
  frame_id: ""
```

## Service Types

### SafeShutdown.srv
Provides a service interface for initiating safe shutdown procedures.

#### Request
| Field | Type | Description |
|-------|------|-------------|
| reason | string | Reason for shutdown ("OVERTEMP", "CRITICAL_ALERT", "MAINTENANCE", etc.) |
| requested_by | string | Node or service that requested shutdown |
| timeout | float64 | Maximum time allowed for shutdown (seconds) |

#### Response
| Field | Type | Description |
|-------|------|-------------|
| success | bool | True if shutdown was initiated successfully |
| message | string | Human-readable status message |

#### Example Request
```
reason: "OVERTEMP"
requested_by: "thermal_protection"
timeout: 30.0
```

#### Example Response
```
success: true
message: "Shutdown initiated successfully in 0.23s"
```

## Topics

### /health_status
- **Type**: `edge_health_interfaces/msg/HealthStatus` (or `edge_health/msg/HealthStatus`)
- **QoS**: Transient Local, Depth 10
- **Description**: Publishes health metrics at regular intervals
- **Default Rate**: 1 Hz (configurable)
- **Publishers**: HealthMonitor nodes
- **Subscribers**: AlertPublisher, Dashboard systems, Logging systems

### /alerts
- **Type**: `edge_health_interfaces/msg/Alert` (or `edge_health/msg/Alert`)
- **QoS**: Transient Local, Depth 10
- **Description**: Publishes alerts when thresholds are exceeded
- **Default Rate**: Event-driven (when thresholds are exceeded)
- **Publishers**: AlertPublisher nodes
- **Subscribers**: Dashboard systems, Logging systems, Safe shutdown triggers

## Services

### /safe_shutdown
- **Type**: `edge_health_interfaces/srv/SafeShutdown`
- **Description**: Service for initiating safe shutdown procedures
- **Node**: SafeShutdownService
- **Request**: Initiate safe shutdown with reason and timeout
- **Response**: Confirmation of shutdown initiation

## Nodes

### HealthMonitor
- **Node Name**: health_monitor
- **Namespace**: (configurable)
- **Description**: Monitors system health metrics and publishes to /health_status
- **Publishers**:
  - `/health_status` (HealthStatus)
- **Timers**:
  - Health collection timer (configurable frequency)
- **Parameters**:
  - `publish_frequency`: How often to publish health status (default: 1.0s)
  - `node_name`: Name of this monitoring node (default: "health_monitor")

### AlertPublisher
- **Node Name**: alert_publisher
- **Namespace**: (configurable)
- **Description**: Subscribes to health status and publishes alerts when thresholds are exceeded
- **Subscribers**:
  - `/health_status` (HealthStatus)
- **Publishers**:
  - `/alerts` (Alert)
- **Parameters**:
  - `node_name`: Name of this alert publisher (default: "alert_publisher")

### SafeShutdownService
- **Node Name**: safe_shutdown_service
- **Namespace**: (configurable)
- **Description**: Provides safe shutdown service interface
- **Services**:
  - `/safe_shutdown` (SafeShutdown)
- **Parameters**:
  - `node_name`: Name of this shutdown service (default: "safe_shutdown_service")

## Parameters

### Common Parameters
- `node_name`: String identifier for the node instance
- `publish_frequency`: Float value for publishing frequency in seconds
- `config_file`: Path to YAML configuration file

### Configuration File Structure
The nodes support configuration via YAML files with the following structure:

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

## Usage Examples

### Subscribing to Health Status
```python
import rclpy
from rclpy.node import Node
from edge_health_interfaces.msg import HealthStatus

class HealthSubscriber(Node):
    def __init__(self):
        super().__init__('health_subscriber')
        self.subscription = self.create_subscription(
            HealthStatus,
            '/health_status',
            self.health_callback,
            10)
        self.subscription  # prevent unused variable warning

    def health_callback(self, msg):
        self.get_logger().info(f'Health: {msg.status} - CPU: {msg.cpu_percent}%')

def main(args=None):
    rclpy.init(args=args)
    health_subscriber = HealthSubscriber()
    rclpy.spin(health_subscriber)
    health_subscriber.destroy_node()
    rclpy.shutdown()
```

### Calling Safe Shutdown Service
```python
import rclpy
from rclpy.node import Node
from edge_health_interfaces.srv import SafeShutdown

class ShutdownClient(Node):
    def __init__(self):
        super().__init__('shutdown_client')
        self.cli = self.create_client(SafeShutdown, '/safe_shutdown')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self, reason, requested_by, timeout):
        request = SafeShutdown.Request()
        request.reason = reason
        request.requested_by = requested_by
        request.timeout = timeout
        future = self.cli.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)
    shutdown_client = ShutdownClient()

    future = shutdown_client.send_request("OVERTEMP", "thermal_monitor", 30.0)
    rclpy.spin_until_future_complete(shutdown_client, future)

    response = future.result()
    shutdown_client.get_logger().info(f'Shutdown response: {response.success}, {response.message}')

    shutdown_client.destroy_node()
    rclpy.shutdown()
```

## Error Handling

### Common Error Conditions
- **Invalid configuration**: Nodes will log errors and use default values
- **Sensor failures**: Nodes will publish -1.0 for unavailable metrics
- **Service timeouts**: Safe shutdown requests will fail with timeout error
- **Communication errors**: Appropriate error messages will be logged

### Status Values
- **OK**: All metrics within normal ranges
- **WARNING**: One or more metrics in warning range
- **CRITICAL**: One or more metrics in critical range
- **UNKNOWN**: Node unable to collect metrics

## Quality of Service (QoS) Settings

### Health Status Topic
- **Reliability**: Reliable
- **Durability**: Transient Local
- **History**: Keep Last
- **Depth**: 10

### Alerts Topic
- **Reliability**: Reliable
- **Durability**: Transient Local
- **History**: Keep Last
- **Depth**: 10

## Launch Files

### Single Node Monitoring
Launches a single health monitoring setup:
```bash
ros2 launch edge_health single_node_monitoring.launch.py
```

### Multi-Node Monitoring
Launches multiple monitoring nodes for distributed systems:
```bash
ros2 launch edge_health multi_node_monitoring.launch.py
```

## Testing

### Command Line Testing
```bash
# Echo health status
ros2 topic echo /health_status

# Echo alerts
ros2 topic echo /alerts

# Call shutdown service
ros2 service call /safe_shutdown edge_health_interfaces/srv/SafeShutdown "{reason: 'test', requested_by: 'test_client', timeout: 30.0}"
```

### Node Information
```bash
# List nodes
ros2 node list

# Get node info
ros2 node info /health_monitor

# Get topic info
ros2 topic info /health_status
```