# Edge Health Package

Physical AI System Monitoring and Control package for ROS2.

## Overview

The edge_health package provides real-time monitoring of CPU, memory, disk, and temperature metrics for Physical AI hardware components. It includes threshold-based alerting and safe shutdown capabilities to protect hardware from damage due to overheating or resource exhaustion.

## Features

- Real-time system health monitoring (CPU, memory, disk, temperature)
- Configurable threshold-based alerting with hysteresis to prevent alert flapping
- Safe shutdown service for critical situations with configurable sequences
- Multi-node deployment support for distributed monitoring
- ROS2 integration with custom message types
- REST API for dashboard integration
- Comprehensive logging and monitoring
- Hardware-specific optimizations

## Architecture

The system consists of three main components:

1. **HealthMonitor**: Monitors system metrics and publishes health status
2. **AlertPublisher**: Subscribes to health status and publishes alerts when thresholds are exceeded
3. **SafeShutdownService**: Provides a service interface for safe shutdown procedures

## Installation

### Prerequisites
- Ubuntu 22.04 LTS
- ROS2 Humble Hawksbill
- Python 3.10 or 3.11
- Required Python packages: `psutil`, `pyyaml`, `rclpy`

### Setup
```bash
cd edge-kit/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Usage

### Running the Health Monitor

```bash
ros2 run edge_health health_monitor
```

### Launch Files

#### Single Node Monitoring
```bash
ros2 launch edge_health single_node_monitoring.launch.py
```

#### Multi-Node Monitoring
```bash
ros2 launch edge_health multi_node_monitoring.launch.py
```

### Dashboard Integration

The system provides a REST API for dashboard integration:

```bash
# Start the health API server
cd /media/awais/New\ Volume/hackathon/edge-kit/api
python3 health_api.py

# Access endpoints
curl http://localhost:5000/health
curl http://localhost:5000/status
curl http://localhost:5000/alerts
```

## Configuration

### Default Configuration
The system uses a YAML configuration file that can be customized for different hardware platforms. See `config/edge_health_config.yaml` for default settings.

### Node-Specific Configurations
- `config/edge_brain_config.yaml` - Optimized for main computing units
- `config/sensor_node_config.yaml` - Optimized for sensor monitoring nodes
- `config/actuator_node_config.yaml` - Optimized for actuator control nodes

### Configuration Parameters
- `node_name`: Name of the monitoring node
- `publish_frequency`: How often to publish health status (seconds)
- `thresholds`: Warning and critical thresholds for each metric
- `enabled_metrics`: List of metrics to monitor
- `log_level`: Logging level (DEBUG, INFO, WARNING, ERROR)
- `hardware_specific`: Hardware-specific settings
- `alert_recipients`: List of alert recipient identifiers

## ROS2 Interfaces

### Topics
- `/health_status` - Health metrics published at configured frequency
- `/alerts` - Alert messages when thresholds are exceeded

### Services
- `/safe_shutdown` - Service for initiating safe shutdown procedures

### Message Types
- `HealthStatus.msg` - System health metrics
- `Alert.msg` - Alert information
- `SafeShutdown.srv` - Safe shutdown request/response

## Performance

### Benchmarks
- **Publishing Rate**: Configurable from 0.1 Hz to 10 Hz (default 1 Hz)
- **Alert Response Time**: < 5 seconds from threshold violation to alert publication
- **Resource Usage**: < 5% CPU at 1 Hz polling
- **Reliability**: 99% message delivery rate

### Optimization
- Adjust `publish_frequency` based on requirements
- Enable only necessary metrics in `enabled_metrics`
- Use appropriate logging levels for production systems

## Testing

### Unit Tests
```bash
# Run unit tests
python3 -m pytest test/ -v
```

### Integration Tests
```bash
# Run integration tests
python3 -m pytest test/test_*integration*.py -v
```

### End-to-End Tests
```bash
# Run end-to-end tests
python3 -m pytest test/test_end_to_end_workflow.py -v
```

## API Documentation

Detailed API documentation is available in [API_DOCUMENTATION.md](API_DOCUMENTATION.md)

## Performance Benchmarks

Performance benchmarks and optimization guidelines are documented in [PERFORMANCE_BENCHMARKS.md](PERFORMANCE_BENCHMARKS.md)

## Quick Start Guide

For detailed setup and usage instructions, see [quickstart.md](../../specs/001-physical-ai-monitoring/quickstart.md)

## License

Apache 2.0