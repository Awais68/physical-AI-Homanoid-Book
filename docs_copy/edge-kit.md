# Physical AI Edge Kit

The Physical AI Edge Kit provides a comprehensive monitoring and control system for educational robotics environments. This kit enables real-time health monitoring, threshold-based alerting, and safe shutdown capabilities for physical AI systems deployed in educational settings.

## Overview

The Edge Kit is built on ROS2 (Robot Operating System 2) and provides:

- **Real-time Health Monitoring**: Continuous monitoring of CPU, memory, disk, and temperature metrics
- **Threshold-based Alerting**: Configurable alerts with WARNING, ERROR, and CRITICAL severity levels
- **Safe Shutdown Service**: ROS2 service interface for initiating safe hardware shutdown procedures
- **Multi-node Deployment**: Support for monitoring distributed Physical AI hardware components
- **Dashboard Integration**: REST API for external dashboard consumption

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Physical AI Edge Kit                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────┐ │
│  │  HealthMonitor  │───▶│ ThresholdMonitor│───▶│AlertPublisher│ │
│  │    (ROS2 Node)  │    │   (ROS2 Node)   │    │ (ROS2 Node) │ │
│  └────────┬────────┘    └─────────────────┘    └──────┬──────┘ │
│           │                                           │        │
│           ▼                                           ▼        │
│  ┌─────────────────┐                        ┌─────────────────┐│
│  │ /health_status  │                        │    /alerts      ││
│  │    (Topic)      │                        │    (Topic)      ││
│  └─────────────────┘                        └─────────────────┘│
│                                                                 │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │                 SafeShutdownService                         ││
│  │                   /safe_shutdown                            ││
│  └─────────────────────────────────────────────────────────────┘│
│                                                                 │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │                    Flask Health API                         ││
│  │              REST API @ localhost:5000                      ││
│  └─────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
```

## Quick Start

### Prerequisites

- Docker and Docker Compose
- At least 2GB free disk space
- 4GB RAM recommended

### Installation

```bash
# Clone the repository
git clone https://github.com/Awais68/physical-AI-Homanoid-Book

# Navigate to the docker directory
cd docker

# Build the production image
docker compose build edge-kit-prod

# Start the deployment
docker compose up -d edge-kit-prod
```

### Verify Installation

```bash
# Check container status
docker compose ps

# Access health API
curl http://localhost:5000/health
curl http://localhost:5000/status
```

## Configuration

The Edge Kit uses YAML-based configuration. Default configuration file location: `edge-kit/config/edge_health_config.yaml`

### Threshold Configuration

```yaml
node_name: "health_monitor"
publish_frequency: 1.0  # Health status published every 1 second

thresholds:
  cpu:
    warning: 80.0      # CPU usage warning threshold
    critical: 90.0     # CPU usage critical threshold
  memory:
    warning: 85.0      # Memory usage warning threshold
    critical: 95.0     # Memory usage critical threshold
  disk:
    warning: 80.0      # Disk usage warning threshold
    critical: 90.0     # Disk usage critical threshold
  temperature:
    warning: 70.0      # Temperature warning threshold (Celsius)
    critical: 85.0     # Temperature critical threshold
```

## API Reference

### REST Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/health` | GET | Current health status |
| `/health/history` | GET | Historical health data |
| `/health/aggregate` | GET | Aggregated health statistics |
| `/alerts` | GET | Current and recent alerts |
| `/status` | GET | Overall system status summary |

### Example Responses

**GET /health**
```json
{
  "cpu_percent": 5.2,
  "memory_percent": 42.1,
  "disk_percent": 35.7,
  "temperature": 45.0,
  "status": "OK",
  "node_name": "health_monitor",
  "timestamp": "2025-12-12T22:00:00Z"
}
```

**GET /alerts**
```json
{
  "data": [
    {
      "alert_id": "alert_001",
      "severity": "WARNING",
      "metric": "cpu_percent",
      "value": 82.5,
      "threshold": 80.0,
      "timestamp": "2025-12-12T21:45:00Z",
      "resolved": true
    }
  ],
  "count": 1
}
```

## ROS2 Topics and Services

### Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/health_status` | `HealthStatus` | Published at 1Hz with current system metrics |
| `/alerts` | `Alert` | Published when thresholds are exceeded |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/safe_shutdown` | `SafeShutdown` | Initiate safe hardware shutdown |

### Example: Calling Safe Shutdown Service

```bash
ros2 service call /safe_shutdown edge_health_interfaces/srv/SafeShutdown \
  "{reason: 'CRITICAL_TEMPERATURE', requested_by: 'operator', timeout: 30.0}"
```

## Multi-Node Deployment

For monitoring multiple hardware components:

```bash
# Launch multiple monitoring nodes
ros2 launch edge_health multi_node_monitoring.launch.py
```

Each node can have its own configuration:
- `edge_brain_config.yaml` - Main computing unit
- `sensor_node_config.yaml` - Sensor monitoring nodes
- `actuator_node_config.yaml` - Actuator monitoring nodes

## Monitoring Scripts

The Edge Kit includes several scripts for automated monitoring:

| Script | Purpose | Schedule |
|--------|---------|----------|
| `health_check.sh` | Container and node status | Every 5 minutes |
| `collect_metrics.sh` | Metrics collection | Every minute |
| `collect_alerts.sh` | Alert aggregation | Every 5 minutes |
| `backup_logs.sh` | Daily log backup | Daily at midnight |
| `generate_7day_report.sh` | Reliability report | After 7-day test |

## 7-Day Reliability Test

The Edge Kit includes infrastructure for conducting a 7-day continuous operation test to verify system reliability.

### Success Criteria

1. **Uptime**: Container runs continuously for 168 hours with < 3 restarts
2. **Reliability**: Health metrics published at > 99% of expected rate (1Hz)
3. **Stability**: No memory leaks (memory usage variance < 10%)
4. **Alerts**: Zero unresolved CRITICAL alerts
5. **Performance**: Average CPU usage < 10%
6. **Data Integrity**: All metrics collected without gaps > 5 minutes

### Running the Test

```bash
# Start the deployment
cd docker
docker compose up -d edge-kit-prod

# Set up monitoring cron jobs (see 7-DAY-DEPLOYMENT-PLAN.md)

# After 7 days, generate the report
./edge-kit/scripts/generate_7day_report.sh
```

## Troubleshooting

### Common Issues

**Container not starting**
```bash
# Check container logs
docker compose logs edge-kit-prod

# Verify Docker resources
docker system df
```

**ROS2 nodes not publishing**
```bash
# Enter container
docker compose exec edge-kit-prod bash

# Check node status
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 node list
ros2 topic info /health_status
```

**Flask API not responding**
```bash
# Check API process
curl -v http://localhost:5000/health

# Restart API service
docker compose restart edge-kit-prod
```

## Educational Use Cases

### For Educators

- Monitor student interactions with physical AI systems
- Set safety thresholds appropriate for age groups
- Review system health before classroom sessions
- Configure alerts for immediate notification of issues

### For Students

- Learn about system monitoring concepts
- Understand threshold-based alerting
- Explore ROS2 topics and services
- Practice safe robotics operation

### For Administrators

- Deploy Edge Kit across multiple classroom environments
- Configure multi-node monitoring for distributed systems
- Access aggregated health metrics for all systems
- Generate reliability reports for compliance

## Resources

- [7-Day Reliability Test Report](./7day-report.md)
- [GitHub Repository](https://github.com/Awais68/physical-AI-Homanoid-Book)

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-12-12 | Initial release with health monitoring, alerting, and safe shutdown |
