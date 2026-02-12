# Quickstart Guide: Physical AI System Monitoring and Control

## Overview
This guide provides a quick setup and run instructions for the Physical AI System Monitoring and Control feature. This system provides real-time monitoring of CPU, memory, disk, and temperature metrics with threshold-based alerts and safe shutdown capabilities.

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS
- ROS2 Humble Hawksbill installed
- Python 3.10 or 3.11
- At least 2GB free disk space
- 4GB RAM recommended

### Dependencies
```bash
# Install ROS2 Humble (if not already installed)
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Install Python dependencies
pip3 install psutil pyyaml rclpy
```

## Installation

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Set up the ROS2 Workspace
```bash
cd edge-kit/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 3. Install Python Package Dependencies
```bash
cd src/edge_health
pip3 install -e .
```

## Running the Health Monitor

### Option 1: Using ROS2 Launch (Recommended)
```bash
# Source the workspace
cd edge-kit/ros2_ws
source install/setup.bash

# Launch the health monitor
ros2 launch edge_health single_node_monitoring.launch.py
```

### Option 2: Direct Python Execution
```bash
# Run the health monitor directly
cd edge-kit/ros2_ws/src/edge_health
python3 -m edge_health.health_monitor
```

### Option 3: Using the Standalone Script
```bash
# Run using the provided script
cd edge-kit
python3 scripts/run_health_monitor.py
```

## Configuration

### Default Configuration
The system uses default configuration from `config/edge_health_config.yaml`. You can modify thresholds and settings:

```yaml
# Example configuration
node_name: "health_monitor"
publish_frequency: 1.0  # Health status published every 1 second
thresholds:
  cpu:
    warning: 80.0      # CPU usage warning threshold
    critical: 90.0     # CPU usage critical threshold
  memory:
    warning: 85.0      # Memory usage warning threshold
    critical: 95.0     # Memory usage critical threshold
```

### Custom Configuration
To use a custom configuration file:
```bash
# Set the configuration file path as an environment variable
export EDGE_HEALTH_CONFIG_PATH="/path/to/custom/config.yaml"

# Or pass it as an argument to the node
ros2 run edge_health health_monitor --config-path /path/to/custom/config.yaml
```

## Testing the System

### 1. Verify Health Status Publishing
```bash
# Check if the health status topic is being published
ros2 topic echo /health_status
```

### 2. Monitor System Load (to trigger alerts)
```bash
# In another terminal, create CPU load to test alerts
stress --cpu 8 --timeout 30s
```

### 3. Check for Alerts
```bash
# Monitor the alerts topic
ros2 topic echo /alerts
```

### 4. Test Safe Shutdown Service
```bash
# Call the safe shutdown service
ros2 service call /safe_shutdown edge_health_interfaces/srv/SafeShutdown "{reason: 'test', requested_by: 'test_client', timeout: 30.0}"
```

## Multi-Node Deployment

### Launch Multiple Monitoring Nodes
```bash
# Launch multiple nodes for different hardware components
ros2 launch edge_health multi_node_monitoring.launch.py
```

### Configuration for Multiple Nodes
Each node can have its own configuration:
- `edge_brain_config.yaml` - Configuration for the main computing unit
- `sensor_node_config.yaml` - Configuration for sensor monitoring nodes
- `actuator_node_config.yaml` - Configuration for actuator monitoring nodes

## Troubleshooting

### Common Issues

1. **"Module not found" errors**
   ```bash
   # Make sure you've sourced the ROS2 workspace
   cd edge-kit/ros2_ws
   source install/setup.bash
   ```

2. **No messages on /health_status topic**
   - Check that the health monitor node is running: `ros2 node list`
   - Verify the node has proper permissions to access system metrics

3. **High CPU usage by monitoring system**
   - Increase the `publish_frequency` in the configuration to reduce polling rate
   - Disable unnecessary metrics in `enabled_metrics`

### Useful Commands
```bash
# List all running ROS2 nodes
ros2 node list

# Check ROS2 topics
ros2 topic list

# Get information about a specific topic
ros2 topic info /health_status

# Check ROS2 services
ros2 service list
```

## Dashboard Integration

### REST API Access
The system provides a REST API for dashboard integration:

```bash
# Start the health API server
cd /media/awais/New Volume/hackathon/edge-kit/api
python3 health_api.py

# Get current health status
curl http://localhost:5000/health

# Get system status summary
curl http://localhost:5000/status

# Get recent alerts
curl http://localhost:5000/alerts

# Get historical health data
curl "http://localhost:5000/health/history?limit=50"

# Get aggregated health statistics
curl "http://localhost:5000/health/aggregate?hours=24"
```

### API Endpoints
- `GET /health`: Current health status
- `GET /health/history`: Historical health data
- `GET /alerts`: Current and recent alerts
- `GET /status`: Overall system status summary
- `GET /health/aggregate`: Aggregated health statistics
- `POST /health`: Submit health data (internal use)
- `POST /alerts`: Submit alert data (internal use)

## Next Steps

1. **Customize thresholds** in the configuration file for your specific hardware
2. **Integrate with your dashboard** by subscribing to `/health_status` and `/alerts` topics
3. **Set up alert handling** to process alerts from the `/alerts` topic
4. **Connect your dashboard** to the REST API at `http://your-server:5000`
5. **Configure multi-node deployment** for monitoring distributed hardware components