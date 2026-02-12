# Physical AI Edge Kit

A comprehensive toolkit for deploying and managing Physical AI applications on edge devices.

## Components

### 1. ROS2 Workspace (`ros2_ws/`)
Contains ROS2 packages for robotics and AI applications:
- `edge_health` - Health monitoring for edge devices

### 2. Scripts (`scripts/`)
Utility scripts for common operations:
- `run_health_monitor.py` - Run the health monitoring service

### 3. Config (`config/`)
Configuration files for various services:
- `edge_health_config.yaml` - Health monitoring configuration

### 4. API (`api/`)
REST API endpoints for monitoring and control:
- `health_api.py` - Health monitoring API

## Getting Started

### Prerequisites
- Python 3.8+
- pip

### Installation
1. Clone this repository
2. Install dependencies:
   ```bash
   pip3 install -r ros2_ws/src/edge_health/requirements.txt
   pip3 install -r api/requirements.txt
   ```

### Running the Health Monitor
```bash
python3 scripts/run_health_monitor.py
```

### Running the API
```bash
cd api
python3 health_api.py
```

The API will be available at `http://localhost:5000`

## API Endpoints

- `GET /health` - Get current health status
- `GET /health/system` - Get detailed system information
- `GET /status` - Simple status check

## Configuration

The health monitoring system can be configured via `config/edge_health_config.yaml`.