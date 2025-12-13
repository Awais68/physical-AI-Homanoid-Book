#!/bin/bash
# Production entrypoint for Edge Health Monitoring System
# Designed for 7-day continuous operation

set -e

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Set RMW implementation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Log startup
echo "[$(date '+%Y-%m-%d %H:%M:%S')] Edge Health Monitoring System starting..."
echo "[$(date '+%Y-%m-%d %H:%M:%S')] ROS_DISTRO: $ROS_DISTRO"
echo "[$(date '+%Y-%m-%d %H:%M:%S')] RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "[$(date '+%Y-%m-%d %H:%M:%S')] Log level: ${EDGE_HEALTH_LOG_LEVEL:-INFO}"

# Create data directories if they don't exist
mkdir -p /ros2_ws/data/logs
mkdir -p /ros2_ws/data/metrics
mkdir -p /ros2_ws/data/alerts
mkdir -p /ros2_ws/data/backups

# Function to handle shutdown signals
shutdown_handler() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Received shutdown signal, initiating graceful shutdown..."

    # Kill background processes
    if [ -n "$FLASK_PID" ]; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Stopping Flask API..."
        kill -TERM $FLASK_PID 2>/dev/null || true
    fi

    if [ -n "$ROS_PID" ]; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Stopping ROS2 nodes..."
        kill -TERM $ROS_PID 2>/dev/null || true
    fi

    # Wait for processes to terminate
    wait

    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Shutdown complete."
    exit 0
}

# Register signal handlers
trap shutdown_handler SIGTERM SIGINT SIGQUIT

# Health check function
health_check() {
    # Check if ROS2 nodes are running
    if ros2 node list 2>/dev/null | grep -q "health_monitor"; then
        echo "ROS2 health_monitor node: OK"
    else
        echo "ROS2 health_monitor node: NOT RUNNING"
        return 1
    fi

    # Check Flask API
    if curl -s -f http://localhost:5000/health > /dev/null 2>&1; then
        echo "Flask API: OK"
    else
        echo "Flask API: NOT RUNNING"
        return 1
    fi

    return 0
}

# Export health check for external use
export -f health_check

# Execute the command passed to the container
exec "$@"
