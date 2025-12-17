#!/bin/bash
# Start all monitoring services for 7-day deployment
# This script launches ROS2 nodes and Flask API

set -e

echo "[$(date '+%Y-%m-%d %H:%M:%S')] Starting Edge Health Monitoring System..."

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Log directory
LOG_DIR="/ros2_ws/data/logs"
mkdir -p "$LOG_DIR"

# Start Flask API in background
echo "[$(date '+%Y-%m-%d %H:%M:%S')] Starting Flask API on port 5000..."
cd /ros2_ws/src/edge_health
python3 -m gunicorn -w 2 -b 0.0.0.0:5000 edge_health.api.health_api:app \
    --access-logfile "$LOG_DIR/flask_access.log" \
    --error-logfile "$LOG_DIR/flask_error.log" \
    --timeout 120 &
FLASK_PID=$!
echo "[$(date '+%Y-%m-%d %H:%M:%S')] Flask API started with PID: $FLASK_PID"

# Wait for Flask to be ready
sleep 3

# Start ROS2 health monitoring nodes
echo "[$(date '+%Y-%m-%d %H:%M:%S')] Starting ROS2 health monitoring nodes..."

# Launch the health monitor node
ros2 run edge_health health_monitor --ros-args \
    -p publish_frequency:=1.0 \
    -p log_level:=${EDGE_HEALTH_LOG_LEVEL:-INFO} \
    > "$LOG_DIR/health_monitor.log" 2>&1 &
HEALTH_MONITOR_PID=$!
echo "[$(date '+%Y-%m-%d %H:%M:%S')] Health monitor started with PID: $HEALTH_MONITOR_PID"

# Launch the threshold monitor node
ros2 run edge_health threshold_monitor --ros-args \
    -p log_level:=${EDGE_HEALTH_LOG_LEVEL:-INFO} \
    > "$LOG_DIR/threshold_monitor.log" 2>&1 &
THRESHOLD_MONITOR_PID=$!
echo "[$(date '+%Y-%m-%d %H:%M:%S')] Threshold monitor started with PID: $THRESHOLD_MONITOR_PID"

# Launch the alert publisher node
ros2 run edge_health alert_publisher --ros-args \
    -p log_level:=${EDGE_HEALTH_LOG_LEVEL:-INFO} \
    > "$LOG_DIR/alert_publisher.log" 2>&1 &
ALERT_PUBLISHER_PID=$!
echo "[$(date '+%Y-%m-%d %H:%M:%S')] Alert publisher started with PID: $ALERT_PUBLISHER_PID"

# Save PIDs for management
echo "$FLASK_PID" > /tmp/flask.pid
echo "$HEALTH_MONITOR_PID" > /tmp/health_monitor.pid
echo "$THRESHOLD_MONITOR_PID" > /tmp/threshold_monitor.pid
echo "$ALERT_PUBLISHER_PID" > /tmp/alert_publisher.pid

echo "[$(date '+%Y-%m-%d %H:%M:%S')] All services started successfully!"
echo ""
echo "=== Service Status ==="
echo "Flask API:          PID $FLASK_PID (port 5000)"
echo "Health Monitor:     PID $HEALTH_MONITOR_PID"
echo "Threshold Monitor:  PID $THRESHOLD_MONITOR_PID"
echo "Alert Publisher:    PID $ALERT_PUBLISHER_PID"
echo ""
echo "Log files in: $LOG_DIR"
echo ""

# Function to check if all services are running
check_services() {
    local all_running=true

    if ! kill -0 $FLASK_PID 2>/dev/null; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] WARNING: Flask API is not running!"
        all_running=false
    fi

    if ! kill -0 $HEALTH_MONITOR_PID 2>/dev/null; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] WARNING: Health monitor is not running!"
        all_running=false
    fi

    if ! kill -0 $THRESHOLD_MONITOR_PID 2>/dev/null; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] WARNING: Threshold monitor is not running!"
        all_running=false
    fi

    if ! kill -0 $ALERT_PUBLISHER_PID 2>/dev/null; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] WARNING: Alert publisher is not running!"
        all_running=false
    fi

    $all_running
}

# Keep the script running and monitoring services
echo "[$(date '+%Y-%m-%d %H:%M:%S')] Entering monitoring loop..."
while true; do
    sleep 60

    if ! check_services; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Some services have stopped. Container will restart if configured with restart policy."
    fi

    # Log heartbeat every hour
    if [ $(($(date +%s) % 3600)) -lt 60 ]; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Heartbeat: All services running"
    fi
done
