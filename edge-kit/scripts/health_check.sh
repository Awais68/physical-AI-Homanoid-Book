#!/bin/bash
# Health check script for edge_health monitoring
# Run manually or via cron every 5 minutes

CONTAINER_NAME="${CONTAINER_NAME:-edge-kit-ros2}"
LOG_DIR="${LOG_DIR:-/media/awais/New Volume/hackathon/edge-kit/data/logs}"
TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")

mkdir -p "$LOG_DIR"

echo "=== Health Check: $TIMESTAMP ===" | tee -a "$LOG_DIR/health_checks.log"

# Check container status
CONTAINER_STATUS=$(docker inspect -f '{{.State.Status}}' "$CONTAINER_NAME" 2>/dev/null)
if [ -z "$CONTAINER_STATUS" ]; then
    echo "ERROR: Container '$CONTAINER_NAME' not found!" | tee -a "$LOG_DIR/health_checks.log"
    exit 1
fi

echo "Container Status: $CONTAINER_STATUS" | tee -a "$LOG_DIR/health_checks.log"

if [ "$CONTAINER_STATUS" != "running" ]; then
    echo "WARNING: Container not running!" | tee -a "$LOG_DIR/health_checks.log"
    exit 1
fi

# Get container uptime
STARTED_AT=$(docker inspect -f '{{.State.StartedAt}}' "$CONTAINER_NAME" 2>/dev/null)
echo "Container Started: $STARTED_AT" | tee -a "$LOG_DIR/health_checks.log"

# Check ROS2 nodes
echo "Checking ROS2 nodes..." | tee -a "$LOG_DIR/health_checks.log"
NODE_LIST=$(docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 node list 2>/dev/null" || echo "FAILED")
NODE_COUNT=$(echo "$NODE_LIST" | grep -v "^$" | wc -l)
echo "Active ROS2 Nodes: $NODE_COUNT" | tee -a "$LOG_DIR/health_checks.log"
echo "$NODE_LIST" | head -10 | tee -a "$LOG_DIR/health_checks.log"

# Check topic publishing rate
echo "Checking /health_status topic..." | tee -a "$LOG_DIR/health_checks.log"
TOPIC_HZ=$(docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout 5 ros2 topic hz /health_status 2>/dev/null | head -3" || echo "Topic not available")
echo "Health Status Topic Hz: $TOPIC_HZ" | tee -a "$LOG_DIR/health_checks.log"

# Check Flask API
echo "Checking Flask API..." | tee -a "$LOG_DIR/health_checks.log"
API_STATUS=$(curl -s -o /dev/null -w "%{http_code}" http://localhost:5000/health 2>/dev/null || echo "unavailable")
echo "Flask API Status Code: $API_STATUS" | tee -a "$LOG_DIR/health_checks.log"

# Get current metrics from Flask API
if [ "$API_STATUS" == "200" ]; then
    HEALTH_DATA=$(curl -s http://localhost:5000/health 2>/dev/null)
    echo "Current Health Data:" | tee -a "$LOG_DIR/health_checks.log"
    echo "$HEALTH_DATA" | jq '.' 2>/dev/null | tee -a "$LOG_DIR/health_checks.log" || echo "$HEALTH_DATA" | tee -a "$LOG_DIR/health_checks.log"

    # Extract key metrics
    CPU=$(echo "$HEALTH_DATA" | jq -r '.cpu_percent // "N/A"' 2>/dev/null)
    MEMORY=$(echo "$HEALTH_DATA" | jq -r '.memory_percent // "N/A"' 2>/dev/null)
    DISK=$(echo "$HEALTH_DATA" | jq -r '.disk_percent // "N/A"' 2>/dev/null)
    TEMP=$(echo "$HEALTH_DATA" | jq -r '.temperature // "N/A"' 2>/dev/null)
    STATUS=$(echo "$HEALTH_DATA" | jq -r '.status // "N/A"' 2>/dev/null)

    echo "" | tee -a "$LOG_DIR/health_checks.log"
    echo "Summary: CPU=$CPU%, MEM=$MEMORY%, DISK=$DISK%, TEMP=$TEMP, STATUS=$STATUS" | tee -a "$LOG_DIR/health_checks.log"
else
    echo "WARNING: Flask API not responding!" | tee -a "$LOG_DIR/health_checks.log"
fi

# Check container resource usage
echo "" | tee -a "$LOG_DIR/health_checks.log"
echo "Container Resource Usage:" | tee -a "$LOG_DIR/health_checks.log"
docker stats "$CONTAINER_NAME" --no-stream --format "CPU: {{.CPUPerc}}, MEM: {{.MemUsage}}, NET: {{.NetIO}}" 2>/dev/null | tee -a "$LOG_DIR/health_checks.log"

# Check for alerts in the last hour
if [ "$API_STATUS" == "200" ]; then
    ALERT_COUNT=$(curl -s "http://localhost:5000/alerts?hours=1" 2>/dev/null | jq '.data | length' 2>/dev/null || echo "0")
    echo "Alerts in last hour: $ALERT_COUNT" | tee -a "$LOG_DIR/health_checks.log"
fi

echo "=== Check Complete ===" | tee -a "$LOG_DIR/health_checks.log"
echo "" >> "$LOG_DIR/health_checks.log"

# Exit with appropriate code
if [ "$CONTAINER_STATUS" == "running" ] && [ "$API_STATUS" == "200" ]; then
    echo "Overall Status: HEALTHY"
    exit 0
else
    echo "Overall Status: UNHEALTHY"
    exit 1
fi
