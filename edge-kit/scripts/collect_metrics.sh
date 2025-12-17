#!/bin/bash
# Metrics collection script - run via cron every minute
# Collects health metrics and appends to daily CSV file

METRICS_DIR="${METRICS_DIR:-/media/awais/New Volume/hackathon/edge-kit/data/metrics}"
DATE=$(date +"%Y-%m-%d")
TIMESTAMP=$(date +"%Y-%m-%d %H:%M:%S")
METRICS_FILE="$METRICS_DIR/metrics_$DATE.csv"

mkdir -p "$METRICS_DIR"

# Create header if file doesn't exist
if [ ! -f "$METRICS_FILE" ]; then
    echo "timestamp,cpu_percent,memory_percent,disk_percent,temperature,status,node_name,uptime_seconds" > "$METRICS_FILE"
fi

# Get metrics from Flask API
RESPONSE=$(curl -s --connect-timeout 5 --max-time 10 http://localhost:5000/health 2>/dev/null)

if [ -n "$RESPONSE" ] && [ "$RESPONSE" != "null" ]; then
    # Parse JSON response
    CPU=$(echo "$RESPONSE" | jq -r '.cpu_percent // -1' 2>/dev/null)
    MEMORY=$(echo "$RESPONSE" | jq -r '.memory_percent // -1' 2>/dev/null)
    DISK=$(echo "$RESPONSE" | jq -r '.disk_percent // -1' 2>/dev/null)
    TEMP=$(echo "$RESPONSE" | jq -r '.temperature // -1' 2>/dev/null)
    STATUS=$(echo "$RESPONSE" | jq -r '.status // "UNKNOWN"' 2>/dev/null)
    NODE=$(echo "$RESPONSE" | jq -r '.node_name // "unknown"' 2>/dev/null)
    UPTIME=$(echo "$RESPONSE" | jq -r '.uptime_seconds // 0' 2>/dev/null)

    # Validate numeric values
    if [[ ! "$CPU" =~ ^-?[0-9]+\.?[0-9]*$ ]]; then CPU=-1; fi
    if [[ ! "$MEMORY" =~ ^-?[0-9]+\.?[0-9]*$ ]]; then MEMORY=-1; fi
    if [[ ! "$DISK" =~ ^-?[0-9]+\.?[0-9]*$ ]]; then DISK=-1; fi
    if [[ ! "$TEMP" =~ ^-?[0-9]+\.?[0-9]*$ ]]; then TEMP=-1; fi
    if [[ ! "$UPTIME" =~ ^-?[0-9]+\.?[0-9]*$ ]]; then UPTIME=0; fi

    # Append to CSV
    echo "$TIMESTAMP,$CPU,$MEMORY,$DISK,$TEMP,$STATUS,$NODE,$UPTIME" >> "$METRICS_FILE"

    # Log if any metric is in warning/critical state
    if [ "$STATUS" == "WARNING" ] || [ "$STATUS" == "CRITICAL" ]; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] ALERT: Status is $STATUS - CPU=$CPU%, MEM=$MEMORY%, DISK=$DISK%, TEMP=$TEMP" >> "$METRICS_DIR/status_alerts.log"
    fi
else
    # Log failure to collect metrics
    echo "$TIMESTAMP,-1,-1,-1,-1,API_UNREACHABLE,unknown,0" >> "$METRICS_FILE"
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] ERROR: Failed to collect metrics from API" >> "$METRICS_DIR/collection_errors.log"
fi
