#!/bin/bash
# Alert collection script - run via cron every 5 minutes
# Collects alerts from the Flask API and stores them

ALERTS_DIR="${ALERTS_DIR:-/media/awais/New Volume/hackathon/edge-kit/data/alerts}"
DATE=$(date +"%Y-%m-%d")
TIMESTAMP=$(date +"%Y-%m-%d %H:%M:%S")
ALERTS_FILE="$ALERTS_DIR/alerts_$DATE.json"

mkdir -p "$ALERTS_DIR"

# Initialize alerts file if it doesn't exist
if [ ! -f "$ALERTS_FILE" ]; then
    echo '{"collected_alerts": []}' > "$ALERTS_FILE"
fi

# Get alerts from Flask API (last hour)
RESPONSE=$(curl -s --connect-timeout 5 --max-time 10 "http://localhost:5000/alerts?hours=1" 2>/dev/null)

if [ -n "$RESPONSE" ] && [ "$RESPONSE" != "null" ]; then
    # Count alerts
    ALERT_COUNT=$(echo "$RESPONSE" | jq '.data | length' 2>/dev/null || echo "0")

    if [ "$ALERT_COUNT" != "0" ] && [ "$ALERT_COUNT" != "null" ]; then
        # Create alert record
        RECORD=$(cat <<EOF
{
  "collected_at": "$TIMESTAMP",
  "alert_count": $ALERT_COUNT,
  "alerts": $RESPONSE
}
EOF
)
        # Append to daily alerts file (using jq to merge)
        CURRENT=$(cat "$ALERTS_FILE")
        echo "$CURRENT" | jq --argjson new "$RECORD" '.collected_alerts += [$new]' > "${ALERTS_FILE}.tmp"
        mv "${ALERTS_FILE}.tmp" "$ALERTS_FILE"

        # Log summary
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Collected $ALERT_COUNT alerts" >> "$ALERTS_DIR/collection.log"

        # Extract and log critical/error alerts
        CRITICAL_COUNT=$(echo "$RESPONSE" | jq '[.data[] | select(.severity == "CRITICAL")] | length' 2>/dev/null || echo "0")
        ERROR_COUNT=$(echo "$RESPONSE" | jq '[.data[] | select(.severity == "ERROR")] | length' 2>/dev/null || echo "0")

        if [ "$CRITICAL_COUNT" != "0" ] || [ "$ERROR_COUNT" != "0" ]; then
            echo "[$(date '+%Y-%m-%d %H:%M:%S')] IMPORTANT: Found $CRITICAL_COUNT CRITICAL and $ERROR_COUNT ERROR alerts" >> "$ALERTS_DIR/important_alerts.log"

            # Log details of critical alerts
            echo "$RESPONSE" | jq -r '.data[] | select(.severity == "CRITICAL" or .severity == "ERROR") | "\(.timestamp) [\(.severity)] \(.metric_name): \(.message)"' >> "$ALERTS_DIR/important_alerts.log" 2>/dev/null
        fi
    else
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] No new alerts" >> "$ALERTS_DIR/collection.log"
    fi
else
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] ERROR: Failed to fetch alerts from API" >> "$ALERTS_DIR/collection.log"
fi
