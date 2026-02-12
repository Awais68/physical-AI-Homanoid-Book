# 7-Day ROS2 Docker Deployment and Monitoring Plan

## Overview

This document outlines the complete deployment and monitoring plan for the Physical AI System Monitoring edge_health package. The goal is to verify system reliability over a continuous 7-day period (Task T060).

**Start Date**: _____________
**End Date**: _____________
**Operator**: _____________

---

## 1. Pre-Deployment Setup

### 1.1 Directory Structure

```bash
# Create required directories
mkdir -p /media/awais/New\ Volume/hackathon/edge-kit/data/logs
mkdir -p /media/awais/New\ Volume/hackathon/edge-kit/data/metrics
mkdir -p /media/awais/New\ Volume/hackathon/edge-kit/data/backups
mkdir -p /media/awais/New\ Volume/hackathon/edge-kit/data/alerts
```

### 1.2 Verify Prerequisites

```bash
# Check Docker version
docker --version

# Check Docker Compose version
docker compose version

# Check available disk space (need at least 10GB)
df -h /media/awais/New\ Volume/hackathon/

# Check system resources
free -h
nproc
```

---

## 2. Docker Configuration

### 2.1 Production Dockerfile

The production Dockerfile is located at `edge-kit/Dockerfile.prod`:

```dockerfile
# See edge-kit/Dockerfile.prod for the full production configuration
```

### 2.2 Docker Compose Configuration

Update `docker/docker-compose.yml` with production settings including:
- Persistent volumes for logs and data
- Health checks
- Restart policies
- Resource limits

### 2.3 Environment Variables

Create `.env` file in `docker/` directory:

```bash
# ROS2 Configuration
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Monitoring Configuration
EDGE_HEALTH_LOG_LEVEL=INFO
EDGE_HEALTH_PUBLISH_FREQUENCY=1.0

# Flask Dashboard
FLASK_HOST=0.0.0.0
FLASK_PORT=5000

# Data Retention
LOG_RETENTION_DAYS=14
METRICS_RETENTION_DAYS=30
```

---

## 3. Deployment Steps

### 3.1 Build Production Image

```bash
cd /media/awais/New\ Volume/hackathon/docker

# Build the production image
docker compose build edge-kit

# Verify the image
docker images | grep edge-kit
```

### 3.2 Start the Deployment

```bash
# Start in detached mode
docker compose up -d edge-kit

# Verify container is running
docker compose ps

# Check container logs
docker compose logs -f edge-kit
```

### 3.3 Verify ROS2 Nodes are Running

```bash
# Enter the container
docker compose exec edge-kit bash

# Inside container: Check ROS2 nodes
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# List running nodes
ros2 node list

# Check topics
ros2 topic list

# Echo health status (Ctrl+C to stop)
ros2 topic echo /health_status

# Check services
ros2 service list

# Exit container
exit
```

### 3.4 Start Flask Dashboard

```bash
# Start the Flask API in a separate terminal or as part of compose
docker compose exec edge-kit bash -c "cd /ros2_ws/src/edge_health && python3 -m edge_health.api.health_api &"

# Or access via host if exposed
curl http://localhost:5000/health
curl http://localhost:5000/status
```

---

## 4. Monitoring Scripts

### 4.1 Health Check Script

Save as `edge-kit/scripts/health_check.sh`:

```bash
#!/bin/bash
# Health check script for edge_health monitoring

CONTAINER_NAME="edge-kit-ros2"
LOG_DIR="/media/awais/New Volume/hackathon/edge-kit/data/logs"
TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")

echo "=== Health Check: $TIMESTAMP ===" | tee -a "$LOG_DIR/health_checks.log"

# Check container status
CONTAINER_STATUS=$(docker inspect -f '{{.State.Status}}' $CONTAINER_NAME 2>/dev/null)
echo "Container Status: $CONTAINER_STATUS" | tee -a "$LOG_DIR/health_checks.log"

if [ "$CONTAINER_STATUS" != "running" ]; then
    echo "WARNING: Container not running!" | tee -a "$LOG_DIR/health_checks.log"
    exit 1
fi

# Check ROS2 nodes
NODE_COUNT=$(docker exec $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 node list 2>/dev/null | wc -l")
echo "Active ROS2 Nodes: $NODE_COUNT" | tee -a "$LOG_DIR/health_checks.log"

# Check topic publishing
TOPIC_ACTIVE=$(docker exec $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout 5 ros2 topic hz /health_status 2>/dev/null | head -1")
echo "Health Status Topic: $TOPIC_ACTIVE" | tee -a "$LOG_DIR/health_checks.log"

# Check Flask API
API_STATUS=$(curl -s -o /dev/null -w "%{http_code}" http://localhost:5000/health 2>/dev/null || echo "unavailable")
echo "Flask API Status: $API_STATUS" | tee -a "$LOG_DIR/health_checks.log"

# Get current metrics
if [ "$API_STATUS" == "200" ]; then
    METRICS=$(curl -s http://localhost:5000/health)
    echo "Current Metrics: $METRICS" | tee -a "$LOG_DIR/health_checks.log"
fi

echo "=== Check Complete ===" | tee -a "$LOG_DIR/health_checks.log"
echo "" >> "$LOG_DIR/health_checks.log"
```

### 4.2 Metrics Collection Script

Save as `edge-kit/scripts/collect_metrics.sh`:

```bash
#!/bin/bash
# Metrics collection script - run via cron every minute

METRICS_DIR="/media/awais/New Volume/hackathon/edge-kit/data/metrics"
DATE=$(date +"%Y-%m-%d")
TIMESTAMP=$(date +"%Y-%m-%d %H:%M:%S")
METRICS_FILE="$METRICS_DIR/metrics_$DATE.csv"

# Create header if file doesn't exist
if [ ! -f "$METRICS_FILE" ]; then
    echo "timestamp,cpu_percent,memory_percent,disk_percent,temperature,status,node_name" > "$METRICS_FILE"
fi

# Get metrics from Flask API
RESPONSE=$(curl -s http://localhost:5000/health 2>/dev/null)

if [ -n "$RESPONSE" ]; then
    CPU=$(echo $RESPONSE | jq -r '.cpu_percent // -1')
    MEMORY=$(echo $RESPONSE | jq -r '.memory_percent // -1')
    DISK=$(echo $RESPONSE | jq -r '.disk_percent // -1')
    TEMP=$(echo $RESPONSE | jq -r '.temperature // -1')
    STATUS=$(echo $RESPONSE | jq -r '.status // "UNKNOWN"')
    NODE=$(echo $RESPONSE | jq -r '.node_name // "unknown"')

    echo "$TIMESTAMP,$CPU,$MEMORY,$DISK,$TEMP,$STATUS,$NODE" >> "$METRICS_FILE"
fi
```

### 4.3 Alert Collection Script

Save as `edge-kit/scripts/collect_alerts.sh`:

```bash
#!/bin/bash
# Alert collection script - run via cron every 5 minutes

ALERTS_DIR="/media/awais/New Volume/hackathon/edge-kit/data/alerts"
DATE=$(date +"%Y-%m-%d")
TIMESTAMP=$(date +"%Y-%m-%d %H:%M:%S")
ALERTS_FILE="$ALERTS_DIR/alerts_$DATE.json"

# Get alerts from Flask API
RESPONSE=$(curl -s "http://localhost:5000/alerts?hours=1" 2>/dev/null)

if [ -n "$RESPONSE" ]; then
    # Append to daily alerts file
    echo "{\"collected_at\": \"$TIMESTAMP\", \"alerts\": $RESPONSE}" >> "$ALERTS_FILE"
fi
```

### 4.4 Log Backup Script

Save as `edge-kit/scripts/backup_logs.sh`:

```bash
#!/bin/bash
# Daily log backup script - run via cron at midnight

BACKUP_DIR="/media/awais/New Volume/hackathon/edge-kit/data/backups"
LOG_DIR="/media/awais/New Volume/hackathon/edge-kit/data/logs"
METRICS_DIR="/media/awais/New Volume/hackathon/edge-kit/data/metrics"
DATE=$(date +"%Y-%m-%d")

# Create backup archive
tar -czf "$BACKUP_DIR/backup_$DATE.tar.gz" \
    "$LOG_DIR" \
    "$METRICS_DIR" \
    2>/dev/null

# Clean up old backups (keep 14 days)
find "$BACKUP_DIR" -name "backup_*.tar.gz" -mtime +14 -delete

# Clean up old metrics files (keep 30 days)
find "$METRICS_DIR" -name "metrics_*.csv" -mtime +30 -delete

echo "Backup completed: $DATE"
```

---

## 5. Cron Jobs Setup

Add the following cron jobs for automated monitoring:

```bash
# Edit crontab
crontab -e

# Add these lines:
# Health check every 5 minutes
*/5 * * * * /media/awais/New\ Volume/hackathon/edge-kit/scripts/health_check.sh

# Metrics collection every minute
* * * * * /media/awais/New\ Volume/hackathon/edge-kit/scripts/collect_metrics.sh

# Alert collection every 5 minutes
*/5 * * * * /media/awais/New\ Volume/hackathon/edge-kit/scripts/collect_alerts.sh

# Daily backup at midnight
0 0 * * * /media/awais/New\ Volume/hackathon/edge-kit/scripts/backup_logs.sh
```

---

## 6. Daily Monitoring Routine

### 6.1 Morning Check (09:00)

```bash
# 1. Check container status
docker compose ps

# 2. Check overnight alerts
cat /media/awais/New\ Volume/hackathon/edge-kit/data/alerts/alerts_$(date +%Y-%m-%d).json | jq '.alerts.data[] | select(.severity == "CRITICAL" or .severity == "ERROR")'

# 3. Check metrics summary
tail -100 /media/awais/New\ Volume/hackathon/edge-kit/data/metrics/metrics_$(date +%Y-%m-%d).csv | awk -F',' '{sum+=$2; count++} END {print "Avg CPU: " sum/count "%"}'

# 4. Verify ROS2 nodes
docker compose exec edge-kit bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 node list"

# 5. Check system resources
docker stats edge-kit-ros2 --no-stream
```

### 6.2 Midday Check (13:00)

```bash
# 1. Quick health check
curl -s http://localhost:5000/status | jq

# 2. Check for any new alerts
curl -s "http://localhost:5000/alerts?hours=4" | jq '.data | length'

# 3. Verify publishing rate
docker compose exec edge-kit bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout 10 ros2 topic hz /health_status"
```

### 6.3 Evening Check (18:00)

```bash
# 1. Full status check
./edge-kit/scripts/health_check.sh

# 2. Review daily metrics
wc -l /media/awais/New\ Volume/hackathon/edge-kit/data/metrics/metrics_$(date +%Y-%m-%d).csv

# 3. Check disk usage
du -sh /media/awais/New\ Volume/hackathon/edge-kit/data/

# 4. Generate daily summary
echo "=== Day Summary ==="
echo "Total metrics collected: $(wc -l < /media/awais/New\ Volume/hackathon/edge-kit/data/metrics/metrics_$(date +%Y-%m-%d).csv)"
echo "Total alerts: $(cat /media/awais/New\ Volume/hackathon/edge-kit/data/alerts/alerts_$(date +%Y-%m-%d).json 2>/dev/null | grep -c alert_id || echo 0)"
```

---

## 7. What to Look For

### 7.1 Success Indicators

- [ ] Container running continuously without restarts
- [ ] ROS2 nodes active and publishing at expected frequency (1Hz)
- [ ] No CRITICAL alerts
- [ ] CPU usage < 10% average
- [ ] Memory usage stable (no memory leaks)
- [ ] Disk usage not growing excessively
- [ ] Flask API responding with 200 status

### 7.2 Warning Signs

- Container restarts (check `docker compose logs`)
- Increasing memory usage over time
- ROS2 node crashes or disconnections
- Publishing frequency drops below 0.9Hz
- Multiple WARNING alerts in short periods
- Flask API timeouts or errors

### 7.3 Critical Issues (Require Immediate Action)

- Container stops and doesn't restart
- All ROS2 nodes crash
- CRITICAL alerts for temperature
- Disk space > 90%
- Memory usage > 90%

---

## 8. Troubleshooting Commands

### 8.1 Container Management

```bash
# Check container logs
docker compose logs -f edge-kit --tail=100

# Restart container
docker compose restart edge-kit

# Stop and start
docker compose stop edge-kit
docker compose start edge-kit

# Rebuild and restart
docker compose up -d --build edge-kit

# Enter container for debugging
docker compose exec edge-kit bash
```

### 8.2 ROS2 Debugging

```bash
# Inside container
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Check node info
ros2 node info /health_monitor

# Check topic info
ros2 topic info /health_status

# Echo messages
ros2 topic echo /health_status --once

# Check service
ros2 service call /safe_shutdown edge_health_interfaces/srv/SafeShutdown "{reason: 'MANUAL', requested_by: 'operator', timeout: 30.0}"
```

### 8.3 Log Analysis

```bash
# Search for errors in container logs
docker compose logs edge-kit 2>&1 | grep -i error

# Check health check history
tail -50 /media/awais/New\ Volume/hackathon/edge-kit/data/logs/health_checks.log

# Analyze metrics trends
awk -F',' 'NR>1 {print $1, $2}' /media/awais/New\ Volume/hackathon/edge-kit/data/metrics/metrics_$(date +%Y-%m-%d).csv
```

---

## 9. Daily Checklist

### Day 1 (Deployment Day)
- [ ] Deploy container
- [ ] Verify all ROS2 nodes running
- [ ] Verify Flask API accessible
- [ ] Set up cron jobs
- [ ] Document initial metrics baseline
- [ ] Notes: _________________________________

### Day 2
- [ ] Morning check complete
- [ ] Midday check complete
- [ ] Evening check complete
- [ ] No critical alerts
- [ ] Container uptime: _______ hours
- [ ] Notes: _________________________________

### Day 3
- [ ] Morning check complete
- [ ] Midday check complete
- [ ] Evening check complete
- [ ] No critical alerts
- [ ] Container uptime: _______ hours
- [ ] Notes: _________________________________

### Day 4
- [ ] Morning check complete
- [ ] Midday check complete
- [ ] Evening check complete
- [ ] No critical alerts
- [ ] Container uptime: _______ hours
- [ ] Notes: _________________________________

### Day 5
- [ ] Morning check complete
- [ ] Midday check complete
- [ ] Evening check complete
- [ ] No critical alerts
- [ ] Container uptime: _______ hours
- [ ] Notes: _________________________________

### Day 6
- [ ] Morning check complete
- [ ] Midday check complete
- [ ] Evening check complete
- [ ] No critical alerts
- [ ] Container uptime: _______ hours
- [ ] Notes: _________________________________

### Day 7 (Final Day)
- [ ] Morning check complete
- [ ] Midday check complete
- [ ] Final metrics collection
- [ ] Generate 7-day report
- [ ] Container uptime: _______ hours
- [ ] Notes: _________________________________

---

## 10. 7-Day Test Completion

### 10.1 Generate Final Report

```bash
# Run final report script
./edge-kit/scripts/generate_7day_report.sh
```

### 10.2 Success Criteria for T060

The 7-day test is **PASSED** if:

1. **Uptime**: Container ran continuously for 168 hours (7 days) with < 3 restarts
2. **Reliability**: Health metrics published at > 99% of expected rate (1Hz)
3. **Stability**: No memory leaks (memory usage variance < 10%)
4. **Alerts**: Zero unresolved CRITICAL alerts
5. **Performance**: Average CPU usage < 10%
6. **Data Integrity**: All metrics collected without gaps > 5 minutes

### 10.3 Mark T060 Complete

After successful 7-day test:

```bash
# Update tasks.md
sed -i 's/\[ \] T060/[x] T060/' /media/awais/New\ Volume/hackathon/specs/001-physical-ai-monitoring/tasks.md

# Commit the change
cd /media/awais/New\ Volume/hackathon
git add specs/001-physical-ai-monitoring/tasks.md
git commit -m "feat: Complete 7-day reliability test (T060)

- Continuous operation for 168 hours verified
- All reliability metrics met
- Edge health monitoring system production-ready

🤖 Generated with Claude Code"
```

---

## 11. Quick Reference Commands

```bash
# Start deployment
docker compose up -d edge-kit

# Check status
docker compose ps
curl http://localhost:5000/status | jq

# View logs
docker compose logs -f edge-kit --tail=50

# Restart
docker compose restart edge-kit

# Stop
docker compose down

# Enter container
docker compose exec edge-kit bash

# Check ROS2 nodes (inside container)
ros2 node list
ros2 topic list
ros2 topic echo /health_status --once

# Health check
./edge-kit/scripts/health_check.sh

# Backup now
./edge-kit/scripts/backup_logs.sh
```

---

## Appendix A: File Locations

| File | Location |
|------|----------|
| Dockerfile | `edge-kit/Dockerfile` |
| docker-compose.yml | `docker/docker-compose.yml` |
| Health check script | `edge-kit/scripts/health_check.sh` |
| Metrics script | `edge-kit/scripts/collect_metrics.sh` |
| Alerts script | `edge-kit/scripts/collect_alerts.sh` |
| Backup script | `edge-kit/scripts/backup_logs.sh` |
| Daily logs | `edge-kit/data/logs/` |
| Metrics CSV | `edge-kit/data/metrics/` |
| Alerts JSON | `edge-kit/data/alerts/` |
| Backups | `edge-kit/data/backups/` |

---

**Document Version**: 1.0
**Last Updated**: 2025-12-12
**Author**: Physical AI Edge Kit Team
