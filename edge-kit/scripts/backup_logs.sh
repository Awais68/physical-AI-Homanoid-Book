#!/bin/bash
# Daily log backup script - run via cron at midnight
# Creates compressed backups and cleans up old files

BACKUP_DIR="${BACKUP_DIR:-/media/awais/New Volume/hackathon/edge-kit/data/backups}"
LOG_DIR="${LOG_DIR:-/media/awais/New Volume/hackathon/edge-kit/data/logs}"
METRICS_DIR="${METRICS_DIR:-/media/awais/New Volume/hackathon/edge-kit/data/metrics}"
ALERTS_DIR="${ALERTS_DIR:-/media/awais/New Volume/hackathon/edge-kit/data/alerts}"

DATE=$(date +"%Y-%m-%d")
TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")

# Retention periods
LOG_RETENTION_DAYS=${LOG_RETENTION_DAYS:-14}
METRICS_RETENTION_DAYS=${METRICS_RETENTION_DAYS:-30}
BACKUP_RETENTION_DAYS=${BACKUP_RETENTION_DAYS:-14}

echo "[$(date '+%Y-%m-%d %H:%M:%S')] Starting backup process..."

mkdir -p "$BACKUP_DIR"

# Create backup archive
BACKUP_FILE="$BACKUP_DIR/backup_$DATE.tar.gz"

echo "[$(date '+%Y-%m-%d %H:%M:%S')] Creating backup archive: $BACKUP_FILE"

tar -czf "$BACKUP_FILE" \
    -C "$(dirname "$LOG_DIR")" "$(basename "$LOG_DIR")" \
    -C "$(dirname "$METRICS_DIR")" "$(basename "$METRICS_DIR")" \
    -C "$(dirname "$ALERTS_DIR")" "$(basename "$ALERTS_DIR")" \
    2>/dev/null

if [ $? -eq 0 ]; then
    BACKUP_SIZE=$(du -h "$BACKUP_FILE" | cut -f1)
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Backup created successfully: $BACKUP_FILE ($BACKUP_SIZE)"
else
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] WARNING: Backup may be incomplete"
fi

# Clean up old backups
echo "[$(date '+%Y-%m-%d %H:%M:%S')] Cleaning up backups older than $BACKUP_RETENTION_DAYS days..."
DELETED_BACKUPS=$(find "$BACKUP_DIR" -name "backup_*.tar.gz" -mtime +$BACKUP_RETENTION_DAYS -delete -print | wc -l)
echo "[$(date '+%Y-%m-%d %H:%M:%S')] Deleted $DELETED_BACKUPS old backup files"

# Clean up old metrics files
echo "[$(date '+%Y-%m-%d %H:%M:%S')] Cleaning up metrics older than $METRICS_RETENTION_DAYS days..."
DELETED_METRICS=$(find "$METRICS_DIR" -name "metrics_*.csv" -mtime +$METRICS_RETENTION_DAYS -delete -print | wc -l)
echo "[$(date '+%Y-%m-%d %H:%M:%S')] Deleted $DELETED_METRICS old metrics files"

# Clean up old alert files
echo "[$(date '+%Y-%m-%d %H:%M:%S')] Cleaning up alerts older than $METRICS_RETENTION_DAYS days..."
DELETED_ALERTS=$(find "$ALERTS_DIR" -name "alerts_*.json" -mtime +$METRICS_RETENTION_DAYS -delete -print | wc -l)
echo "[$(date '+%Y-%m-%d %H:%M:%S')] Deleted $DELETED_ALERTS old alert files"

# Clean up old log files
echo "[$(date '+%Y-%m-%d %H:%M:%S')] Cleaning up logs older than $LOG_RETENTION_DAYS days..."
DELETED_LOGS=$(find "$LOG_DIR" -name "*.log" -mtime +$LOG_RETENTION_DAYS -delete -print | wc -l)
echo "[$(date '+%Y-%m-%d %H:%M:%S')] Deleted $DELETED_LOGS old log files"

# Report disk usage
echo "[$(date '+%Y-%m-%d %H:%M:%S')] Current disk usage:"
du -sh "$LOG_DIR" "$METRICS_DIR" "$ALERTS_DIR" "$BACKUP_DIR" 2>/dev/null

echo "[$(date '+%Y-%m-%d %H:%M:%S')] Backup process completed"

# Log summary
echo "Backup completed: $DATE - Archive: $BACKUP_FILE" >> "$BACKUP_DIR/backup_history.log"
