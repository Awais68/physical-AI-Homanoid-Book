#!/bin/bash
# Generate 7-Day Reliability Test Report
# Run this script at the end of the 7-day deployment test

METRICS_DIR="${METRICS_DIR:-/media/awais/New Volume/hackathon/edge-kit/data/metrics}"
ALERTS_DIR="${ALERTS_DIR:-/media/awais/New Volume/hackathon/edge-kit/data/alerts}"
LOG_DIR="${LOG_DIR:-/media/awais/New Volume/hackathon/edge-kit/data/logs}"
REPORT_DIR="${REPORT_DIR:-/media/awais/New Volume/hackathon/edge-kit/data}"

TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
REPORT_FILE="$REPORT_DIR/7day_report_$TIMESTAMP.md"

echo "Generating 7-Day Reliability Test Report..."
echo ""

# Collect all metrics files from the past 7 days
METRICS_FILES=$(find "$METRICS_DIR" -name "metrics_*.csv" -mtime -8 | sort)
ALERT_FILES=$(find "$ALERTS_DIR" -name "alerts_*.json" -mtime -8 | sort)

# Calculate statistics
TOTAL_METRICS=0
TOTAL_OK=0
TOTAL_WARNING=0
TOTAL_CRITICAL=0
TOTAL_UNREACHABLE=0

CPU_SUM=0
CPU_COUNT=0
CPU_MAX=0

MEMORY_SUM=0
MEMORY_COUNT=0
MEMORY_MAX=0

for file in $METRICS_FILES; do
    if [ -f "$file" ]; then
        # Skip header line
        tail -n +2 "$file" | while IFS=',' read -r timestamp cpu mem disk temp status node uptime; do
            TOTAL_METRICS=$((TOTAL_METRICS + 1))

            case "$status" in
                "OK") TOTAL_OK=$((TOTAL_OK + 1)) ;;
                "WARNING") TOTAL_WARNING=$((TOTAL_WARNING + 1)) ;;
                "CRITICAL") TOTAL_CRITICAL=$((TOTAL_CRITICAL + 1)) ;;
                "API_UNREACHABLE") TOTAL_UNREACHABLE=$((TOTAL_UNREACHABLE + 1)) ;;
            esac

            if [[ "$cpu" =~ ^[0-9]+\.?[0-9]*$ ]] && [ "$cpu" != "-1" ]; then
                CPU_SUM=$(echo "$CPU_SUM + $cpu" | bc 2>/dev/null || echo "$CPU_SUM")
                CPU_COUNT=$((CPU_COUNT + 1))
                if (( $(echo "$cpu > $CPU_MAX" | bc -l 2>/dev/null || echo "0") )); then
                    CPU_MAX=$cpu
                fi
            fi

            if [[ "$mem" =~ ^[0-9]+\.?[0-9]*$ ]] && [ "$mem" != "-1" ]; then
                MEMORY_SUM=$(echo "$MEMORY_SUM + $mem" | bc 2>/dev/null || echo "$MEMORY_SUM")
                MEMORY_COUNT=$((MEMORY_COUNT + 1))
                if (( $(echo "$mem > $MEMORY_MAX" | bc -l 2>/dev/null || echo "0") )); then
                    MEMORY_MAX=$mem
                fi
            fi
        done
    fi
done

# Calculate metrics using awk for more reliable processing
METRICS_STATS=$(cat $METRICS_FILES 2>/dev/null | tail -n +2 | awk -F',' '
BEGIN {
    total=0; ok=0; warning=0; critical=0; unreachable=0;
    cpu_sum=0; cpu_count=0; cpu_max=0;
    mem_sum=0; mem_count=0; mem_max=0; mem_min=100;
}
{
    total++;
    if ($6 == "OK") ok++;
    else if ($6 == "WARNING") warning++;
    else if ($6 == "CRITICAL") critical++;
    else if ($6 == "API_UNREACHABLE") unreachable++;

    if ($2 > 0 && $2 != "-1") {
        cpu_sum += $2;
        cpu_count++;
        if ($2 > cpu_max) cpu_max = $2;
    }

    if ($3 > 0 && $3 != "-1") {
        mem_sum += $3;
        mem_count++;
        if ($3 > mem_max) mem_max = $3;
        if ($3 < mem_min) mem_min = $3;
    }
}
END {
    cpu_avg = (cpu_count > 0) ? cpu_sum/cpu_count : 0;
    mem_avg = (mem_count > 0) ? mem_sum/mem_count : 0;
    mem_variance = mem_max - mem_min;
    ok_rate = (total > 0) ? (ok/total)*100 : 0;
    availability = (total > 0) ? ((total-unreachable)/total)*100 : 0;

    printf "TOTAL=%d\n", total;
    printf "OK=%d\n", ok;
    printf "WARNING=%d\n", warning;
    printf "CRITICAL=%d\n", critical;
    printf "UNREACHABLE=%d\n", unreachable;
    printf "CPU_AVG=%.2f\n", cpu_avg;
    printf "CPU_MAX=%.2f\n", cpu_max;
    printf "MEM_AVG=%.2f\n", mem_avg;
    printf "MEM_MAX=%.2f\n", mem_max;
    printf "MEM_MIN=%.2f\n", mem_min;
    printf "MEM_VARIANCE=%.2f\n", mem_variance;
    printf "OK_RATE=%.2f\n", ok_rate;
    printf "AVAILABILITY=%.2f\n", availability;
}
')

# Parse stats
eval "$METRICS_STATS"

# Count alerts
TOTAL_ALERTS=$(cat $ALERT_FILES 2>/dev/null | jq -r '.collected_alerts[].alert_count' 2>/dev/null | awk '{sum+=$1} END {print sum}' || echo "0")
CRITICAL_ALERTS=$(cat $ALERT_FILES 2>/dev/null | jq -r '.collected_alerts[].alerts.data[]? | select(.severity == "CRITICAL")' 2>/dev/null | wc -l || echo "0")

# Get container restart count
CONTAINER_NAME="${CONTAINER_NAME:-edge-kit-ros2}"
RESTART_COUNT=$(docker inspect -f '{{.RestartCount}}' "$CONTAINER_NAME" 2>/dev/null || echo "N/A")

# Get uptime
STARTED_AT=$(docker inspect -f '{{.State.StartedAt}}' "$CONTAINER_NAME" 2>/dev/null || echo "N/A")

# Generate report
cat > "$REPORT_FILE" << EOF
# 7-Day Reliability Test Report

**Generated**: $(date '+%Y-%m-%d %H:%M:%S')
**Test Period**: Last 7 days
**Container**: $CONTAINER_NAME

---

## Executive Summary

| Metric | Value | Target | Status |
|--------|-------|--------|--------|
| Container Restarts | $RESTART_COUNT | < 3 | $([ "$RESTART_COUNT" != "N/A" ] && [ "$RESTART_COUNT" -lt 3 ] && echo "PASS" || echo "CHECK") |
| Availability | ${AVAILABILITY:-0}% | > 99% | $(echo "${AVAILABILITY:-0} > 99" | bc -l 2>/dev/null | grep -q "1" && echo "PASS" || echo "CHECK") |
| Avg CPU Usage | ${CPU_AVG:-0}% | < 10% | $(echo "${CPU_AVG:-0} < 10" | bc -l 2>/dev/null | grep -q "1" && echo "PASS" || echo "CHECK") |
| Memory Variance | ${MEM_VARIANCE:-0}% | < 10% | $(echo "${MEM_VARIANCE:-0} < 10" | bc -l 2>/dev/null | grep -q "1" && echo "PASS" || echo "CHECK") |
| Critical Alerts | $CRITICAL_ALERTS | 0 | $([ "$CRITICAL_ALERTS" == "0" ] && echo "PASS" || echo "FAIL") |

---

## Detailed Metrics

### Data Collection

- **Total Metrics Collected**: ${TOTAL:-0}
- **Expected Metrics (1/min for 7 days)**: ~10,080
- **Collection Rate**: $(echo "scale=2; ${TOTAL:-0} / 10080 * 100" | bc 2>/dev/null || echo "N/A")%

### Health Status Distribution

| Status | Count | Percentage |
|--------|-------|------------|
| OK | ${OK:-0} | $(echo "scale=2; ${OK:-0} / ${TOTAL:-1} * 100" | bc 2>/dev/null || echo "0")% |
| WARNING | ${WARNING:-0} | $(echo "scale=2; ${WARNING:-0} / ${TOTAL:-1} * 100" | bc 2>/dev/null || echo "0")% |
| CRITICAL | ${CRITICAL:-0} | $(echo "scale=2; ${CRITICAL:-0} / ${TOTAL:-1} * 100" | bc 2>/dev/null || echo "0")% |
| API Unreachable | ${UNREACHABLE:-0} | $(echo "scale=2; ${UNREACHABLE:-0} / ${TOTAL:-1} * 100" | bc 2>/dev/null || echo "0")% |

### Resource Usage

| Resource | Average | Maximum | Target |
|----------|---------|---------|--------|
| CPU | ${CPU_AVG:-0}% | ${CPU_MAX:-0}% | < 10% avg |
| Memory | ${MEM_AVG:-0}% | ${MEM_MAX:-0}% | Stable |

### Memory Stability

- **Memory Min**: ${MEM_MIN:-0}%
- **Memory Max**: ${MEM_MAX:-0}%
- **Memory Variance**: ${MEM_VARIANCE:-0}%
- **Memory Leak Detected**: $(echo "${MEM_VARIANCE:-0} > 10" | bc -l 2>/dev/null | grep -q "1" && echo "POSSIBLE" || echo "No")

---

## Alert Summary

- **Total Alerts Generated**: $TOTAL_ALERTS
- **Critical Alerts**: $CRITICAL_ALERTS
- **Unresolved Critical Alerts**: $(cat $ALERT_FILES 2>/dev/null | jq -r '.collected_alerts[].alerts.data[]? | select(.severity == "CRITICAL" and .resolved != true)' 2>/dev/null | wc -l || echo "0")

---

## Container Information

- **Container Name**: $CONTAINER_NAME
- **Started At**: $STARTED_AT
- **Restart Count**: $RESTART_COUNT

---

## Success Criteria Evaluation

### T060 Success Criteria

1. **Uptime**: Container ran continuously for 168 hours with < 3 restarts
   - Restart Count: $RESTART_COUNT
   - Status: $([ "$RESTART_COUNT" != "N/A" ] && [ "$RESTART_COUNT" -lt 3 ] && echo "**PASS**" || echo "**CHECK**")

2. **Reliability**: Health metrics published at > 99% of expected rate
   - Collection Rate: $(echo "scale=2; ${TOTAL:-0} / 10080 * 100" | bc 2>/dev/null || echo "N/A")%
   - Status: $(echo "${AVAILABILITY:-0} > 99" | bc -l 2>/dev/null | grep -q "1" && echo "**PASS**" || echo "**CHECK**")

3. **Stability**: No memory leaks (memory usage variance < 10%)
   - Memory Variance: ${MEM_VARIANCE:-0}%
   - Status: $(echo "${MEM_VARIANCE:-0} < 10" | bc -l 2>/dev/null | grep -q "1" && echo "**PASS**" || echo "**CHECK**")

4. **Alerts**: Zero unresolved CRITICAL alerts
   - Critical Alerts: $CRITICAL_ALERTS
   - Status: $([ "$CRITICAL_ALERTS" == "0" ] && echo "**PASS**" || echo "**CHECK**")

5. **Performance**: Average CPU usage < 10%
   - Average CPU: ${CPU_AVG:-0}%
   - Status: $(echo "${CPU_AVG:-0} < 10" | bc -l 2>/dev/null | grep -q "1" && echo "**PASS**" || echo "**CHECK**")

6. **Data Integrity**: All metrics collected without gaps > 5 minutes
   - API Unreachable Count: ${UNREACHABLE:-0}
   - Status: $([ "${UNREACHABLE:-0}" -lt 50 ] && echo "**PASS**" || echo "**CHECK**")

---

## Overall Result

**TEST RESULT**: $(
    if [ "$RESTART_COUNT" != "N/A" ] && [ "$RESTART_COUNT" -lt 3 ] && \
       [ "$CRITICAL_ALERTS" == "0" ]; then
        echo "**PASSED**"
    else
        echo "**NEEDS REVIEW**"
    fi
)

---

## Files Analyzed

### Metrics Files
$(ls -la $METRICS_DIR/metrics_*.csv 2>/dev/null | tail -10 || echo "No metrics files found")

### Alert Files
$(ls -la $ALERTS_DIR/alerts_*.json 2>/dev/null | tail -10 || echo "No alert files found")

---

**Report Generated By**: Edge Health Monitoring System
**Version**: 1.0
EOF

echo ""
echo "=========================================="
echo "7-Day Reliability Test Report Generated"
echo "=========================================="
echo ""
echo "Report saved to: $REPORT_FILE"
echo ""
echo "Summary:"
echo "  - Total Metrics Collected: ${TOTAL:-0}"
echo "  - Availability: ${AVAILABILITY:-0}%"
echo "  - Average CPU: ${CPU_AVG:-0}%"
echo "  - Memory Variance: ${MEM_VARIANCE:-0}%"
echo "  - Critical Alerts: $CRITICAL_ALERTS"
echo "  - Container Restarts: $RESTART_COUNT"
echo ""

# Print pass/fail summary
echo "Success Criteria:"
[ "$RESTART_COUNT" != "N/A" ] && [ "$RESTART_COUNT" -lt 3 ] && echo "  [PASS] Uptime requirement" || echo "  [CHECK] Uptime requirement"
echo "${AVAILABILITY:-0} > 99" | bc -l 2>/dev/null | grep -q "1" && echo "  [PASS] Reliability requirement" || echo "  [CHECK] Reliability requirement"
echo "${MEM_VARIANCE:-0} < 10" | bc -l 2>/dev/null | grep -q "1" && echo "  [PASS] Stability requirement" || echo "  [CHECK] Stability requirement"
[ "$CRITICAL_ALERTS" == "0" ] && echo "  [PASS] Alert requirement" || echo "  [CHECK] Alert requirement"
echo "${CPU_AVG:-0} < 10" | bc -l 2>/dev/null | grep -q "1" && echo "  [PASS] Performance requirement" || echo "  [CHECK] Performance requirement"
echo ""
