# Performance Benchmarks for Edge Health Package

## Overview
This document outlines the performance benchmarks and optimization guidelines for the edge_health package.

## Performance Goals

### Real-time Health Monitoring
- **Target Publishing Rate**: 1 Hz (once per second)
- **Actual Publishing Rate**: Configurable (0.1 Hz to 10 Hz)
- **Reliability**: 99% message delivery rate
- **Latency**: < 100ms from metric collection to topic publication

### Threshold Monitoring
- **Response Time**: Detect and publish alerts within 5 seconds of threshold violation
- **Accuracy**: 100% detection of threshold violations
- **Hysteresis**: Prevent alert flapping with 5% threshold buffer

### Safe Shutdown Service
- **Response Time**: Initiate shutdown within 1 second of service call
- **Completion Time**: Complete shutdown sequence within 30 seconds (configurable)
- **Reliability**: 99.9% successful shutdown completion

## Resource Usage

### CPU Usage
- **Idle**: < 1% CPU usage when not actively monitoring
- **Active Monitoring**: < 5% CPU usage at 1 Hz polling
- **High Frequency**: < 15% CPU usage at 10 Hz polling

### Memory Usage
- **Startup**: < 50 MB RAM usage per node
- **Runtime**: < 60 MB RAM usage per node (after garbage collection)
- **Peak**: < 75 MB RAM usage under load

### Network Usage
- **Health Status Topic**: ~1 KB per message at 1 Hz = ~8.6 KB/min
- **Alerts Topic**: Event-driven, average < 0.1 Hz = < 0.8 KB/min
- **Total**: < 10 KB/min under normal conditions

## Benchmarking Scripts

### CPU and Memory Monitoring
```bash
# Monitor resource usage
watch -n 1 'ps aux | grep -E "(health_monitor|alert_publisher|safe_shutdown)"'

# Or use system tools
top -p $(pgrep -f health_monitor)
```

### Message Rate Verification
```bash
# Check message rate
ros2 topic hz /health_status

# Monitor message delivery
ros2 topic echo /health_status --field timestamp | ts '%.s'
```

### Stress Testing
```bash
# Simulate high CPU usage to trigger alerts
stress --cpu 8 --timeout 60s

# Monitor response time
time ros2 service call /safe_shutdown edge_health_interfaces/srv/SafeShutdown "{reason: 'test', requested_by: 'test', timeout: 30.0}"
```

## Optimization Strategies

### Configuration-Based Optimization
1. **Adjust Publish Frequency**: Higher frequency = more accuracy, lower frequency = less resource usage
2. **Disable Unnecessary Metrics**: Only monitor required metrics
3. **Set Appropriate Thresholds**: Avoid overly sensitive thresholds that cause excessive alerts

### Hardware-Specific Optimizations
1. **CPU Affinity**: Bind monitoring processes to specific CPU cores
2. **Memory Management**: Use memory pools for frequent allocations
3. **I/O Optimization**: Batch operations where possible

### Code-Level Optimizations
1. **Efficient Data Structures**: Use appropriate data structures for metric storage
2. **Minimize Allocations**: Reuse objects where possible
3. **Asynchronous Operations**: Use async operations for non-critical tasks

## Performance Testing Scenarios

### Baseline Test
- **Duration**: 1 hour
- **Metrics**: CPU, memory, message delivery rate
- **Expected**: Stable resource usage, consistent message delivery

### Load Test
- **Duration**: 30 minutes
- **Load**: 80-90% CPU utilization
- **Metrics**: Response time, alert generation, system stability
- **Expected**: No missed alerts, stable operation

### Stress Test
- **Duration**: 10 minutes
- **Load**: 95-100% CPU utilization
- **Metrics**: System stability, shutdown response
- **Expected**: Proper alert generation, graceful degradation

### Long-term Stability Test
- **Duration**: 7 days
- **Metrics**: Memory leaks, performance degradation, message delivery
- **Expected**: No memory leaks, stable performance

## Monitoring Commands

### Real-time Performance Monitoring
```bash
# Monitor all edge_health nodes
ros2 run plotjuggler plotjuggler --plugins_directory /opt/ros/humble/lib/x86_64-linux-gnu/plotjuggler/

# System resource monitoring
htop
iotop
iftop

# ROS2 specific monitoring
ros2 run topic_tools relay /health_status /health_status_monitored
```

### Performance Analysis
```bash
# Profile Python code
python3 -m cProfile -o profile.stats edge_health/health_monitor.py

# Analyze profile
pyprof2calltree -i profile.stats -k

# Memory profiling
pip3 install memory-profiler
python3 -m memory_profiler edge_health/health_monitor.py
```

## Performance Metrics Dashboard

The REST API provides performance metrics endpoints:
- `/health/aggregate` - Aggregated performance statistics
- `/status` - Overall system performance summary
- `/health/history` - Historical performance data

## Performance Tuning Guide

### For Resource-Constrained Systems
1. Increase `publish_frequency` to reduce polling (e.g., 2-5 seconds)
2. Disable non-critical metrics in `enabled_metrics`
3. Reduce logging level to ERROR or WARNING
4. Use simpler threshold monitoring (disable hysteresis if not needed)

### For High-Performance Systems
1. Decrease `publish_frequency` for more frequent monitoring (e.g., 0.1-0.5 seconds)
2. Enable all metrics including hardware-specific ones
3. Use detailed logging (DEBUG level)
4. Implement advanced alerting with hysteresis

## Performance Regression Testing

### Automated Tests
```bash
# Run performance regression tests
python3 -m pytest test/performance_tests.py -v

# Benchmark comparison
python3 benchmark_runner.py --baseline=previous_run --current=current_run
```

### Continuous Monitoring
- Integrate performance tests into CI/CD pipeline
- Monitor performance metrics in production
- Set up alerts for performance degradation