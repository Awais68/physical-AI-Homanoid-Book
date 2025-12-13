from flask import Flask, jsonify
import psutil
import time
import yaml
from pathlib import Path

app = Flask(__name__)

# Load configuration
config_path = Path("../config/edge_health_config.yaml")  # When running from api directory
if not config_path.exists():
    config_path = Path("config/edge_health_config.yaml")  # When running from edge-kit directory
if not config_path.exists():
    config_path = Path("../edge-kit/config/edge_health_config.yaml")  # When running from project root
if not config_path.exists():
    # Default configuration if file not found
    config = {
        'health_monitor': {
            'cpu_threshold_percent': 80.0,
            'memory_threshold_percent': 85.0,
            'disk_threshold_percent': 90.0
        }
    }
else:
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)

@app.route('/health', methods=['GET'])
def get_health():
    """Get the current health status of the edge device"""
    try:
        cpu_percent = psutil.cpu_percent(interval=1)
        memory_percent = psutil.virtual_memory().percent
        disk_percent = psutil.disk_usage('/').percent

        # Determine status based on thresholds
        cpu_threshold = config['health_monitor']['cpu_threshold_percent']
        memory_threshold = config['health_monitor']['memory_threshold_percent']
        disk_threshold = config['health_monitor']['disk_threshold_percent']

        status = 'OK'
        issues = []

        if cpu_percent > cpu_threshold:
            status = 'WARNING'
            issues.append(f"High CPU usage: {cpu_percent}%")
        if memory_percent > memory_threshold:
            status = 'WARNING'
            issues.append(f"High memory usage: {memory_percent}%")
        if disk_percent > disk_threshold:
            status = 'WARNING'
            issues.append(f"High disk usage: {disk_percent}%")

        health_data = {
            'timestamp': time.time(),
            'status': status,
            'cpu_percent': cpu_percent,
            'memory_percent': memory_percent,
            'disk_percent': disk_percent,
            'issues': issues,
            'config': {
                'cpu_threshold_percent': cpu_threshold,
                'memory_threshold_percent': memory_threshold,
                'disk_threshold_percent': disk_threshold
            }
        }

        return jsonify(health_data)
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/health/system', methods=['GET'])
def get_system_info():
    """Get detailed system information"""
    try:
        system_info = {
            'cpu_count': psutil.cpu_count(),
            'cpu_freq': psutil.cpu_freq()._asdict() if psutil.cpu_freq() else None,
            'memory_total': psutil.virtual_memory().total,
            'memory_available': psutil.virtual_memory().available,
            'disk_total': psutil.disk_usage('/').total,
            'disk_free': psutil.disk_usage('/').free,
            'boot_time': psutil.boot_time(),
            'uptime_seconds': time.time() - psutil.boot_time()
        }

        return jsonify(system_info)
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/status', methods=['GET'])
def get_status():
    """Simple status endpoint"""
    return jsonify({'status': 'Edge Health API is running', 'timestamp': time.time()})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)