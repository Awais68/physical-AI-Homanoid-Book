"""REST API for external health status access."""

from flask import Flask, jsonify, request
import json
import os
from datetime import datetime, timedelta
from threading import Lock
import logging

app = Flask(__name__)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# In-memory storage for health data (in production, this would be a database)
health_data_store = []
alerts_store = []
recent_metrics = {}
store_lock = Lock()

# Configuration
API_VERSION = "1.0.0"
MAX_STORED_RECORDS = 1000  # Maximum number of records to keep in memory


@app.route('/health', methods=['GET'])
def get_health_status():
    """Get the current health status of the system."""
    try:
        with store_lock:
            if recent_metrics:
                # Return the most recent metrics
                latest_metrics = recent_metrics.copy()
                latest_metrics['timestamp'] = datetime.fromtimestamp(latest_metrics.get('timestamp', 0)).isoformat()
                return jsonify(latest_metrics), 200
            else:
                # Return a default healthy status if no data is available
                default_status = {
                    'node_name': 'api_gateway',
                    'timestamp': datetime.now().isoformat(),
                    'cpu_percent': -1.0,
                    'memory_percent': -1.0,
                    'disk_percent': -1.0,
                    'temperature': None,
                    'status': 'UNKNOWN',
                    'metrics': {}
                }
                return jsonify(default_status), 200
    except Exception as e:
        logger.error(f"Error getting health status: {e}")
        return jsonify({'error': 'Failed to get health status', 'details': str(e)}), 500


@app.route('/health/history', methods=['GET'])
def get_health_history():
    """Get historical health data."""
    try:
        limit = int(request.args.get('limit', 100))
        limit = min(limit, MAX_STORED_RECORDS)  # Cap the limit

        with store_lock:
            # Return the most recent records up to the limit
            recent_data = health_data_store[-limit:] if len(health_data_store) >= limit else health_data_store[:]

            # Convert timestamps to ISO format
            for record in recent_data:
                if 'timestamp' in record:
                    record['timestamp'] = datetime.fromtimestamp(record['timestamp']).isoformat()

            return jsonify({
                'count': len(recent_data),
                'data': recent_data
            }), 200
    except Exception as e:
        logger.error(f"Error getting health history: {e}")
        return jsonify({'error': 'Failed to get health history', 'details': str(e)}), 500


@app.route('/alerts', methods=['GET'])
def get_alerts():
    """Get current and recent alerts."""
    try:
        limit = int(request.args.get('limit', 50))
        severity = request.args.get('severity', None)  # Filter by severity
        hours = int(request.args.get('hours', 24))  # Filter by time window

        time_threshold = datetime.now() - timedelta(hours=hours)

        with store_lock:
            # Filter alerts by time window
            recent_alerts = [
                alert for alert in alerts_store
                if datetime.fromtimestamp(alert.get('timestamp', 0)) >= time_threshold
            ]

            # Further filter by severity if specified
            if severity:
                recent_alerts = [alert for alert in recent_alerts if alert.get('severity') == severity]

            # Take the most recent up to the limit
            recent_alerts = recent_alerts[-limit:] if len(recent_alerts) >= limit else recent_alerts

            # Convert timestamps to ISO format
            for alert in recent_alerts:
                if 'timestamp' in alert:
                    alert['timestamp'] = datetime.fromtimestamp(alert['timestamp']).isoformat()

            return jsonify({
                'count': len(recent_alerts),
                'data': recent_alerts
            }), 200
    except Exception as e:
        logger.error(f"Error getting alerts: {e}")
        return jsonify({'error': 'Failed to get alerts', 'details': str(e)}), 500


@app.route('/status', methods=['GET'])
def get_system_status():
    """Get overall system status summary."""
    try:
        with store_lock:
            # Calculate summary statistics
            active_alerts = [a for a in alerts_store if a.get('timestamp', 0) > (datetime.now() - timedelta(minutes=5)).timestamp()]
            recent_health_records = health_data_store[-10:] if health_data_store else []

            # Determine overall status
            overall_status = 'OK'
            if any(alert['severity'] == 'CRITICAL' for alert in active_alerts):
                overall_status = 'CRITICAL'
            elif any(alert['severity'] == 'ERROR' for alert in active_alerts):
                overall_status = 'ERROR'
            elif any(alert['severity'] == 'WARNING' for alert in active_alerts):
                overall_status = 'WARNING'

            summary = {
                'api_version': API_VERSION,
                'timestamp': datetime.now().isoformat(),
                'overall_status': overall_status,
                'total_health_records': len(health_data_store),
                'total_alerts': len(alerts_store),
                'active_alerts_count': len(active_alerts),
                'recent_records_count': len(recent_health_records),
                'last_update': datetime.fromtimestamp(max([h.get('timestamp', 0) for h in health_data_store] + [0])).isoformat() if health_data_store else None
            }

            return jsonify(summary), 200
    except Exception as e:
        logger.error(f"Error getting system status: {e}")
        return jsonify({'error': 'Failed to get system status', 'details': str(e)}), 500


@app.route('/health', methods=['POST'])
def post_health_data():
    """Receive health data from monitoring nodes (internal use)."""
    try:
        data = request.json

        if not data:
            return jsonify({'error': 'No data provided'}), 400

        # Validate required fields
        required_fields = ['node_name', 'timestamp', 'cpu_percent', 'memory_percent', 'disk_percent', 'status']
        for field in required_fields:
            if field not in data:
                return jsonify({'error': f'Missing required field: {field}'}), 400

        # Add to health data store
        with store_lock:
            # Add the data to our store
            health_data_store.append(data.copy())

            # Update recent metrics
            recent_metrics.update(data)

            # Maintain size limit
            if len(health_data_store) > MAX_STORED_RECORDS:
                health_data_store.pop(0)  # Remove oldest record

        logger.info(f"Received health data from {data['node_name']}: {data['status']}")
        return jsonify({'status': 'success', 'message': 'Health data received'}), 200
    except Exception as e:
        logger.error(f"Error posting health data: {e}")
        return jsonify({'error': 'Failed to process health data', 'details': str(e)}), 500


@app.route('/alerts', methods=['POST'])
def post_alert_data():
    """Receive alert data from monitoring nodes (internal use)."""
    try:
        data = request.json

        if not data:
            return jsonify({'error': 'No data provided'}), 400

        # Validate required fields for alerts
        required_fields = ['alert_id', 'timestamp', 'severity', 'component', 'message', 'node_name']
        for field in required_fields:
            if field not in data:
                return jsonify({'error': f'Missing required field: {field}'}), 400

        # Add to alerts store
        with store_lock:
            alerts_store.append(data.copy())

            # Maintain size limit
            if len(alerts_store) > MAX_STORED_RECORDS:
                alerts_store.pop(0)  # Remove oldest record

        logger.info(f"Received alert from {data['node_name']}: {data['severity']} - {data['message']}")
        return jsonify({'status': 'success', 'message': 'Alert data received'}), 200
    except Exception as e:
        logger.error(f"Error posting alert data: {e}")
        return jsonify({'error': 'Failed to process alert data', 'details': str(e)}), 500


@app.route('/health/aggregate', methods=['GET'])
def get_aggregated_health():
    """Get aggregated health statistics."""
    try:
        hours = int(request.args.get('hours', 24))
        time_threshold = datetime.now() - timedelta(hours=hours)

        with store_lock:
            # Filter data for the time window
            relevant_data = [
                h for h in health_data_store
                if datetime.fromtimestamp(h.get('timestamp', 0)) >= time_threshold
            ]

            if not relevant_data:
                return jsonify({
                    'aggregated_stats': {},
                    'time_window_hours': hours,
                    'record_count': 0
                }), 200

            # Calculate aggregates
            cpu_values = [h['cpu_percent'] for h in relevant_data if h['cpu_percent'] >= 0]
            memory_values = [h['memory_percent'] for h in relevant_data if h['memory_percent'] >= 0]
            disk_values = [h['disk_percent'] for h in relevant_data if h['disk_percent'] >= 0]

            aggregates = {}
            if cpu_values:
                aggregates['cpu'] = {
                    'avg': sum(cpu_values) / len(cpu_values),
                    'min': min(cpu_values),
                    'max': max(cpu_values),
                    'count': len(cpu_values)
                }
            if memory_values:
                aggregates['memory'] = {
                    'avg': sum(memory_values) / len(memory_values),
                    'min': min(memory_values),
                    'max': max(memory_values),
                    'count': len(memory_values)
                }
            if disk_values:
                aggregates['disk'] = {
                    'avg': sum(disk_values) / len(disk_values),
                    'min': min(disk_values),
                    'max': max(disk_values),
                    'count': len(disk_values)
                }

            return jsonify({
                'aggregated_stats': aggregates,
                'time_window_hours': hours,
                'record_count': len(relevant_data)
            }), 200
    except Exception as e:
        logger.error(f"Error getting aggregated health: {e}")
        return jsonify({'error': 'Failed to get aggregated health', 'details': str(e)}), 500


@app.route('/', methods=['GET'])
def api_info():
    """Get API information and endpoints."""
    return jsonify({
        'api_name': 'Edge Health Monitoring API',
        'version': API_VERSION,
        'description': 'REST API for accessing health status and metrics from edge devices',
        'endpoints': {
            'GET /health': 'Current health status',
            'GET /health/history': 'Historical health data',
            'GET /alerts': 'Current and recent alerts',
            'GET /status': 'Overall system status summary',
            'GET /health/aggregate': 'Aggregated health statistics',
            'POST /health': 'Submit health data (internal use)',
            'POST /alerts': 'Submit alert data (internal use)'
        },
        'timestamp': datetime.now().isoformat()
    }), 200


if __name__ == '__main__':
    # Set up logging
    app.logger.setLevel(logging.INFO)

    # Run the API server
    app.run(host='0.0.0.0', port=5000, debug=False)