"""
Device Monitoring Service
Handles device connection monitoring, status tracking, and health checks
"""

from typing import Dict, List, Optional
from datetime import datetime
from src.models.edge_device import EdgeDevice

class DeviceMonitoringService:
    """
    Service class to handle device monitoring, connection status tracking,
    and health checks for connected physical AI devices.
    """

    def __init__(self):
        # In-memory storage for device statuses (in production, use database)
        self._device_statuses: Dict[str, Dict] = {}
        self._connection_history: Dict[str, List[Dict]] = {}

    async def register_device(self, device: EdgeDevice) -> bool:
        """
        Register a new device in the monitoring system
        """
        device_id = device.id
        self._device_statuses[device_id] = {
            'status': 'registered',
            'last_seen': datetime.utcnow().isoformat(),
            'connection_attempts': 0,
            'last_connection_attempt': None,
            'health_score': 100
        }

        if device_id not in self._connection_history:
            self._connection_history[device_id] = []

        return True

    async def update_device_status(self, device_id: str, status: str) -> bool:
        """
        Update the status of a connected device
        """
        if device_id not in self._device_statuses:
            return False

        self._device_statuses[device_id]['status'] = status
        self._device_statuses[device_id]['last_seen'] = datetime.utcnow().isoformat()

        # Record connection event
        self._record_connection_event(device_id, status)

        return True

    async def get_device_status(self, device_id: str) -> Optional[Dict]:
        """
        Get the current status of a specific device
        """
        if device_id not in self._device_statuses:
            return None

        return self._device_statuses[device_id]

    async def get_all_device_statuses(self) -> Dict[str, Dict]:
        """
        Get statuses for all monitored devices
        """
        return self._device_statuses.copy()

    async def heartbeat_received(self, device_id: str) -> bool:
        """
        Process a heartbeat signal from a device to confirm it's alive
        """
        if device_id not in self._device_statuses:
            return False

        self._device_statuses[device_id]['status'] = 'online'
        self._device_statuses[device_id]['last_seen'] = datetime.utcnow().isoformat()
        self._device_statuses[device_id]['connection_attempts'] += 1

        return True

    async def device_disconnected(self, device_id: str) -> bool:
        """
        Mark a device as disconnected
        """
        if device_id not in self._device_statuses:
            return False

        self._device_statuses[device_id]['status'] = 'offline'
        self._record_connection_event(device_id, 'disconnected')

        return True

    async def is_device_online(self, device_id: str) -> bool:
        """
        Check if a specific device is currently online
        """
        if device_id not in self._device_statuses:
            return False

        return self._device_statuses[device_id]['status'] == 'online'

    async def get_connection_history(self, device_id: str, limit: int = 10) -> List[Dict]:
        """
        Get connection history for a specific device
        """
        if device_id not in self._connection_history:
            return []

        # Return the most recent events
        return self._connection_history[device_id][-limit:]

    async def check_device_health(self, device_id: str) -> Dict:
        """
        Perform a health check on a specific device
        """
        if device_id not in self._device_statuses:
            return {'status': 'unknown', 'error': 'Device not found'}

        status_info = self._device_statuses[device_id]

        # Calculate health based on last seen time and status
        health_score = status_info['health_score']
        if status_info['status'] == 'offline':
            health_score = max(0, health_score - 20)
        elif status_info['status'] == 'error':
            health_score = max(0, health_score - 30)

        return {
            'deviceId': device_id,
            'status': status_info['status'],
            'lastSeen': status_info['last_seen'],
            'healthScore': health_score,
            'connected': status_info['status'] == 'online',
            'connectionAttempts': status_info['connection_attempts']
        }

    def _record_connection_event(self, device_id: str, event_type: str):
        """
        Internal method to record connection events
        """
        if device_id not in self._connection_history:
            self._connection_history[device_id] = []

        event = {
            'timestamp': datetime.utcnow().isoformat(),
            'eventType': event_type,
            'deviceId': device_id
        }

        self._connection_history[device_id].append(event)

        # Keep only the last 100 events to prevent memory issues
        if len(self._connection_history[device_id]) > 100:
            self._connection_history[device_id] = self._connection_history[device_id][-100:]

# Global instance of the service
device_monitor_service = DeviceMonitoringService()