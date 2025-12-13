"""
Device Status Tracker Service
Manages and tracks the status of all connected physical AI devices
"""

import asyncio
from typing import Dict, List, Optional, Any
from datetime import datetime
from enum import Enum
import json
import uuid

class DeviceStatus(Enum):
    """Enumeration of possible device statuses"""
    ONLINE = "online"
    OFFLINE = "offline"
    ERROR = "error"
    MAINTENANCE = "maintenance"
    REGISTERED = "registered"
    CONNECTING = "connecting"
    DISCONNECTING = "disconnecting"


class DeviceHealthStatus(Enum):
    """Enumeration of device health levels"""
    EXCELLENT = "excellent"
    GOOD = "good"
    FAIR = "fair"
    POOR = "poor"
    CRITICAL = "critical"


class DeviceStatusRecord:
    """
    Class representing a device's status record
    """

    def __init__(self, device_id: str, initial_status: DeviceStatus = DeviceStatus.OFFLINE):
        self.device_id = device_id
        self.status = initial_status
        self.health_status = DeviceHealthStatus.GOOD
        self.last_seen = None
        self.first_seen = datetime.utcnow().isoformat()
        self.connection_start_time = None
        self.connection_end_time = None
        self.error_count = 0
        self.success_count = 0
        self.last_error = None
        self.metrics = {
            'response_time_avg': 0.0,
            'availability': 0.0,
            'error_rate': 0.0
        }
        self.properties = {}
        self.status_history = []

    def update_status(self, new_status: DeviceStatus, reason: str = None):
        """
        Update the device status and record the change
        """
        old_status = self.status
        self.status = new_status
        self.last_seen = datetime.utcnow().isoformat()

        # Record status change in history
        status_change = {
            'timestamp': self.last_seen,
            'old_status': old_status.value,
            'new_status': new_status.value,
            'reason': reason
        }
        self.status_history.append(status_change)

        # Keep only the last 100 status changes to prevent memory issues
        if len(self.status_history) > 100:
            self.status_history = self.status_history[-100:]

    def update_health_status(self, health_status: DeviceHealthStatus):
        """
        Update the health status of the device
        """
        self.health_status = health_status

    def record_success(self, response_time_ms: float = None):
        """
        Record a successful interaction with the device
        """
        self.success_count += 1
        if response_time_ms is not None:
            # Update average response time (simple calculation)
            total_time = self.metrics['response_time_avg'] * (self.success_count - 1) + response_time_ms
            self.metrics['response_time_avg'] = total_time / self.success_count

    def record_error(self, error_details: str = None):
        """
        Record an error when interacting with the device
        """
        self.error_count += 1
        self.last_error = {
            'timestamp': datetime.utcnow().isoformat(),
            'details': error_details
        }

        # Update error rate
        total_interactions = self.error_count + self.success_count
        if total_interactions > 0:
            self.metrics['error_rate'] = (self.error_count / total_interactions) * 100

    def update_availability(self):
        """
        Update the availability metric based on connection duration
        """
        if self.connection_start_time:
            duration_seconds = (datetime.utcnow() - datetime.fromisoformat(self.connection_start_time)).total_seconds()
            # Availability is calculated as a percentage of uptime
            # This is a simplified calculation - in reality, this would be more complex
            self.metrics['availability'] = min(100.0, self.success_count / max(1, self.error_count + self.success_count) * 100)

    def get_current_status_info(self) -> Dict[str, Any]:
        """
        Get the current status information for the device
        """
        return {
            'device_id': self.device_id,
            'status': self.status.value,
            'health_status': self.health_status.value,
            'last_seen': self.last_seen,
            'first_seen': self.first_seen,
            'error_count': self.error_count,
            'success_count': self.success_count,
            'last_error': self.last_error,
            'metrics': self.metrics,
            'properties': self.properties,
            'status_history_length': len(self.status_history)
        }


class DeviceStatusTracker:
    """
    Service class to track and manage the status of all devices
    """

    def __init__(self):
        self.devices: Dict[str, DeviceStatusRecord] = {}
        self.status_callbacks: Dict[str, List[callable]] = {}
        self.health_thresholds = {
            DeviceHealthStatus.EXCELLENT: 95,
            DeviceHealthStatus.GOOD: 85,
            DeviceHealthStatus.FAIR: 70,
            DeviceHealthStatus.POOR: 50,
            DeviceHealthStatus.CRITICAL: 0
        }

    async def register_device(self, device_id: str, initial_properties: Dict[str, Any] = None) -> bool:
        """
        Register a new device in the status tracking system
        """
        if device_id in self.devices:
            print(f"Device {device_id} already registered in status tracker")
            return False

        # Create a new status record for the device
        self.devices[device_id] = DeviceStatusRecord(device_id, DeviceStatus.REGISTERED)

        # Set initial properties if provided
        if initial_properties:
            self.devices[device_id].properties.update(initial_properties)

        print(f"Registered device {device_id} in status tracker")
        return True

    async def unregister_device(self, device_id: str) -> bool:
        """
        Remove a device from the status tracking system
        """
        if device_id not in self.devices:
            print(f"Device {device_id} not found in status tracker")
            return False

        del self.devices[device_id]

        # Remove any callbacks associated with this device
        if device_id in self.status_callbacks:
            del self.status_callbacks[device_id]

        print(f"Unregistered device {device_id} from status tracker")
        return True

    async def update_device_status(self, device_id: str, new_status: DeviceStatus, reason: str = None) -> bool:
        """
        Update the status of a device
        """
        if device_id not in self.devices:
            print(f"Device {device_id} not found in status tracker")
            return False

        old_status = self.devices[device_id].status
        self.devices[device_id].update_status(new_status, reason)

        # Trigger callbacks if status changed
        if old_status != new_status:
            await self._trigger_status_callbacks(device_id, old_status, new_status, reason)

        # Update availability metric
        self.devices[device_id].update_availability()

        return True

    async def update_device_health_status(self, device_id: str, health_status: DeviceHealthStatus) -> bool:
        """
        Update the health status of a device
        """
        if device_id not in self.devices:
            print(f"Device {device_id} not found in status tracker")
            return False

        self.devices[device_id].update_health_status(health_status)
        return True

    async def record_device_interaction(self, device_id: str, success: bool, response_time_ms: float = None, error_details: str = None) -> bool:
        """
        Record an interaction with the device (success or failure)
        """
        if device_id not in self.devices:
            print(f"Device {device_id} not found in status tracker")
            return False

        if success:
            self.devices[device_id].record_success(response_time_ms)
        else:
            self.devices[device_id].record_error(error_details)

        # Recalculate health status based on metrics
        await self._recalculate_health_status(device_id)

        return True

    async def get_device_status(self, device_id: str) -> Optional[Dict[str, Any]]:
        """
        Get the current status of a specific device
        """
        if device_id not in self.devices:
            return None

        return self.devices[device_id].get_current_status_info()

    async def get_all_device_statuses(self) -> Dict[str, Dict[str, Any]]:
        """
        Get the status of all tracked devices
        """
        statuses = {}
        for device_id, record in self.devices.items():
            statuses[device_id] = record.get_current_status_info()
        return statuses

    async def get_devices_by_status(self, status: DeviceStatus) -> List[str]:
        """
        Get all devices with a specific status
        """
        matching_devices = []
        for device_id, record in self.devices.items():
            if record.status == status:
                matching_devices.append(device_id)
        return matching_devices

    async def get_devices_by_health_status(self, health_status: DeviceHealthStatus) -> List[str]:
        """
        Get all devices with a specific health status
        """
        matching_devices = []
        for device_id, record in self.devices.items():
            if record.health_status == health_status:
                matching_devices.append(device_id)
        return matching_devices

    async def get_status_summary(self) -> Dict[str, Any]:
        """
        Get a summary of all device statuses
        """
        summary = {
            'total_devices': len(self.devices),
            'status_counts': {},
            'health_counts': {},
            'timestamp': datetime.utcnow().isoformat()
        }

        # Count devices by status
        for record in self.devices.values():
            status_val = record.status.value
            if status_val not in summary['status_counts']:
                summary['status_counts'][status_val] = 0
            summary['status_counts'][status_val] += 1

        # Count devices by health status
        for record in self.devices.values():
            health_val = record.health_status.value
            if health_val not in summary['health_counts']:
                summary['health_counts'][health_val] = 0
            summary['health_counts'][health_val] += 1

        return summary

    async def get_device_status_history(self, device_id: str, limit: int = 10) -> Optional[List[Dict[str, Any]]]:
        """
        Get the status history for a specific device
        """
        if device_id not in self.devices:
            return None

        history = self.devices[device_id].status_history
        # Return the most recent entries
        return history[-limit:] if len(history) >= limit else history[:]

    async def add_status_callback(self, device_id: str, callback: callable) -> bool:
        """
        Add a callback function to be called when the device status changes
        """
        if device_id not in self.devices:
            print(f"Device {device_id} not found in status tracker")
            return False

        if device_id not in self.status_callbacks:
            self.status_callbacks[device_id] = []

        self.status_callbacks[device_id].append(callback)
        return True

    async def remove_status_callback(self, device_id: str, callback: callable) -> bool:
        """
        Remove a callback function
        """
        if device_id not in self.status_callbacks:
            return False

        if callback in self.status_callbacks[device_id]:
            self.status_callbacks[device_id].remove(callback)
            return True

        return False

    async def update_device_properties(self, device_id: str, properties: Dict[str, Any]) -> bool:
        """
        Update the properties of a device
        """
        if device_id not in self.devices:
            print(f"Device {device_id} not found in status tracker")
            return False

        self.devices[device_id].properties.update(properties)
        return True

    async def _trigger_status_callbacks(self, device_id: str, old_status: DeviceStatus, new_status: DeviceStatus, reason: str = None):
        """
        Internal method to trigger status change callbacks
        """
        if device_id in self.status_callbacks:
            for callback in self.status_callbacks[device_id]:
                try:
                    # Call the callback with device ID, old status, new status, and reason
                    if asyncio.iscoroutinefunction(callback):
                        await callback(device_id, old_status, new_status, reason)
                    else:
                        callback(device_id, old_status, new_status, reason)
                except Exception as e:
                    print(f"Error in status callback for device {device_id}: {str(e)}")

    async def _recalculate_health_status(self, device_id: str):
        """
        Internal method to recalculate the health status based on metrics
        """
        if device_id not in self.devices:
            return

        record = self.devices[device_id]
        metrics = record.metrics

        # Calculate health score based on various factors
        # This is a simplified calculation - in reality, this would be more sophisticated
        availability_factor = metrics.get('availability', 0)
        error_rate_factor = 100 - metrics.get('error_rate', 0)
        response_time_factor = 100 - min(100, metrics.get('response_time_avg', 0) / 10)  # Assuming avg response time in ms

        # Weighted average calculation
        health_score = (availability_factor * 0.4 + error_rate_factor * 0.4 + response_time_factor * 0.2)

        # Determine health status based on thresholds
        if health_score >= self.health_thresholds[DeviceHealthStatus.EXCELLENT]:
            new_health_status = DeviceHealthStatus.EXCELLENT
        elif health_score >= self.health_thresholds[DeviceHealthStatus.GOOD]:
            new_health_status = DeviceHealthStatus.GOOD
        elif health_score >= self.health_thresholds[DeviceHealthStatus.FAIR]:
            new_health_status = DeviceHealthStatus.FAIR
        elif health_score >= self.health_thresholds[DeviceHealthStatus.POOR]:
            new_health_status = DeviceHealthStatus.POOR
        else:
            new_health_status = DeviceHealthStatus.CRITICAL

        # Update health status if it has changed
        if record.health_status != new_health_status:
            record.health_status = new_health_status


# Global instance of the device status tracker
device_status_tracker = DeviceStatusTracker()