"""
Device Communication Module
Handles communication protocols with physical AI devices
"""

import asyncio
import json
from typing import Dict, Any, Optional
from abc import ABC, abstractmethod
from datetime import datetime

class DeviceCommunicator(ABC):
    """
    Abstract base class for device communication protocols
    """

    @abstractmethod
    async def connect(self, device_info: Dict[str, Any]) -> bool:
        """Connect to a device using the specific protocol"""
        pass

    @abstractmethod
    async def disconnect(self, device_id: str) -> bool:
        """Disconnect from a device"""
        pass

    @abstractmethod
    async def send_command(self, device_id: str, command: Dict[str, Any]) -> Dict[str, Any]:
        """Send a command to the device and await response"""
        pass

    @abstractmethod
    async def receive_data(self, device_id: str) -> Optional[Dict[str, Any]]:
        """Receive data from the device"""
        pass

    @abstractmethod
    async def is_connected(self, device_id: str) -> bool:
        """Check if the device is currently connected"""
        pass


class ROSBridgeCommunicator(DeviceCommunicator):
    """
    Communicator for ROS (Robot Operating System) based devices
    """

    def __init__(self):
        self.connections = {}
        self.ros_bridge_url = None  # Will be configured externally

    async def connect(self, device_info: Dict[str, Any]) -> bool:
        """
        Connect to a ROS-enabled device via ROS Bridge
        """
        device_id = device_info.get('id')
        ros_url = device_info.get('connectionInfo', {}).get('ros_url')

        if not ros_url:
            return False

        # In a real implementation, this would connect to the ROS Bridge
        # For now, we'll simulate the connection
        self.connections[device_id] = {
            'protocol': 'ros',
            'url': ros_url,
            'connected_at': datetime.utcnow().isoformat(),
            'status': 'connected'
        }

        return True

    async def disconnect(self, device_id: str) -> bool:
        """
        Disconnect from a ROS device
        """
        if device_id in self.connections:
            self.connections[device_id]['status'] = 'disconnected'
            self.connections[device_id]['disconnected_at'] = datetime.utcnow().isoformat()
            return True
        return False

    async def send_command(self, device_id: str, command: Dict[str, Any]) -> Dict[str, Any]:
        """
        Send a command to a ROS device
        """
        if device_id not in self.connections:
            return {'error': 'Device not connected'}

        # In a real implementation, this would send the command via ROS Bridge
        # For simulation, we'll just return a success response
        return {
            'command_id': command.get('id'),
            'status': 'sent',
            'timestamp': datetime.utcnow().isoformat(),
            'device_response': 'Command received'
        }

    async def receive_data(self, device_id: str) -> Optional[Dict[str, Any]]:
        """
        Receive data from a ROS device
        """
        if device_id not in self.connections:
            return None

        # In a real implementation, this would receive data via ROS Bridge
        # For simulation, we'll return mock sensor data
        return {
            'device_id': device_id,
            'timestamp': datetime.utcnow().isoformat(),
            'sensor_data': {
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                'battery_level': 95.5,
                'temperature': 23.4
            },
            'status': 'ok'
        }

    async def is_connected(self, device_id: str) -> bool:
        """
        Check if a ROS device is connected
        """
        return (
            device_id in self.connections and
            self.connections[device_id]['status'] == 'connected'
        )


class MQTTCommunicator(DeviceCommunicator):
    """
    Communicator for MQTT-based devices
    """

    def __init__(self):
        self.connections = {}
        self.mqtt_broker_url = None  # Will be configured externally

    async def connect(self, device_info: Dict[str, Any]) -> bool:
        """
        Connect to an MQTT device
        """
        device_id = device_info.get('id')
        mqtt_broker = device_info.get('connectionInfo', {}).get('mqtt_broker')
        client_id = device_info.get('connectionInfo', {}).get('client_id', device_id)

        if not mqtt_broker:
            return False

        # In a real implementation, this would connect to the MQTT broker
        # For now, we'll simulate the connection
        self.connections[device_id] = {
            'protocol': 'mqtt',
            'broker': mqtt_broker,
            'client_id': client_id,
            'connected_at': datetime.utcnow().isoformat(),
            'status': 'connected',
            'topics': {
                'command': f'device/{device_id}/command',
                'status': f'device/{device_id}/status',
                'data': f'device/{device_id}/data'
            }
        }

        return True

    async def disconnect(self, device_id: str) -> bool:
        """
        Disconnect from an MQTT device
        """
        if device_id in self.connections:
            self.connections[device_id]['status'] = 'disconnected'
            self.connections[device_id]['disconnected_at'] = datetime.utcnow().isoformat()
            return True
        return False

    async def send_command(self, device_id: str, command: Dict[str, Any]) -> Dict[str, Any]:
        """
        Send a command to an MQTT device
        """
        if device_id not in self.connections:
            return {'error': 'Device not connected'}

        # In a real implementation, this would publish to MQTT topic
        # For simulation, we'll just return a success response
        return {
            'command_id': command.get('id'),
            'status': 'published',
            'timestamp': datetime.utcnow().isoformat(),
            'topic': self.connections[device_id]['topics']['command'],
            'device_response': 'Command published to topic'
        }

    async def receive_data(self, device_id: str) -> Optional[Dict[str, Any]]:
        """
        Receive data from an MQTT device
        """
        if device_id not in self.connections:
            return None

        # In a real implementation, this would subscribe to MQTT topics
        # For simulation, we'll return mock sensor data
        return {
            'device_id': device_id,
            'timestamp': datetime.utcnow().isoformat(),
            'topic': self.connections[device_id]['topics']['data'],
            'payload': {
                'sensor_readings': {
                    'temperature': 24.2,
                    'humidity': 45.3,
                    'motion_detected': False
                },
                'status': 'active'
            }
        }

    async def is_connected(self, device_id: str) -> bool:
        """
        Check if an MQTT device is connected
        """
        return (
            device_id in self.connections and
            self.connections[device_id]['status'] == 'connected'
        )


class DeviceCommunicationManager:
    """
    Manager class that orchestrates communication with various types of devices
    """

    def __init__(self):
        self.ros_communicator = ROSBridgeCommunicator()
        self.mqtt_communicator = MQTTCommunicator()
        self.active_connections = {}

    async def connect_device(self, device_info: Dict[str, Any]) -> bool:
        """
        Connect to a device based on its communication protocol
        """
        device_id = device_info.get('id')
        protocol = device_info.get('connectionInfo', {}).get('protocol', 'mqtt')

        success = False
        if protocol.lower() == 'ros':
            success = await self.ros_communicator.connect(device_info)
        elif protocol.lower() == 'mqtt':
            success = await self.mqtt_communicator.connect(device_info)
        else:
            # Default to MQTT if protocol not specified
            success = await self.mqtt_communicator.connect(device_info)

        if success:
            self.active_connections[device_id] = protocol.lower()
            return True

        return False

    async def disconnect_device(self, device_id: str) -> bool:
        """
        Disconnect from a device
        """
        if device_id not in self.active_connections:
            return False

        protocol = self.active_connections[device_id]
        success = False

        if protocol == 'ros':
            success = await self.ros_communicator.disconnect(device_id)
        elif protocol == 'mqtt':
            success = await self.mqtt_communicator.disconnect(device_id)

        if success:
            del self.active_connections[device_id]

        return success

    async def send_command_to_device(self, device_id: str, command: Dict[str, Any]) -> Dict[str, Any]:
        """
        Send a command to a specific device
        """
        if device_id not in self.active_connections:
            return {'error': 'Device not connected'}

        protocol = self.active_connections[device_id]

        if protocol == 'ros':
            return await self.ros_communicator.send_command(device_id, command)
        elif protocol == 'mqtt':
            return await self.mqtt_communicator.send_command(device_id, command)

        return {'error': 'Unknown protocol'}

    async def receive_data_from_device(self, device_id: str) -> Optional[Dict[str, Any]]:
        """
        Receive data from a specific device
        """
        if device_id not in self.active_connections:
            return None

        protocol = self.active_connections[device_id]

        if protocol == 'ros':
            return await self.ros_communicator.receive_data(device_id)
        elif protocol == 'mqtt':
            return await self.mqtt_communicator.receive_data(device_id)

        return None

    async def is_device_connected(self, device_id: str) -> bool:
        """
        Check if a specific device is connected
        """
        if device_id not in self.active_connections:
            return False

        protocol = self.active_connections[device_id]

        if protocol == 'ros':
            return await self.ros_communicator.is_connected(device_id)
        elif protocol == 'mqtt':
            return await self.mqtt_communicator.is_connected(device_id)

        return False


# Global instance of the communication manager
device_communication_manager = DeviceCommunicationManager()