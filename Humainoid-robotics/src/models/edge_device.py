from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime

class EdgeDevice(BaseModel):
    """
    Represents a physical AI device or humanoid robot connected to the edge kit.

    Attributes:
        id: Unique identifier for the device
        name: Display name for the device
        type: Type/model of the device
        status: Current status of the device (online, offline, error, maintenance)
        connectionInfo: Connection information (IP, port, protocol)
        safetyParameters: Safety settings and parameters for the device
        lastSeen: Timestamp of last communication with the device
        capabilities: List of capabilities the device supports
    """
    id: str
    name: str
    type: str
    status: str  # online, offline, error, maintenance
    connectionInfo: dict  # IP address, port, communication protocol
    safetyParameters: dict  # speed limits, operational boundaries, emergency settings
    lastSeen: Optional[datetime] = None
    capabilities: List[str] = []