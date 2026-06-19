from fastapi import APIRouter, HTTPException, Depends
from typing import List
from pydantic import BaseModel
from src.models.edge_device import EdgeDevice
from src.services.device_monitor import DeviceMonitoringService

router = APIRouter()

# Pydantic models for request/response
class EdgeDeviceCreate(BaseModel):
    id: str
    name: str
    type: str
    connectionInfo: dict
    safetyParameters: dict
    capabilities: List[str] = []

class EdgeDeviceResponse(EdgeDeviceCreate):
    id: str
    status: str = "offline"
    lastSeen: str = None

# Mock device storage (in real implementation, this would connect to database)
devices_db = {}

@router.post("/devices", response_model=EdgeDeviceResponse)
async def register_device(device: EdgeDeviceCreate):
    """
    Register a new physical AI device with the edge kit
    """
    # Check if device already exists
    if device.id in devices_db:
        raise HTTPException(status_code=400, detail="Device with this ID already exists")

    # Create device entry
    new_device = EdgeDeviceResponse(
        id=device.id,
        name=device.name,
        type=device.type,
        connectionInfo=device.connectionInfo,
        safetyParameters=device.safetyParameters,
        capabilities=device.capabilities,
        status="registered"
    )

    devices_db[device.id] = new_device
    return new_device

@router.get("/devices", response_model=List[EdgeDeviceResponse])
async def list_devices():
    """
    List all connected physical AI devices
    """
    return list(devices_db.values())

@router.get("/devices/{device_id}", response_model=EdgeDeviceResponse)
async def get_device(device_id: str):
    """
    Get details of a specific device
    """
    device = devices_db.get(device_id)
    if not device:
        raise HTTPException(status_code=404, detail="Device not found")
    return device

@router.put("/devices/{device_id}", response_model=EdgeDeviceResponse)
async def update_device(device_id: str, device: EdgeDeviceCreate):
    """
    Update device information
    """
    existing_device = devices_db.get(device_id)
    if not existing_device:
        raise HTTPException(status_code=404, detail="Device not found")

    updated_device = EdgeDeviceResponse(
        id=device_id,
        name=device.name,
        type=device.type,
        connectionInfo=device.connectionInfo,
        safetyParameters=device.safetyParameters,
        capabilities=device.capabilities,
        status=existing_device.status
    )

    devices_db[device_id] = updated_device
    return updated_device

@router.delete("/devices/{device_id}")
async def remove_device(device_id: str):
    """
    Remove a device from the system
    """
    if device_id not in devices_db:
        raise HTTPException(status_code=404, detail="Device not found")

    del devices_db[device_id]
    return {"message": "Device removed successfully"}

# Device monitoring endpoint (health check)
@router.get("/devices/{device_id}/health")
async def device_health(device_id: str):
    """
    Get health status of a specific device
    """
    device = devices_db.get(device_id)
    if not device:
        raise HTTPException(status_code=404, detail="Device not found")

    # In a real implementation, this would check actual device connectivity
    # For now, we'll return a mock status
    health_status = {
        "deviceId": device_id,
        "status": device.status,
        "lastSeen": device.lastSeen,
        "connected": device.status == "online",
        "capabilities": device.capabilities
    }

    return health_status