"""System metric collection utilities for the edge_health package."""

import psutil
import platform
from typing import Dict, Optional, Tuple
import logging
import time
from functools import wraps


def get_cpu_usage() -> float:
    """
    Get current CPU usage percentage.

    Returns:
        CPU usage percentage as a float (0-100)
    """
    try:
        return psutil.cpu_percent(interval=1)
    except Exception as e:
        logging.getLogger('edge_health.system_metrics').warning(f"Could not get CPU usage: {e}")
        return -1.0


def get_memory_usage() -> float:
    """
    Get current memory usage percentage.

    Returns:
        Memory usage percentage as a float (0-100)
    """
    try:
        memory = psutil.virtual_memory()
        return memory.percent
    except Exception as e:
        logging.getLogger('edge_health.system_metrics').warning(f"Could not get memory usage: {e}")
        return -1.0


def get_disk_usage(path: str = '/') -> float:
    """
    Get disk usage percentage for the specified path.

    Args:
        path: Path to check disk usage for (default: root '/')

    Returns:
        Disk usage percentage as a float (0-100)
    """
    try:
        disk = psutil.disk_usage(path)
        return (disk.used / disk.total) * 100
    except Exception as e:
        logging.getLogger('edge_health.system_metrics').warning(f"Could not get disk usage for {path}: {e}")
        return -1.0


def get_temperature() -> Optional[float]:
    """
    Get system temperature if available.

    Returns:
        Temperature in Celsius as a float, or None if not available
    """
    try:
        # Try to get temperature from different sources depending on platform
        if platform.system() == "Linux":
            # Check for thermal zones (common on Linux)
            import os
            thermal_path = "/sys/class/thermal/thermal_zone0/temp"
            if os.path.exists(thermal_path):
                with open(thermal_path, 'r') as f:
                    temp = float(f.read().strip()) / 1000.0  # Convert from millidegrees
                    return temp

            # Try psutil sensors if available
            if hasattr(psutil, "sensors_temperatures"):
                temps = psutil.sensors_temperatures()
                if temps:
                    # Get the first available temperature sensor
                    for name, entries in temps.items():
                        if entries:
                            return entries[0].current
    except Exception as e:
        logging.getLogger('edge_health.system_metrics').warning(f"Could not get temperature: {e}")

    return None


def get_gpu_usage() -> Optional[float]:
    """
    Get GPU usage if available (for NVIDIA GPUs).

    Returns:
        GPU usage percentage as a float, or None if not available
    """
    try:
        # Try to get GPU usage using nvidia-ml-py (if available)
        import pynvml
        pynvml.nvmlInit()
        handle = pynvml.nvmlDeviceGetHandleByIndex(0)
        util = pynvml.nvmlDeviceGetUtilizationRates(handle)
        return float(util.gpu)
    except ImportError:
        # nvidia-ml-py not available
        pass
    except Exception as e:
        logging.getLogger('edge_health.system_metrics').warning(f"Could not get GPU usage: {e}")

    return None


def get_power_usage() -> Optional[float]:
    """
    Get power usage if available (platform dependent).

    Returns:
        Power usage in watts as a float, or None if not available
    """
    # This is highly platform dependent and may not be available
    # Placeholder for future implementation
    return None


def get_all_metrics() -> Dict[str, float]:
    """
    Get all available system metrics.

    Returns:
        Dictionary containing all available metrics
    """
    metrics = {
        'cpu_percent': get_cpu_usage(),
        'memory_percent': get_memory_usage(),
        'disk_percent': get_disk_usage(),
        'temperature': get_temperature() or -1.0,
    }

    # Add GPU metrics if available
    gpu_usage = get_gpu_usage()
    if gpu_usage is not None:
        metrics['gpu_percent'] = gpu_usage

    # Add power metrics if available
    power_usage = get_power_usage()
    if power_usage is not None:
        metrics['power_watts'] = power_usage

    return metrics


def get_system_info() -> Dict[str, str]:
    """
    Get basic system information.

    Returns:
        Dictionary containing system information
    """
    return {
        'platform': platform.system(),
        'platform_release': platform.release(),
        'platform_version': platform.version(),
        'architecture': platform.machine(),
        'hostname': platform.node(),
        'processor': platform.processor(),
        'cpu_count': str(psutil.cpu_count()),
    }


def check_sensors_available() -> Dict[str, bool]:
    """
    Check which sensors are available on the system.

    Returns:
        Dictionary indicating availability of each sensor type
    """
    sensors = {
        'cpu': True,  # Always available
        'memory': True,  # Always available
        'disk': True,  # Always available
        'temperature': get_temperature() is not None,
        'gpu': get_gpu_usage() is not None,
        'power': get_power_usage() is not None,
    }

    return sensors