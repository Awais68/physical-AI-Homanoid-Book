"""Logging utilities for the edge_health package."""

import logging
import os
from pathlib import Path
from datetime import datetime
import json
from typing import Dict, Any


class StructuredFormatter(logging.Formatter):
    """Custom formatter that can output structured logs in JSON or standard format."""

    def __init__(self, fmt=None, datefmt=None, style='%', use_json=False):
        super().__init__(fmt, datefmt, style)
        self.use_json = use_json

    def format(self, record):
        if self.use_json:
            # Create structured log record in JSON format
            log_entry = {
                'timestamp': datetime.fromtimestamp(record.created).isoformat(),
                'level': record.levelname,
                'logger': record.name,
                'message': record.getMessage(),
                'module': record.module,
                'function': record.funcName,
                'line': record.lineno,
            }

            # Add any extra fields that were passed to the log call
            for key, value in record.__dict__.items():
                if key not in ['name', 'msg', 'args', 'levelname', 'levelno', 'pathname',
                              'filename', 'module', 'lineno', 'funcName', 'created',
                              'msecs', 'relativeCreated', 'thread', 'threadName',
                              'processName', 'process', 'getMessage', 'exc_info',
                              'exc_text', 'stack_info']:
                    log_entry[key] = value

            return json.dumps(log_entry)
        else:
            # Use standard format with more detailed timestamp
            return super().format(record)


def setup_logging(log_level: str = "INFO", log_file: str = None, use_json: bool = False):
    """
    Set up logging configuration for the edge health monitoring system.

    Args:
        log_level: The logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        log_file: Optional file path to write logs to
        use_json: Whether to use JSON format for logs
    """
    # Convert string log level to logging constant
    level = getattr(logging, log_level.upper(), logging.INFO)

    # Create formatter
    if use_json:
        formatter = StructuredFormatter(use_json=True)
    else:
        formatter = StructuredFormatter(
            fmt='%(asctime)s.%(msecs)03d - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )

    # Get the root logger
    logger = logging.getLogger('edge_health')
    logger.setLevel(level)

    # Clear any existing handlers
    logger.handlers.clear()

    # Console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(level)
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)

    # File handler (if specified)
    if log_file:
        # Ensure directory exists
        log_path = Path(log_file)
        log_path.parent.mkdir(parents=True, exist_ok=True)

        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(level)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    return logger


def get_logger(name: str = None):
    """
    Get a logger instance for the specified name.

    Args:
        name: Optional name for the logger (will be prefixed with edge_health)

    Returns:
        logging.Logger instance
    """
    if name:
        logger_name = f'edge_health.{name}'
    else:
        logger_name = 'edge_health'

    return logging.getLogger(logger_name)


def create_log_directory(base_dir: str = None):
    """
    Create a log directory for the edge health system.

    Args:
        base_dir: Base directory for logs (default: /tmp/edge_health_logs)

    Returns:
        Path to the log directory
    """
    if base_dir is None:
        base_dir = "/tmp/edge_health_logs"

    log_dir = Path(base_dir)
    log_dir.mkdir(parents=True, exist_ok=True)

    return log_dir


def setup_component_logger(component_name: str, config, use_json: bool = False):
    """
    Set up a logger for a specific component with the configuration.

    Args:
        component_name: Name of the component (e.g., 'health_monitor', 'threshold_monitor')
        config: MonitoringConfiguration instance
        use_json: Whether to use JSON format for logs

    Returns:
        logging.Logger instance for the component
    """
    log_level = config.log_level

    # Create log file path
    log_dir = create_log_directory()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")  # More granular timestamp
    log_file = log_dir / f"{component_name}_{timestamp}.log"

    # Set up the logger
    logger = logging.getLogger(f'edge_health.{component_name}')
    logger.setLevel(getattr(logging, log_level.upper(), logging.INFO))

    # Clear existing handlers
    logger.handlers.clear()

    # Create formatter
    if use_json:
        formatter = StructuredFormatter(use_json=True)
    else:
        formatter = StructuredFormatter(
            fmt='%(asctime)s.%(msecs)03d - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )

    # Console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(getattr(logging, log_level.upper(), logging.INFO))
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)

    # File handler
    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(getattr(logging, log_level.upper(), logging.INFO))
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    # Prevent propagation to avoid duplicate logs
    logger.propagate = False

    return logger