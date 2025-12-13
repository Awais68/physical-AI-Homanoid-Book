#!/usr/bin/env python3
"""
Script to run the Edge Health Monitor from the edge-kit directory structure.
This script properly handles the Python path to make the health monitor work correctly.
"""

import sys
import os
from pathlib import Path


def main():
    # Add the edge_health package to the Python path
    edge_kit_root = Path(__file__).resolve().parent.parent
    ros2_ws_src = edge_kit_root / "ros2_ws" / "src"
    edge_health_pkg = ros2_ws_src / "edge_health"

    # Add both the ros2_ws/src directory and the edge_health package directory to Python path
    sys.path.insert(0, str(ros2_ws_src))
    sys.path.insert(0, str(edge_health_pkg))

    # Import and run the health monitor
    try:
        from edge_health.health_monitor import main as health_main
        print("Starting Edge Health Monitor...")
        print(f"Configuration file location: {edge_kit_root}/config/edge_health_config.yaml")
        health_main()
    except ImportError as e:
        print(f"Failed to import health monitor: {e}")
        print("Make sure you're running from the edge-kit directory or the path is set correctly")
        print(f"Current path: {os.getcwd()}")
        print(f"Added to path: {str(ros2_ws_src)} and {str(edge_health_pkg)}")
        print("Available modules in edge_health:")
        for item in edge_health_pkg.iterdir():
            print(f"  - {item.name}")
        return 1
    except KeyboardInterrupt:
        print("\nHealth monitor stopped by user.")
        return 0


if __name__ == "__main__":
    sys.exit(main())