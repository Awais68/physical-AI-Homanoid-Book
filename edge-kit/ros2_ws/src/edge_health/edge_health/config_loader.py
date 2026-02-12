import yaml
import os
from pathlib import Path


class EdgeHealthConfig:
    """
    Configuration manager for the Edge Health Monitoring System
    """

    def __init__(self, config_path: str = None):
        possible_paths = []  # Initialize possible_paths outside the if block
        if config_path is None:
            # Look for config in common locations
            possible_paths = [
                "config/edge_health_config.yaml",
                "../config/edge_health_config.yaml",
                "../../config/edge_health_config.yaml",
                "/app/config/edge_health_config.yaml",
                str(Path.home() / ".edge_kit" / "edge_health_config.yaml")
            ]

            config_path_found = None
            for path in possible_paths:
                if os.path.exists(path):
                    config_path_found = path
                    break

            config_path = config_path_found

        if config_path is None or not os.path.exists(config_path):
            raise FileNotFoundError(f"Configuration file not found. Looked in: {possible_paths}")

        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)

    def get(self, key_path: str, default=None):
        """
        Get a configuration value using dot notation
        Example: get('health_monitor.publish_frequency')
        """
        keys = key_path.split('.')
        value = self.config

        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default

        return value

    def get_publish_frequency(self) -> float:
        return self.get('health_monitor.publish_frequency', 0.5)

    def get_cpu_threshold(self) -> float:
        return self.get('health_monitor.cpu_threshold_percent', 80.0)

    def get_memory_threshold(self) -> float:
        return self.get('health_monitor.memory_threshold_percent', 85.0)

    def get_disk_threshold(self) -> float:
        return self.get('health_monitor.disk_threshold_percent', 90.0)

    def get_log_level(self) -> str:
        return self.get('health_monitor.log_level', 'INFO')

    def get_temperature_threshold(self) -> float:
        return self.get('health_monitor.temperature_threshold_celsius', 75.0)


# Example usage
if __name__ == "__main__":
    try:
        config = EdgeHealthConfig()
        print("Edge Health Configuration loaded successfully!")
        print(f"Publish frequency: {config.get_publish_frequency()}s")
        print(f"CPU threshold: {config.get_cpu_threshold()}%")
        print(f"Memory threshold: {config.get_memory_threshold()}%")
        print(f"Temperature threshold: {config.get_temperature_threshold()}°C")
    except FileNotFoundError as e:
        print(f"Error: {e}")