import yaml
import json
from pathlib import Path
from typing import Dict, Any, Union

def load_agent_config(agent_file_path: str) -> Dict[str, Any]:
    """
    Properly load an agent configuration file (YAML or JSON) using the appropriate parser.

    Args:
        agent_file_path: Path to the agent configuration file

    Returns:
        Dictionary containing the agent configuration
    """
    path = Path(agent_file_path)

    if not path.exists():
        raise FileNotFoundError(f"Agent configuration file not found: {agent_file_path}")

    with open(path, 'r', encoding='utf-8') as file:
        content = file.read()

    # Determine file type and parse accordingly
    if path.suffix.lower() in ['.yaml', '.yml']:
        try:
            # Use yaml.safe_load for security (avoids arbitrary code execution)
            config = yaml.safe_load(content)
            if config is None:
                raise ValueError(f"YAML file is empty or contains only comments: {agent_file_path}")
            return config
        except yaml.YAMLError as e:
            raise ValueError(f"Invalid YAML in {agent_file_path}: {str(e)}")
    elif path.suffix.lower() == '.json':
        try:
            return json.loads(content)
        except json.JSONDecodeError as e:
            raise ValueError(f"Invalid JSON in {agent_file_path}: {str(e)}")
    else:
        # Default to YAML for agent files
        try:
            config = yaml.safe_load(content)
            if config is None:
                raise ValueError(f"File is empty or contains only comments: {agent_file_path}")
            return config
        except yaml.YAMLError as e:
            raise ValueError(f"Could not parse {agent_file_path} as YAML: {str(e)}")


def validate_agent_config(config: Dict[str, Any]) -> bool:
    """
    Validate that the agent configuration has required fields.

    Args:
        config: Agent configuration dictionary

    Returns:
        True if valid, raises ValueError if invalid
    """
    required_fields = ['name', 'description']

    for field in required_fields:
        if field not in config:
            raise ValueError(f"Missing required field '{field}' in agent configuration")

    # Validate name is a string
    if not isinstance(config['name'], str):
        raise ValueError("Agent name must be a string")

    # Validate description is a string
    if not isinstance(config['description'], str):
        raise ValueError("Agent description must be a string")

    # Validate optional fields if they exist
    if 'skills' in config and not isinstance(config['skills'], list):
        raise ValueError("Agent skills must be a list")

    if 'inputs' in config and not isinstance(config['inputs'], dict):
        raise ValueError("Agent inputs must be a dictionary")

    if 'outputs' in config and not isinstance(config['outputs'], dict):
        raise ValueError("Agent outputs must be a dictionary")

    if 'behavior' in config and not isinstance(config['behavior'], dict):
        raise ValueError("Agent behavior must be a dictionary")

    return True


def main():
    """Test the agent loader with the indexer-agent.yaml file."""
    try:
        # Load the indexer agent configuration
        config = load_agent_config('.claude/skills/agents/indexer-agent.yaml')

        print("Successfully loaded agent configuration:")
        print(f"Name: {config.get('name')}")
        print(f"Description: {config.get('description')}")
        print(f"Skills: {config.get('skills', [])}")
        print(f"Inputs: {list(config.get('inputs', {}).keys())}")
        print(f"Outputs: {list(config.get('outputs', {}).keys())}")

        # Validate the configuration
        validate_agent_config(config)
        print("\n✓ Agent configuration is valid!")

    except Exception as e:
        print(f"Error loading agent configuration: {str(e)}")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())