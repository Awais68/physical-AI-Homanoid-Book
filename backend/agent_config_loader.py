"""
Agent Configuration Loader Module

This module provides utilities to properly load and validate agent configuration files,
ensuring that YAML files are parsed with the correct parser instead of JSON.
"""

import yaml
import json
from pathlib import Path
from typing import Dict, Any, Union, List, Optional

class AgentConfigError(Exception):
    """Custom exception for agent configuration errors."""
    pass


def load_agent_config(agent_file_path: Union[str, Path]) -> Dict[str, Any]:
    """
    Properly load an agent configuration file (YAML or JSON) using the appropriate parser.

    Args:
        agent_file_path: Path to the agent configuration file

    Returns:
        Dictionary containing the agent configuration

    Raises:
        AgentConfigError: If the file cannot be loaded or parsed
    """
    path = Path(agent_file_path)

    if not path.exists():
        raise AgentConfigError(f"Agent configuration file not found: {agent_file_path}")

    try:
        with open(path, 'r', encoding='utf-8') as file:
            content = file.read()

        # Determine file type and parse accordingly
        if path.suffix.lower() in ['.yaml', '.yml']:
            try:
                # Use yaml.safe_load for security (avoids arbitrary code execution)
                config = yaml.safe_load(content)
                if config is None:
                    raise AgentConfigError(f"YAML file is empty or contains only comments: {agent_file_path}")
                return config
            except yaml.YAMLError as e:
                raise AgentConfigError(f"Invalid YAML in {agent_file_path}: {str(e)}")
        elif path.suffix.lower() == '.json':
            try:
                return json.loads(content)
            except json.JSONDecodeError as e:
                raise AgentConfigError(f"Invalid JSON in {agent_file_path}: {str(e)}")
        else:
            # Default to YAML for agent files since most agent configs are in YAML
            try:
                config = yaml.safe_load(content)
                if config is None:
                    raise AgentConfigError(f"File is empty or contains only comments: {agent_file_path}")
                return config
            except yaml.YAMLError as e:
                raise AgentConfigError(f"Could not parse {agent_file_path} as YAML: {str(e)}")
    except UnicodeDecodeError as e:
        raise AgentConfigError(f"Could not decode file {agent_file_path} as UTF-8: {str(e)}")
    except Exception as e:
        raise AgentConfigError(f"Error reading file {agent_file_path}: {str(e)}")


def validate_agent_config(config: Dict[str, Any]) -> List[str]:
    """
    Validate that the agent configuration has required fields.

    Args:
        config: Agent configuration dictionary

    Returns:
        List of validation errors (empty if valid)
    """
    errors = []

    # Required fields
    required_fields = ['name', 'description']
    for field in required_fields:
        if field not in config:
            errors.append(f"Missing required field '{field}' in agent configuration")

    # Validate name is a string
    if 'name' in config and not isinstance(config['name'], str):
        errors.append("Agent name must be a string")

    # Validate description is a string
    if 'description' in config and not isinstance(config['description'], str):
        errors.append("Agent description must be a string")

    # Validate optional fields if they exist
    if 'skills' in config and not isinstance(config['skills'], list):
        errors.append("Agent skills must be a list")

    if 'inputs' in config and not isinstance(config['inputs'], dict):
        errors.append("Agent inputs must be a dictionary")

    if 'outputs' in config and not isinstance(config['outputs'], dict):
        errors.append("Agent outputs must be a dictionary")

    if 'behavior' in config and not isinstance(config['behavior'], dict):
        errors.append("Agent behavior must be a dictionary")

    return errors


def get_all_agent_configs(agents_dir: Union[str, Path]) -> Dict[str, Dict[str, Any]]:
    """
    Load all agent configuration files from a directory.

    Args:
        agents_dir: Directory containing agent configuration files

    Returns:
        Dictionary mapping agent names to their configurations
    """
    agents_dir = Path(agents_dir)
    if not agents_dir.exists():
        raise AgentConfigError(f"Agents directory does not exist: {agents_dir}")

    agent_configs = {}

    for config_file in agents_dir.glob("*.yaml"):
        try:
            config = load_agent_config(config_file)
            errors = validate_agent_config(config)
            if errors:
                raise AgentConfigError(f"Validation errors in {config_file}: {'; '.join(errors)}")

            agent_name = config.get('name')
            if agent_name:
                agent_configs[agent_name] = config
            else:
                raise AgentConfigError(f"Agent configuration in {config_file} has no name")
        except AgentConfigError as e:
            # Log the error but continue with other files
            print(f"Warning: Could not load agent config {config_file}: {e}")

    for config_file in agents_dir.glob("*.yml"):
        try:
            config = load_agent_config(config_file)
            errors = validate_agent_config(config)
            if errors:
                raise AgentConfigError(f"Validation errors in {config_file}: {'; '.join(errors)}")

            agent_name = config.get('name')
            if agent_name:
                agent_configs[agent_name] = config
            else:
                raise AgentConfigError(f"Agent configuration in {config_file} has no name")
        except AgentConfigError as e:
            # Log the error but continue with other files
            print(f"Warning: Could not load agent config {config_file}: {e}")

    for config_file in agents_dir.glob("*.json"):
        try:
            config = load_agent_config(config_file)
            errors = validate_agent_config(config)
            if errors:
                raise AgentConfigError(f"Validation errors in {config_file}: {'; '.join(errors)}")

            agent_name = config.get('name')
            if agent_name:
                agent_configs[agent_name] = config
            else:
                raise AgentConfigError(f"Agent configuration in {config_file} has no name")
        except AgentConfigError as e:
            # Log the error but continue with other files
            print(f"Warning: Could not load agent config {config_file}: {e}")

    return agent_configs


def create_agent_from_config(agent_config: Dict[str, Any]) -> Dict[str, Any]:
    """
    Create an agent instance from a configuration.

    Args:
        agent_config: Agent configuration dictionary

    Returns:
        Agent instance with validated configuration
    """
    errors = validate_agent_config(agent_config)
    if errors:
        raise AgentConfigError(f"Invalid agent configuration: {'; '.join(errors)}")

    # Return a properly structured agent instance
    return {
        'name': agent_config['name'],
        'description': agent_config['description'],
        'skills': agent_config.get('skills', []),
        'inputs': agent_config.get('inputs', {}),
        'outputs': agent_config.get('outputs', {}),
        'behavior': agent_config.get('behavior', {}),
        'raw_config': agent_config
    }


# Example usage and testing
if __name__ == "__main__":
    try:
        # Test loading the indexer agent configuration
        config = load_agent_config('.claude/skills/agents/indexer-agent.yaml')

        print("Successfully loaded agent configuration:")
        print(f"  Name: {config.get('name')}")
        print(f"  Description: {config.get('description')}")
        print(f"  Skills: {config.get('skills', [])}")
        print(f"  Inputs: {list(config.get('inputs', {}).keys())}")
        print(f"  Outputs: {list(config.get('outputs', {}).keys())}")

        # Validate the configuration
        errors = validate_agent_config(config)
        if errors:
            print(f"Validation errors: {errors}")
        else:
            print("\n✓ Agent configuration is valid!")

        # Test creating an agent instance
        agent_instance = create_agent_from_config(config)
        print(f"\n✓ Created agent instance for: {agent_instance['name']}")

        # Test loading all agents from the directory
        all_agents = get_all_agent_configs('.claude/skills/agents')
        print(f"\n✓ Found {len(all_agents)} agent configurations")
        for agent_name in all_agents:
            print(f"  - {agent_name}")

    except AgentConfigError as e:
        print(f"Error: {e}")
        exit(1)
    except Exception as e:
        print(f"Unexpected error: {e}")
        exit(1)