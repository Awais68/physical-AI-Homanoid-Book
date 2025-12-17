# Agent Configuration Loading Fix

## Problem
When creating sub-agents, the system was throwing the error: `Expected property name or '}' in JSON at position 2`. This occurred because agent configuration files in YAML format were being parsed using a JSON parser instead of a YAML parser.

## Root Cause
The error happens when:
1. An agent configuration file is created in YAML format (e.g., `indexer-agent.yaml`)
2. The system attempts to parse it using `JSON.parse()` instead of a YAML parser
3. The YAML content is invalid JSON, causing the parser to fail at the first character

## Solution
Created proper agent configuration loading utilities that:

1. **Detect file type** by extension (.yaml, .yml, .json)
2. **Use appropriate parser** for each file type:
   - YAML files: Use `yaml.safe_load()`
   - JSON files: Use `json.loads()`
3. **Validate configuration** after loading to ensure required fields exist
4. **Handle errors gracefully** with descriptive error messages

## Files Created

### `agent_config_loader.py`
Main module with utilities for loading and validating agent configurations:
- `load_agent_config()` - Loads config files with appropriate parser
- `validate_agent_config()` - Validates required fields
- `get_all_agent_configs()` - Loads all agents from a directory
- `create_agent_from_config()` - Creates agent instances from configs

### Dependencies Added
- Added `pyyaml>=6.0` to `pyproject.toml` for YAML parsing support

## Usage
```python
from agent_config_loader import load_agent_config, validate_agent_config

# Load agent configuration (works for both YAML and JSON)
config = load_agent_config('.claude/skills/agents/indexer-agent.yaml')

# Validate the configuration
errors = validate_agent_config(config)
if not errors:
    print("Agent configuration is valid!")
```

## Testing
The fix has been tested with the existing `indexer-agent.yaml` file and works correctly:
- Successfully loads YAML agent configurations
- Validates required fields
- Creates proper agent instances
- Handles errors gracefully

## Prevention
To prevent similar issues in the future:
1. Always use the provided loader functions for agent configs
2. Ensure YAML files have proper extensions (.yaml or .yml)
3. Include required fields (name, description) in all agent configs
4. Validate configurations before using them