from agent_config_loader import load_agent_config, validate_agent_config, create_agent_from_config

def test_general_agent():
    """Test the general agent configuration."""
    try:
        # Load the general agent configuration
        config = load_agent_config('.claude/skills/agents/general-agent.yaml')
        print(f"General agent config loaded: {config['name']}")

        # Validate the configuration
        errors = validate_agent_config(config)
        if errors:
            print(f"Validation errors: {errors}")
            return False
        else:
            print("✓ General agent configuration is valid!")

        # Create an agent instance
        agent_instance = create_agent_from_config(config)
        print(f"✓ Created agent instance: {agent_instance['name']}")

        # Test the agent implementation
        from general_agent_impl import GeneralAgent
        agent = GeneralAgent(config)
        result = agent.execute('text:uppercase', {'text': 'hello world'})
        print(f"✓ Agent execution test result: {result['result']}")

        return True
    except Exception as e:
        print(f"Error testing general agent: {e}")
        return False

if __name__ == "__main__":
    success = test_general_agent()
    if success:
        print("\n✓ All tests passed! General agent is working correctly.")
    else:
        print("\n✗ Tests failed!")
        exit(1)