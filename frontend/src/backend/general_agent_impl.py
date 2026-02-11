"""
General Agent Implementation

This module provides the implementation for the general-purpose agent
defined in general-agent.yaml.
"""

import time
import subprocess
from typing import Dict, Any, Optional
from pathlib import Path


class GeneralAgent:
    """
    A general-purpose agent for various automation and processing tasks.
    """

    def __init__(self, config: Dict[str, Any]):
        """
        Initialize the general agent with its configuration.

        Args:
            config: Agent configuration dictionary
        """
        self.config = config
        self.name = config['name']
        self.description = config['description']
        self.behavior = config.get('behavior', {})

    def execute(self, task: str, parameters: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Execute a general task based on the provided description.

        Args:
            task: Description of the task to perform
            parameters: Additional parameters for the task

        Returns:
            Dictionary containing the result of the execution
        """
        start_time = time.time()
        result = {
            'result': '',
            'success': True,
            'execution_time': '',
            'errors': []
        }

        try:
            # Process the task based on its content
            if 'bash' in self.config.get('skills', []) and task.startswith('bash:'):
                # Execute a bash command
                command = task[5:].strip()  # Remove 'bash:' prefix
                output = self._execute_bash_command(command)
                result['result'] = output

            elif 'file-operations' in self.config.get('skills', []) and task.startswith('file:'):
                # Perform a file operation
                file_task = task[6:].strip()  # Remove 'file:' prefix
                output = self._execute_file_operation(file_task, parameters)
                result['result'] = output

            elif 'text-processing' in self.config.get('skills', []) and task.startswith('text:'):
                # Perform text processing
                text_task = task[5:].strip()  # Remove 'text:' prefix
                output = self._execute_text_processing(text_task, parameters)
                result['result'] = output

            else:
                # Default behavior: return the task as result
                result['result'] = f"Executed general task: {task}"

        except Exception as e:
            result['success'] = False
            result['errors'].append(str(e))

        finally:
            execution_time = time.time() - start_time
            result['execution_time'] = f"{execution_time:.2f}s"

        return result

    def _execute_bash_command(self, command: str) -> str:
        """
        Execute a bash command safely.

        Args:
            command: The command to execute

        Returns:
            Output of the command
        """
        try:
            result = subprocess.run(
                command,
                shell=True,
                capture_output=True,
                text=True,
                timeout=30  # 30 second timeout
            )
            if result.returncode == 0:
                return result.stdout
            else:
                raise Exception(f"Command failed with return code {result.returncode}: {result.stderr}")
        except subprocess.TimeoutExpired:
            raise Exception("Command timed out")
        except Exception as e:
            raise e

    def _execute_file_operation(self, operation: str, parameters: Optional[Dict[str, Any]] = None) -> str:
        """
        Perform a file operation.

        Args:
            operation: The file operation to perform
            parameters: Additional parameters for the operation

        Returns:
            Result of the file operation
        """
        if operation == 'list':
            path = parameters.get('path', '.')
            path_obj = Path(path)
            if path_obj.exists() and path_obj.is_dir():
                files = [str(p) for p in path_obj.iterdir()]
                return f"Files in {path}: {', '.join(files)}"
            else:
                raise Exception(f"Path does not exist or is not a directory: {path}")
        elif operation == 'read':
            file_path = parameters.get('file_path')
            if not file_path:
                raise Exception("file_path parameter required for read operation")
            path_obj = Path(file_path)
            if path_obj.exists() and path_obj.is_file():
                return path_obj.read_text()
            else:
                raise Exception(f"File does not exist: {file_path}")
        else:
            raise Exception(f"Unsupported file operation: {operation}")

    def _execute_text_processing(self, operation: str, parameters: Optional[Dict[str, Any]] = None) -> str:
        """
        Perform text processing operations.

        Args:
            operation: The text processing operation to perform
            parameters: Additional parameters for the operation

        Returns:
            Result of the text processing
        """
        if operation == 'count':
            text = parameters.get('text', '')
            return f"Text length: {len(text)} characters, {len(text.split())} words"
        elif operation == 'uppercase':
            text = parameters.get('text', '')
            return text.upper()
        elif operation == 'lowercase':
            text = parameters.get('text', '')
            return text.lower()
        else:
            raise Exception(f"Unsupported text operation: {operation}")


def create_agent_from_config(config: Dict[str, Any]) -> GeneralAgent:
    """
    Factory function to create a GeneralAgent instance from configuration.

    Args:
        config: Agent configuration dictionary

    Returns:
        GeneralAgent instance
    """
    return GeneralAgent(config)


# Example usage
if __name__ == "__main__":
    # Example configuration similar to what's in general-agent.yaml
    example_config = {
        'name': 'general-agent',
        'description': 'A general-purpose agent for various automation and processing tasks',
        'skills': ['bash', 'file-operations', 'text-processing'],
        'behavior': {'logging': True, 'error_handling': 'comprehensive'}
    }

    agent = GeneralAgent(example_config)

    # Test different types of tasks
    print("Testing bash command:")
    result = agent.execute('bash:echo "Hello from general agent!"')
    print(f"Result: {result}")

    print("\nTesting text processing:")
    result = agent.execute('text:uppercase', {'text': 'hello world'})
    print(f"Result: {result}")

    print("\nTesting file operation:")
    result = agent.execute('file:list', {'path': '.'})
    print(f"Result: {result}")