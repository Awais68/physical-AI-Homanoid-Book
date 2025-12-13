# Contributing to Edge Health

Thank you for your interest in contributing to the Edge Health package! This document outlines the process for contributing to this project.

## Getting Started

1. Fork the repository
2. Create a feature branch for your changes
3. Make your changes following the coding standards below
4. Test your changes thoroughly
5. Submit a pull request with a clear description of your changes

## Development Setup

```bash
# Clone your fork
git clone https://github.com/your-username/repository-name.git
cd repository-name

# Navigate to the ROS2 workspace
cd edge-kit/ros2_ws

# Source ROS2
source /opt/ros/humble/setup.bash

# Build the package
colcon build --packages-select edge_health

# Source the built package
source install/setup.bash
```

## Coding Standards

- Follow PEP 8 style guidelines
- Use type hints for all function parameters and return values
- Write docstrings for all public functions, classes, and modules
- Use meaningful variable and function names
- Keep functions small and focused on a single responsibility

## Testing

All contributions must include appropriate tests:

- Unit tests for all core functionality
- Integration tests for ROS2 node interactions
- System tests for end-to-end functionality

Run tests with:
```bash
colcon test --packages-select edge_health
```

## Pull Request Guidelines

- Keep pull requests focused on a single feature or bug fix
- Include tests for new functionality
- Update documentation as needed
- Follow the existing code style
- Provide a clear description of the changes and why they're needed

## Code of Conduct

Please follow our Code of Conduct in all interactions with the project.