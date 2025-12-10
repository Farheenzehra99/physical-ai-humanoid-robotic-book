# ROS2 Core Skill

**Skill ID**: `ros2_core`
**Category**: Simulation
**Complexity**: Low
**Introduced**: Week 1

## Overview

The ROS2 Core skill provides foundational ROS 2 setup, workspace management, and basic node operations. This skill is a prerequisite for all other robotics skills.

## Capabilities

1. **Environment Setup**
   - Install ROS 2 Jazzy/Iron on Ubuntu 22.04
   - Configure ROS 2 environment variables
   - Create and manage ROS 2 workspaces

2. **Workspace Management**
   - Create colcon workspaces
   - Build packages with colcon
   - Source workspace overlays

3. **Node Operations**
   - Create basic ROS 2 nodes (Python/C++)
   - Publish/subscribe to topics
   - Call services and action servers

4. **Package Management**
   - Create ROS 2 packages
   - Manage dependencies with rosdep
   - Configure package.xml and CMakeLists.txt

## Dependencies

### System Packages
- `ros-jazzy-desktop` or `ros-iron-desktop`
- `python3-colcon-common-extensions`
- `python3-rosdep`

### Python Packages
- None (uses system ROS 2 Python packages)

## API Contract

See [API Contract](../../specs/001-physical-ai-humanoid/contracts/skills/ros2_core-api.json) for complete API specification.

## Usage

See [Usage Guide](usage.md) for detailed examples.

## Testing

```bash
pytest tests/ --cov=src
```

## Maintainers

- Physical AI & Humanoid Robotics Project

## License

MIT License - see [LICENSE](../../LICENSE)
