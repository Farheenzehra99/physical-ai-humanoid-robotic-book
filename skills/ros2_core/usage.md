# ROS2 Core - Usage Guide

## Example 1: Setup ROS2 Environment

```python
from skills.ros2_core.src import ROS2CoreSkill

skill = ROS2CoreSkill()

# Setup ROS2 environment
result = skill.setup_ros2_environment(
    ros_distro="jazzy",
    workspace_name="humanoid_ws"
)

print(f"Workspace created at: {result['workspace_path']}")
```

## Example 2: Create ROS2 Package

```python
# Create a new ROS2 package
result = skill.create_ros2_package(
    workspace_path="/home/user/humanoid_ws",
    package_name="humanoid_control",
    dependencies=["rclpy", "std_msgs", "geometry_msgs"]
)

print(f"Package created: {result['package_path']}")
```

## Example 3: Build Workspace

```python
# Build the ROS2 workspace
result = skill.build_workspace(
    workspace_path="/home/user/humanoid_ws",
    packages=["humanoid_control"],
    parallel_jobs=4
)

print(f"Build success: {result['build_success']}")
```

## CLI Usage

```bash
# Setup ROS2 environment
python -m skills.ros2_core.src.main setup-environment --distro jazzy

# Create package
python -m skills.ros2_core.src.main create-package --name my_package

# Build workspace
python -m skills.ros2_core.src.main build-workspace --path ~/my_ws
```

## Error Handling

```python
try:
    result = skill.setup_ros2_environment(ros_distro="invalid")
except Exception as e:
    print(f"Setup failed: {e}")
```

## Common Errors

| Error Code | Description | Solution |
|------------|-------------|----------|
| `ROS_DISTRO_NOT_FOUND` | ROS distribution not installed | Install ROS2 Jazzy or Iron |
| `WORKSPACE_EXISTS` | Workspace already exists | Use different name or delete existing |
| `BUILD_FAILED` | Colcon build failed | Check package dependencies |
