"""Unit tests for ROS2 Core Skill - API Contract Validation."""
import pytest
from skills.ros2_core.src.main import ROS2CoreSkill


def test_setup_ros2_environment_validates_api_contract():
    """Test that setup_ros2_environment follows API contract."""
    skill = ROS2CoreSkill()
    result = skill.setup_ros2_environment("jazzy", "test_ws")

    # API contract requires: workspace_path, sourcing_command, status
    assert "workspace_path" in result
    assert "sourcing_command" in result
    assert "status" in result
    assert result["status"] == "success"


def test_create_ros2_package_validates_api_contract():
    """Test that create_ros2_package follows API contract."""
    skill = ROS2CoreSkill()
    # This will fail until implementation is complete
    assert hasattr(skill, 'create_ros2_package')


def test_build_workspace_validates_api_contract():
    """Test that build_workspace follows API contract."""
    skill = ROS2CoreSkill()
    # This will fail until implementation is complete
    assert hasattr(skill, 'build_workspace')
