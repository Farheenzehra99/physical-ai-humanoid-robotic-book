"""Unit tests for ROS2 Core Skill."""

import pytest
from pathlib import Path
from unittest.mock import Mock, patch, MagicMock
from skills.ros2_core.src.main import ROS2CoreSkill


class TestROS2CoreSkill:
    """Test suite for ROS2CoreSkill."""

    def setup_method(self):
        """Setup test fixtures."""
        self.skill = ROS2CoreSkill()

    def test_init(self):
        """Test skill initialization."""
        assert self.skill is not None
        assert "jazzy" in self.skill.supported_distros
        assert "iron" in self.skill.supported_distros

    def test_setup_ros2_environment_invalid_distro(self):
        """Test setup with invalid ROS distro."""
        with pytest.raises(ValueError, match="Unsupported ROS distro"):
            self.skill.setup_ros2_environment("invalid_distro")

    @patch("pathlib.Path.exists")
    @patch("pathlib.Path.mkdir")
    def test_setup_ros2_environment_success(self, mock_mkdir, mock_exists):
        """Test successful ROS2 environment setup."""
        mock_exists.return_value = True

        result = self.skill.setup_ros2_environment("jazzy", "test_ws")

        assert result["status"] == "success"
        assert "test_ws" in result["workspace_path"]
        assert "jazzy" in result["sourcing_command"]

    @patch("pathlib.Path.exists")
    def test_setup_ros2_environment_not_installed(self, mock_exists):
        """Test setup when ROS2 is not installed."""
        mock_exists.return_value = False

        with pytest.raises(RuntimeError, match="not installed"):
            self.skill.setup_ros2_environment("jazzy")

    @patch("subprocess.run")
    @patch("pathlib.Path.exists")
    def test_create_ros2_package(self, mock_exists, mock_run):
        """Test ROS2 package creation."""
        mock_exists.return_value = True
        mock_run.return_value = MagicMock(returncode=0)

        result = self.skill.create_ros2_package(
            workspace_path="/tmp/test_ws",
            package_name="test_pkg",
            dependencies=["rclpy"],
        )

        assert "test_pkg" in result["package_path"]
        mock_run.assert_called_once()

    @patch("pathlib.Path.exists")
    def test_create_ros2_package_workspace_not_found(self, mock_exists):
        """Test package creation when workspace doesn't exist."""
        mock_exists.return_value = False

        with pytest.raises(FileNotFoundError):
            self.skill.create_ros2_package("/nonexistent", "test_pkg")

    @patch("subprocess.run")
    @patch("pathlib.Path.exists")
    def test_build_workspace_success(self, mock_exists, mock_run):
        """Test successful workspace build."""
        mock_exists.return_value = True
        mock_run.return_value = MagicMock(
            returncode=0, stdout="Build success", stderr=""
        )

        result = self.skill.build_workspace("/tmp/test_ws", parallel_jobs=2)

        assert result["build_success"] is True
        assert "Build success" in result["stdout"]

    @patch("subprocess.run")
    @patch("pathlib.Path.exists")
    def test_build_workspace_failure(self, mock_exists, mock_run):
        """Test failed workspace build."""
        mock_exists.return_value = True
        mock_run.return_value = MagicMock(returncode=1, stdout="", stderr="Build failed")

        result = self.skill.build_workspace("/tmp/test_ws")

        assert result["build_success"] is False
        assert "Build failed" in result["stderr"]


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
