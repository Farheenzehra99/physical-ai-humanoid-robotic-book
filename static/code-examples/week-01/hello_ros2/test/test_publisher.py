"""Tests for minimal publisher."""

import pytest


def test_publisher_imports():
    """Test that publisher module can be imported."""
    from hello_ros2.publisher import MinimalPublisher
    assert MinimalPublisher is not None


def test_publisher_initialization():
    """Test publisher node initialization."""
    # This would require rclpy context - skip for now
    pytest.skip("Requires ROS 2 runtime")
