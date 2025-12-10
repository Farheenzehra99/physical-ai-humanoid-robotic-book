#!/bin/bash
# Verify ROS 2 installation

set -e

echo "Verifying ROS 2 setup..."

# Check ROS 2 version
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS 2 not found. Please install ROS 2 Jazzy."
    exit 1
fi

echo "✅ ROS 2 version: $(ros2 --version)"

# Check workspace
if [ ! -d "$HOME/ros2_ws" ]; then
    echo "❌ Workspace not found at ~/ros2_ws"
    exit 1
fi

echo "✅ Workspace found"

echo "All checks passed!"
