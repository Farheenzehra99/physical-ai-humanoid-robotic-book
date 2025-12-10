#!/bin/bash
# ROS 2 Jazzy Installation Script for Ubuntu 22.04

set -e

echo "Installing ROS 2 Jazzy..."

# Add ROS 2 repository
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros2bag \
  ros-jazzy-rqt* python3-colcon-common-extensions

# Source setup
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

echo "ROS 2 Jazzy installed successfully!"
echo "Please restart your terminal or run: source ~/.bashrc"
