#!/bin/bash

# 2025-06-26
# Unitree CycloneDDS Dependencies Installation Script
# Based on the official unitree_ros2 documentation installation steps

set -e

echo "Installing Unitree CycloneDDS Dependencies..."

# 1. Install ROS2 Foxy CycloneDDS dependencies
echo "Installing ROS2 Foxy CycloneDDS packages..."
sudo apt update
sudo apt install -y ros-foxy-rmw-cyclonedds-cpp
sudo apt install -y ros-foxy-rosidl-generator-dds-idl

# 2. Install build tools
echo "Installing build tools..."
sudo apt install -y cmake build-essential

# 3. Clone and build CycloneDDS (for unitree_sdk2py)
echo "Cloning and building CycloneDDS..."
cd ~
if [ ! -d "cyclonedds" ]; then
    git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
fi

cd cyclonedds
if [ ! -d "build" ]; then
    mkdir build install
fi

cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install

echo "Setting up environment..."
# Set environment variables (add to .bashrc)
if ! grep -q "CYCLONEDDS_HOME" ~/.bashrc; then
    echo 'export CYCLONEDDS_HOME="$HOME/cyclonedds/install"' >> ~/.bashrc
fi

# 4. Source ROS2 environment
echo "Sourcing ROS2 environment..."
source /opt/ros/foxy/setup.bash

echo "CycloneDDS dependencies installation completed!"
echo ""
echo "Next steps:"
echo "   1. source ~/.bashrc  # Reload environment variables"
echo "   2. Clone unitree_sdk2py and unitree_ros2 repositories"
echo "   3. Build cyclonedds_ws workspace: cd cyclonedds_ws && colcon build"
echo ""
echo "Environment variables set:"
echo "   CYCLONEDDS_HOME=$HOME/cyclonedds/install"
