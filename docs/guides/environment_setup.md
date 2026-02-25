# Claudia Robot Environment Configuration Guide

## Overview

This guide provides detailed instructions for the complete environment configuration of the Claudia robot project, with particular focus on the critical DDS communication environment setup.

## System Requirements

### Hardware Requirements
- **Compute Platform**: NVIDIA Jetson Orin NX
- **Robot**: Unitree Go2 R&D Plus
- **Memory**: At least 8GB RAM
- **Storage**: At least 64GB available space

### Software Requirements
- **Operating System**: Ubuntu 20.04.5 LTS (aarch64)
- **ROS Version**: ROS2 Foxy
- **Python Version**: Python 3.8+
- **DDS Implementation**: CycloneDDS

## Installation Steps

### 1. Basic Environment Preparation

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install basic dependencies
sudo apt install -y \
    curl \
    wget \
    git \
    cmake \
    build-essential \
    python3-pip \
    python3-dev
```

### 2. ROS2 Foxy Installation

```bash
# Set up ROS2 source
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Foxy
sudo apt update
sudo apt install -y ros-foxy-desktop python3-argcomplete
```

### 3. CycloneDDS Workspace Configuration (Critical)

This is the **most critical** configuration step, ensuring the correct DDS communication environment:

```bash
# Enter project root directory
cd ~/claudia

# Create CycloneDDS workspace (if it doesn't exist; skip if already present)
mkdir -p cyclonedds_ws/src
cd cyclonedds_ws

# Note: In the actual project, CycloneDDS was installed via other means
# This documents the workspace structure
```

### 4. Unitree SDK2 Python Installation

```bash
# Clone SDK
cd ~/claudia
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git

# Install dependencies
cd unitree_sdk2_python
pip3 install -e .
```

## **Critical Environment Configuration**

### Correct DDS Environment Setup

**Before running any Unitree-related tests, you must execute the following in this order:**

```bash
# 1. First source the CycloneDDS workspace
source cyclonedds_ws/install/setup.bash

# 2. Then set the RMW implementation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 3. Verify environment variables
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
```

**Important Notes:**
- Must execute in the order shown above
- It is **not** `rmw_cyclonedx_cpp`, but `rmw_cyclonedds_cpp`
- Each new terminal requires re-setup
- Incorrect settings will cause DDS library loading failures

### Automated Environment Setup Script

Create a convenient environment setup script:

```bash
# Create environment setup script
cat > scripts/setup/setup_environment.sh << 'EOF'
#!/bin/bash
# Claudia Robot Environment Setup Script
# Generated: 2025-06-26 18:40:00

echo "Setting up Claudia robot environment..."

# Check project root directory
if [ ! -f "pyproject.toml" ]; then
    echo "Please run this script from the project root directory"
    exit 1
fi

# Set up ROS2 environment
source /opt/ros/foxy/setup.bash
echo "ROS2 Foxy environment loaded"

# Set up CycloneDDS workspace
if [ -f "cyclonedds_ws/install/setup.bash" ]; then
    source cyclonedds_ws/install/setup.bash
    echo "CycloneDDS workspace loaded"
else
    echo "WARNING: CycloneDDS workspace not found, please build first"
fi

# Set RMW implementation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "RMW_IMPLEMENTATION set to: $RMW_IMPLEMENTATION"

# Set Python path
export PYTHONPATH=$PYTHONPATH:~/claudia/unitree_sdk2_python
echo "Python path set"

echo "Environment setup complete! Ready to run Unitree tests"
EOF

chmod +x scripts/setup/setup_environment.sh
```

### Using the Environment Setup Script

```bash
# Run in project root directory
source scripts/setup/setup_environment.sh
```

## Environment Verification

### Verify DDS Communication

```bash
# Set up environment
source scripts/setup/setup_environment.sh

# Run basic connection test
python3 test/hardware/test_unitree_connection.py

# Run communication performance test
python3 test/hardware/test_communication_performance.py
```

### Expected Output Example

After correct configuration, you should see:
```
Successfully imported all required modules
Environment variable set: rmw_cyclonedds_cpp
Initializing Sport client...
Sport client initialization successful
Communication performance test starting...
```

## Common Issues

### Issue 1: DDS Library Loading Failure
```
OSError: libddsc.so.0: cannot open shared object file
```

**Solution:**
```bash
# Ensure environment is set up in the correct order
source cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Issue 2: Environment Variable Name Error
```
Set rmw_cyclonedx_cpp but still failing
```

**Solution:**
```bash
# Use the correct environment variable name
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Note: it's cyclonedds, not cyclonedx
```

### Issue 3: Workspace Not Built
```
cyclonedds_ws/install/setup.bash: No such file or directory
```

**Solution:**
```bash
cd cyclonedds_ws
colcon build --symlink-install
```

## Performance Benchmarks

### Communication Latency Benchmarks (Task 3.7 Results)

- **Lightweight Command (Sit)**: Average 36.64ms, 97% <50ms
- **Complex Action Command (StandUp)**: Average 640.87ms
- **Moderate Complexity Command (Damp)**: Average 214.72ms

## Environment Reset

If environment issues occur, you can reset:

```bash
# Clean build files
rm -rf cyclonedds_ws/build cyclonedds_ws/install cyclonedds_ws/log

# Rebuild
cd cyclonedds_ws
colcon build --symlink-install

# Re-setup environment
source scripts/setup/setup_environment.sh
```

## Configuration Checklist

Use this checklist to verify environment configuration:

- [ ] Ubuntu 20.04 installed
- [ ] ROS2 Foxy installed
- [ ] CycloneDDS workspace built
- [ ] Unitree SDK2 Python installed
- [ ] Environment setup script runs successfully
- [ ] Basic connection test passes
- [ ] Communication performance test passes

---

**Document Updated**: 2025-06-26 18:40:00
**Applicable Version**: Claudia v0.1.0
**Test Platform**: NVIDIA Jetson Orin NX + Unitree Go2
