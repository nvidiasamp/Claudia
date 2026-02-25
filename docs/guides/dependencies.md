# Third-Party Dependency Acquisition Guide

## Overview

To keep the GitHub repository lightweight, Claudia project's large third-party dependencies are not included in version control. This guide explains how to obtain and set up these required dependencies.

## Dependencies That Need Manual Acquisition

### 1. CycloneDDS
**Size**: ~101MB
**Purpose**: DDS communication middleware

```bash
# Clone CycloneDDS
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds && mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install
```

### 2. Unitree SDK2 Python
**Size**: ~2.3MB
**Purpose**: Unitree robot Python SDK

```bash
# Clone Unitree SDK2 Python
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip3 install -e .
```

### 3. CycloneDDS Workspace
**Size**: ~413MB
**Purpose**: ROS2 message definitions and build artifacts

```bash
# Create cyclonedds_ws workspace
mkdir -p cyclonedds_ws/src
cd cyclonedds_ws/src

# Clone Unitree ROS2 packages
git clone https://github.com/unitreerobotics/unitree_ros2.git
git clone https://github.com/unitreerobotics/unitree_sdk2.git

# Build workspace
cd ..
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```

## Automated Setup Scripts

The project provides automated setup scripts to simplify the dependency acquisition process:

```bash
# Use the project-provided environment setup script
source scripts/setup/setup_environment.sh

# Or run individual installation scripts separately
bash scripts/setup/install_cyclonedds_deps.sh
bash scripts/setup/install_unitree_sdks.sh
bash scripts/setup/setup_cyclonedds_workspace.sh
```

## Important Notes

1. **Storage Location**: All dependencies should be placed in the project root directory
2. **Environment Variables**: Running scripts will automatically set necessary environment variables
3. **Version Compatibility**: Ensure you use the specified version branches for compatibility
4. **Disk Space**: Approximately 500MB of total disk space required

## Verify Installation

After installation, you can verify using the following commands:

```bash
# Verify environment configuration
source scripts/setup/setup_environment.sh

# Run connection test
python3 test/hardware/test_unitree_connection.py

# Run communication performance test
python3 test/hardware/test_communication_performance.py
```

## Troubleshooting

If you encounter issues, refer to:
- [Troubleshooting Guide](../troubleshooting/README.md)
- [Environment Configuration Guide](environment_setup.md)
- [Task 3 Completion Report](../tasks/task-3-completed.md) - Contains detailed installation verification process
