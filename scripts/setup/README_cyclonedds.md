# CycloneDDS Environment Configuration Guide

## Background

The Unitree robot SDK uses CycloneDDS 0.10.x for communication. The default pip-installed version has symbol compatibility issues, resulting in the following error:

```
ImportError: undefined symbol: ddsi_sertype_v0
```

## Solution

This directory provides complete CycloneDDS environment configuration scripts that resolve the version compatibility issue.

## Usage

### 1. Basic Configuration (Run Before Each Use)

```bash
# Run from the project root directory
source scripts/setup/setup_cyclonedds.sh
```

### 2. Configure and Test

```bash
# Configure environment and test unitree_sdk2py import
source scripts/setup/setup_cyclonedds.sh --test
```

### 3. Add to Shell Startup Script (Optional)

To automatically configure the environment each time a terminal is opened:

```bash
echo "source ~/claudia/scripts/setup/setup_cyclonedds.sh" >> ~/.bashrc
```

**Note**: Only add to ~/.bashrc after confirming the configuration is stable.

## Requirements

- CycloneDDS 0.10.x compiled and installed at `~/cyclonedds/install`
- unitree_sdk2py properly installed
- ROS2 Foxy environment

## Verification

Run the following Python code to verify the configuration:

```python
import unitree_sdk2py
from unitree_sdk2py.core.channel import ChannelSubscriber
print("CycloneDDS environment configuration successful")
```

## Troubleshooting

### 1. "CycloneDDS not found" Error

```bash
# Check if CycloneDDS has been compiled
ls ~/cyclonedds/install/lib/libddsc.so
```

If the file does not exist, CycloneDDS needs to be recompiled:

```bash
cd ~/cyclonedds
rm -rf build install
mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install
```

### 2. "Import failed" Error

```bash
# Reinstall unitree_sdk2py
cd ~/unitree_sdk2_python
pip3 uninstall unitree_sdk2py cyclonedx -y
source ~/claudia/scripts/setup/setup_cyclonedds.sh
pip3 install -e .
```

## Related Files

- `setup_cyclonedds.sh` - Main configuration script
- `README_cyclonedds.md` - This documentation
- `~/cyclonedds/install/` - CycloneDDS installation directory
- `~/unitree_sdk2_python/` - Unitree SDK source directory

---

**Last updated**: June 27, 2024
**Applicable versions**: CycloneDDS 0.10.x, ROS2 Foxy, Ubuntu 20.04
