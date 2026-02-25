#!/bin/bash
# Claudia Robot Environment Setup Script
# Generated: 2025-06-26 18:40:00
# Purpose: Automatically set up the runtime environment for the Claudia robot project

set -e

echo "Setting up Claudia robot environment..."

# Check project root directory
if [ ! -f "pyproject.toml" ]; then
    echo "Please run this script from the project root directory"
    echo "Current directory: $(pwd)"
    echo "Please switch to the directory containing pyproject.toml"
    exit 1
fi

echo "Project root directory: $(pwd)"

# Set up ROS2 environment
if [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
    echo "ROS2 Foxy environment loaded"
else
    echo "ROS2 Foxy not found, please ensure it is correctly installed"
fi

# Set up CycloneDDS workspace
if [ -f "cyclonedds_ws/install/setup.bash" ]; then
    source cyclonedds_ws/install/setup.bash
    echo "CycloneDDS workspace loaded"
else
    echo "CycloneDDS workspace not found (cyclonedds_ws/install/setup.bash)"
    echo "If running for the first time, this is normal. Please build the workspace first"
fi

# Set RMW implementation - this is critical!
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "RMW_IMPLEMENTATION set to: $RMW_IMPLEMENTATION"

# Set Python path
if [ -d "unitree_sdk2_python" ]; then
    export PYTHONPATH=$PYTHONPATH:$(pwd)/unitree_sdk2_python
    echo "Python path set: $(pwd)/unitree_sdk2_python"
else
    echo "unitree_sdk2_python directory not found"
fi

# Verify key environment variables
echo ""
echo "Environment variable verification:"
echo "   ROS_DISTRO: ${ROS_DISTRO:-not set}"
echo "   RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-not set}"
echo "   PYTHONPATH: ${PYTHONPATH:-not set}"

# Check key files
echo ""
echo "Key file check:"
[ -f "cyclonedds_ws/install/setup.bash" ] && echo "   [OK] CycloneDDS setup.bash" || echo "   [MISSING] CycloneDDS setup.bash"
[ -d "unitree_sdk2_python" ] && echo "   [OK] Unitree SDK2 Python" || echo "   [MISSING] Unitree SDK2 Python"
[ -d "test/hardware" ] && echo "   [OK] Hardware test directory" || echo "   [MISSING] Hardware test directory"

echo ""
echo "Environment setup complete!"
echo ""
echo "You can now run:"
echo "   python3 test/hardware/test_unitree_connection.py           # Basic connection test"
echo "   python3 test/hardware/test_basic_control_commands.py       # Basic control command test"
echo "   python3 test/hardware/test_communication_performance.py    # Communication performance test"
echo "   python3 test/run_tests.py --type hardware                  # Run all hardware tests"
