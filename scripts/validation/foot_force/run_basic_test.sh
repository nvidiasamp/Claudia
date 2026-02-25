#!/bin/bash
# scripts/validation/foot_force/run_basic_test.sh
# Generated: 2025-06-27 14:10:00 CST
# Purpose: Run foot force sensor basic test

set -e

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../../" && pwd)"
VALIDATION_DIR="$SCRIPT_DIR/foot_force_validation"

echo "Foot Force Sensor Basic Test Startup Script"
echo "Start time: $(date '+%Y-%m-%d %H:%M:%S')"
echo "Project root: $PROJECT_ROOT"
echo "Validation directory: $VALIDATION_DIR"

# Check current directory
cd "$PROJECT_ROOT"
echo "Current working directory: $(pwd)"

# Environment check
echo ""
echo "Environment check..."

# Check Python environment
if ! command -v python3 &> /dev/null; then
    echo "Python3 not found"
    exit 1
fi
echo "Python3: $(python3 --version)"

# Check required Python packages
echo "Checking Python dependencies..."
python3 -c "
import sys
required_packages = ['numpy', 'json', 'threading', 'pathlib']
missing_packages = []

for package in required_packages:
    try:
        __import__(package)
        print(f'  {package}: OK')
    except ImportError:
        missing_packages.append(package)
        print(f'  {package}: MISSING')

if missing_packages:
    print(f'Missing packages: {missing_packages}')
    sys.exit(1)
else:
    print('All basic dependencies are installed')
"

# Check Unitree SDK
echo "Checking Unitree SDK..."
python3 -c "
try:
    from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
    print('Unitree SDK2 import successful')
except ImportError as e:
    print(f'Unitree SDK2 import failed: {e}')
    print('Please ensure unitree_sdk2py is properly installed and configured')
    exit(1)
"

# Check validation directory and files
echo "Checking validation files..."
required_files=(
    "$VALIDATION_DIR/foot_force_config.py"
    "$VALIDATION_DIR/data_collector.py"
    "$VALIDATION_DIR/validation_config.json"
    "$VALIDATION_DIR/basic_test.py"
)

for file in "${required_files[@]}"; do
    if [ -f "$file" ]; then
        echo "  $(basename "$file"): OK"
    else
        echo "  File not found: $file"
        exit 1
    fi
done

# Create necessary directories
echo ""
echo "Creating output directories..."
mkdir -p "$VALIDATION_DIR/logs"
mkdir -p "$VALIDATION_DIR/output"
echo "Output directories created"

# Set environment variables
echo ""
echo "Setting environment variables..."
export PYTHONPATH="$VALIDATION_DIR:$PYTHONPATH"
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

echo "PYTHONPATH: $PYTHONPATH"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"

# Check network interface
echo ""
echo "Checking network interface..."
if ip link show eth0 &> /dev/null; then
    echo "eth0 network interface exists"
    ip addr show eth0 | grep -E "inet " || echo "WARNING: eth0 has no IP address configured"
else
    echo "WARNING: eth0 network interface does not exist, will use default configuration"
fi

# Ask for test parameters
echo ""
echo "Test configuration..."

# Default test duration
DEFAULT_DURATION=10
read -p "Enter test duration in seconds (default: ${DEFAULT_DURATION}): " duration
duration=${duration:-$DEFAULT_DURATION}

# Whether to enable debug mode
read -p "Enable debug mode? (y/n, default: n): " debug_mode
debug_mode=${debug_mode:-n}

# Build command arguments
cmd_args="--duration $duration"
if [[ "$debug_mode" =~ ^[Yy]$ ]]; then
    cmd_args="$cmd_args --debug"
fi

# Display final configuration
echo ""
echo "Test configuration summary:"
echo "   Test duration: ${duration} seconds"
echo "   Debug mode: $([ "$debug_mode" = 'y' ] && echo 'enabled' || echo 'disabled')"
echo "   Output directory: $VALIDATION_DIR/output"
echo "   Log directory: $VALIDATION_DIR/logs"

# Confirm execution
echo ""
read -p "Start test execution? (y/n): " confirm
if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
    echo "Test cancelled"
    exit 0
fi

# Execute test
echo ""
echo "Starting foot force sensor basic test..."
echo "=========================================="

cd "$VALIDATION_DIR"

# Run Python test script
if python3 basic_test.py $cmd_args; then
    echo ""
    echo "=========================================="
    echo "Foot force sensor basic test complete"
    echo "Test results saved to: $VALIDATION_DIR/output/"
    echo "Log files: $VALIDATION_DIR/logs/"
    echo ""

    # Display generated files
    if [ -d "$VALIDATION_DIR/output" ]; then
        echo "Generated files:"
        find "$VALIDATION_DIR/output" -type f -name "*$(date '+%Y%m%d')*" 2>/dev/null | head -10 | while read file; do
            echo "   - $(basename "$file")"
        done
    fi

    echo ""
    echo "Test completed successfully!"

else
    echo ""
    echo "=========================================="
    echo "Foot force sensor basic test failed"
    echo "Please check log files: $VALIDATION_DIR/logs/"
    echo ""
    echo "Common troubleshooting steps:"
    echo "   1. Check robot connection status"
    echo "   2. Verify network interface configuration"
    echo "   3. Verify Unitree SDK installation"
    echo "   4. Review detailed error logs"
    echo ""
    exit 1
fi

echo ""
echo "Test end time: $(date '+%Y-%m-%d %H:%M:%S')"
