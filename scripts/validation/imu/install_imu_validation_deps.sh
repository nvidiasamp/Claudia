#!/bin/bash
# Generated: 2025-01-27 11:55:00
# Purpose: Install dependencies for IMU validation system
# Platform: Jetson Xavier NX Ubuntu 18.04

set -e

echo "IMU Validation System Dependency Installation Script"
echo "=========================================="

# Check runtime environment
echo "Current time: $(date '+%Y-%m-%d %H:%M:%S %Z')"
echo "Current directory: $(pwd)"
echo "Disk usage: $(df . | tail -1 | awk '{print $5}')"
echo "Memory usage: $(free | grep Mem | awk '{printf "%.0f%%", $3/$2 * 100.0}')"

# Error handling function
handle_error() {
    local exit_code=$?
    local line_number=$1
    echo "Error occurred at line $line_number"
    echo "Exit code: $exit_code"
    echo "Time: $(date '+%Y-%m-%d %H:%M:%S')"
    exit $exit_code
}

trap 'handle_error $LINENO' ERR

echo ""
echo "Checking and installing Python dependencies..."

# Ensure pip is available
if ! command -v pip3 &> /dev/null; then
    echo "Installing pip3..."
    sudo apt update
    sudo apt install -y python3-pip
fi

# Check and install required Python packages
REQUIRED_PACKAGES=(
    "numpy>=1.19.0"
    "matplotlib>=3.3.0"
    "scipy>=1.5.0"
    "pyserial>=3.4"
    "pyyaml>=5.3.0"
    "jsonschema>=3.2.0"
)

echo "Checking Python package dependencies..."
for package in "${REQUIRED_PACKAGES[@]}"; do
    package_name=$(echo $package | cut -d'>' -f1 | cut -d'=' -f1)
    echo -n "  Checking $package_name ... "

    if python3 -c "import $package_name" 2>/dev/null; then
        echo "Installed"
    else
        echo "Not installed, installing..."
        pip3 install "$package" --user
        echo "Installation complete"
    fi
done

echo ""
echo "Checking Unitree SDK2 Python dependency..."

# Check unitree_sdk2py
echo -n "  Checking unitree_sdk2py ... "
if python3 -c "import unitree_sdk2py" 2>/dev/null; then
    echo "Installed"
else
    echo "Not installed"
    echo "    Please follow these steps to manually install unitree_sdk2py:"
    echo "    1. git clone https://github.com/unitreerobotics/unitree_sdk2_python.git"
    echo "    2. cd unitree_sdk2_python"
    echo "    3. pip3 install -e ."
fi

echo ""
echo "Checking system dependencies..."

# Check required system tools
SYSTEM_TOOLS=("git" "wget" "curl")
for tool in "${SYSTEM_TOOLS[@]}"; do
    echo -n "  Checking $tool ... "
    if command -v "$tool" &> /dev/null; then
        echo "Available"
    else
        echo "Not found, installing..."
        sudo apt install -y "$tool"
        echo "Installation complete"
    fi
done

echo ""
echo "Verifying installation results..."

# Create verification script
cat > /tmp/verify_imu_deps.py << 'EOF'
#!/usr/bin/env python3
import sys

def check_import(module_name, package_name=None):
    try:
        __import__(module_name)
        print(f"  {package_name or module_name} - OK")
        return True
    except ImportError as e:
        print(f"  {package_name or module_name} - MISSING: {e}")
        return False

print("Verifying Python dependencies:")
all_ok = True
all_ok &= check_import("numpy", "NumPy")
all_ok &= check_import("matplotlib", "Matplotlib")
all_ok &= check_import("scipy", "SciPy")
all_ok &= check_import("serial", "PySerial")
all_ok &= check_import("yaml", "PyYAML")
all_ok &= check_import("jsonschema", "JsonSchema")

print("\nVerifying Unitree SDK:")
all_ok &= check_import("unitree_sdk2py", "Unitree SDK2 Python")

if all_ok:
    print("\nAll dependencies installed successfully!")
    sys.exit(0)
else:
    print("\nSome dependencies are missing, please check the installation")
    sys.exit(1)
EOF

python3 /tmp/verify_imu_deps.py
VERIFICATION_RESULT=$?

# Clean up temporary files
rm -f /tmp/verify_imu_deps.py

echo ""
echo "Installation summary:"
echo "  - Python dependencies: checked and installed"
echo "  - System tools: checked and installed"
echo "  - Unitree SDK: please confirm installation manually"

if [ $VERIFICATION_RESULT -eq 0 ]; then
    echo ""
    echo "IMU validation system dependency installation complete!"
    echo "You can now run: ./run_imu_validation.sh"
else
    echo ""
    echo "Some dependencies may need manual handling"
    echo "Please check the error messages above and manually install missing dependencies"
fi

echo ""
echo "Installation completed at: $(date '+%Y-%m-%d %H:%M:%S %Z')"
