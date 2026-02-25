#!/bin/bash
# SDK Exclusive Mode Launcher - Ensure APP does not interfere

echo "===================================="
echo "Claudia SDK Exclusive Mode Launcher"
echo "===================================="
echo ""
echo "IMPORTANT:"
echo "SDK and APP cannot control the robot simultaneously!"
echo "This is Unitree's safety design, not a malfunction."
echo ""
echo "Pre-launch checklist:"
echo "[ ] 1. Unitree Go APP is completely closed"
echo "[ ] 2. Robot has been restarted (clear APP connection)"
echo "[ ] 3. Network connection is normal (192.168.123.x)"
echo ""

read -p "Confirm all conditions are met? [y/N]: " confirm
if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
    echo ""
    echo "Please follow these steps:"
    echo "1. Close the Unitree Go APP on your phone"
    echo "2. Hold the robot's power button to restart"
    echo "3. Wait 30 seconds before running this script again"
    exit 1
fi

echo ""
echo "Configuring SDK environment..."

# Set up CycloneDDS environment
export CYCLONEDDS_HOME="$HOME/claudia/cyclonedds/install"
export LD_LIBRARY_PATH="${CYCLONEDDS_HOME}/lib:$LD_LIBRARY_PATH"
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"

# Use inline XML configuration (officially recommended)
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name="eth0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'

echo "Environment configuration complete"
echo ""

# Check network
echo "Checking network connection..."
if ping -c 1 192.168.123.161 > /dev/null 2>&1; then
    echo "Robot network is reachable (192.168.123.161)"
else
    echo "Cannot connect to robot, please check network"
    echo "   Hint: Ensure you are on the same subnet 192.168.123.x"
    exit 1
fi

echo ""
echo "Starting SDK exclusive control..."
echo "----------------------------------------"
echo "Hint: If error 3103 is returned, the APP is still occupying"
echo "      the robot. Restart the robot and ensure the APP is closed."
echo "----------------------------------------"
echo ""

# Start production brain
cd $HOME/claudia
python3 production_commander.py

echo ""
echo "Session ended"
echo ""
echo "To switch to APP control:"
echo "1. Stop this program first"
echo "2. Restart the robot"
echo "3. Open the APP to connect"
