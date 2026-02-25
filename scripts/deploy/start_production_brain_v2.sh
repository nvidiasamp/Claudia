#!/bin/bash
# Claudia Production Brain Launch Script v2.0 (Track A Optimized)
# Fix: Explicit source of ROS2, Ollama pre-check, warmup, DDS validation

set -e

echo "=================================="
echo "Claudia Production Brain v2.0"
echo "=================================="
echo ""

# Working directory
cd $HOME/claudia

# ========================
# 1. ROS2 Foxy Environment
# ========================
echo "Loading ROS2 Foxy environment..."
if [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
    echo "   ROS2 Foxy loaded"
else
    echo "   ROS2 Foxy not found (will use simulation mode)"
fi

# Project ROS2 configuration
if [ -f ".env.ros2" ]; then
    source .env.ros2
    echo "   Project ROS2 configuration loaded"
fi

# ========================
# 2. CycloneDDS Configuration (fix DDS symbol issue)
# ========================
echo "Configuring CycloneDDS (rmw_cyclonedds_cpp + eth0)..."
# Use independently compiled CycloneDDS 0.10.x (resolves symbol mismatch issue)
export CYCLONEDDS_HOME=$HOME/cyclonedds/install
export LD_LIBRARY_PATH=$HOME/cyclonedds/install/lib:$LD_LIBRARY_PATH

# ROS2 configuration (still uses ROS2 Foxy's CycloneDDS RMW)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

# Inline DDS configuration (eth0 fixed)
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="eth0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'

echo "   DDS configuration complete (eth0, Domain 0)"

# ========================
# 3. Network Validation
# ========================
echo "Validating network..."
LOCAL_IP=$(ip addr show eth0 2>/dev/null | grep '192.168.123' | awk '{print $2}' | cut -d'/' -f1 || echo "not configured")
echo "   Local IP: $LOCAL_IP"
echo "   Robot IP: 192.168.123.161"
echo "   ROS Domain: $ROS_DOMAIN_ID"

if ping -c 1 -W 1 192.168.123.161 >/dev/null 2>&1; then
    echo "   Robot network is reachable"
else
    echo "   Cannot ping robot (may not be connected)"
fi

# ========================
# 4. Ollama Service & Models
# ========================
echo "Checking Ollama service..."
if ! curl -s http://localhost:11434/api/tags >/dev/null 2>&1; then
    echo "   Ollama not running, attempting to start..."
    nohup ollama serve >/dev/null 2>&1 &
    sleep 3

    if curl -s http://localhost:11434/api/tags >/dev/null 2>&1; then
        echo "   Ollama started"
    else
        echo "   Failed to start Ollama"
        exit 1
    fi
else
    echo "   Ollama is running"
fi

# Check models
echo "Checking LLM models..."
for model in "claudia-go2-3b:v11.2" "claudia-go2-7b:v7"; do
    if ollama list | grep -q "$model"; then
        echo "   $model"
    else
        echo "   $model does not exist, attempting to pull..."
        ollama pull "$model" 2>/dev/null || echo "   Pull failed, skipping"
    fi
done

# LLM warmup (avoid high latency on first call)
echo "Warming up LLM..."
python3 - <<'PYWARMUP' 2>/dev/null || echo "   Skipping warmup (requests library not available)"
try:
    import requests, json
    resp = requests.post(
        "http://localhost:11434/api/generate",
        json={"model":"claudia-go2-3b:v11.2","prompt":"test","stream":False},
        timeout=10
    )
    if resp.status_code == 200:
        print("   LLM warmup complete")
except:
    pass
PYWARMUP

# ========================
# 5. Python Dependencies
# ========================
echo "Checking Python dependencies..."
python3 -c "import ollama" 2>/dev/null && echo "   ollama library" || {
    echo "   ollama library not installed, attempting to install..."
    pip3 install ollama --quiet 2>/dev/null || echo "   Installation failed"
}

# ========================
# 6. Launch
# ========================
echo ""
echo "=================================="
echo "Please select run mode:"
echo "1) Simulation mode (safe testing)"
echo "2) Real hardware mode (connect to Go2)"
echo "=================================="
echo ""
read -p "Select [1/2]: " choice

case $choice in
    1)
        echo ""
        echo "Starting simulation mode..."
        echo ""
        python3 production_commander.py
        ;;
    2)
        echo ""
        echo "Real hardware mode - Please confirm:"
        echo "   1. Robot is powered on and connected (IP:192.168.123.161)"
        echo "   2. At least 2m of open space around"
        echo "   3. Unitree App is closed (to avoid occupancy conflicts)"
        echo ""
        read -p "Confirm to continue? [y/N]: " confirm
        if [[ $confirm =~ ^[yY]$ ]]; then
            echo ""
            echo "Starting real hardware mode..."
            echo ""
            python3 production_commander.py --hardware
        else
            echo "Cancelled"
            exit 0
        fi
        ;;
    *)
        echo "Invalid selection"
        exit 1
        ;;
esac
