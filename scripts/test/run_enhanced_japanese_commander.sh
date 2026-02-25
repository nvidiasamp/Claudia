#!/bin/bash
# Enhanced Interactive Japanese Commander Startup Script
# Generated: 2025-07-10
# Purpose: Launch the LLM-integrated intelligent Japanese robot control interface

set -e

# Color definitions
RED='\033[91m'
GREEN='\033[92m'
YELLOW='\033[93m'
BLUE='\033[94m'
PURPLE='\033[95m'
CYAN='\033[96m'
WHITE='\033[97m'
BOLD='\033[1m'
END='\033[0m'

SCRIPT_NAME="Enhanced Japanese Commander v3.2"
TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')

echo -e "${BOLD}${CYAN}$SCRIPT_NAME Startup Script${END}"
echo -e "${CYAN}Time: $TIMESTAMP${END}"
echo -e "${CYAN}======================================${END}"

# Pre-startup check function
pre_startup_check() {
    echo -e "\n${BLUE}Pre-startup checks...${END}"

    # Check current time and system status
    echo -e "Current time: $(date '+%Y-%m-%d %H:%M:%S %Z')"
    echo -e "Current directory: $(pwd)"
    echo -e "Disk usage: $(df . | tail -1 | awk '{print $5}')"
    echo -e "Memory usage: $(free | grep Mem | awk '{printf "%.0f%%", $3/$2 * 100.0}')"

    # Check project root directory
    if [ ! -f "README.md" ] || [ ! -d "src/claudia" ]; then
        echo -e "${RED}Error: Please run this script from the project root directory${END}"
        exit 1
    fi

    echo -e "${GREEN}Basic environment check passed${END}"
}

# Python environment check
check_python_env() {
    echo -e "\n${BLUE}Python environment check...${END}"

    # Check Python version
    if ! command -v python3 &> /dev/null; then
        echo -e "${RED}Python3 is not installed${END}"
        exit 1
    fi

    PYTHON_VERSION=$(python3 --version 2>&1 | cut -d' ' -f2)
    echo -e "Python version: $PYTHON_VERSION"

    # Check required Python modules
    REQUIRED_MODULES=("asyncio" "requests" "json" "pathlib")
    for module in "${REQUIRED_MODULES[@]}"; do
        if python3 -c "import $module" 2>/dev/null; then
            echo -e "[OK] $module"
        else
            echo -e "${RED}[MISSING] $module module${END}"
            exit 1
        fi
    done

    echo -e "${GREEN}Python environment check passed${END}"
}

# LLM service check
check_llm_service() {
    echo -e "\n${BLUE}LLM service check...${END}"

    # Check Ollama service
    if curl -s http://127.0.0.1:11434/api/tags > /dev/null 2>&1; then
        echo -e "${GREEN}Ollama service is running${END}"

        # Check available models
        MODELS=$(curl -s http://127.0.0.1:11434/api/tags | python3 -c "
import json, sys
try:
    data = json.load(sys.stdin)
    models = [model['name'] for model in data.get('models', [])]
    print('Available models:', ', '.join(models) if models else 'none')

    # Check if claudia-v3.2:3b model exists
    claudia_models = [m for m in models if 'claudia-v3.2:3b' in m]
    if claudia_models:
        print('[OK] Claudia optimized model ready:', ', '.join(claudia_models))
    else:
        print('[WARN] Claudia optimized model (claudia-v3.2:3b) not found')
except:
    print('Failed to parse model list')
")
        echo -e "$MODELS"

    else
        echo -e "${YELLOW}Ollama service is not running or unreachable${END}"
        echo -e "${YELLOW}   LLM functionality may be limited, but the program will continue to start${END}"
    fi
}

# CycloneDDS environment check
check_cyclonedds_env() {
    echo -e "\n${BLUE}CycloneDDS environment check...${END}"

    # Check workspace
    if [ -d "cyclonedds_ws" ]; then
        echo -e "[OK] CycloneDDS workspace exists"

        # Check install directory
        if [ -d "cyclonedds_ws/install" ]; then
            echo -e "[OK] CycloneDDS has been built"

            # Set environment variables
            echo -e "${CYAN}Setting up CycloneDDS environment...${END}"
            source /opt/ros/foxy/setup.bash
            source cyclonedds_ws/install/setup.bash
            export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

            echo -e "[OK] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
        else
            echo -e "${YELLOW}[WARN] CycloneDDS not built, robot control may be limited${END}"
        fi
    else
        echo -e "${YELLOW}[WARN] CycloneDDS workspace does not exist${END}"
    fi
}

# Robot SDK check
check_robot_sdk() {
    echo -e "\n${BLUE}Robot SDK check...${END}"

    if [ -d "unitree_sdk2_python" ]; then
        echo -e "[OK] Unitree SDK2 Python exists"

        # Check SDK import
        if python3 -c "
import sys
sys.path.append('unitree_sdk2_python')
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    print('SDK import successful')
except ImportError as e:
    print(f'SDK import failed: {e}')
" 2>/dev/null | grep -q "SDK import successful"; then
            echo -e "[OK] SDK import test passed"
        else
            echo -e "${YELLOW}[WARN] SDK import failed, real robot control unavailable${END}"
        fi
    else
        echo -e "${YELLOW}[WARN] Unitree SDK2 not found${END}"
    fi
}

# Project file check
check_project_files() {
    echo -e "\n${BLUE}Project file check...${END}"

    REQUIRED_FILES=(
        "src/claudia/interactive_japanese_commander_enhanced.py"
        "src/claudia/robot_controller/action_mapping_engine_real.py"
        "scripts/llm/claudia_llm_interface.py"
    )

    for file in "${REQUIRED_FILES[@]}"; do
        if [ -f "$file" ]; then
            echo -e "[OK] $file"
        else
            echo -e "${RED}[MISSING] $file${END}"
            exit 1
        fi
    done

    echo -e "${GREEN}Core file check passed${END}"
}

# Start interface
start_interface() {
    echo -e "\n${BOLD}${GREEN}Launching enhanced Japanese command interface...${END}"

    # Ensure log directory exists
    mkdir -p logs

    # Launch interface
    echo -e "${CYAN}Starting...${END}"
    python3 src/claudia/interactive_japanese_commander_enhanced.py
}

# Cleanup function
cleanup_on_exit() {
    echo -e "\n${YELLOW}Cleaning up...${END}"

    # Clean temporary files
    find /tmp -name "claudia*" -mtime +1 -delete 2>/dev/null || true

    echo -e "${GREEN}Cleanup complete${END}"
}

# Error handling
handle_error() {
    local exit_code=$?
    local line_number=$1

    echo -e "\n${RED}Error occurred at line $line_number${END}"
    echo -e "${RED}Exit code: $exit_code${END}"
    echo -e "${RED}Time: $(date '+%Y-%m-%d %H:%M:%S')${END}"

    # Record error
    ERROR_LOG="logs/errors/$(date '+%Y%m%d_%H%M%S')_enhanced_japanese_commander_error.log"
    mkdir -p "$(dirname "$ERROR_LOG")"

    {
        echo "Enhanced Japanese Commander Error Report"
        echo "Time: $(date '+%Y-%m-%d %H:%M:%S %Z')"
        echo "Exit Code: $exit_code"
        echo "Line: $line_number"
        echo "Working Directory: $(pwd)"
        echo "Environment:"
        env | grep -E "(ROS|CYCLONE|OLLAMA|UNITREE)" || true
    } > "$ERROR_LOG"

    cleanup_on_exit
    exit $exit_code
}

# Set up error handling
set -e
trap 'handle_error $LINENO' ERR
trap cleanup_on_exit EXIT

# Main execution flow
main() {
    echo -e "${BOLD}Starting pre-launch check sequence...${END}"

    # Execute all checks
    pre_startup_check
    check_python_env
    check_llm_service
    check_cyclonedds_env
    check_robot_sdk
    check_project_files

    echo -e "\n${BOLD}${GREEN}All checks complete, system ready!${END}"

    # Display system summary
    echo -e "\n${BOLD}${PURPLE}System Summary${END}"
    echo -e "Python: $(python3 --version | cut -d' ' -f2)"
    echo -e "LLM service: $(curl -s http://127.0.0.1:11434/api/tags > /dev/null 2>&1 && echo 'Running' || echo 'Offline')"
    echo -e "Target model: claudia-v3.2:3b"
    echo -e "CycloneDDS: $([ -d cyclonedds_ws/install ] && echo 'Built' || echo 'Not built')"
    echo -e "Robot SDK: $([ -d unitree_sdk2_python ] && echo 'Available' || echo 'Not available')"

    # Startup confirmation
    echo -e "\n${YELLOW}Ready to launch Enhanced Japanese Command Interface...${END}"
    read -p "Press Enter to continue, or Ctrl+C to cancel: "

    # Launch
    start_interface
}

# Execute main function
main

echo -e "\n${GREEN}Enhanced Japanese Commander startup script execution complete${END}"
