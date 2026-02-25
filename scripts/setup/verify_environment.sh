#!/bin/bash

#1.1
# 2025-06-26
# Environment Verification Script for Claudia Robot System
# Based on TaskMaster-researched Jetson Orin NX environment verification best practices
# Verifies Ubuntu 20.04.5 LTS + Python 3.8.10 + CUDA 11.4 environment

set -e

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[PASS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[FAIL]${NC} $1"
}

# Check result counters
TOTAL_CHECKS=0
PASSED_CHECKS=0
FAILED_CHECKS=0

run_check() {
    local check_name="$1"
    local check_function="$2"

    TOTAL_CHECKS=$((TOTAL_CHECKS + 1))
    log_info "Checking: $check_name"

    if $check_function; then
        log_success "$check_name"
        PASSED_CHECKS=$((PASSED_CHECKS + 1))
        return 0
    else
        log_error "$check_name"
        FAILED_CHECKS=$((FAILED_CHECKS + 1))
        return 1
    fi
}

# Basic system information check
check_system_info() {
    echo "=== Basic System Information ==="
    uname -a
    if [ -f /etc/nv_tegra_release ]; then
        echo "Tegra version:"
        cat /etc/nv_tegra_release
    fi
    echo
    return 0
}

# Ubuntu version check
check_ubuntu_version() {
    local expected_version="20.04"
    local actual_version=$(lsb_release -rs 2>/dev/null || echo "unknown")

    if [[ "$actual_version" == "$expected_version"* ]]; then
        echo "Ubuntu version: $actual_version (OK)"
        return 0
    else
        echo "Ubuntu version mismatch: expected=$expected_version, actual=$actual_version"
        return 1
    fi
}

# Python version check
check_python_version() {
    local expected_version="3.8.10"
    local actual_version=$(python3 --version 2>/dev/null | cut -d' ' -f2 || echo "unknown")

    if [[ "$actual_version" == "$expected_version"* ]]; then
        echo "Python version: $actual_version (OK)"
        return 0
    else
        echo "Python version mismatch: expected=$expected_version, actual=$actual_version"
        return 1
    fi
}

# CUDA toolkit check
check_cuda_toolkit() {
    local expected_version="11.4"

    if command -v nvcc >/dev/null 2>&1; then
        local actual_version=$(nvcc --version | grep "release" | sed -n 's/.*release \([0-9]\+\.[0-9]\+\).*/\1/p')
        if [[ "$actual_version" == "$expected_version"* ]]; then
            echo "CUDA version: $actual_version (OK)"
            return 0
        else
            echo "CUDA version mismatch: expected=$expected_version, actual=$actual_version"
            return 1
        fi
    else
        echo "nvcc command not found"
        return 1
    fi
}

# GPU check
check_gpu_info() {
    if command -v nvidia-smi >/dev/null 2>&1; then
        nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader,nounits
        return 0
    else
        # nvidia-smi may not be available on Jetson, try alternative methods
        if [ -f /proc/driver/nvidia/version ]; then
            echo "NVIDIA driver info:"
            cat /proc/driver/nvidia/version
            return 0
        else
            echo "Unable to retrieve GPU info (this is normal on Jetson)"
            return 0
        fi
    fi
}

# CUDA sample run test
check_cuda_samples() {
    local samples_dir="/usr/local/cuda/samples/1_Utilities/deviceQuery"

    if [ -d "$samples_dir" ]; then
        log_info "Compiling and running CUDA deviceQuery sample..."
        cd "$samples_dir"

        if [ ! -f "./deviceQuery" ]; then
            if sudo make >/dev/null 2>&1; then
                echo "CUDA sample compilation successful"
            else
                echo "CUDA sample compilation failed"
                return 1
            fi
        fi

        if ./deviceQuery | grep -q "Result = PASS"; then
            echo "CUDA device query successful"
            return 0
        else
            echo "CUDA device query failed"
            return 1
        fi
    else
        echo "CUDA sample directory not found: $samples_dir"
        return 1
    fi
}

# Memory check
check_memory() {
    local total_mem=$(free -h | awk '/^Mem:/{print $2}')
    local available_mem=$(free -h | awk '/^Mem:/{print $7}')
    local swap_mem=$(free -h | awk '/^Swap:/{print $2}')

    echo "Total memory: $total_mem"
    echo "Available memory: $available_mem"
    echo "Swap space: $swap_mem"

    # Check if there is enough memory (at least 8GB)
    local total_mem_gb=$(free -g | awk '/^Mem:/{print $2}')
    if [ "$total_mem_gb" -ge 8 ]; then
        echo "Memory capacity is sufficient"
        return 0
    else
        echo "Memory may be insufficient, at least 8GB recommended"
        return 1
    fi
}

# ROS2 environment check
check_ros2_environment() {
    if [ -f "/opt/ros/foxy/setup.bash" ]; then
        echo "ROS2 Foxy installation check passed"

        # Temporarily source ROS2 environment
        source /opt/ros/foxy/setup.bash >/dev/null 2>&1

        if command -v ros2 >/dev/null 2>&1; then
            echo "ROS2 command is available"
            return 0
        else
            echo "ROS2 command is not available"
            return 1
        fi
    else
        echo "ROS2 Foxy is not installed"
        return 1
    fi
}

# Network configuration check
check_network_config() {
    local eth_interface=$(ip route | grep default | awk '{print $5}' | head -1)
    local ip_address=$(ip addr show "$eth_interface" 2>/dev/null | grep 'inet ' | awk '{print $2}' | cut -d'/' -f1)

    if [ -n "$ip_address" ]; then
        echo "Network interface: $eth_interface"
        echo "IP address: $ip_address"

        # Check if on the expected subnet
        if [[ "$ip_address" =~ ^192\.168\. ]]; then
            echo "Network configuration is correct"
            return 0
        else
            echo "Network configuration may need adjustment"
            return 1
        fi
    else
        echo "Unable to retrieve network information"
        return 1
    fi
}

# Key directory check
check_project_structure() {
    local project_dirs=("src/claudia" "config" "scripts" "cyclonedds_ws")

    for dir in "${project_dirs[@]}"; do
        if [ -d "$dir" ]; then
            echo "Directory exists: $dir"
        else
            echo "Directory missing: $dir"
            return 1
        fi
    done

    return 0
}

# Main execution function
main() {
    echo "Claudia Robot System Environment Verification Script"
    echo "==============================================="
    echo "Verification target: Ubuntu 20.04.5 LTS + Python 3.8.10 + CUDA 11.4"
    echo

    # Execute all checks
    run_check "Basic System Information" check_system_info
    run_check "Ubuntu Version Check" check_ubuntu_version
    run_check "Python Version Check" check_python_version
    run_check "CUDA Toolkit Check" check_cuda_toolkit
    run_check "GPU Information Check" check_gpu_info
    run_check "Memory Configuration Check" check_memory
    run_check "ROS2 Environment Check" check_ros2_environment
    run_check "Network Configuration Check" check_network_config
    run_check "Project Structure Check" check_project_structure

    # Optional: CUDA sample test (takes longer)
    if [ "${1:-}" != "--quick" ]; then
        run_check "CUDA Sample Test" check_cuda_samples
    fi

    echo
    echo "==============================================="
    echo "Verification Results Summary:"
    echo "Total checks: $TOTAL_CHECKS"
    echo "Passed: $PASSED_CHECKS"
    echo "Failed: $FAILED_CHECKS"

    if [ "$FAILED_CHECKS" -eq 0 ]; then
        log_success "All environment checks passed! System is ready"
        return 0
    else
        log_error "Found $FAILED_CHECKS environment issues, please check the output above"
        return 1
    fi
}

# Script help
if [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  --quick    Quick check (skip CUDA sample test)"
    echo "  --help     Show help information"
    exit 0
fi

# Execute main function
main "$@"
