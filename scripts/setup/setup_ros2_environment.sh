#!/bin/bash

#1.4
# ROS2 Environment Setup Script - Claudia Robot System
# Generated: 2025-06-26
# Purpose: Set up and test ROS2 integration with the Claudia project

set -e

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Project paths
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CYCLONEDDS_WS="$PROJECT_ROOT/cyclonedds_ws"

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Set up CUDA environment variables
setup_cuda_environment() {
    log_info "Setting up CUDA environment variables..."

    export PATH=/usr/local/cuda/bin:$PATH
    export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
    export CUDA_HOME=/usr/local/cuda

    if command -v nvcc >/dev/null 2>&1; then
        log_success "CUDA environment setup complete: $(nvcc --version | grep release | sed 's/.*release //' | sed 's/,.*//')"
    else
        log_error "CUDA environment setup failed"
        return 1
    fi
}

# Set up ROS2 environment
setup_ros2_environment() {
    log_info "Setting up ROS2 environment..."

    # Source ROS2 Foxy
    if [ -f "/opt/ros/foxy/setup.bash" ]; then
        source /opt/ros/foxy/setup.bash
        log_success "ROS2 Foxy environment loaded"
    else
        log_error "ROS2 Foxy setup file not found"
        return 1
    fi

    # Set ROS2 environment variables
    export ROS_VERSION=2
    export ROS_DISTRO=foxy
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DOMAIN_ID=0

    # Source workspace
    if [ -f "$CYCLONEDDS_WS/install/setup.bash" ]; then
        source "$CYCLONEDDS_WS/install/setup.bash"
        log_success "cyclonedds_ws workspace loaded"
    else
        log_warning "Workspace setup file not found, attempting to build..."
        if build_workspace; then
            source "$CYCLONEDDS_WS/install/setup.bash"
            log_success "Workspace built and loaded successfully"
        else
            log_error "Workspace build failed"
            return 1
        fi
    fi

    # Set Python path
    PYTHON_PATH="$CYCLONEDDS_WS/install/lib/python3.8/site-packages"
    if [ -d "$PYTHON_PATH" ]; then
        export PYTHONPATH="$PYTHONPATH:$PYTHON_PATH"
        log_success "Python path set"
    fi
}

# Build workspace
build_workspace() {
    log_info "Building cyclonedds_ws workspace..."

    cd "$CYCLONEDDS_WS"

    # Ensure source directory exists
    if [ ! -d "src" ]; then
        log_error "Workspace src directory does not exist"
        return 1
    fi

    # Clean and build
    if colcon build --symlink-install --event-handlers console_direct+; then
        log_success "Workspace build successful"
        return 0
    else
        log_error "Workspace build failed"
        return 1
    fi
}

# Verify ROS2 installation
verify_ros2_installation() {
    log_info "Verifying ROS2 installation..."

    # Check ROS2 command
    if ! command -v ros2 >/dev/null 2>&1; then
        log_error "ros2 command is not available"
        return 1
    fi

    # Check ROS2 packages
    local package_count=$(ros2 pkg list | wc -l)
    log_success "ROS2 package check passed ($package_count packages)"

    # Check Unitree packages
    if ros2 pkg list | grep -q "unitree"; then
        log_success "Unitree ROS2 packages are installed"
        ros2 pkg list | grep unitree | while read -r pkg; do
            echo "  - $pkg"
        done
    else
        log_warning "Unitree ROS2 packages not found"
    fi
}

# Test Python integration
test_python_integration() {
    log_info "Testing Python integration..."

    cd "$PROJECT_ROOT"

    # Test basic ROS2 package imports
    python3 -c "
import sys
try:
    import rclpy
    import std_msgs.msg
    import geometry_msgs.msg
    print('Basic ROS2 Python package import successful')
except ImportError as e:
    print(f'ROS2 Python package import failed: {e}')
    sys.exit(1)

# Test Unitree package imports
try:
    import unitree_go.msg
    import unitree_api.msg
    print('Unitree Python package import successful')
except ImportError as e:
    print(f'Unitree Python package import failed: {e}')

# Test Claudia ROS2 manager
try:
    sys.path.insert(0, 'src')
    from claudia.common.ros2_manager import ROS2Manager
    manager = ROS2Manager()
    print('Claudia ROS2 manager import successful')
    print(f'  Project root: {manager.project_root}')
    print(f'  Workspace: {manager.cyclonedds_ws}')
except ImportError as e:
    print(f'Claudia ROS2 manager import failed: {e}')
    sys.exit(1)
except Exception as e:
    print(f'ROS2 manager initialization failed: {e}')
    sys.exit(1)
"

    if [ $? -eq 0 ]; then
        log_success "Python integration test passed"
    else
        log_error "Python integration test failed"
        return 1
    fi
}

# Test ROS2 communication
test_ros2_communication() {
    log_info "Testing ROS2 communication..."

    # Check available topics
    local topics=$(ros2 topic list 2>/dev/null | wc -l)
    if [ "$topics" -gt 0 ]; then
        log_success "ROS2 topic discovery successful ($topics topics)"

        # Display key topics
        echo "Key topics:"
        ros2 topic list | grep -E "(sportmode|state|cmd)" | while read -r topic; do
            echo "  - $topic"
        done
    else
        log_warning "No ROS2 topics discovered (robot may not be connected)"
    fi

    # Check available services
    local services=$(ros2 service list 2>/dev/null | wc -l)
    if [ "$services" -gt 0 ]; then
        log_success "ROS2 service discovery successful ($services services)"
    else
        log_warning "No ROS2 services discovered"
    fi
}

# Create environment variable file
create_env_file() {
    log_info "Creating environment variable file..."

    local env_file="$PROJECT_ROOT/.env.ros2"

    cat > "$env_file" << EOF
# ROS2 Environment Variables - Claudia Robot System

# CUDA environment
export PATH=/usr/local/cuda/bin:\$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:\$LD_LIBRARY_PATH
export CUDA_HOME=/usr/local/cuda

# ROS2 environment
export ROS_VERSION=2
export ROS_DISTRO=foxy
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

# Workspace
source /opt/ros/foxy/setup.bash
source $CYCLONEDDS_WS/install/setup.bash

# Python path
export PYTHONPATH=$CYCLONEDDS_WS/install/lib/python3.8/site-packages:\$PYTHONPATH
EOF

    chmod +x "$env_file"
    log_success "Environment variable file created: $env_file"

    echo "Usage: source $env_file"
}

# Main function
main() {
    echo "Claudia Robot ROS2 Environment Setup Script"
    echo "=============================================="
    echo "Project root: $PROJECT_ROOT"
    echo "Workspace: $CYCLONEDDS_WS"
    echo

    # Set up environment
    setup_cuda_environment || exit 1
    setup_ros2_environment || exit 1

    # Verify installation
    verify_ros2_installation || exit 1

    # Test integration
    test_python_integration || exit 1
    test_ros2_communication

    # Create environment file
    create_env_file

    echo
    echo "=============================================="
    log_success "ROS2 environment setup complete!"
    echo
    echo "Next steps:"
    echo "1. source $PROJECT_ROOT/.env.ros2"
    echo "2. Test the connection to the Unitree Go2 robot"
    echo "3. Run Claudia AI components"

    return 0
}

# Script help
if [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  --help     Show help information"
    echo "  --build    Force rebuild workspace"
    exit 0
fi

# Force build option
if [ "${1:-}" = "--build" ]; then
    log_info "Force rebuilding workspace..."
    cd "$CYCLONEDDS_WS"
    rm -rf build/ install/ log/
    build_workspace || exit 1
fi

# Execute main function
main "$@"
