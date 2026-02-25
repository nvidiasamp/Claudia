#!/bin/bash

#1.3
# CycloneDDS Workspace Setup Script - Claudia Robot System
# Generated: 2025-06-26
# Purpose: Task 1.3 - Set up cyclonedds_ws workspace and compile Unitree ROS2 packages
# Platform: aarch64 Ubuntu 20.04.5 LTS

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

# Error handling
cleanup_on_failure() {
    local operation_name="$1"
    local failure_reason="$2"

    log_error "$operation_name failed: $failure_reason"

    # Record failure information
    FAILURE_LOG="logs/failures/$(date '+%Y%m%d_%H%M%S')_${operation_name}_failure.log"
    mkdir -p "$(dirname "$FAILURE_LOG")"

    cat > "$FAILURE_LOG" << EOF
Operation: $operation_name
Time: $(date '+%Y-%m-%d %H:%M:%S %Z')
Reason: $failure_reason
Platform: $(uname -a)
Working Directory: $(pwd)
Git Status: $(git status --porcelain 2>/dev/null || echo "Not a git repository")
Disk Space: $(df -h .)
Memory: $(free -h)
EOF

    echo "Failure information recorded: $FAILURE_LOG"
}

# Set error handling
trap 'cleanup_on_failure "cyclonedds_workspace_setup" "Command execution failed"' ERR

# Start setup
echo "Starting CycloneDDS workspace setup - $(date '+%Y-%m-%d %H:%M:%S')"
echo "Project root directory: $PROJECT_ROOT"
echo "Workspace directory: $CYCLONEDDS_WS"

# 1. Check prerequisites
log_info "Checking prerequisites..."

# Check ROS2 Foxy
if ! command -v ros2 > /dev/null 2>&1; then
    log_error "ROS2 Foxy not installed or not in PATH"
    exit 1
fi

# Check colcon
if ! command -v colcon > /dev/null 2>&1; then
    log_error "colcon build tool not installed"
    log_info "Please run: sudo apt install python3-colcon-common-extensions"
    exit 1
fi

# Check network connection
if ! ping -c 1 github.com > /dev/null 2>&1; then
    log_warning "Network connectivity check failed, may affect repository cloning"
fi

log_success "Prerequisites check complete"

# 2. Create workspace directory structure
log_info "Creating workspace directory structure..."

if [ -d "$CYCLONEDDS_WS" ]; then
    log_warning "Workspace directory already exists: $CYCLONEDDS_WS"
    read -p "Recreate workspace? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        log_info "Backing up existing workspace..."
        BACKUP_DIR="${CYCLONEDDS_WS}.backup_$(date '+%Y%m%d_%H%M%S')"
        mv "$CYCLONEDDS_WS" "$BACKUP_DIR"
        log_info "Backup complete: $BACKUP_DIR"
    else
        log_info "Using existing workspace directory"
    fi
fi

# Create workspace structure
mkdir -p "$CYCLONEDDS_WS/src"
cd "$CYCLONEDDS_WS"

log_success "Workspace directory structure created"

# 3. Clone Unitree repositories
log_info "Cloning Unitree repositories..."

cd src

# Clone unitree_ros2
if [ ! -d "unitree_ros2" ]; then
    log_info "Cloning unitree_ros2 repository..."
    git clone https://github.com/unitreerobotics/unitree_ros2.git
    log_success "unitree_ros2 cloned"
else
    log_info "unitree_ros2 already exists, skipping clone"
fi

# Clone unitree_sdk2
if [ ! -d "unitree_sdk2" ]; then
    log_info "Cloning unitree_sdk2 repository..."
    git clone https://github.com/unitreerobotics/unitree_sdk2.git
    log_success "unitree_sdk2 cloned"
else
    log_info "unitree_sdk2 already exists, skipping clone"
fi

# Verify clone results
log_info "Verifying cloned repositories..."
for repo in unitree_ros2 unitree_sdk2; do
    if [ -d "$repo" ] && [ "$(ls -A $repo)" ]; then
        log_success "$repo clone verification successful"
    else
        log_error "$repo clone failed or directory is empty"
        exit 1
    fi
done

cd ..

# 4. Set up ROS2 environment
log_info "Setting up ROS2 environment..."

# Ensure correct RMW implementation name is used
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

# Source ROS2 environment
source /opt/ros/foxy/setup.bash

log_success "ROS2 environment setup complete (RMW: $RMW_IMPLEMENTATION)"

# 5. Build workspace
log_info "Starting workspace build..."

BUILD_START_TIME=$(date '+%s')

# Build all packages
if colcon build --symlink-install --event-handlers console_direct+; then
    BUILD_END_TIME=$(date '+%s')
    BUILD_DURATION=$((BUILD_END_TIME - BUILD_START_TIME))
    log_success "Workspace build complete (elapsed: ${BUILD_DURATION} seconds)"
else
    log_error "Workspace build failed"
    exit 1
fi

# 6. Verify build results
log_info "Verifying build results..."

# Check install directory
if [ -d "install" ] && [ "$(ls -A install)" ]; then
    log_success "install directory created successfully"
else
    log_error "install directory not created or is empty"
    exit 1
fi

# Source workspace environment
source install/setup.bash

# Check ROS2 packages
log_info "Checking installed ROS2 packages..."
UNITREE_PACKAGES=$(ros2 pkg list | grep unitree | wc -l)
if [ "$UNITREE_PACKAGES" -gt 0 ]; then
    log_success "Found $UNITREE_PACKAGES Unitree ROS2 packages"
    ros2 pkg list | grep unitree
else
    log_warning "No Unitree ROS2 packages found"
fi

# 7. Test Python imports
log_info "Testing Python package imports..."

python3 -c "
try:
    import unitree_go.msg
    import unitree_api.msg
    print('Python package import successful')
except ImportError as e:
    print(f'Python package import failed: {e}')
    exit(1)
"

# 8. Create environment setup script
log_info "Creating environment setup script..."

ENV_SCRIPT="$CYCLONEDDS_WS/setup_env.sh"
cat > "$ENV_SCRIPT" << 'EOF'
#!/bin/bash
# CycloneDDS Workspace Environment Setup
# Note: Using correct RMW implementation name

# ROS2 base environment
source /opt/ros/foxy/setup.bash

# Workspace environment
source "$(dirname "${BASH_SOURCE[0]}")/install/setup.bash"

# DDS configuration - using correct rmw_cyclonedds_cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

# CycloneDDS network configuration
export CYCLONEDDS_URI='<CycloneDX><Domain><General><Interfaces>
 <NetworkInterface name="eth0" priority="default" multicast="default" />
 </Interfaces></General></Domain></CycloneDX>'

echo "CycloneDDS workspace environment loaded"
echo "   RMW Implementation: $RMW_IMPLEMENTATION"
echo "   ROS Domain ID: $ROS_DOMAIN_ID"
EOF

chmod +x "$ENV_SCRIPT"
log_success "Environment setup script created: $ENV_SCRIPT"

# 9. Generate status report
log_info "Generating workspace status report..."

REPORT_FILE="$CYCLONEDDS_WS/workspace_status.txt"
cat > "$REPORT_FILE" << EOF
CycloneDDS Workspace Status Report
Generated: $(date '+%Y-%m-%d %H:%M:%S %Z')
====================================

Workspace path: $CYCLONEDDS_WS
Build time: ${BUILD_DURATION} seconds
RMW implementation: rmw_cyclonedds_cpp (corrected)

Directory structure:
$(tree -L 2 2>/dev/null || find . -maxdepth 2 -type d | sort)

ROS2 package list:
$(ros2 pkg list | grep unitree)

Python package verification:
$(python3 -c "import unitree_go.msg, unitree_api.msg; print('Python import successful')" 2>/dev/null || echo "Python import failed")

Disk usage:
$(du -sh .)

Environment script: $ENV_SCRIPT
Report file: $REPORT_FILE
EOF

log_success "Status report generated: $REPORT_FILE"

# 10. Clean temporary files
log_info "Cleaning temporary files..."
find /tmp -name "claudia*" -mtime +1 -type f -delete 2>/dev/null || true
find . -name "*.tmp" -delete 2>/dev/null || true

# Final success message
echo ""
echo "=========================="
echo "CycloneDDS Workspace Setup Complete!"
echo "=========================="
echo ""
echo "Instructions:"
echo "   1. Load environment: source $CYCLONEDDS_WS/setup_env.sh"
echo "   2. View status: cat $CYCLONEDDS_WS/workspace_status.txt"
echo "   3. Rebuild: cd $CYCLONEDDS_WS && colcon build"
echo ""
echo "Task 1.3 complete - $(date '+%Y-%m-%d %H:%M:%S')"

# Return to project root directory
cd "$PROJECT_ROOT"
