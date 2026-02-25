#!/bin/bash
# scripts/validation/camera/install_camera_validation_deps.sh
# Generated: 2025-06-27 14:10:00
# Purpose: Install dependencies for the front camera validation system

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
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running as root
check_root() {
    if [ "$EUID" -eq 0 ]; then
        log_warning "It is recommended to run this script as a regular user"
        log_warning "System packages will be installed using sudo"
    fi
}

# Update system packages
update_system_packages() {
    log_info "Updating system package list..."

    if command -v apt-get &> /dev/null; then
        sudo apt-get update
        log_success "APT package list update complete"
    elif command -v yum &> /dev/null; then
        sudo yum update
        log_success "YUM package list update complete"
    else
        log_warning "Unrecognized package manager, skipping system update"
    fi
}

# Install system dependencies
install_system_deps() {
    log_info "Installing system dependency packages..."

    if command -v apt-get &> /dev/null; then
        # Ubuntu/Debian systems
        local packages=(
            "python3"
            "python3-pip"
            "python3-dev"
            "build-essential"
            "cmake"
            "pkg-config"
            "libjpeg-dev"
            "libtiff5-dev"
            "libpng-dev"
            "libavcodec-dev"
            "libavformat-dev"
            "libswscale-dev"
            "libv4l-dev"
            "libxvidcore-dev"
            "libx264-dev"
            "libgtk-3-dev"
            "libatlas-base-dev"
            "gfortran"
        )

        log_info "Installing APT packages: ${packages[*]}"
        sudo apt-get install -y "${packages[@]}"

    elif command -v yum &> /dev/null; then
        # CentOS/RHEL systems
        local packages=(
            "python3"
            "python3-pip"
            "python3-devel"
            "gcc"
            "gcc-c++"
            "cmake"
            "pkgconfig"
            "libjpeg-turbo-devel"
            "libpng-devel"
            "libtiff-devel"
            "opencv-devel"
        )

        log_info "Installing YUM packages: ${packages[*]}"
        sudo yum install -y "${packages[@]}"

    else
        log_error "Unrecognized Linux distribution, please install dependencies manually"
        return 1
    fi

    log_success "System dependency installation complete"
}

# Install Python packages
install_python_deps() {
    log_info "Installing Python dependency packages..."

    # Upgrade pip
    python3 -m pip install --upgrade pip

    # Core dependency packages
    local python_packages=(
        "opencv-python>=4.0.0"
        "numpy>=1.18.0"
        "scikit-image>=0.17.0"
        "matplotlib>=3.0.0"
        "Pillow>=7.0.0"
    )

    log_info "Installing Python packages: ${python_packages[*]}"

    # Install in user mode to avoid permission issues
    python3 -m pip install --user "${python_packages[@]}"

    log_success "Python dependency installation complete"
}

# Verify installation
verify_installation() {
    log_info "Verifying installation results..."

    # Check Python packages
    local packages_to_verify=("cv2" "numpy" "skimage" "matplotlib" "PIL")

    for package in "${packages_to_verify[@]}"; do
        if python3 -c "import $package" &> /dev/null; then
            log_success "$package installed successfully"
        else
            log_error "$package installation failed"
        fi
    done

    # Check camera devices
    if ls /dev/video* &> /dev/null; then
        log_success "Camera devices found: $(ls /dev/video* | tr '\n' ' ')"
    else
        log_warning "No camera devices found"
    fi

    # Check user permissions
    if groups | grep -q video; then
        log_success "User is in the video group"
    else
        log_warning "User is not in the video group"
        log_info "Run the following command to add video group permissions:"
        log_info "sudo usermod -a -G video $USER"
        log_info "Then log out and log back in"
    fi

    log_success "Installation verification complete"
}

# Post-installation configuration
post_install_config() {
    log_info "Performing post-installation configuration..."

    # Check if user needs to be added to video group
    if ! groups | grep -q video; then
        log_info "Adding user to video group..."
        sudo usermod -a -G video "$USER"
        log_warning "You need to log out and log back in for group permissions to take effect"
    fi

    # Create necessary directories
    mkdir -p logs/camera_validation
    mkdir -p tmp/downloads

    log_success "Post-installation configuration complete"
}

# Display installation completion information
show_completion_info() {
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Front camera validation system dependency installation complete!${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""

    log_info "Next steps:"
    echo "1. If video group permissions were added, please log out and log back in"
    echo "2. Run the validation script:"
    echo "   ./scripts/validation/camera/run_front_camera_validation.sh --dry-run"
    echo "3. Perform actual validation:"
    echo "   ./scripts/validation/camera/run_front_camera_validation.sh"
    echo ""

    log_info "If you encounter issues, please refer to:"
    echo "- scripts/validation/camera/front_camera_validation/README_front_camera_validation.md"
    echo "- Log file: front_camera_validation.log"
}

# Main function
main() {
    echo -e "${BLUE}================================${NC}"
    echo -e "${BLUE}Front Camera Validation System Dependency Installation${NC}"
    echo -e "${BLUE}================================${NC}"
    echo ""

    check_root

    log_info "Starting dependency installation..."

    # Check network connectivity
    if ! ping -c 1 google.com &> /dev/null && ! ping -c 1 baidu.com &> /dev/null; then
        log_error "Network connection failed, unable to download dependency packages"
        exit 1
    fi

    # Execute installation steps
    update_system_packages
    install_system_deps
    install_python_deps
    verify_installation
    post_install_config
    show_completion_info

    log_success "Dependency installation script execution complete!"
}

# Script entry point
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
