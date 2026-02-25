#!/bin/bash
# scripts/validation/camera/run_front_camera_validation.sh
# Generated: 2024-12-26 16:30:00
# Purpose: Unitree Go2 front camera validation quick start script

set -e

# Script information
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VALIDATION_DIR="${SCRIPT_DIR}/front_camera_validation"
TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')

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

# Display help information
show_help() {
    cat << EOF
Unitree Go2 Front Camera Validation Quick Start Script

Usage:
    $0 [options]

Options:
    -h, --help          Show this help information
    -q, --quick         Quick validation mode (basic tests only)
    -f, --full          Full validation mode (includes stress test)
    -c, --config FILE   Specify configuration file
    -o, --output DIR    Specify output directory
    -v, --verbose       Verbose output mode
    --dry-run          Dry-run mode (does not execute actual tests)

Examples:
    $0                  # Default full validation
    $0 -q               # Quick validation
    $0 -c my_config.json -o ./my_results -v  # Custom config and output

EOF
}

# Check environment
check_environment() {
    log_info "Checking runtime environment..."

    # Check Python
    if ! command -v python3 &> /dev/null; then
        log_error "Python3 not found, please install Python 3.7+"
        exit 1
    fi

    local python_version=$(python3 --version | cut -d' ' -f2)
    log_info "Python version: $python_version"

    # Check required Python packages
    local required_packages=("cv2" "numpy" "skimage")
    for package in "${required_packages[@]}"; do
        if ! python3 -c "import $package" &> /dev/null; then
            log_warning "Python package $package not found, may affect functionality"
        fi
    done

    # Check camera devices
    if ls /dev/video* &> /dev/null; then
        log_info "Camera devices found: $(ls /dev/video*)"
    else
        log_warning "No camera devices found, validation may fail"
    fi

    # Check permissions
    if groups | grep -q video; then
        log_info "User is already in the video group"
    else
        log_warning "User is not in the video group, camera permissions may be required"
    fi

    log_success "Environment check complete"
}

# Verify scripts exist
check_scripts() {
    log_info "Checking validation scripts..."

    if [ ! -d "$VALIDATION_DIR" ]; then
        log_error "Validation directory does not exist: $VALIDATION_DIR"
        exit 1
    fi

    local required_files=(
        "main_validation_script.py"
        "camera_config.py"
        "performance_tester.py"
        "image_quality_analyzer.py"
        "validation_config.json"
    )

    for file in "${required_files[@]}"; do
        if [ ! -f "$VALIDATION_DIR/$file" ]; then
            log_error "Missing required file: $file"
            exit 1
        fi
    done

    log_success "Validation script check complete"
}

# Create output directory
setup_output_dir() {
    local output_dir="$1"
    if [ -z "$output_dir" ]; then
        output_dir="logs/camera_validation"
    fi

    mkdir -p "$output_dir"
    log_info "Output directory: $output_dir"
}

# Run validation
run_validation() {
    local mode="$1"
    local config_file="$2"
    local output_dir="$3"
    local verbose="$4"
    local dry_run="$5"

    log_info "Starting front camera validation..."
    log_info "Mode: $mode"
    log_info "Time: $TIMESTAMP"

    # Build command
    local cmd="python3 $VALIDATION_DIR/main_validation_script.py"

    if [ -n "$config_file" ]; then
        cmd="$cmd --config $config_file"
    fi

    if [ -n "$output_dir" ]; then
        cmd="$cmd --output $output_dir"
    fi

    if [ "$verbose" = "true" ]; then
        cmd="$cmd --verbose"
    fi

    log_info "Executing command: $cmd"

    if [ "$dry_run" = "true" ]; then
        log_warning "Dry-run mode - not executing actual tests"
        return 0
    fi

    # Switch to validation directory
    cd "$VALIDATION_DIR"

    # Execute validation
    if eval "$cmd"; then
        log_success "Validation complete!"
        return 0
    else
        log_error "Validation failed!"
        return 1
    fi
}

# Main function
main() {
    # Default parameters
    local mode="full"
    local config_file=""
    local output_dir=""
    local verbose="false"
    local dry_run="false"

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_help
                exit 0
                ;;
            -q|--quick)
                mode="quick"
                shift
                ;;
            -f|--full)
                mode="full"
                shift
                ;;
            -c|--config)
                config_file="$2"
                shift 2
                ;;
            -o|--output)
                output_dir="$2"
                shift 2
                ;;
            -v|--verbose)
                verbose="true"
                shift
                ;;
            --dry-run)
                dry_run="true"
                shift
                ;;
            *)
                log_error "Unknown argument: $1"
                show_help
                exit 1
                ;;
        esac
    done

    # Display welcome message
    echo -e "${BLUE}================================${NC}"
    echo -e "${BLUE}Unitree Go2 Front Camera Validation System${NC}"
    echo -e "${BLUE}================================${NC}"
    echo ""

    # Execute pre-checks
    check_environment
    check_scripts
    setup_output_dir "$output_dir"

    echo ""
    log_info "Preparing to start validation..."

    # Run validation
    if run_validation "$mode" "$config_file" "$output_dir" "$verbose" "$dry_run"; then
        echo ""
        log_success "Front camera validation completed successfully!"
        echo -e "${GREEN}View results: ${output_dir:-logs/camera_validation}${NC}"
        exit 0
    else
        echo ""
        log_error "Front camera validation failed!"
        log_info "Please check the log files for detailed information"
        exit 1
    fi
}

# Execute main function
main "$@"
