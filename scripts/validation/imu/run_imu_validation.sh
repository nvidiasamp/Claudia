#!/bin/bash
# Generated: 2025-01-27 11:55:00
# Purpose: Execute IMU validation system with various options
# Platform: Jetson Xavier NX Ubuntu 18.04

set -e

# Script directory and path setup
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VALIDATION_DIR="$SCRIPT_DIR/imu_validation"
MAIN_SCRIPT="$VALIDATION_DIR/main_validation_script.py"

echo "Unitree Go2 IMU Validation System"
echo "=========================================="

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

# Show help information
show_help() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  -h, --help              Show this help message"
    echo "  -c, --config <file>     Use custom configuration file"
    echo "  -t, --test              Run quick test mode (5 seconds)"
    echo "  -v, --verbose           Verbose output mode"
    echo "  -r, --report-only       Generate report only, no new data collection"
    echo "  --install-deps          Install required dependencies"
    echo "  --check-env             Check environment and dependencies"
    echo "  --interactive           Interactive mode to select validation items"
    echo ""
    echo "Examples:"
    echo "  $0                              # Run full validation"
    echo "  $0 --test                       # Quick test"
    echo "  $0 -c custom_config.json        # Use custom configuration"
    echo "  $0 --interactive                # Interactive validation item selection"
    echo "  $0 --check-env                  # Check environment status"
    echo ""
}

# Check environment and dependencies
check_environment() {
    echo "Checking runtime environment..."
    echo "Current time: $(date '+%Y-%m-%d %H:%M:%S %Z')"
    echo "Working directory: $(pwd)"
    echo "Disk usage: $(df . | tail -1 | awk '{print $5}')"
    echo "Memory usage: $(free | grep Mem | awk '{printf "%.0f%%", $3/$2 * 100.0}')"

    # Check Python
    echo -n "Checking Python3... "
    if command -v python3 &> /dev/null; then
        echo "$(python3 --version)"
    else
        echo "Python3 not found"
        exit 1
    fi

    # Check validation script
    echo -n "Checking validation script... "
    if [ -f "$MAIN_SCRIPT" ]; then
        echo "Main validation script found"
    else
        echo "Main validation script not found: $MAIN_SCRIPT"
        exit 1
    fi

    # Check Python dependencies
    echo "Checking Python dependencies:"
    local deps_ok=true
    local required_modules=("numpy" "matplotlib" "scipy" "yaml" "jsonschema")

    for module in "${required_modules[@]}"; do
        echo -n "  $module... "
        if python3 -c "import $module" 2>/dev/null; then
            echo "OK"
        else
            echo "MISSING"
            deps_ok=false
        fi
    done

    # Check Unitree SDK
    echo -n "  unitree_sdk2py... "
    if python3 -c "import unitree_sdk2py" 2>/dev/null; then
        echo "OK"
    else
        echo "MISSING (manual installation required)"
        echo "    Please run: $0 --install-deps"
        deps_ok=false
    fi

    if [ "$deps_ok" = false ]; then
        echo ""
        echo "Some dependencies are missing, recommend running: $0 --install-deps"
        if [ "$1" = "--strict" ]; then
            exit 1
        fi
    else
        echo "All dependency checks passed"
    fi

    echo ""
}

# Interactive mode
interactive_mode() {
    echo "Interactive Validation Mode"
    echo "Please select the validation item to execute:"
    echo ""
    echo "1) Full validation process (includes all tests)"
    echo "2) Static stability test"
    echo "3) Dynamic response test"
    echo "4) Calibration accuracy analysis"
    echo "5) Real-time data visualization"
    echo "6) Quick test mode (5 seconds)"
    echo "7) Environment check"
    echo "8) Install dependencies"
    echo ""

    read -p "Please enter option (1-8): " choice

    case $choice in
        1)
            echo "Starting full validation process..."
            run_validation
            ;;
        2)
            echo "Starting static stability test..."
            python3 "$MAIN_SCRIPT" --mode static
            ;;
        3)
            echo "Starting dynamic response test..."
            python3 "$MAIN_SCRIPT" --mode dynamic
            ;;
        4)
            echo "Starting calibration accuracy analysis..."
            python3 "$MAIN_SCRIPT" --mode calibration
            ;;
        5)
            echo "Starting real-time data visualization..."
            python3 "$MAIN_SCRIPT" --mode visualize
            ;;
        6)
            echo "Starting quick test mode..."
            python3 "$MAIN_SCRIPT" --test-duration 5
            ;;
        7)
            check_environment --strict
            ;;
        8)
            "$SCRIPT_DIR/install_imu_validation_deps.sh"
            ;;
        *)
            echo "Invalid option: $choice"
            exit 1
            ;;
    esac
}

# Core function to run validation
run_validation() {
    local config_file=""
    local test_mode=false
    local verbose=false
    local report_only=false
    local extra_args=()

    # Process arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -c|--config)
                config_file="$2"
                shift 2
                ;;
            -t|--test)
                test_mode=true
                shift
                ;;
            -v|--verbose)
                verbose=true
                shift
                ;;
            -r|--report-only)
                report_only=true
                shift
                ;;
            *)
                extra_args+=("$1")
                shift
                ;;
        esac
    done

    # Build command
    local cmd="python3 \"$MAIN_SCRIPT\""

    if [ -n "$config_file" ]; then
        if [ -f "$config_file" ]; then
            cmd="$cmd --config \"$config_file\""
        else
            echo "Configuration file does not exist: $config_file"
            exit 1
        fi
    fi

    if [ "$test_mode" = true ]; then
        cmd="$cmd --test-duration 5"
    fi

    if [ "$verbose" = true ]; then
        cmd="$cmd --verbose"
    fi

    if [ "$report_only" = true ]; then
        cmd="$cmd --report-only"
    fi

    # Add extra arguments
    for arg in "${extra_args[@]}"; do
        cmd="$cmd \"$arg\""
    done

    echo "Starting IMU validation..."
    echo "Command: $cmd"
    echo ""

    # Switch to validation directory and execute
    cd "$VALIDATION_DIR"
    eval $cmd

    local exit_code=$?

    if [ $exit_code -eq 0 ]; then
        echo ""
        echo "IMU validation complete!"
        echo "Please check the generated report files"
    else
        echo ""
        echo "IMU validation failed (exit code: $exit_code)"
        echo "Please check the error messages and log files"
    fi

    return $exit_code
}

# Main program logic
main() {
    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_help
                exit 0
                ;;
            --install-deps)
                "$SCRIPT_DIR/install_imu_validation_deps.sh"
                exit $?
                ;;
            --check-env)
                check_environment --strict
                exit $?
                ;;
            --interactive)
                check_environment
                interactive_mode
                exit $?
                ;;
            *)
                # Other arguments passed to validation function
                check_environment
                run_validation "$@"
                exit $?
                ;;
        esac
    done

    # If no arguments, run default validation
    check_environment
    echo "Starting default validation mode..."
    echo "Tip: Use --interactive to enter interactive mode"
    echo "     Use --help to see all options"
    echo ""

    run_validation
}

# Execute main program
main "$@"
