#!/bin/bash
# scripts/validation/foot_force/run_static_validation.sh
# Generated: 2025-06-27 14:30:00 CST
# Purpose: Unitree Go2 foot force sensor static validation startup script

set -e

# Script constants
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VALIDATION_DIR="${SCRIPT_DIR}/foot_force_validation"
MAIN_SCRIPT="${VALIDATION_DIR}/static_validation.py"
CONFIG_FILE="${VALIDATION_DIR}/validation_config.json"

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Log functions
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

log_header() {
    echo -e "${PURPLE}$1${NC}"
}

# Display header
show_header() {
    clear
    echo -e "${CYAN}"
    echo "================================================================================"
    echo "               Unitree Go2 Foot Force Sensor Static Validation System"
    echo "================================================================================"
    echo "Version: 1.0.0"
    echo "Generated: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "Script location: ${SCRIPT_DIR}"
    echo "================================================================================"
    echo -e "${NC}"
}

# Check system environment
check_environment() {
    log_header "System Environment Check"

    local all_ok=true

    # Check Python version
    if command -v python3 &> /dev/null; then
        local python_version=$(python3 --version | cut -d' ' -f2)
        log_success "Python3 version: ${python_version}"
    else
        log_error "Python3 not installed"
        all_ok=false
    fi

    # Check required Python modules
    local required_modules=("numpy" "matplotlib" "scipy" "pandas")
    for module in "${required_modules[@]}"; do
        if python3 -c "import ${module}" &> /dev/null; then
            log_success "Python module ${module} available"
        else
            log_error "Python module ${module} not installed"
            all_ok=false
        fi
    done

    # Check Unitree SDK
    if python3 -c "from unitree_sdk2py.core.channel import ChannelSubscriber" &> /dev/null; then
        log_success "Unitree SDK2 Python available"
    else
        log_warning "Unitree SDK2 Python not available, will use simulation mode"
    fi

    # Check required files
    if [[ -f "${MAIN_SCRIPT}" ]]; then
        log_success "Main script file exists: ${MAIN_SCRIPT}"
    else
        log_error "Main script file not found: ${MAIN_SCRIPT}"
        all_ok=false
    fi

    if [[ -f "${CONFIG_FILE}" ]]; then
        log_success "Configuration file exists: ${CONFIG_FILE}"
    else
        log_warning "Configuration file not found: ${CONFIG_FILE}"
    fi

    # Check directory permissions
    if [[ -w "${VALIDATION_DIR}" ]]; then
        log_success "Validation directory is writable"
    else
        log_error "Validation directory is not writable: ${VALIDATION_DIR}"
        all_ok=false
    fi

    # Check disk space
    local available_space=$(df "${VALIDATION_DIR}" | tail -1 | awk '{print $4}')
    local available_mb=$((available_space / 1024))

    if [[ ${available_mb} -gt 1000 ]]; then
        log_success "Sufficient disk space: ${available_mb}MB"
    else
        log_warning "Low disk space: ${available_mb}MB (at least 1GB recommended)"
    fi

    echo ""

    if [[ "${all_ok}" == true ]]; then
        log_success "Environment check passed!"
        return 0
    else
        log_error "Environment check failed!"
        return 1
    fi
}

# Display menu
show_menu() {
    echo ""
    log_header "Static Validation Options Menu"
    echo ""
    echo "1. Full Static Validation (recommended)"
    echo "2. Quick Test Mode"
    echo "3. Data Analysis and Visualization Only"
    echo "4. Custom Parameter Validation"
    echo "5. View Historical Reports"
    echo "6. Clean Up Output Files"
    echo "7. Show Help Information"
    echo "8. Exit"
    echo ""
}

# Get user selection
get_user_choice() {
    local choice
    while true; do
        read -p "Select an option [1-8]: " choice
        case $choice in
            [1-8])
                echo $choice
                return 0
                ;;
            *)
                log_warning "Invalid selection, please enter 1-8"
                ;;
        esac
    done
}

# Confirm user action
confirm_action() {
    local prompt="$1"
    local response

    while true; do
        read -p "${prompt} [y/N]: " response
        case $response in
            [Yy]|[Yy][Ee][Ss])
                return 0
                ;;
            [Nn]|[Nn][Oo]|"")
                return 1
                ;;
            *)
                log_warning "Please enter y (yes) or n (no)"
                ;;
        esac
    done
}

# Run full static validation
run_full_validation() {
    log_header "Running Full Static Validation"

    echo ""
    echo "Full static validation includes the following test items:"
    echo "  - Zero-load test (robot suspended state)"
    echo "  - Static standing test (robot standing normally)"
    echo "  - Zero-point drift analysis (long-term stability)"
    echo "  - Comprehensive data analysis"
    echo "  - Visualization chart generation"
    echo ""

    local estimated_time=15
    log_info "Estimated total time: ${estimated_time} minutes"

    if confirm_action "Confirm starting full static validation?"; then
        log_info "Starting full static validation..."
        cd "${VALIDATION_DIR}"
        python3 static_validation.py --config validation_config.json --log-level INFO
    else
        log_info "Operation cancelled"
    fi
}

# Run quick test
run_quick_test() {
    log_header "Running Quick Test Mode"

    echo ""
    echo "Quick test mode features:"
    echo "  - Shortened test times (zero-load: 10s, standing: 20s, drift: 60s)"
    echo "  - Includes complete test workflow"
    echo "  - Suitable for system debugging and preliminary validation"
    echo ""

    local estimated_time=3
    log_info "Estimated total time: ${estimated_time} minutes"

    if confirm_action "Confirm starting quick test?"; then
        log_info "Starting quick test mode..."
        cd "${VALIDATION_DIR}"
        python3 static_validation.py --config validation_config.json --test-mode --log-level INFO
    else
        log_info "Operation cancelled"
    fi
}

# Run analysis and visualization only
run_analysis_only() {
    log_header "Running Data Analysis and Visualization Only"

    echo ""
    echo "This option will:"
    echo "  - Skip data collection process"
    echo "  - Use existing data for analysis"
    echo "  - Generate visualization charts"
    echo "  - Generate analysis report"
    echo ""

    log_warning "Note: Requires previously collected data files"

    if confirm_action "Confirm running analysis and visualization only?"; then
        log_info "Starting analysis and visualization..."
        cd "${VALIDATION_DIR}"
        python3 static_validation.py --config validation_config.json --skip-data-collection --log-level INFO
    else
        log_info "Operation cancelled"
    fi
}

# Custom parameter validation
run_custom_validation() {
    log_header "Custom Parameter Validation"

    echo ""
    echo "Please configure test parameters:"

    # Zero-load test duration
    local zero_load_duration
    while true; do
        read -p "Zero-load test duration (seconds) [default: 30]: " zero_load_duration
        zero_load_duration=${zero_load_duration:-30}
        if [[ "$zero_load_duration" =~ ^[0-9]+$ ]] && [[ "$zero_load_duration" -ge 5 ]] && [[ "$zero_load_duration" -le 300 ]]; then
            break
        else
            log_warning "Please enter an integer between 5-300"
        fi
    done

    # Static standing test duration
    local standing_duration
    while true; do
        read -p "Static standing test duration (seconds) [default: 60]: " standing_duration
        standing_duration=${standing_duration:-60}
        if [[ "$standing_duration" =~ ^[0-9]+$ ]] && [[ "$standing_duration" -ge 10 ]] && [[ "$standing_duration" -le 600 ]]; then
            break
        else
            log_warning "Please enter an integer between 10-600"
        fi
    done

    # Zero-point drift analysis duration
    local drift_duration
    while true; do
        read -p "Zero-point drift analysis duration (seconds) [default: 300]: " drift_duration
        drift_duration=${drift_duration:-300}
        if [[ "$drift_duration" =~ ^[0-9]+$ ]] && [[ "$drift_duration" -ge 60 ]] && [[ "$drift_duration" -le 1800 ]]; then
            break
        else
            log_warning "Please enter an integer between 60-1800"
        fi
    done

    # Log level
    local log_level
    echo ""
    echo "Select log level:"
    echo "1. DEBUG (detailed debug information)"
    echo "2. INFO (standard information)"
    echo "3. WARNING (warnings and errors only)"
    echo "4. ERROR (errors only)"

    while true; do
        read -p "Log level [1-4, default: 2]: " log_choice
        log_choice=${log_choice:-2}
        case $log_choice in
            1) log_level="DEBUG"; break ;;
            2) log_level="INFO"; break ;;
            3) log_level="WARNING"; break ;;
            4) log_level="ERROR"; break ;;
            *) log_warning "Please enter 1-4" ;;
        esac
    done

    # Whether to skip visualization
    local skip_viz=""
    if confirm_action "Skip visualization generation to speed up?"; then
        skip_viz="--skip-visualization"
    fi

    echo ""
    log_info "Configuration summary:"
    log_info "  Zero-load test: ${zero_load_duration} seconds"
    log_info "  Static standing test: ${standing_duration} seconds"
    log_info "  Zero-point drift analysis: ${drift_duration} seconds"
    log_info "  Log level: ${log_level}"
    log_info "  Skip visualization: $([ -n "$skip_viz" ] && echo "yes" || echo "no")"

    local total_time=$((zero_load_duration + standing_duration + drift_duration / 60 + 2))
    log_info "  Estimated total time: ${total_time} minutes"

    if confirm_action "Confirm starting custom validation?"; then
        log_info "Starting custom validation..."

        # Create temporary configuration file
        local temp_config="/tmp/custom_validation_config.json"
        cp "${CONFIG_FILE}" "${temp_config}"

        # Modify configuration parameters
        python3 -c "
import json
with open('${temp_config}', 'r') as f:
    config = json.load(f)
config['static_validation']['zero_load_test_duration'] = ${zero_load_duration}
config['static_validation']['static_standing_duration'] = ${standing_duration}
config['static_validation']['zero_drift_duration'] = ${drift_duration}
with open('${temp_config}', 'w') as f:
    json.dump(config, f, indent=2)
"

        cd "${VALIDATION_DIR}"
        python3 static_validation.py --config "${temp_config}" --log-level "${log_level}" ${skip_viz}

        # Clean up temporary files
        rm -f "${temp_config}"
    else
        log_info "Operation cancelled"
    fi
}

# View historical reports
view_reports() {
    log_header "View Historical Reports"

    local output_dir="${VALIDATION_DIR}/output"

    if [[ ! -d "${output_dir}" ]]; then
        log_warning "Output directory does not exist: ${output_dir}"
        return
    fi

    # Find report files
    local reports=($(find "${output_dir}" -name "*final_report*.json" -type f | sort -r))

    if [[ ${#reports[@]} -eq 0 ]]; then
        log_warning "No historical report files found"
        return
    fi

    echo ""
    echo "Found ${#reports[@]} historical reports:"
    echo ""

    for i in "${!reports[@]}"; do
        local report="${reports[$i]}"
        local basename=$(basename "${report}")
        local timestamp=$(stat -c %y "${report}" | cut -d' ' -f1-2)
        local size=$(du -h "${report}" | cut -f1)

        echo "$((i+1)). ${basename}"
        echo "   Time: ${timestamp}"
        echo "   Size: ${size}"
        echo ""
    done

    while true; do
        read -p "Select report number [1-${#reports[@]}, 0=back]: " report_choice

        if [[ "$report_choice" == "0" ]]; then
            return
        elif [[ "$report_choice" =~ ^[0-9]+$ ]] && [[ "$report_choice" -ge 1 ]] && [[ "$report_choice" -le ${#reports[@]} ]]; then
            local selected_report="${reports[$((report_choice-1))]}"
            log_info "Displaying report: $(basename "${selected_report}")"

            # Use jq for formatted display (if available)
            if command -v jq &> /dev/null; then
                cat "${selected_report}" | jq '.'
            else
                cat "${selected_report}"
            fi

            echo ""
            read -p "Press Enter to continue..."
            break
        else
            log_warning "Invalid selection"
        fi
    done
}

# Clean up output files
cleanup_outputs() {
    log_header "Clean Up Output Files"

    local output_dir="${VALIDATION_DIR}/output"
    local log_dir="${VALIDATION_DIR}/logs"

    echo ""
    echo "Cleanable content:"
    echo "1. All files in output directory (${output_dir})"
    echo "2. All files in log directory (${log_dir})"
    echo "3. Temporary files and cache"
    echo "4. Clean up everything"
    echo "5. Return to main menu"
    echo ""

    while true; do
        read -p "Select content to clean [1-5]: " cleanup_choice

        case $cleanup_choice in
            1)
                if [[ -d "${output_dir}" ]]; then
                    local file_count=$(find "${output_dir}" -type f | wc -l)
                    if [[ $file_count -gt 0 ]]; then
                        log_info "Output directory contains ${file_count} files"
                        if confirm_action "Confirm cleaning output directory?"; then
                            rm -rf "${output_dir}"/*
                            log_success "Output directory cleaned"
                        fi
                    else
                        log_info "Output directory is already empty"
                    fi
                else
                    log_info "Output directory does not exist"
                fi
                break
                ;;
            2)
                if [[ -d "${log_dir}" ]]; then
                    local file_count=$(find "${log_dir}" -type f | wc -l)
                    if [[ $file_count -gt 0 ]]; then
                        log_info "Log directory contains ${file_count} files"
                        if confirm_action "Confirm cleaning log directory?"; then
                            rm -rf "${log_dir}"/*
                            log_success "Log directory cleaned"
                        fi
                    else
                        log_info "Log directory is already empty"
                    fi
                else
                    log_info "Log directory does not exist"
                fi
                break
                ;;
            3)
                log_info "Cleaning temporary files and cache..."
                find "${VALIDATION_DIR}" -name "*.pyc" -delete 2>/dev/null || true
                find "${VALIDATION_DIR}" -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
                find "${VALIDATION_DIR}" -name "*.tmp" -delete 2>/dev/null || true
                rm -f /tmp/custom_validation_config.json 2>/dev/null || true
                log_success "Temporary files cleaned"
                break
                ;;
            4)
                log_warning "This will delete all output files, logs, and temporary files"
                if confirm_action "Confirm cleaning everything?"; then
                    [[ -d "${output_dir}" ]] && rm -rf "${output_dir}"/*
                    [[ -d "${log_dir}" ]] && rm -rf "${log_dir}"/*
                    find "${VALIDATION_DIR}" -name "*.pyc" -delete 2>/dev/null || true
                    find "${VALIDATION_DIR}" -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
                    find "${VALIDATION_DIR}" -name "*.tmp" -delete 2>/dev/null || true
                    rm -f /tmp/custom_validation_config.json 2>/dev/null || true
                    log_success "All files cleaned"
                fi
                break
                ;;
            5)
                return
                ;;
            *)
                log_warning "Invalid selection, please enter 1-5"
                ;;
        esac
    done
}

# Show help information
show_help() {
    log_header "Help Information"

    echo ""
    echo "Unitree Go2 Foot Force Sensor Static Validation System Help"
    echo ""
    echo "System Overview:"
    echo "  This system validates the accuracy, stability, and consistency of the"
    echo "  Unitree Go2 robot's foot force sensors. It uses multiple tests to ensure"
    echo "  sensor reliability under static conditions."
    echo ""
    echo "Test Items:"
    echo "  1. Zero-load test     - Zero-point verification with robot suspended"
    echo "  2. Static standing    - Force distribution verification while standing normally"
    echo "  3. Zero-point drift   - Long-term stability monitoring"
    echo "  4. Comprehensive analysis - Statistical analysis, frequency domain analysis, anomaly detection"
    echo "  5. Visualization report   - Chart and dashboard generation"
    echo ""
    echo "System Requirements:"
    echo "  - Python 3.6+"
    echo "  - NumPy, SciPy, Matplotlib, Pandas"
    echo "  - Unitree SDK2 Python (optional, simulation mode available)"
    echo "  - At least 1GB disk space"
    echo ""
    echo "Robot Preparation:"
    echo "  - Zero-load test: Robot fully suspended, feet not touching any surface"
    echo "  - Static standing test: Robot standing normally, all four feet in stable contact with ground"
    echo "  - Keep test environment quiet, avoid vibration interference"
    echo ""
    echo "Output Files:"
    echo "  - Validation report (JSON format)"
    echo "  - Raw data (CSV format)"
    echo "  - Visualization charts (PNG format)"
    echo "  - Log files"
    echo ""
    echo "Troubleshooting:"
    echo "  - Check Python module installation: pip3 install numpy scipy matplotlib pandas"
    echo "  - Verify Unitree SDK connection is working"
    echo "  - Check that disk space is sufficient"
    echo "  - Review log files for detailed error information"
    echo ""
    echo "Technical Support:"
    echo "  - See README.md for detailed documentation"
    echo "  - Check logs/ directory for diagnostic information"
    echo "  - Run environment check to verify system status"
    echo ""

    read -p "Press Enter to return to main menu..."
}

# Main loop
main_loop() {
    while true; do
        show_menu
        local choice=$(get_user_choice)

        echo ""
        case $choice in
            1)
                run_full_validation
                ;;
            2)
                run_quick_test
                ;;
            3)
                run_analysis_only
                ;;
            4)
                run_custom_validation
                ;;
            5)
                view_reports
                ;;
            6)
                cleanup_outputs
                ;;
            7)
                show_help
                ;;
            8)
                log_info "Thank you for using this tool! Goodbye!"
                exit 0
                ;;
        esac

        echo ""
        read -p "Press Enter to continue..."
    done
}

# Main function
main() {
    # Set signal handling
    trap 'echo ""; log_warning "Operation interrupted"; exit 130' INT TERM

    # Display header
    show_header

    # Check environment
    if ! check_environment; then
        echo ""
        log_error "Environment check failed. Please resolve the issues above and try again"

        if confirm_action "Would you like to view troubleshooting help?"; then
            show_help
        fi

        exit 1
    fi

    # Enter main loop
    main_loop
}

# Check if running directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
