#!/bin/bash
# Unitree Go2 Audio I/O System Validation Startup Script
# Generated: 2025-06-30 13:06:45
# Platform: Ubuntu 20.04 - aarch64

set -e

# Script configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
VALIDATION_SCRIPT="$SCRIPT_DIR/audio_validation_main.py"

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
Unitree Go2 Audio I/O System Validation

Usage: $0 [options]

Options:
  -p, --phases PHASES     Validation phases to execute (A,B,C,D,E), default: A,B
  -c, --config CONFIG     Configuration file path
  -sr, --sample-rate RATE Sample rate (default: 44100)
  -ch, --channels NUM     Number of audio channels (default: 2)
  -d, --duration SECONDS  Test duration (default: 5.0)
  -i, --install           Install dependencies
  -h, --help              Show this help message

Phase descriptions:
  Phase A: Hardware connection and basic capture verification
  Phase B: Microphone array full-range testing
  Phase C: Speaker calibration and audio quality assessment (pending)
  Phase D: ROS2 audio topic integration verification (pending)
  Phase E: Comprehensive visualization and performance report generation (pending)

Examples:
  $0                                    # Run default validation (Phase A,B)
  $0 -p A B C                          # Run specified phases
  $0 -sr 48000 -ch 2 -d 10.0          # Custom audio parameters
  $0 -i                                 # Install dependencies
  $0 -c custom_config.json             # Use custom configuration

EOF
}

# Check dependencies
check_dependencies() {
    log_info "Checking Python dependencies..."

    local missing_deps=()

    # Check Python libraries
    for dep in sounddevice scipy librosa matplotlib numpy; do
        if ! python3 -c "import $dep" 2>/dev/null; then
            missing_deps+=("$dep")
        fi
    done

    if [ ${#missing_deps[@]} -gt 0 ]; then
        log_warning "Missing dependencies: ${missing_deps[*]}"
        log_info "Run '$0 -i' to install dependencies"
        return 1
    fi

    log_success "All dependencies satisfied"
    return 0
}

# Install dependencies
install_dependencies() {
    log_info "Starting audio validation dependency installation..."

    # Update package list
    log_info "Updating system package list..."
    sudo apt update

    # Install system dependencies
    log_info "Installing system audio dependencies..."
    sudo apt install -y \
        portaudio19-dev \
        libasound2-dev \
        libsndfile1-dev \
        libfftw3-dev \
        python3-pip \
        python3-dev

    # Install Python dependencies
    log_info "Installing Python audio processing libraries..."
    pip3 install --user \
        sounddevice \
        scipy \
        librosa \
        matplotlib \
        numpy \
        audio-common-msgs || log_warning "audio-common-msgs installation failed, ROS2 integration will be unavailable"

    log_success "Dependency installation complete!"
}

# Check audio devices
check_audio_devices() {
    log_info "Checking audio devices..."

    # Check ALSA devices
    if command -v aplay &> /dev/null; then
        log_info "Available audio playback devices:"
        aplay -l | grep -E "^card" || log_warning "No audio playback devices found"
    fi

    if command -v arecord &> /dev/null; then
        log_info "Available audio recording devices:"
        arecord -l | grep -E "^card" || log_warning "No audio recording devices found"
    fi

    # Check PulseAudio
    if command -v pactl &> /dev/null; then
        log_info "PulseAudio source devices:"
        pactl list short sources 2>/dev/null || log_warning "PulseAudio is not running"

        log_info "PulseAudio sink devices:"
        pactl list short sinks 2>/dev/null || log_warning "PulseAudio is not running"
    fi
}

# Pre-validation environment check
pre_validation_check() {
    log_info "Performing pre-validation checks..."

    # Check Python version
    python_version=$(python3 --version 2>&1 | awk '{print $2}')
    log_info "Python version: $python_version"

    # Check if in Unitree environment
    if [ -f "$PROJECT_ROOT/cyclonedx_ws/install/setup.bash" ]; then
        log_info "Unitree workspace detected"
        source "$PROJECT_ROOT/cyclonedx_ws/install/setup.bash" 2>/dev/null || true
    fi

    # Check ROS2 environment
    if command -v ros2 &> /dev/null; then
        log_info "ROS2 environment detected"
        export ROS2_AVAILABLE=1
    else
        log_warning "ROS2 environment not detected, ROS2 integration tests will be skipped"
        export ROS2_AVAILABLE=0
    fi

    # Check permissions
    if ! groups | grep -q audio; then
        log_warning "Current user is not in the audio group, may encounter audio device permission issues"
        log_info "Run: sudo usermod -a -G audio \$USER"
    fi

    check_audio_devices
}

# Main validation function
run_validation() {
    local phases="$1"
    local config="$2"
    local sample_rate="$3"
    local channels="$4"
    local duration="$5"

    log_info "Starting audio I/O system validation..."
    log_info "Phases: $phases"
    log_info "Sample rate: ${sample_rate}Hz"
    log_info "Channels: $channels"
    log_info "Test duration: ${duration}s"

    # Build command arguments
    local cmd_args=()

    if [ -n "$phases" ]; then
        IFS=',' read -ra PHASE_ARRAY <<< "$phases"
        cmd_args+=("--phases" "${PHASE_ARRAY[@]}")
    fi

    if [ -n "$config" ]; then
        cmd_args+=("--config" "$config")
    fi

    if [ -n "$sample_rate" ]; then
        cmd_args+=("--sample-rate" "$sample_rate")
    fi

    if [ -n "$channels" ]; then
        cmd_args+=("--channels" "$channels")
    fi

    if [ -n "$duration" ]; then
        cmd_args+=("--duration" "$duration")
    fi

    # Switch to project root directory
    cd "$PROJECT_ROOT"

    # Run validation script
    log_info "Executing validation script..."
    python3 "$VALIDATION_SCRIPT" "${cmd_args[@]}"

    local exit_code=$?

    if [ $exit_code -eq 0 ]; then
        log_success "Audio validation complete!"
    else
        log_error "Audio validation failed (exit code: $exit_code)"
        return $exit_code
    fi
}

# Main function
main() {
    local phases=""
    local config=""
    local sample_rate="44100"
    local channels="2"
    local duration="5.0"
    local install_deps=false

    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -p|--phases)
                phases="$2"
                shift 2
                ;;
            -c|--config)
                config="$2"
                shift 2
                ;;
            -sr|--sample-rate)
                sample_rate="$2"
                shift 2
                ;;
            -ch|--channels)
                channels="$2"
                shift 2
                ;;
            -d|--duration)
                duration="$2"
                shift 2
                ;;
            -i|--install)
                install_deps=true
                shift
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            *)
                log_error "Unknown argument: $1"
                show_help
                exit 1
                ;;
        esac
    done

    # Display startup information
    echo "Unitree Go2 Audio I/O System Validation"
    echo "=========================================="
    echo "Time: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "Platform: $(uname -a)"
    echo "Project: $PROJECT_ROOT"
    echo ""

    # Install dependencies mode
    if [ "$install_deps" = true ]; then
        install_dependencies
        exit 0
    fi

    # Check if validation script exists
    if [ ! -f "$VALIDATION_SCRIPT" ]; then
        log_error "Validation script not found: $VALIDATION_SCRIPT"
        exit 1
    fi

    # Check dependencies
    if ! check_dependencies; then
        log_error "Dependency check failed, please install dependencies first"
        exit 1
    fi

    # Pre-validation check
    pre_validation_check

    # Run validation
    run_validation "$phases" "$config" "$sample_rate" "$channels" "$duration"
}

# Script entry point
main "$@"
