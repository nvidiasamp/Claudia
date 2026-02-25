#!/bin/bash
# Claudia Robot Project Daily Cleanup Script
# Generated: 2025-06-26 18:43:27
# Purpose: Automatically clean temporary files, log files, build caches, etc.

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

# Check system status
check_system_status() {
    log_info "Checking system status..."

    # Disk space check
    local disk_usage=$(df . | tail -1 | awk '{print $5}' | sed 's/%//')
    if [ "$disk_usage" -gt 90 ]; then
        log_warning "Low disk space: ${disk_usage}%"
        echo "Starting emergency cleanup..."
    else
        log_info "Disk usage: ${disk_usage}%"
    fi

    # Memory check
    local mem_usage=$(free | grep Mem | awk '{printf "%.0f", $3/$2 * 100.0}')
    if [ "$mem_usage" -gt 90 ]; then
        log_warning "High memory usage: ${mem_usage}%"
    else
        log_info "Memory usage: ${mem_usage}%"
    fi
}

# Clean build files
cleanup_build_files() {
    log_info "Cleaning build files..."
    local cleaned_count=0

    # Python cache files
    if [ -n "$(find . -name "*.pyc" -type f 2>/dev/null)" ]; then
        find . -name "*.pyc" -delete
        cleaned_count=$((cleaned_count + $(find . -name "*.pyc" -type f 2>/dev/null | wc -l)))
    fi

    # __pycache__ directories
    if [ -n "$(find . -name "__pycache__" -type d 2>/dev/null)" ]; then
        find . -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
        cleaned_count=$((cleaned_count + 1))
    fi

    # pytest cache
    if [ -n "$(find . -name ".pytest_cache" -type d 2>/dev/null)" ]; then
        find . -name ".pytest_cache" -type d -exec rm -rf {} + 2>/dev/null || true
        cleaned_count=$((cleaned_count + 1))
    fi

    # ROS2 build cache (but keep install directory)
    if [ -d "cyclonedds_ws/build" ]; then
        local build_size=$(du -sh cyclonedds_ws/build 2>/dev/null | cut -f1)
        log_info "ROS2 build cache size: $build_size"
        # Only clean build cache larger than 1GB
        local build_size_mb=$(du -sm cyclonedds_ws/build 2>/dev/null | cut -f1)
        if [ "$build_size_mb" -gt 1024 ]; then
            log_warning "Build cache is too large (${build_size}MB), manual cleanup recommended"
        fi
    fi

    log_success "Build file cleanup complete (cleaned $cleaned_count items)"
}

# Clean editor temporary files
cleanup_editor_temp() {
    log_info "Cleaning editor temporary files..."
    local cleaned_count=0

    # Vim/Emacs temporary files
    for pattern in "*~" ".#*" "#*#" ".*.swp" ".*.swo"; do
        if [ -n "$(find . -name "$pattern" -type f 2>/dev/null)" ]; then
            local count=$(find . -name "$pattern" -type f -delete -print 2>/dev/null | wc -l)
            cleaned_count=$((cleaned_count + count))
        fi
    done

    # VS Code temporary files
    if [ -d ".vscode" ]; then
        find .vscode -name "*.log" -mtime +7 -delete 2>/dev/null || true
    fi

    log_success "Editor temporary file cleanup complete (cleaned $cleaned_count files)"
}

# Clean log files
cleanup_logs() {
    log_info "Cleaning log files..."

    if [ ! -d "logs" ]; then
        log_info "Log directory does not exist, skipping log cleanup"
        return 0
    fi

    local total_logs=$(find logs/ -name "*.log" 2>/dev/null | wc -l)
    local old_logs=0
    local compressed_logs=0

    # Delete logs older than 30 days
    if [ -n "$(find logs/ -name "*.log" -mtime +30 2>/dev/null)" ]; then
        old_logs=$(find logs/ -name "*.log" -mtime +30 -delete -print 2>/dev/null | wc -l)
    fi

    # Compress logs older than 7 days
    if [ -n "$(find logs/ -name "*.log" -mtime +7 2>/dev/null)" ]; then
        find logs/ -name "*.log" -mtime +7 -exec gzip {} \; 2>/dev/null || true
        compressed_logs=$(find logs/ -name "*.log.gz" -mtime +7 2>/dev/null | wc -l)
    fi

    # Display log directory size
    if [ -d "logs" ]; then
        local logs_size=$(du -sh logs/ 2>/dev/null | cut -f1)
        log_info "Log directory size: $logs_size"
    fi

    log_success "Log cleanup complete (total logs: $total_logs, deleted old: $old_logs, compressed: $compressed_logs)"
}

# Clean temporary download files
cleanup_downloads() {
    log_info "Cleaning temporary download files..."

    if [ ! -d "tmp" ]; then
        log_info "Temporary directory does not exist, skipping download file cleanup"
        return 0
    fi

    local cleaned_count=0

    # Clean failed downloads
    if [ -n "$(find tmp/ -name "*.tmp" 2>/dev/null)" ]; then
        cleaned_count=$(find tmp/ -name "*.tmp" -delete -print 2>/dev/null | wc -l)
    fi

    # Clean empty files
    if [ -n "$(find tmp/ -size 0 2>/dev/null)" ]; then
        local empty_count=$(find tmp/ -size 0 -delete -print 2>/dev/null | wc -l)
        cleaned_count=$((cleaned_count + empty_count))
    fi

    # Clean download files older than 7 days
    if [ -n "$(find tmp/downloads/ -type f -mtime +7 2>/dev/null)" ]; then
        local old_downloads=$(find tmp/downloads/ -type f -mtime +7 -delete -print 2>/dev/null | wc -l)
        cleaned_count=$((cleaned_count + old_downloads))
    fi

    log_success "Download file cleanup complete (cleaned $cleaned_count files)"
}

# Clean empty directories
cleanup_empty_dirs() {
    log_info "Cleaning empty directories..."

    # Clean empty directories, excluding .git directory
    local empty_dirs=$(find . -type d -empty -not -path "./.git*" 2>/dev/null | wc -l)
    if [ "$empty_dirs" -gt 0 ]; then
        find . -type d -empty -not -path "./.git*" -delete 2>/dev/null || true
        log_success "Cleaned $empty_dirs empty directories"
    else
        log_info "No empty directories found"
    fi
}

# Git status check
check_git_status() {
    if ! git rev-parse --git-dir > /dev/null 2>&1; then
        log_info "Not in a Git repository, skipping Git status check"
        return 0
    fi

    log_info "Checking Git status..."

    # Check untracked files
    local untracked=$(git ls-files --others --exclude-standard | wc -l)
    if [ "$untracked" -gt 0 ]; then
        log_warning "Found $untracked untracked files"
        echo "Untracked files list:"
        git ls-files --others --exclude-standard | head -10
        if [ "$untracked" -gt 10 ]; then
            echo "... ($((untracked - 10)) more files)"
        fi
    fi

    # Check uncommitted changes
    if ! git diff --quiet; then
        log_warning "There are uncommitted changes"
    else
        log_info "Working directory is clean"
    fi
}

# Main cleanup function
main_cleanup() {
    local start_time=$(date '+%s')
    local cleanup_date=$(date '+%Y-%m-%d %H:%M:%S %Z')

    echo "=================================================="
    echo "Claudia Robot Project Daily Cleanup"
    echo "Start time: $cleanup_date"
    echo "=================================================="

    # System status check
    check_system_status

    echo ""
    echo "Starting cleanup operations..."

    # Execute various cleanups
    cleanup_build_files
    cleanup_editor_temp
    cleanup_logs
    cleanup_downloads
    cleanup_empty_dirs

    echo ""
    echo "Final status check..."

    # Git status check
    check_git_status

    # Final system status
    check_system_status

    # Calculate elapsed time
    local end_time=$(date '+%s')
    local duration=$((end_time - start_time))

    echo ""
    echo "=================================================="
    echo "Cleanup complete!"
    echo "End time: $(date '+%Y-%m-%d %H:%M:%S %Z')"
    echo "Total elapsed: ${duration} seconds"
    echo "=================================================="

    # Record to cleanup log
    mkdir -p logs/maintenance
    echo "$(date '+%Y-%m-%d %H:%M:%S') | daily_cleanup | ${duration}s | success" >> logs/maintenance/cleanup.log
}

# Error handling
handle_error() {
    local exit_code=$?
    local line_number=$1

    log_error "Cleanup script failed at line $line_number (exit code: $exit_code)"

    # Record error
    mkdir -p logs/maintenance
    echo "$(date '+%Y-%m-%d %H:%M:%S') | daily_cleanup | error | line:$line_number | exit:$exit_code" >> logs/maintenance/cleanup.log

    exit $exit_code
}

# Set error handling
trap 'handle_error $LINENO' ERR

# Help information
show_help() {
    echo "Claudia Robot Project Daily Cleanup Script"
    echo ""
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  -h, --help     Show help information"
    echo "  -q, --quiet    Quiet mode, show only important information"
    echo "  -v, --verbose  Verbose mode, show more information"
    echo ""
    echo "Examples:"
    echo "  $0              # Normal cleanup"
    echo "  $0 --quiet      # Quiet cleanup"
    echo "  $0 --verbose    # Verbose cleanup"
}

# Argument handling
QUIET_MODE=false
VERBOSE_MODE=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        -q|--quiet)
            QUIET_MODE=true
            shift
            ;;
        -v|--verbose)
            VERBOSE_MODE=true
            shift
            ;;
        *)
            log_error "Unknown argument: $1"
            echo "Use $0 --help for help"
            exit 1
            ;;
    esac
done

# Adjust output based on mode
if [ "$QUIET_MODE" = true ]; then
    # Quiet mode: redirect most output
    main_cleanup 2>&1 | grep -E "(SUCCESS|ERROR|WARNING|Cleanup complete)" || true
elif [ "$VERBOSE_MODE" = true ]; then
    # Verbose mode: show debug information
    set -x
    main_cleanup
else
    # Normal mode
    main_cleanup
fi
