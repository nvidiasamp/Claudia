#!/bin/bash
# Claudia LLM Deployment & Management Script
# Automated deployment and management of Claudia robot's optimized LLM models
# Generated: $(date '+%Y-%m-%d %H:%M:%S')

set -e

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Project paths
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MODELFILE_PATH="$PROJECT_ROOT/ClaudiaOptimizedModelfile_v2_3B"
LOG_DIR="$PROJECT_ROOT/logs/llm"
TEST_SCRIPT="$PROJECT_ROOT/test/test_claudia_llm_performance.py"

# Create log directory
mkdir -p "$LOG_DIR"

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
    echo "$(date '+%Y-%m-%d %H:%M:%S') [INFO] $1" >> "$LOG_DIR/deployment.log"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
    echo "$(date '+%Y-%m-%d %H:%M:%S') [SUCCESS] $1" >> "$LOG_DIR/deployment.log"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
    echo "$(date '+%Y-%m-%d %H:%M:%S') [WARNING] $1" >> "$LOG_DIR/deployment.log"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
    echo "$(date '+%Y-%m-%d %H:%M:%S') [ERROR] $1" >> "$LOG_DIR/deployment.log"
}

# Check dependencies
check_dependencies() {
    log_info "Checking dependencies..."

    if ! command -v ollama &> /dev/null; then
        log_error "Ollama is not installed. Please install Ollama first."
        exit 1
    fi

    if ! command -v python3 &> /dev/null; then
        log_error "Python3 is not installed."
        exit 1
    fi

    if [ ! -f "$MODELFILE_PATH" ]; then
        log_error "Model file does not exist: $MODELFILE_PATH"
        exit 1
    fi

    log_success "Dependency check passed"
}

# Check system resources
check_system_resources() {
    log_info "Checking system resources..."

    # Check memory
    local mem_total=$(free -m | awk '/^Mem:/{print $2}')
    local mem_available=$(free -m | awk '/^Mem:/{print $7}')

    if [ "$mem_available" -lt 2048 ]; then
        log_warning "Available memory less than 2GB, may affect model performance"
    fi

    # Check disk space
    local disk_usage=$(df "$PROJECT_ROOT" | tail -1 | awk '{print $5}' | sed 's/%//')
    if [ "$disk_usage" -gt 90 ]; then
        log_warning "High disk usage: ${disk_usage}%"
    fi

    log_info "Memory: ${mem_available}MB available / ${mem_total}MB total"
    log_info "Disk usage: ${disk_usage}%"
}

# Deploy model
deploy_model() {
    local model_name="${1:-claudia-v3.2:3b}"

    log_info "Deploying model: $model_name"

    # Check if already exists
    if ollama list | grep -q "$model_name"; then
        log_warning "Model $model_name already exists, recreating..."
        ollama rm "$model_name" 2>/dev/null || true
    fi

    # Create model
    log_info "Creating model..."
    if ollama create "$model_name" -f "$MODELFILE_PATH"; then
        log_success "Model $model_name created successfully"
    else
        log_error "Model creation failed"
        exit 1
    fi
}

# Run performance test
run_performance_test() {
    local model_name="${1:-claudia-v3.2:3b}"

    log_info "Running performance test: $model_name"

    if [ ! -f "$TEST_SCRIPT" ]; then
        log_error "Test script does not exist: $TEST_SCRIPT"
        return 1
    fi

    # Run test and save results
    local test_output="$LOG_DIR/test_$(date '+%Y%m%d_%H%M%S').log"
    if python3 "$TEST_SCRIPT" --model "$model_name" > "$test_output" 2>&1; then
        log_success "Performance test complete, results saved to: $test_output"

        # Extract accuracy
        local accuracy=$(grep "Overall Accuracy" "$test_output" | grep -o '[0-9.]*%' || echo "unknown")
        log_info "Model accuracy: $accuracy"
    else
        log_error "Performance test failed"
        return 1
    fi
}

# Quick test
quick_test() {
    local model_name="${1:-claudia-v3.2:3b}"

    log_info "Running quick functionality test..."

    # Japanese robot commands and expected Japanese responses (functional test data)
    local test_commands=("座って" "停止" "ダンス" "バランス")
    local expected_outputs=("座ります" "緊急停止" "踊ります" "バランス調整")

    local passed=0
    local total=${#test_commands[@]}

    for i in "${!test_commands[@]}"; do
        local cmd="${test_commands[$i]}"
        local expected="${expected_outputs[$i]}"

        log_info "Test: '$cmd'"
        local output=$(timeout 10 ollama run "$model_name" "$cmd" 2>/dev/null | grep -v "^$" | tail -1)

        if [[ "$output" == *"$expected"* ]]; then
            log_success "'$cmd' -> '$output'"
            ((passed++))
        else
            log_error "'$cmd' -> '$output' (expected: $expected)"
        fi
    done

    local accuracy=$((passed * 100 / total))
    log_info "Quick test accuracy: $accuracy% ($passed/$total)"

    if [ "$accuracy" -ge 75 ]; then
        log_success "Quick test passed"
        return 0
    else
        log_error "Quick test failed"
        return 1
    fi
}

# Monitor model performance
monitor_model() {
    local model_name="${1:-claudia-v3.2:3b}"
    local duration="${2:-60}"

    log_info "Monitoring model performance for $duration seconds..."

    local start_time=$(date +%s)
    local end_time=$((start_time + duration))
    local test_count=0
    local success_count=0

    while [ $(date +%s) -lt $end_time ]; do
        # Japanese robot command and expected response (functional test data)
        local test_cmd="座って"
        local expected="座ります"

        local start_response=$(date +%s.%3N)
        local output=$(ollama run "$model_name" "$test_cmd" 2>/dev/null | tail -1)
        local end_response=$(date +%s.%3N)

        local response_time=$(echo "$end_response - $start_response" | bc)

        ((test_count++))

        if [[ "$output" == *"$expected"* ]]; then
            ((success_count++))
            log_info "Test $test_count: PASS ${response_time}s"
        else
            log_warning "Test $test_count: FAIL ${response_time}s"
        fi

        sleep 2
    done

    local success_rate=$((success_count * 100 / test_count))
    log_info "Monitoring results: $success_rate% success rate ($success_count/$test_count)"
}

# List available models
list_models() {
    log_info "Available Claudia models:"
    ollama list | grep -E "(claudia|kura)" || log_warning "No Claudia-related models found"
}

# Switch model
switch_model() {
    local new_model="$1"

    if [ -z "$new_model" ]; then
        log_error "Please specify the model name to switch to"
        return 1
    fi

    if ! ollama list | grep -q "$new_model"; then
        log_error "Model $new_model does not exist"
        return 1
    fi

    # Update config file (if exists)
    local config_file="$PROJECT_ROOT/.env"
    if [ -f "$config_file" ]; then
        if grep -q "CLAUDIA_MODEL" "$config_file"; then
            sed -i "s/CLAUDIA_MODEL=.*/CLAUDIA_MODEL=$new_model/" "$config_file"
        else
            echo "CLAUDIA_MODEL=$new_model" >> "$config_file"
        fi
        log_success "Switched to model: $new_model"
    else
        log_warning "Config file does not exist, please manually configure model: $new_model"
    fi
}

# Backup model
backup_model() {
    local model_name="${1:-claudia-v3.2:3b}"
    local backup_name="${model_name}-backup-$(date '+%Y%m%d_%H%M%S')"

    log_info "Backing up model $model_name as $backup_name"

    # Create backup (by recreating)
    if ollama create "$backup_name" -f "$MODELFILE_PATH"; then
        log_success "Model backup successful: $backup_name"
    else
        log_error "Model backup failed"
        return 1
    fi
}

# Clean old models
cleanup_old_models() {
    log_info "Cleaning old Claudia models..."

    # List all claudia models (except the latest)
    local models=$(ollama list | grep -E "claudia.*:3b" | awk '{print $1}' | head -n -1)

    if [ -z "$models" ]; then
        log_info "No old models need cleaning"
        return 0
    fi

    for model in $models; do
        if [[ "$model" == *"backup"* ]]; then
            continue  # Keep backup models
        fi

        log_warning "Deleting old model: $model"
        ollama rm "$model" 2>/dev/null || true
    done

    log_success "Old model cleanup complete"
}

# Show help
show_help() {
    cat << 'EOF'
Claudia LLM Deployment & Management Script

Usage:
  ./deploy_claudia_llm.sh [command] [options]

Commands:
  deploy [model_name]    - Deploy specified model (default: claudia-v3.2:3b)
  test [model_name]      - Run full performance test
  quick [model_name]     - Run quick functionality test
  monitor [model_name] [seconds] - Monitor model performance
  list                   - List all available models
  switch [model_name]    - Switch current model in use
  backup [model_name]    - Backup specified model
  cleanup                - Clean old models
  full [model_name]      - Full deployment flow (deploy+test+monitor)

Examples:
  ./deploy_claudia_llm.sh deploy              # Deploy default model
  ./deploy_claudia_llm.sh test claudia-v3.2:3b # Test specified model
  ./deploy_claudia_llm.sh quick               # Quick test
  ./deploy_claudia_llm.sh monitor claudia-v3.2:3b 120 # Monitor for 2 minutes
  ./deploy_claudia_llm.sh full                # Full flow

Log location: logs/llm/deployment.log
EOF
}

# Full deployment flow
full_deployment() {
    local model_name="${1:-claudia-v3.2:3b}"

    log_info "Starting full deployment flow: $model_name"

    # 1. Check environment
    check_dependencies
    check_system_resources

    # 2. Deploy model
    deploy_model "$model_name"

    # 3. Quick test
    if quick_test "$model_name"; then
        log_success "Quick test passed, continuing with full test..."

        # 4. Full performance test
        run_performance_test "$model_name"

        # 5. Short-term monitoring
        monitor_model "$model_name" 30

        log_success "Full deployment flow complete!"
        log_info "Model $model_name is ready for use"
    else
        log_error "Quick test failed, please check model configuration"
        exit 1
    fi
}

# Main function
main() {
    local command="${1:-help}"
    shift || true

    case "$command" in
        deploy)
            check_dependencies
            deploy_model "$@"
            ;;
        test)
            run_performance_test "$@"
            ;;
        quick)
            quick_test "$@"
            ;;
        monitor)
            monitor_model "$@"
            ;;
        list)
            list_models
            ;;
        switch)
            switch_model "$@"
            ;;
        backup)
            backup_model "$@"
            ;;
        cleanup)
            cleanup_old_models
            ;;
        full)
            full_deployment "$@"
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            log_error "Unknown command: $command"
            show_help
            exit 1
            ;;
    esac
}

# Execute main function
main "$@"
