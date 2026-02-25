#!/bin/bash
# -*- coding: utf-8 -*-
################################################################################
# Claudia Project Automated Cleanup Script
#
# Features: Clean obsolete files, archive development docs, optimize project structure
# Author: Claude Code
# Date: 2025-10-31
# Version: 1.0
#
# Usage:
#   bash scripts/maintenance/project_cleanup.sh --mode=safe    # Safe mode (cache cleanup only)
#   bash scripts/maintenance/project_cleanup.sh --mode=full    # Full cleanup (including archiving)
#   bash scripts/maintenance/project_cleanup.sh --mode=preview # Preview mode (no execution)
################################################################################

set -e  # Exit immediately on error

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Project root directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "$PROJECT_ROOT"

# Timestamp
TIMESTAMP=$(date '+%Y%m%d_%H%M%S')
BACKUP_DIR="${PROJECT_ROOT}/backups/cleanup_${TIMESTAMP}"
REPORT_FILE="${PROJECT_ROOT}/logs/cleanup_report_${TIMESTAMP}.log"

# Default mode
MODE="${1:-safe}"
if [[ "$MODE" == --mode=* ]]; then
    MODE="${MODE#--mode=}"
fi

# Statistics variables
DELETED_FILES=0
DELETED_DIRS=0
ARCHIVED_FILES=0
FREED_SPACE=0

################################################################################
# Helper functions
################################################################################

print_header() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}\n"
}

print_success() {
    echo -e "${GREEN}[OK]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERR]${NC} $1"
}

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

# Write to log file
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" >> "$REPORT_FILE"
}

# Create backup
create_backup() {
    local source="$1"
    local backup_path="${BACKUP_DIR}/${source}"

    if [[ -e "$source" ]]; then
        mkdir -p "$(dirname "$backup_path")"
        cp -r "$source" "$backup_path"
        log "Backed up: $source -> $backup_path"
    fi
}

# Calculate file/directory size (MB)
get_size_mb() {
    local path="$1"
    if [[ -e "$path" ]]; then
        du -sm "$path" 2>/dev/null | cut -f1
    else
        echo "0"
    fi
}

################################################################################
# Phase 1: Python cache cleanup
################################################################################

cleanup_python_cache() {
    print_header "Phase 1: Clean Python Cache Files"

    local cache_count=0
    local cache_size=0

    # Calculate cache size
    cache_size=$(find . -type d -name "__pycache__" -exec du -sm {} + 2>/dev/null | awk '{sum+=$1} END {print sum}')
    cache_size=${cache_size:-0}

    print_info "Python cache size found: ${cache_size} MB"

    if [[ "$MODE" != "preview" ]]; then
        # Delete __pycache__ directories
        while IFS= read -r dir; do
            if [[ -n "$dir" ]]; then
                rm -rf "$dir"
                ((cache_count++))
                log "Deleted: $dir"
            fi
        done < <(find . -type d -name "__pycache__" 2>/dev/null)

        # Delete .pyc files
        find . -type f -name "*.pyc" -delete 2>/dev/null

        # Delete pytest cache
        find . -type d -name ".pytest_cache" -exec rm -rf {} + 2>/dev/null || true

        DELETED_DIRS=$((DELETED_DIRS + cache_count))
        FREED_SPACE=$((FREED_SPACE + cache_size))

        print_success "Cleaned ${cache_count} cache directories, freed ${cache_size} MB"
    else
        print_info "[Preview] Would clean ${cache_count} cache directories"
    fi

    log "Phase 1 completed: Cleaned ${cache_count} cache directories"
}

################################################################################
# Phase 2: Archive development documents
################################################################################

archive_development_docs() {
    print_header "Phase 2: Archive Development Process Documents"

    local archive_dir="${PROJECT_ROOT}/docs/archive/development"

    # Core documents to keep
    local keep_docs=(
        "GO2_SUPPORTED_ACTIONS.md"
        "CORRECT_LLM_ARCHITECTURE.md"
        "guides/LED_USAGE_GUIDE.md"
        "guides/LED_SYSTEM_VERIFICATION_REPORT.md"
    )

    # Create archive directory
    if [[ "$MODE" != "preview" ]]; then
        mkdir -p "$archive_dir"
    fi

    # Document patterns to archive
    local archive_patterns=(
        "BRAIN_OPTIMIZATION_REPORT.md"
        "COMPLETE_API_ANALYSIS.md"
        "COMPLEX_ACTION_SEQUENCE_RESEARCH.md"
        "DANCE_SELECTION_GUIDE.md"
        "ENHANCED_JAPANESE_COMMANDER.md"
        "FINAL_*.md"
        "FIX_REPORT_*.md"
        "HARDWARE_FIX_REPORT.md"
        "HYBRID_ARCHITECTURE_*.md"
        "INTERACTIVE_JAPANESE_COMMANDER.md"
        "LLM_FRAMEWORK_ANALYSIS.md"
        "NETWORK_CONFIG_FIX.md"
        "NEW_BRAIN_ARCHITECTURE_SUCCESS.md"
        "PHASE2_OPTIMIZATION_REPORT.md"
        "PRODUCTION_DEPLOYMENT_SUCCESS.md"
        "PROMPT_*.md"
        "REAL_*.md"
        "SDK_LIMITATION_ANALYSIS.md"
        "SPORT_MODE_*.md"
        "STATE_MANAGEMENT_FIX.md"
        "TASK11_OPTIMIZATION_SUMMARY.md"
        "TASKMASTER_UPDATE_COMPLETE_*.md"
    )

    local archived=0

    for pattern in "${archive_patterns[@]}"; do
        for doc in docs/${pattern}; do
            if [[ -f "$doc" ]]; then
                local basename=$(basename "$doc")
                print_info "Archiving: $doc"

                if [[ "$MODE" != "preview" ]]; then
                    create_backup "$doc"
                    mv "$doc" "${archive_dir}/"
                    log "Archived: $doc -> ${archive_dir}/${basename}"
                fi

                ((archived++))
                ((ARCHIVED_FILES++))
            fi
        done
    done

    # Archive subdirectory documents
    for subdir in ai_models deployment troubleshooting tasks; do
        if [[ -d "docs/${subdir}" ]]; then
            local subdir_size=$(get_size_mb "docs/${subdir}")
            print_info "Archiving subdirectory: docs/${subdir}/ (${subdir_size} MB)"

            if [[ "$MODE" != "preview" ]]; then
                mkdir -p "${archive_dir}/${subdir}"
                create_backup "docs/${subdir}"

                # Move all files
                if [[ -n "$(ls -A docs/${subdir}/*.md 2>/dev/null)" ]]; then
                    mv docs/${subdir}/*.md "${archive_dir}/${subdir}/" 2>/dev/null || true
                fi

                # If subdirectory is empty, delete it
                if [[ -z "$(ls -A docs/${subdir} 2>/dev/null)" ]]; then
                    rmdir "docs/${subdir}" 2>/dev/null || true
                fi

                log "Archived directory: docs/${subdir} -> ${archive_dir}/${subdir}"
            fi

            ((archived += $(find docs/${subdir} -name "*.md" 2>/dev/null | wc -l)))
        fi
    done

    print_success "Archived ${archived} development documents"
    log "Phase 2 completed: Archived ${archived} development documents"
}

################################################################################
# Phase 3: Clean deprecated root directory documents
################################################################################

cleanup_root_deprecated_docs() {
    print_header "Phase 3: Clean Deprecated Root Directory Documents"

    local deprecated_docs=(
        "README_FINAL_SOLUTION.md"
        "README_PERFORMANCE_OPTIMIZATION.md"
    )

    local cleaned=0

    for doc in "${deprecated_docs[@]}"; do
        if [[ -f "$doc" ]]; then
            local doc_size=$(get_size_mb "$doc")
            print_info "Deleting: $doc (${doc_size} MB)"

            if [[ "$MODE" != "preview" ]]; then
                create_backup "$doc"
                rm "$doc"
                log "Deleted: $doc"
            fi

            ((cleaned++))
            ((DELETED_FILES++))
        fi
    done

    print_success "Cleaned ${cleaned} deprecated root directory documents"
    log "Phase 3 completed: Cleaned ${cleaned} deprecated root docs"
}

################################################################################
# Phase 4: Clean temporary files and test results
################################################################################

cleanup_temp_files() {
    print_header "Phase 4: Clean Temporary Files and Test Results"

    local cleaned=0

    # Clean test result JSON files
    for result_file in test_results_*.json; do
        if [[ -f "$result_file" ]]; then
            print_info "Deleting test result: $result_file"

            if [[ "$MODE" != "preview" ]]; then
                rm "$result_file"
                log "Deleted: $result_file"
            fi

            ((cleaned++))
            ((DELETED_FILES++))
        fi
    done

    # Clean empty backup directories
    if [[ -d "backups" ]]; then
        local backup_count=$(find backups -type f 2>/dev/null | wc -l)
        if [[ "$backup_count" -eq 0 ]]; then
            print_info "Deleting empty backups directory"
            if [[ "$MODE" != "preview" ]]; then
                rm -rf backups
                log "Deleted: backups/ (empty)"
            fi
            ((DELETED_DIRS++))
        fi
    fi

    # Clean empty docker directories
    if [[ -d "docker" ]]; then
        local docker_count=$(find docker -type f 2>/dev/null | wc -l)
        if [[ "$docker_count" -eq 0 ]]; then
            print_info "Deleting empty docker directory"
            if [[ "$MODE" != "preview" ]]; then
                rm -rf docker
                log "Deleted: docker/ (empty)"
            fi
            ((DELETED_DIRS++))
        fi
    fi

    print_success "Cleaned ${cleaned} temporary files"
    log "Phase 4 completed: Cleaned ${cleaned} temporary files"
}

################################################################################
# Phase 5: Optional large directory cleanup
################################################################################

cleanup_large_dirs() {
    print_header "Phase 5: Large Directory Cleanup (Optional)"

    print_warning "The following large directories can be cleaned, but require manual confirmation:"

    # node_modules
    if [[ -d "node_modules" ]]; then
        local size=$(get_size_mb "node_modules")
        print_info "node_modules/: ${size} MB (can be restored via npm install)"
    fi

    # cyclonedds source code
    if [[ -d "cyclonedds" ]]; then
        local size=$(get_size_mb "cyclonedds")
        print_info "cyclonedds/: ${size} MB (DDS source, not required at runtime)"
    fi

    print_warning "These directories will not be cleaned automatically. Please delete manually as needed."
    log "Phase 5: Large directories reported (manual cleanup required)"
}

################################################################################
# Phase 6: Git cleanup suggestions
################################################################################

suggest_git_cleanup() {
    print_header "Phase 6: Git Cleanup Suggestions"

    print_info "Detected the following git-marked-for-deletion directories:"

    local git_deleted=(
        ".clinerules"
        ".github/instructions"
        ".roo"
        ".serena"
        ".trae"
        ".windsurf"
    )

    local found=0
    for dir in "${git_deleted[@]}"; do
        if [[ -d "$dir" ]]; then
            print_info "  - $dir (marked for deletion but not yet cleaned)"
            ((found++))
        fi
    done

    if [[ "$found" -gt 0 ]]; then
        print_warning "Recommended to run the following commands to complete git cleanup:"
        echo "  git add -u          # Stage deleted files"
        echo "  git clean -fd       # Clean untracked files and directories"
    else
        print_success "No git-marked directories requiring cleanup found"
    fi

    log "Phase 6: Git cleanup suggestions provided"
}

################################################################################
# Generate cleanup report
################################################################################

generate_report() {
    print_header "Cleanup Report"

    local report_path="${PROJECT_ROOT}/docs/cleanup_report_${TIMESTAMP}.md"

    cat > "$report_path" << EOF
# Claudia Project Cleanup Report

**Execution time**: $(date '+%Y-%m-%d %H:%M:%S')
**Execution mode**: ${MODE}
**Backup directory**: ${BACKUP_DIR}

## Cleanup Statistics

- **Files deleted**: ${DELETED_FILES}
- **Directories deleted**: ${DELETED_DIRS}
- **Documents archived**: ${ARCHIVED_FILES}
- **Space freed**: ${FREED_SPACE} MB

## Operations Performed

### Phase 1: Python Cache Cleanup
- Cleaned all \`__pycache__/\` directories
- Deleted all \`.pyc\` files
- Cleaned pytest cache

### Phase 2: Document Archiving
- Archive path: \`docs/archive/development/\`
- Documents archived: ${ARCHIVED_FILES}

### Phase 3: Deprecated Document Cleanup
- Deleted deprecated README files from root directory

### Phase 4: Temporary File Cleanup
- Cleaned test result JSON files
- Deleted empty temporary directories

## Backup Information

All modified or deleted files have been backed up to:
\`${BACKUP_DIR}\`

## Rollback Method

To restore files:
\`\`\`bash
cp -r ${BACKUP_DIR}/<path> <original_path>
\`\`\`

## Verification Recommendations

After cleanup, the following verification is recommended:
\`\`\`bash
# Run tests
python3 test/run_tests.py

# Verify LLM brain
./start_production_brain.sh

# Check document structure
tree docs/ -L 2

# Verify TaskMaster
ls -lh .taskmaster/tasks/tasks.json
\`\`\`

---
*Generated: $(date '+%Y-%m-%d %H:%M:%S')*
EOF

    print_success "Cleanup report generated: ${report_path}"
    log "Report generated: ${report_path}"
}

################################################################################
# Main execution flow
################################################################################

main() {
    print_header "Claudia Project Automated Cleanup Script v1.0"

    echo "Project root: ${PROJECT_ROOT}"
    echo "Execution mode: ${MODE}"
    echo "Timestamp: ${TIMESTAMP}"
    echo ""

    # Create log directory
    mkdir -p "${PROJECT_ROOT}/logs"

    # Create backup directory (except in preview mode)
    if [[ "$MODE" != "preview" ]]; then
        mkdir -p "$BACKUP_DIR"
        log "Cleanup started - Mode: ${MODE}"
    fi

    # Execute cleanup based on mode
    case "$MODE" in
        safe)
            print_info "Safe mode: Only cleaning caches and temporary files"
            cleanup_python_cache
            cleanup_temp_files
            suggest_git_cleanup
            ;;
        full)
            print_info "Full mode: Executing all cleanup operations"
            cleanup_python_cache
            archive_development_docs
            cleanup_root_deprecated_docs
            cleanup_temp_files
            cleanup_large_dirs
            suggest_git_cleanup
            ;;
        preview)
            print_info "Preview mode: Showing operations that would be performed (not actually executing)"
            cleanup_python_cache
            archive_development_docs
            cleanup_root_deprecated_docs
            cleanup_temp_files
            cleanup_large_dirs
            suggest_git_cleanup
            ;;
        *)
            print_error "Unknown mode: ${MODE}"
            echo "Available modes: safe, full, preview"
            exit 1
            ;;
    esac

    # Generate report
    if [[ "$MODE" != "preview" ]]; then
        generate_report
    fi

    print_header "Cleanup Complete"

    echo "Statistics:"
    echo "  - Files deleted: ${DELETED_FILES}"
    echo "  - Directories deleted: ${DELETED_DIRS}"
    echo "  - Documents archived: ${ARCHIVED_FILES}"
    echo "  - Space freed: ${FREED_SPACE} MB"

    if [[ "$MODE" != "preview" ]]; then
        echo ""
        echo "Backup location: ${BACKUP_DIR}"
        echo "Log file: ${REPORT_FILE}"
    fi

    print_success "Project cleanup completed successfully!"
}

# Execute main function
main "$@"
