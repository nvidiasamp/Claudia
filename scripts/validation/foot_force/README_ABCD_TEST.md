# Unitree Go2 Foot Force Sensor ABCD Validation Workflow

## Overview

This document explains how to run the complete ABCD validation workflow for the Unitree Go2 foot force sensors.

## Test Phases

### Phase A: Data Reading Framework Validation
- **Objective**: Validate foot force data reading capability
- **Components**: `FootForceConfig`, `DataCollector`
- **Test Content**: Sensor connection, data sampling rate, basic data format

### Phase B: Static Force Distribution Validation
- **Objective**: Validate force distribution accuracy under static conditions
- **Components**: `StaticFootForceTester`
- **Test Content**: Zero-load test, static standing test, weight distribution analysis

### Phase C: Dynamic Response Testing
- **Objective**: Validate sensor response under dynamic conditions
- **Components**: `DynamicFootForceTester`
- **Test Content**: Slow walking, normal walking, impact testing

### Phase D: Comprehensive Visualization and Documentation
- **Objective**: Generate comprehensive reports and visualizations
- **Components**: `ComprehensiveFootForceDashboard`
- **Output**: JSON report, HTML report, visualization charts

## How to Run

### 1. Quick Component Test

First run the quick test to verify all components are working correctly:

```bash
cd ~/claudia
python3 scripts/validation/foot_force/run_quick_abcd_test.py
```

**Features**:
- Tests all ABCD component imports
- Runs mock data tests
- Generates test reports
- Does not require connection to real robot

### 2. Full ABCD Validation Workflow

After confirming component tests pass, run the full validation:

```bash
cd ~/claudia
python3 scripts/validation/foot_force/run_complete_validation.py
```

**Features**:
- Connects to real Unitree Go2 robot
- Sequentially executes all four ABCD phases
- Generates complete validation report
- Requires robot network connection (192.168.123.161)

## Environment Requirements

### System Environment
```bash
# ROS2 Foxy + CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/foxy/setup.bash
source cyclonedx_ws/install/setup.bash
```

### Python Dependencies
- numpy
- matplotlib
- pandas (optional)
- unitree_sdk2py

### Network Connection
- Robot IP: 192.168.123.161
- Network interface: eth0

## Output Files

All test results are saved in: `scripts/validation/foot_force/foot_force_validation/output/`

### Directory Structure
```
output/
├── logs/                           # Test logs
│   └── complete_validation_*.log
├── mock_test_report_*.json         # Quick test report
├── final_validation_report_*.json  # Full validation report
├── dynamic_test_results_*.json     # Dynamic test results
└── comprehensive_report_*.html     # Comprehensive HTML report
```

## Troubleshooting

### 1. Component Import Failure
```bash
# Check Python path
export PYTHONPATH="~/claudia:$PYTHONPATH"
```

### 2. Robot Connection Failure
```bash
# Check network connection
ping 192.168.123.161

# Check CycloneDDS environment
ros2 topic list
```

### 3. Data Collection Failure
- Confirm the robot is powered on and connected
- Verify DDS communication is working
- Check sensor status

## Test Workflow

### Full Validation Workflow
1. **Pre-check**: Network connection, environment variables
2. **Phase A**: Data reading framework validation (5 seconds)
3. **Phase B**: Static validation test (10 seconds)
4. **Phase C**: Dynamic test suite (135 seconds)
   - Slow walking (60 seconds)
   - Normal walking (45 seconds)
   - Impact test (30 seconds)
5. **Phase D**: Report generation and visualization
6. **Post-processing**: Resource cleanup, final report generation

### Expected Run Time
- Quick test: < 30 seconds
- Full validation: 5-10 minutes

## Scoring Criteria

### Phase Scoring
- **Phase A**: Data sampling rate, connection stability
- **Phase B**: Zero-point accuracy, force distribution balance (static 60% weight)
- **Phase C**: Gait consistency, dynamic response (dynamic 40% weight)
- **Phase D**: Report generation success rate

### Overall Scoring
- **Grade A**: >=90 points - Excellent
- **Grade B**: 80-89 points - Good
- **Grade C**: 70-79 points - Acceptable
- **Grade D**: 60-69 points - Needs Improvement
- **Grade F**: <60 points - Failing

## Important Reminders

**Safety Notes**:
- Ensure sufficient space around the robot during dynamic tests
- Follow robot operation safety guidelines
- Maintain safe distance from personnel during testing

**Data Processing**:
- All data is saved automatically, no manual backup needed
- Reports include timestamps for historical comparison
- Supports multiple runs, results are saved cumulatively

**Maintenance Recommendations**:
- Regularly run quick tests to verify system status
- Run full validation before important tests
- Save historical test data for trend analysis
