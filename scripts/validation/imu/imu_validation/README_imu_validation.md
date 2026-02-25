# Unitree Go2 IMU Validation System

## Overview

This system provides comprehensive validation and testing functionality for the IMU (Inertial Measurement Unit) of the Unitree Go2 robot. Through script-driven data collection, real-time visualization, and thorough documentation, it ensures the accuracy and reliability of the IMU sensor system.

## Features

### Core Functionality
- **Automated IMU initialization and configuration**
- **Real-time data collection and processing**
- **Multi-dimensional sensor validation**
- **Interactive visualization interface**
- **Comprehensive calibration analysis**
- **Detailed validation reports**

### Test Modules
1. **Static Stability Test** - Validates sensor stability in stationary state
2. **Dynamic Response Test** - Tests response performance under motion
3. **Calibration Verification Analysis** - Analyzes sensor calibration quality
4. **Real-time Visualization** - 3D orientation and time-series plot display
5. **Comprehensive Assessment Report** - Generates detailed validation documentation

## System Requirements

### Hardware Requirements
- Unitree Go2 robot (with IMU sensor)
- Development host (Ubuntu 20.04+)
- Stable network connection

### Software Dependencies
```bash
# Python package dependencies
numpy>=1.20.0
matplotlib>=3.5.0
scipy>=1.7.0
unitree_sdk2py>=1.0.0

# System dependencies
sudo apt update
sudo apt install python3-dev python3-pip
pip3 install numpy matplotlib scipy
```

## Quick Start

### 1. Environment Setup
```bash
# Navigate to the IMU validation directory
cd scripts/validation/imu/imu_validation

# Check Python environment
python3 --version  # Requires Python 3.8+

# Install dependencies
pip3 install -r requirements.txt  # If requirements file exists
```

### 2. Configuration File Setup
```bash
# Check configuration file
cat validation_config.json

# Modify configuration as needed
vim validation_config.json
```

### 3. Run Full Validation
```bash
# Execute the full IMU validation process
python3 main_validation_script.py

# Or specify a configuration file
python3 main_validation_script.py --config custom_config.json
```

### 4. View Results
```bash
# After validation completes, results are saved in the output directory
ls output/imu_validation/

# View the latest test report
cat output/imu_validation/*/imu_validation_report.txt
```

## Detailed Usage

### Command Line Options
```bash
python3 main_validation_script.py [options]

Options:
  --config, -c     Configuration file path
  --test-type, -t  Test type (full|static|dynamic|calibration|visualization)
  --output, -o     Output directory
  --verbose, -v    Verbose output
  --help, -h       Show help information
```

### Test Type Descriptions

#### Full Validation (--test-type full)
Runs all validation modules, including:
- IMU initialization check
- Static stability test (60 seconds)
- Dynamic response test (pitch/roll/yaw tests)
- Calibration verification analysis (multi-pose data collection)
- Real-time visualization function verification

#### Static Test (--test-type static)
Focuses on sensor stability under static conditions:
- Accelerometer stability analysis
- Gyroscope bias measurement
- Gravity accuracy verification
- Noise level assessment

#### Dynamic Test (--test-type dynamic)
Tests response performance under dynamic conditions:
- Response time measurement
- Tracking accuracy analysis
- Dynamic range assessment
- Frequency response characteristics

#### Calibration Analysis (--test-type calibration)
In-depth calibration quality analysis:
- Scale factor analysis
- Cross-axis coupling measurement
- Temperature compensation verification
- Attitude fusion quality assessment

#### Visualization Verification (--test-type visualization)
Verifies visualization functionality:
- Real-time 3D orientation display
- Multi-sensor time-series plots
- Data export functionality

## Configuration Guide

### Main Configuration Parameters

```json
{
  "imu_config": {
    "sampling_rate_hz": 100,        // IMU sampling frequency
    "test_duration_seconds": 30,    // Default test duration
    "timeout_seconds": 10,          // Connection timeout
    "network_interface": "eth0"     // Network interface
  },
  "test_parameters": {
    "static_test": {
      "duration_seconds": 60,       // Static test duration
      "stability_threshold": {
        "accelerometer_std_max": 0.05,  // Accelerometer stability threshold
        "gyroscope_std_max": 0.1,       // Gyroscope stability threshold
        "quaternion_drift_max": 0.01    // Quaternion drift threshold
      }
    },
    "dynamic_test": {
      "duration_seconds": 120,      // Dynamic test duration
      "response_tests": [           // Dynamic test items
        "pitch_test",
        "roll_test",
        "yaw_test",
        "translation_test"
      ],
      "response_threshold_ms": 50   // Response time threshold (milliseconds)
    }
  },
  "visualization": {
    "real_time_plots": true,        // Enable real-time plotting
    "plot_update_interval_ms": 100, // Plot update interval
    "max_plot_points": 500,         // Maximum plot points
    "enable_3d_orientation": true   // Enable 3D orientation display
  },
  "quality_thresholds": {
    "accuracy": {
      "gravity_error_max_percent": 2.0  // Maximum gravity measurement error percentage
    },
    "noise_levels": {
      "accelerometer_noise_max": 0.02,  // Accelerometer maximum noise
      "gyroscope_noise_max": 0.01       // Gyroscope maximum noise
    }
  }
}
```

## Test Process Details

### Phase 1: System Initialization
1. **IMU Connection Verification** - Check connection with Unitree Go2
2. **Sensor Specification Confirmation** - Verify sampling rate, range, and other parameters
3. **Data Stream Test** - Confirm IMU data can be received normally
4. **Basic Function Verification** - Test data parsing and processing

### Phase 2: Static Stability Test
1. **Environment Preparation**
   ```
   Ensure the robot is in a stationary state
   Avoid vibration and external interference
   Test duration: 60 seconds
   ```

2. **Data Collection**
   - Continuous IMU data collection
   - Real-time data quality monitoring
   - Automatic outlier detection

3. **Stability Analysis**
   - Accelerometer standard deviation per axis
   - Gyroscope bias and noise
   - Quaternion consistency check
   - Gravity accuracy verification

### Phase 3: Dynamic Response Test
1. **Pitch Test**
   ```
   Instructions: Please slowly tilt the robot forward and backward (pitch axis rotation),
   then quickly return to horizontal position
   Test metrics: Response time, tracking accuracy, overshoot
   ```

2. **Roll Test**
   ```
   Instructions: Please slowly tilt the robot left and right (roll axis rotation),
   then quickly return to horizontal position
   Test metrics: Response time, dynamic range, linearity
   ```

3. **Yaw Test**
   ```
   Instructions: Please slowly rotate the robot left and right (yaw axis rotation),
   then quickly return to the original direction
   Test metrics: Angular velocity response, integration drift, frequency response
   ```

4. **Translation Test**
   ```
   Instructions: Please steadily move the robot forward/backward and left/right,
   avoiding rotation
   Test metrics: Acceleration response, noise suppression, signal quality
   ```

### Phase 4: Calibration Verification Analysis
1. **Multi-Pose Data Collection**
   ```
   Position 1: Horizontal rest (normal pose)
   Position 2: Left side tilted 90 degrees
   Position 3: Right side tilted 90 degrees
   Position 4: Forward tilt 90 degrees
   Position 5: Backward tilt 90 degrees
   Position 6: Inverted 180 degrees
   ```

2. **Calibration Quality Analysis**
   - Gravity vector consistency
   - Scale factor error
   - Cross-axis coupling
   - Temperature coefficient (if applicable)

### Phase 5: Visualization Verification
1. **Real-time Plotting Test**
   - Launch multi-window visualization
   - Verify data updates
   - Test 3D orientation display
   - Check plotting performance

2. **Data Export Test**
   - PNG format image saving
   - Data file export
   - Visualization statistics generation

## Results Interpretation

### Test Status Codes
- **PASS** - All metrics meet requirements
- **WARNING** - Minor issues, acceptable for use
- **FAIL** - Serious issues, repair needed
- **ERROR** - Test execution error
- **UNKNOWN** - Status unknown

### Key Metric Interpretation

#### Static Test Metrics
- **Accelerometer Stability** - Standard deviation < 0.05 m/s^2
- **Gyroscope Bias** - Bias magnitude < 0.1 rad/s
- **Gravity Accuracy** - Error < 2%
- **Quaternion Consistency** - Magnitude error < 0.01

#### Dynamic Test Metrics
- **Response Time** - < 50ms
- **Tracking Accuracy** - Correlation > 0.8
- **Dynamic Range** - > 10 degrees of angle change
- **Overshoot** - < 10%

#### Calibration Quality Metrics
- **Gravity Calibration** - Error < 2%
- **Bias Stability** - Change < 0.05 rad/s
- **Cross-Axis Coupling** - < 10%
- **Overall Quality Score** - > 0.7

## Troubleshooting

### Common Issues

#### 1. IMU Initialization Failure
```
Symptom: "IMU not initialized, cannot perform test"
Solution:
- Check network connection
- Confirm robot power status
- Verify SDK version compatibility
- Restart network interface
```

#### 2. Data Collection Timeout
```
Symptom: "Data collection timeout, forcing stop"
Solution:
- Check network latency
- Increase timeout_seconds configuration
- Confirm robot is responding normally
- Reduce collection duration
```

#### 3. Visualization Display Failure
```
Symptom: "Visualization startup failed"
Solution:
- Install matplotlib dependencies
- Configure X11 forwarding (if using SSH)
- Check display environment variables
- Disable real_time_plots configuration
```

#### 4. Insufficient Test Accuracy
```
Symptom: Multiple test items showing FAIL
Solution:
- Ensure test environment is stable
- Check for mechanical vibration interference
- Recalibrate the IMU sensor
- Adjust quality threshold configuration
```

### Log Analysis
```bash
# View detailed logs
tail -f logs/imu_validation/imu_validation_*.log

# Search for error messages
grep "ERROR\|FAIL" logs/imu_validation/imu_validation_*.log

# Analyze network connection issues
grep "timeout\|connection" logs/imu_validation/imu_validation_*.log
```

## Output File Description

### Results Directory Structure
```
output/imu_validation/YYYYMMDD_HHMMSS/
├── imu_validation_results.json      # Complete JSON results
├── imu_validation_report.txt        # Simplified text report
├── imu_timeseries_YYYYMMDD_HHMMSS.png    # Time-series chart
├── imu_3d_orientation_YYYYMMDD_HHMMSS.png  # 3D orientation chart
└── data_exports/                     # Raw data exports
    ├── static_test_data.json
    ├── dynamic_test_data.json
    └── calibration_data.json
```

### JSON Results Format
```json
{
  "test_info": {
    "start_time": "ISO timestamp",
    "test_version": "1.0.0",
    "robot_model": "Unitree Go2",
    "test_operator": "username"
  },
  "initialization": {
    "status": "SUCCESS|FAILED",
    "imu_specs": { ... }
  },
  "static_test": {
    "test_status": "PASS|WARNING|FAIL",
    "accelerometer_stability": { ... },
    "gyroscope_stability": { ... },
    "recommendations": [ ... ]
  },
  "dynamic_test": {
    "pitch_test": { ... },
    "roll_test": { ... },
    "overall": { ... }
  },
  "calibration_analysis": { ... },
  "visualization_test": { ... },
  "overall_assessment": {
    "status": "PASS|WARNING|FAIL",
    "pass_rate_percent": 85.5,
    "overall_conclusion": "Conclusion text"
  }
}
```

## Best Practices

### Test Environment
1. **Stable Physical Environment**
   - Avoid vibration and shock
   - Control temperature changes
   - Minimize electromagnetic interference

2. **Network Environment**
   - Use wired connection
   - Ensure low latency
   - Avoid network congestion

### Test Operations
1. **Preparation Phase**
   - Warm up the robot system
   - Check battery level
   - Confirm sensor status

2. **Execution Phase**
   - Follow the prompts to perform operations
   - Keep movements smooth
   - Avoid sudden movements

3. **Results Analysis**
   - Read recommendations carefully
   - Focus on key metrics
   - Save test records

## Technical Support

### Contact Information
- Project Repository: [GitHub link]
- Technical Documentation: [Documentation link]
- Issue Reporting: [Issue link]

### Version History
- v1.0.0 (2025-06-27) - Initial release
  - Complete IMU validation functionality
  - Real-time visualization support
  - Comprehensive calibration analysis

---

**Note**: This validation system is designed for the Unitree Go2 robot. Please confirm compatibility before using with other robot models.
