# Unitree Go2 Front Camera Validation System

## Overview

This validation system is designed specifically for the Unitree Go2 robot front camera, providing comprehensive performance testing, image quality analysis, and environmental adaptability validation.

## Validation Content

### Core Validation Items
- **Resolution Verification**: Confirm 1280x720 HD specification
- **Performance Testing**: Frame rate, latency, stability measurement
- **Image Quality**: Color accuracy, sharpness, noise analysis
- **Environmental Adaptation**: Performance under different lighting conditions
- **Stress Testing**: System stability under high load

### Technical Metrics
- **Resolution**: Target 1280x720, fallback 480x1280
- **Frame Rate**: Target 30fps, minimum 20fps
- **Latency**: <100ms (real-time control requirement)
- **Image Quality**: SSIM>0.8, color Delta E<5
- **Success Rate**: >95% capture success rate

## System Architecture

```
front_camera_validation/
├── validation_config.json      # Configuration file
├── camera_config.py            # Camera configuration management
├── performance_tester.py       # Performance testing module
├── image_quality_analyzer.py   # Image quality analysis
├── main_validation_script.py   # Main validation script
└── README_front_camera_validation.md  # This document
```

## Quick Start

### Environment Requirements
```bash
# Python dependencies
pip install opencv-python numpy scikit-image matplotlib

# System requirements
- Python 3.7+
- OpenCV 4.0+
- Camera access permissions
```

### Basic Usage

1. **Quick Validation** (recommended):
```bash
cd scripts/validation/camera/front_camera_validation
python3 main_validation_script.py
```

2. **Custom Configuration**:
```bash
python3 main_validation_script.py --config custom_config.json --output /path/to/output
```

3. **Verbose Output**:
```bash
python3 main_validation_script.py --verbose
```

### Individual Module Testing

1. **Camera Configuration Test**:
```bash
python3 camera_config.py
```

2. **Performance Test**:
```bash
python3 performance_tester.py
```

3. **Image Quality Analysis**:
```bash
python3 image_quality_analyzer.py
```

## Validation Workflow

### Standard Validation Sequence
1. **Camera Initialization** - Connection and configuration verification
2. **Resolution Verification** - HD specification confirmation
3. **Basic Performance Test** - 30-second performance benchmark
4. **Image Quality Analysis** - 20-sample quality assessment
5. **Stress Test** - 60-second high-load test
6. **Report Generation** - HTML and JSON result reports

### Test Duration
- **Quick Validation**: ~5 minutes
- **Full Validation**: ~10 minutes
- **Extended Validation**: ~15 minutes (includes environmental tests)

## Configuration Guide

### Main Configuration Items

```json
{
  "camera_config": {
    "target_resolution": [1280, 720],
    "fallback_resolution": [480, 1280],
    "target_fps": 30,
    "camera_id": 0
  },
  "performance_thresholds": {
    "max_latency_ms": 100,
    "min_fps": 20,
    "min_capture_success_rate": 0.95
  },
  "image_quality_thresholds": {
    "min_ssim": 0.8,
    "max_color_delta_e": 5.0,
    "min_sharpness_score": 0.7
  }
}
```

### Custom Validation Sequence

```json
{
  "validation_sequence": [
    "camera_initialization",
    "resolution_verification",
    "basic_performance_test",
    "image_quality_analysis",
    "stress_test",
    "report_generation"
  ]
}
```

## Results Interpretation

### Performance Metrics

| Metric | Excellent | Good | Acceptable | Needs Improvement |
|--------|-----------|------|------------|-------------------|
| FPS | >=30 | 25-29 | 20-24 | <20 |
| Latency (ms) | <50 | 50-80 | 80-100 | >100 |
| Success Rate (%) | >=99 | 97-98 | 95-96 | <95 |

### Image Quality Metrics

| Metric | Excellent | Good | Acceptable | Needs Improvement |
|--------|-----------|------|------------|-------------------|
| Sharpness | >=90 | 80-89 | 70-79 | <70 |
| Color Accuracy | Delta E<2 | 2-3 | 3-5 | >5 |
| Noise Level | <0.05 | 0.05-0.08 | 0.08-0.1 | >0.1 |

### Overall Rating

- **EXCELLENT**: >=90 points, all key metrics excellent
- **GOOD**: 80-89 points, main metrics good
- **ACCEPTABLE**: 70-79 points, basically meets requirements
- **POOR**: 60-69 points, significant issues present
- **UNACCEPTABLE**: <60 points, major improvements needed

## Output Results

### File Structure
```
logs/camera_validation/validation_YYYYMMDD_HHMMSS/
├── validation_results.json          # Complete validation results
├── validation_report.html           # HTML visual report
├── validation_report.json          # JSON format report
├── basic_performance_metrics.json  # Basic performance data
├── image_quality_metrics.json      # Image quality data
└── stress_test_metrics.json        # Stress test data
```

### HTML Report Features
- Intuitive status display (pass/fail/warning)
- Detailed performance charts
- Image quality sample showcase
- Optimization suggestions and problem diagnosis

### JSON Report Uses
- Automated analysis and monitoring
- Historical data comparison
- CI/CD pipeline integration
- Trend report generation

## Troubleshooting

### Common Issues

1. **Camera Cannot Initialize**
```bash
# Check camera devices
ls /dev/video*
# Check permissions
sudo usermod -a -G video $USER
# Log out and log back in, then retry
```

2. **Resolution Mismatch**
```bash
# Check camera supported resolutions
v4l2-ctl --list-formats-ext
# Modify target_resolution in configuration file
```

3. **Performance Issues**
```bash
# Check system load
htop
# Close unnecessary applications
# Adjust buffer_size in camera_config
```

4. **Permission Issues**
```bash
# Add camera permissions
sudo usermod -a -G video $USER
# Log out and log back in for changes to take effect
```

### Log Analysis

1. **View Detailed Logs**:
```bash
tail -f front_camera_validation.log
```

2. **Debug Mode**:
```bash
python3 main_validation_script.py --verbose
```

3. **Check Configuration**:
```bash
python3 -c "import json; print(json.load(open('validation_config.json')))"
```

## Advanced Usage

### Custom Tests

1. **Performance Test Only**:
```python
from performance_tester import PerformanceTester
from camera_config import CameraConfig

with CameraConfig() as camera:
    if camera.initialize_camera():
        tester = PerformanceTester(camera)
        metrics = tester.run_basic_performance_test(60.0)
        print(f"FPS: {metrics.fps_actual:.2f}")
```

2. **Quality Analysis Only**:
```python
from image_quality_analyzer import ImageQualityAnalyzer
from camera_config import CameraConfig

with CameraConfig() as camera:
    if camera.initialize_camera():
        analyzer = ImageQualityAnalyzer(camera)
        metrics = analyzer.analyze_image_quality(50)
        print(f"Quality score: {metrics.overall_quality_score:.2f}")
```

### Batch Validation

```bash
#!/bin/bash
# Multiple validations to obtain statistical data
for i in {1..5}; do
    echo "Validation round $i"
    python3 main_validation_script.py --output logs/batch_$i
    sleep 30
done
```

### CI/CD Integration

```bash
#!/bin/bash
# Automated validation script
python3 main_validation_script.py --config ci_config.json
exit_code=$?

if [ $exit_code -eq 0 ]; then
    echo "Validation passed"
else
    echo "Validation failed"
    exit 1
fi
```

## Technical Support

### Contact
- Technical Documentation: [Internal technical documentation]
- Issue Reporting: [Internal issue tracking system]
- Technical Support: [Internal technical support]

### Version Information
- Current Version: 1.0.0
- Update Date: 2024-12-26
- Compatibility: Unitree Go2, Python 3.7+

### License Information
Internal use, following company technical development standards.
