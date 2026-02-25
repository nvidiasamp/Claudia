# Claudia Robot LED Control System Usage Guide

> Generated: 2025-07-04
> Cleanup version: LED control system architecture after restructuring

## System Overview

The Claudia robot's LED control system uses a **dual controller architecture**, each serving its purpose:

- **ClaudiaLEDController**: Focused on flash patterns and status indication
- **UnifiedLEDController**: Focused on ambient light detection and adaptive brightness

## Architecture Components

### Core Controllers

#### 1. ClaudiaLEDController (`src/claudia/robot_controller/led_controller.py`)
- **Purpose**: LED flash indication of robot status
- **Features**:
  - Supports 6 flash modes (NORMAL, WAITING, WARNING, ERROR, SPECIAL, OFF)
  - VUI brightness control and LowCmd RGB control
  - Single flash, double flash, fast flash, breathing light effects
- **Main API**:
  ```python
  controller = ClaudiaLEDController()
  controller.start_flash_mode(LEDControlMode.WAITING)  # Start flash
  controller.stop_flash_mode()  # Stop flash
  ```

#### 2. UnifiedLEDController (`src/claudia/robot_controller/unified_led_controller.py`)
- **Purpose**: Environment-adaptive intelligent LED control
- **Features**:
  - Ambient light detection and analysis (AdvancedEnvironmentalAnalyzer)
  - Adaptive brightness adjustment
  - System status monitoring
  - Advanced environmental adaptation algorithms
- **Control methods**: VUI_CLIENT and LOW_CMD two modes

### Support Modules

#### 3. LED Mode Definitions (`src/claudia/robot_controller/led_patterns.py`)
- **ClaudiaLEDMode**: Enumeration defining all LED modes
- **LEDPattern**: LED mode parameter data structure
- **ClaudiaLEDModeDefinitions**: Complete mode parameter configuration

Supported LED modes:
```python
- WAKE_CONFIRM: Wake confirmation
- PROCESSING_VOICE: Voice processing
- EXECUTING_ACTION: Action execution
- ACTION_COMPLETE: Action complete
- ERROR_STATE: Error state
- SYSTEM_BOOT: System boot
- SYSTEM_CALIBRATION: System calibration
- LOW_BATTERY: Low battery warning
- SEARCH_LIGHT: Search lighting
```

#### 4. LED State Machine (`src/claudia/robot_controller/led_state_machine.py`)
- LED state transition logic
- State management for complex LED behaviors

## Usage Methods

### Basic Flash Control Example

```python
#!/usr/bin/env python3
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent))

from src.claudia.robot_controller.led_controller import ClaudiaLEDController, LEDControlMode

def basic_flash_demo():
    """Basic flash demonstration"""
    controller = ClaudiaLEDController()

    try:
        # Initialize connection
        if not controller.connect():
            print("LED controller connection failed")
            return

        # Waiting mode flash (single flash 1Hz)
        print("Starting waiting mode flash...")
        controller.start_flash_mode(LEDControlMode.WAITING)
        time.sleep(5)

        # Warning mode flash (fast flash 3Hz)
        print("Switching to warning mode flash...")
        controller.start_flash_mode(LEDControlMode.WARNING)
        time.sleep(3)

        # Stop flash
        print("Stopping flash")
        controller.stop_flash_mode()

    finally:
        controller.disconnect()

if __name__ == "__main__":
    basic_flash_demo()
```

### Environmental Adaptation Control Example

```python
from src.claudia.robot_controller.unified_led_controller import UnifiedLEDController

def environmental_adaptation_demo():
    """Environmental adaptation demonstration"""
    controller = UnifiedLEDController()

    # Automatically detect ambient light and adjust brightness
    light_info = controller.analyze_environmental_light()
    print(f"Ambient light type: {light_info.category}")
    print(f"Suggested brightness: {light_info.suggested_brightness}")

    # Auto-adjust based on environment
    controller.set_adaptive_brightness_enabled(True)
```

## Test Framework

### Complete Test Suite (`test/led_system/`)

Run all LED tests:
```bash
python3 test/led_system/run_led_tests.py
```

Categorized tests:
```bash
# Basic functionality test
python3 test/led_system/test_led_modes.py

# Performance test
python3 test/led_system/test_performance.py

# Quick verification
python3 test/led_system/quick_test.py
```

Test framework features:
- **led_test_base.py**: Test base class and utilities
- **Simulated connections**: Support for hardware-free test runs
- **Performance monitoring**: Latency and resource usage analysis
- **Error injection**: Fault scenario testing

## Demo Scripts

### Flash Mode Demo (`scripts/led_demos/flash_modes_demo.py`)

**Purpose**: Demonstrate all available LED flash modes

**Run methods**:
```bash
# Run complete demo
python3 scripts/led_demos/flash_modes_demo.py

# Run specific mode
python3 scripts/led_demos/flash_modes_demo.py --mode WAITING

# View help
python3 scripts/led_demos/flash_modes_demo.py --help
```

**Supported modes**:
- `NORMAL`: Normal operation - steady
- `WAITING`: Waiting for processing - single flash 1Hz
- `WARNING`: Warning state - fast flash 3Hz
- `ERROR`: Error state - double flash
- `SPECIAL`: Special state - breathing light

## Troubleshooting

### Common Issues

1. **Controller connection failure**
   ```bash
   # Check ROS2 environment
   source /opt/ros/foxy/setup.bash
   export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
   ```

2. **Flash mode not taking effect**
   - Check LED hardware connection
   - Verify control permissions
   - Review error logs

3. **Ambient light detection abnormal**
   - Confirm camera device is available
   - Check cv2 dependency installation
   - Verify image capture permissions

### Debugging Tips

Enable verbose logging:
```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

Check LED controller status:
```python
controller = ClaudiaLEDController()
print(f"Connection status: {controller.is_connected()}")
print(f"Current mode: {controller.get_current_mode()}")
```

## File Organization Structure

```
claudia/
+-- src/claudia/robot_controller/
|   +-- led_controller.py           # ClaudiaLEDController - flash control
|   +-- unified_led_controller.py   # UnifiedLEDController - environmental adaptation
|   +-- led_patterns.py            # LED mode definitions
|   +-- led_state_machine.py       # LED state machine
+-- test/led_system/                # Complete test framework
|   +-- run_led_tests.py           # Test runner
|   +-- test_led_modes.py          # Mode tests
|   +-- ...
+-- scripts/led_demos/              # Demo scripts
|   +-- flash_modes_demo.py        # Flash mode demo
+-- scripts/archive/                # Deprecated files
|   +-- validation_deprecated_*/    # Old validation scripts
|   +-- diagnostics_deprecated_*/  # Old diagnostic scripts
+-- docs/
    +-- LED_USAGE_GUIDE.md         # This document
```

## Best Practices

1. **Choose the appropriate controller**:
   - Status indication and flash -> ClaudiaLEDController
   - Environmental adaptive brightness -> UnifiedLEDController

2. **Error handling**:
   - Always use try-finally to ensure disconnection
   - Check controller connection status
   - Catch and log exceptions

3. **Performance optimization**:
   - Avoid frequent mode switching
   - Set reasonable flash frequencies
   - Monitor resource usage

4. **Test verification**:
   - Run basic tests before new features
   - Use simulation mode for development testing
   - Execute complete test suite before deployment

---

## Support Information

- **Testing issues**: See `test/led_system/README.md`
- **Development guide**: Refer to development docs in project root directory
- **Fault reporting**: Run relevant test scripts for detailed information

**Last Updated**: 2025-07-04
**Version**: LED control system cleanup restructured version
