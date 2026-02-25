#!/usr/bin/env python3
"""
LED Test Configuration Manager
Manages test parameters, environment settings, and test options
"""

import os
import json
from dataclasses import dataclass, asdict, field
from typing import Dict, Any, List, Optional
from pathlib import Path

@dataclass
class PerformanceConfig:
    """Performance test configuration"""
    max_response_time_ms: float = 200.0  # Maximum response time (milliseconds)
    stress_test_iterations: int = 100    # Stress test iteration count
    stress_test_duration: float = 10.0   # Stress test max duration (seconds)
    performance_samples: int = 50        # Performance sample count
    baseline_cpu_threshold: float = 80.0 # CPU usage threshold (%)
    baseline_memory_mb: float = 100.0    # Memory usage baseline (MB)

@dataclass
class LEDModeConfig:
    """LED mode test configuration"""
    wake_confirm_duration: float = 2.0      # Wake confirmation duration
    processing_timeout: float = 30.0        # Processing timeout
    action_complete_duration: float = 1.5   # Action complete duration
    error_state_duration: float = 2.5       # Error state duration
    mode_transition_delay: float = 0.1      # Mode transition delay

@dataclass
class EnvironmentConfig:
    """Environment test configuration"""
    light_levels: List[str] = field(default_factory=lambda: ["extremely_dark", "dark", "dim", "normal", "bright", "extremely_bright"])
    brightness_levels: List[int] = field(default_factory=lambda: [0, 25, 50, 75, 100])
    adaptation_timeout: float = 5.0         # Adaptation timeout
    camera_warmup_time: float = 2.0         # Camera warmup time

@dataclass
class HardwareConfig:
    """Hardware test configuration"""
    unitree_ip: str = "192.168.123.161"     # Unitree robot IP
    connection_timeout: float = 5.0          # Connection timeout
    retry_attempts: int = 3                  # Retry attempts
    hardware_required: bool = False          # Whether hardware is required
    mock_hardware: bool = True               # Whether to use mock hardware

@dataclass
class StabilityConfig:
    """Stability test configuration"""
    long_duration_hours: float = 1.0        # Long-duration test duration (hours)
    memory_leak_threshold_mb: float = 50.0  # Memory leak threshold (MB)
    error_rate_threshold: float = 1.0       # Error rate threshold (%)
    recovery_test_cycles: int = 10           # Recovery test cycle count

class LEDTestConfig:
    """LED test configuration manager"""

    def __init__(self, config_file: Optional[str] = None):
        """Initialize configuration manager"""
        self.config_file = config_file or "test/led_system/config.json"

        # Default configuration
        self.performance = PerformanceConfig()
        self.led_modes = LEDModeConfig()
        self.environment = EnvironmentConfig()
        self.hardware = HardwareConfig()
        self.stability = StabilityConfig()

        # Load config file (if exists)
        self.load_config()

        # Environment variable overrides
        self._apply_environment_overrides()

    def load_config(self):
        """Load configuration from file"""
        config_path = Path(self.config_file)
        if config_path.exists():
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    config_data = json.load(f)

                # Update configuration
                if 'performance' in config_data:
                    self._update_dataclass(self.performance, config_data['performance'])
                if 'led_modes' in config_data:
                    self._update_dataclass(self.led_modes, config_data['led_modes'])
                if 'environment' in config_data:
                    self._update_dataclass(self.environment, config_data['environment'])
                if 'hardware' in config_data:
                    self._update_dataclass(self.hardware, config_data['hardware'])
                if 'stability' in config_data:
                    self._update_dataclass(self.stability, config_data['stability'])

                print(f"Configuration loaded from {config_path}")

            except Exception as e:
                print(f"Config file load failed: {e}")

    def save_config(self):
        """Save configuration to file"""
        config_data = {
            'performance': asdict(self.performance),
            'led_modes': asdict(self.led_modes),
            'environment': asdict(self.environment),
            'hardware': asdict(self.hardware),
            'stability': asdict(self.stability)
        }

        config_path = Path(self.config_file)
        config_path.parent.mkdir(parents=True, exist_ok=True)

        try:
            with open(config_path, 'w', encoding='utf-8') as f:
                json.dump(config_data, f, indent=2, ensure_ascii=False)
            print(f"Configuration saved to {config_path}")
        except Exception as e:
            print(f"Config save failed: {e}")

    def _update_dataclass(self, dataclass_instance, update_dict: Dict[str, Any]):
        """Update dataclass instance"""
        for key, value in update_dict.items():
            if hasattr(dataclass_instance, key):
                setattr(dataclass_instance, key, value)

    def _apply_environment_overrides(self):
        """Apply environment variable overrides"""
        # Performance config environment variables
        max_response_time = os.getenv('LED_TEST_MAX_RESPONSE_TIME')
        if max_response_time is not None:
            self.performance.max_response_time_ms = float(max_response_time)

        stress_iterations = os.getenv('LED_TEST_STRESS_ITERATIONS')
        if stress_iterations is not None:
            self.performance.stress_test_iterations = int(stress_iterations)

        # Hardware config environment variables
        unitree_ip = os.getenv('UNITREE_IP')
        if unitree_ip is not None:
            self.hardware.unitree_ip = unitree_ip

        hardware_required = os.getenv('LED_TEST_HARDWARE_REQUIRED')
        if hardware_required is not None:
            self.hardware.hardware_required = hardware_required.lower() == 'true'

        mock_hardware = os.getenv('LED_TEST_MOCK_HARDWARE')
        if mock_hardware is not None:
            self.hardware.mock_hardware = mock_hardware.lower() == 'true'

    def get_test_mode(self) -> str:
        """Get test mode"""
        if self.hardware.hardware_required and not self.hardware.mock_hardware:
            return "hardware"
        elif self.hardware.mock_hardware:
            return "simulation"
        else:
            return "mixed"

    def is_performance_test_enabled(self) -> bool:
        """Whether performance tests are enabled"""
        return os.getenv('LED_TEST_SKIP_PERFORMANCE', 'false').lower() != 'true'

    def is_stress_test_enabled(self) -> bool:
        """Whether stress tests are enabled"""
        return os.getenv('LED_TEST_SKIP_STRESS', 'false').lower() != 'true'

    def is_long_duration_test_enabled(self) -> bool:
        """Whether long-duration tests are enabled"""
        return os.getenv('LED_TEST_LONG_DURATION', 'false').lower() == 'true'

    def get_test_output_dir(self) -> Path:
        """Get test output directory"""
        output_dir = os.getenv('LED_TEST_OUTPUT_DIR', 'logs/led_tests')
        return Path(output_dir)

    def print_config_summary(self):
        """Print configuration summary"""
        print("LED Test Configuration Summary:")
        print(f"   Test mode: {self.get_test_mode()}")
        print(f"   Max response time: {self.performance.max_response_time_ms}ms")
        print(f"   Stress test iterations: {self.performance.stress_test_iterations}")
        print(f"   Hardware IP: {self.hardware.unitree_ip}")
        print(f"   Performance tests: {'enabled' if self.is_performance_test_enabled() else 'disabled'}")
        print(f"   Stress tests: {'enabled' if self.is_stress_test_enabled() else 'disabled'}")
        print(f"   Long-duration tests: {'enabled' if self.is_long_duration_test_enabled() else 'disabled'}")
        print(f"   Output directory: {self.get_test_output_dir()}")

# Global config instance
_global_config = None

def get_led_test_config() -> LEDTestConfig:
    """Get global LED test configuration instance"""
    global _global_config
    if _global_config is None:
        _global_config = LEDTestConfig()
    return _global_config

def reset_led_test_config():
    """Reset global configuration (for testing)"""
    global _global_config
    _global_config = None

if __name__ == "__main__":
    # Configuration demo
    config = LEDTestConfig()
    config.print_config_summary()

    # Save default config
    config.save_config()
    print("Default configuration generated")
