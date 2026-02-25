"""
Claudia Robot Project Test Helper Utilities

Provides commonly used helper functions and utility classes for testing.
"""

import os
import sys
import time
import tempfile
import shutil
from pathlib import Path
from typing import Any, Dict, Optional, List
from contextlib import contextmanager

# Add project root directory to Python path
PROJECT_ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

class TestEnvironment:
    """Test environment management"""

    def __init__(self, test_name: str):
        self.test_name = test_name
        self.start_time = time.time()
        self.temp_dirs = []
        self.cleanup_callbacks = []

    def create_temp_dir(self, prefix: str = "claudia_test_") -> Path:
        """Create a temporary directory"""
        temp_dir = Path(tempfile.mkdtemp(prefix=f"{prefix}{self.test_name}_"))
        self.temp_dirs.append(temp_dir)
        return temp_dir

    def add_cleanup(self, callback):
        """Add a cleanup callback function"""
        self.cleanup_callbacks.append(callback)

    def cleanup(self):
        """Clean up the test environment"""
        # Execute cleanup callbacks
        for callback in self.cleanup_callbacks:
            try:
                callback()
            except Exception as e:
                print(f"Cleanup callback execution failed: {e}")

        # Clean up temporary directories
        for temp_dir in self.temp_dirs:
            if temp_dir.exists():
                try:
                    shutil.rmtree(temp_dir)
                except Exception as e:
                    print(f"Failed to clean up temporary directory {temp_dir}: {e}")

        # Print test duration
        duration = time.time() - self.start_time
        print(f"Test {self.test_name} run time: {duration:.2f}s")

def setup_test_environment(test_name: str = "unknown") -> TestEnvironment:
    """Set up the test environment"""
    return TestEnvironment(test_name)

@contextmanager
def mock_environment_variables(**env_vars):
    """Temporarily set environment variables"""
    old_env = {}

    # Save original environment variables
    for key, value in env_vars.items():
        old_env[key] = os.environ.get(key)
        os.environ[key] = str(value)

    try:
        yield
    finally:
        # Restore original environment variables
        for key, old_value in old_env.items():
            if old_value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = old_value

def wait_for_condition(condition_func, timeout: float = 10.0,
                      interval: float = 0.1, description: str = "condition met") -> bool:
    """Wait for a condition to be met"""
    start_time = time.time()

    while time.time() - start_time < timeout:
        try:
            if condition_func():
                return True
        except Exception as e:
            # Condition check function raised an error, continue waiting
            pass

        time.sleep(interval)

    print(f"Wait timed out: {description} (timeout: {timeout}s)")
    return False

def create_mock_config(config_data: Dict[str, Any],
                      config_file: Optional[Path] = None) -> Path:
    """Create a mock configuration file"""
    if config_file is None:
        config_file = Path(tempfile.mktemp(suffix=".yaml"))

    import yaml
    with open(config_file, 'w', encoding='utf-8') as f:
        yaml.dump(config_data, f, default_flow_style=False, allow_unicode=True)

    return config_file

def simulate_robot_response(topic: str, message_type: str,
                          data: Dict[str, Any]) -> Dict[str, Any]:
    """Simulate a robot response message"""
    return {
        'topic': topic,
        'message_type': message_type,
        'timestamp': time.time(),
        'data': data,
        'simulated': True
    }

class MockRobotConnection:
    """Mock robot connection"""

    def __init__(self, simulation_mode: bool = True):
        self.simulation_mode = simulation_mode
        self.connected = False
        self.mock_data = {}

    def connect(self) -> bool:
        """Simulate connection"""
        if self.simulation_mode:
            self.connected = True
            return True
        else:
            # Actual hardware connection logic
            return False

    def disconnect(self):
        """Disconnect"""
        self.connected = False

    def set_mock_data(self, topic: str, data: Any):
        """Set mock data"""
        self.mock_data[topic] = data

    def get_data(self, topic: str) -> Optional[Any]:
        """Get data"""
        if self.simulation_mode:
            return self.mock_data.get(topic)
        else:
            # Actual hardware data retrieval logic
            return None

def validate_ros2_environment() -> bool:
    """Validate that the ROS2 environment is correctly configured"""
    required_env_vars = [
        'ROS_VERSION',
        'ROS_DISTRO',
        'RMW_IMPLEMENTATION'
    ]

    missing_vars = []
    for var in required_env_vars:
        if var not in os.environ:
            missing_vars.append(var)

    if missing_vars:
        print(f"Missing ROS2 environment variables: {missing_vars}")
        return False

    # Check ROS2 version
    if os.environ.get('ROS_VERSION') != '2':
        print(f"ROS2 required, current version: {os.environ.get('ROS_VERSION')}")
        return False

    return True

def check_network_connectivity(host: str = "8.8.8.8", port: int = 53, timeout: float = 3.0) -> bool:
    """Check network connectivity"""
    import socket

    try:
        socket.setdefaulttimeout(timeout)
        socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect((host, port))
        return True
    except socket.error:
        return False

def get_available_network_interfaces() -> List[str]:
    """Get available network interfaces"""
    import subprocess

    try:
        # Linux/macOS
        result = subprocess.run(['ip', 'link', 'show'],
                              capture_output=True, text=True)
        interfaces = []
        for line in result.stdout.split('\n'):
            if ': ' in line and 'state UP' in line:
                interface = line.split(':')[1].strip().split('@')[0]
                interfaces.append(interface)
        return interfaces
    except:
        try:
            # Fallback method
            result = subprocess.run(['ifconfig'],
                                  capture_output=True, text=True)
            interfaces = []
            for line in result.stdout.split('\n'):
                if line and not line.startswith(' ') and ':' in line:
                    interface = line.split(':')[0]
                    interfaces.append(interface)
            return interfaces
        except:
            return ['eth0', 'enp2s0', 'wlan0']  # Default interface names

def create_test_log_file(test_name: str, content: str) -> Path:
    """Create a test log file"""
    log_dir = PROJECT_ROOT / "logs" / "tests"
    log_dir.mkdir(parents=True, exist_ok=True)

    timestamp = time.strftime('%Y%m%d_%H%M%S')
    log_file = log_dir / f"{timestamp}_{test_name}.log"

    with open(log_file, 'w', encoding='utf-8') as f:
        f.write(f"Test: {test_name}\n")
        f.write(f"Time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write("-" * 50 + "\n")
        f.write(content)

    return log_file

# Test decorators
def hardware_test(require_robot: bool = True):
    """Hardware test decorator"""
    def decorator(func):
        def wrapper(*args, **kwargs):
            if require_robot:
                print(f"Hardware test {func.__name__} requires a real robot connection")
            return func(*args, **kwargs)
        return wrapper
    return decorator

def integration_test(dependencies: Optional[List[str]] = None):
    """Integration test decorator"""
    def decorator(func):
        def wrapper(*args, **kwargs):
            if dependencies:
                print(f"Integration test {func.__name__} dependencies: {', '.join(dependencies)}")
            return func(*args, **kwargs)
        return wrapper
    return decorator
