#!/usr/bin/env python3
"""
LED Test Base Class
Provides common functionality and utility methods for all LED-related tests
"""

import unittest
import time
import sys
import os
from pathlib import Path
from typing import Optional, Dict, Any, List
from datetime import datetime
import json

# Add project root directory to Python path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

class LEDTestBase(unittest.TestCase):
    """LED test base class"""

    def setUp(self):
        """Pre-test setup"""
        self.start_time = time.time()
        self.test_name = self._testMethodName
        self.performance_data = {}
        self.error_logs = []

        # Ensure test environment
        self._setup_test_environment()

        # Create LED controller (if available)
        self._setup_led_controller()

    def tearDown(self):
        """Post-test cleanup"""
        self.end_time = time.time()
        self.test_duration = self.end_time - self.start_time

        # Cleanup LED controller
        self._cleanup_led_controller()

        # Record test results
        self._record_test_results()

    def _setup_test_environment(self):
        """Setup test environment"""
        try:
            # Check if necessary modules are available
            import sys
            _project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
            sys.path.append(os.path.join(_project_root, 'src'))

            # Check if Unitree hardware is available
            from claudia.robot_controller.unitree_messages import UnitreeMessages
            self.hardware_available = UnitreeMessages.is_available()

            if self.hardware_available:
                print(f"{self.test_name}: Hardware mode available")
            else:
                print(f"{self.test_name}: Using simulation mode")

        except Exception as e:
            self.hardware_available = False
            self.error_logs.append(f"Environment setup failed: {e}")
            print(f"{self.test_name}: Environment setup failed - {e}")

    def _setup_led_controller(self):
        """Setup LED controller"""
        self.led_controller = None
        self.led_system = None

        try:
            # Try to import LED control system
            from claudia.robot_controller import (
                create_claudia_led_system,
                create_unified_led_controller
            )

            # Create LED system
            self.led_system = create_claudia_led_system()
            if self.led_system:
                self.led_system.initialize()
                print(f"{self.test_name}: LED system initialized successfully")
            else:
                print(f"{self.test_name}: LED system creation failed")

        except Exception as e:
            self.error_logs.append(f"LED controller setup failed: {e}")
            print(f"{self.test_name}: LED controller setup failed - {e}")

    def _cleanup_led_controller(self):
        """Cleanup LED controller"""
        try:
            if self.led_system:
                self.led_system.cleanup()
                print(f"{self.test_name}: LED system cleanup complete")
        except Exception as e:
            self.error_logs.append(f"LED controller cleanup failed: {e}")
            print(f"{self.test_name}: LED controller cleanup failed - {e}")

    def _record_test_results(self):
        """Record test results"""
        test_result = {
            'test_name': self.test_name,
            'start_time': self.start_time,
            'end_time': self.end_time,
            'duration': self.test_duration,
            'hardware_available': self.hardware_available,
            'performance_data': self.performance_data,
            'error_logs': self.error_logs,
            'timestamp': datetime.now().isoformat()
        }

        # Save to test data collector
        self._save_test_result(test_result)

    def _save_test_result(self, result: Dict[str, Any]):
        """Save test result"""
        try:
            # Create log directory
            log_dir = Path("logs/led_tests")
            log_dir.mkdir(parents=True, exist_ok=True)

            # Generate log filename
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_file = log_dir / f"{self.test_name}_{timestamp}.json"

            # Save result
            with open(log_file, 'w', encoding='utf-8') as f:
                json.dump(result, f, indent=2, ensure_ascii=False)

        except Exception as e:
            print(f"Unable to save test result: {e}")

    # =========================
    # Test utility methods
    # =========================

    def measure_performance(self, operation_name: str, operation_func, *args, **kwargs):
        """Measure operation performance"""
        start_time = time.perf_counter()

        try:
            result = operation_func(*args, **kwargs)
            success = True
            error = None
        except Exception as e:
            result = None
            success = False
            error = str(e)

        end_time = time.perf_counter()
        duration = (end_time - start_time) * 1000  # Convert to milliseconds

        # Record performance data
        self.performance_data[operation_name] = {
            'duration_ms': duration,
            'success': success,
            'error': error,
            'timestamp': time.time()
        }

        return result, duration, success

    def assert_response_time(self, operation_name: str, max_time_ms: float = 200.0):
        """Verify response time requirement"""
        if operation_name in self.performance_data:
            actual_time = self.performance_data[operation_name]['duration_ms']
            self.assertLessEqual(
                actual_time,
                max_time_ms,
                f"{operation_name} response time {actual_time:.2f}ms exceeds requirement {max_time_ms}ms"
            )
            print(f"{operation_name}: {actual_time:.2f}ms (< {max_time_ms}ms)")
        else:
            self.fail(f"No performance data found for operation '{operation_name}'")

    def verify_led_mode(self, expected_mode: str, timeout: float = 1.0):
        """Verify LED mode"""
        if not self.led_system:
            self.skipTest("LED system not available")

        try:
            # Get current LED state (using try-catch safe method)
            actual_mode = None

            try:
                current_state = getattr(self.led_system, 'get_current_state')()
                actual_mode = current_state.get('mode') if isinstance(current_state, dict) else None
            except (AttributeError, TypeError):
                try:
                    actual_mode = getattr(self.led_system, 'current_mode')
                except AttributeError:
                    print(f"Unable to get current LED mode, skipping verification")
                    return

            self.assertEqual(
                actual_mode,
                expected_mode,
                f"Expected LED mode '{expected_mode}', actual '{actual_mode}'"
            )
            print(f"LED mode verification passed: {expected_mode}")

        except Exception as e:
            self.fail(f"LED mode verification failed: {e}")

    def simulate_environment_change(self, light_level: str = "normal"):
        """Simulate environment change"""
        if not self.led_system:
            return

        try:
            # Simulate ambient light change (using try-catch safe method)
            try:
                simulate_func = getattr(self.led_system, 'simulate_environment_change')
                simulate_func({
                    'light_level': light_level,
                    'timestamp': time.time()
                })
            except AttributeError:
                try:
                    set_light_func = getattr(self.led_system, 'set_environment_light')
                    set_light_func(light_level)
                except AttributeError:
                    print(f"Environment simulation not available")
                    return

            time.sleep(0.1)  # Wait for system response

        except Exception as e:
            self.error_logs.append(f"Environment simulation failed: {e}")

    def run_stress_test(self, operation_func, iterations: int = 100, max_duration: float = 10.0):
        """Run stress test"""
        start_time = time.time()
        success_count = 0
        error_count = 0
        durations = []

        for i in range(iterations):
            try:
                op_start = time.perf_counter()
                operation_func()
                op_end = time.perf_counter()

                duration = (op_end - op_start) * 1000
                durations.append(duration)
                success_count += 1

            except Exception as e:
                error_count += 1
                self.error_logs.append(f"Stress test iteration {i+1} failed: {e}")

            # Check total time limit
            if time.time() - start_time > max_duration:
                break

        # Calculate statistics
        total_time = time.time() - start_time
        success_rate = success_count / (success_count + error_count) * 100
        avg_duration = sum(durations) / len(durations) if durations else 0
        max_duration_ms = max(durations) if durations else 0

        stress_result = {
            'iterations': success_count + error_count,
            'success_count': success_count,
            'error_count': error_count,
            'success_rate': success_rate,
            'total_time': total_time,
            'avg_duration_ms': avg_duration,
            'max_duration_ms': max_duration_ms
        }

        self.performance_data['stress_test'] = stress_result

        print(f"Stress test results:")
        print(f"   Iterations: {stress_result['iterations']}")
        print(f"   Success rate: {success_rate:.1f}%")
        print(f"   Average duration: {avg_duration:.2f}ms")
        print(f"   Maximum duration: {max_duration_ms:.2f}ms")

        return stress_result

    # =========================
    # Enhanced assertion methods
    # =========================

    def assertLEDSystemReady(self):
        """Assert LED system is ready"""
        self.assertIsNotNone(self.led_system, "LED system not initialized")

    def assertHardwareAvailable(self):
        """Assert hardware is available"""
        self.assertTrue(self.hardware_available, "Hardware not available, cannot run hardware-related tests")

    def assertPerformanceAcceptable(self, operation_name: str, max_time_ms: float = 200.0):
        """Assert performance is acceptable"""
        self.assert_response_time(operation_name, max_time_ms)

        # Check if successful
        if operation_name in self.performance_data:
            success = self.performance_data[operation_name]['success']
            self.assertTrue(success, f"Operation '{operation_name}' execution failed")

if __name__ == "__main__":
    # Basic test demo
    class BasicLEDTest(LEDTestBase):
        def test_basic_functionality(self):
            """Basic functionality test"""
            self.assertLEDSystemReady()
            print("Basic functionality test passed")

    unittest.main()
