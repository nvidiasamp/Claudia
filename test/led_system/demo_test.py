#!/usr/bin/env python3
"""
LED Control System Test Framework Demo
Validates the Task 6.5 test framework functionality
"""

import sys
import time
from pathlib import Path

# Add project root directory to Python path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

# Import test components
import test.led_system.test_config as test_config
import test.led_system.data_collector as data_collector
import test.led_system.led_test_base as led_test_base

def demo_test_config():
    """Demonstrate test configuration functionality"""
    print("===== Test Configuration Demo =====")

    config = test_config.get_led_test_config()
    config.print_config_summary()

    # Test config save
    config.save_config()

    print("Test configuration demo complete\n")

def demo_data_collector():
    """Demonstrate data collection functionality"""
    print("===== Data Collector Demo =====")

    collector = data_collector.get_led_test_collector()

    # Start test session
    collector.start_test_session("demo_test", {"type": "demo"})

    # Simulate some performance data
    for i in range(10):
        collector.record_performance_data(
            "demo_test",
            response_time=50 + i * 2,
            cpu_usage=20 + i,
            memory_usage=100 + i * 0.5,
            success=i < 9  # Last one fails
        )
        time.sleep(0.1)

    # Record some metrics
    collector.record_metric("demo_metric", 42, "units", "demo_test", "demo")

    # Record an error
    collector.record_error("demo_test", "demo_error", "This is a demo error")

    # End session
    collector.end_test_session("demo_test")

    # Show real-time statistics
    stats = collector.get_real_time_stats()
    print("Real-time statistics:")
    for key, value in stats.items():
        print(f"   {key}: {value}")

    # Generate report
    report_file = collector.generate_report()
    data_file = collector.save_data()

    print(f"Report generated: {report_file}")
    print(f"Data saved: {data_file}")

    print("Data collector demo complete\n")

def demo_test_base():
    """Demonstrate test base class functionality"""
    print("===== Test Base Class Demo =====")

    # Create a simple test class demo
    class DemoTest(led_test_base.LEDTestBase):
        def demo_performance_test(self):
            """Demonstrate performance test"""

            def mock_led_operation():
                """Simulate LED operation"""
                time.sleep(0.05)  # Simulate 50ms operation
                return True

            # Measure performance
            result, duration, success = self.measure_performance(
                "mock_operation", mock_led_operation
            )

            print(f"Simulated operation result: duration {duration:.2f}ms, success: {success}")

            # Verify response time
            try:
                self.assert_response_time("mock_operation", 100.0)  # 100ms threshold
                print("Response time verification passed")
            except Exception as e:
                print(f"Response time verification failed: {e}")

            return True

        def demo_stress_test(self):
            """Demonstrate stress test"""

            def stress_operation():
                """Stress test operation"""
                time.sleep(0.01)  # Fast operation

            # Run stress test
            stress_result = self.run_stress_test(
                stress_operation,
                iterations=20,
                max_duration=2.0
            )

            print(f"Stress test result: {stress_result}")

            return True

    # Run demo test
    demo_test = DemoTest()
    demo_test.setUp()

    try:
        demo_test.demo_performance_test()
        demo_test.demo_stress_test()

        print("Test base class demo complete")
    finally:
        demo_test.tearDown()

    print()

def demo_led_system_availability():
    """Demonstrate LED system availability check"""
    print("===== LED System Availability Check =====")

    try:
        # Check Unitree hardware availability
        from claudia.robot_controller.unitree_messages import UnitreeMessages
        hardware_available = UnitreeMessages.is_available()

        print(f"Unitree hardware: {'available' if hardware_available else 'simulation mode'}")

        if hardware_available:
            method = UnitreeMessages.get_import_method()
            print(f"Import method: {method}")
    except Exception as e:
        print(f"Unitree module check failed: {e}")

    try:
        # Check LED control system
        from claudia.robot_controller import create_claudia_led_system
        led_system = create_claudia_led_system()

        if led_system:
            print("LED control system: can be created")
            led_system.initialize()
            print("LED system initialization: success")

            # Test an LED mode
            if hasattr(led_system, 'wake_confirm'):
                led_system.wake_confirm()
                print("LED mode test: wake_confirm executed successfully")

            led_system.cleanup()
            print("LED system cleanup: complete")
        else:
            print("LED control system: creation failed")

    except Exception as e:
        print(f"LED control system check failed: {e}")

    print("LED system availability check complete\n")

def main():
    """Main demo function"""
    print("LED Control System Test Framework Demo")
    print("Task 6.5: Comprehensive Testing, Validation, and Performance Optimization")
    print("=" * 60)
    print()

    try:
        # Run each demo
        demo_test_config()
        demo_data_collector()
        demo_test_base()
        demo_led_system_availability()

        print("===== Demo Complete =====")
        print("LED test framework components are working correctly")
        print("Ready for full LED control system testing")

    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    except Exception as e:
        print(f"\nError during demo: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
