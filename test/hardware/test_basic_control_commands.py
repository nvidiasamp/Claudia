#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unitree Go2 Basic Control Command Tests
Generated: 2024-12-26 17:56:16
Safety: Includes safety confirmation prompts to prevent unexpected robot falls
Updated: 2025-06-26 18:17:00 - Corrected to official example standard initialization method
"""

import time
import os
import sys
from datetime import datetime

# Add SDK path
_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.append(os.path.join(_PROJECT_ROOT, 'unitree_sdk2_python'))

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    print("Successfully imported all required modules")
except ImportError as e:
    print(f"Import error: {e}")
    print("Please ensure unitree_sdk2py is properly installed")
    sys.exit(1)

class BasicControlTest:
    def __init__(self):
        self.results = []
        self.start_time = None
        self.network_interface = "eth0"  # Go2 default network interface

    def log_result(self, command, success, duration, notes=""):
        """Record test result"""
        result = {
            'command': command,
            'success': success,
            'duration': duration,
            'timestamp': datetime.now().strftime('%H:%M:%S'),
            'notes': notes
        }
        self.results.append(result)
        status = "PASS" if success else "FAIL"
        print(f"[{status}] {command}: {duration:.3f}s - {notes}")

    def safety_confirmation(self):
        """Safety confirmation prompt"""
        print("\n" + "="*60)
        print("SAFETY NOTICE - Unitree Go2 Basic Control Command Test")
        print("="*60)
        print("WARNING: Please ensure:")
        print("   1. There is sufficient safe space around the robot")
        print("   2. No people or obstacles are within the robot's operating range")
        print("   3. You are ready to press the emergency stop button on the remote in case of emergency")
        print("   4. The robot battery is sufficiently charged")
        print("\nTest will execute the following safety sequence:")
        print("   1. Sit() - Safe sitting state")
        print("   2. StandUp() - Stand up action")
        print("   3. StandDown() - Lie down action")
        print("   4. RecoveryStand() - Recovery stand")
        print("   5. Maintain standing state - For subsequent operations")
        print("\n" + "="*60)

        response = input("Confirm safety conditions are met, continue testing? (yes/no): ").lower().strip()
        if response not in ['yes', 'y']:
            print("Test cancelled")
            return False
        return True

    def run_test(self):
        """Execute basic control command tests (following official example style)"""
        if not self.safety_confirmation():
            return

        print(f"\nStarting basic control command test - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"Network interface: {self.network_interface}")

        try:
            # Initialize DDS channel following official example
            print("Initializing DDS channel factory...")
            ChannelFactoryInitialize(0, self.network_interface)
            print("DDS channel factory initialized successfully")

            # Create SportClient (following official example style)
            print("Creating SportClient...")
            client = SportClient()
            client.SetTimeout(10.0)

            # Following official example, call Init() directly without checking return value
            print("Initializing SportClient...")
            client.Init()
            print("SportClient initialization complete (official example style)")

            # Wait for connection to stabilize
            time.sleep(2)

            # Test sequence - safety-optimized version
            test_commands = [
                ("Sit", lambda: client.Sit(), "Safe sitting state"),
                ("StandUp", lambda: client.StandUp(), "Stand up action"),
                ("StandDown", lambda: client.StandDown(), "Lie down action"),
                ("RecoveryStand", lambda: client.RecoveryStand(), "Recovery stand"),
            ]

            print(f"\nExecuting test sequence ({len(test_commands)} commands):")
            print("-" * 50)

            for i, (cmd_name, cmd_func, description) in enumerate(test_commands, 1):
                print(f"\n[{i}/{len(test_commands)}] Executing {cmd_name}() - {description}")

                start_time = time.time()
                try:
                    result = cmd_func()
                    duration = time.time() - start_time

                    if result == 0:  # 0 means success
                        self.log_result(cmd_name, True, duration, f"{description} - Command sent successfully")

                        # Wait for action to complete
                        if cmd_name in ["StandUp", "StandDown", "RecoveryStand"]:
                            print(f"   Waiting for {description} to complete...")
                            time.sleep(5)  # Give robot enough time to complete action
                        else:
                            time.sleep(2)  # Shorter wait for other commands

                    else:
                        self.log_result(cmd_name, False, duration, f"Command failed, return code: {result}")

                except Exception as e:
                    duration = time.time() - start_time
                    self.log_result(cmd_name, False, duration, f"Execution exception: {str(e)}")

            print("\nTest complete, robot maintains standing state for subsequent operations")

        except Exception as e:
            print(f"Test process exception: {e}")
            print("Please verify robot network connection and SDK configuration")
            import traceback
            traceback.print_exc()
            return

        # Generate test report
        self.generate_report()

    def generate_report(self):
        """Generate detailed test report"""
        print("\n" + "="*60)
        print("Basic Control Command Test Report")
        print("="*60)

        success_count = sum(1 for r in self.results if r['success'])
        total_count = len(self.results)
        success_rate = (success_count / total_count * 100) if total_count > 0 else 0

        print(f"Overall result: {success_count}/{total_count} passed ({success_rate:.1f}%)")
        print(f"Test time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

        print("\nDetailed results:")
        print("-" * 60)
        for result in self.results:
            status = "PASS" if result['success'] else "FAIL"
            print(f"[{status}] {result['timestamp']} | {result['command']:<12} | "
                  f"{result['duration']:>6.3f}s | {result['notes']}")

        # Performance statistics
        if self.results:
            durations = [r['duration'] for r in self.results if r['success']]
            if durations:
                avg_duration = sum(durations) / len(durations)
                max_duration = max(durations)
                min_duration = min(durations)

                print(f"\nPerformance metrics:")
                print(f"   Average response time: {avg_duration:.3f}s")
                print(f"   Fastest response time: {min_duration:.3f}s")
                print(f"   Slowest response time: {max_duration:.3f}s")

        print("\nBasic control function verification complete")
        print("Robot is now in standing state, ready for subsequent operations")
        print("="*60)

def main():
    """Main function"""
    # Set correct environment variables (corrected)
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

    print("Unitree Go2 Basic Control Command Test (Official Example Style)")
    print(f"Start time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("Initialization method: Official standard (ChannelFactoryInitialize + direct Init)")

    tester = BasicControlTest()
    tester.run_test()

if __name__ == "__main__":
    main()
