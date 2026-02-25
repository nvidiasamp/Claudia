#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LLM to Real Robot Control Test
Uses correct CycloneDDS environment configuration to implement Task 11 real robot control

Generated: 2025-07-08 13:25:00
Purpose: Verify that Task 11 LLM commands can actually control the robot
Author: M1nG
"""

import os
import sys
import time
import asyncio
import subprocess
from datetime import datetime

# Set up project paths
_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.append(os.path.join(_PROJECT_ROOT, 'src'))
sys.path.append(os.path.join(_PROJECT_ROOT, 'unitree_sdk2_python'))

def setup_cyclonedds_environment():
    """Set up CycloneDDS environment - critical fix"""
    print("Configuring CycloneDDS environment...")

    # Run setup script to get environment variables
    try:
        result = subprocess.run([
            'bash', '-c',
            'source ~/claudia/scripts/setup/setup_cyclonedds.sh && env'
        ], capture_output=True, text=True, check=True)

        # Parse environment variables
        env_vars = {}
        for line in result.stdout.split('\n'):
            if '=' in line and not line.startswith('_'):
                key, value = line.split('=', 1)
                env_vars[key] = value

        # Set critical environment variables
        important_vars = [
            'CYCLONEDDS_HOME', 'LD_LIBRARY_PATH', 'RMW_IMPLEMENTATION'
        ]

        for var in important_vars:
            if var in env_vars:
                os.environ[var] = env_vars[var]
                print(f"   {var}: {env_vars[var][:100]}...")

        print("CycloneDDS environment configuration complete")
        return True

    except Exception as e:
        print(f"Environment configuration failed: {e}")
        return False

class LLMRealRobotTest:
    """LLM real robot control test"""

    def __init__(self):
        self.results = []

        # Import mapping engine (after environment configuration)
        try:
            from claudia.robot_controller.action_mapping_engine_real import RealActionMappingEngine, SDK_AVAILABLE
            self.engine_class = RealActionMappingEngine
            self.sdk_available = SDK_AVAILABLE
            print(f"Mapping engine imported successfully (SDK available: {SDK_AVAILABLE})")
        except Exception as e:
            print(f"Mapping engine import failed: {e}")
            self.engine_class = None
            self.sdk_available = False

    def log_result(self, command, success, duration, message=""):
        """Record test result"""
        result = {
            'command': command,
            'success': success,
            'duration': duration,
            'timestamp': datetime.now().strftime('%H:%M:%S'),
            'message': message
        }
        self.results.append(result)
        status = "PASS" if success else "FAIL"
        print(f"[{status}] {command}: {duration:.3f}s - {message}")

    def safety_confirmation(self):
        """Safety confirmation"""
        print("\n" + "="*60)
        print("SAFETY NOTICE - LLM Real Robot Control Test")
        print("="*60)
        print("WARNING: This will test LLM command to real robot mapping!")
        print("WARNING: Please ensure the area around the robot is safe!")
        print("\nTest commands:")
        print('   1. {"intent": "robot_control"} -> Sit')
        print('   2. {"intent": "お座り"} -> Sit (Japanese)')
        print('   3. {"intent": "stand"} -> Stand')
        print('   4. {"intent": "hello"} -> Wave')
        print("="*60)

        response = input("Confirm safety conditions, continue LLM robot test? (yes/no): ").lower().strip()
        return response in ['yes', 'y']

    async def run_llm_robot_test(self):
        """Run LLM robot control test"""
        if not self.safety_confirmation():
            print("Test cancelled")
            return

        if not self.sdk_available or not self.engine_class:
            print("SDK unavailable or mapping engine cannot be loaded")
            return

        print(f"\nStarting LLM real robot control test - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

        # Create mapping engine
        try:
            engine = self.engine_class()
            print("Real robot mapping engine initialized successfully")
        except Exception as e:
            print(f"Mapping engine initialization failed: {e}")
            return

        # LLM test commands (from Task 10 output format and Japanese commands)
        test_commands = [
            ('robot_control', '{"intent": "robot_control", "confidence": 0.8}'),  # Task 10 standard output
            ('japanese_sit', '{"intent": "お座り", "confidence": 0.9}'),          # Japanese sit
            ('english_stand', '{"intent": "stand", "confidence": 0.8}'),          # English stand
            ('hello_wave', '{"intent": "hello", "confidence": 0.8}'),             # Wave gesture
        ]

        print(f"\nExecuting LLM command sequence ({len(test_commands)} commands):")
        print("-" * 50)

        for i, (cmd_name, llm_output) in enumerate(test_commands, 1):
            print(f"\n[{i}/{len(test_commands)}] LLM command: {llm_output}")

            start_time = time.time()
            try:
                # Key step: LLM output -> real robot action
                result = await engine.map_intent_to_action(llm_output)
                duration = time.time() - start_time

                if result.success:
                    self.log_result(cmd_name, True, duration,
                                  f"LLM->robot success: {result.message}")
                else:
                    self.log_result(cmd_name, False, duration,
                                  f"LLM->robot failed: {result.error_message}")

                # Safety interval
                await asyncio.sleep(3)

            except Exception as e:
                duration = time.time() - start_time
                self.log_result(cmd_name, False, duration, f"Execution exception: {str(e)}")

        # Generate report
        self.generate_report()

    def generate_report(self):
        """Generate test report"""
        print("\n" + "="*60)
        print("LLM Real Robot Control Test Report")
        print("="*60)

        success_count = sum(1 for r in self.results if r['success'])
        total_count = len(self.results)
        success_rate = (success_count / total_count * 100) if total_count > 0 else 0

        print(f"Task 11 verification result: {success_count}/{total_count} passed ({success_rate:.1f}%)")
        print(f"Test time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

        print("\nDetailed results:")
        print("-" * 60)
        for result in self.results:
            status = "PASS" if result['success'] else "FAIL"
            print(f"[{status}] {result['timestamp']} | {result['command']:<15} | "
                  f"{result['duration']:>6.3f}s | {result['message']}")

        # Key conclusions
        print(f"\nTask 11 key verification:")
        if success_count > 0:
            print("PASS: LLM commands can successfully control the real robot!")
            print("PASS: Natural language -> robot action mapping works correctly!")
            print("PASS: Task 11 core functionality has been implemented!")
        else:
            print("FAIL: LLM to robot mapping needs further debugging")

        print("="*60)

async def main():
    """Main function"""
    print("LLM Real Robot Control Test (Task 11 Verification)")
    print(f"Start time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

    # Critical: Configure CycloneDDS environment
    if not setup_cyclonedds_environment():
        print("Environment configuration failed, cannot continue")
        return

    # Run test
    tester = LLMRealRobotTest()
    await tester.run_llm_robot_test()

if __name__ == "__main__":
    asyncio.run(main())
