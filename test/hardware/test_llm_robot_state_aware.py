#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
State-Aware LLM Robot Control Test
Avoids duplicate state command conflicts, validates Task 11 state management

Generated: 2025-07-08 13:30:00
Purpose: Resolve state conflict issues, optimize LLM command sequences
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
    """Set up CycloneDDS environment"""
    print("Configuring CycloneDDS environment...")

    try:
        result = subprocess.run([
            'bash', '-c',
            'source ~/claudia/scripts/setup/setup_cyclonedds.sh && env'
        ], capture_output=True, text=True, check=True)

        env_vars = {}
        for line in result.stdout.split('\n'):
            if '=' in line and not line.startswith('_'):
                key, value = line.split('=', 1)
                env_vars[key] = value

        important_vars = ['CYCLONEDDS_HOME', 'LD_LIBRARY_PATH', 'RMW_IMPLEMENTATION']

        for var in important_vars:
            if var in env_vars:
                os.environ[var] = env_vars[var]
                print(f"   {var}: {env_vars[var][:100]}...")

        print("CycloneDDS environment configuration complete")
        return True

    except Exception as e:
        print(f"Environment configuration failed: {e}")
        return False

class StateAwareLLMTest:
    """State-aware LLM robot test"""

    def __init__(self):
        self.results = []
        self.current_robot_state = "unknown"  # Track robot state

        try:
            from claudia.robot_controller.action_mapping_engine_real import RealActionMappingEngine, SDK_AVAILABLE
            self.engine_class = RealActionMappingEngine
            self.sdk_available = SDK_AVAILABLE
            print(f"Mapping engine imported successfully (SDK available: {SDK_AVAILABLE})")
        except Exception as e:
            print(f"Mapping engine import failed: {e}")
            self.engine_class = None
            self.sdk_available = False

    def log_result(self, command, success, duration, message="", robot_state=""):
        """Record test result and state"""
        result = {
            'command': command,
            'success': success,
            'duration': duration,
            'timestamp': datetime.now().strftime('%H:%M:%S'),
            'message': message,
            'robot_state': robot_state
        }
        self.results.append(result)
        status = "PASS" if success else "FAIL"
        state_info = f" [state: {robot_state}]" if robot_state else ""
        print(f"[{status}] {command}: {duration:.3f}s - {message}{state_info}")

    def update_robot_state(self, action_api, success):
        """Update robot state based on execution result"""
        if not success:
            return

        state_mapping = {
            1009: "sitting",      # Sit
            1004: "standing",     # StandUp
            1005: "lying",        # StandDown
            1016: "standing",     # Hello (maintains standing)
            1017: "standing",     # Stretch (maintains standing)
            1006: "standing",     # RecoveryStand
        }

        if action_api in state_mapping:
            old_state = self.current_robot_state
            self.current_robot_state = state_mapping[action_api]
            print(f"Robot state updated: {old_state} -> {self.current_robot_state}")

    def predict_command_conflict(self, intent, target_api):
        """Predict whether a command will conflict with the current state"""
        conflict_rules = {
            # API code: [list of current states that would conflict]
            1009: ["sitting"],       # Sit command conflicts when already sitting
            1004: ["standing"],      # StandUp command may conflict when already standing
            1005: ["lying"],         # StandDown command conflicts when already lying down
        }

        if target_api in conflict_rules:
            if self.current_robot_state in conflict_rules[target_api]:
                return True, f"State conflict: robot is already in {self.current_robot_state} state"

        return False, ""

    def safety_confirmation(self):
        """Safety confirmation"""
        print("\n" + "="*60)
        print("SAFETY NOTICE - State-Aware LLM Robot Control Test")
        print("="*60)
        print("This will test state conflict detection and resolution!")
        print("WARNING: Please ensure the area around the robot is safe!")
        print("\nOptimized test sequence (avoids state conflicts):")
        print('   1. {"intent": "robot_control"} -> Sit')
        print('   2. {"intent": "stand"} -> Stand (different state)')
        print('   3. {"intent": "hello"} -> Wave (maintains standing)')
        print('   4. {"intent": "お座り"} -> Sit (Japanese)')
        print('   5. Verify state conflict detection')
        print("="*60)

        response = input("Confirm to continue state-aware test? (yes/no): ").lower().strip()
        return response in ['yes', 'y']

    async def run_state_aware_test(self):
        """Run state-aware test"""
        if not self.safety_confirmation():
            print("Test cancelled")
            return

        if not self.sdk_available or not self.engine_class:
            print("SDK unavailable or mapping engine cannot be loaded")
            return

        print(f"\nStarting state-aware LLM robot control test - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

        engine = self.engine_class()
        print("Real robot mapping engine initialized successfully")

        # Optimized test sequence - avoids consecutive same-state commands
        test_commands = [
            ('robot_control', '{"intent": "robot_control", "confidence": 0.8}', 1009),  # Sit
            ('english_stand', '{"intent": "stand", "confidence": 0.8}', 1004),          # Stand
            ('hello_wave', '{"intent": "hello", "confidence": 0.8}', 1016),             # Wave
            ('japanese_sit', '{"intent": "お座り", "confidence": 0.9}', 1009),          # Japanese sit
            ('conflict_test', '{"intent": "robot_control", "confidence": 0.8}', 1009),  # Intentional repeat test
        ]

        print(f"\nExecuting state-aware test sequence ({len(test_commands)} commands):")
        print("-" * 60)

        for i, (cmd_name, llm_output, expected_api) in enumerate(test_commands, 1):
            print(f"\n[{i}/{len(test_commands)}] Current state: {self.current_robot_state}")
            print(f"LLM command: {llm_output}")

            # Predict conflict
            will_conflict, conflict_reason = self.predict_command_conflict(llm_output, expected_api)
            if will_conflict:
                print(f"WARNING: Predicted conflict: {conflict_reason}")

            start_time = time.time()
            try:
                result = await engine.map_intent_to_action(llm_output)
                duration = time.time() - start_time

                # Update state
                if result.success:
                    self.update_robot_state(result.action_code, True)
                    self.log_result(cmd_name, True, duration,
                                  f"LLM->robot success: {result.message}",
                                  self.current_robot_state)
                else:
                    conflict_detected = "state conflict" in str(result.error_message).lower() or result.robot_response == -1
                    conflict_msg = " (state conflict detected)" if conflict_detected else ""

                    self.log_result(cmd_name, False, duration,
                                  f"LLM->robot failed: {result.error_message}{conflict_msg}",
                                  self.current_robot_state)

                # Verify prediction accuracy
                if will_conflict and not result.success:
                    print("State conflict prediction was accurate!")
                elif will_conflict and result.success:
                    print("Predicted conflict but execution succeeded, possible state judgment error")

                # Safety interval
                await asyncio.sleep(2)

            except Exception as e:
                duration = time.time() - start_time
                self.log_result(cmd_name, False, duration, f"Execution exception: {str(e)}")

        self.generate_report()

    def generate_report(self):
        """Generate detailed report"""
        print("\n" + "="*60)
        print("State-Aware LLM Robot Control Test Report")
        print("="*60)

        success_count = sum(1 for r in self.results if r['success'])
        total_count = len(self.results)
        success_rate = (success_count / total_count * 100) if total_count > 0 else 0

        print(f"Overall result: {success_count}/{total_count} passed ({success_rate:.1f}%)")
        print(f"Test time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

        print(f"\nFinal robot state: {self.current_robot_state}")

        print("\nDetailed results:")
        print("-" * 60)
        for result in self.results:
            status = "PASS" if result['success'] else "FAIL"
            state_info = f" -> {result['robot_state']}" if result['robot_state'] else ""
            print(f"[{status}] {result['timestamp']} | {result['command']:<15} | "
                  f"{result['duration']:>6.3f}s | {result['message']}{state_info}")

        # State conflict analysis
        conflict_failures = [r for r in self.results if not r['success'] and "conflict" in r['message'].lower()]

        print(f"\nState management analysis:")
        print(f"   State conflict failures: {len(conflict_failures)}")
        print(f"   State tracking accuracy: Automatically updated based on execution results")

        if len(conflict_failures) > 0:
            print("PASS: Successfully detected and handled state conflicts!")
            print("PASS: State-aware mechanism is working correctly!")

        print("="*60)

async def main():
    """Main function"""
    print("State-Aware LLM Robot Control Test")
    print(f"Start time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

    if not setup_cyclonedds_environment():
        print("Environment configuration failed, cannot continue")
        return

    tester = StateAwareLLMTest()
    await tester.run_state_aware_test()

if __name__ == "__main__":
    asyncio.run(main())
