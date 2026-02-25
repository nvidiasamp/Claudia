#!/usr/bin/env python3
"""
Unitree Dance Command Return Code Test
Verify the meaning of Dance1/Dance2 return codes

Purpose: Determine whether return codes like 3104 are normal completion status codes
"""

import sys
import time
import os

# Set up environment and paths
_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.append(os.path.join(_PROJECT_ROOT, 'unitree_sdk2_python'))
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

def test_dance_return_codes():
    """Specifically test the meaning of dance action return codes"""

    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.sport.sport_client import SportClient

        print("Dance Return Code Dedicated Test")
        print("=" * 50)

        # Initialize
        ChannelFactoryInitialize(0, "eth0")
        client = SportClient()
        client.SetTimeout(10.0)
        client.Init()

        print("SportClient initialization successful\n")

        # Test return codes for various commands
        test_commands = [
            ("Sit", client.Sit, "Sit down"),
            ("StandUp", client.StandUp, "Stand up"),
            ("Hello", client.Hello, "Wave"),
            ("Dance1", client.Dance1, "Dance 1"),
            ("Dance2", client.Dance2, "Dance 2"),
            ("Stretch", client.Stretch, "Stretch"),
        ]

        return_codes = {}

        for cmd_name, cmd_func, description in test_commands:
            print(f"Testing {cmd_name}() - {description}")

            try:
                start_time = time.time()
                result = cmd_func()
                duration = time.time() - start_time

                return_codes[cmd_name] = result

                print(f"   Return code: {result}")
                print(f"   Execution time: {duration:.3f}s")

                # Wait different amounts depending on command type
                if "Dance" in cmd_name:
                    print(f"   Waiting for dance action to complete...")
                    time.sleep(8)  # Dance actions need more time
                elif cmd_name in ["StandUp", "Hello", "Stretch"]:
                    print(f"   Waiting for action to complete...")
                    time.sleep(4)
                else:
                    time.sleep(2)

                print(f"   Status: {'Traditional success' if result == 0 else 'Needs verification'}\n")

            except Exception as e:
                print(f"   Execution exception: {e}\n")
                return_codes[cmd_name] = f"Exception: {e}"

        # Analyze return code patterns
        print("Return Code Analysis Report")
        print("=" * 50)

        print(f"{'Command':<12} {'Code':<8} {'Possible Meaning'}")
        print("-" * 40)

        for cmd_name, code in return_codes.items():
            if isinstance(code, int):
                if code == 0:
                    meaning = "Standard success"
                elif code in [3104, 3105, 3106]:  # Common completion status codes
                    meaning = "Likely a completion status code"
                elif code > 3000:
                    meaning = "High-value status code (possibly normal)"
                else:
                    meaning = "Needs further verification"

                print(f"{cmd_name:<12} {code:<8} {meaning}")
            else:
                print(f"{cmd_name:<12} {'ERROR':<8} {code}")

        # Provide recommendations
        print("\nSuggested success judgment logic modifications:")
        successful_codes = [0]  # Default success

        # Analyze which high return codes might indicate success
        for cmd_name, code in return_codes.items():
            if isinstance(code, int) and code > 0:
                if "Dance" in cmd_name and code == 3104:
                    successful_codes.append(3104)
                    print(f"   - 3104: Dance1 completion status code (based on user observation)")
                elif code in [3105, 3106, 3107]:  # Other possible completion codes
                    print(f"   - {code}: Possible completion status code for {cmd_name}")

        print(f"\nSuggested success return code list: {sorted(set(successful_codes))}")

        return return_codes

    except Exception as e:
        print(f"Test failed: {e}")
        import traceback
        traceback.print_exc()
        return None

if __name__ == "__main__":
    print("Please confirm the robot is connected and in a safe environment")
    input("Press Enter to start the test...")

    results = test_dance_return_codes()

    if results:
        print("\nTest complete! Please observe the robot's actual action execution")
        print("If Dance1 executed the correct action but returned a non-zero code, that code is a normal completion status")
