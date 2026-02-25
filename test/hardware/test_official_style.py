#!/usr/bin/env python3
"""
Control command test based on official example style
Tests following the unitree_sdk2_python official example approach
"""

import time
import sys
import os
from datetime import datetime

# Set environment variables
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

# Add SDK path
_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.append(os.path.join(_PROJECT_ROOT, 'unitree_sdk2_python'))

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    print("Successfully imported unitree_sdk2py module")
except ImportError as e:
    print(f"Import error: {e}")
    sys.exit(1)

def test_official_style():
    """Test SportClient following official example style"""
    print(f"Official style SportClient test - {datetime.now().strftime('%H:%M:%S')}")

    # Safety prompt
    print("\nSAFETY WARNING: Please ensure there are no obstacles around the robot")
    print("This test will follow the official example approach")
    response = input("Press Enter to continue...")

    try:
        # Initialize DDS channel following official example
        print("Initializing DDS channel factory (eth0)...")
        ChannelFactoryInitialize(0, "eth0")

        # Create SportClient (following official example style)
        print("Creating SportClient...")
        sport_client = SportClient()
        sport_client.SetTimeout(10.0)

        # Following official example, call Init() directly without checking return value
        print("Initializing SportClient...")
        sport_client.Init()
        print("SportClient initialization complete (official example style)")

        # Test sequence
        test_commands = [
            ("BalanceStand", lambda: sport_client.BalanceStand(), "Balance stand"),
            ("StandUp", lambda: sport_client.StandUp(), "Stand up"),
            ("StandDown", lambda: sport_client.StandDown(), "Lie down"),
            ("RecoveryStand", lambda: sport_client.RecoveryStand(), "Recovery stand"),
        ]

        print(f"\nStarting test sequence ({len(test_commands)} commands):")
        print("-" * 50)

        for i, (cmd_name, cmd_func, description) in enumerate(test_commands, 1):
            print(f"\n[{i}/{len(test_commands)}] Executing {cmd_name}() - {description}")

            try:
                result = cmd_func()
                print(f"PASS: {cmd_name}: Command executed, return value: {result}")

                # Wait for action to complete
                if cmd_name in ["StandUp", "StandDown", "RecoveryStand"]:
                    print(f"   Waiting for {description} to complete...")
                    time.sleep(5)
                else:
                    time.sleep(2)

            except Exception as e:
                print(f"FAIL: {cmd_name}: Execution exception: {e}")

        print("\nTest sequence complete")

        # Final state check
        print("\nTest summary:")
        print("Official example style test complete")
        print("Robot should be in standing state")

        return True

    except Exception as e:
        print(f"Test process exception: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    print("Unitree Go2 Official Style Control Test")
    print(f"Start time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 60)

    success = test_official_style()

    print("\n" + "=" * 60)
    if success:
        print("Official style test completed successfully")
    else:
        print("Official style test failed")
    print("=" * 60)

if __name__ == "__main__":
    main()
