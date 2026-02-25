#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Complete test of all declared action methods in Unitree Go2 SDK
Confirms which actions are truly available on Go2 hardware
"""

import os
import sys
import time
import json

# Add paths
_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.append(_PROJECT_ROOT)
sys.path.append(os.path.join(_PROJECT_ROOT, 'unitree_sdk2_python'))

# Set environment variables
os.environ['CYCLONEDDS_HOME'] = os.path.join(_PROJECT_ROOT, 'cyclonedds', 'install')
ld_path = os.environ.get('LD_LIBRARY_PATH', '')
cyclone_lib = os.path.join(_PROJECT_ROOT, 'cyclonedds', 'install', 'lib')
unitree_lib = os.path.join(_PROJECT_ROOT, 'cyclonedds_ws', 'install', 'unitree_sdk2', 'lib')
if cyclone_lib not in ld_path:
    os.environ['LD_LIBRARY_PATH'] = f"{cyclone_lib}:{unitree_lib}:{ld_path}"
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
os.environ['CYCLONEDDS_URI'] = '''<CycloneDDS><Domain><General><Interfaces>
                        <NetworkInterface name="eth0" priority="default" multicast="default" />
                    </Interfaces></General></Domain></CycloneDDS>'''

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

# Define all action methods declared in the SDK
ALL_ACTIONS = [
    # Basic control actions (no parameters)
    (1001, "Damp", None, "Damping"),
    (1002, "BalanceStand", None, "Balance stand"),
    (1003, "StopMove", None, "Stop movement"),
    (1004, "StandUp", None, "Stand up"),
    (1005, "StandDown", None, "Lie down"),
    (1006, "RecoveryStand", None, "Recovery stand"),
    (1009, "Sit", None, "Sit"),
    (1010, "RiseSit", None, "Rise from sit"),
    (1012, "Trigger", None, "Trigger"),

    # Performance actions
    (1016, "Hello", None, "Hello gesture"),
    (1017, "Stretch", None, "Stretch"),
    (1021, "Wallow", None, "Wallow/roll"),
    (1022, "Dance1", None, "Dance 1"),
    (1023, "Dance2", None, "Dance 2"),

    # Advanced actions
    (1029, "Scrape", None, "Scrape"),
    (1030, "FrontFlip", None, "Front flip"),
    (1031, "FrontJump", None, "Front jump"),
    (1032, "FrontPounce", None, "Front pounce"),
    (1033, "WiggleHips", None, "Wiggle hips"),
    (1036, "Heart", None, "Heart gesture"),

    # Advanced actions (may not be supported)
    (1042, "LeftFlip", None, "Left flip"),
    (1044, "BackFlip", None, "Back flip"),

    # Parameterized actions (require parameters)
    (1007, "Euler", (0.0, 0.0, 0.0), "Euler angles"),
    (1008, "Move", (0.0, 0.0, 0.0), "Move"),
    (1011, "SwitchGait", (0,), "Switch gait"),
    (1013, "BodyHeight", (0.0,), "Body height"),
    (1014, "FootRaiseHeight", (0.0,), "Foot raise height"),
    (1015, "SpeedLevel", (0,), "Speed level"),
    (1019, "ContinuousGait", (1,), "Continuous gait"),
    (1027, "SwitchJoystick", (True,), "Switch joystick"),
    (1028, "Pose", (True,), "Pose"),
    (1035, "EconomicGait", (True,), "Economic gait"),
    (1045, "FreeWalk", (True,), "Free walk"),
    (1046, "FreeBound", (True,), "Free bound"),
    (1047, "FreeJump", (True,), "Free jump"),
    (1048, "FreeAvoid", (True,), "Free avoid"),
    (1049, "WalkStair", (True,), "Walk stairs"),
    (1050, "WalkUpright", (True,), "Walk upright"),
    (1051, "CrossStep", (True,), "Cross step"),
]

def test_all_actions():
    """Test all SDK actions"""

    print("="*80)
    print("Unitree Go2 SDK Complete Action Test")
    print("="*80)

    # Initialize
    print("\nInitializing DDS channel...")
    ChannelFactoryInitialize(0, "eth0")

    client = SportClient()
    client.SetTimeout(10.0)
    client.Init()

    # Test connection
    print("Testing connection...")
    test_result = client.RecoveryStand()
    if test_result == 0:
        print("Connection successful")
    elif test_result == 3103:
        print("App is occupying the robot, please close the app and restart the robot")
        return
    else:
        print(f"Connection test return code: {test_result}")

    time.sleep(1)

    # Statistics
    results = {
        "supported": [],
        "unsupported_3203": [],
        "unsupported_3104": [],
        "error_other": [],
        "not_found": []
    }

    print("\n" + "="*80)
    print("Starting to test all actions...")
    print("="*80)

    for api_id, method_name, params, description in ALL_ACTIONS:
        print(f"\nTest {api_id:4d} | {method_name:20s} | {description:20s}", end=" ")

        # Check if method exists
        if not hasattr(client, method_name):
            print(f"FAIL: Method not found")
            results["not_found"].append((api_id, method_name, description))
            continue

        # Get method
        method = getattr(client, method_name)

        try:
            # Call method (with or without parameters)
            if params is None:
                result = method()
            else:
                result = method(*params)

            # Analyze return code
            if result == 0:
                print(f"PASS: Success (0)")
                results["supported"].append((api_id, method_name, description))
            elif result == -1:
                print(f"PASS: Already in state (-1)")
                results["supported"].append((api_id, method_name, description))
            elif result == 3203:
                print(f"FAIL: Not implemented (3203)")
                results["unsupported_3203"].append((api_id, method_name, description))
            elif result == 3104:
                print(f"WARN: Special return (3104)")
                results["unsupported_3104"].append((api_id, method_name, description))
            else:
                print(f"UNKNOWN: Return code ({result})")
                results["error_other"].append((api_id, method_name, description, result))

        except Exception as e:
            print(f"ERROR: Exception: {e}")
            results["error_other"].append((api_id, method_name, description, str(e)))

        # Brief delay to avoid command conflicts
        time.sleep(0.5)

    # Print summary
    print("\n" + "="*80)
    print("Test Results Summary")
    print("="*80)

    print(f"\n**Supported actions** ({len(results['supported'])}):")
    for api_id, method, desc in results['supported']:
        print(f"   {api_id:4d} | {method:20s} | {desc}")

    print(f"\n**Not implemented (3203)** ({len(results['unsupported_3203'])}):")
    for api_id, method, desc in results['unsupported_3203']:
        print(f"   {api_id:4d} | {method:20s} | {desc}")

    print(f"\n**Special return (3104)** ({len(results['unsupported_3104'])}):")
    for api_id, method, desc in results['unsupported_3104']:
        print(f"   {api_id:4d} | {method:20s} | {desc}")

    print(f"\n**Method not found** ({len(results['not_found'])}):")
    for api_id, method, desc in results['not_found']:
        print(f"   {api_id:4d} | {method:20s} | {desc}")

    if results['error_other']:
        print(f"\n**Other errors** ({len(results['error_other'])}):")
        for item in results['error_other']:
            print(f"   {item}")

    # Save results
    result_file = os.path.join(_PROJECT_ROOT, f"test_results_{int(time.time())}.json")
    with open(result_file, 'w', encoding='utf-8') as f:
        json.dump({
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "results": {
                "supported": [(a, m, d) for a, m, d in results['supported']],
                "unsupported_3203": [(a, m, d) for a, m, d in results['unsupported_3203']],
                "unsupported_3104": [(a, m, d) for a, m, d in results['unsupported_3104']],
                "not_found": [(a, m, d) for a, m, d in results['not_found']],
                "error_other": results['error_other']
            },
            "summary": {
                "total_tested": len(ALL_ACTIONS),
                "supported": len(results['supported']),
                "unsupported": len(results['unsupported_3203']) + len(results['unsupported_3104']),
                "not_found": len(results['not_found'])
            }
        }, f, indent=2, ensure_ascii=False)

    print(f"\nResults saved to: {result_file}")

    # Final statistics
    print("\n" + "="*80)
    print("Final Statistics")
    print("="*80)
    print(f"Total actions tested: {len(ALL_ACTIONS)}")
    print(f"Supported: {len(results['supported'])}")
    print(f"Not supported (3203): {len(results['unsupported_3203'])}")
    print(f"Special (3104): {len(results['unsupported_3104'])}")
    print(f"Method not found: {len(results['not_found'])}")
    print("="*80)

if __name__ == "__main__":
    test_all_actions()
