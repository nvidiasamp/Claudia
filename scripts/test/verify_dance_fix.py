#!/usr/bin/env python3
"""
Verify Dance Return Code Fix
Test whether return codes like 3104 are now correctly recognized as success
"""

import sys
from pathlib import Path

# Add project path
sys.path.append(str(Path(__file__).parent.parent.parent))

def test_return_code_logic():
    """Test the new return code judgment logic"""

    print("Testing return code judgment logic fix")
    print("=" * 40)

    # Import the fixed class
    from src.claudia.robot_controller.action_mapping_engine_real import RealRobotController

    # Create controller instance (no real connection needed)
    controller = RealRobotController()

    # Test cases
    test_cases = [
        # (return_code, method_name, expected_result, description)
        (0, "Sit", True, "Traditional success code"),
        (3104, "Dance1", True, "Dance1 completion code (would fail before fix)"),
        (3105, "Dance2", True, "Dance2 completion code"),
        (3106, "Hello", True, "Other completion code"),
        (1, "Sit", False, "Actual error code"),
        (999, "Dance1", False, "Unknown error code"),
    ]

    print(f"{'Code':<8} {'Method':<10} {'Expected':<10} {'Actual':<10} {'Status':<8} {'Description'}")
    print("-" * 60)

    all_passed = True

    for return_code, method_name, expected, description in test_cases:
        actual = controller._is_command_successful(return_code, method_name)

        status = "PASS" if actual == expected else "FAIL"
        if actual != expected:
            all_passed = False

        print(f"{return_code:<8} {method_name:<10} {expected!s:<10} {actual!s:<10} {status:<8} {description}")

    print("-" * 60)
    print(f"Overall result: {'All passed' if all_passed else 'Some failures detected'}")

    if all_passed:
        print("\nFix successful! Dance command return code 3104 is now correctly recognized as success")
        print("This means dance actions will no longer be falsely reported as errors")
    else:
        print("\nFix may have issues, please check the code")

    return all_passed

if __name__ == "__main__":
    test_return_code_logic()
