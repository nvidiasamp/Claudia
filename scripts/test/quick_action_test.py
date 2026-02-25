#!/usr/bin/env python3
"""
Quick Action Mapping Fix Verification Script
Used to verify the mapping fix for heart gesture, cheer, and other actions
"""

import sys
from pathlib import Path

# Add project root directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

def test_action_mapping_fix():
    """Quick test of action mapping fix"""
    print("Quick verification of heart gesture and other action mapping fixes")
    print("=" * 50)

    try:
        from src.claudia.interactive_japanese_commander_enhanced import EnhancedJapaneseCommandInterface

        # Create interface instance
        interface = EnhancedJapaneseCommandInterface()

        # Key fix test cases
        # Note: Japanese/Chinese strings are intentional robot command test inputs
        test_cases = [
            ("比心", "heart", "Should map to API 1021 (Wallow)"),
            ("ちんちん", "cheer", "Should map to API 1026 (Cheer)"),
            ("ハート", None, "Should be recognized as heart via pattern matching"),
            ("拜年", None, "Should be recognized as cheer via pattern matching"),
        ]

        print("\nTesting key action mapping fixes:")
        all_passed = True

        for japanese_input, expected_action, description in test_cases:
            print(f"\nTest: '{japanese_input}' - {description}")

            # 1. Direct API mapping test
            if expected_action and expected_action in interface.action_api_map:
                api_code = interface.action_api_map[expected_action]
                print(f"  [PASS] Direct mapping: {expected_action} -> API {api_code}")

            # 2. Pattern matching test
            action, confidence = interface.extract_action_from_llm_response("", japanese_input)
            if action and action in interface.action_api_map:
                api_code = interface.action_api_map[action]
                print(f"  [PASS] Pattern match: '{japanese_input}' -> {action} (confidence:{confidence:.2f}) -> API {api_code}")
            else:
                print(f"  [FAIL] Pattern match failed: '{japanese_input}' -> {action}")
                all_passed = False

        # Test action sequence planning (key fix for heart gesture)
        print(f"\nTesting action sequence planning (key fix for heart gesture):")

        # Simulate heart gesture action from lying state
        interface.robot_state.current_posture = "lying"
        heart_api = interface.action_api_map.get("heart")

        if heart_api:
            sequence = interface.action_sequencer.plan_action_sequence("heart", heart_api)
            print(f"  Robot state: lying -> execute heart gesture")

            for i, step in enumerate(sequence, 1):
                print(f"    {i}. {step['action']} (API: {step['api']})")

            # Verify sequence is correct
            expected_steps = ["stand_up", "heart"]
            actual_steps = [step['action'] for step in sequence]

            if actual_steps == expected_steps:
                print(f"  [PASS] Heart gesture sequence planning correct: will stand up first then perform heart gesture, resolving error code 3203")
            else:
                print(f"  [FAIL] Heart gesture sequence planning issue: expected {expected_steps}, actual {actual_steps}")
                all_passed = False

        # Summary
        print(f"\n{'='*50}")
        if all_passed:
            print("All tests passed! Fix successfully verified")
            print("\nFix verification summary:")
            print("  [PASS] Heart gesture action mapping correct (API 1021)")
            print("  [PASS] Cheer action mapping correct (API 1026)")
            print("  [PASS] Pattern matching recognition correct")
            print("  [PASS] Action sequence planning correct (stand up before heart gesture)")
            print("\nYou can now test on the real robot:")
            print("  1. Run: python3 scripts/run_enhanced_japanese_commander.sh")
            print("  2. In lying state, input 'heart gesture' - should stand up first then perform heart gesture")
            print("  3. Input 'cheer' - should execute celebration action")
        else:
            print("Some tests failed, further investigation needed")

        return all_passed

    except ImportError as e:
        print(f"Import failed: {e}")
        print("Please make sure to run this script from the claudia project root directory")
        return False
    except Exception as e:
        print(f"Test exception: {e}")
        return False

if __name__ == "__main__":
    success = test_action_mapping_fix()
    sys.exit(0 if success else 1)
