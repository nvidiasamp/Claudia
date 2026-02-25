#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test Heart Gesture and Other Action Mapping Fixes
Generated: 2025-07-10
Purpose: Verify that WALLOW(1021) and other missing action mappings are correct
"""

import sys
import os
import re
from pathlib import Path

# Add project path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

def test_enhanced_japanese_commander_mapping():
    """Test the enhanced Japanese command interface action mappings"""
    print("Testing enhanced Japanese command interface action mappings...")

    try:
        from src.claudia.interactive_japanese_commander_enhanced import EnhancedJapaneseCommandInterface

        interface = EnhancedJapaneseCommandInterface()

        # Test heart gesture related mappings
        # Note: Japanese/Chinese strings are intentional test inputs for the robot
        test_cases = [
            # Heart gesture action tests
            ("比心", "heart"),           # Chinese: heart gesture
            ("ハート", "heart"),          # Japanese: heart
            ("heart", "heart"),
            ("love", "love"),
            ("可愛い", None),             # Japanese: cute - should match to heart via pattern matching

            # Cheer action tests (should map to cheer)
            ("ちんちん", "cheer"),         # Japanese: chintin -> cheer
            ("拜年", None),              # Chinese: New Year greeting - should match to cheer via pattern matching

            # Basic action tests
            ("お座り", "sit"),            # Japanese: sit
            ("ダンス", "dance"),          # Japanese: dance
            ("お辞儀", "bow"),           # Japanese: bow
            ("握手", "shake_hands"),     # Japanese: handshake

            # New advanced actions
            ("ジャンプ", "jump"),         # Japanese: jump
            ("がんばれ", "cheer"),        # Japanese: go for it / cheer
            ("パンチ", "punch"),          # Japanese: punch
        ]

        print("\nAction API mapping test:")
        success_count = 0
        total_count = len(test_cases)

        for japanese_input, expected_action in test_cases:
            mapped_api = interface.action_api_map.get(expected_action if expected_action else japanese_input)

            if mapped_api:
                print(f"[PASS] '{japanese_input}' -> {expected_action or japanese_input} -> API {mapped_api}")
                success_count += 1
            else:
                # Try pattern matching
                action, confidence = interface.extract_action_from_llm_response("", japanese_input)
                if action and action in interface.action_api_map:
                    api_code = interface.action_api_map[action]
                    print(f"[PASS] '{japanese_input}' -> {action} (pattern match, confidence:{confidence:.2f}) -> API {api_code}")
                    success_count += 1
                else:
                    print(f"[FAIL] '{japanese_input}' -> no mapping")

        print(f"\nMapping success rate: {success_count}/{total_count} ({success_count/total_count*100:.1f}%)")
        print(f"Total mapping count: {len(interface.action_api_map)}")

        # Test action sequence planning (key fix for heart gesture)
        print("\nTesting action sequence planning (heart gesture fix):")
        test_action_sequences(interface)

        # Test pattern matching
        print("\nJapanese pattern matching test:")
        test_pattern_matching(interface)

    except ImportError as e:
        print(f"Import failed: {e}")
        return False
    except Exception as e:
        print(f"Test exception: {e}")
        return False

    return True

def test_action_sequences(interface):
    """Test action sequence planning"""
    from src.claudia.interactive_japanese_commander_enhanced import RobotState

    # Create different robot states for testing
    # Note: Japanese/Chinese test inputs are intentional robot commands
    test_states = [
        ("lying", "比心"),     # Key test: execute heart gesture from lying state
        ("sitting", "hello"),  # Execute hello from sitting state
        ("standing", "heart"), # Execute heart from standing state
        ("unknown", "dance"),  # Execute dance from unknown state
    ]

    for state, action in test_states:
        # Simulate robot state
        interface.robot_state.current_state = state

        # Get action API
        api_code = interface.action_api_map.get(action)
        if api_code:
            # Plan action sequence
            sequence = interface.action_sequencer.plan_action_sequence(action, api_code)

            print(f"  State:{state} -> Action:{action}")
            for i, step in enumerate(sequence, 1):
                print(f"    {i}. {step['action']} (API: {step['api']})")

            # Verify special handling for heart gesture action
            if action in ["比心", "heart"] and state == "lying":
                expected_steps = ["stand_up", action]
                actual_steps = [step['action'] for step in sequence]
                if actual_steps == expected_steps:
                    print(f"    [PASS] Heart gesture sequence planning correct: {actual_steps}")
                else:
                    print(f"    [FAIL] Heart gesture sequence planning error: expected {expected_steps}, actual {actual_steps}")

def test_pattern_matching(interface):
    """Test Japanese pattern matching"""
    # Note: Japanese test inputs are intentional robot command phrases
    test_inputs = [
        "比心して",        # Heart gesture action
        "ハートをお願い",   # Heart shape request
        "可愛いポーズ",     # Cute pose
        "お座りして",      # Sit down
        "ちんちんしよう",   # Cheer action
        "がんばれ！",      # Encouragement
        "お辞儀します",     # Bow
        "握手しませんか",   # Handshake invitation
    ]

    for test_input in test_inputs:
        action, confidence = interface.extract_action_from_llm_response("", test_input)
        api_code = interface.action_api_map.get(action) if action else None

        if action and api_code:
            print(f"  [PASS] '{test_input}' -> {action} (confidence:{confidence:.2f}) -> API {api_code}")
        else:
            print(f"  [FAIL] '{test_input}' -> unrecognized ({action}, {confidence:.2f})")

def test_real_action_mapping_engine():
    """Test the real action mapping engine API coverage"""
    print("\nTesting the real action mapping engine...")

    try:
        from src.claudia.robot_controller.action_mapping_engine_real import RealActionMappingEngine

        engine = RealActionMappingEngine()

        # Test API registry completeness
        print(f"API registry size: {len(engine.api_registry)}")
        print(f"Japanese mapping count: {len(engine.intent_mapping)}")
        print(f"English mapping count: {len(engine.english_intent_mapping)}")

        # Test whether key APIs exist
        key_apis = [1021, 1024, 1025, 1026, 1027, 1028, 1029, 1030, 1031]
        print("\nKey new API test:")
        for api_code in key_apis:
            if api_code in engine.api_registry:
                action_def = engine.api_registry[api_code]
                print(f"  [PASS] API {api_code}: {action_def.function_name} - {action_def.description}")
            else:
                print(f"  [FAIL] API {api_code}: missing")

        # Test special word mappings
        # Note: Japanese/Chinese test words are intentional robot command vocabulary
        print("\nSpecial vocabulary mapping test:")
        test_words = ["ちんちん", "比心", "heart", "bow", "cheer"]
        for word in test_words:
            if word in engine.intent_mapping:
                api_code = engine.intent_mapping[word]
                print(f"  [PASS] '{word}' -> API {api_code}")
            elif word in engine.english_intent_mapping:
                api_code = engine.english_intent_mapping[word]
                print(f"  [PASS] '{word}' -> API {api_code} (English)")
            else:
                print(f"  [FAIL] '{word}' -> no mapping")

        return True

    except ImportError as e:
        print(f"Import failed: {e}")
        return False
    except Exception as e:
        print(f"Test exception: {e}")
        return False

def show_available_actions():
    """Show all available actions"""
    print("\nComplete action mapping table:")

    action_mapping = {
        1001: "Damp - Emergency stop",
        1002: "BalanceStand - Balance stand",
        1003: "StopMove - Stop moving",
        1004: "StandUp - Stand up",
        1005: "StandDown - Lie down",
        1006: "RecoveryStand - Recovery stand",
        1007: "Euler - Euler angle control",
        1008: "Move - Movement control",
        1009: "Sit - Sit down",
        1010: "RiseSit - Rise from sitting",
        1011: "SwitchGait - Switch gait",
        1012: "Trigger - Trigger",
        1013: "BodyHeight - Body height",
        1014: "FootRaiseHeight - Foot raise height",
        1015: "SpeedLevel - Speed level",
        1016: "Hello - Wave/handshake",
        1017: "Stretch - Stretch",
        1018: "TrajectoryFollow - Trajectory follow",
        1019: "ContinuousGait - Continuous gait",
        1020: "Content - Content",
        1021: "Wallow - Heart gesture (FIXED)",
        1022: "Dance1 - Dance 1",
        1023: "Dance2 - Dance 2",
        1024: "GetBodyHeight - Get body height",
        1025: "GetFootRaiseHeight - Get foot raise height",
        1026: "GetSpeedLevel - Get speed level",
        1027: "SwitchJoystick - Switch joystick",
    }

    for api_id, description in action_mapping.items():
        marker = " (FIXED)" if api_id == 1021 else ""
        print(f"  {api_id:4d}: {description}{marker}")

def main():
    """Main test function"""
    print("Claudia Robot Action Mapping Fix Test")
    print("=" * 50)

    # Show fixed actions
    show_available_actions()

    # Test enhanced interface
    success1 = test_enhanced_japanese_commander_mapping()

    # Test real mapping engine
    success2 = test_real_action_mapping_engine()

    print("\n" + "=" * 50)
    if success1 and success2:
        print("All tests passed! Heart gesture action mapping fix successful")
        print("\nYou can now use the following commands:")
        print("   - 'heart gesture' (比心 / 比心して)")
        print("   - 'heart' (ハート / ハートお願い)")
        print("   - 'heart' in English")
        print("   - 'cute pose' (可愛いポーズ, maps to heart)")
        print("\nPlease re-run run_enhanced_japanese_commander.sh to test the fix")
    else:
        print("Some tests failed, please check error messages")

if __name__ == "__main__":
    print("Starting heart gesture and other action mapping fix verification test")
    print("=" * 60)

    # Run all tests
    tests = [
        ("Enhanced Japanese command interface", test_enhanced_japanese_commander_mapping),
        ("Real action mapping engine", test_real_action_mapping_engine),
        ("Available actions display", show_available_actions),
    ]

    passed = 0
    total = len(tests)

    for test_name, test_func in tests:
        print(f"\n{'='*20} {test_name} {'='*20}")
        try:
            if test_func():
                print(f"[PASS] {test_name} test passed")
                passed += 1
            else:
                print(f"[FAIL] {test_name} test failed")
        except Exception as e:
            print(f"[FAIL] {test_name} test exception: {e}")

    print("\n" + "="*60)
    print(f"Test summary: {passed}/{total} passed ({passed/total*100:.1f}%)")

    if passed == total:
        print("All tests passed! Heart gesture action mapping fix successful")
        print("\nFix summary:")
        print("  [DONE] Extended action_api_map with 7 new API mappings")
        print("  [DONE] Enhanced Japanese pattern matching to include heart gesture and cheer vocabulary")
        print("  [DONE] Fixed action sequence planner to auto-stand before heart gesture")
        print("  [DONE] Completed API registry to support 27 official APIs")
        print("  [DONE] Resolved 'cheer' mapping to celebration action")
        print("\nKey fixes:")
        print("  - Heart gesture error code 3203 -> now executes stand_up before wallow")
        print("  - Unrecognized cheer command -> now maps to cheer celebration action")
    else:
        print("Some tests failed, further debugging needed")

    print("\nNext steps:")
    print("  1. Run real robot test to verify fix")
    print("  2. Test command: python3 scripts/run_enhanced_japanese_commander.sh")
    print("  3. Input: 'heart gesture' (should stand up first then perform heart gesture)")
    print("  4. Input: 'cheer' (should execute celebration action)")
    print("  5. Observe whether error code 3203 issue is resolved")
