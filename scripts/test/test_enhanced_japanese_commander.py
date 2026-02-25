#!/usr/bin/env python3
"""
Enhanced Japanese Command Interface Test Script

Tests LLM integration, robot state management, and intelligent action sequencing
"""

import sys
import asyncio
from pathlib import Path

# Add project root directory to Python path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.claudia.interactive_japanese_commander_enhanced import EnhancedJapaneseCommandInterface

async def test_enhanced_japanese_commander():
    """Test the main features of the enhanced Japanese command interface"""

    print("Enhanced Japanese Command Interface Test Start")
    print("=" * 50)

    # Create interface instance
    interface = EnhancedJapaneseCommandInterface()

    # Initialize system
    print("\nInitializing system...")
    init_success = await interface.initialize()

    if not init_success:
        print("Initialization failed, test terminated")
        return

    print("System initialization successful")

    # Test cases (Japanese robot commands with English descriptions)
    test_commands = [
        "こんにちは、お座りしてください",  # Greeting + sit command
        "立ち上がってからダンスして",      # Stand up + dance combo command
        "ストレッチしてください",           # Stretch command
        "機器人の状態を確認して",           # Status query
        "緊急停止！",                     # Emergency stop
    ]

    print(f"\nStarting test of {len(test_commands)} Japanese commands...")

    for i, command in enumerate(test_commands, 1):
        print(f"\n{'='*60}")
        print(f"Test {i}/{len(test_commands)}: {command}")
        print('='*60)

        try:
            result = await interface.process_japanese_command(command)

            # Display test result summary
            print(f"\nTest result summary:")
            print(f"  - Execution status: {'PASS' if result['success'] else 'FAIL'}")
            print(f"  - Total time: {result['total_time']:.2f}s")

            if result.get('llm_analysis'):
                llm_analysis = result['llm_analysis']
                print(f"  - LLM analysis: {llm_analysis.get('analysis_time', 0):.2f}s")
                if llm_analysis.get('extracted_action'):
                    print(f"  - Recognized action: {llm_analysis['extracted_action']} (confidence: {llm_analysis.get('confidence', 0):.1%})")

            if result.get('action_sequence'):
                seq = result['action_sequence']
                print(f"  - Action sequence: {len(seq)} steps")
                for j, step in enumerate(seq, 1):
                    print(f"    {j}. {step['action']} (API: {step['api']})")

            if result.get('execution_result'):
                exec_result = result['execution_result']
                if exec_result.get('completed_steps'):
                    print(f"  - Execution result: {exec_result['completed_steps']}/{exec_result.get('total_steps', 0)} steps succeeded")

        except Exception as e:
            print(f"Test exception: {str(e)}")

        print(f"\n{'='*60}")

        # Brief pause
        await asyncio.sleep(1)

    # Display final statistics
    print(f"\nTest complete!")
    print(f"Command history: {len(interface.command_history)} entries")
    print(f"Current robot state: {interface.robot_state.current_posture}")
    print(f"Battery level: {interface.robot_state.battery_level}%")

    # Display detailed history
    print(f"\nDetailed execution history:")
    for i, cmd in enumerate(interface.command_history, 1):
        status = "PASS" if cmd.get('success') else "FAIL"
        print(f"  {i}. [{status}] {cmd['user_input']} ({cmd['total_time']:.1f}s)")

async def test_llm_extraction():
    """Test LLM action extraction independently"""
    print("\nLLM Action Extraction Test")
    print("-" * 40)

    interface = EnhancedJapaneseCommandInterface()

    # Japanese test phrases (robot commands)
    test_phrases = [
        "お座りしてください",    # please sit
        "立ち上がって",         # stand up
        "ダンスしましょう",      # let's dance
        "こんにちは",           # hello
        "ストレッチしてみて",    # try stretching
        "停止してください",     # please stop
        "回ってください",       # please turn
        "緊急停止！",          # emergency stop!
    ]

    for phrase in test_phrases:
        action, confidence = interface.extract_action_from_llm_response(phrase)
        print(f"  '{phrase}' -> {action or 'unknown'} (confidence: {confidence:.1%})")

async def test_action_sequencing():
    """Test action sequence planning"""
    print("\nAction Sequence Planning Test")
    print("-" * 40)

    interface = EnhancedJapaneseCommandInterface()

    # Test action planning from different states
    test_scenarios = [
        ("sitting", "hello", 1016, "Execute greeting from sitting state"),
        ("lying", "sit", 1009, "Sit down from lying state"),
        ("standing", "dance", 1022, "Dance from standing state"),
        ("unknown", "stretch", 1017, "Stretch from unknown state"),
    ]

    for current_state, target_action, api_code, description in test_scenarios:
        interface.robot_state.current_posture = current_state
        sequence = interface.action_sequencer.plan_action_sequence(target_action, api_code)

        print(f"  {description}:")
        print(f"    Current state: {current_state}")
        print(f"    Target action: {target_action}")
        print(f"    Planned sequence: {len(sequence)} steps")
        for i, step in enumerate(sequence, 1):
            print(f"      {i}. {step['action']} (API: {step['api']})")
        print()

async def main():
    """Main function"""
    print("Claudia Robot - Enhanced Japanese Command Interface Full Test")
    print("=" * 60)

    try:
        # Run LLM extraction test
        await test_llm_extraction()

        # Run action sequence test
        await test_action_sequencing()

        # Run full functionality test
        await test_enhanced_japanese_commander()

        print("\nAll tests complete!")

    except Exception as e:
        print(f"\nException during testing: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(main())
