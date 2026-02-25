#!/usr/bin/env python3
"""
Test Model Configuration Fix
Verify that the system correctly uses claudia-optimized:v2.1 instead of the latest version
"""

import sys
import time
from pathlib import Path

# Add project path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from scripts.llm.claudia_llm_interface import ClaudiaLLMInterface

def test_default_model_configuration():
    """Test default model configuration"""
    print("Testing default model configuration...")

    # Test 1: Default initialization
    llm = ClaudiaLLMInterface()
    print(f"  Default model: {llm.model_name}")

    # Test 2: Explicitly specify v2.1
    llm_v21 = ClaudiaLLMInterface(model_name="claudia-optimized:v2.1")
    print(f"  Specified v2.1 model: {llm_v21.model_name}")

    # Test 3: Check connection status
    status = llm.get_status()
    print(f"  Connection status: {status['connection']}")
    print(f"  Active model: {status['model']}")

    if status.get('available_models'):
        print(f"  Available models: {', '.join(status['available_models'])}")

    return llm.model_name == "claudia-optimized:v2.1"

def test_enhanced_interface_configuration():
    """Test the enhanced interface model configuration"""
    print("\nTesting enhanced interface model configuration...")

    try:
        from src.claudia.interactive_japanese_commander_enhanced import EnhancedJapaneseCommandInterface

        interface = EnhancedJapaneseCommandInterface()

        # Simulate LLM initialization (without starting the full interface)
        interface.llm_interface = ClaudiaLLMInterface(model_name="claudia-optimized:v2.1")

        if interface.llm_interface:
            print(f"  Enhanced interface using model: {interface.llm_interface.model_name}")
            return interface.llm_interface.model_name == "claudia-optimized:v2.1"
        else:
            print("  Failed to initialize LLM interface")
            return False

    except Exception as e:
        print(f"  Test failed: {e}")
        return False

def test_single_llm_call():
    """Test a single LLM call (ensure no duplicate calls)"""
    print("\nTesting single LLM call...")

    llm = ClaudiaLLMInterface(model_name="claudia-optimized:v2.1")

    # Record call start time
    start_time = time.time()

    # Simple test call (Japanese greeting: "hello")
    test_input = "こんにちは"

    print(f"  Test input: {test_input}")
    print(f"  Using model: {llm.model_name}")

    try:
        response = llm.robot_command_interpreter(test_input)
        elapsed_time = time.time() - start_time

        print(f"  Call successful")
        print(f"  Response time: {elapsed_time:.2f}s")
        print(f"  Response length: {len(response)} characters")

        # Check if response is reasonable (not an error message)
        if "ERROR" in response:
            print(f"  Response contains error message: {response[:100]}...")
            return False

        return True

    except Exception as e:
        print(f"  Call failed: {e}")
        return False

def main():
    """Main test function"""
    print("Claudia Model Configuration Fix Verification")
    print("=" * 50)

    tests = [
        ("Default model configuration", test_default_model_configuration),
        ("Enhanced interface configuration", test_enhanced_interface_configuration),
        ("Single LLM call", test_single_llm_call)
    ]

    results = []

    for test_name, test_func in tests:
        print(f"\nExecuting test: {test_name}")
        try:
            result = test_func()
            results.append((test_name, result))
            status = "PASS" if result else "FAIL"
            print(f"  Result: {status}")
        except Exception as e:
            results.append((test_name, False))
            print(f"  Exception: {e}")

    # Summary
    print(f"\nTest summary:")
    passed = sum(1 for _, result in results if result)
    total = len(results)

    for test_name, result in results:
        status = "[PASS]" if result else "[FAIL]"
        print(f"  {status} {test_name}")

    print(f"\nPass rate: {passed}/{total} ({passed/total*100:.1f}%)")

    if passed == total:
        print("All tests passed! Model configuration fix successful")
    else:
        print("Some tests failed, further investigation needed")

if __name__ == "__main__":
    main()
