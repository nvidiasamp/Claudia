#!/usr/bin/env python3
"""
Quick LLM Test Script - Verify timeout fix
"""

import sys
import time
from pathlib import Path

# Add project path
sys.path.append(str(Path(__file__).parent.parent.parent))

from scripts.llm.claudia_llm_interface import ClaudiaLLMInterface

def quick_test():
    """Quick LLM performance test"""
    print("Quick LLM Test - Verify timeout fix")
    print("=" * 40)

    # Initialize interface
    llm = ClaudiaLLMInterface()

    # Display configuration
    print(f"Model: {llm.model_name}")
    print(f"Timeout: {llm.request_timeout} seconds")
    print(f"Parameters: {llm.model_params}")

    # Test simple commands (Japanese robot commands: sit, hello, stand up)
    test_commands = ["座る", "こんにちは", "立って"]

    for i, cmd in enumerate(test_commands, 1):
        print(f"\n[{i}] Test: '{cmd}'")

        start_time = time.time()
        try:
            response = llm.generate_response(f"指令: {cmd}")
            end_time = time.time()

            duration = end_time - start_time
            success = not response.startswith("ERROR")

            status = "PASS" if success else "FAIL"
            print(f"  {status} - {duration:.2f}s")
            print(f"  Response: {response[:50]}...")

            if not success:
                print(f"  Failure reason needs further investigation")
                break

        except Exception as e:
            print(f"  Exception: {e}")
            break

    print(f"\nTest complete")

if __name__ == "__main__":
    quick_test()
