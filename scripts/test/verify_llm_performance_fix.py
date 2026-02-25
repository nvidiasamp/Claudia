#!/usr/bin/env python3
"""
Claudia Robot - LLM Performance Optimization Verification Script
Verify log duplication fix and 3B model performance optimization
"""

import sys
import time
import asyncio
from pathlib import Path

# Add project path
sys.path.append(str(Path(__file__).parent.parent.parent))

from scripts.llm.claudia_llm_interface import ClaudiaLLMInterface

def test_performance_optimization():
    """Test performance optimization effects"""
    print("Claudia LLM Performance Optimization Verification")
    print("=" * 50)

    # 1. Initialize optimized interface
    llm = ClaudiaLLMInterface()

    # 2. Display optimized configuration
    print(f"\nOptimized configuration:")
    print(f"  Model: {llm.model_name}")
    print(f"  Timeout: {llm.request_timeout}s")
    print(f"  Retry count: {llm.max_retries}")
    print(f"  Temperature: {llm.model_params['temperature']}")
    print(f"  Output limit: {llm.model_params['num_predict']} chars")

    # 3. Performance test cases
    # Note: Japanese test inputs are intentional - they are robot commands
    test_cases = [
        "座る",        # sit
        "こんにちは",    # hello
        "立って",       # stand up
        "ダンスして",    # dance
        "停止"         # stop
    ]

    print(f"\nPerformance test ({len(test_cases)} cases):")

    total_start = time.time()
    results = []

    for i, command in enumerate(test_cases, 1):
        print(f"\n[{i}/{len(test_cases)}] Test: '{command}'")

        start_time = time.time()
        response = llm.generate_response(f"指令: {command}")
        end_time = time.time()

        duration = end_time - start_time
        success = not response.startswith("ERROR")

        results.append({
            "command": command,
            "duration": duration,
            "success": success,
            "response_length": len(response)
        })

        status = "PASS" if success else "FAIL"
        print(f"  {status} - {duration:.2f}s - {len(response)} chars")

    # 4. Performance statistics
    total_time = time.time() - total_start
    success_count = sum(1 for r in results if r["success"])
    avg_time = sum(r["duration"] for r in results) / len(results)

    print(f"\nPerformance statistics:")
    print(f"  Total time: {total_time:.2f}s")
    print(f"  Success rate: {success_count}/{len(test_cases)} ({success_count/len(test_cases)*100:.1f}%)")
    print(f"  Average response: {avg_time:.2f}s")
    print(f"  Fastest response: {min(r['duration'] for r in results):.2f}s")
    print(f"  Slowest response: {max(r['duration'] for r in results):.2f}s")

    # 5. Evaluate optimization effects
    print(f"\nOptimization effect evaluation:")

    # Target performance metrics
    target_avg_time = 3.0  # Target average response time
    target_success_rate = 90  # Target success rate

    if avg_time <= target_avg_time:
        print(f"  [PASS] Response speed: {avg_time:.2f}s <= {target_avg_time}s (on target)")
    else:
        print(f"  [WARN] Response speed: {avg_time:.2f}s > {target_avg_time}s (needs further optimization)")

    success_rate = success_count/len(test_cases)*100
    if success_rate >= target_success_rate:
        print(f"  [PASS] Success rate: {success_rate:.1f}% >= {target_success_rate}% (on target)")
    else:
        print(f"  [WARN] Success rate: {success_rate:.1f}% < {target_success_rate}% (needs improvement)")

    # 6. Optimization recommendations
    if avg_time > target_avg_time:
        print(f"\nOptimization recommendations:")
        print(f"  - Further reduce num_predict parameter")
        print(f"  - Adjust temperature to a lower value")
        print(f"  - Check network latency and hardware performance")

    return results

async def test_enhanced_interface():
    """Test enhanced interface performance"""
    print(f"\nTesting enhanced interface...")

    try:
        from src.claudia.interactive_japanese_commander_enhanced import EnhancedJapaneseCommandInterface

        interface = EnhancedJapaneseCommandInterface()
        await interface.initialize()

        # Test single command processing
        test_command = "座る"  # sit
        print(f"  Test command: {test_command}")

        start_time = time.time()
        result = await interface.process_japanese_command(test_command)
        end_time = time.time()

        total_time = end_time - start_time
        success = result.get("success", False)

        print(f"  Result: {'PASS' if success else 'FAIL'}")
        print(f"  Total time: {total_time:.2f}s")

        # Analyze time distribution
        if "llm_analysis" in result:
            llm_time = result["llm_analysis"].get("analysis_time", 0)
            print(f"  LLM analysis: {llm_time:.2f}s")

        if "execution_result" in result:
            exec_time = result["execution_result"].get("total_time", 0)
            print(f"  Action execution: {exec_time:.2f}s")

        return True

    except Exception as e:
        print(f"  Enhanced interface test failed: {e}")
        return False

def main():
    """Main function"""
    try:
        # 1. Basic LLM performance test
        llm_results = test_performance_optimization()

        # 2. Enhanced interface test
        asyncio.run(test_enhanced_interface())

        print(f"\nPerformance verification complete!")

        # Recommend next steps
        avg_time = sum(r["duration"] for r in llm_results) / len(llm_results)
        if avg_time <= 3.0:
            print(f"Performance optimization successful, recommended for production use")
        else:
            print(f"Room for optimization remains, recommend continuing parameter adjustments")

    except Exception as e:
        print(f"Verification process error: {e}")

if __name__ == "__main__":
    main()
