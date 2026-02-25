#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Claudia Robot Project - Performance Optimization Verification Test
Tests performance improvements after optimization
"""

import sys
import time
import statistics
from pathlib import Path

# Add project path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from scripts.llm.claudia_llm_interface import ClaudiaLLMInterface

class PerformanceTestSuite:
    """Performance test suite"""

    def __init__(self):
        self.results = {
            "response_times": [],
            "success_count": 0,
            "error_count": 0,
            "model_info": None
        }

        # Test command set (Japanese robot commands)
        self.test_commands = [
            "立って",       # stand up
            "座って",       # sit down
            "こんにちは",    # hello
            "停止",         # stop
            "ダンスして",    # dance
            "伏せ"          # lie down
        ]

    def run_performance_test(self):
        """Run performance test"""
        print("Claudia Performance Optimization Verification Test")
        print("=" * 50)
        print(f"Test time: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"Test command count: {len(self.test_commands)}")
        print()

        # Initialize LLM interface
        try:
            llm = ClaudiaLLMInterface()
            print(f"Connected to model: {llm.model_name}")

            # Get model status
            status = llm.get_status()
            self.results["model_info"] = status
            print(f"Connection status: {status.get('connection', 'unknown')}")
            print()

        except Exception as e:
            print(f"LLM initialization failed: {e}")
            return

        # Execute tests
        print("Starting performance test...")
        print("-" * 30)

        total_start_time = time.time()

        for i, command in enumerate(self.test_commands, 1):
            print(f"[{i}/{len(self.test_commands)}] Test: '{command}'")

            # Test single command
            start_time = time.time()
            try:
                result = llm.robot_command_interpreter(command)
                response_time = time.time() - start_time

                self.results["response_times"].append(response_time)
                self.results["success_count"] += 1

                # Display result
                if response_time < 15:
                    status_label = "[FAST]"
                elif response_time < 30:
                    status_label = "[MODERATE]"
                else:
                    status_label = "[SLOW]"
                print(f"  {status_label} Time: {response_time:.2f}s")
                print(f"  Response: {result[:60]}{'...' if len(result) > 60 else ''}")

            except Exception as e:
                self.results["error_count"] += 1
                print(f"  Error: {str(e)}")

            print()

        total_time = time.time() - total_start_time

        # Analyze results
        self._analyze_results(total_time)

    def _analyze_results(self, total_time):
        """Analyze test results"""
        print("Performance Analysis Results")
        print("=" * 50)

        if not self.results["response_times"]:
            print("No successful test results")
            return

        # Basic statistics
        response_times = self.results["response_times"]
        avg_time = statistics.mean(response_times)
        min_time = min(response_times)
        max_time = max(response_times)
        median_time = statistics.median(response_times)

        print(f"Test statistics:")
        print(f"  Total tests: {len(self.test_commands)}")
        print(f"  Successful: {self.results['success_count']}")
        print(f"  Failed: {self.results['error_count']}")
        print(f"  Success rate: {(self.results['success_count']/len(self.test_commands)*100):.1f}%")

        print(f"\nResponse times:")
        print(f"  Average: {avg_time:.2f}s")
        print(f"  Fastest: {min_time:.2f}s")
        print(f"  Slowest: {max_time:.2f}s")
        print(f"  Median: {median_time:.2f}s")
        print(f"  Total time: {total_time:.2f}s")

        # Performance grade evaluation
        print(f"\nPerformance grade:")
        if avg_time <= 5:
            grade = "A+ Excellent"
        elif avg_time <= 10:
            grade = "A Good"
        elif avg_time <= 20:
            grade = "B Average"
        elif avg_time <= 35:
            grade = "C Slow"
        else:
            grade = "D Too slow"

        print(f"  {grade} (average {avg_time:.2f}s)")

        # Comparison with pre-optimization
        print(f"\nOptimization effect comparison:")
        baseline_time = 40.0  # Pre-optimization baseline time
        improvement = ((baseline_time - avg_time) / baseline_time) * 100
        print(f"  Before optimization: ~{baseline_time:.1f}s")
        print(f"  After optimization: {avg_time:.2f}s")
        if improvement > 0:
            print(f"  Improvement: {improvement:.1f}% faster")
        else:
            print(f"  Regression: {abs(improvement):.1f}% slower")

        # Model information
        model_info = self.results.get("model_info", {})
        print(f"\nModel information:")
        print(f"  Model: {model_info.get('model', 'unknown')}")
        print(f"  Timeout setting: {model_info.get('timeout', 'unknown')}s")

        # Recommendations
        print(f"\nOptimization recommendations:")
        if avg_time > 15:
            print("  - Consider using a lighter model")
            print("  - Reduce context length")
            print("  - Enable command caching")
        elif avg_time > 8:
            print("  - Can further optimize parameter configuration")
            print("  - Consider preprocessing common commands")
        else:
            print("  - Performance has reached an excellent level!")
            print("  - Consider adding more complex features")

def main():
    """Main function"""
    try:
        tester = PerformanceTestSuite()
        tester.run_performance_test()

    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"\nTest exception: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
