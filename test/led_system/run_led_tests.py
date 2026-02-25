 #!/usr/bin/env python3
"""
LED Control System Test Framework Main Runner
Task 6.5: Comprehensive Testing, Validation, and Performance Optimization

Provides unified test execution, report generation, and result analysis
"""

import os
import sys
import time
import argparse
import unittest
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Any, Optional

# Add project root directory to Python path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from .test_config import get_led_test_config, reset_led_test_config
from .data_collector import get_led_test_collector, reset_led_test_collector
from .test_led_modes import LEDModesFunctionalTest, LEDModesStressTest
from .test_performance import LEDPerformanceBenchmark, LEDPerformanceRegression


class LEDTestRunner:
    """LED test framework main runner"""

    def __init__(self, config_overrides: Optional[Dict[str, Any]] = None):
        """Initialize test runner"""
        self.config = get_led_test_config()
        self.collector = get_led_test_collector()

        # Apply config overrides
        if config_overrides:
            self._apply_config_overrides(config_overrides)

        self.test_suites = {
            'functional': self._create_functional_suite,
            'performance': self._create_performance_suite,
            'stress': self._create_stress_suite,
            'regression': self._create_regression_suite,
            'all': self._create_all_suites
        }

        print(f"LED Control System Test Framework v1.0")
        print(f"Initialization time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"Test mode: {self.config.get_test_mode()}")

    def _apply_config_overrides(self, overrides: Dict[str, Any]):
        """Apply config overrides"""
        for key, value in overrides.items():
            if hasattr(self.config, key):
                setattr(self.config, key, value)
                print(f"Config override: {key} = {value}")

    def run_tests(self, test_type: str = "all", verbosity: int = 2,
                 output_dir: Optional[str] = None) -> Dict[str, Any]:
        """Run tests of specified type"""

        print(f"\n{'='*80}")
        print(f"Starting {test_type.upper()} tests")
        print(f"{'='*80}")

        # Create test suite
        if test_type not in self.test_suites:
            raise ValueError(f"Unsupported test type: {test_type}. Supported types: {list(self.test_suites.keys())}")

        suite_creator = self.test_suites[test_type]

        if test_type == 'all':
            # Run all test suites
            results = {}
            for suite_name in ['functional', 'performance', 'stress', 'regression']:
                if suite_name == 'stress' and not self.config.is_stress_test_enabled():
                    print(f"Skipping {suite_name} tests (stress tests not enabled)")
                    continue

                print(f"\nRunning {suite_name.upper()} test suite...")
                suite = self.test_suites[suite_name]()
                result = self._run_test_suite(suite, verbosity)
                results[suite_name] = result

            # Combine all results
            combined_result = self._combine_results(results)

        else:
            # Run single test suite
            if test_type == 'stress' and not self.config.is_stress_test_enabled():
                print("Stress tests are disabled, please check configuration")
                return {'error': 'stress_tests_disabled'}

            suite = suite_creator()
            combined_result = self._run_test_suite(suite, verbosity)

        # Generate reports
        self._generate_test_reports(output_dir)

        # Display test summary
        self._display_test_summary(combined_result)

        return combined_result

    def _create_functional_suite(self) -> unittest.TestSuite:
        """Create functional test suite"""
        suite = unittest.TestSuite()
        suite.addTest(unittest.makeSuite(LEDModesFunctionalTest))
        return suite

    def _create_performance_suite(self) -> unittest.TestSuite:
        """Create performance test suite"""
        suite = unittest.TestSuite()
        suite.addTest(unittest.makeSuite(LEDPerformanceBenchmark))
        return suite

    def _create_stress_suite(self) -> unittest.TestSuite:
        """Create stress test suite"""
        suite = unittest.TestSuite()
        suite.addTest(unittest.makeSuite(LEDModesStressTest))
        return suite

    def _create_regression_suite(self) -> unittest.TestSuite:
        """Create regression test suite"""
        suite = unittest.TestSuite()
        suite.addTest(unittest.makeSuite(LEDPerformanceRegression))
        return suite

    def _create_all_suites(self) -> unittest.TestSuite:
        """Create all test suites"""
        # This method is not called directly because 'all' type is handled specially in run_tests
        return unittest.TestSuite()

    def _run_test_suite(self, suite: unittest.TestSuite, verbosity: int) -> Dict[str, Any]:
        """Run test suite and return results"""
        start_time = time.time()

        # Create test runner
        runner = unittest.TextTestRunner(
            verbosity=verbosity,
            stream=sys.stdout,
            buffer=False
        )

        # Run tests
        result = runner.run(suite)

        end_time = time.time()
        duration = end_time - start_time

        # Compile results
        test_results = {
            'tests_run': result.testsRun,
            'failures': len(result.failures),
            'errors': len(result.errors),
            'skipped': len(result.skipped) if hasattr(result, 'skipped') else 0,
            'success_count': result.testsRun - len(result.failures) - len(result.errors),
            'success_rate': ((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100) if result.testsRun > 0 else 0,
            'duration': duration,
            'failure_details': [{'test': str(test), 'traceback': traceback} for test, traceback in result.failures],
            'error_details': [{'test': str(test), 'traceback': traceback} for test, traceback in result.errors]
        }

        return test_results

    def _combine_results(self, results: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
        """Combine multiple test results"""
        combined = {
            'tests_run': 0,
            'failures': 0,
            'errors': 0,
            'skipped': 0,
            'success_count': 0,
            'total_duration': 0,
            'suite_results': results,
            'failure_details': [],
            'error_details': []
        }

        for suite_name, result in results.items():
            if 'error' in result:
                continue

            combined['tests_run'] += result['tests_run']
            combined['failures'] += result['failures']
            combined['errors'] += result['errors']
            combined['skipped'] += result['skipped']
            combined['success_count'] += result['success_count']
            combined['total_duration'] += result['duration']
            combined['failure_details'].extend(result['failure_details'])
            combined['error_details'].extend(result['error_details'])

        combined['success_rate'] = (combined['success_count'] / combined['tests_run'] * 100) if combined['tests_run'] > 0 else 0

        return combined

    def _generate_test_reports(self, output_dir: Optional[str] = None):
        """Generate test reports"""
        if output_dir:
            self.collector.output_dir = Path(output_dir)

        # Generate HTML report
        html_report = self.collector.generate_report()

        # Save JSON data
        json_data = self.collector.save_data(format_type="json")

        # Save CSV data
        csv_data = self.collector.save_data(format_type="csv")

        print(f"\nTest reports generated:")
        print(f"   HTML report: {html_report}")
        print(f"   JSON data: {json_data}")
        print(f"   CSV data: {csv_data}")

    def _display_test_summary(self, results: Dict[str, Any]):
        """Display test summary"""
        print(f"\n{'='*80}")
        print(f"LED Control System Test Summary")
        print(f"{'='*80}")

        if 'error' in results:
            print(f"Test execution error: {results['error']}")
            return

        # Basic statistics
        print(f"Total tests: {results['tests_run']}")
        print(f"Success: {results['success_count']}")
        print(f"Failures: {results['failures']}")
        print(f"Errors: {results['errors']}")
        print(f"Skipped: {results['skipped']}")
        print(f"Success rate: {results['success_rate']:.1f}%")
        print(f"Total duration: {results['total_duration']:.2f}s")

        # Status assessment
        if results['success_rate'] >= 95:
            status_text = "Excellent"
        elif results['success_rate'] >= 90:
            status_text = "Good"
        elif results['success_rate'] >= 80:
            status_text = "Fair"
        else:
            status_text = "Needs Improvement"

        print(f"\nOverall assessment: {status_text}")

        # Suite details (if multiple suites)
        if 'suite_results' in results:
            print(f"\nPer-suite results:")
            for suite_name, suite_result in results['suite_results'].items():
                if 'error' in suite_result:
                    print(f"   {suite_name}: {suite_result['error']}")
                else:
                    rate = suite_result['success_rate']
                    print(f"   {suite_name}: {suite_result['success_count']}/{suite_result['tests_run']} ({rate:.1f}%)")

        # Real-time statistics
        real_time_stats = self.collector.get_real_time_stats()
        if real_time_stats:
            print(f"\nPerformance metrics:")
            if 'response_time' in real_time_stats:
                rt = real_time_stats['response_time']
                print(f"   Response time: avg {rt.get('overall_avg', 0):.2f}ms, max {rt.get('overall_max', 0):.2f}ms")

            if 'cpu_usage' in real_time_stats:
                cpu = real_time_stats['cpu_usage']
                print(f"   CPU usage: avg {cpu.get('avg', 0):.1f}%, max {cpu.get('max', 0):.1f}%")

            if 'memory_usage' in real_time_stats:
                mem = real_time_stats['memory_usage']
                print(f"   Memory usage: avg {mem.get('avg', 0):.1f}MB, max {mem.get('max', 0):.1f}MB")

        # Failure details (if any)
        if results['failures'] > 0 or results['errors'] > 0:
            print(f"\nFailure details:")
            for failure in results['failure_details'][:3]:  # Show first 3
                print(f"   FAILURE: {failure['test']}")

            for error in results['error_details'][:3]:  # Show first 3
                print(f"   ERROR: {error['test']}")

            if len(results['failure_details']) + len(results['error_details']) > 6:
                print(f"   ... see full report for more details")

        print(f"\n{'='*80}")


def main():
    """Main function"""
    parser = argparse.ArgumentParser(description="LED Control System Test Framework")

    parser.add_argument('--type', '-t',
                       choices=['functional', 'performance', 'stress', 'regression', 'all'],
                       default='all',
                       help='Test type (default: all)')

    parser.add_argument('--verbosity', '-v',
                       type=int, choices=[0, 1, 2], default=2,
                       help='Verbosity level (0=silent, 1=normal, 2=verbose)')

    parser.add_argument('--output', '-o',
                       help='Output directory (default: logs/led_tests)')

    parser.add_argument('--hardware',
                       action='store_true',
                       help='Enable hardware test mode')

    parser.add_argument('--stress',
                       action='store_true',
                       help='Enable stress tests')

    parser.add_argument('--performance-samples',
                       type=int, default=50,
                       help='Performance test sample count (default: 50)')

    parser.add_argument('--max-response-time',
                       type=float, default=200.0,
                       help='Maximum response time threshold in ms (default: 200.0)')

    args = parser.parse_args()

    # Prepare config overrides
    config_overrides = {}

    if args.hardware:
        config_overrides['hardware.hardware_required'] = True
        config_overrides['hardware.mock_hardware'] = False

    if args.stress:
        config_overrides['stress_tests_enabled'] = True

    if args.performance_samples != 50:
        config_overrides['performance.performance_samples'] = args.performance_samples

    if args.max_response_time != 200.0:
        config_overrides['performance.max_response_time_ms'] = args.max_response_time

    try:
        # Reset collector to ensure clean test environment
        reset_led_test_collector()

        # Create test runner
        runner = LEDTestRunner(config_overrides)

        # Run tests
        results = runner.run_tests(
            test_type=args.type,
            verbosity=args.verbosity,
            output_dir=args.output
        )

        # Set exit code based on results
        if 'error' in results:
            sys.exit(1)
        elif results['failures'] > 0 or results['errors'] > 0:
            sys.exit(1)
        else:
            sys.exit(0)

    except KeyboardInterrupt:
        print("\nTests interrupted by user")
        sys.exit(130)
    except Exception as e:
        print(f"\nTest framework error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
