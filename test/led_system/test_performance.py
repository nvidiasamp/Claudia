#!/usr/bin/env python3
"""
LED System Performance Benchmark Tests
Validates response time, resource usage, concurrent performance, and other key metrics
"""

import time
import threading
import psutil
import os
import unittest
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import List, Dict, Any, Tuple

from .led_test_base import LEDTestBase
from .test_config import get_led_test_config
from .data_collector import get_led_test_collector


class LEDPerformanceBenchmark(LEDTestBase):
    """LED performance benchmark tests"""

    def setUp(self):
        """Pre-test setup"""
        super().setUp()
        self.config = get_led_test_config()
        self.collector = get_led_test_collector()
        self.collector.start_test_session("led_performance_benchmark",
                                         {'test_type': 'performance', 'module': 'led_system'})

        # Get baseline resource usage
        self.baseline_memory = self._get_memory_usage()
        self.baseline_cpu = self._get_cpu_usage()

        print(f"Baseline resource usage - Memory: {self.baseline_memory:.2f}MB, CPU: {self.baseline_cpu:.1f}%")

    def tearDown(self):
        """Post-test cleanup"""
        self.collector.end_test_session("led_performance_benchmark")
        super().tearDown()

    def test_led_response_time_benchmark(self):
        """LED response time benchmark test"""
        self.assertLEDSystemReady()

        # Test response time for all LED modes
        modes = ["wake_confirm", "processing_voice", "executing_action", "action_complete", "error_state"]
        response_times = {}

        for mode in modes:
            if not hasattr(self.led_system, mode):
                continue

            # Measure multiple times and average
            times = []
            for i in range(self.config.performance.performance_samples):
                start_time = time.perf_counter()

                try:
                    getattr(self.led_system, mode)()
                    end_time = time.perf_counter()
                    response_time = (end_time - start_time) * 1000  # Convert to milliseconds
                    times.append(response_time)

                    # Record each measurement
                    self.collector.record_performance_data("led_performance_benchmark",
                                                          response_time=response_time)

                except Exception as e:
                    self.collector.record_error("led_performance_benchmark", "response_time",
                                               f"Mode {mode} response time test failed: {e}")
                    continue

                # Brief wait to avoid excessive call frequency
                time.sleep(0.01)

            if times:
                avg_time = sum(times) / len(times)
                min_time = min(times)
                max_time = max(times)

                response_times[mode] = {
                    'average': avg_time,
                    'min': min_time,
                    'max': max_time,
                    'samples': len(times)
                }

                # Verify performance requirement
                self.assertLessEqual(avg_time, self.config.performance.max_response_time_ms,
                                   f"Mode {mode} average response time too long: {avg_time:.2f}ms")

                # Record performance metrics
                self.collector.record_metric(f"{mode}_avg_response_time", avg_time, "ms",
                                            "led_performance_benchmark", "performance")
                self.collector.record_metric(f"{mode}_max_response_time", max_time, "ms",
                                            "led_performance_benchmark", "performance")

                print(f"{mode}: avg {avg_time:.2f}ms, range {min_time:.2f}-{max_time:.2f}ms")

        # Calculate overall performance statistics
        if response_times:
            all_averages = [times['average'] for times in response_times.values()]
            overall_avg = sum(all_averages) / len(all_averages)
            overall_max = max(times['max'] for times in response_times.values())

            self.collector.record_metric("overall_avg_response_time", overall_avg, "ms",
                                        "led_performance_benchmark", "performance")
            self.collector.record_metric("overall_max_response_time", overall_max, "ms",
                                        "led_performance_benchmark", "performance")

            print(f"Overall performance - avg: {overall_avg:.2f}ms, max: {overall_max:.2f}ms")

        print("LED response time benchmark test passed")

    def test_resource_usage_monitoring(self):
        """Resource usage monitoring test"""
        self.assertLEDSystemReady()

        # Record initial resource usage
        initial_memory = self._get_memory_usage()
        initial_cpu = self._get_cpu_usage()

        resource_samples = []
        test_duration = 10.0  # Monitor for 10 seconds
        sample_interval = 0.5  # Sample every 0.5 seconds

        start_time = time.time()

        def resource_monitor():
            """Resource monitor thread"""
            while time.time() - start_time < test_duration:
                memory = self._get_memory_usage()
                cpu = self._get_cpu_usage()

                resource_samples.append({
                    'timestamp': time.time() - start_time,
                    'memory_mb': memory,
                    'cpu_percent': cpu,
                    'memory_delta': memory - initial_memory,
                    'cpu_delta': cpu - initial_cpu
                })

                # Record real-time data
                self.collector.record_performance_data("led_performance_benchmark",
                                                      cpu_usage=cpu, memory_usage=memory)

                time.sleep(sample_interval)

        def led_activity():
            """LED activity thread"""
            modes = ["wake_confirm", "processing_voice", "executing_action", "action_complete"]

            while time.time() - start_time < test_duration:
                for mode in modes:
                    if hasattr(self.led_system, mode):
                        try:
                            getattr(self.led_system, mode)()
                            time.sleep(0.2)
                        except Exception as e:
                            self.collector.record_error("led_performance_benchmark", "resource_test",
                                                       f"LED activity failed {mode}: {e}")

                        if time.time() - start_time >= test_duration:
                            break

        # Run monitor and LED activity concurrently
        monitor_thread = threading.Thread(target=resource_monitor)
        activity_thread = threading.Thread(target=led_activity)

        monitor_thread.start()
        activity_thread.start()

        monitor_thread.join()
        activity_thread.join()

        # Analyze resource usage data
        if resource_samples:
            memory_usage = [s['memory_mb'] for s in resource_samples]
            cpu_usage = [s['cpu_percent'] for s in resource_samples]
            memory_deltas = [s['memory_delta'] for s in resource_samples]

            avg_memory = sum(memory_usage) / len(memory_usage)
            max_memory = max(memory_usage)
            avg_cpu = sum(cpu_usage) / len(cpu_usage)
            max_cpu = max(cpu_usage)
            max_memory_delta = max(memory_deltas)

            # Verify resource usage within acceptable range
            self.assertLessEqual(max_memory_delta, self.config.performance.baseline_memory_mb,
                               f"Memory growth too large: {max_memory_delta:.2f}MB")
            self.assertLessEqual(avg_cpu, self.config.performance.baseline_cpu_threshold,
                               f"CPU usage too high: {avg_cpu:.1f}%")

            # Record resource usage metrics
            self.collector.record_metric("avg_memory_usage", avg_memory, "MB",
                                        "led_performance_benchmark", "resource")
            self.collector.record_metric("max_memory_usage", max_memory, "MB",
                                        "led_performance_benchmark", "resource")
            self.collector.record_metric("max_memory_delta", max_memory_delta, "MB",
                                        "led_performance_benchmark", "resource")
            self.collector.record_metric("avg_cpu_usage", avg_cpu, "%",
                                        "led_performance_benchmark", "resource")
            self.collector.record_metric("max_cpu_usage", max_cpu, "%",
                                        "led_performance_benchmark", "resource")

            print(f"Resource usage statistics:")
            print(f"   Memory: avg {avg_memory:.2f}MB, max {max_memory:.2f}MB, growth {max_memory_delta:.2f}MB")
            print(f"   CPU: avg {avg_cpu:.1f}%, max {max_cpu:.1f}%")

        print("Resource usage monitoring test passed")

    def test_concurrent_performance(self):
        """Concurrent performance test"""
        self.assertLEDSystemReady()

        # Concurrent level tests
        concurrent_levels = [1, 5, 10, 20]
        performance_results = {}

        for concurrent_count in concurrent_levels:
            print(f"Testing concurrent level: {concurrent_count}")

            # Prepare tasks
            def led_task(task_id: int) -> Dict[str, Any]:
                mode = ["wake_confirm", "processing_voice", "executing_action"][task_id % 3]

                start_time = time.perf_counter()
                try:
                    if hasattr(self.led_system, mode):
                        getattr(self.led_system, mode)()
                    end_time = time.perf_counter()

                    return {
                        'task_id': task_id,
                        'mode': mode,
                        'duration': (end_time - start_time) * 1000,
                        'success': True
                    }
                except Exception as e:
                    return {
                        'task_id': task_id,
                        'mode': mode,
                        'error': str(e),
                        'success': False
                    }

            # Execute concurrent tasks
            start_time = time.perf_counter()

            with ThreadPoolExecutor(max_workers=concurrent_count) as executor:
                futures = [executor.submit(led_task, i) for i in range(concurrent_count)]
                results = [future.result() for future in as_completed(futures)]

            end_time = time.perf_counter()
            total_duration = end_time - start_time

            # Analyze results
            successful_tasks = [r for r in results if r['success']]
            failed_tasks = [r for r in results if not r['success']]

            if successful_tasks:
                durations = [r['duration'] for r in successful_tasks]
                avg_duration = sum(durations) / len(durations)
                max_duration = max(durations)
                min_duration = min(durations)

                success_rate = (len(successful_tasks) / len(results)) * 100
                throughput = len(successful_tasks) / total_duration  # tasks/second

                performance_results[concurrent_count] = {
                    'success_rate': success_rate,
                    'avg_duration': avg_duration,
                    'max_duration': max_duration,
                    'min_duration': min_duration,
                    'throughput': throughput,
                    'total_duration': total_duration * 1000,  # Convert to milliseconds
                    'failed_count': len(failed_tasks)
                }

                # Record concurrent performance metrics
                self.collector.record_metric(f"concurrent_{concurrent_count}_success_rate", success_rate, "%",
                                            "led_performance_benchmark", "concurrent")
                self.collector.record_metric(f"concurrent_{concurrent_count}_avg_duration", avg_duration, "ms",
                                            "led_performance_benchmark", "concurrent")
                self.collector.record_metric(f"concurrent_{concurrent_count}_throughput", throughput, "ops/s",
                                            "led_performance_benchmark", "concurrent")

                print(f"   Success rate: {success_rate:.1f}%, avg duration: {avg_duration:.2f}ms, throughput: {throughput:.1f} ops/s")

                # Verify performance requirement
                self.assertGreaterEqual(success_rate, 95.0,
                                       f"Concurrent level {concurrent_count} success rate too low: {success_rate:.1f}%")

                # Record failed tasks
                for failed_task in failed_tasks:
                    self.collector.record_error("led_performance_benchmark", "concurrent_task",
                                               f"Task {failed_task['task_id']} failed: {failed_task.get('error', 'Unknown')}")

            # Brief rest to avoid system overload
            time.sleep(0.5)

        # Analyze concurrent performance trends
        if len(performance_results) > 1:
            throughputs = [r['throughput'] for r in performance_results.values()]
            max_throughput = max(throughputs)
            optimal_concurrent = max(performance_results.keys(),
                                   key=lambda k: performance_results[k]['throughput'])

            self.collector.record_metric("max_throughput", max_throughput, "ops/s",
                                        "led_performance_benchmark", "concurrent")
            self.collector.record_metric("optimal_concurrent_level", optimal_concurrent, "count",
                                        "led_performance_benchmark", "concurrent")

            print(f"Concurrent performance analysis - max throughput: {max_throughput:.1f} ops/s (concurrent level: {optimal_concurrent})")

        print("Concurrent performance test passed")

    def test_memory_leak_detection(self):
        """Memory leak detection test"""
        self.assertLEDSystemReady()

        # Record initial memory
        initial_memory = self._get_memory_usage()
        memory_samples = [initial_memory]

        # Execute large number of LED operations
        operations_count = 1000
        sample_interval = 100  # Sample every 100 operations

        modes = ["wake_confirm", "processing_voice", "executing_action", "action_complete"]

        for i in range(operations_count):
            mode = modes[i % len(modes)]

            try:
                if hasattr(self.led_system, mode):
                    getattr(self.led_system, mode)()

                # Periodically sample memory usage
                if (i + 1) % sample_interval == 0:
                    current_memory = self._get_memory_usage()
                    memory_samples.append(current_memory)

                    memory_growth = current_memory - initial_memory
                    self.collector.record_metric(f"memory_at_operation_{i+1}", current_memory, "MB",
                                                "led_performance_benchmark", "memory_leak")

                    print(f"Operation {i+1}/{operations_count}: memory {current_memory:.2f}MB (growth: {memory_growth:+.2f}MB)")

            except Exception as e:
                self.collector.record_error("led_performance_benchmark", "memory_leak_test",
                                           f"Operation {i+1} failed: {e}")

        # Analyze memory growth trend
        if len(memory_samples) >= 3:
            final_memory = memory_samples[-1]
            total_growth = final_memory - initial_memory

            # Calculate memory growth rate
            memory_deltas = [memory_samples[i] - memory_samples[i-1]
                           for i in range(1, len(memory_samples))]
            avg_growth_per_sample = sum(memory_deltas) / len(memory_deltas)

            # Detect significant memory leak
            leak_threshold = self.config.stability.memory_leak_threshold_mb

            self.assertLessEqual(total_growth, leak_threshold,
                               f"Memory leak detected: total growth {total_growth:.2f}MB exceeds threshold {leak_threshold}MB")

            # Record memory leak analysis results
            self.collector.record_metric("total_memory_growth", total_growth, "MB",
                                        "led_performance_benchmark", "memory_leak")
            self.collector.record_metric("avg_growth_per_sample", avg_growth_per_sample, "MB",
                                        "led_performance_benchmark", "memory_leak")
            self.collector.record_metric("operations_tested", operations_count, "count",
                                        "led_performance_benchmark", "memory_leak")

            # Determine memory leak risk level
            if total_growth <= leak_threshold * 0.3:
                risk_level = "Low"
            elif total_growth <= leak_threshold * 0.7:
                risk_level = "Medium"
            else:
                risk_level = "High"

            print(f"Memory leak analysis:")
            print(f"   Total memory growth: {total_growth:.2f}MB")
            print(f"   Average growth rate: {avg_growth_per_sample:.3f}MB/sample")
            print(f"   Risk level: {risk_level}")

        print("Memory leak detection test passed")

    def test_performance_under_load(self):
        """Performance under load test"""
        self.assertLEDSystemReady()

        # Simulate system load
        def cpu_load_generator():
            """CPU load generator"""
            end_time = time.time() + 5.0  # Run for 5 seconds
            while time.time() < end_time:
                # Execute some CPU-intensive operations
                sum(i * i for i in range(1000))

        def memory_load_generator():
            """Memory load generator"""
            # Allocate some memory (not too much to affect the system)
            data = [list(range(1000)) for _ in range(100)]
            time.sleep(5.0)
            del data

        # Test LED performance under different load conditions
        load_conditions = [
            ("normal", None),
            ("cpu_load", cpu_load_generator),
            ("memory_load", memory_load_generator)
        ]

        for condition_name, load_generator in load_conditions:
            print(f"Testing load condition: {condition_name}")

            # Start load (if any)
            load_thread = None
            if load_generator:
                load_thread = threading.Thread(target=load_generator)
                load_thread.start()

            # Test LED performance
            led_response_times = []
            test_operations = 50

            for i in range(test_operations):
                mode = ["wake_confirm", "processing_voice", "executing_action"][i % 3]

                start_time = time.perf_counter()
                try:
                    if hasattr(self.led_system, mode):
                        getattr(self.led_system, mode)()
                    end_time = time.perf_counter()

                    response_time = (end_time - start_time) * 1000
                    led_response_times.append(response_time)

                except Exception as e:
                    self.collector.record_error("led_performance_benchmark", "load_test",
                                               f"Operation failed under load {condition_name}: {e}")

                time.sleep(0.05)  # Brief interval

            # Wait for load thread to finish
            if load_thread:
                load_thread.join()

            # Analyze performance under current load
            if led_response_times:
                avg_response = sum(led_response_times) / len(led_response_times)
                max_response = max(led_response_times)
                min_response = min(led_response_times)

                # Record performance metrics under load
                self.collector.record_metric(f"response_time_under_{condition_name}_avg", avg_response, "ms",
                                            "led_performance_benchmark", "load_test")
                self.collector.record_metric(f"response_time_under_{condition_name}_max", max_response, "ms",
                                            "led_performance_benchmark", "load_test")

                # Verify performance still acceptable under load
                acceptable_threshold = self.config.performance.max_response_time_ms * 1.5  # Allow 50% performance degradation
                self.assertLessEqual(avg_response, acceptable_threshold,
                                   f"Average response time too long under load {condition_name}: {avg_response:.2f}ms")

                print(f"   Average response time: {avg_response:.2f}ms, range: {min_response:.2f}-{max_response:.2f}ms")

        print("Performance under load test passed")

    def _get_memory_usage(self) -> float:
        """Get current process memory usage (MB)"""
        try:
            process = psutil.Process(os.getpid())
            memory_info = process.memory_info()
            return memory_info.rss / 1024 / 1024  # Convert to MB
        except Exception:
            return 0.0

    def _get_cpu_usage(self) -> float:
        """Get current CPU usage (%)"""
        try:
            return psutil.cpu_percent(interval=0.1)
        except Exception:
            return 0.0


class LEDPerformanceRegression(LEDTestBase):
    """LED performance regression tests"""

    def setUp(self):
        """Pre-test setup"""
        super().setUp()
        self.config = get_led_test_config()
        self.collector = get_led_test_collector()
        self.collector.start_test_session("led_performance_regression",
                                         {'test_type': 'regression', 'module': 'led_system'})

    def tearDown(self):
        """Post-test cleanup"""
        self.collector.end_test_session("led_performance_regression")
        super().tearDown()

    def test_performance_baseline_comparison(self):
        """Performance baseline comparison test"""
        self.assertLEDSystemReady()

        # Define performance baselines (these values should be established from prior tests)
        performance_baselines = {
            'wake_confirm_response_time': 50.0,      # ms
            'processing_voice_response_time': 45.0,  # ms
            'executing_action_response_time': 40.0,  # ms
            'action_complete_response_time': 55.0,   # ms
            'error_state_response_time': 30.0,       # ms
            'memory_usage_threshold': 150.0,         # MB
            'cpu_usage_threshold': 25.0              # %
        }

        # Test current performance
        current_performance = {}

        modes = ["wake_confirm", "processing_voice", "executing_action", "action_complete", "error_state"]

        for mode in modes:
            if not hasattr(self.led_system, mode):
                continue

            # Measure multiple times and average
            response_times = []
            for _ in range(20):
                start_time = time.perf_counter()
                try:
                    getattr(self.led_system, mode)()
                    end_time = time.perf_counter()
                    response_times.append((end_time - start_time) * 1000)
                except Exception as e:
                    self.collector.record_error("led_performance_regression", "baseline_test",
                                               f"Mode {mode} baseline test failed: {e}")
                time.sleep(0.01)

            if response_times:
                avg_response_time = sum(response_times) / len(response_times)
                current_performance[f"{mode}_response_time"] = avg_response_time

                # Compare with baseline
                baseline_key = f"{mode}_response_time"
                if baseline_key in performance_baselines:
                    baseline_value = performance_baselines[baseline_key]
                    performance_ratio = avg_response_time / baseline_value

                    # Allow 10% performance fluctuation
                    self.assertLessEqual(performance_ratio, 1.1,
                                       f"Mode {mode} performance regression: current {avg_response_time:.2f}ms > baseline {baseline_value:.2f}ms")

                    # Record performance comparison
                    self.collector.record_metric(f"{mode}_performance_ratio", performance_ratio, "ratio",
                                                "led_performance_regression", "comparison")

                    status = "PASS" if performance_ratio <= 1.0 else "WARN" if performance_ratio <= 1.1 else "FAIL"
                    print(f"{status} {mode}: {avg_response_time:.2f}ms (baseline: {baseline_value:.2f}ms, ratio: {performance_ratio:.2f})")

        # Check resource usage
        current_memory = self._get_memory_usage()
        current_cpu = self._get_cpu_usage()

        current_performance['memory_usage'] = current_memory
        current_performance['cpu_usage'] = current_cpu

        # Compare resource usage with baseline
        if current_memory > performance_baselines['memory_usage_threshold']:
            print(f"WARN: Memory usage exceeds baseline: {current_memory:.2f}MB > {performance_baselines['memory_usage_threshold']}MB")

        if current_cpu > performance_baselines['cpu_usage_threshold']:
            print(f"WARN: CPU usage exceeds baseline: {current_cpu:.1f}% > {performance_baselines['cpu_usage_threshold']}%")

        # Record complete performance report
        self.collector.record_metric("performance_baseline_check", "completed", "status",
                                    "led_performance_regression", "regression")

        print("Performance baseline comparison test complete")

    def _get_memory_usage(self) -> float:
        """Get current process memory usage (MB)"""
        try:
            process = psutil.Process(os.getpid())
            memory_info = process.memory_info()
            return memory_info.rss / 1024 / 1024
        except Exception:
            return 0.0

    def _get_cpu_usage(self) -> float:
        """Get current CPU usage (%)"""
        try:
            return psutil.cpu_percent(interval=0.1)
        except Exception:
            return 0.0


if __name__ == "__main__":
    # Create test suite
    suite = unittest.TestSuite()

    # Add benchmark tests
    suite.addTest(unittest.makeSuite(LEDPerformanceBenchmark))

    # Add regression tests
    suite.addTest(unittest.makeSuite(LEDPerformanceRegression))

    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    # Generate report
    collector = get_led_test_collector()
    collector.save_data("led_performance_test_results")

    print(f"\n{'='*60}")
    print(f"LED performance tests complete - success: {result.testsRun - len(result.failures) - len(result.errors)}, "
          f"failures: {len(result.failures)}, errors: {len(result.errors)}")
    print(f"{'='*60}")
