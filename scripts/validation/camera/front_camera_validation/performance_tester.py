#!/usr/bin/env python3
# scripts/validation/camera/front_camera_validation/performance_tester.py
# Generated: 2025-06-27
# Purpose: Unitree Go2 front camera real-time performance testing

import time
import threading
import statistics
import numpy as np
import logging
from typing import Dict, List, Tuple, Any, Optional
from dataclasses import dataclass, field
from collections import deque
from camera_config import CameraConfig

@dataclass
class PerformanceMetrics:
    """Performance metrics data class"""
    fps_actual: float = 0.0
    fps_target: float = 30.0
    frame_drop_rate: float = 0.0
    capture_success_rate: float = 0.0
    avg_latency_ms: float = 0.0
    max_latency_ms: float = 0.0
    min_latency_ms: float = 0.0
    latency_std_ms: float = 0.0
    cpu_usage_percent: float = 0.0
    memory_usage_mb: float = 0.0
    test_duration_seconds: float = 0.0
    total_frames_captured: int = 0
    total_frames_attempted: int = 0
    frame_times: List[float] = field(default_factory=list)
    latencies: List[float] = field(default_factory=list)

class PerformanceTester:
    """Front camera performance tester"""

    def __init__(self, camera_config: CameraConfig, config: Dict[str, Any] = None):
        """
        Initialize performance tester

        Args:
            camera_config: Camera configuration object
            config: Test configuration
        """
        self.camera_config = camera_config
        self.config = config or {}
        self.logger = logging.getLogger(__name__)

        # Performance data collection
        self.frame_times = deque(maxlen=1000)
        self.latencies = deque(maxlen=1000)
        self.capture_results = deque(maxlen=1000)

        # Test control
        self.is_testing = False
        self.test_start_time = 0
        self.test_thread = None

        # Real-time statistics
        self.frames_captured = 0
        self.frames_attempted = 0
        self.last_fps_calculation = 0
        self.current_fps = 0.0

    def run_basic_performance_test(self, duration_seconds: float = 30.0) -> PerformanceMetrics:
        """
        Run basic performance test

        Args:
            duration_seconds: Test duration (seconds)

        Returns:
            PerformanceMetrics: Performance test results
        """
        self.logger.info(f"Starting basic performance test, duration: {duration_seconds}s")

        metrics = PerformanceMetrics()
        metrics.test_duration_seconds = duration_seconds

        # Get performance thresholds
        thresholds = self.config.get("performance_thresholds", {})
        metrics.fps_target = thresholds.get("target_fps", 30.0)

        # Reset counters
        self._reset_counters()

        start_time = time.time()
        end_time = start_time + duration_seconds

        self.logger.info("Performance test started...")

        while time.time() < end_time:
            # Record frame capture start time
            capture_start = time.time()

            # Attempt to capture frame
            ret, frame = self.camera_config.capture_frame()
            capture_end = time.time()

            # Calculate latency
            latency_ms = (capture_end - capture_start) * 1000

            # Record results
            self.frames_attempted += 1
            if ret and frame is not None:
                self.frames_captured += 1
                self.frame_times.append(capture_end)
                self.latencies.append(latency_ms)
                self.capture_results.append(True)
            else:
                self.capture_results.append(False)

            # Control test frequency to approach target frame rate
            expected_interval = 1.0 / metrics.fps_target
            elapsed = capture_end - capture_start
            if elapsed < expected_interval:
                time.sleep(expected_interval - elapsed)

        # Calculate performance metrics
        metrics = self._calculate_metrics(metrics, start_time, time.time())

        self.logger.info("Basic performance test complete")
        self.logger.info(f"Actual FPS: {metrics.fps_actual:.2f}, avg latency: {metrics.avg_latency_ms:.2f}ms")

        return metrics

    def run_stress_test(self, duration_seconds: float = 60.0, high_frequency: bool = True) -> PerformanceMetrics:
        """
        Run stress test

        Args:
            duration_seconds: Test duration (seconds)
            high_frequency: Whether to use high-frequency capture

        Returns:
            PerformanceMetrics: Stress test results
        """
        self.logger.info(f"Starting stress test, duration: {duration_seconds}s")

        metrics = PerformanceMetrics()
        metrics.test_duration_seconds = duration_seconds
        metrics.fps_target = 60.0 if high_frequency else 30.0

        # Reset counters
        self._reset_counters()

        start_time = time.time()
        end_time = start_time + duration_seconds

        # Multi-threaded stress test
        stress_threads = []
        for i in range(2):  # Start two concurrent capture threads
            thread = threading.Thread(target=self._stress_capture_worker, args=(end_time,))
            stress_threads.append(thread)
            thread.start()

        # Wait for all threads to complete
        for thread in stress_threads:
            thread.join()

        # Calculate performance metrics
        metrics = self._calculate_metrics(metrics, start_time, time.time())

        self.logger.info("Stress test complete")
        self.logger.info(f"Stress test FPS: {metrics.fps_actual:.2f}, frame drop rate: {metrics.frame_drop_rate:.2%}")

        return metrics

    def _stress_capture_worker(self, end_time: float):
        """Stress test worker thread"""
        while time.time() < end_time:
            capture_start = time.time()
            ret, frame = self.camera_config.capture_frame()
            capture_end = time.time()

            latency_ms = (capture_end - capture_start) * 1000

            # Thread-safe counter update
            with threading.Lock():
                self.frames_attempted += 1
                if ret and frame is not None:
                    self.frames_captured += 1
                    self.frame_times.append(capture_end)
                    self.latencies.append(latency_ms)
                    self.capture_results.append(True)
                else:
                    self.capture_results.append(False)

    def run_realtime_monitoring(self, duration_seconds: float = 10.0,
                              callback=None) -> PerformanceMetrics:
        """
        Run real-time monitoring test

        Args:
            duration_seconds: Monitoring duration (seconds)
            callback: Real-time data callback function

        Returns:
            PerformanceMetrics: Real-time monitoring results
        """
        self.logger.info(f"Starting real-time monitoring, duration: {duration_seconds}s")

        metrics = PerformanceMetrics()
        metrics.test_duration_seconds = duration_seconds

        # Reset counters
        self._reset_counters()

        start_time = time.time()
        end_time = start_time + duration_seconds
        last_report_time = start_time

        while time.time() < end_time:
            capture_start = time.time()
            ret, frame = self.camera_config.capture_frame()
            capture_end = time.time()

            latency_ms = (capture_end - capture_start) * 1000

            # Record data
            self.frames_attempted += 1
            if ret and frame is not None:
                self.frames_captured += 1
                self.frame_times.append(capture_end)
                self.latencies.append(latency_ms)
                self.capture_results.append(True)
            else:
                self.capture_results.append(False)

            # Report real-time data every second
            if capture_end - last_report_time >= 1.0:
                current_metrics = self._calculate_current_metrics(start_time, capture_end)
                if callback:
                    callback(current_metrics)
                else:
                    self.logger.info(f"Real-time FPS: {current_metrics.fps_actual:.1f}, "
                                   f"latency: {current_metrics.avg_latency_ms:.1f}ms")
                last_report_time = capture_end

            time.sleep(0.01)  # Brief sleep to avoid excessive CPU usage

        # Final calculation
        metrics = self._calculate_metrics(metrics, start_time, time.time())

        self.logger.info("Real-time monitoring complete")
        return metrics

    def run_latency_test(self, samples: int = 100) -> PerformanceMetrics:
        """
        Run dedicated latency test

        Args:
            samples: Number of test samples

        Returns:
            PerformanceMetrics: Latency test results
        """
        self.logger.info(f"Starting dedicated latency test, sample count: {samples}")

        metrics = PerformanceMetrics()
        latency_samples = []

        start_time = time.time()

        for i in range(samples):
            # High-precision time measurement
            capture_start = time.perf_counter()
            ret, frame = self.camera_config.capture_frame()
            capture_end = time.perf_counter()

            if ret and frame is not None:
                latency_ms = (capture_end - capture_start) * 1000
                latency_samples.append(latency_ms)

            # Appropriate interval
            time.sleep(0.05)

        end_time = time.time()

        if latency_samples:
            metrics.avg_latency_ms = statistics.mean(latency_samples)
            metrics.min_latency_ms = min(latency_samples)
            metrics.max_latency_ms = max(latency_samples)
            metrics.latency_std_ms = statistics.stdev(latency_samples) if len(latency_samples) > 1 else 0
            metrics.latencies = latency_samples
            metrics.capture_success_rate = len(latency_samples) / samples

        metrics.test_duration_seconds = end_time - start_time
        metrics.total_frames_attempted = samples
        metrics.total_frames_captured = len(latency_samples)

        self.logger.info(f"Latency test complete: avg {metrics.avg_latency_ms:.2f}ms, "
                        f"range {metrics.min_latency_ms:.2f}-{metrics.max_latency_ms:.2f}ms")

        return metrics

    def _reset_counters(self):
        """Reset counters"""
        self.frame_times.clear()
        self.latencies.clear()
        self.capture_results.clear()
        self.frames_captured = 0
        self.frames_attempted = 0
        self.current_fps = 0.0

    def _calculate_metrics(self, metrics: PerformanceMetrics,
                          start_time: float, end_time: float) -> PerformanceMetrics:
        """Calculate performance metrics"""
        duration = end_time - start_time

        # Basic statistics
        metrics.total_frames_attempted = self.frames_attempted
        metrics.total_frames_captured = self.frames_captured
        metrics.test_duration_seconds = duration

        if self.frames_attempted > 0:
            metrics.capture_success_rate = self.frames_captured / self.frames_attempted
            metrics.frame_drop_rate = 1.0 - metrics.capture_success_rate

        if duration > 0 and self.frames_captured > 0:
            metrics.fps_actual = self.frames_captured / duration

        # Latency statistics
        if self.latencies:
            latency_list = list(self.latencies)
            metrics.avg_latency_ms = statistics.mean(latency_list)
            metrics.min_latency_ms = min(latency_list)
            metrics.max_latency_ms = max(latency_list)
            metrics.latency_std_ms = statistics.stdev(latency_list) if len(latency_list) > 1 else 0
            metrics.latencies = latency_list

        # Frame time statistics
        if self.frame_times:
            metrics.frame_times = list(self.frame_times)

        return metrics

    def _calculate_current_metrics(self, start_time: float, current_time: float) -> PerformanceMetrics:
        """Calculate performance metrics at current moment"""
        metrics = PerformanceMetrics()

        duration = current_time - start_time
        if duration > 0 and self.frames_captured > 0:
            metrics.fps_actual = self.frames_captured / duration

        if self.latencies:
            recent_latencies = list(self.latencies)[-10:]  # Latency of last 10 frames
            metrics.avg_latency_ms = statistics.mean(recent_latencies)

        if self.frames_attempted > 0:
            metrics.capture_success_rate = self.frames_captured / self.frames_attempted

        return metrics

    def evaluate_performance(self, metrics: PerformanceMetrics) -> Dict[str, Any]:
        """
        Evaluate performance results

        Args:
            metrics: Performance metrics

        Returns:
            Dict[str, Any]: Evaluation results
        """
        thresholds = self.config.get("performance_thresholds", {})

        evaluation = {
            "overall_status": "PASS",
            "issues": [],
            "recommendations": [],
            "scores": {}
        }

        # FPS evaluation
        min_fps = thresholds.get("min_fps", 20)
        target_fps = thresholds.get("target_fps", 30)

        if metrics.fps_actual < min_fps:
            evaluation["overall_status"] = "FAIL"
            evaluation["issues"].append(f"FPS too low: {metrics.fps_actual:.1f} < {min_fps}")
        elif metrics.fps_actual < target_fps:
            evaluation["issues"].append(f"FPS below target: {metrics.fps_actual:.1f} < {target_fps}")

        fps_score = min(100, (metrics.fps_actual / target_fps) * 100)
        evaluation["scores"]["fps"] = fps_score

        # Latency evaluation
        max_latency = thresholds.get("max_latency_ms", 100)

        if metrics.avg_latency_ms > max_latency:
            evaluation["overall_status"] = "FAIL"
            evaluation["issues"].append(f"Latency too high: {metrics.avg_latency_ms:.1f}ms > {max_latency}ms")

        latency_score = max(0, 100 - (metrics.avg_latency_ms / max_latency) * 100)
        evaluation["scores"]["latency"] = latency_score

        # Stability evaluation
        max_drop_rate = thresholds.get("max_frame_drop_rate", 0.05)
        min_success_rate = thresholds.get("min_capture_success_rate", 0.95)

        if metrics.frame_drop_rate > max_drop_rate:
            evaluation["issues"].append(f"Frame drop rate too high: {metrics.frame_drop_rate:.1%} > {max_drop_rate:.1%}")

        if metrics.capture_success_rate < min_success_rate:
            evaluation["overall_status"] = "FAIL"
            evaluation["issues"].append(f"Capture success rate too low: {metrics.capture_success_rate:.1%} < {min_success_rate:.1%}")

        stability_score = metrics.capture_success_rate * 100
        evaluation["scores"]["stability"] = stability_score

        # Generate recommendations
        if fps_score < 80:
            evaluation["recommendations"].append("Consider lowering resolution or adjusting camera parameters to improve frame rate")

        if latency_score < 80:
            evaluation["recommendations"].append("Optimize image capture pipeline to reduce latency")

        if stability_score < 95:
            evaluation["recommendations"].append("Check camera connection and driver status")

        # Overall score
        evaluation["scores"]["overall"] = (fps_score + latency_score + stability_score) / 3

        return evaluation

# Test function
def test_performance_tester():
    """Test performance tester functionality"""
    import logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    # Create camera configuration
    with CameraConfig() as camera_config:
        if not camera_config.initialize_camera():
            print("Camera initialization failed, unable to perform performance test")
            return

        # Create performance tester
        tester = PerformanceTester(camera_config)

        # Run basic performance test
        print("Running basic performance test...")
        metrics = tester.run_basic_performance_test(duration_seconds=10.0)

        # Evaluate performance
        evaluation = tester.evaluate_performance(metrics)

        print(f"\nPerformance test results:")
        print(f"Actual FPS: {metrics.fps_actual:.2f}")
        print(f"Average latency: {metrics.avg_latency_ms:.2f}ms")
        print(f"Capture success rate: {metrics.capture_success_rate:.1%}")
        print(f"Overall score: {evaluation['scores']['overall']:.1f}")
        print(f"Evaluation status: {evaluation['overall_status']}")

if __name__ == "__main__":
    test_performance_tester()
