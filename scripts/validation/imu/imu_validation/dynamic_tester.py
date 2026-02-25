#!/usr/bin/env python3
# scripts/validation/imu/imu_validation/dynamic_tester.py
# Generated: 2025-06-27 11:54:45 CST
# Purpose: Unitree Go2 IMU dynamic response testing

import time
import statistics
import numpy as np
import logging
from typing import Dict, List, Tuple, Any, Optional
from dataclasses import dataclass, field
import threading

from imu_config import IMUConfig, IMUReading
from data_collector import IMUDataCollector, CollectionMetrics

@dataclass
class DynamicTestResults:
    """Dynamic test results"""
    test_duration: float = 0.0
    test_type: str = ""
    sample_count: int = 0
    valid_samples: int = 0

    # Responsiveness metrics
    response_time_ms: float = 0.0
    rise_time_ms: float = 0.0
    settling_time_ms: float = 0.0
    overshoot_percent: float = 0.0

    # Dynamic accuracy metrics
    tracking_accuracy: Dict[str, float] = field(default_factory=dict)
    dynamic_range: Dict[str, float] = field(default_factory=dict)
    linearity: Dict[str, float] = field(default_factory=dict)

    # Frequency response
    frequency_response: Dict[str, float] = field(default_factory=dict)

    # Test evaluation
    test_status: str = "UNKNOWN"
    pass_criteria: Dict[str, bool] = field(default_factory=dict)
    recommendations: List[str] = field(default_factory=list)

class IMUDynamicTester:
    """IMU dynamic tester"""

    def __init__(self, imu_config: IMUConfig, data_collector: IMUDataCollector, config: Dict[str, Any]):
        """
        Initialize dynamic tester

        Args:
            imu_config: IMU configuration instance
            data_collector: Data collector instance
            config: Validation configuration
        """
        self.imu_config = imu_config
        self.data_collector = data_collector
        self.config = config
        self.logger = logging.getLogger(__name__)

        # Test configuration
        self.test_config = config.get("test_parameters", {}).get("dynamic_test", {})
        self.quality_thresholds = config.get("quality_thresholds", {})

        # Test parameters
        self.test_duration = self.test_config.get("duration_seconds", 120)
        self.response_tests = self.test_config.get("response_tests", ["pitch_test", "roll_test", "yaw_test", "translation_test"])
        self.response_threshold_ms = self.test_config.get("response_threshold_ms", 50)

    def run_dynamic_response_test(self, test_type: str = "comprehensive") -> Dict[str, DynamicTestResults]:
        """
        Run dynamic response test

        Args:
            test_type: Test type ("comprehensive", "quick", "specific")

        Returns:
            Dict[str, DynamicTestResults]: Results for each test type
        """
        self.logger.info("=" * 50)
        self.logger.info("Starting IMU dynamic response test")
        self.logger.info("=" * 50)

        all_results = {}

        try:
            # Ensure IMU is initialized
            if not self.imu_config.is_initialized:
                self.logger.error("IMU is not initialized, cannot perform dynamic test")
                return all_results

            if test_type == "comprehensive":
                test_list = self.response_tests
            elif test_type == "quick":
                test_list = ["pitch_test", "translation_test"]  # Quick test
            else:
                test_list = [test_type] if test_type in self.response_tests else ["pitch_test"]

            for test_name in test_list:
                self.logger.info(f"\nRunning {test_name}...")
                result = self._run_single_dynamic_test(test_name)
                all_results[test_name] = result

                # Brief rest
                if len(test_list) > 1:
                    self.logger.info("Continuing to next test in 5 seconds...")
                    time.sleep(5)

            # Generate comprehensive assessment
            overall_result = self._generate_overall_assessment(all_results)
            all_results["overall"] = overall_result

            self.logger.info("=" * 50)
            self.logger.info("Dynamic tests complete")
            self.logger.info("=" * 50)

            return all_results

        except Exception as e:
            self.logger.error(f"Dynamic test failed: {e}")
            return all_results

    def _run_single_dynamic_test(self, test_name: str) -> DynamicTestResults:
        """Run a single dynamic test"""
        result = DynamicTestResults()
        result.test_type = test_name

        try:
            self.logger.info(f"Preparing for {test_name} test")

            # Get test instructions
            test_instructions = self._get_test_instructions(test_name)
            self.logger.info(f"Test instructions: {test_instructions}")

            # Wait for user preparation
            self.logger.info("Test will start in 10 seconds, please prepare...")
            time.sleep(10)

            # Start data collection
            self.logger.info("Starting data collection...")
            test_duration = min(60, self.test_duration // len(self.response_tests))  # Per-test duration

            success = self.data_collector.start_collection(test_duration)

            if not success:
                self.logger.error("Unable to start data collection")
                result.test_status = "FAIL"
                return result

            # Monitor test progress
            self._monitor_test_progress(test_duration, test_name)

            # Stop collection and get data
            collection_metrics = self.data_collector.stop_collection()
            collected_data = self.data_collector.get_collected_data()

            if not collected_data:
                self.logger.error("No data collected")
                result.test_status = "FAIL"
                return result

            self.logger.info(f"{test_name} collection complete, obtained {len(collected_data)} samples")

            # Analyze dynamic response
            result = self._analyze_dynamic_response(collected_data, collection_metrics, test_name)

            # Evaluate test results
            self._evaluate_dynamic_results(result)

            return result

        except Exception as e:
            self.logger.error(f"{test_name} test failed: {e}")
            result.test_status = "ERROR"
            return result

    def _get_test_instructions(self, test_name: str) -> str:
        """Get test instructions"""
        instructions = {
            "pitch_test": "Please slowly tilt the robot forward and backward (pitch axis rotation), then quickly return to level position",
            "roll_test": "Please slowly tilt the robot left and right (roll axis rotation), then quickly return to level position",
            "yaw_test": "Please slowly rotate the robot left and right (yaw axis rotation), then quickly return to original direction",
            "translation_test": "Please smoothly move the robot forward/backward and left/right, avoiding rotation"
        }

        return instructions.get(test_name, "Perform the test according to standard procedures")

    def _monitor_test_progress(self, duration: float, test_name: str):
        """Monitor test progress"""
        start_time = time.time()

        while self.data_collector.is_collecting:
            elapsed = time.time() - start_time

            if elapsed >= duration + 10:  # Extra 10 second timeout
                self.logger.warning(f"{test_name} test timed out, forcing stop")
                break

            # Show progress and real-time metrics
            if int(elapsed) % 10 == 0:
                metrics = self.data_collector.get_real_time_metrics()
                self.logger.info(f"{test_name} progress: {elapsed:.1f}/{duration} seconds, sampling rate: {metrics.get('current_fps', 0):.1f}Hz")

            time.sleep(1)

    def _analyze_dynamic_response(self, data: List[IMUReading], collection_metrics: CollectionMetrics, test_name: str) -> DynamicTestResults:
        """Analyze dynamic response data"""
        result = DynamicTestResults()
        result.test_type = test_name
        result.test_duration = collection_metrics.collection_duration
        result.sample_count = len(data)
        result.valid_samples = collection_metrics.valid_samples

        try:
            # Extract sensor data
            timestamps = np.array([r.timestamp for r in data])
            accel_data = np.array([[r.accelerometer[0], r.accelerometer[1], r.accelerometer[2]] for r in data])
            gyro_data = np.array([[r.gyroscope[0], r.gyroscope[1], r.gyroscope[2]] for r in data])
            quat_data = np.array([[r.quaternion[0], r.quaternion[1], r.quaternion[2], r.quaternion[3]] for r in data])

            # Convert to Euler angles
            euler_data = []
            for quat in quat_data:
                roll, pitch, yaw = self.imu_config.quaternion_to_euler(tuple(quat))
                euler_data.append([np.degrees(roll), np.degrees(pitch), np.degrees(yaw)])
            euler_data = np.array(euler_data)

            # Analyze response based on test type
            if test_name == "pitch_test":
                result = self._analyze_rotation_response(result, timestamps, euler_data[:, 1], gyro_data[:, 1], "pitch")
            elif test_name == "roll_test":
                result = self._analyze_rotation_response(result, timestamps, euler_data[:, 0], gyro_data[:, 0], "roll")
            elif test_name == "yaw_test":
                result = self._analyze_rotation_response(result, timestamps, euler_data[:, 2], gyro_data[:, 2], "yaw")
            elif test_name == "translation_test":
                result = self._analyze_translation_response(result, timestamps, accel_data)

            # General dynamic analysis
            result = self._analyze_general_dynamics(result, timestamps, accel_data, gyro_data, euler_data)

            return result

        except Exception as e:
            self.logger.error(f"Dynamic response analysis failed: {e}")
            result.test_status = "ERROR"
            return result

    def _analyze_rotation_response(self, result: DynamicTestResults, timestamps: np.ndarray,
                                 angle_data: np.ndarray, gyro_data: np.ndarray, axis: str) -> DynamicTestResults:
        """Analyze rotation response"""
        try:
            # Detect motion events
            motion_events = self._detect_motion_events(angle_data, threshold=5.0)  # 5 degree threshold

            if motion_events:
                # Analyze the largest motion event
                largest_event = max(motion_events, key=lambda x: x['magnitude'])

                # Calculate response time
                gyro_response_time = self._calculate_gyro_response_time(
                    timestamps, gyro_data, largest_event['start_idx'], largest_event['end_idx']
                )

                result.response_time_ms = gyro_response_time * 1000

                # Analyze step response characteristics
                step_response = self._analyze_step_response(
                    timestamps[largest_event['start_idx']:largest_event['end_idx']],
                    angle_data[largest_event['start_idx']:largest_event['end_idx']]
                )

                result.rise_time_ms = step_response.get('rise_time', 0) * 1000
                result.settling_time_ms = step_response.get('settling_time', 0) * 1000
                result.overshoot_percent = step_response.get('overshoot_percent', 0)

                # Tracking accuracy
                result.tracking_accuracy[f'{axis}_correlation'] = self._calculate_tracking_correlation(angle_data, gyro_data, timestamps)
                result.tracking_accuracy[f'{axis}_phase_delay_ms'] = self._calculate_phase_delay(angle_data, gyro_data, timestamps) * 1000

            # Dynamic range
            result.dynamic_range[f'{axis}_angle_range'] = np.max(angle_data) - np.min(angle_data)
            result.dynamic_range[f'{axis}_gyro_range'] = np.max(gyro_data) - np.min(gyro_data)

            return result

        except Exception as e:
            self.logger.error(f"Rotation response analysis failed: {e}")
            return result

    def _analyze_translation_response(self, result: DynamicTestResults, timestamps: np.ndarray,
                                    accel_data: np.ndarray) -> DynamicTestResults:
        """Analyze translation response"""
        try:
            # Calculate acceleration magnitude
            accel_magnitude = np.linalg.norm(accel_data, axis=1)

            # Detect acceleration events
            baseline = np.median(accel_magnitude)
            motion_threshold = baseline * 0.1  # 10% change threshold

            motion_events = self._detect_acceleration_events(accel_magnitude, motion_threshold)

            if motion_events:
                # Analyze the largest acceleration event
                largest_event = max(motion_events, key=lambda x: x['magnitude'])

                # Calculate response time (time from acceleration change to stabilization)
                response_time = self._calculate_acceleration_response_time(
                    timestamps, accel_magnitude, largest_event['start_idx'], largest_event['end_idx']
                )

                result.response_time_ms = response_time * 1000

            # Translation dynamic range
            for i, axis in enumerate(['x', 'y', 'z']):
                result.dynamic_range[f'accel_{axis}_range'] = np.max(accel_data[:, i]) - np.min(accel_data[:, i])

            result.dynamic_range['accel_magnitude_range'] = np.max(accel_magnitude) - np.min(accel_magnitude)

            # Linearity analysis (linear relationship between acceleration and expected values)
            result.linearity['acceleration_linearity'] = self._calculate_acceleration_linearity(accel_data)

            return result

        except Exception as e:
            self.logger.error(f"Translation response analysis failed: {e}")
            return result

    def _analyze_general_dynamics(self, result: DynamicTestResults, timestamps: np.ndarray,
                                accel_data: np.ndarray, gyro_data: np.ndarray, euler_data: np.ndarray) -> DynamicTestResults:
        """General dynamic analysis"""
        try:
            # Frequency response analysis
            sampling_rate = len(timestamps) / (timestamps[-1] - timestamps[0])

            # Analyze frequency response for each axis
            for i, axis in enumerate(['x', 'y', 'z']):
                # Gyroscope frequency response
                gyro_freq_response = self._analyze_frequency_response(gyro_data[:, i], sampling_rate)
                result.frequency_response[f'gyro_{axis}_bandwidth_hz'] = gyro_freq_response.get('bandwidth', 0)
                result.frequency_response[f'gyro_{axis}_peak_freq_hz'] = gyro_freq_response.get('peak_frequency', 0)

                # Accelerometer frequency response
                accel_freq_response = self._analyze_frequency_response(accel_data[:, i], sampling_rate)
                result.frequency_response[f'accel_{axis}_bandwidth_hz'] = accel_freq_response.get('bandwidth', 0)
                result.frequency_response[f'accel_{axis}_peak_freq_hz'] = accel_freq_response.get('peak_frequency', 0)

            return result

        except Exception as e:
            self.logger.error(f"General dynamic analysis failed: {e}")
            return result

    def _detect_motion_events(self, data: np.ndarray, threshold: float) -> List[Dict[str, Any]]:
        """Detect motion events"""
        try:
            events = []

            # Calculate rate of change
            diff = np.abs(np.diff(data))
            motion_mask = diff > threshold

            # Find continuous motion segments
            in_motion = False
            start_idx = 0

            for i, is_moving in enumerate(motion_mask):
                if is_moving and not in_motion:
                    # Motion start
                    start_idx = i
                    in_motion = True
                elif not is_moving and in_motion:
                    # Motion end
                    end_idx = i
                    magnitude = np.max(np.abs(data[start_idx:end_idx])) - np.min(np.abs(data[start_idx:end_idx]))

                    events.append({
                        'start_idx': start_idx,
                        'end_idx': end_idx,
                        'magnitude': magnitude,
                        'duration': end_idx - start_idx
                    })

                    in_motion = False

            return events

        except Exception as e:
            self.logger.error(f"Motion event detection failed: {e}")
            return []

    def _detect_acceleration_events(self, data: np.ndarray, threshold: float) -> List[Dict[str, Any]]:
        """Detect acceleration events"""
        try:
            events = []
            baseline = np.median(data)

            # Detect regions with significant deviation from baseline
            deviation = np.abs(data - baseline)
            event_mask = deviation > threshold

            # Find continuous event segments
            in_event = False
            start_idx = 0

            for i, is_event in enumerate(event_mask):
                if is_event and not in_event:
                    start_idx = i
                    in_event = True
                elif not is_event and in_event:
                    end_idx = i
                    magnitude = np.max(data[start_idx:end_idx]) - np.min(data[start_idx:end_idx])

                    events.append({
                        'start_idx': start_idx,
                        'end_idx': end_idx,
                        'magnitude': magnitude,
                        'duration': end_idx - start_idx
                    })

                    in_event = False

            return events

        except Exception as e:
            self.logger.error(f"Acceleration event detection failed: {e}")
            return []

    def _calculate_gyro_response_time(self, timestamps: np.ndarray, gyro_data: np.ndarray,
                                    start_idx: int, end_idx: int) -> float:
        """Calculate gyroscope response time"""
        try:
            if end_idx <= start_idx:
                return 0.0

            # Find the time point when the gyroscope signal starts to change
            segment = gyro_data[start_idx:end_idx]
            baseline = np.mean(gyro_data[max(0, start_idx-10):start_idx])

            # Find the time point when the signal reaches 10% of the change amplitude
            peak_value = np.max(np.abs(segment))
            threshold = baseline + 0.1 * (peak_value - baseline)

            response_idx = start_idx
            for i, value in enumerate(segment):
                if abs(value) > threshold:
                    response_idx = start_idx + i
                    break

            response_time = timestamps[response_idx] - timestamps[start_idx]
            return max(0, response_time)

        except Exception as e:
            self.logger.error(f"Gyroscope response time calculation failed: {e}")
            return 0.0

    def _calculate_acceleration_response_time(self, timestamps: np.ndarray, accel_data: np.ndarray,
                                           start_idx: int, end_idx: int) -> float:
        """Calculate acceleration response time"""
        try:
            if end_idx <= start_idx:
                return 0.0

            # Simplified response time calculation
            segment_duration = timestamps[end_idx] - timestamps[start_idx]

            # Assume response time is 10% of event duration (empirical value)
            response_time = segment_duration * 0.1

            return response_time

        except Exception as e:
            self.logger.error(f"Acceleration response time calculation failed: {e}")
            return 0.0

    def _analyze_step_response(self, timestamps: np.ndarray, data: np.ndarray) -> Dict[str, float]:
        """Analyze step response characteristics"""
        try:
            if len(data) < 10:
                return {}

            # Simplified step response analysis
            initial_value = np.mean(data[:5])
            final_value = np.mean(data[-5:])
            step_size = abs(final_value - initial_value)

            if step_size < 1e-6:
                return {}

            # Rise time (10% to 90%)
            threshold_10 = initial_value + 0.1 * step_size
            threshold_90 = initial_value + 0.9 * step_size

            idx_10 = np.argmax(np.abs(data - initial_value) >= 0.1 * step_size)
            idx_90 = np.argmax(np.abs(data - initial_value) >= 0.9 * step_size)

            rise_time = timestamps[idx_90] - timestamps[idx_10] if idx_90 > idx_10 else 0

            # Overshoot
            if final_value > initial_value:
                overshoot = np.max(data) - final_value
            else:
                overshoot = final_value - np.min(data)

            overshoot_percent = (overshoot / step_size) * 100 if step_size > 0 else 0

            # Settling time (simplified to 80% of data length)
            settling_time = (timestamps[-1] - timestamps[0]) * 0.8

            return {
                'rise_time': rise_time,
                'settling_time': settling_time,
                'overshoot_percent': overshoot_percent
            }

        except Exception as e:
            self.logger.error(f"Step response analysis failed: {e}")
            return {}

    def _calculate_tracking_correlation(self, angle_data: np.ndarray, gyro_data: np.ndarray,
                                      timestamps: np.ndarray) -> float:
        """Calculate tracking correlation"""
        try:
            # Calculate numerical derivative of angle
            dt = np.mean(np.diff(timestamps))
            angle_rate = np.gradient(angle_data, dt)

            # Convert to same units (degrees/second vs radians/second)
            gyro_deg = np.degrees(gyro_data)

            # Calculate correlation coefficient
            correlation = np.corrcoef(angle_rate, gyro_deg)[0, 1]

            return correlation if not np.isnan(correlation) else 0.0

        except Exception as e:
            self.logger.error(f"Tracking correlation calculation failed: {e}")
            return 0.0

    def _calculate_phase_delay(self, angle_data: np.ndarray, gyro_data: np.ndarray,
                             timestamps: np.ndarray) -> float:
        """Calculate phase delay"""
        try:
            # Simplified cross-correlation analysis
            dt = np.mean(np.diff(timestamps))
            angle_rate = np.gradient(angle_data, dt)
            gyro_deg = np.degrees(gyro_data)

            # Calculate cross-correlation
            correlation = np.correlate(angle_rate, gyro_deg, mode='full')
            delay_samples = np.argmax(correlation) - len(gyro_deg) + 1
            delay_time = delay_samples * dt

            return delay_time

        except Exception as e:
            self.logger.error(f"Phase delay calculation failed: {e}")
            return 0.0

    def _calculate_acceleration_linearity(self, accel_data: np.ndarray) -> float:
        """Calculate acceleration linearity"""
        try:
            # Simplified linearity analysis: calculate linear correlation between axes
            correlations = []

            for i in range(3):
                for j in range(i+1, 3):
                    corr = np.corrcoef(accel_data[:, i], accel_data[:, j])[0, 1]
                    if not np.isnan(corr):
                        correlations.append(abs(corr))

            # Return average linearity
            return np.mean(correlations) if correlations else 0.0

        except Exception as e:
            self.logger.error(f"Acceleration linearity calculation failed: {e}")
            return 0.0

    def _analyze_frequency_response(self, data: np.ndarray, sampling_rate: float) -> Dict[str, float]:
        """Analyze frequency response"""
        try:
            # Simplified frequency analysis
            fft = np.fft.fft(data)
            freqs = np.fft.fftfreq(len(data), 1/sampling_rate)

            # Consider only positive frequencies
            positive_freqs = freqs[:len(freqs)//2]
            magnitude = np.abs(fft[:len(fft)//2])

            # Find peak frequency
            peak_idx = np.argmax(magnitude)
            peak_frequency = positive_freqs[peak_idx]

            # Estimate bandwidth (-3dB point)
            peak_magnitude = magnitude[peak_idx]
            threshold = peak_magnitude * 0.707  # -3dB

            bandwidth_indices = np.where(magnitude >= threshold)[0]
            if len(bandwidth_indices) > 1:
                bandwidth = positive_freqs[bandwidth_indices[-1]] - positive_freqs[bandwidth_indices[0]]
            else:
                bandwidth = 0

            return {
                'peak_frequency': peak_frequency,
                'bandwidth': bandwidth,
                'peak_magnitude': peak_magnitude
            }

        except Exception as e:
            self.logger.error(f"Frequency response analysis failed: {e}")
            return {}

    def _generate_overall_assessment(self, all_results: Dict[str, DynamicTestResults]) -> DynamicTestResults:
        """Generate comprehensive assessment"""
        overall = DynamicTestResults()
        overall.test_type = "overall_assessment"

        try:
            # Collect all test statuses
            statuses = [result.test_status for result in all_results.values()]

            # Determine overall status
            if all(status == "PASS" for status in statuses):
                overall.test_status = "PASS"
            elif any(status == "FAIL" for status in statuses):
                overall.test_status = "FAIL"
            elif any(status == "WARNING" for status in statuses):
                overall.test_status = "WARNING"
            else:
                overall.test_status = "UNKNOWN"

            # Summarize response times
            response_times = [result.response_time_ms for result in all_results.values() if result.response_time_ms > 0]
            if response_times:
                overall.response_time_ms = statistics.mean(response_times)

            # Summarize recommendations
            all_recommendations = []
            for result in all_results.values():
                all_recommendations.extend(result.recommendations)

            overall.recommendations = list(set(all_recommendations))  # Deduplicate

            if not overall.recommendations:
                overall.recommendations = ["All dynamic tests meet requirements"]

            return overall

        except Exception as e:
            self.logger.error(f"Comprehensive assessment generation failed: {e}")
            overall.test_status = "ERROR"
            return overall

    def _evaluate_dynamic_results(self, result: DynamicTestResults):
        """Evaluate dynamic test results"""
        try:
            pass_criteria = {}
            recommendations = []

            # Evaluate response time
            response_time_pass = result.response_time_ms <= self.response_threshold_ms
            pass_criteria['response_time'] = response_time_pass

            if not response_time_pass:
                recommendations.append(f"Response time is too long ({result.response_time_ms:.1f}ms), check sensor bandwidth and filter settings")

            # Evaluate tracking accuracy
            tracking_correlations = [v for k, v in result.tracking_accuracy.items() if 'correlation' in k]
            if tracking_correlations:
                avg_correlation = statistics.mean(tracking_correlations)
                tracking_pass = avg_correlation > 0.8  # 80% correlation threshold
                pass_criteria['tracking_accuracy'] = tracking_pass

                if not tracking_pass:
                    recommendations.append("Tracking accuracy is insufficient, check sensor calibration and data fusion algorithm")

            # Evaluate dynamic range
            angle_ranges = [v for k, v in result.dynamic_range.items() if 'angle' in k]
            if angle_ranges:
                max_range = max(angle_ranges)
                range_pass = max_range > 10  # At least 10 degrees of dynamic range
                pass_criteria['dynamic_range'] = range_pass

                if not range_pass:
                    recommendations.append("Dynamic range is insufficient, test motion amplitude may not be enough")

            # Determine overall status
            if all(pass_criteria.values()):
                result.test_status = "PASS"
            elif any(pass_criteria.values()):
                result.test_status = "WARNING"
            else:
                result.test_status = "FAIL"

            if not recommendations:
                recommendations.append(f"{result.test_type} test metrics meet requirements")

            result.pass_criteria = pass_criteria
            result.recommendations = recommendations

        except Exception as e:
            self.logger.error(f"Dynamic test results evaluation failed: {e}")
            result.test_status = "ERROR"
            result.recommendations = [f"Evaluation failed: {str(e)}"]
