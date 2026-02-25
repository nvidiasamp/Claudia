#!/usr/bin/env python3
# scripts/validation/imu/imu_validation/static_tester.py
# Generated: 2025-06-27 11:54:45 CST
# Purpose: Unitree Go2 IMU static stability testing

import time
import statistics
import numpy as np
import logging
from typing import Dict, List, Tuple, Any, Optional
from dataclasses import dataclass, field

from imu_config import IMUConfig, IMUReading
from data_collector import IMUDataCollector, CollectionMetrics

@dataclass
class StaticTestResults:
    """Static test results"""
    test_duration: float = 0.0
    sample_count: int = 0
    valid_samples: int = 0

    # Stability metrics
    accelerometer_stability: Dict[str, float] = field(default_factory=dict)
    gyroscope_stability: Dict[str, float] = field(default_factory=dict)
    quaternion_stability: Dict[str, float] = field(default_factory=dict)

    # Accuracy metrics
    gravity_accuracy: Dict[str, float] = field(default_factory=dict)
    bias_analysis: Dict[str, float] = field(default_factory=dict)
    noise_analysis: Dict[str, float] = field(default_factory=dict)

    # Temperature analysis (if available)
    temperature_analysis: Dict[str, float] = field(default_factory=dict)

    # Test evaluation
    test_status: str = "UNKNOWN"
    pass_criteria: Dict[str, bool] = field(default_factory=dict)
    recommendations: List[str] = field(default_factory=list)

class IMUStaticTester:
    """IMU static tester"""

    def __init__(self, imu_config: IMUConfig, data_collector: IMUDataCollector, config: Dict[str, Any]):
        """
        Initialize static tester

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
        self.test_config = config.get("test_parameters", {}).get("static_test", {})
        self.quality_thresholds = config.get("quality_thresholds", {})

        # Test parameters
        self.test_duration = self.test_config.get("duration_seconds", 60)
        self.stability_thresholds = self.test_config.get("stability_threshold", {})

    def run_static_stability_test(self) -> StaticTestResults:
        """
        Run static stability test

        Returns:
            StaticTestResults: Test results
        """
        self.logger.info("=" * 50)
        self.logger.info("Starting IMU static stability test")
        self.logger.info("=" * 50)

        results = StaticTestResults()

        try:
            # Ensure IMU is initialized
            if not self.imu_config.is_initialized:
                self.logger.error("IMU is not initialized, cannot perform static test")
                results.test_status = "FAIL"
                return results

            self.logger.info(f"Test duration: {self.test_duration} seconds")
            self.logger.info("Please ensure the robot is in a stationary state...")

            # Wait a few seconds for user preparation
            self.logger.info("Test will start in 5 seconds...")
            time.sleep(5)

            # Start data collection
            self.logger.info("Starting data collection...")
            success = self.data_collector.start_collection(self.test_duration)

            if not success:
                self.logger.error("Unable to start data collection")
                results.test_status = "FAIL"
                return results

            # Wait for collection to complete
            start_time = time.time()
            while self.data_collector.is_collecting:
                elapsed = time.time() - start_time
                if elapsed >= self.test_duration + 10:  # Extra 10 second timeout
                    self.logger.warning("Data collection timed out, forcing stop")
                    break

                # Show progress
                if int(elapsed) % 10 == 0:
                    self.logger.info(f"Test progress: {elapsed:.1f}/{self.test_duration} seconds")

                time.sleep(1)

            # Stop collection and get metrics
            collection_metrics = self.data_collector.stop_collection()

            # Get collected data
            collected_data = self.data_collector.get_collected_data()

            if not collected_data:
                self.logger.error("No data collected")
                results.test_status = "FAIL"
                return results

            self.logger.info(f"Collection complete, obtained {len(collected_data)} samples")

            # Analyze data
            results = self._analyze_static_data(collected_data, collection_metrics)

            # Evaluate test results
            self._evaluate_static_results(results)

            self.logger.info("=" * 50)
            self.logger.info(f"Static test complete, status: {results.test_status}")
            self.logger.info("=" * 50)

            return results

        except Exception as e:
            self.logger.error(f"Static test failed: {e}")
            results.test_status = "ERROR"
            return results

    def _analyze_static_data(self, data: List[IMUReading], collection_metrics: CollectionMetrics) -> StaticTestResults:
        """Analyze static test data"""
        results = StaticTestResults()

        try:
            results.test_duration = collection_metrics.collection_duration
            results.sample_count = len(data)
            results.valid_samples = collection_metrics.valid_samples

            # Extract sensor data
            accel_data = np.array([[r.accelerometer[0], r.accelerometer[1], r.accelerometer[2]] for r in data])
            gyro_data = np.array([[r.gyroscope[0], r.gyroscope[1], r.gyroscope[2]] for r in data])
            quat_data = np.array([[r.quaternion[0], r.quaternion[1], r.quaternion[2], r.quaternion[3]] for r in data])

            # Analyze accelerometer stability
            results.accelerometer_stability = self._analyze_accelerometer_stability(accel_data)

            # Analyze gyroscope stability
            results.gyroscope_stability = self._analyze_gyroscope_stability(gyro_data)

            # Analyze quaternion stability
            results.quaternion_stability = self._analyze_quaternion_stability(quat_data)

            # Analyze gravity accuracy
            results.gravity_accuracy = self._analyze_gravity_accuracy(accel_data)

            # Analyze bias
            results.bias_analysis = self._analyze_bias(accel_data, gyro_data)

            # Analyze noise
            results.noise_analysis = self._analyze_noise(accel_data, gyro_data)

            # Temperature analysis (if temperature data is available)
            temp_data = [r.temperature for r in data if r.temperature is not None]
            if temp_data:
                results.temperature_analysis = self._analyze_temperature(temp_data)

            return results

        except Exception as e:
            self.logger.error(f"Static data analysis failed: {e}")
            results.test_status = "ERROR"
            return results

    def _analyze_accelerometer_stability(self, accel_data: np.ndarray) -> Dict[str, float]:
        """Analyze accelerometer stability"""
        try:
            stability = {}

            # Calculate per-axis standard deviation
            stability['std_x'] = np.std(accel_data[:, 0])
            stability['std_y'] = np.std(accel_data[:, 1])
            stability['std_z'] = np.std(accel_data[:, 2])

            # Calculate total standard deviation
            stability['std_total'] = np.sqrt(stability['std_x']**2 + stability['std_y']**2 + stability['std_z']**2)

            # Calculate magnitude stability
            magnitudes = np.linalg.norm(accel_data, axis=1)
            stability['magnitude_mean'] = np.mean(magnitudes)
            stability['magnitude_std'] = np.std(magnitudes)
            stability['magnitude_variation'] = stability['magnitude_std'] / stability['magnitude_mean'] * 100  # Percentage

            # Calculate peak-to-peak variation
            stability['peak_to_peak_x'] = np.max(accel_data[:, 0]) - np.min(accel_data[:, 0])
            stability['peak_to_peak_y'] = np.max(accel_data[:, 1]) - np.min(accel_data[:, 1])
            stability['peak_to_peak_z'] = np.max(accel_data[:, 2]) - np.min(accel_data[:, 2])

            # Calculate mean values
            stability['mean_x'] = np.mean(accel_data[:, 0])
            stability['mean_y'] = np.mean(accel_data[:, 1])
            stability['mean_z'] = np.mean(accel_data[:, 2])

            return stability

        except Exception as e:
            self.logger.error(f"Accelerometer stability analysis failed: {e}")
            return {}

    def _analyze_gyroscope_stability(self, gyro_data: np.ndarray) -> Dict[str, float]:
        """Analyze gyroscope stability"""
        try:
            stability = {}

            # Calculate per-axis standard deviation
            stability['std_x'] = np.std(gyro_data[:, 0])
            stability['std_y'] = np.std(gyro_data[:, 1])
            stability['std_z'] = np.std(gyro_data[:, 2])

            # Calculate total standard deviation
            stability['std_total'] = np.sqrt(stability['std_x']**2 + stability['std_y']**2 + stability['std_z']**2)

            # Calculate average bias (should be near zero at rest)
            stability['bias_x'] = np.mean(gyro_data[:, 0])
            stability['bias_y'] = np.mean(gyro_data[:, 1])
            stability['bias_z'] = np.mean(gyro_data[:, 2])
            stability['bias_magnitude'] = np.sqrt(stability['bias_x']**2 + stability['bias_y']**2 + stability['bias_z']**2)

            # Calculate peak-to-peak variation
            stability['peak_to_peak_x'] = np.max(gyro_data[:, 0]) - np.min(gyro_data[:, 0])
            stability['peak_to_peak_y'] = np.max(gyro_data[:, 1]) - np.min(gyro_data[:, 1])
            stability['peak_to_peak_z'] = np.max(gyro_data[:, 2]) - np.min(gyro_data[:, 2])

            # Calculate angular drift (integration)
            if len(gyro_data) > 1:
                # Assuming sampling rate from configuration
                dt = 1.0 / self.imu_config.target_spec.sampling_rate_hz

                # Calculate angular drift (simple integration)
                drift_x = np.cumsum(gyro_data[:, 0]) * dt
                drift_y = np.cumsum(gyro_data[:, 1]) * dt
                drift_z = np.cumsum(gyro_data[:, 2]) * dt

                stability['drift_x_deg'] = np.degrees(drift_x[-1] - drift_x[0])
                stability['drift_y_deg'] = np.degrees(drift_y[-1] - drift_y[0])
                stability['drift_z_deg'] = np.degrees(drift_z[-1] - drift_z[0])
                stability['total_drift_deg'] = np.sqrt(stability['drift_x_deg']**2 + stability['drift_y_deg']**2 + stability['drift_z_deg']**2)

            return stability

        except Exception as e:
            self.logger.error(f"Gyroscope stability analysis failed: {e}")
            return {}

    def _analyze_quaternion_stability(self, quat_data: np.ndarray) -> Dict[str, float]:
        """Analyze quaternion stability"""
        try:
            stability = {}

            # Calculate quaternion component stability
            stability['std_w'] = np.std(quat_data[:, 0])
            stability['std_x'] = np.std(quat_data[:, 1])
            stability['std_y'] = np.std(quat_data[:, 2])
            stability['std_z'] = np.std(quat_data[:, 3])

            # Calculate quaternion magnitude stability (should always be near 1)
            magnitudes = np.linalg.norm(quat_data, axis=1)
            stability['magnitude_mean'] = np.mean(magnitudes)
            stability['magnitude_std'] = np.std(magnitudes)
            stability['magnitude_error'] = abs(stability['magnitude_mean'] - 1.0)

            # Calculate Euler angle stability
            euler_angles = []
            for quat in quat_data:
                roll, pitch, yaw = self.imu_config.quaternion_to_euler(tuple(quat))
                euler_angles.append([np.degrees(roll), np.degrees(pitch), np.degrees(yaw)])

            euler_array = np.array(euler_angles)
            stability['euler_std_roll'] = np.std(euler_array[:, 0])
            stability['euler_std_pitch'] = np.std(euler_array[:, 1])
            stability['euler_std_yaw'] = np.std(euler_array[:, 2])

            # Calculate orientation drift
            if len(euler_array) > 1:
                stability['orientation_drift_roll'] = euler_array[-1, 0] - euler_array[0, 0]
                stability['orientation_drift_pitch'] = euler_array[-1, 1] - euler_array[0, 1]
                stability['orientation_drift_yaw'] = euler_array[-1, 2] - euler_array[0, 2]
                stability['total_orientation_drift'] = np.sqrt(
                    stability['orientation_drift_roll']**2 +
                    stability['orientation_drift_pitch']**2 +
                    stability['orientation_drift_yaw']**2
                )

            return stability

        except Exception as e:
            self.logger.error(f"Quaternion stability analysis failed: {e}")
            return {}

    def _analyze_gravity_accuracy(self, accel_data: np.ndarray) -> Dict[str, float]:
        """Analyze gravity accuracy"""
        try:
            accuracy = {}

            # Calculate acceleration magnitude
            magnitudes = np.linalg.norm(accel_data, axis=1)

            # Gravity reference value
            gravity_ref = self.test_config.get("gravity_reference", 9.81)

            # Average gravity measurement
            accuracy['measured_gravity'] = np.mean(magnitudes)
            accuracy['gravity_error'] = accuracy['measured_gravity'] - gravity_ref
            accuracy['gravity_error_percent'] = abs(accuracy['gravity_error']) / gravity_ref * 100

            # Gravity measurement stability
            accuracy['gravity_std'] = np.std(magnitudes)
            accuracy['gravity_variation_percent'] = accuracy['gravity_std'] / accuracy['measured_gravity'] * 100

            # Gravity direction analysis (Z-axis should be primarily affected by gravity)
            accuracy['gravity_distribution_x'] = abs(np.mean(accel_data[:, 0]))
            accuracy['gravity_distribution_y'] = abs(np.mean(accel_data[:, 1]))
            accuracy['gravity_distribution_z'] = abs(np.mean(accel_data[:, 2]))

            # Calculate tilt angle (relative to gravity)
            mean_accel = np.mean(accel_data, axis=0)
            mean_accel_norm = mean_accel / np.linalg.norm(mean_accel)

            # Assume gravity is along negative Z direction
            gravity_vector = np.array([0, 0, -1])
            dot_product = np.dot(mean_accel_norm, gravity_vector)
            accuracy['tilt_angle_deg'] = np.degrees(np.arccos(np.clip(dot_product, -1.0, 1.0)))

            return accuracy

        except Exception as e:
            self.logger.error(f"Gravity accuracy analysis failed: {e}")
            return {}

    def _analyze_bias(self, accel_data: np.ndarray, gyro_data: np.ndarray) -> Dict[str, float]:
        """Analyze sensor bias"""
        try:
            bias = {}

            # Accelerometer bias analysis
            # At rest, accelerometer should measure gravity
            gravity_ref = self.test_config.get("gravity_reference", 9.81)

            mean_accel = np.mean(accel_data, axis=0)
            expected_accel = np.array([0, 0, -gravity_ref])  # Assuming Z-axis is up

            bias['accel_bias_x'] = mean_accel[0] - expected_accel[0]
            bias['accel_bias_y'] = mean_accel[1] - expected_accel[1]
            bias['accel_bias_z'] = mean_accel[2] - expected_accel[2]
            bias['accel_bias_magnitude'] = np.linalg.norm([bias['accel_bias_x'], bias['accel_bias_y'], bias['accel_bias_z']])

            # Gyroscope bias analysis
            # At rest, gyroscope readings should be zero
            mean_gyro = np.mean(gyro_data, axis=0)

            bias['gyro_bias_x'] = mean_gyro[0]
            bias['gyro_bias_y'] = mean_gyro[1]
            bias['gyro_bias_z'] = mean_gyro[2]
            bias['gyro_bias_magnitude'] = np.linalg.norm(mean_gyro)

            # Bias stability (simplified Allan variance)
            bias['accel_bias_stability_x'] = np.std(accel_data[:, 0])
            bias['accel_bias_stability_y'] = np.std(accel_data[:, 1])
            bias['accel_bias_stability_z'] = np.std(accel_data[:, 2])

            bias['gyro_bias_stability_x'] = np.std(gyro_data[:, 0])
            bias['gyro_bias_stability_y'] = np.std(gyro_data[:, 1])
            bias['gyro_bias_stability_z'] = np.std(gyro_data[:, 2])

            return bias

        except Exception as e:
            self.logger.error(f"Bias analysis failed: {e}")
            return {}

    def _analyze_noise(self, accel_data: np.ndarray, gyro_data: np.ndarray) -> Dict[str, float]:
        """Analyze sensor noise"""
        try:
            noise = {}

            # Remove trend (detrending)
            def detrend(data):
                """Simple linear detrending"""
                n = len(data)
                t = np.arange(n)
                coeffs = np.polyfit(t, data, 1)
                trend = np.polyval(coeffs, t)
                return data - trend

            # Accelerometer noise analysis
            accel_detrended = np.array([detrend(accel_data[:, i]) for i in range(3)]).T

            noise['accel_noise_x'] = np.std(accel_detrended[:, 0])
            noise['accel_noise_y'] = np.std(accel_detrended[:, 1])
            noise['accel_noise_z'] = np.std(accel_detrended[:, 2])
            noise['accel_noise_total'] = np.sqrt(np.sum([noise[f'accel_noise_{ax}']**2 for ax in ['x', 'y', 'z']]))

            # Gyroscope noise analysis
            gyro_detrended = np.array([detrend(gyro_data[:, i]) for i in range(3)]).T

            noise['gyro_noise_x'] = np.std(gyro_detrended[:, 0])
            noise['gyro_noise_y'] = np.std(gyro_detrended[:, 1])
            noise['gyro_noise_z'] = np.std(gyro_detrended[:, 2])
            noise['gyro_noise_total'] = np.sqrt(np.sum([noise[f'gyro_noise_{ax}']**2 for ax in ['x', 'y', 'z']]))

            # Signal-to-noise ratio estimation
            accel_signal = np.mean(np.linalg.norm(accel_data, axis=1))
            gyro_signal = np.mean(np.linalg.norm(gyro_data, axis=1))

            if noise['accel_noise_total'] > 0:
                noise['accel_snr_db'] = 20 * np.log10(accel_signal / noise['accel_noise_total'])
            else:
                noise['accel_snr_db'] = float('inf')

            if noise['gyro_noise_total'] > 0:
                noise['gyro_snr_db'] = 20 * np.log10(gyro_signal / noise['gyro_noise_total'])
            else:
                noise['gyro_snr_db'] = float('inf')

            return noise

        except Exception as e:
            self.logger.error(f"Noise analysis failed: {e}")
            return {}

    def _analyze_temperature(self, temp_data: List[float]) -> Dict[str, float]:
        """Analyze temperature data"""
        try:
            temp_analysis = {}

            temp_analysis['mean_temperature'] = statistics.mean(temp_data)
            temp_analysis['temperature_std'] = statistics.stdev(temp_data) if len(temp_data) > 1 else 0
            temp_analysis['temperature_range'] = max(temp_data) - min(temp_data)
            temp_analysis['temperature_drift'] = temp_data[-1] - temp_data[0] if len(temp_data) > 1 else 0

            return temp_analysis

        except Exception as e:
            self.logger.error(f"Temperature analysis failed: {e}")
            return {}

    def _evaluate_static_results(self, results: StaticTestResults):
        """Evaluate static test results"""
        try:
            pass_criteria = {}
            recommendations = []

            # Evaluate accelerometer stability
            accel_std_threshold = self.stability_thresholds.get("accelerometer_std_max", 0.05)
            accel_pass = all(results.accelerometer_stability.get(f'std_{ax}', 0) < accel_std_threshold
                           for ax in ['x', 'y', 'z'])
            pass_criteria['accelerometer_stability'] = accel_pass

            if not accel_pass:
                recommendations.append("Accelerometer stability is insufficient, check mechanical vibration or sensor calibration")

            # Evaluate gyroscope stability
            gyro_std_threshold = self.stability_thresholds.get("gyroscope_std_max", 0.1)
            gyro_pass = all(results.gyroscope_stability.get(f'std_{ax}', 0) < gyro_std_threshold
                          for ax in ['x', 'y', 'z'])
            pass_criteria['gyroscope_stability'] = gyro_pass

            if not gyro_pass:
                recommendations.append("Gyroscope stability is insufficient, check temperature stability or sensor calibration")

            # Evaluate gravity accuracy
            gravity_error_threshold = self.quality_thresholds.get("accuracy", {}).get("gravity_error_max_percent", 2)
            gravity_pass = results.gravity_accuracy.get('gravity_error_percent', 100) < gravity_error_threshold
            pass_criteria['gravity_accuracy'] = gravity_pass

            if not gravity_pass:
                recommendations.append("Gravity measurement accuracy is insufficient, accelerometer calibration needed")

            # Evaluate quaternion stability
            quat_drift_threshold = self.stability_thresholds.get("quaternion_drift_max", 0.01)
            quat_pass = results.quaternion_stability.get('magnitude_error', 1) < quat_drift_threshold
            pass_criteria['quaternion_stability'] = quat_pass

            if not quat_pass:
                recommendations.append("Quaternion stability is insufficient, check attitude fusion algorithm")

            # Evaluate noise levels
            accel_noise_threshold = self.quality_thresholds.get("noise_levels", {}).get("accelerometer_noise_max", 0.02)
            gyro_noise_threshold = self.quality_thresholds.get("noise_levels", {}).get("gyroscope_noise_max", 0.01)

            accel_noise_pass = results.noise_analysis.get('accel_noise_total', 1) < accel_noise_threshold
            gyro_noise_pass = results.noise_analysis.get('gyro_noise_total', 1) < gyro_noise_threshold

            pass_criteria['accelerometer_noise'] = accel_noise_pass
            pass_criteria['gyroscope_noise'] = gyro_noise_pass

            if not accel_noise_pass:
                recommendations.append("Accelerometer noise is too high, check electromagnetic interference or filter settings")

            if not gyro_noise_pass:
                recommendations.append("Gyroscope noise is too high, check power supply stability or filter settings")

            # Determine overall status
            all_passed = all(pass_criteria.values())
            critical_passed = all([
                pass_criteria.get('accelerometer_stability', False),
                pass_criteria.get('gyroscope_stability', False),
                pass_criteria.get('gravity_accuracy', False)
            ])

            if all_passed:
                results.test_status = "PASS"
            elif critical_passed:
                results.test_status = "WARNING"
            else:
                results.test_status = "FAIL"

            if not recommendations:
                recommendations.append("All static test metrics meet requirements")

            results.pass_criteria = pass_criteria
            results.recommendations = recommendations

        except Exception as e:
            self.logger.error(f"Static test results evaluation failed: {e}")
            results.test_status = "ERROR"
            results.recommendations = [f"Evaluation failed: {str(e)}"]
