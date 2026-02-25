#!/usr/bin/env python3
# scripts/validation/imu/imu_validation/calibration_analyzer.py
# Generated: 2025-06-27 11:54:45 CST
# Purpose: Unitree Go2 IMU calibration validation and analysis

import time
import statistics
import numpy as np
import logging
from typing import Dict, List, Tuple, Any, Optional
from dataclasses import dataclass, field
import json
from pathlib import Path

from imu_config import IMUConfig, IMUReading
from data_collector import IMUDataCollector, CollectionMetrics

@dataclass
class CalibrationResults:
    """Calibration analysis results"""
    calibration_type: str = ""
    sample_count: int = 0
    test_duration: float = 0.0

    # Accelerometer calibration
    accelerometer_calibration: Dict[str, float] = field(default_factory=dict)
    gravity_calibration: Dict[str, float] = field(default_factory=dict)
    accel_bias: Dict[str, float] = field(default_factory=dict)
    accel_scale_factor: Dict[str, float] = field(default_factory=dict)
    accel_cross_axis: Dict[str, float] = field(default_factory=dict)

    # Gyroscope calibration
    gyroscope_calibration: Dict[str, float] = field(default_factory=dict)
    gyro_bias: Dict[str, float] = field(default_factory=dict)
    gyro_scale_factor: Dict[str, float] = field(default_factory=dict)
    gyro_noise_characteristics: Dict[str, float] = field(default_factory=dict)

    # Attitude calibration
    attitude_calibration: Dict[str, float] = field(default_factory=dict)
    quaternion_consistency: Dict[str, float] = field(default_factory=dict)
    euler_accuracy: Dict[str, float] = field(default_factory=dict)

    # Temperature compensation (if available)
    temperature_compensation: Dict[str, float] = field(default_factory=dict)

    # Calibration quality metrics
    calibration_quality: Dict[str, float] = field(default_factory=dict)

    # Test evaluation
    test_status: str = "UNKNOWN"
    pass_criteria: Dict[str, bool] = field(default_factory=dict)
    recommendations: List[str] = field(default_factory=list)

class IMUCalibrationAnalyzer:
    """IMU calibration analyzer"""

    def __init__(self, imu_config: IMUConfig, data_collector: IMUDataCollector, config: Dict[str, Any]):
        """
        Initialize calibration analyzer

        Args:
            imu_config: IMU configuration instance
            data_collector: Data collector instance
            config: Validation configuration
        """
        self.imu_config = imu_config
        self.data_collector = data_collector
        self.config = config
        self.logger = logging.getLogger(__name__)

        # Calibration configuration
        self.calib_config = config.get("test_parameters", {}).get("calibration_analysis", {})
        self.quality_thresholds = config.get("quality_thresholds", {})

        # Calibration parameters
        self.gravity_reference = self.calib_config.get("gravity_reference", 9.81)
        self.gravity_tolerance = self.calib_config.get("gravity_tolerance", 0.2)
        self.bias_samples = self.calib_config.get("bias_analysis_samples", 1000)
        self.enable_temperature = self.calib_config.get("temperature_analysis", True)

    def run_comprehensive_calibration_analysis(self) -> CalibrationResults:
        """
        Run comprehensive calibration analysis

        Returns:
            CalibrationResults: Calibration analysis results
        """
        self.logger.info("=" * 50)
        self.logger.info("Starting IMU calibration validation analysis")
        self.logger.info("=" * 50)

        results = CalibrationResults()
        results.calibration_type = "comprehensive"

        try:
            # Ensure IMU is initialized
            if not self.imu_config.is_initialized:
                self.logger.error("IMU is not initialized, cannot perform calibration analysis")
                results.test_status = "FAIL"
                return results

            # Collect calibration analysis data
            self.logger.info("Collecting calibration analysis data...")
            self.logger.info("Please ensure the robot is kept still in multiple different orientations...")

            calibration_data = self._collect_calibration_data()

            if not calibration_data:
                self.logger.error("Calibration data collection failed")
                results.test_status = "FAIL"
                return results

            results.sample_count = len(calibration_data)

            # Execute various calibration analyses
            self.logger.info("Performing accelerometer calibration analysis...")
            results = self._analyze_accelerometer_calibration(results, calibration_data)

            self.logger.info("Performing gyroscope calibration analysis...")
            results = self._analyze_gyroscope_calibration(results, calibration_data)

            self.logger.info("Performing attitude calibration analysis...")
            results = self._analyze_attitude_calibration(results, calibration_data)

            # Temperature compensation analysis (if temperature data is available)
            if self.enable_temperature:
                self.logger.info("Performing temperature compensation analysis...")
                results = self._analyze_temperature_compensation(results, calibration_data)

            # Calculate overall calibration quality
            self.logger.info("Calculating calibration quality metrics...")
            results = self._calculate_calibration_quality(results, calibration_data)

            # Evaluate calibration results
            self._evaluate_calibration_results(results)

            self.logger.info("=" * 50)
            self.logger.info(f"Calibration analysis complete, status: {results.test_status}")
            self.logger.info("=" * 50)

            return results

        except Exception as e:
            self.logger.error(f"Calibration analysis failed: {e}")
            results.test_status = "ERROR"
            return results

    def _collect_calibration_data(self) -> List[IMUReading]:
        """Collect calibration data"""
        try:
            all_data = []

            # Define test orientations
            positions = [
                "Horizontal rest (normal orientation)",
                "Tilted left 90 degrees",
                "Tilted right 90 degrees",
                "Tilted forward 90 degrees",
                "Tilted backward 90 degrees",
                "Inverted 180 degrees"
            ]

            for i, position in enumerate(positions):
                self.logger.info(f"\nPosition {i+1}/{len(positions)}: {position}")
                self.logger.info("Please adjust the robot to the specified position and keep it still...")
                self.logger.info("Data collection will start in 15 seconds...")
                time.sleep(15)

                # Collect data at this position
                self.logger.info("Starting data collection...")
                success = self.data_collector.start_collection(30.0)  # 30 seconds of collection

                if not success:
                    self.logger.error(f"Data collection failed to start at position {position}")
                    continue

                # Wait for collection to complete
                start_time = time.time()
                while self.data_collector.is_collecting:
                    elapsed = time.time() - start_time
                    if elapsed >= 40:  # 40 second timeout
                        self.logger.warning("Data collection timed out")
                        break

                    if int(elapsed) % 5 == 0:
                        self.logger.info(f"Collection progress: {elapsed:.1f}/30.0 seconds")

                    time.sleep(1)

                # Get data
                self.data_collector.stop_collection()
                position_data = self.data_collector.get_collected_data()

                if position_data:
                    # Add position identifier
                    for reading in position_data:
                        reading.position_id = i

                    all_data.extend(position_data)
                    self.logger.info(f"Position {position} collection complete, obtained {len(position_data)} samples")
                else:
                    self.logger.warning(f"Position {position} yielded no data")

                # Rest between positions
                if i < len(positions) - 1:
                    self.logger.info("Continuing to next position in 10 seconds...")
                    time.sleep(10)

            self.logger.info(f"Calibration data collection complete, total samples: {len(all_data)}")
            return all_data

        except Exception as e:
            self.logger.error(f"Calibration data collection failed: {e}")
            return []

    def _analyze_accelerometer_calibration(self, results: CalibrationResults, data: List[IMUReading]) -> CalibrationResults:
        """Analyze accelerometer calibration"""
        try:
            if not data:
                return results

            # Extract accelerometer data
            accel_data = np.array([[r.accelerometer[0], r.accelerometer[1], r.accelerometer[2]] for r in data])

            # Calculate gravity calibration accuracy
            magnitudes = np.linalg.norm(accel_data, axis=1)

            results.gravity_calibration['measured_gravity_mean'] = np.mean(magnitudes)
            results.gravity_calibration['measured_gravity_std'] = np.std(magnitudes)
            results.gravity_calibration['gravity_error'] = results.gravity_calibration['measured_gravity_mean'] - self.gravity_reference
            results.gravity_calibration['gravity_error_percent'] = abs(results.gravity_calibration['gravity_error']) / self.gravity_reference * 100

            # Analyze per-axis bias
            mean_accel = np.mean(accel_data, axis=0)
            results.accel_bias['bias_x'] = mean_accel[0]
            results.accel_bias['bias_y'] = mean_accel[1]
            results.accel_bias['bias_z'] = mean_accel[2]
            results.accel_bias['bias_magnitude'] = np.linalg.norm(mean_accel)

            # Analyze scale factors (if multi-orientation data is available)
            if hasattr(data[0], 'position_id'):
                results = self._analyze_scale_factors(results, accel_data, data, 'accelerometer')

            # Cross-axis coupling analysis
            results = self._analyze_cross_axis_coupling(results, accel_data, 'accelerometer')

            # Accelerometer nonlinearity
            results.accelerometer_calibration['nonlinearity'] = self._calculate_nonlinearity(accel_data)

            return results

        except Exception as e:
            self.logger.error(f"Accelerometer calibration analysis failed: {e}")
            return results

    def _analyze_gyroscope_calibration(self, results: CalibrationResults, data: List[IMUReading]) -> CalibrationResults:
        """Analyze gyroscope calibration"""
        try:
            if not data:
                return results

            # Extract gyroscope data
            gyro_data = np.array([[r.gyroscope[0], r.gyroscope[1], r.gyroscope[2]] for r in data])

            # Static bias analysis
            mean_gyro = np.mean(gyro_data, axis=0)
            std_gyro = np.std(gyro_data, axis=0)

            results.gyro_bias['bias_x'] = mean_gyro[0]
            results.gyro_bias['bias_y'] = mean_gyro[1]
            results.gyro_bias['bias_z'] = mean_gyro[2]
            results.gyro_bias['bias_magnitude'] = np.linalg.norm(mean_gyro)

            # Bias stability
            results.gyro_bias['bias_stability_x'] = std_gyro[0]
            results.gyro_bias['bias_stability_y'] = std_gyro[1]
            results.gyro_bias['bias_stability_z'] = std_gyro[2]

            # Noise characteristics analysis
            results = self._analyze_gyro_noise_characteristics(results, gyro_data)

            # Scale factor analysis (if dynamic data is available)
            if hasattr(data[0], 'position_id'):
                results = self._analyze_scale_factors(results, gyro_data, data, 'gyroscope')

            # Cross-axis coupling
            results = self._analyze_cross_axis_coupling(results, gyro_data, 'gyroscope')

            # Angle random walk
            results.gyro_noise_characteristics['angle_random_walk'] = self._calculate_angle_random_walk(gyro_data)

            return results

        except Exception as e:
            self.logger.error(f"Gyroscope calibration analysis failed: {e}")
            return results

    def _analyze_attitude_calibration(self, results: CalibrationResults, data: List[IMUReading]) -> CalibrationResults:
        """Analyze attitude calibration"""
        try:
            if not data:
                return results

            # Extract quaternion data
            quat_data = np.array([[r.quaternion[0], r.quaternion[1], r.quaternion[2], r.quaternion[3]] for r in data])

            # Quaternion consistency check
            quat_magnitudes = np.linalg.norm(quat_data, axis=1)
            results.quaternion_consistency['magnitude_mean'] = np.mean(quat_magnitudes)
            results.quaternion_consistency['magnitude_std'] = np.std(quat_magnitudes)
            results.quaternion_consistency['magnitude_error'] = abs(results.quaternion_consistency['magnitude_mean'] - 1.0)

            # Euler angle accuracy analysis
            euler_data = []
            for quat in quat_data:
                roll, pitch, yaw = self.imu_config.quaternion_to_euler(tuple(quat))
                euler_data.append([np.degrees(roll), np.degrees(pitch), np.degrees(yaw)])

            euler_array = np.array(euler_data)

            # Calculate Euler angle stability
            results.euler_accuracy['roll_std'] = np.std(euler_array[:, 0])
            results.euler_accuracy['pitch_std'] = np.std(euler_array[:, 1])
            results.euler_accuracy['yaw_std'] = np.std(euler_array[:, 2])

            # If position identifiers exist, analyze accuracy at different orientations
            if hasattr(data[0], 'position_id'):
                results = self._analyze_attitude_accuracy_by_position(results, euler_array, data)

            # Attitude fusion quality
            results.attitude_calibration['fusion_quality'] = self._calculate_attitude_fusion_quality(quat_data, euler_array)

            return results

        except Exception as e:
            self.logger.error(f"Attitude calibration analysis failed: {e}")
            return results

    def _analyze_temperature_compensation(self, results: CalibrationResults, data: List[IMUReading]) -> CalibrationResults:
        """Analyze temperature compensation"""
        try:
            # Extract temperature data
            temp_data = [r.temperature for r in data if r.temperature is not None]

            if not temp_data:
                self.logger.info("No temperature data available, skipping temperature compensation analysis")
                return results

            accel_data = np.array([[r.accelerometer[0], r.accelerometer[1], r.accelerometer[2]] for r in data])
            gyro_data = np.array([[r.gyroscope[0], r.gyroscope[1], r.gyroscope[2]] for r in data])

            # Temperature range
            results.temperature_compensation['temp_range'] = max(temp_data) - min(temp_data)
            results.temperature_compensation['temp_mean'] = statistics.mean(temp_data)
            results.temperature_compensation['temp_std'] = statistics.stdev(temp_data) if len(temp_data) > 1 else 0

            # Temperature coefficient analysis (simplified)
            if results.temperature_compensation['temp_range'] > 5:  # Analyze when temperature variation exceeds 5 degrees
                temp_array = np.array(temp_data)

                # Accelerometer temperature coefficient
                for i, axis in enumerate(['x', 'y', 'z']):
                    temp_coeff = np.polyfit(temp_array, accel_data[:, i], 1)[0]  # Linear fit slope
                    results.temperature_compensation[f'accel_temp_coeff_{axis}'] = temp_coeff

                # Gyroscope temperature coefficient
                for i, axis in enumerate(['x', 'y', 'z']):
                    temp_coeff = np.polyfit(temp_array, gyro_data[:, i], 1)[0]
                    results.temperature_compensation[f'gyro_temp_coeff_{axis}'] = temp_coeff
            else:
                self.logger.info("Temperature variation range is insufficient, cannot analyze temperature coefficients")

            return results

        except Exception as e:
            self.logger.error(f"Temperature compensation analysis failed: {e}")
            return results

    def _analyze_scale_factors(self, results: CalibrationResults, sensor_data: np.ndarray,
                             data: List[IMUReading], sensor_type: str) -> CalibrationResults:
        """Analyze scale factors"""
        try:
            # Group data by position
            position_groups = {}
            for i, reading in enumerate(data):
                if hasattr(reading, 'position_id'):
                    pos_id = reading.position_id
                    if pos_id not in position_groups:
                        position_groups[pos_id] = []
                    position_groups[pos_id].append(i)

            if len(position_groups) < 2:
                return results

            # Calculate mean for each position
            position_means = {}
            for pos_id, indices in position_groups.items():
                position_means[pos_id] = np.mean(sensor_data[indices], axis=0)

            # Analyze scale factors (simplified analysis)
            scale_factors = []
            for axis in range(3):
                axis_values = [mean[axis] for mean in position_means.values()]
                if len(set(np.sign(axis_values))) > 1:  # Sign changes exist
                    scale_factor = np.std(axis_values) / np.mean(np.abs(axis_values))
                    scale_factors.append(scale_factor)

            if scale_factors:
                key_prefix = 'accel_scale_factor' if sensor_type == 'accelerometer' else 'gyro_scale_factor'
                target_dict = results.accel_scale_factor if sensor_type == 'accelerometer' else results.gyro_scale_factor

                target_dict['mean_scale_factor'] = np.mean(scale_factors)
                target_dict['scale_factor_std'] = np.std(scale_factors) if len(scale_factors) > 1 else 0
                target_dict['scale_factor_error_percent'] = abs(target_dict['mean_scale_factor'] - 1.0) * 100

            return results

        except Exception as e:
            self.logger.error(f"{sensor_type} scale factor analysis failed: {e}")
            return results

    def _analyze_cross_axis_coupling(self, results: CalibrationResults, sensor_data: np.ndarray,
                                   sensor_type: str) -> CalibrationResults:
        """Analyze cross-axis coupling"""
        try:
            # Calculate cross-axis correlation
            correlation_matrix = np.corrcoef(sensor_data.T)

            key_prefix = 'accel_cross_axis' if sensor_type == 'accelerometer' else 'gyro_cross_axis'
            target_dict = results.accel_cross_axis if sensor_type == 'accelerometer' else getattr(results, 'gyro_cross_axis', {})

            # Off-diagonal elements (cross-coupling)
            target_dict['xy_coupling'] = abs(correlation_matrix[0, 1])
            target_dict['xz_coupling'] = abs(correlation_matrix[0, 2])
            target_dict['yz_coupling'] = abs(correlation_matrix[1, 2])
            target_dict['max_coupling'] = max(target_dict['xy_coupling'], target_dict['xz_coupling'], target_dict['yz_coupling'])

            # Set back to results
            if sensor_type == 'accelerometer':
                results.accel_cross_axis = target_dict
            else:
                setattr(results, 'gyro_cross_axis', target_dict)

            return results

        except Exception as e:
            self.logger.error(f"{sensor_type} cross-axis coupling analysis failed: {e}")
            return results

    def _analyze_gyro_noise_characteristics(self, results: CalibrationResults, gyro_data: np.ndarray) -> CalibrationResults:
        """Analyze gyroscope noise characteristics"""
        try:
            # White noise density
            for i, axis in enumerate(['x', 'y', 'z']):
                std_dev = np.std(gyro_data[:, i])
                # Assuming 100Hz sampling rate, white noise density = standard deviation * sqrt(sampling rate)
                sampling_rate = self.imu_config.target_spec.sampling_rate_hz
                noise_density = std_dev * np.sqrt(sampling_rate)
                results.gyro_noise_characteristics[f'noise_density_{axis}'] = noise_density

            # Overall noise level
            total_noise = np.sqrt(np.sum([results.gyro_noise_characteristics[f'noise_density_{ax}']**2 for ax in ['x', 'y', 'z']]))
            results.gyro_noise_characteristics['total_noise_density'] = total_noise

            return results

        except Exception as e:
            self.logger.error(f"Gyroscope noise characteristics analysis failed: {e}")
            return results

    def _calculate_nonlinearity(self, data: np.ndarray) -> float:
        """Calculate nonlinearity"""
        try:
            # Simplified nonlinearity analysis
            # Calculate second-order polynomial fit residuals for each axis
            nonlinearities = []

            for axis in range(data.shape[1]):
                axis_data = data[:, axis]
                x = np.arange(len(axis_data))

                # Linear fit
                linear_fit = np.polyfit(x, axis_data, 1)
                linear_values = np.polyval(linear_fit, x)

                # Quadratic fit
                quad_fit = np.polyfit(x, axis_data, 2)
                quad_values = np.polyval(quad_fit, x)

                # Nonlinearity = quadratic term contribution / total signal range
                nonlinearity = np.std(quad_values - linear_values) / (np.max(axis_data) - np.min(axis_data))
                nonlinearities.append(nonlinearity)

            return np.mean(nonlinearities)

        except Exception as e:
            self.logger.error(f"Nonlinearity calculation failed: {e}")
            return 0.0

    def _calculate_angle_random_walk(self, gyro_data: np.ndarray) -> float:
        """Calculate angle random walk"""
        try:
            # Simplified angle random walk calculation
            # ARW = gyroscope noise standard deviation / sqrt(integration time)
            dt = 1.0 / self.imu_config.target_spec.sampling_rate_hz

            # Calculate ARW for each axis
            arw_values = []
            for axis in range(gyro_data.shape[1]):
                noise_std = np.std(gyro_data[:, axis])
                arw = noise_std / np.sqrt(dt)  # Proper unit conversion needed for deg/sqrt(hour)
                arw_values.append(arw)

            return np.mean(arw_values)

        except Exception as e:
            self.logger.error(f"Angle random walk calculation failed: {e}")
            return 0.0

    def _analyze_attitude_accuracy_by_position(self, results: CalibrationResults, euler_array: np.ndarray,
                                             data: List[IMUReading]) -> CalibrationResults:
        """Analyze attitude accuracy by position"""
        try:
            # Group by position
            position_groups = {}
            for i, reading in enumerate(data):
                if hasattr(reading, 'position_id'):
                    pos_id = reading.position_id
                    if pos_id not in position_groups:
                        position_groups[pos_id] = []
                    position_groups[pos_id].append(i)

            # Analyze attitude accuracy for each position
            position_accuracies = []
            for pos_id, indices in position_groups.items():
                position_euler = euler_array[indices]

                # Calculate attitude standard deviation for this position
                roll_std = np.std(position_euler[:, 0])
                pitch_std = np.std(position_euler[:, 1])
                yaw_std = np.std(position_euler[:, 2])

                total_std = np.sqrt(roll_std**2 + pitch_std**2 + yaw_std**2)
                position_accuracies.append(total_std)

            results.euler_accuracy['position_consistency'] = np.mean(position_accuracies)
            results.euler_accuracy['worst_position_accuracy'] = max(position_accuracies)
            results.euler_accuracy['best_position_accuracy'] = min(position_accuracies)

            return results

        except Exception as e:
            self.logger.error(f"Position-based attitude accuracy analysis failed: {e}")
            return results

    def _calculate_attitude_fusion_quality(self, quat_data: np.ndarray, euler_data: np.ndarray) -> float:
        """Calculate attitude fusion quality"""
        try:
            # Calculate quaternion consistency
            quat_magnitudes = np.linalg.norm(quat_data, axis=1)
            magnitude_consistency = 1.0 - np.std(quat_magnitudes)

            # Calculate Euler angle smoothness
            euler_derivatives = np.abs(np.diff(euler_data, axis=0))
            smoothness = 1.0 / (1.0 + np.mean(euler_derivatives))

            # Combined quality score
            fusion_quality = 0.7 * magnitude_consistency + 0.3 * smoothness

            return max(0, min(1, fusion_quality))

        except Exception as e:
            self.logger.error(f"Attitude fusion quality calculation failed: {e}")
            return 0.0

    def _calculate_calibration_quality(self, results: CalibrationResults, data: List[IMUReading]) -> CalibrationResults:
        """Calculate calibration quality metrics"""
        try:
            quality_scores = []

            # Gravity accuracy quality
            gravity_error = results.gravity_calibration.get('gravity_error_percent', 100)
            gravity_quality = max(0, 1 - gravity_error / 5.0)  # 5% error scores 0
            quality_scores.append(gravity_quality)

            # Gyroscope bias quality
            gyro_bias_mag = results.gyro_bias.get('bias_magnitude', 1.0)
            bias_quality = max(0, 1 - gyro_bias_mag / 0.1)  # 0.1 rad/s bias scores 0
            quality_scores.append(bias_quality)

            # Quaternion consistency quality
            quat_error = results.quaternion_consistency.get('magnitude_error', 1.0)
            quat_quality = max(0, 1 - quat_error / 0.1)  # 0.1 error scores 0
            quality_scores.append(quat_quality)

            # Calculate overall quality score
            overall_quality = np.mean(quality_scores)

            results.calibration_quality['overall_score'] = overall_quality
            results.calibration_quality['gravity_quality'] = gravity_quality
            results.calibration_quality['bias_quality'] = bias_quality
            results.calibration_quality['quaternion_quality'] = quat_quality

            # Quality grade
            if overall_quality >= 0.9:
                results.calibration_quality['quality_grade'] = "EXCELLENT"
            elif overall_quality >= 0.8:
                results.calibration_quality['quality_grade'] = "GOOD"
            elif overall_quality >= 0.6:
                results.calibration_quality['quality_grade'] = "FAIR"
            else:
                results.calibration_quality['quality_grade'] = "POOR"

            return results

        except Exception as e:
            self.logger.error(f"Calibration quality calculation failed: {e}")
            return results

    def _evaluate_calibration_results(self, results: CalibrationResults):
        """Evaluate calibration results"""
        try:
            pass_criteria = {}
            recommendations = []

            # Evaluate gravity accuracy
            gravity_error_threshold = self.quality_thresholds.get("accuracy", {}).get("gravity_error_max_percent", 2)
            gravity_pass = results.gravity_calibration.get('gravity_error_percent', 100) < gravity_error_threshold
            pass_criteria['gravity_accuracy'] = gravity_pass

            if not gravity_pass:
                recommendations.append("Gravity measurement accuracy does not meet requirements, accelerometer recalibration needed")

            # Evaluate gyroscope bias
            gyro_bias_threshold = 0.05  # 0.05 rad/s
            gyro_bias_pass = results.gyro_bias.get('bias_magnitude', 1.0) < gyro_bias_threshold
            pass_criteria['gyroscope_bias'] = gyro_bias_pass

            if not gyro_bias_pass:
                recommendations.append("Gyroscope bias is too large, bias calibration needed")

            # Evaluate quaternion consistency
            quat_error_threshold = 0.01
            quat_pass = results.quaternion_consistency.get('magnitude_error', 1.0) < quat_error_threshold
            pass_criteria['quaternion_consistency'] = quat_pass

            if not quat_pass:
                recommendations.append("Quaternion consistency is insufficient, check attitude fusion algorithm")

            # Evaluate cross-axis coupling
            max_coupling_threshold = 0.1  # 10%
            accel_coupling_pass = results.accel_cross_axis.get('max_coupling', 1.0) < max_coupling_threshold
            pass_criteria['accelerometer_coupling'] = accel_coupling_pass

            if not accel_coupling_pass:
                recommendations.append("Accelerometer cross-axis coupling is too high, calibration correction needed")

            # Evaluate overall calibration quality
            overall_quality = results.calibration_quality.get('overall_score', 0)
            quality_pass = overall_quality > 0.7  # 70% quality threshold
            pass_criteria['overall_quality'] = quality_pass

            if not quality_pass:
                recommendations.append("Overall calibration quality is insufficient, comprehensive recalibration recommended")

            # Determine overall status
            critical_tests = ['gravity_accuracy', 'gyroscope_bias', 'quaternion_consistency']
            critical_passed = all(pass_criteria.get(test, False) for test in critical_tests)
            all_passed = all(pass_criteria.values())

            if all_passed:
                results.test_status = "PASS"
            elif critical_passed:
                results.test_status = "WARNING"
            else:
                results.test_status = "FAIL"

            if not recommendations:
                recommendations.append("All calibration metrics meet requirements")

            results.pass_criteria = pass_criteria
            results.recommendations = recommendations

        except Exception as e:
            self.logger.error(f"Calibration results evaluation failed: {e}")
            results.test_status = "ERROR"
            results.recommendations = [f"Evaluation failed: {str(e)}"]
