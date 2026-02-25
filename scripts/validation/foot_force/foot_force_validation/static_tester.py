#!/usr/bin/env python3
# scripts/validation/foot_force/foot_force_validation/static_tester.py
# Generated: 2025-06-27 14:18:30 CST
# Purpose: Unitree Go2 foot force sensor static validation tester

import time
import numpy as np
import logging
import threading
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass, field
from collections import deque
import json
from pathlib import Path
from datetime import datetime

from foot_force_config import FootForceConfig, FootForceReading
from data_collector import FootForceDataCollector, FootForceData

@dataclass
class StaticTestResult:
    """Static test result"""
    test_name: str
    status: str  # PASS, FAIL, WARNING
    score: float  # 0-100 points
    measurements: Dict[str, Any]
    analysis: Dict[str, Any]
    recommendations: List[str]
    timestamp: float

@dataclass
class ZeroDriftResult:
    """Zero drift test result"""
    initial_baseline: Dict[str, Tuple[float, float, float]]  # Initial baseline for each foot
    final_baseline: Dict[str, Tuple[float, float, float]]    # Final baseline
    drift_values: Dict[str, Tuple[float, float, float]]      # Drift values
    drift_rates: Dict[str, Tuple[float, float, float]]       # Drift rates (N/min)
    max_drift: float                                          # Maximum drift value
    drift_stability: float                                    # Drift stability score (0-100)

@dataclass
class ConsistencyResult:
    """Four-foot consistency test result"""
    cross_correlation: Dict[str, float]                       # Inter-foot correlation coefficients
    synchronization_delay: Dict[str, float]                   # Synchronization delay (ms)
    response_matching: Dict[str, float]                       # Response matching degree (0-1)
    calibration_differences: Dict[str, Tuple[float, float, float]]  # Calibration differences
    consistency_score: float                                  # Consistency score (0-100)

@dataclass
class BalanceAnalysis:
    """Balance analysis result"""
    center_of_pressure: Tuple[float, float]                  # Average center of pressure
    cop_std: Tuple[float, float]                             # Center of pressure standard deviation
    cop_range: Tuple[float, float]                           # Center of pressure range
    weight_distribution: Dict[str, float]                    # Weight distribution percentage
    stability_metrics: Dict[str, float]                      # Stability metrics
    balance_quality: float                                    # Balance quality score (0-100)

class StaticForceTester:
    """Static force distribution validation tester"""

    def __init__(self, config: Dict[str, Any], foot_force_config: FootForceConfig):
        """
        Initialize static tester

        Args:
            config: Configuration dictionary
            foot_force_config: Foot force configuration instance
        """
        self.config = config
        self.foot_force_config = foot_force_config
        self.logger = logging.getLogger(__name__)

        # Test parameters
        self.static_config = config.get('static_validation', {})
        self.force_threshold = config.get('foot_force_config', {}).get('force_threshold', 5.0)
        self.max_force_per_foot = config.get('foot_force_config', {}).get('max_force_per_foot', 200.0)

        # Test result storage
        self.test_results: List[StaticTestResult] = []
        self.zero_drift_result: Optional[ZeroDriftResult] = None
        self.consistency_result: Optional[ConsistencyResult] = None
        self.balance_analysis: Optional[BalanceAnalysis] = None

        # Data collector
        self.data_collector = FootForceDataCollector(config, foot_force_config)

        self.logger.info("Static force distribution validation tester initialization complete")

    def run_zero_load_test(self, duration: float = 30.0) -> StaticTestResult:
        """
        Zero load test - zero point test with robot suspended in air

        Args:
            duration: Test duration (seconds)

        Returns:
            StaticTestResult: Test result
        """
        self.logger.info(f"Starting zero load test, duration: {duration}s")

        try:
            # Start data collection
            if not self.data_collector.start_collection(duration):
                raise Exception("Zero load test data collection failed to start")

            # Wait for data collection to complete
            while self.data_collector.is_collecting:
                time.sleep(1.0)

            # Get data
            collected_data = self.data_collector.get_data()

            if len(collected_data) < 10:
                raise Exception("Insufficient zero load test data")

            # Analyze zero load data
            measurements = self._analyze_zero_load_data(collected_data)

            # Evaluate results
            score, status, analysis = self._evaluate_zero_load_results(measurements)

            # Generate recommendations
            recommendations = self._generate_zero_load_recommendations(measurements, analysis)

            result = StaticTestResult(
                test_name="zero_load_test",
                status=status,
                score=score,
                measurements=measurements,
                analysis=analysis,
                recommendations=recommendations,
                timestamp=time.time()
            )

            self.test_results.append(result)
            self.logger.info(f"Zero load test complete, score: {score:.1f}, status: {status}")

            return result

        except Exception as e:
            self.logger.error(f"Zero load test failed: {e}")
            error_result = StaticTestResult(
                test_name="zero_load_test",
                status="FAIL",
                score=0.0,
                measurements={},
                analysis={"error": str(e)},
                recommendations=["Check robot suspension state", "Verify sensor connection"],
                timestamp=time.time()
            )
            self.test_results.append(error_result)
            return error_result

    def _analyze_zero_load_data(self, data: List[FootForceData]) -> Dict[str, Any]:
        """Analyze zero load data"""
        measurements = {}

        # Extract force data
        forces_array = np.array([d.foot_forces for d in data])  # (samples, 4_feet, 3_axes)

        # Analyze each foot
        for i, foot_label in enumerate(FootForceConfig.FOOT_LABELS):
            foot_forces = forces_array[:, i, :]  # (samples, 3_axes)

            measurements[f"{foot_label}_stats"] = {
                "mean_fx": float(np.mean(foot_forces[:, 0])),
                "mean_fy": float(np.mean(foot_forces[:, 1])),
                "mean_fz": float(np.mean(foot_forces[:, 2])),
                "std_fx": float(np.std(foot_forces[:, 0])),
                "std_fy": float(np.std(foot_forces[:, 1])),
                "std_fz": float(np.std(foot_forces[:, 2])),
                "max_magnitude": float(np.max([np.linalg.norm(f) for f in foot_forces])),
                "rms_noise": float(np.sqrt(np.mean(np.sum(foot_forces**2, axis=1))))
            }

        # Overall statistics
        total_forces = np.array([d.total_force for d in data])
        measurements["total_force_stats"] = {
            "mean": float(np.mean(total_forces)),
            "std": float(np.std(total_forces)),
            "max": float(np.max(total_forces)),
            "rms": float(np.sqrt(np.mean(total_forces**2)))
        }

        # Center of pressure analysis
        cops = np.array([d.center_of_pressure for d in data])
        measurements["cop_stats"] = {
            "mean_x": float(np.mean(cops[:, 0])),
            "mean_y": float(np.mean(cops[:, 1])),
            "std_x": float(np.std(cops[:, 0])),
            "std_y": float(np.std(cops[:, 1])),
            "range_x": float(np.max(cops[:, 0]) - np.min(cops[:, 0])),
            "range_y": float(np.max(cops[:, 1]) - np.min(cops[:, 1]))
        }

        return measurements

    def _evaluate_zero_load_results(self, measurements: Dict[str, Any]) -> Tuple[float, str, Dict[str, Any]]:
        """Evaluate zero load test results"""
        analysis = {}
        score_components = []

        zero_threshold = self.static_config.get('zero_force_threshold', 2.0)

        # Evaluate zero point performance for each foot
        for foot_label in FootForceConfig.FOOT_LABELS:
            foot_stats = measurements[f"{foot_label}_stats"]

            # Score for mean value closeness to zero
            mean_magnitude = np.sqrt(foot_stats["mean_fx"]**2 +
                                   foot_stats["mean_fy"]**2 +
                                   foot_stats["mean_fz"]**2)
            zero_score = max(0, 100 - (mean_magnitude / zero_threshold) * 100)

            # Noise level score
            noise_score = max(0, 100 - (foot_stats["rms_noise"] / zero_threshold) * 100)

            # Stability score
            stability_score = max(0, 100 - (foot_stats["std_fz"] / zero_threshold) * 100)

            foot_score = (zero_score + noise_score + stability_score) / 3
            score_components.append(foot_score)

            analysis[f"{foot_label}_evaluation"] = {
                "zero_score": zero_score,
                "noise_score": noise_score,
                "stability_score": stability_score,
                "overall_score": foot_score,
                "mean_magnitude": mean_magnitude,
                "meets_zero_threshold": mean_magnitude < zero_threshold
            }

        # Overall score
        overall_score = np.mean(score_components)

        # Status determination
        if overall_score >= 90:
            status = "PASS"
        elif overall_score >= 70:
            status = "WARNING"
        else:
            status = "FAIL"

        analysis["overall_evaluation"] = {
            "score": overall_score,
            "status": status,
            "zero_threshold": zero_threshold,
            "all_feet_pass": all(measurements[f"{foot}_stats"]["rms_noise"] < zero_threshold
                               for foot in FootForceConfig.FOOT_LABELS)
        }

        return overall_score, status, analysis

    def _generate_zero_load_recommendations(self, measurements: Dict[str, Any],
                                          analysis: Dict[str, Any]) -> List[str]:
        """Generate zero load test recommendations"""
        recommendations = []

        zero_threshold = self.static_config.get('zero_force_threshold', 2.0)

        # Check each foot
        for foot_label in FootForceConfig.FOOT_LABELS:
            foot_stats = measurements[f"{foot_label}_stats"]
            foot_eval = analysis[f"{foot_label}_evaluation"]

            if not foot_eval["meets_zero_threshold"]:
                recommendations.append(f"{foot_label} foot zero offset too large ({foot_eval['mean_magnitude']:.2f}N), recalibration recommended")

            if foot_stats["rms_noise"] > zero_threshold * 0.5:
                recommendations.append(f"{foot_label} foot noise level too high ({foot_stats['rms_noise']:.2f}N), check sensor connection")

            if foot_stats["std_fz"] > zero_threshold * 0.3:
                recommendations.append(f"{foot_label} foot vertical force unstable, check mechanical installation")

        # Overall recommendations
        if analysis["overall_evaluation"]["score"] < 70:
            recommendations.append("Overall zero point performance is poor, full system recalibration recommended")
        elif analysis["overall_evaluation"]["score"] < 90:
            recommendations.append("Zero point performance acceptable but has room for improvement, regular monitoring recommended")

        if not recommendations:
            recommendations.append("Zero load test performance excellent, sensor zero point performance is normal")

        return recommendations

    def run_static_standing_test(self, duration: float = 60.0) -> StaticTestResult:
        """
        Static standing test - force distribution test during normal robot standing

        Args:
            duration: Test duration (seconds)

        Returns:
            StaticTestResult: Test result
        """
        self.logger.info(f"Starting static standing test, duration: {duration}s")

        try:
            # Start data collection
            if not self.data_collector.start_collection(duration):
                raise Exception("Static standing test data collection failed to start")

            # Wait for data collection to complete
            while self.data_collector.is_collecting:
                time.sleep(2.0)  # Check status every 2 seconds

            # Get data
            collected_data = self.data_collector.get_data()

            if len(collected_data) < 30:
                raise Exception("Insufficient static standing test data")

            # Analyze static standing data
            measurements = self._analyze_static_standing_data(collected_data)

            # Evaluate results
            score, status, analysis = self._evaluate_static_standing_results(measurements)

            # Generate recommendations
            recommendations = self._generate_static_standing_recommendations(measurements, analysis)

            result = StaticTestResult(
                test_name="static_standing_test",
                status=status,
                score=score,
                measurements=measurements,
                analysis=analysis,
                recommendations=recommendations,
                timestamp=time.time()
            )

            self.test_results.append(result)
            self.logger.info(f"Static standing test complete, score: {score:.1f}, status: {status}")

            return result

        except Exception as e:
            self.logger.error(f"Static standing test failed: {e}")
            error_result = StaticTestResult(
                test_name="static_standing_test",
                status="FAIL",
                score=0.0,
                measurements={},
                analysis={"error": str(e)},
                recommendations=["Check robot standing posture", "Verify ground level"],
                timestamp=time.time()
            )
            self.test_results.append(error_result)
            return error_result

    def _analyze_static_standing_data(self, data: List[FootForceData]) -> Dict[str, Any]:
        """Analyze static standing data"""
        measurements = {}

        # Extract data arrays
        forces_array = np.array([d.foot_forces for d in data])
        total_forces = np.array([d.total_force for d in data])
        cops = np.array([d.center_of_pressure for d in data])
        stability_indices = np.array([d.stability_index for d in data])
        balance_indices = np.array([d.force_balance for d in data])

        # Detailed analysis for each foot
        for i, foot_label in enumerate(FootForceConfig.FOOT_LABELS):
            foot_forces = forces_array[:, i, :]

            measurements[f"{foot_label}_analysis"] = {
                "mean_force": foot_forces.mean(axis=0).tolist(),
                "std_force": foot_forces.std(axis=0).tolist(),
                "mean_vertical": float(np.mean(foot_forces[:, 2])),
                "std_vertical": float(np.std(foot_forces[:, 2])),
                "force_range": float(np.max(foot_forces[:, 2]) - np.min(foot_forces[:, 2])),
                "contact_ratio": float(np.mean(foot_forces[:, 2] > self.force_threshold)),
                "force_stability": float(1.0 - np.std(foot_forces[:, 2]) / (np.mean(foot_forces[:, 2]) + 1e-6))
            }

        # Weight distribution analysis
        vertical_forces = forces_array[:, :, 2]  # Extract vertical forces
        mean_vertical_forces = np.mean(vertical_forces, axis=0)
        total_weight = np.sum(mean_vertical_forces)

        weight_distribution = {}
        for i, foot_label in enumerate(FootForceConfig.FOOT_LABELS):
            weight_distribution[foot_label] = float(mean_vertical_forces[i] / total_weight * 100) if total_weight > 0 else 0.0

        measurements["weight_distribution"] = weight_distribution
        measurements["total_weight"] = float(total_weight)

        # Balance analysis
        measurements["balance_analysis"] = {
            "cop_mean": cops.mean(axis=0).tolist(),
            "cop_std": cops.std(axis=0).tolist(),
            "cop_range": (cops.max(axis=0) - cops.min(axis=0)).tolist(),
            "stability_mean": float(np.mean(stability_indices)),
            "stability_std": float(np.std(stability_indices)),
            "balance_mean": float(np.mean(balance_indices)),
            "balance_std": float(np.std(balance_indices))
        }

        # Total force analysis
        measurements["total_force_analysis"] = {
            "mean": float(np.mean(total_forces)),
            "std": float(np.std(total_forces)),
            "coefficient_of_variation": float(np.std(total_forces) / (np.mean(total_forces) + 1e-6)),
            "force_range": float(np.max(total_forces) - np.min(total_forces))
        }

        return measurements

    def _evaluate_static_standing_results(self, measurements: Dict[str, Any]) -> Tuple[float, str, Dict[str, Any]]:
        """Evaluate static standing test results"""
        analysis = {}
        score_components = []

        expected_total_force = self.static_config.get('expected_total_force', 150.0)  # N
        tolerance = self.static_config.get('tolerance', 10.0)  # N
        balance_threshold = self.static_config.get('balance_threshold', 0.1)

        # 1. Total weight accuracy score
        total_weight = measurements["total_weight"]
        weight_error = abs(total_weight - expected_total_force)
        weight_score = max(0, 100 - (weight_error / tolerance) * 100)
        score_components.append(weight_score)

        analysis["weight_accuracy"] = {
            "measured_weight": total_weight,
            "expected_weight": expected_total_force,
            "error": weight_error,
            "error_percentage": weight_error / expected_total_force * 100,
            "score": weight_score,
            "within_tolerance": weight_error <= tolerance
        }

        # 2. Weight distribution balance score
        weight_dist = measurements["weight_distribution"]
        ideal_distribution = 25.0  # Ideally 25% per foot
        distribution_errors = [abs(weight_dist[foot] - ideal_distribution) for foot in FootForceConfig.FOOT_LABELS]
        max_distribution_error = max(distribution_errors)
        distribution_score = max(0, 100 - max_distribution_error * 4)  # 4 is the scaling factor
        score_components.append(distribution_score)

        analysis["weight_distribution_analysis"] = {
            "distribution": weight_dist,
            "distribution_errors": dict(zip(FootForceConfig.FOOT_LABELS, distribution_errors)),
            "max_error": max_distribution_error,
            "score": distribution_score,
            "is_balanced": max_distribution_error <= 10.0  # Within 10% error
        }

        # 3. Stability score
        balance_analysis = measurements["balance_analysis"]
        stability_score = balance_analysis["stability_mean"] * 100
        score_components.append(stability_score)

        # 4. Center of pressure stability score
        cop_std = np.linalg.norm(balance_analysis["cop_std"])
        cop_score = max(0, 100 - cop_std * 1000)  # Assuming ideal COP std < 0.1m
        score_components.append(cop_score)

        analysis["stability_analysis"] = {
            "stability_index": balance_analysis["stability_mean"],
            "balance_index": balance_analysis["balance_mean"],
            "cop_stability": cop_std,
            "stability_score": stability_score,
            "cop_score": cop_score
        }

        # 5. Consistency score for each foot
        foot_scores = []
        for foot_label in FootForceConfig.FOOT_LABELS:
            foot_analysis = measurements[f"{foot_label}_analysis"]
            foot_stability = foot_analysis["force_stability"]
            foot_score = foot_stability * 100
            foot_scores.append(foot_score)

            analysis[f"{foot_label}_performance"] = {
                "mean_vertical_force": foot_analysis["mean_vertical"],
                "force_stability": foot_stability,
                "contact_ratio": foot_analysis["contact_ratio"],
                "score": foot_score
            }

        consistency_score = np.mean(foot_scores)
        score_components.append(consistency_score)

        # Overall score
        overall_score = np.mean(score_components)

        # Status determination
        if overall_score >= 85:
            status = "PASS"
        elif overall_score >= 70:
            status = "WARNING"
        else:
            status = "FAIL"

        analysis["overall_evaluation"] = {
            "score": overall_score,
            "status": status,
            "score_components": {
                "weight_accuracy": weight_score,
                "weight_distribution": distribution_score,
                "stability": stability_score,
                "cop_stability": cop_score,
                "foot_consistency": consistency_score
            }
        }

        return overall_score, status, analysis

    def _generate_static_standing_recommendations(self, measurements: Dict[str, Any],
                                                analysis: Dict[str, Any]) -> List[str]:
        """Generate static standing test recommendations"""
        recommendations = []

        # Weight accuracy recommendations
        weight_analysis = analysis["weight_accuracy"]
        if not weight_analysis["within_tolerance"]:
            recommendations.append(f"Total weight error too large ({weight_analysis['error']:.1f}N), sensor calibration check recommended")

        # Weight distribution recommendations
        dist_analysis = analysis["weight_distribution_analysis"]
        if not dist_analysis["is_balanced"]:
            max_error_foot = max(dist_analysis["distribution_errors"].items(), key=lambda x: x[1])
            recommendations.append(f"{max_error_foot[0]} foot bearing abnormal ({max_error_foot[1]:.1f}% deviation), check robot posture or ground level")

        # Stability recommendations
        stability_analysis = analysis["stability_analysis"]
        if stability_analysis["stability_score"] < 70:
            recommendations.append("Force distribution stability is poor, check mechanical structure and sensor installation")

        if stability_analysis["cop_score"] < 70:
            recommendations.append("Center of pressure fluctuation too large, check ground condition and robot balance")

        # Individual foot recommendations
        for foot_label in FootForceConfig.FOOT_LABELS:
            foot_perf = analysis[f"{foot_label}_performance"]
            if foot_perf["score"] < 70:
                recommendations.append(f"{foot_label} foot performance is poor, check sensor and mechanical components for that foot")

            if foot_perf["contact_ratio"] < 0.95:
                recommendations.append(f"{foot_label} foot contact unstable, check ground contact condition")

        if not recommendations:
            recommendations.append("Static standing test performance excellent, force distribution and stability are normal")

        return recommendations

    def run_zero_drift_analysis(self, duration: float = 300.0) -> ZeroDriftResult:
        """
        Zero drift analysis - long-term monitoring of sensor zero point stability

        Args:
            duration: Monitoring duration (seconds)

        Returns:
            ZeroDriftResult: Drift analysis result
        """
        self.logger.info(f"Starting zero drift analysis, duration: {duration}s")

        try:
            # Collect data in segments to analyze drift trends
            segment_duration = 30.0  # 30 seconds per segment
            num_segments = int(duration / segment_duration)

            baseline_data = []
            timestamps = []

            for segment in range(num_segments):
                self.logger.info(f"Collecting segment {segment+1}/{num_segments} data...")

                # Collect data for this segment
                if not self.data_collector.start_collection(segment_duration):
                    raise Exception(f"Segment {segment+1} data collection failed")

                while self.data_collector.is_collecting:
                    time.sleep(1.0)

                segment_data = self.data_collector.get_data()

                if len(segment_data) < 10:
                    self.logger.warning(f"Segment {segment+1} data insufficient, skipping")
                    continue

                # Calculate baseline for this segment
                forces_array = np.array([d.foot_forces for d in segment_data])
                segment_baseline = {}

                for i, foot_label in enumerate(FootForceConfig.FOOT_LABELS):
                    foot_forces = forces_array[:, i, :]
                    segment_baseline[foot_label] = tuple(np.mean(foot_forces, axis=0))

                baseline_data.append(segment_baseline)
                timestamps.append(time.time())

                # Rest between segments
                if segment < num_segments - 1:
                    time.sleep(2.0)

            if len(baseline_data) < 3:
                raise Exception("Insufficient data segments for drift analysis")

            # Analyze drift trends
            drift_result = self._analyze_zero_drift(baseline_data, timestamps)

            self.zero_drift_result = drift_result
            self.logger.info(f"Zero drift analysis complete, max drift: {drift_result.max_drift:.3f}N")

            return drift_result

        except Exception as e:
            self.logger.error(f"Zero drift analysis failed: {e}")
            # Return empty drift result
            return ZeroDriftResult(
                initial_baseline={},
                final_baseline={},
                drift_values={},
                drift_rates={},
                max_drift=0.0,
                drift_stability=0.0
            )

    def _analyze_zero_drift(self, baseline_data: List[Dict[str, Tuple[float, float, float]]],
                           timestamps: List[float]) -> ZeroDriftResult:
        """Analyze zero drift data"""

        if len(baseline_data) < 2:
            return ZeroDriftResult({}, {}, {}, {}, 0.0, 0.0)

        initial_baseline = baseline_data[0]
        final_baseline = baseline_data[-1]

        # Calculate drift values and drift rates
        drift_values = {}
        drift_rates = {}
        max_drift = 0.0

        duration_minutes = (timestamps[-1] - timestamps[0]) / 60.0

        for foot_label in FootForceConfig.FOOT_LABELS:
            if foot_label in initial_baseline and foot_label in final_baseline:
                initial = np.array(initial_baseline[foot_label])
                final = np.array(final_baseline[foot_label])

                drift = final - initial
                drift_magnitude = np.linalg.norm(drift)
                drift_rate = drift / duration_minutes if duration_minutes > 0 else np.zeros_like(drift)

                drift_values[foot_label] = tuple(drift)
                drift_rates[foot_label] = tuple(drift_rate)

                max_drift = max(max_drift, drift_magnitude)

        # Calculate drift stability score
        drift_stability = max(0, 100 - max_drift * 50)  # Assuming drift over 2N is very bad

        return ZeroDriftResult(
            initial_baseline=initial_baseline,
            final_baseline=final_baseline,
            drift_values=drift_values,
            drift_rates=drift_rates,
            max_drift=max_drift,
            drift_stability=drift_stability
        )

    def generate_static_validation_report(self) -> Dict[str, Any]:
        """Generate comprehensive static validation report"""
        report = {
            "test_timestamp": datetime.now().isoformat(),
            "test_results": [result.__dict__ for result in self.test_results],
            "zero_drift_analysis": self.zero_drift_result.__dict__ if self.zero_drift_result else None,
            "consistency_analysis": self.consistency_result.__dict__ if self.consistency_result else None,
            "balance_analysis": self.balance_analysis.__dict__ if self.balance_analysis else None
        }

        # Overall assessment
        if self.test_results:
            overall_score = np.mean([result.score for result in self.test_results])
            all_pass = all(result.status == "PASS" for result in self.test_results)

            report["overall_assessment"] = {
                "overall_score": overall_score,
                "status": "PASS" if all_pass and overall_score >= 80 else "FAIL" if overall_score < 60 else "WARNING",
                "total_tests": len(self.test_results),
                "passed_tests": sum(1 for result in self.test_results if result.status == "PASS"),
                "failed_tests": sum(1 for result in self.test_results if result.status == "FAIL")
            }

        return report

    def save_static_validation_report(self, filepath: str) -> bool:
        """Save static validation report"""
        try:
            report = self.generate_static_validation_report()

            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(report, f, indent=2, ensure_ascii=False)

            self.logger.info(f"Static validation report saved to: {filepath}")
            return True

        except Exception as e:
            self.logger.error(f"Failed to save static validation report: {e}")
            return False

    def run_quick_static_test(self, duration: float = 10.0) -> Dict[str, Any]:
        """
        Run quick static test - for complete validation workflow

        Args:
            duration: Quick test duration (seconds)

        Returns:
            Dict: Quick test result
        """
        self.logger.info(f"Starting quick static test, duration: {duration}s")

        try:
            # Run zero load test
            zero_result = self.run_zero_load_test(duration)

            # Generate simplified result
            quick_result = {
                'success': zero_result.status != "FAIL",
                'test_name': 'quick_static_test',
                'status': zero_result.status,
                'final_score': zero_result.score,
                'duration': duration,
                'measurements': zero_result.measurements,
                'recommendations': zero_result.recommendations,
                'timestamp': zero_result.timestamp
            }

            self.logger.info(f"Quick static test complete, score: {zero_result.score:.1f}")
            return quick_result

        except Exception as e:
            self.logger.error(f"Quick static test failed: {e}")
            return {
                'success': False,
                'test_name': 'quick_static_test',
                'status': 'FAIL',
                'final_score': 0.0,
                'error': str(e),
                'timestamp': time.time()
            }


# Add alias for compatibility
StaticFootForceTester = StaticForceTester
