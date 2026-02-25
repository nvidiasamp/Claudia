#!/usr/bin/env python3
# scripts/validation/foot_force/foot_force_validation/dynamic_tester.py
# Generated: 2025-06-26 18:40:00
# Purpose: Unitree Go2 foot force sensor dynamic response test framework

import time
import threading
import logging
import numpy as np
from typing import Dict, List, Tuple, Any, Optional, Callable
from dataclasses import dataclass, field
import json
from datetime import datetime
from pathlib import Path

from foot_force_config import FootForceConfig, FootForceReading
from data_collector import FootForceDataCollector, FootForceData, FootForceCollectionMetrics

@dataclass
class GaitPhase:
    """Gait phase data"""
    phase_name: str  # "stance", "swing", "impact", "lift_off"
    start_time: float
    end_time: float
    foot_id: int
    peak_force: float
    average_force: float
    force_profile: List[float]
    contact_quality: float

@dataclass
class DynamicTestResult:
    """Dynamic test result"""
    test_name: str
    start_time: float
    end_time: float
    duration: float
    total_samples: int

    # Gait analysis results
    gait_phases: List[GaitPhase] = field(default_factory=list)
    step_frequency: float = 0.0
    stride_length: float = 0.0
    contact_time_avg: float = 0.0
    flight_time_avg: float = 0.0

    # Mechanical characteristics
    peak_forces: Dict[str, float] = field(default_factory=dict)
    impulse_values: Dict[str, float] = field(default_factory=dict)
    force_rise_times: Dict[str, float] = field(default_factory=dict)
    force_fall_times: Dict[str, float] = field(default_factory=dict)

    # Stability metrics
    stability_variance: float = 0.0
    balance_consistency: float = 0.0
    coordination_index: float = 0.0

    # Test score
    test_score: float = 0.0
    pass_criteria: Dict[str, bool] = field(default_factory=dict)

    # Raw data statistics
    raw_data_stats: Dict[str, Any] = field(default_factory=dict)

class DynamicFootForceTester:
    """Dynamic foot force sensor tester"""

    def __init__(self, config: Dict, foot_force_config: FootForceConfig):
        """
        Initialize dynamic tester

        Args:
            config: Configuration dictionary
            foot_force_config: FootForceConfig instance
        """
        self.config = config
        self.foot_force_config = foot_force_config
        self.logger = logging.getLogger(__name__)

        # Dynamic validation configuration
        self.dynamic_config = config.get('dynamic_validation', {})

        # Data collector
        self.data_collector = FootForceDataCollector(config, foot_force_config)

        # Test state
        self.is_testing = False
        self.current_test = None

        # Real-time analysis
        self.step_detector = StepDetector(self.dynamic_config)
        self.gait_analyzer = GaitAnalyzer(self.dynamic_config)

        self.logger.info("Dynamic foot force tester initialization complete")

    def run_dynamic_test_suite(self) -> Dict[str, DynamicTestResult]:
        """Run the complete dynamic test suite"""
        test_results = {}

        # Get test scenarios from configuration
        test_scenarios = self.config.get('test_scenarios', {}).get('dynamic_tests', [])

        self.logger.info(f"Starting dynamic test suite with {len(test_scenarios)} tests")

        for scenario in test_scenarios:
            test_name = scenario['name']
            self.logger.info(f"Starting test: {test_name}")

            try:
                # Run single test
                result = self._run_single_dynamic_test(scenario)
                test_results[test_name] = result

                # Wait between tests
                time.sleep(5.0)

            except Exception as e:
                self.logger.error(f"Test {test_name} failed: {e}")
                # Create failure result
                test_results[test_name] = DynamicTestResult(
                    test_name=test_name,
                    start_time=time.time(),
                    end_time=time.time(),
                    duration=0.0,
                    total_samples=0,
                    test_score=0.0
                )

        self.logger.info("Dynamic test suite complete")
        return test_results

    def _run_single_dynamic_test(self, scenario: Dict) -> DynamicTestResult:
        """Run a single dynamic test"""
        test_name = scenario['name']
        duration = scenario.get('duration', 60.0)

        self.logger.info(f"Executing dynamic test: {test_name}, duration: {duration}s")

        # Initialize test result
        result = DynamicTestResult(
            test_name=test_name,
            start_time=time.time(),
            end_time=0.0,
            duration=duration,
            total_samples=0
        )

        # Reset analyzers
        self.step_detector.reset()
        self.gait_analyzer.reset()

        # Set data callback
        self.data_collector.add_data_callback(self._dynamic_analysis_callback)

        # Start data collection
        if not self.data_collector.start_collection(duration):
            raise RuntimeError(f"Unable to start data collection: {test_name}")

        # Wait for test to complete
        self.is_testing = True
        self.current_test = test_name

        # Display test instructions
        self._display_test_instructions(scenario)

        # Wait for data collection to complete
        time.sleep(duration + 1.0)

        # Stop collection and get metrics
        collection_metrics = self.data_collector.stop_collection()
        self.is_testing = False

        # Complete result statistics
        result.end_time = time.time()
        result.total_samples = collection_metrics.total_samples

        # Perform dynamic analysis
        collected_data = self.data_collector.get_data()
        result = self._analyze_dynamic_data(result, collected_data, scenario)

        # Calculate test score
        result.test_score = self._calculate_dynamic_test_score(result, scenario)

        self.logger.info(f"Test {test_name} complete, score: {result.test_score:.1f}/100")

        return result

    def _display_test_instructions(self, scenario: Dict):
        """Display test instructions"""
        test_name = scenario['name']
        duration = scenario.get('duration', 60)

        print(f"\n{'='*60}")
        print(f"Dynamic Test: {test_name}")
        print(f"Description: {scenario.get('description', 'No description')}")
        print(f"Duration: {duration}s")
        print(f"{'='*60}")

        if test_name == "slow_walk":
            print("Please perform the following:")
            print("   1. Have the robot walk at a slow speed")
            print("   2. Maintain a steady gait rhythm")
            print("   3. Avoid sudden stops or turns")

        elif test_name == "normal_walk":
            print("Please perform the following:")
            print("   1. Have the robot walk at normal speed")
            print("   2. Observe four-legged coordinated movement")
            print("   3. Minor turns are acceptable")

        elif test_name == "impact_test":
            print("Please perform the following:")
            print("   1. Have the robot perform a jump")
            print("   2. Observe the impact force upon landing")
            print("   3. Ensure safe distance")

        print(f"\nTest will begin in 3 seconds...")
        for i in range(3, 0, -1):
            print(f"   {i}...")
            time.sleep(1)
        print("Test started!")

    def _dynamic_analysis_callback(self, data: FootForceData):
        """Dynamic analysis data callback"""
        if not self.is_testing:
            return

        try:
            # Gait detection
            self.step_detector.process_data(data)

            # Gait analysis
            self.gait_analyzer.process_data(data)

        except Exception as e:
            self.logger.error(f"Dynamic analysis callback error: {e}")

    def _analyze_dynamic_data(self, result: DynamicTestResult, data: List[FootForceData], scenario: Dict) -> DynamicTestResult:
        """Analyze dynamic data"""
        if not data:
            return result

        # Get gait detection results
        result.gait_phases = self.gait_analyzer.get_gait_phases()
        result.step_frequency = self.step_detector.get_step_frequency()

        # Calculate contact times
        contact_times = self._calculate_contact_times(data)
        result.contact_time_avg = np.mean(contact_times) if contact_times else 0.0

        # Calculate peak forces
        result.peak_forces = self._calculate_peak_forces(data)

        # Calculate impulse values
        result.impulse_values = self._calculate_impulse_values(data)

        # Calculate force rise/fall times
        result.force_rise_times, result.force_fall_times = self._calculate_force_transition_times(data)

        # Stability analysis
        result.stability_variance = self._calculate_stability_variance(data)
        result.balance_consistency = self._calculate_balance_consistency(data)
        result.coordination_index = self._calculate_coordination_index(data)

        # Raw data statistics
        result.raw_data_stats = self._calculate_raw_data_statistics(data)

        return result

    def _calculate_contact_times(self, data: List[FootForceData]) -> List[float]:
        """Calculate contact times"""
        contact_times = []

        for foot_id in range(4):
            contact_periods = []
            in_contact = False
            contact_start = 0.0

            for sample in data:
                if sample.contact_states[foot_id] and not in_contact:
                    # Contact started
                    in_contact = True
                    contact_start = sample.timestamp
                elif not sample.contact_states[foot_id] and in_contact:
                    # Contact ended
                    in_contact = False
                    contact_duration = sample.timestamp - contact_start
                    contact_periods.append(contact_duration)

            contact_times.extend(contact_periods)

        return contact_times

    def _calculate_peak_forces(self, data: List[FootForceData]) -> Dict[str, float]:
        """Calculate peak forces"""
        peak_forces = {}

        for foot_id, foot_label in enumerate(FootForceConfig.FOOT_LABELS):
            foot_forces = [sample.force_magnitude[foot_id] for sample in data]
            peak_forces[foot_label] = float(np.max(foot_forces)) if foot_forces else 0.0

        # Total peak force
        total_forces = [sample.total_force for sample in data]
        peak_forces['total'] = float(np.max(total_forces)) if total_forces else 0.0

        return peak_forces

    def _calculate_impulse_values(self, data: List[FootForceData]) -> Dict[str, float]:
        """Calculate impulse values (force multiplied by time)"""
        impulse_values = {}

        if len(data) < 2:
            return impulse_values

        dt = (data[-1].timestamp - data[0].timestamp) / len(data)

        for foot_id, foot_label in enumerate(FootForceConfig.FOOT_LABELS):
            foot_forces = [sample.force_magnitude[foot_id] for sample in data]
            impulse = float(np.sum(foot_forces) * dt)
            impulse_values[foot_label] = impulse

        return impulse_values

    def _calculate_force_transition_times(self, data: List[FootForceData]) -> Tuple[Dict[str, float], Dict[str, float]]:
        """Calculate force rise and fall times"""
        rise_times = {}
        fall_times = {}

        for foot_id, foot_label in enumerate(FootForceConfig.FOOT_LABELS):
            foot_forces = [sample.force_magnitude[foot_id] for sample in data]

            # Find force rise and fall edges
            rise_time_samples = []
            fall_time_samples = []

            for i in range(1, len(foot_forces)):
                force_change = foot_forces[i] - foot_forces[i-1]

                if force_change > 5.0:  # Rapid force increase
                    rise_time_samples.append(i)
                elif force_change < -5.0:  # Rapid force decrease
                    fall_time_samples.append(i)

            # Calculate average time (simplified calculation)
            rise_times[foot_label] = len(rise_time_samples) * 0.002 if rise_time_samples else 0.0  # Assuming 500Hz sampling rate
            fall_times[foot_label] = len(fall_time_samples) * 0.002 if fall_time_samples else 0.0

        return rise_times, fall_times

    def _calculate_stability_variance(self, data: List[FootForceData]) -> float:
        """Calculate stability variance"""
        stability_indices = [sample.stability_index for sample in data]
        return float(np.var(stability_indices)) if stability_indices else 0.0

    def _calculate_balance_consistency(self, data: List[FootForceData]) -> float:
        """Calculate balance consistency"""
        balance_indices = [sample.force_balance for sample in data]
        return float(np.mean(balance_indices)) if balance_indices else 0.0

    def _calculate_coordination_index(self, data: List[FootForceData]) -> float:
        """Calculate coordination index"""
        # Analyze the phase relationship of four feet
        foot_phases = [[] for _ in range(4)]

        for sample in data:
            for foot_id in range(4):
                # Simplified phase calculation: based on force magnitude
                phase = sample.force_magnitude[foot_id] / (sample.total_force + 1e-6)
                foot_phases[foot_id].append(phase)

        # Calculate inter-foot correlation
        correlations = []
        for i in range(4):
            for j in range(i+1, 4):
                if foot_phases[i] and foot_phases[j]:
                    corr = np.corrcoef(foot_phases[i], foot_phases[j])[0, 1]
                    if not np.isnan(corr):
                        correlations.append(abs(corr))

        return float(np.mean(correlations)) if correlations else 0.0

    def _calculate_raw_data_statistics(self, data: List[FootForceData]) -> Dict[str, Any]:
        """Calculate raw data statistics"""
        if not data:
            return {}

        total_forces = [sample.total_force for sample in data]
        cops_x = [sample.center_of_pressure[0] for sample in data]
        cops_y = [sample.center_of_pressure[1] for sample in data]

        return {
            'total_force_mean': float(np.mean(total_forces)),
            'total_force_std': float(np.std(total_forces)),
            'total_force_max': float(np.max(total_forces)),
            'cop_x_range': float(np.max(cops_x) - np.min(cops_x)),
            'cop_y_range': float(np.max(cops_y) - np.min(cops_y)),
            'contact_rate_avg': float(np.mean([np.mean(sample.contact_states) for sample in data])),
            'sample_count': len(data)
        }

    def _calculate_dynamic_test_score(self, result: DynamicTestResult, scenario: Dict) -> float:
        """Calculate dynamic test score"""
        score = 0.0
        max_score = 100.0

        # Data quality score (30%)
        if result.total_samples > 0:
            expected_samples = scenario.get('duration', 60) * 500  # Assuming 500Hz
            sample_ratio = min(result.total_samples / expected_samples, 1.0)
            score += 30.0 * sample_ratio

        # Gait consistency score (25%)
        if result.coordination_index > 0:
            coordination_score = min(result.coordination_index * 100, 25.0)
            score += coordination_score

        # Stability score (20%)
        if result.stability_variance >= 0:
            stability_score = max(0, 20.0 - result.stability_variance * 100)
            score += min(stability_score, 20.0)

        # Balance score (15%)
        balance_score = result.balance_consistency * 15.0
        score += min(balance_score, 15.0)

        # Specific test metrics (10%)
        if scenario['name'] == 'impact_test':
            # Impact test: check if peak force is reasonable
            max_expected_force = scenario.get('max_expected_force', 250)
            total_peak = result.peak_forces.get('total', 0)
            if 50 <= total_peak <= max_expected_force:
                score += 10.0
        else:
            # Walking test: check if step frequency is reasonable
            if 0.5 <= result.step_frequency <= 3.0:
                score += 10.0

        return min(score, max_score)

    def save_dynamic_test_results(self, results: Dict[str, DynamicTestResult], output_dir: str) -> str:
        """Save dynamic test results"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = Path(output_dir) / f"dynamic_test_results_{timestamp}.json"

        # Convert to serializable format
        serializable_results = {}
        for test_name, result in results.items():
            serializable_results[test_name] = {
                'test_name': result.test_name,
                'start_time': result.start_time,
                'end_time': result.end_time,
                'duration': result.duration,
                'total_samples': result.total_samples,
                'step_frequency': result.step_frequency,
                'contact_time_avg': result.contact_time_avg,
                'peak_forces': result.peak_forces,
                'impulse_values': result.impulse_values,
                'stability_variance': result.stability_variance,
                'balance_consistency': result.balance_consistency,
                'coordination_index': result.coordination_index,
                'test_score': result.test_score,
                'raw_data_stats': result.raw_data_stats
            }

        # Add summary information
        summary = {
            'total_tests': len(results),
            'average_score': np.mean([r.test_score for r in results.values()]) if results else 0.0,
            'test_timestamp': timestamp,
            'config_used': self.dynamic_config
        }

        output_data = {
            'summary': summary,
            'test_results': serializable_results
        }

        try:
            output_file.parent.mkdir(parents=True, exist_ok=True)
            with open(output_file, 'w', encoding='utf-8') as f:
                json.dump(output_data, f, indent=2, ensure_ascii=False)

            self.logger.info(f"Dynamic test results saved to: {output_file}")
            return str(output_file)

        except Exception as e:
            self.logger.error(f"Failed to save dynamic test results: {e}")
            return ""


class StepDetector:
    """Step detector"""

    def __init__(self, config: Dict):
        self.config = config
        self.threshold = config.get('step_detection_threshold', 15.0)
        self.step_events = []
        self.last_contact_states = [False] * 4

    def reset(self):
        """Reset detector"""
        self.step_events.clear()
        self.last_contact_states = [False] * 4

    def process_data(self, data: FootForceData):
        """Process data and detect steps"""
        # Detect contact changes for each foot
        for foot_id in range(4):
            current_contact = data.contact_states[foot_id]
            last_contact = self.last_contact_states[foot_id]

            # Detect touchdown event
            if current_contact and not last_contact:
                self.step_events.append({
                    'timestamp': data.timestamp,
                    'foot_id': foot_id,
                    'event_type': 'touchdown',
                    'force': data.force_magnitude[foot_id]
                })

            # Detect liftoff event
            elif not current_contact and last_contact:
                self.step_events.append({
                    'timestamp': data.timestamp,
                    'foot_id': foot_id,
                    'event_type': 'liftoff',
                    'force': data.force_magnitude[foot_id]
                })

        self.last_contact_states = data.contact_states.copy()

    def get_step_frequency(self) -> float:
        """Calculate step frequency"""
        touchdown_events = [e for e in self.step_events if e['event_type'] == 'touchdown']

        if len(touchdown_events) < 2:
            return 0.0

        total_time = touchdown_events[-1]['timestamp'] - touchdown_events[0]['timestamp']
        step_count = len(touchdown_events)

        return (step_count - 1) / total_time if total_time > 0 else 0.0


class GaitAnalyzer:
    """Gait analyzer"""

    def __init__(self, config: Dict):
        self.config = config
        self.gait_phases = []
        self.current_phases = {}  # Track current phase for each foot

    def reset(self):
        """Reset analyzer"""
        self.gait_phases.clear()
        self.current_phases.clear()

    def process_data(self, data: FootForceData):
        """Process data and analyze gait"""
        # Analyze current phase for each foot
        for foot_id in range(4):
            self._analyze_foot_phase(foot_id, data)

    def _analyze_foot_phase(self, foot_id: int, data: FootForceData):
        """Analyze gait phase for a single foot"""
        is_contact = data.contact_states[foot_id]
        force = data.force_magnitude[foot_id]

        # Get current phase state
        current_phase = self.current_phases.get(foot_id, None)

        if is_contact and force > 10.0:
            # Stance phase
            if current_phase is None or current_phase['phase_name'] != 'stance':
                # Start new stance phase
                if current_phase:
                    # End previous phase
                    current_phase['end_time'] = data.timestamp
                    self.gait_phases.append(GaitPhase(**current_phase))

                # Start new phase
                self.current_phases[foot_id] = {
                    'phase_name': 'stance',
                    'start_time': data.timestamp,
                    'end_time': 0.0,
                    'foot_id': foot_id,
                    'peak_force': force,
                    'average_force': force,
                    'force_profile': [force],
                    'contact_quality': 1.0 if is_contact else 0.0
                }
            else:
                # Continue stance phase
                current_phase['peak_force'] = max(current_phase['peak_force'], force)
                current_phase['force_profile'].append(force)
                # Update average force
                current_phase['average_force'] = np.mean(current_phase['force_profile'])

        else:
            # Swing phase
            if current_phase and current_phase['phase_name'] == 'stance':
                # End stance phase
                current_phase['end_time'] = data.timestamp
                self.gait_phases.append(GaitPhase(**current_phase))

                # Start swing phase
                self.current_phases[foot_id] = {
                    'phase_name': 'swing',
                    'start_time': data.timestamp,
                    'end_time': 0.0,
                    'foot_id': foot_id,
                    'peak_force': 0.0,
                    'average_force': 0.0,
                    'force_profile': [0.0],
                    'contact_quality': 0.0
                }
            elif current_phase and current_phase['phase_name'] == 'swing':
                # Continue swing phase
                current_phase['force_profile'].append(0.0)

    def get_gait_phases(self) -> List[GaitPhase]:
        """Get gait phase list"""
        # End all ongoing phases
        current_time = time.time()
        for foot_id, phase in self.current_phases.items():
            if phase and phase['end_time'] == 0.0:
                phase['end_time'] = current_time
                self.gait_phases.append(GaitPhase(**phase))

        return self.gait_phases
