#!/usr/bin/env python3
# scripts/validation/imu/imu_validation/main_validation_script.py
# Generated: 2025-06-27 11:54:45 CST
# Purpose: Unitree Go2 IMU main validation script - complete IMU sensor validation process

import os
import sys
import time
import json
import logging
import argparse
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Any, Optional

# Add module path
current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir))

from imu_config import IMUConfig, IMUSpec
from data_collector import IMUDataCollector
from visualizer import IMUVisualizer
from static_tester import IMUStaticTester
from dynamic_tester import IMUDynamicTester
from calibration_analyzer import IMUCalibrationAnalyzer

class IMUValidationSuite:
    """IMU validation test suite"""

    def __init__(self, config_file: str = None):
        """
        Initialize validation suite

        Args:
            config_file: Configuration file path
        """
        self.logger = self._setup_logging()
        self.config = self._load_config(config_file)

        # Initialize components
        self.imu_config = None
        self.data_collector = None
        self.visualizer = None
        self.static_tester = None
        self.dynamic_tester = None
        self.calibration_analyzer = None

        # Test results
        self.test_results = {
            'test_info': {
                'start_time': datetime.now().isoformat(),
                'test_version': '1.0.0',
                'robot_model': 'Unitree Go2',
                'test_operator': os.getenv('USER', 'unknown')
            },
            'initialization': {},
            'static_test': {},
            'dynamic_test': {},
            'calibration_analysis': {},
            'visualization_test': {},
            'overall_assessment': {}
        }

    def _setup_logging(self) -> logging.Logger:
        """Set up logging"""
        logger = logging.getLogger('IMUValidation')
        logger.setLevel(logging.INFO)

        # Create log directory
        log_dir = Path('logs/imu_validation')
        log_dir.mkdir(parents=True, exist_ok=True)

        # File handler
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_file = log_dir / f'imu_validation_{timestamp}.log'

        file_handler = logging.FileHandler(log_file, encoding='utf-8')
        file_handler.setLevel(logging.DEBUG)

        # Console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)

        # Formatter
        formatter = logging.Formatter(
            '%(asctime)s | %(levelname)-8s | %(name)s | %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )

        file_handler.setFormatter(formatter)
        console_handler.setFormatter(formatter)

        logger.addHandler(file_handler)
        logger.addHandler(console_handler)

        return logger

    def _load_config(self, config_file: str = None) -> Dict[str, Any]:
        """Load configuration file"""
        try:
            if config_file and Path(config_file).exists():
                with open(config_file, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    self.logger.info(f"Configuration file loaded: {config_file}")
            else:
                # Use default configuration file
                default_config = current_dir / 'validation_config.json'
                if default_config.exists():
                    with open(default_config, 'r', encoding='utf-8') as f:
                        config = json.load(f)
                        self.logger.info(f"Default configuration loaded: {default_config}")
                else:
                    self.logger.warning("Configuration file not found, using built-in default configuration")
                    config = self._get_default_config()

            return config

        except Exception as e:
            self.logger.error(f"Configuration file loading failed: {e}")
            return self._get_default_config()

    def _get_default_config(self) -> Dict[str, Any]:
        """Get default configuration"""
        return {
            "imu_config": {
                "sampling_rate_hz": 100,
                "test_duration_seconds": 30,
                "timeout_seconds": 10,
                "network_interface": "eth0"
            },
            "test_parameters": {
                "static_test": {
                    "duration_seconds": 60,
                    "stability_threshold": {
                        "accelerometer_std_max": 0.05,
                        "gyroscope_std_max": 0.1,
                        "quaternion_drift_max": 0.01
                    }
                },
                "dynamic_test": {
                    "duration_seconds": 120,
                    "response_tests": ["pitch_test", "roll_test", "yaw_test"],
                    "response_threshold_ms": 50
                }
            },
            "visualization": {
                "real_time_plots": True,
                "plot_update_interval_ms": 100,
                "max_plot_points": 500,
                "enable_3d_orientation": True
            },
            "quality_thresholds": {
                "accuracy": {
                    "gravity_error_max_percent": 2.0
                },
                "noise_levels": {
                    "accelerometer_noise_max": 0.02,
                    "gyroscope_noise_max": 0.01
                }
            }
        }

    def run_full_validation(self) -> Dict[str, Any]:
        """Run the complete IMU validation process"""
        self.logger.info("=" * 60)
        self.logger.info("Starting Unitree Go2 IMU complete validation process")
        self.logger.info("=" * 60)

        try:
            # 1. Initialize IMU system
            self.logger.info("\nPhase 1: Initialize IMU system")
            if not self._initialize_imu_system():
                self.test_results['overall_assessment']['status'] = 'INITIALIZATION_FAILED'
                return self.test_results

            # 2. Static stability test
            self.logger.info("\nPhase 2: Static stability test")
            static_results = self._run_static_stability_test()
            self.test_results['static_test'] = static_results

            # 3. Dynamic response test
            self.logger.info("\nPhase 3: Dynamic response test")
            dynamic_results = self._run_dynamic_validation()
            self.test_results['dynamic_test'] = dynamic_results

            # 4. Calibration validation analysis
            self.logger.info("\nPhase 4: Calibration validation analysis")
            calibration_results = self._run_calibration_validation()
            self.test_results['calibration_analysis'] = calibration_results

            # 5. Visualization validation
            self.logger.info("\nPhase 5: Visualization functionality validation")
            visualization_results = self._run_visualization_validation()
            self.test_results['visualization_test'] = visualization_results

            # 6. Generate comprehensive assessment
            self.logger.info("\nPhase 6: Generate comprehensive assessment report")
            overall_assessment = self._generate_overall_assessment()
            self.test_results['overall_assessment'] = overall_assessment

            # 7. Save test results
            self._save_test_results()

            self.logger.info("\n" + "=" * 60)
            self.logger.info("IMU validation process complete")
            self.logger.info(f"Overall status: {overall_assessment.get('status', 'UNKNOWN')}")
            self.logger.info("=" * 60)

            return self.test_results

        except Exception as e:
            self.logger.error(f"Validation process failed: {e}")
            self.test_results['overall_assessment']['status'] = 'VALIDATION_ERROR'
            self.test_results['overall_assessment']['error'] = str(e)
            return self.test_results

    def _initialize_imu_system(self) -> bool:
        """Initialize IMU system"""
        try:
            self.logger.info("Initializing IMU configuration...")

            # Create IMU configuration
            self.imu_config = IMUConfig(self.config)

            # Initialize IMU connection
            success = self.imu_config.initialize_imu()

            if not success:
                self.logger.error("IMU initialization failed")
                self.test_results['initialization']['status'] = 'FAILED'
                return False

            # Create data collector
            self.data_collector = IMUDataCollector(self.config, self.imu_config)

            # Create testers
            self.logger.info("Creating test components...")

            # Import tester classes
            from static_tester import IMUStaticTester
            from dynamic_tester import IMUDynamicTester
            from calibration_analyzer import IMUCalibrationAnalyzer
            from visualizer import IMUVisualizer

            # Create tester instances
            self.static_tester = IMUStaticTester(self.imu_config, self.data_collector, self.config)
            self.dynamic_tester = IMUDynamicTester(self.imu_config, self.data_collector, self.config)
            self.calibration_analyzer = IMUCalibrationAnalyzer(self.imu_config, self.data_collector, self.config)
            self.visualizer = IMUVisualizer(self.imu_config, self.data_collector, self.config)

            # Verify initialization state
            test_reading = self.imu_config.get_latest_reading()

            if test_reading:
                self.logger.info("IMU system initialization successful")
                self.logger.info(f"Current acceleration: {test_reading.accelerometer}")
                self.logger.info(f"Current gyroscope: {test_reading.gyroscope}")
                self.logger.info(f"Current attitude: {test_reading.quaternion}")

                self.test_results['initialization']['status'] = 'SUCCESS'
                self.test_results['initialization']['imu_specs'] = {
                    'sampling_rate_hz': self.imu_config.target_spec.sampling_rate_hz,
                    'accelerometer_range': self.imu_config.target_spec.accelerometer_range,
                    'gyroscope_range': self.imu_config.target_spec.gyroscope_range
                }

                return True
            else:
                self.logger.error("IMU data read test failed")
                self.test_results['initialization']['status'] = 'NO_DATA'
                return False

        except Exception as e:
            self.logger.error(f"IMU system initialization exception: {e}")
            self.test_results['initialization']['status'] = 'ERROR'
            self.test_results['initialization']['error'] = str(e)
            return False

    def _run_static_stability_test(self) -> Dict:
        """Run static stability test"""
        self.logger.info("Starting static stability test...")

        try:
            # Check if static tester is available
            if not self.static_tester:
                self.logger.error("Static tester not initialized")
                return {'status': 'FAIL', 'error': 'Static tester not initialized'}

            # Run static stability test - using the correct method name
            static_results = self.static_tester.run_static_stability_test()

            # Convert StaticTestResults to dictionary format
            result_dict = {
                'status': static_results.test_status,
                'test_duration': static_results.test_duration,
                'sample_count': static_results.sample_count,
                'valid_samples': static_results.valid_samples,
                'accelerometer_stability': static_results.accelerometer_stability,
                'gyroscope_stability': static_results.gyroscope_stability,
                'quaternion_stability': static_results.quaternion_stability,
                'gravity_accuracy': static_results.gravity_accuracy,
                'bias_analysis': static_results.bias_analysis,
                'noise_analysis': static_results.noise_analysis,
                'temperature_analysis': static_results.temperature_analysis,
                'pass_criteria': static_results.pass_criteria,
                'recommendations': static_results.recommendations
            }

            self.logger.info(f"Static test complete, status: {static_results.test_status}")
            return result_dict

        except Exception as e:
            self.logger.error(f"Static test failed: {e}")
            return {'status': 'ERROR', 'error': str(e)}

    def _run_dynamic_validation(self) -> Dict[str, Any]:
        """Run dynamic validation test"""
        try:
            self.logger.info("Starting dynamic response test...")

            # Run dynamic test
            dynamic_results = self.dynamic_tester.run_dynamic_response_test("comprehensive")

            # Convert results to dictionary format
            result_dict = {}

            for test_name, test_result in dynamic_results.items():
                result_dict[test_name] = {
                    'test_duration': test_result.test_duration,
                    'sample_count': test_result.sample_count,
                    'response_time_ms': test_result.response_time_ms,
                    'rise_time_ms': test_result.rise_time_ms,
                    'settling_time_ms': test_result.settling_time_ms,
                    'overshoot_percent': test_result.overshoot_percent,
                    'tracking_accuracy': test_result.tracking_accuracy,
                    'dynamic_range': test_result.dynamic_range,
                    'frequency_response': test_result.frequency_response,
                    'test_status': test_result.test_status,
                    'pass_criteria': test_result.pass_criteria,
                    'recommendations': test_result.recommendations
                }

            overall_status = dynamic_results.get('overall', type('', (), {'test_status': 'UNKNOWN'})).test_status
            self.logger.info(f"Dynamic test complete, overall status: {overall_status}")

            return result_dict

        except Exception as e:
            self.logger.error(f"Dynamic validation test failed: {e}")
            return {
                'overall': {
                    'test_status': 'ERROR',
                    'error': str(e),
                    'recommendations': ['Dynamic test execution failed, check system status']
                }
            }

    def _run_calibration_validation(self) -> Dict[str, Any]:
        """Run calibration validation analysis"""
        try:
            self.logger.info("Starting calibration validation analysis...")

            # Run calibration analysis
            calibration_results = self.calibration_analyzer.run_comprehensive_calibration_analysis()

            # Convert results to dictionary format
            result_dict = {
                'calibration_type': calibration_results.calibration_type,
                'sample_count': calibration_results.sample_count,
                'test_duration': calibration_results.test_duration,
                'accelerometer_calibration': calibration_results.accelerometer_calibration,
                'gravity_calibration': calibration_results.gravity_calibration,
                'accel_bias': calibration_results.accel_bias,
                'accel_scale_factor': calibration_results.accel_scale_factor,
                'gyroscope_calibration': calibration_results.gyroscope_calibration,
                'gyro_bias': calibration_results.gyro_bias,
                'gyro_noise_characteristics': calibration_results.gyro_noise_characteristics,
                'attitude_calibration': calibration_results.attitude_calibration,
                'quaternion_consistency': calibration_results.quaternion_consistency,
                'euler_accuracy': calibration_results.euler_accuracy,
                'temperature_compensation': calibration_results.temperature_compensation,
                'calibration_quality': calibration_results.calibration_quality,
                'test_status': calibration_results.test_status,
                'pass_criteria': calibration_results.pass_criteria,
                'recommendations': calibration_results.recommendations
            }

            self.logger.info(f"Calibration analysis complete, status: {calibration_results.test_status}")

            return result_dict

        except Exception as e:
            self.logger.error(f"Calibration validation analysis failed: {e}")
            return {
                'test_status': 'ERROR',
                'error': str(e),
                'recommendations': ['Calibration analysis execution failed, check system status']
            }

    def _run_visualization_validation(self) -> Dict[str, Any]:
        """Run visualization validation"""
        try:
            self.logger.info("Starting visualization functionality validation...")

            # Start visualization
            viz_success = self.visualizer.start_visualization("all")

            if not viz_success:
                return {
                    'test_status': 'FAIL',
                    'error': 'Visualization startup failed',
                    'recommendations': ['Check matplotlib dependencies and display environment']
                }

            # Short-term data collection for visualization test
            self.logger.info("Collecting visualization test data...")
            collect_success = self.data_collector.start_collection(20.0)  # 20 seconds

            if collect_success:
                # Wait for collection to complete
                time.sleep(25)

                # Stop collection
                self.data_collector.stop_collection()

                # Get visualization statistics
                viz_stats = self.visualizer.get_plot_statistics()

                # Save visualization images
                output_dir = f"output/imu_validation/{datetime.now().strftime('%Y%m%d_%H%M%S')}"
                self.visualizer.save_current_plots(output_dir)

                # Stop visualization
                self.visualizer.stop_visualization()

                result_dict = {
                    'test_status': 'PASS',
                    'visualization_started': True,
                    'data_points_plotted': viz_stats.get('data_points', 0),
                    'plot_statistics': viz_stats,
                    'output_directory': output_dir,
                    'recommendations': ['Visualization functionality is working normally']
                }

            else:
                result_dict = {
                    'test_status': 'FAIL',
                    'error': 'Visualization data collection failed',
                    'recommendations': ['Check data collector status']
                }

            self.logger.info(f"Visualization validation complete, status: {result_dict['test_status']}")

            return result_dict

        except Exception as e:
            self.logger.error(f"Visualization validation failed: {e}")
            return {
                'test_status': 'ERROR',
                'error': str(e),
                'recommendations': ['Visualization validation execution failed, check dependencies and environment']
            }

    def _generate_overall_assessment(self) -> Dict[str, Any]:
        """Generate comprehensive assessment"""
        try:
            # Collect status from each phase
            init_status = self.test_results['initialization'].get('status', 'UNKNOWN')
            static_status = self.test_results['static_test'].get('status', 'UNKNOWN')
            dynamic_status = self.test_results['dynamic_test'].get('overall', {}).get('test_status', 'UNKNOWN')
            calib_status = self.test_results['calibration_analysis'].get('test_status', 'UNKNOWN')
            viz_status = self.test_results['visualization_test'].get('test_status', 'UNKNOWN')

            all_statuses = [init_status, static_status, dynamic_status, calib_status, viz_status]

            # Determine overall status
            if init_status != 'SUCCESS':
                overall_status = 'INITIALIZATION_FAILED'
            elif 'ERROR' in all_statuses:
                overall_status = 'ERROR'
            elif 'FAIL' in all_statuses:
                overall_status = 'FAIL'
            elif 'WARNING' in all_statuses:
                overall_status = 'WARNING'
            elif all(status in ['PASS', 'SUCCESS'] for status in all_statuses):
                overall_status = 'PASS'
            else:
                overall_status = 'PARTIAL'

            # Collect all recommendations
            all_recommendations = []

            for test_key in ['static_test', 'calibration_analysis']:
                recommendations = self.test_results.get(test_key, {}).get('recommendations', [])
                all_recommendations.extend(recommendations)

            # Dynamic test recommendations
            dynamic_tests = self.test_results.get('dynamic_test', {})
            for test_name, test_data in dynamic_tests.items():
                if isinstance(test_data, dict) and 'recommendations' in test_data:
                    all_recommendations.extend(test_data['recommendations'])

            # Visualization recommendations
            viz_recommendations = self.test_results.get('visualization_test', {}).get('recommendations', [])
            all_recommendations.extend(viz_recommendations)

            # Deduplicate
            unique_recommendations = list(set(all_recommendations))

            # Generate summary
            test_summary = {
                'initialization': init_status,
                'static_stability': static_status,
                'dynamic_response': dynamic_status,
                'calibration_quality': calib_status,
                'visualization': viz_status
            }

            # Calculate pass rate
            passed_tests = sum(1 for status in all_statuses if status in ['PASS', 'SUCCESS'])
            pass_rate = (passed_tests / len(all_statuses)) * 100

            assessment = {
                'status': overall_status,
                'test_summary': test_summary,
                'pass_rate_percent': pass_rate,
                'total_recommendations': len(unique_recommendations),
                'critical_issues': [rec for rec in unique_recommendations if any(word in rec.lower() for word in ['failed', 'error', 'cannot', 'insufficient', 'too high', 'too low'])],
                'recommendations': unique_recommendations,
                'test_completion_time': datetime.now().isoformat(),
                'overall_conclusion': self._generate_conclusion(overall_status, pass_rate)
            }

            return assessment

        except Exception as e:
            self.logger.error(f"Comprehensive assessment generation failed: {e}")
            return {
                'status': 'ASSESSMENT_ERROR',
                'error': str(e),
                'test_completion_time': datetime.now().isoformat()
            }

    def _generate_conclusion(self, status: str, pass_rate: float) -> str:
        """Generate conclusion text"""
        if status == 'PASS':
            return f"IMU validation fully passed, pass rate {pass_rate:.1f}%. All sensor metrics meet requirements, ready for production use."
        elif status == 'WARNING':
            return f"IMU validation basically passed, pass rate {pass_rate:.1f}%. Minor issues exist, please review related recommendations."
        elif status == 'FAIL':
            return f"IMU validation failed, pass rate {pass_rate:.1f}%. Serious issues exist, need to be fixed before retesting."
        elif status == 'INITIALIZATION_FAILED':
            return "IMU initialization failed, unable to perform complete validation. Check hardware connection and drivers."
        else:
            return f"IMU validation partially completed, pass rate {pass_rate:.1f}%. Please review detailed results and recommendations."

    def _save_test_results(self):
        """Save test results"""
        try:
            # Create output directory
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            output_dir = Path(f'output/imu_validation/{timestamp}')
            output_dir.mkdir(parents=True, exist_ok=True)

            # Save complete test results
            results_file = output_dir / 'imu_validation_results.json'
            with open(results_file, 'w', encoding='utf-8') as f:
                json.dump(self.test_results, f, indent=2, ensure_ascii=False)

            # Save simplified report
            report_file = output_dir / 'imu_validation_report.txt'
            self._generate_text_report(report_file)

            self.logger.info(f"Test results saved to: {output_dir}")
            self.logger.info(f"Detailed results: {results_file}")
            self.logger.info(f"Simplified report: {report_file}")

        except Exception as e:
            self.logger.error(f"Failed to save test results: {e}")

    def _generate_text_report(self, report_file: Path):
        """Generate text report"""
        try:
            with open(report_file, 'w', encoding='utf-8') as f:
                f.write("=" * 80 + "\n")
                f.write("Unitree Go2 IMU Validation Report\n")
                f.write("=" * 80 + "\n\n")

                # Basic information
                test_info = self.test_results['test_info']
                f.write("Test Information:\n")
                f.write(f"  Test time: {test_info['start_time']}\n")
                f.write(f"  Robot model: {test_info['robot_model']}\n")
                f.write(f"  Test version: {test_info['test_version']}\n")
                f.write(f"  Operator: {test_info['test_operator']}\n\n")

                # Overall results
                overall = self.test_results['overall_assessment']
                f.write("Overall Assessment:\n")
                f.write(f"  Status: {overall.get('status', 'UNKNOWN')}\n")
                f.write(f"  Pass rate: {overall.get('pass_rate_percent', 0):.1f}%\n")
                f.write(f"  Conclusion: {overall.get('overall_conclusion', 'N/A')}\n\n")

                # Per-phase results
                f.write("Detailed Results:\n")
                test_summary = overall.get('test_summary', {})
                for test_name, status in test_summary.items():
                    f.write(f"  {test_name}: {status}\n")
                f.write("\n")

                # Recommendations
                recommendations = overall.get('recommendations', [])
                if recommendations:
                    f.write("Recommendations:\n")
                    for i, rec in enumerate(recommendations, 1):
                        f.write(f"  {i}. {rec}\n")

                f.write("\n" + "=" * 80 + "\n")

        except Exception as e:
            self.logger.error(f"Failed to generate text report: {e}")

def main():
    """Main function"""
    parser = argparse.ArgumentParser(description='Unitree Go2 IMU Validation Tool')
    parser.add_argument('--config', '-c', help='Configuration file path')
    parser.add_argument('--test-type', '-t',
                       choices=['full', 'static', 'dynamic', 'calibration', 'visualization'],
                       default='full', help='Test type')
    parser.add_argument('--output', '-o', help='Output directory')
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose output')

    args = parser.parse_args()

    try:
        # Create validation suite
        suite = IMUValidationSuite(args.config)

        if args.verbose:
            suite.logger.setLevel(logging.DEBUG)

        # Run validation
        if args.test_type == 'full':
            results = suite.run_full_validation()
        else:
            suite.logger.info(f"Running single test: {args.test_type}")
            # Single test logic can be added here
            results = suite.run_full_validation()  # For now, still run full validation

        # Output results summary
        overall_status = results.get('overall_assessment', {}).get('status', 'UNKNOWN')
        pass_rate = results.get('overall_assessment', {}).get('pass_rate_percent', 0)

        print("\n" + "=" * 60)
        print("IMU Validation Complete")
        print(f"Status: {overall_status}")
        print(f"Pass rate: {pass_rate:.1f}%")
        print("=" * 60)

        # Return appropriate exit code
        if overall_status in ['PASS', 'WARNING']:
            sys.exit(0)
        else:
            sys.exit(1)

    except KeyboardInterrupt:
        print("\nUser interrupted test")
        sys.exit(2)
    except Exception as e:
        print(f"\nValidation process failed: {e}")
        sys.exit(3)

if __name__ == "__main__":
    main()
