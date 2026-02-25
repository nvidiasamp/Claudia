#!/usr/bin/env python3
# scripts/validation/foot_force/run_complete_validation.py
# Generated: 2025-06-26 19:00:00
# Purpose: Unitree Go2 foot force sensor complete ABCD validation workflow

import os
import sys
import time
import json
import logging
from pathlib import Path
from datetime import datetime
from typing import Dict, Any, Optional

# Add project path
project_root = Path(__file__).parent.parent.parent.parent
sys.path.append(str(project_root))
sys.path.append(str(project_root / "scripts" / "validation" / "foot_force" / "foot_force_validation"))

# Set environment variables
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

try:
    from foot_force_validation.foot_force_config import FootForceConfig
    from foot_force_validation.basic_test import main as run_basic_test
    from foot_force_validation.static_validation import main as run_static_validation
    from foot_force_validation.dynamic_tester import DynamicFootForceTester
    from foot_force_validation.comprehensive_dashboard import ComprehensiveFootForceDashboard
except ImportError as e:
    print(f"Import error: {e}")
    print("Please ensure you are running this script from the correct directory")
    sys.exit(1)

class CompleteFootForceValidation:
    """Complete foot force sensor validation workflow"""

    def __init__(self, output_dir: str = "output"):
        """Initialize complete validation workflow"""
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Set up logging
        self.setup_logging()

        # Load configuration
        config_path = Path(__file__).parent / "foot_force_validation" / "validation_config.json"
        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = json.load(f)

        # Initialize components
        self.foot_force_config = None
        self.test_results = {
            'phase_a': None,
            'phase_b': None,
            'phase_c': None,
            'phase_d': None
        }

        self.logger.info("Complete foot force validation workflow initialized")

    def setup_logging(self):
        """Set up logging system"""
        log_dir = self.output_dir / "logs"
        log_dir.mkdir(exist_ok=True)

        log_file = log_dir / f"complete_validation_{self.timestamp}.log"

        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_file, encoding='utf-8'),
                logging.StreamHandler()
            ]
        )

        self.logger = logging.getLogger(__name__)

    def run_complete_validation(self) -> Dict[str, Any]:
        """Run complete ABCD validation workflow"""
        print("\n" + "="*80)
        print("Unitree Go2 Foot Force Sensor Complete Validation Workflow")
        print(f"Start time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("="*80)

        overall_success = True

        try:
            # Phase A: Data reading framework validation
            print("\nPhase A: Data Reading Framework Validation")
            phase_a_success = self.run_phase_a()
            if not phase_a_success:
                overall_success = False
                print("Phase A failed, but continuing with subsequent tests")

            # Phase B: Static force distribution validation
            print("\nPhase B: Static Force Distribution Validation")
            phase_b_success = self.run_phase_b()
            if not phase_b_success:
                overall_success = False
                print("Phase B failed, but continuing with subsequent tests")

            # Phase C: Dynamic response testing
            print("\nPhase C: Dynamic Response Testing")
            phase_c_success = self.run_phase_c()
            if not phase_c_success:
                overall_success = False
                print("Phase C failed, but continuing with subsequent tests")

            # Phase D: Comprehensive visualization and documentation
            print("\nPhase D: Comprehensive Visualization and Documentation Generation")
            phase_d_success = self.run_phase_d()
            if not phase_d_success:
                overall_success = False
                print("Phase D failed")

            # Generate final report
            final_report = self.generate_final_report(overall_success)

            print("\n" + "="*80)
            if overall_success:
                print("Complete validation workflow executed successfully!")
            else:
                print("WARNING: Validation workflow complete, but some phases failed")
            print(f"Final report: {final_report}")
            print("="*80)

            return {
                'success': overall_success,
                'timestamp': self.timestamp,
                'test_results': self.test_results,
                'final_report': final_report
            }

        except Exception as e:
            self.logger.error(f"Complete validation workflow failed: {e}")
            print(f"Validation workflow terminated abnormally: {e}")
            return {
                'success': False,
                'error': str(e),
                'timestamp': self.timestamp
            }

    def run_phase_a(self) -> bool:
        """Run Phase A: Data reading framework validation"""
        try:
            print("  Initializing foot force sensor configuration...")

            # Initialize FootForceConfig
            self.foot_force_config = FootForceConfig(
                sampling_rate=self.config['foot_force_config']['sampling_rate_hz'],
                force_threshold=self.config['foot_force_config']['force_threshold'],
                max_force_per_foot=self.config['foot_force_config']['max_force_per_foot']
            )

            print("  Establishing robot connection...")
            if not self.foot_force_config.initialize_connection():
                print("  Robot connection failed")
                return False

            print("  Testing data reading capability...")
            # Brief data reading test
            test_start = time.time()
            test_duration = 5.0
            sample_count = 0

            while time.time() - test_start < test_duration:
                reading = self.foot_force_config.get_latest_reading()
                if reading:
                    sample_count += 1
                time.sleep(0.01)

            if sample_count > 0:
                sample_rate = sample_count / test_duration
                print(f"  Data reading successful, sampling rate: {sample_rate:.1f} Hz")

                self.test_results['phase_a'] = {
                    'success': True,
                    'sample_rate': sample_rate,
                    'duration': test_duration,
                    'samples': sample_count
                }
                return True
            else:
                print("  Failed to receive foot force data")
                return False

        except Exception as e:
            self.logger.error(f"Phase A execution failed: {e}")
            print(f"  Phase A execution error: {e}")
            return False

    def run_phase_b(self) -> bool:
        """Run Phase B: Static force distribution validation"""
        try:
            if not self.foot_force_config:
                print("  Foot force configuration not initialized")
                return False

            print("  Running static validation test...")

            # Run static validation (simplified version)
            from foot_force_validation.static_tester import StaticFootForceTester
            static_tester = StaticFootForceTester(self.config, self.foot_force_config)

            # Run quick static test
            static_results = static_tester.run_quick_static_test()

            if static_results and static_results.get('success', False):
                print(f"  Static validation complete, score: {static_results.get('final_score', 0):.1f}")
                self.test_results['phase_b'] = static_results
                return True
            else:
                print("  Static validation failed")
                self.test_results['phase_b'] = {'success': False, 'error': 'Static test execution failed'}
                return False

        except Exception as e:
            self.logger.error(f"Phase B execution failed: {e}")
            print(f"  Phase B execution error: {e}")
            return False

    def run_phase_c(self) -> bool:
        """Run Phase C: Dynamic response testing"""
        try:
            if not self.foot_force_config:
                print("  Foot force configuration not initialized")
                return False

            print("  Initializing dynamic tester...")
            dynamic_tester = DynamicFootForceTester(self.config, self.foot_force_config)

            print("  Running dynamic test suite...")
            print("\n  " + "-"*60)
            print("  WARNING: Please prepare for the following dynamic tests:")
            print("     1. Slow walking test (60 seconds)")
            print("     2. Normal walking test (45 seconds)")
            print("     3. Impact test (30 seconds)")
            print("  " + "-"*60)

            # Ask user if ready
            response = input("\n  Ready to start dynamic tests? (y/N): ").strip().lower()
            if response not in ['y', 'yes']:
                print("  User cancelled dynamic tests")
                self.test_results['phase_c'] = {'success': False, 'cancelled': True}
                return False

            # Run dynamic test suite
            dynamic_results = dynamic_tester.run_dynamic_test_suite()

            if dynamic_results:
                avg_score = sum(r.test_score for r in dynamic_results.values()) / len(dynamic_results)
                print(f"\n  Dynamic tests complete, average score: {avg_score:.1f}")

                # Save results
                results_file = dynamic_tester.save_dynamic_test_results(
                    dynamic_results, str(self.output_dir)
                )

                self.test_results['phase_c'] = {
                    'success': True,
                    'test_results': {name: {
                        'test_score': result.test_score,
                        'duration': result.duration,
                        'total_samples': result.total_samples
                    } for name, result in dynamic_results.items()},
                    'average_score': avg_score,
                    'results_file': results_file
                }
                return True
            else:
                print("  Dynamic tests failed")
                return False

        except Exception as e:
            self.logger.error(f"Phase C execution failed: {e}")
            print(f"  Phase C execution error: {e}")
            return False

    def run_phase_d(self) -> bool:
        """Run Phase D: Comprehensive visualization and documentation generation"""
        try:
            print("  Initializing comprehensive dashboard...")
            dashboard = ComprehensiveFootForceDashboard(self.config, str(self.output_dir))

            print("  Generating comprehensive validation report...")
            report_file = dashboard.generate_comprehensive_report(
                static_results=self.test_results['phase_b'],
                dynamic_results=self.test_results['phase_c'],
                foot_force_config=self.foot_force_config
            )

            if report_file:
                print(f"  Comprehensive report generation complete: {Path(report_file).name}")
                self.test_results['phase_d'] = {
                    'success': True,
                    'report_file': report_file
                }
                return True
            else:
                print("  Comprehensive report generation failed")
                return False

        except Exception as e:
            self.logger.error(f"Phase D execution failed: {e}")
            print(f"  Phase D execution error: {e}")
            return False

    def generate_final_report(self, overall_success: bool) -> str:
        """Generate final test report"""
        try:
            report_data = {
                'validation_id': f"COMPLETE_FFVR_{self.timestamp}",
                'timestamp': datetime.now().isoformat(),
                'overall_success': overall_success,
                'phases': {
                    'phase_a': {
                        'name': 'Data Reading Framework Validation',
                        'success': self.test_results['phase_a'] is not None and self.test_results['phase_a'].get('success', False),
                        'results': self.test_results['phase_a']
                    },
                    'phase_b': {
                        'name': 'Static Force Distribution Validation',
                        'success': self.test_results['phase_b'] is not None and self.test_results['phase_b'].get('success', False),
                        'results': self.test_results['phase_b']
                    },
                    'phase_c': {
                        'name': 'Dynamic Response Testing',
                        'success': self.test_results['phase_c'] is not None and self.test_results['phase_c'].get('success', False),
                        'results': self.test_results['phase_c']
                    },
                    'phase_d': {
                        'name': 'Comprehensive Visualization and Documentation',
                        'success': self.test_results['phase_d'] is not None and self.test_results['phase_d'].get('success', False),
                        'results': self.test_results['phase_d']
                    }
                },
                'config': self.config,
            }

            # Calculate summary information
            successful_phases = sum(1 for phase in report_data['phases'].values() if phase['success'])
            report_data['summary'] = {
                'total_phases': 4,
                'successful_phases': successful_phases,
                'completion_rate': successful_phases / 4 * 100
            }

            # Save final report
            report_file = self.output_dir / f"final_validation_report_{self.timestamp}.json"
            with open(report_file, 'w', encoding='utf-8') as f:
                json.dump(report_data, f, indent=2, ensure_ascii=False)

            return str(report_file)

        except Exception as e:
            self.logger.error(f"Failed to generate final report: {e}")
            return ""

    def cleanup(self):
        """Clean up resources"""
        if self.foot_force_config:
            try:
                self.foot_force_config.cleanup()
            except:
                pass


def main():
    """Main function"""
    print("Starting Unitree Go2 foot force sensor complete validation workflow")

    # Check environment
    print("Checking runtime environment...")

    # Check network connection
    import subprocess
    try:
        result = subprocess.run(['ping', '-c', '1', '192.168.123.161'],
                              capture_output=True, timeout=5)
        if result.returncode != 0:
            print("WARNING: Cannot ping robot IP 192.168.123.161")
            response = input("Continue with test execution? (y/N): ").strip().lower()
            if response not in ['y', 'yes']:
                print("User cancelled test")
                return
    except:
        print("WARNING: Network check failed, continuing with test execution")

    # Create output directory
    output_dir = Path("scripts/validation/foot_force/foot_force_validation/output")

    # Run complete validation
    validator = CompleteFootForceValidation(str(output_dir))

    try:
        results = validator.run_complete_validation()

        print(f"\nValidation results summary:")
        print(f"   Status: {'SUCCESS' if results['success'] else 'FAILED'}")
        print(f"   Timestamp: {results['timestamp']}")

        if 'final_report' in results:
            print(f"   Final report: {results['final_report']}")

    except KeyboardInterrupt:
        print("\nUser interrupted test")
    except Exception as e:
        print(f"\nTest execution error: {e}")
    finally:
        validator.cleanup()


if __name__ == "__main__":
    main()
