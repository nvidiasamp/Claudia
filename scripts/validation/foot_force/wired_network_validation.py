#!/usr/bin/env python3
"""
Foot force sensor validation script for wired network environment
Test plan optimized for Ethernet cable length constraints
Generated: 2025-06-27 16:58:15
"""

import sys
import time
import logging
from pathlib import Path

# Add project root to Python path
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from foot_force_validation.foot_force_config import FootForceConfig
from foot_force_validation.data_collector import FootForceDataCollector
from foot_force_validation.static_tester import StaticFootForceTester
from foot_force_validation.dynamic_tester import DynamicFootForceTester
from foot_force_validation.comprehensive_dashboard import ComprehensiveFootForceDashboard

class WiredNetworkValidator:
    """Foot force validator for wired network environment"""

    def __init__(self):
        """Initialize validator"""
        self.setup_logging()
        self.robot_ip = "192.168.123.161"  # Confirmed robot IP
        self.local_ip = "192.168.123.18"   # Local machine IP
        self.config = None
        self.results = {}

        self.logger.info(f"Wired network validator initialized")
        self.logger.info(f"Robot IP: {self.robot_ip}")
        self.logger.info(f"Local IP: {self.local_ip}")

    def setup_logging(self):
        """Set up logging"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(__name__)

    def test_connection(self):
        """Test robot connection"""
        self.logger.info("Testing robot connection...")

        try:
            self.config = FootForceConfig()
            # Try to establish connection
            connection_test = self.config.test_connection()
            if connection_test:
                self.logger.info("Robot connection successful")
                return True
            else:
                self.logger.warning("WARNING: Robot connection test failed, but configuration initialized")
                return False
        except Exception as e:
            self.logger.error(f"Connection test error: {e}")
            return False

    def run_stationary_tests(self):
        """Run static tests (suitable for wired network)"""
        self.logger.info("Starting static tests (suitable for wired network environment)...")

        try:
            # Static tests do not require movement, suitable for wired network
            static_tester = StaticFootForceTester(self.config)

            # Zero-load test (robot stationary)
            self.logger.info("Zero-load test - Keep robot stationary...")
            zero_load_result = static_tester.run_zero_load_test(duration=10.0)

            # Static standing test
            self.logger.info("Static standing test - Robot standing normally...")
            standing_result = static_tester.run_static_standing_test(duration=15.0)

            # Small amplitude center of gravity shift test (suitable for cable constraints)
            self.logger.info("Center of gravity shift test - Small amplitude movement...")
            weight_shift_result = static_tester.run_weight_shift_test(duration=20.0)

            self.results['static'] = {
                'zero_load': zero_load_result,
                'standing': standing_result,
                'weight_shift': weight_shift_result
            }

            self.logger.info("Static tests complete")
            return True

        except Exception as e:
            self.logger.error(f"Static tests failed: {e}")
            self.results['static'] = {'error': str(e)}
            return False

    def run_limited_dynamic_tests(self):
        """Run limited dynamic tests (considering cable length)"""
        self.logger.info("Starting limited dynamic tests (adapted for cable length constraints)...")

        try:
            dynamic_tester = DynamicFootForceTester(self.config)

            # Marching in place test (no movement required)
            self.logger.info("Marching in place test...")
            marching_result = self._run_marching_test(dynamic_tester)

            # Small amplitude sway test (within cable range)
            self.logger.info("Small amplitude sway test...")
            sway_result = self._run_sway_test(dynamic_tester)

            # Sit-stand test (vertical movement)
            self.logger.info("Sit-stand test...")
            sit_stand_result = self._run_sit_stand_test(dynamic_tester)

            self.results['dynamic'] = {
                'marching': marching_result,
                'sway': sway_result,
                'sit_stand': sit_stand_result
            }

            self.logger.info("Limited dynamic tests complete")
            return True

        except Exception as e:
            self.logger.error(f"Dynamic tests failed: {e}")
            self.results['dynamic'] = {'error': str(e)}
            return False

    def _run_marching_test(self, dynamic_tester):
        """Marching in place test"""
        print("\n" + "="*60)
        print("Marching In Place Test")
        print("Have the robot perform marching in place, test duration: 30 seconds")
        print("Advantage: No movement needed, cable is not restricted")
        print("="*60)

        input("Press Enter to start marching in place test...")

        # Run marching in place test
        return dynamic_tester.run_single_test("marching_in_place", duration=30.0)

    def _run_sway_test(self, dynamic_tester):
        """Small amplitude sway test"""
        print("\n" + "="*60)
        print("Small Amplitude Sway Test")
        print("Have the robot perform small left-right sway, test duration: 20 seconds")
        print("Movement range: Within cable length (recommended <1 meter)")
        print("="*60)

        input("Press Enter to start sway test...")

        return dynamic_tester.run_single_test("limited_sway", duration=20.0)

    def _run_sit_stand_test(self, dynamic_tester):
        """Sit-stand test"""
        print("\n" + "="*60)
        print("Sit-Stand Test")
        print("Have the robot perform sit-down and stand-up motions, test duration: 25 seconds")
        print("Advantage: Primarily vertical movement, minimal cable impact")
        print("="*60)

        input("Press Enter to start sit-stand test...")

        return dynamic_tester.run_single_test("sit_stand_cycle", duration=25.0)

    def generate_wired_report(self):
        """Generate wired network-specific report"""
        self.logger.info("Generating wired network environment validation report...")

        try:
            dashboard = ComprehensiveFootForceDashboard(self.config)

            # Generate wired network-specific report
            report = dashboard.generate_wired_network_report(
                static_results=self.results.get('static'),
                dynamic_results=self.results.get('dynamic'),
                network_config={
                    'robot_ip': self.robot_ip,
                    'local_ip': self.local_ip,
                    'connection_type': 'wired_ethernet',
                    'limitations': 'cable_length_restricted_movement'
                }
            )

            self.logger.info("Wired network report generation complete")
            return report

        except Exception as e:
            self.logger.error(f"Report generation failed: {e}")
            return None

    def run_complete_wired_validation(self):
        """Run complete wired network validation workflow"""
        print("\n" + "="*80)
        print("Unitree Go2 Wired Network Foot Force Sensor Validation")
        print("Test plan optimized for cable length constraints")
        print("="*80)

        # Stage 1: Connection test
        print("\nStage 1: Robot Connection Test")
        if not self.test_connection():
            print("Connection failed, attempting to continue with mock testing...")

        # Stage 2: Static tests (not restricted by cable)
        print("\nStage 2: Static Tests (suitable for wired network)")
        self.run_stationary_tests()

        # Stage 3: Limited dynamic tests
        print("\nStage 3: Limited Dynamic Tests (considering cable length)")
        self.run_limited_dynamic_tests()

        # Stage 4: Generate report
        print("\nStage 4: Generate Wired Network-Specific Report")
        report = self.generate_wired_report()

        # Summary
        print("\n" + "="*80)
        print("Wired Network Validation Complete Summary:")
        print(f"   Robot IP: {self.robot_ip}")
        print(f"   Local IP: {self.local_ip}")
        print("   Static tests: Fully suitable for wired network")
        print("   Dynamic tests: Optimized for cable constraints")
        print("   Dedicated report: Includes network environment analysis")
        print("="*80)

        return report

if __name__ == "__main__":
    validator = WiredNetworkValidator()
    validator.run_complete_wired_validation()
