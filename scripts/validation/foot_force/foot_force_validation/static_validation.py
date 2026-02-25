#!/usr/bin/env python3
# scripts/validation/foot_force/foot_force_validation/static_validation.py
# Generated: 2025-06-27 14:27:00 CST
# Purpose: Unitree Go2 foot force sensor static validation main script

import os
import sys
import time
import json
import logging
import argparse
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Any, Optional

# Add module path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from foot_force_config import FootForceConfig
from data_collector import FootForceDataCollector
from static_tester import StaticForceTester, StaticTestResult
from visualizer import FootForceVisualizer
from analyzer import FootForceAnalyzer

def setup_logging(log_level: str = 'INFO', log_file: Optional[str] = None) -> None:
    """Set up logging system"""

    # Create log directory
    log_dir = Path(__file__).parent / 'logs'
    log_dir.mkdir(exist_ok=True)

    # Configure log format
    log_format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'

    # Set log level
    level = getattr(logging, log_level.upper(), logging.INFO)

    # Configure root logger
    handlers = [logging.StreamHandler(sys.stdout)]

    if log_file:
        handlers.append(logging.FileHandler(log_file))
    else:
        # Default log file
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        default_log_file = log_dir / f'static_validation_{timestamp}.log'
        handlers.append(logging.FileHandler(default_log_file))

    logging.basicConfig(
        level=level,
        format=log_format,
        handlers=handlers
    )

def load_configuration(config_path: str) -> Dict[str, Any]:
    """Load configuration file"""

    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = json.load(f)

        logging.info(f"Configuration file loaded successfully: {config_path}")
        return config

    except Exception as e:
        logging.error(f"Failed to load configuration file: {e}")
        raise

def validate_environment() -> bool:
    """Validate runtime environment"""

    logger = logging.getLogger(__name__)

    try:
        # Check required module imports
        import numpy as np
        import matplotlib.pyplot as plt
        import scipy

        logger.info("Required modules imported successfully")

        # Check output directory
        output_dir = Path(__file__).parent / 'output'
        output_dir.mkdir(exist_ok=True)

        # Check log directory
        log_dir = Path(__file__).parent / 'logs'
        log_dir.mkdir(exist_ok=True)

        logger.info("Directory structure verified successfully")

        return True

    except Exception as e:
        logger.error(f"Environment validation failed: {e}")
        return False

def run_zero_load_validation(tester: StaticForceTester, config: Dict[str, Any]) -> StaticTestResult:
    """Run zero load validation"""

    logger = logging.getLogger(__name__)
    logger.info("=" * 60)
    logger.info("Phase B.1: Zero Load Validation")
    logger.info("=" * 60)

    # Get test parameters
    static_config = config.get('static_validation', {})
    zero_load_duration = static_config.get('zero_load_test_duration', 30.0)

    logger.info(f"Starting zero load test, duration: {zero_load_duration}s")
    logger.info("WARNING: Ensure the robot is suspended in air with feet not touching any surface!")

    # Countdown
    for i in range(5, 0, -1):
        logger.info(f"Test will begin in {i} seconds...")
        time.sleep(1.0)

    # Execute zero load test
    result = tester.run_zero_load_test(zero_load_duration)

    # Display results
    logger.info(f"Zero load test complete")
    logger.info(f"Score: {result.score:.1f}/100")
    logger.info(f"Status: {result.status}")

    if result.recommendations:
        logger.info("Recommendations:")
        for rec in result.recommendations:
            logger.info(f"  - {rec}")

    return result

def run_static_standing_validation(tester: StaticForceTester, config: Dict[str, Any]) -> StaticTestResult:
    """Run static standing validation"""

    logger = logging.getLogger(__name__)
    logger.info("=" * 60)
    logger.info("Phase B.2: Static Standing Validation")
    logger.info("=" * 60)

    # Get test parameters
    static_config = config.get('static_validation', {})
    standing_duration = static_config.get('static_standing_duration', 60.0)
    expected_weight = static_config.get('expected_total_force', 150.0)

    logger.info(f"Starting static standing test, duration: {standing_duration}s")
    logger.info(f"Expected total weight: {expected_weight}N")
    logger.info("WARNING: Ensure the robot is in normal standing position with all four feet stably contacting the ground!")

    # Countdown
    for i in range(5, 0, -1):
        logger.info(f"Test will begin in {i} seconds...")
        time.sleep(1.0)

    # Execute static standing test
    result = tester.run_static_standing_test(standing_duration)

    # Display results
    logger.info(f"Static standing test complete")
    logger.info(f"Score: {result.score:.1f}/100")
    logger.info(f"Status: {result.status}")

    if result.measurements:
        total_weight = result.measurements.get('total_weight', 0)
        weight_dist = result.measurements.get('weight_distribution', {})

        logger.info(f"Measured total weight: {total_weight:.1f}N")
        logger.info("Weight distribution:")
        for foot, percentage in weight_dist.items():
            logger.info(f"  {foot}: {percentage:.1f}%")

    if result.recommendations:
        logger.info("Recommendations:")
        for rec in result.recommendations:
            logger.info(f"  - {rec}")

    return result

def run_zero_drift_analysis(tester: StaticForceTester, config: Dict[str, Any]):
    """Run zero drift analysis"""

    logger = logging.getLogger(__name__)
    logger.info("=" * 60)
    logger.info("Phase B.3: Zero Drift Analysis")
    logger.info("=" * 60)

    # Get test parameters
    static_config = config.get('static_validation', {})
    drift_duration = static_config.get('zero_drift_duration', 300.0)

    logger.info(f"Starting zero drift analysis, duration: {drift_duration}s")
    logger.info("WARNING: Keep the robot suspended in air to analyze sensor zero point stability!")

    # Countdown
    for i in range(5, 0, -1):
        logger.info(f"Analysis will begin in {i} seconds...")
        time.sleep(1.0)

    # Execute zero drift analysis
    drift_result = tester.run_zero_drift_analysis(drift_duration)

    # Display results
    logger.info(f"Zero drift analysis complete")
    logger.info(f"Maximum drift: {drift_result.max_drift:.3f}N")
    logger.info(f"Drift stability score: {drift_result.drift_stability:.1f}/100")

    if drift_result.drift_values:
        logger.info("Drift status for each foot:")
        for foot, drift in drift_result.drift_values.items():
            drift_magnitude = (drift[0]**2 + drift[1]**2 + drift[2]**2)**0.5
            logger.info(f"  {foot}: {drift_magnitude:.3f}N")

    return drift_result

def run_comprehensive_analysis(analyzer: FootForceAnalyzer, data_collector: FootForceDataCollector,
                             output_dir: Path) -> Dict[str, Any]:
    """Run comprehensive data analysis"""

    logger = logging.getLogger(__name__)
    logger.info("=" * 60)
    logger.info("Phase B.4: Comprehensive Data Analysis")
    logger.info("=" * 60)

    # Get collected data
    collected_data = data_collector.get_data()

    if len(collected_data) < 10:
        logger.error("Insufficient data for comprehensive analysis")
        return {}

    logger.info(f"Analyzing data points: {len(collected_data)}")

    # Execute comprehensive analysis
    analysis_report = analyzer.generate_comprehensive_report(collected_data)

    # Save analysis report
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    report_file = output_dir / f'comprehensive_analysis_{timestamp}.json'

    if analyzer.save_analysis_report(analysis_report, str(report_file)):
        logger.info(f"Comprehensive analysis report saved: {report_file}")

    # Export CSV data
    csv_file = output_dir / f'static_validation_data_{timestamp}.csv'
    if analyzer.export_to_csv(collected_data, str(csv_file)):
        logger.info(f"Raw data exported: {csv_file}")

    # Display key results
    if 'overall_assessment' in analysis_report:
        assessment = analysis_report['overall_assessment']
        logger.info("Comprehensive assessment results:")
        logger.info(f"  Data quality score: {assessment.get('data_quality_score', 0):.1f}/100")
        logger.info(f"  Sensor consistency: {assessment.get('sensor_consistency_score', 0):.1f}/100")
        logger.info(f"  Stability score: {assessment.get('stability_score', 0):.1f}/100")
        logger.info(f"  Overall score: {assessment.get('overall_score', 0):.1f}/100")

        if assessment.get('key_findings'):
            logger.info("Key findings:")
            for finding in assessment['key_findings']:
                logger.info(f"  - {finding}")

    return analysis_report

def generate_visualizations(visualizer: FootForceVisualizer, data_collector: FootForceDataCollector,
                          test_results: List[StaticTestResult], output_dir: Path) -> bool:
    """Generate visualization charts"""

    logger = logging.getLogger(__name__)
    logger.info("=" * 60)
    logger.info("Phase B.5: Generate Visualization Charts")
    logger.info("=" * 60)

    try:
        # Get data
        collected_data = data_collector.get_data()

        if len(collected_data) < 10:
            logger.error("Insufficient data for visualization chart generation")
            return False

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        # 1. Static analysis chart
        static_plot_file = output_dir / f'static_analysis_{timestamp}.png'
        if visualizer.plot_static_analysis(collected_data, str(static_plot_file)):
            logger.info(f"Static analysis chart generated: {static_plot_file}")

        # 2. 3D force distribution chart
        force_3d_file = output_dir / f'force_distribution_3d_{timestamp}.png'
        if visualizer.plot_force_distribution_3d(collected_data, str(force_3d_file)):
            logger.info(f"3D force distribution chart generated: {force_3d_file}")

        # 3. Test result summary dashboard
        dashboard_file = output_dir / f'validation_dashboard_{timestamp}.png'
        if visualizer.create_summary_dashboard(test_results, str(dashboard_file)):
            logger.info(f"Validation summary dashboard generated: {dashboard_file}")

        return True

    except Exception as e:
        logger.error(f"Failed to generate visualization charts: {e}")
        return False

def generate_final_report(test_results: List[StaticTestResult], analysis_report: Dict[str, Any],
                         output_dir: Path) -> bool:
    """Generate final validation report"""

    logger = logging.getLogger(__name__)
    logger.info("=" * 60)
    logger.info("Generating Final Validation Report")
    logger.info("=" * 60)

    try:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        # Consolidate all results
        final_report = {
            'validation_timestamp': datetime.now().isoformat(),
            'validation_type': 'static_validation',
            'test_summary': {
                'total_tests': len(test_results),
                'passed_tests': sum(1 for r in test_results if r.status == 'PASS'),
                'warning_tests': sum(1 for r in test_results if r.status == 'WARNING'),
                'failed_tests': sum(1 for r in test_results if r.status == 'FAIL'),
                'average_score': sum(r.score for r in test_results) / len(test_results) if test_results else 0
            },
            'test_results': [r.__dict__ for r in test_results],
            'comprehensive_analysis': analysis_report,
            'final_assessment': {}
        }

        # Generate final assessment
        avg_score = final_report['test_summary']['average_score']
        pass_rate = final_report['test_summary']['passed_tests'] / len(test_results) * 100 if test_results else 0

        if avg_score >= 85 and pass_rate >= 80:
            final_status = "PASS"
            status_desc = "Sensor static validation passed, ready for production use"
        elif avg_score >= 70 and pass_rate >= 60:
            final_status = "WARNING"
            status_desc = "Sensor static validation marginally passed, further optimization recommended"
        else:
            final_status = "FAIL"
            status_desc = "Sensor static validation failed, recalibration or repair needed"

        final_report['final_assessment'] = {
            'status': final_status,
            'description': status_desc,
            'overall_score': avg_score,
            'pass_rate': pass_rate,
            'recommendations': []
        }

        # Collect all recommendations
        all_recommendations = []
        for result in test_results:
            all_recommendations.extend(result.recommendations)

        # Deduplicate and add to final report
        unique_recommendations = list(set(all_recommendations))
        final_report['final_assessment']['recommendations'] = unique_recommendations

        # Save final report
        report_file = output_dir / f'static_validation_final_report_{timestamp}.json'
        with open(report_file, 'w', encoding='utf-8') as f:
            json.dump(final_report, f, indent=2, ensure_ascii=False)

        logger.info(f"Final validation report saved: {report_file}")

        # Display final results
        logger.info("=" * 60)
        logger.info("Static Validation Final Results")
        logger.info("=" * 60)
        logger.info(f"Status: {final_status}")
        logger.info(f"Description: {status_desc}")
        logger.info(f"Overall score: {avg_score:.1f}/100")
        logger.info(f"Pass rate: {pass_rate:.1f}%")
        logger.info(f"Test items: {len(test_results)}")

        if unique_recommendations:
            logger.info("Main recommendations:")
            for rec in unique_recommendations[:5]:  # Show top 5 recommendations
                logger.info(f"  - {rec}")

        return True

    except Exception as e:
        logger.error(f"Failed to generate final validation report: {e}")
        return False

def main():
    """Main function"""

    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Unitree Go2 Foot Force Sensor Static Validation')
    parser.add_argument('--config', default='validation_config.json', help='Configuration file path')
    parser.add_argument('--log-level', default='INFO', choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
                       help='Log level')
    parser.add_argument('--log-file', help='Log file path')
    parser.add_argument('--skip-visualization', action='store_true', help='Skip visualization generation')
    parser.add_argument('--test-mode', action='store_true', help='Test mode (shorter test durations)')

    args = parser.parse_args()

    # Set up logging
    setup_logging(args.log_level, args.log_file)
    logger = logging.getLogger(__name__)

    logger.info("=" * 80)
    logger.info("Unitree Go2 Foot Force Sensor Static Validation System")
    logger.info("=" * 80)
    logger.info(f"Start time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

    try:
        # 1. Validate environment
        if not validate_environment():
            logger.error("Environment validation failed, exiting program")
            return 1

        # 2. Load configuration
        config = load_configuration(args.config)

        # Test mode adjustments
        if args.test_mode:
            logger.info("Test mode: using shorter test durations")
            config['static_validation']['zero_load_test_duration'] = 10.0
            config['static_validation']['static_standing_duration'] = 20.0
            config['static_validation']['zero_drift_duration'] = 60.0

        # 3. Initialize components
        logger.info("Initializing system components...")

        # Foot force configuration
        foot_force_config = FootForceConfig(config)
        logger.info("Foot force configuration initialization complete")

        # Data collector
        data_collector = FootForceDataCollector(config, foot_force_config)
        logger.info("Data collector initialization complete")

        # Static tester
        static_tester = StaticForceTester(config, foot_force_config)
        logger.info("Static tester initialization complete")

        # Visualizer
        visualizer = FootForceVisualizer(config)
        logger.info("Visualizer initialization complete")

        # Data analyzer
        analyzer = FootForceAnalyzer(config)
        logger.info("Data analyzer initialization complete")

        # Output directory
        output_dir = Path(__file__).parent / 'output'
        output_dir.mkdir(exist_ok=True)

        logger.info("All components initialized, starting static validation...")

        # 4. Execute validation tests
        test_results = []

        # 4.1 Zero load validation
        zero_load_result = run_zero_load_validation(static_tester, config)
        test_results.append(zero_load_result)

        # 4.2 Static standing validation
        standing_result = run_static_standing_validation(static_tester, config)
        test_results.append(standing_result)

        # 4.3 Zero drift analysis
        drift_result = run_zero_drift_analysis(static_tester, config)

        # 4.4 Comprehensive data analysis
        analysis_report = run_comprehensive_analysis(analyzer, data_collector, output_dir)

        # 5. Generate visualization charts
        if not args.skip_visualization:
            generate_visualizations(visualizer, data_collector, test_results, output_dir)
        else:
            logger.info("Skipping visualization generation")

        # 6. Generate final report
        generate_final_report(test_results, analysis_report, output_dir)

        logger.info("=" * 80)
        logger.info("Static validation complete!")
        logger.info(f"End time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        logger.info(f"Result files saved in: {output_dir}")
        logger.info("=" * 80)

        return 0

    except KeyboardInterrupt:
        logger.warning("User interrupted operation")
        return 130

    except Exception as e:
        logger.error(f"Static validation execution failed: {e}")
        return 1

if __name__ == '__main__':
    exit(main())
