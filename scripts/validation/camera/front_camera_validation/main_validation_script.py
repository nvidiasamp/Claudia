#!/usr/bin/env python3
# scripts/validation/camera/front_camera_validation/main_validation_script.py
# Generated: 2025-06-27
# Purpose: Unitree Go2 front camera main validation script

import os
import sys
import time
import json
import logging
import argparse
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, List, Tuple

# Add current directory to Python path
current_dir = Path(__file__).parent
sys.path.append(str(current_dir))

from camera_config import CameraConfig
from performance_tester import PerformanceTester, PerformanceMetrics
from image_quality_analyzer import ImageQualityAnalyzer, ImageQualityMetrics

class FrontCameraValidator:
    """Front camera comprehensive validator"""

    def __init__(self, config_path: str = None, output_dir: str = None):
        """
        Initialize validator

        Args:
            config_path: Configuration file path
            output_dir: Output directory
        """
        # Set up logging
        self._setup_logging()
        self.logger = logging.getLogger(__name__)

        # Load configuration
        self.config = self._load_config(config_path)

        # Set up output directory
        self.output_dir = Path(output_dir) if output_dir else Path("logs/camera_validation")
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Create timestamped directory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = self.output_dir / f"validation_{timestamp}"
        self.session_dir.mkdir(parents=True, exist_ok=True)

        self.logger.info(f"Validation session directory: {self.session_dir}")

        # Validation components
        self.camera_config = None
        self.performance_tester = None
        self.image_quality_analyzer = None

        # Validation results
        self.validation_results = {}

    def _setup_logging(self):
        """Set up logging"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.StreamHandler(),
                logging.FileHandler('front_camera_validation.log')
            ]
        )

    def _load_config(self, config_path: str) -> Dict[str, Any]:
        """Load configuration file"""
        default_config_path = current_dir / "validation_config.json"
        config_file = Path(config_path) if config_path else default_config_path

        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                config = json.load(f)
                self.logger.info(f"Loaded configuration file: {config_file}")
                return config
        except Exception as e:
            self.logger.error(f"Configuration file loading failed: {e}")
            return self._get_default_config()

    def _get_default_config(self) -> Dict[str, Any]:
        """Get default configuration"""
        return {
            "camera_config": {
                "target_resolution": [1280, 720],
                "target_fps": 30,
                "timeout_seconds": 10
            },
            "validation_sequence": [
                "camera_initialization",
                "resolution_verification",
                "basic_performance_test",
                "image_quality_analysis",
                "stress_test",
                "report_generation"
            ]
        }

    def run_full_validation(self) -> Dict[str, Any]:
        """
        Run the full validation workflow

        Returns:
            Dict[str, Any]: Validation results
        """
        self.logger.info("=" * 60)
        self.logger.info("Starting Unitree Go2 front camera full validation")
        self.logger.info("=" * 60)

        start_time = time.time()

        try:
            # Get validation sequence
            sequence = self.config.get("validation_sequence", [])

            for step in sequence:
                self.logger.info(f"\n{'='*20} {step.upper()} {'='*20}")

                if step == "camera_initialization":
                    result = self._step_camera_initialization()
                elif step == "resolution_verification":
                    result = self._step_resolution_verification()
                elif step == "basic_performance_test":
                    result = self._step_basic_performance_test()
                elif step == "image_quality_analysis":
                    result = self._step_image_quality_analysis()
                elif step == "stress_test":
                    result = self._step_stress_test()
                elif step == "report_generation":
                    result = self._step_report_generation()
                else:
                    self.logger.warning(f"Unknown validation step: {step}")
                    result = {"status": "SKIPPED", "reason": "Unknown step"}

                self.validation_results[step] = result

                # If a critical step fails, stop validation
                if step in ["camera_initialization"] and result.get("status") == "FAIL":
                    self.logger.error(f"Critical step failed: {step}, stopping validation")
                    break

            # Calculate overall results
            end_time = time.time()
            self.validation_results["summary"] = self._generate_summary(start_time, end_time)

            # Save results
            self._save_validation_results()

            self.logger.info("=" * 60)
            self.logger.info("Front camera validation complete")
            self.logger.info("=" * 60)

            return self.validation_results

        except Exception as e:
            self.logger.error(f"Error occurred during validation: {e}")
            self.validation_results["error"] = str(e)
            return self.validation_results

        finally:
            # Clean up resources
            self._cleanup()

    def _step_camera_initialization(self) -> Dict[str, Any]:
        """Step 1: Camera initialization"""
        self.logger.info("Initializing camera configuration...")

        try:
            self.camera_config = CameraConfig(config_path=None)

            # Try to initialize camera
            if self.camera_config.initialize_camera("opencv"):
                self.logger.info("Camera initialization successful")

                # Get camera properties
                properties = self.camera_config.get_camera_properties()

                # Initialize other components
                self.performance_tester = PerformanceTester(self.camera_config, self.config)
                self.image_quality_analyzer = ImageQualityAnalyzer(self.camera_config, self.config)

                return {
                    "status": "PASS",
                    "camera_properties": properties,
                    "actual_spec": self.camera_config.actual_spec.__dict__ if self.camera_config.actual_spec else None
                }
            else:
                self.logger.error("Camera initialization failed")
                return {
                    "status": "FAIL",
                    "reason": "Camera initialization failed"
                }

        except Exception as e:
            self.logger.error(f"Camera initialization exception: {e}")
            return {
                "status": "FAIL",
                "reason": f"Exception during initialization: {str(e)}"
            }

    def _step_resolution_verification(self) -> Dict[str, Any]:
        """Step 2: Resolution verification"""
        self.logger.info("Verifying camera resolution...")

        if not self.camera_config or not self.camera_config.is_initialized:
            return {"status": "SKIP", "reason": "Camera not initialized"}

        try:
            # Capture test frame
            ret, frame = self.camera_config.capture_frame()

            if ret and frame is not None:
                actual_resolution = (frame.shape[1], frame.shape[0])  # (width, height)
                target_resolution = tuple(self.config.get("camera_config", {}).get("target_resolution", [1280, 720]))

                resolution_match = actual_resolution == target_resolution

                self.logger.info(f"Actual resolution: {actual_resolution}")
                self.logger.info(f"Target resolution: {target_resolution}")
                self.logger.info(f"Resolution match: {resolution_match}")

                return {
                    "status": "PASS" if resolution_match else "WARNING",
                    "actual_resolution": actual_resolution,
                    "target_resolution": target_resolution,
                    "resolution_match": resolution_match
                }
            else:
                return {
                    "status": "FAIL",
                    "reason": "Failed to capture frame for resolution verification"
                }

        except Exception as e:
            self.logger.error(f"Resolution verification exception: {e}")
            return {
                "status": "FAIL",
                "reason": f"Exception during resolution verification: {str(e)}"
            }

    def _step_basic_performance_test(self) -> Dict[str, Any]:
        """Step 3: Basic performance test"""
        self.logger.info("Running basic performance test...")

        if not self.performance_tester:
            return {"status": "SKIP", "reason": "Performance tester not initialized"}

        try:
            # Run basic performance test
            metrics = self.performance_tester.run_basic_performance_test(duration_seconds=30.0)

            # Evaluate performance
            evaluation = self.performance_tester.evaluate_performance(metrics)

            # Save performance data
            self._save_performance_data(metrics, "basic_performance")

            self.logger.info(f"Basic performance test complete - FPS: {metrics.fps_actual:.2f}, "
                           f"latency: {metrics.avg_latency_ms:.2f}ms")

            return {
                "status": evaluation["overall_status"],
                "metrics": self._serialize_metrics(metrics),
                "evaluation": evaluation
            }

        except Exception as e:
            self.logger.error(f"Basic performance test exception: {e}")
            return {
                "status": "FAIL",
                "reason": f"Exception during basic performance test: {str(e)}"
            }

    def _step_image_quality_analysis(self) -> Dict[str, Any]:
        """Step 4: Image quality analysis"""
        self.logger.info("Running image quality analysis...")

        if not self.image_quality_analyzer:
            return {"status": "SKIP", "reason": "Image quality analyzer not initialized"}

        try:
            # Run image quality analysis
            metrics = self.image_quality_analyzer.analyze_image_quality(sample_count=20)

            # Evaluate quality
            evaluation = self.image_quality_analyzer.evaluate_quality_thresholds(metrics)

            # Save quality data
            self._save_quality_data(metrics, "image_quality")

            self.logger.info(f"Image quality analysis complete - overall score: {metrics.overall_quality_score:.2f} "
                           f"({metrics.quality_grade})")

            return {
                "status": evaluation["overall_status"],
                "metrics": self._serialize_quality_metrics(metrics),
                "evaluation": evaluation
            }

        except Exception as e:
            self.logger.error(f"Image quality analysis exception: {e}")
            return {
                "status": "FAIL",
                "reason": f"Exception during image quality analysis: {str(e)}"
            }

    def _step_stress_test(self) -> Dict[str, Any]:
        """Step 5: Stress test"""
        self.logger.info("Running stress test...")

        if not self.performance_tester:
            return {"status": "SKIP", "reason": "Performance tester not initialized"}

        try:
            # Run stress test
            metrics = self.performance_tester.run_stress_test(duration_seconds=60.0)

            # Evaluate performance
            evaluation = self.performance_tester.evaluate_performance(metrics)

            # Save stress test data
            self._save_performance_data(metrics, "stress_test")

            self.logger.info(f"Stress test complete - FPS: {metrics.fps_actual:.2f}, "
                           f"frame drop rate: {metrics.frame_drop_rate:.2%}")

            return {
                "status": evaluation["overall_status"],
                "metrics": self._serialize_metrics(metrics),
                "evaluation": evaluation
            }

        except Exception as e:
            self.logger.error(f"Stress test exception: {e}")
            return {
                "status": "FAIL",
                "reason": f"Exception during stress test: {str(e)}"
            }

    def _step_report_generation(self) -> Dict[str, Any]:
        """Step 6: Report generation"""
        self.logger.info("Generating validation report...")

        try:
            # Generate HTML report
            report_path = self._generate_html_report()

            # Generate JSON report
            json_report_path = self._generate_json_report()

            self.logger.info(f"Report generation complete:")
            self.logger.info(f"  HTML report: {report_path}")
            self.logger.info(f"  JSON report: {json_report_path}")

            return {
                "status": "PASS",
                "html_report": str(report_path),
                "json_report": str(json_report_path)
            }

        except Exception as e:
            self.logger.error(f"Report generation exception: {e}")
            return {
                "status": "FAIL",
                "reason": f"Exception during report generation: {str(e)}"
            }

    def _generate_summary(self, start_time: float, end_time: float) -> Dict[str, Any]:
        """Generate validation summary"""
        duration = end_time - start_time

        # Count step statuses
        status_counts = {"PASS": 0, "FAIL": 0, "WARNING": 0, "SKIP": 0}
        for step, result in self.validation_results.items():
            if step != "summary" and isinstance(result, dict):
                status = result.get("status", "UNKNOWN")
                if status in status_counts:
                    status_counts[status] += 1

        # Determine overall status
        if status_counts["FAIL"] > 0:
            overall_status = "FAIL"
        elif status_counts["WARNING"] > 0:
            overall_status = "WARNING"
        else:
            overall_status = "PASS"

        return {
            "overall_status": overall_status,
            "duration_seconds": duration,
            "test_timestamp": datetime.now().isoformat(),
            "status_counts": status_counts,
            "session_directory": str(self.session_dir)
        }

    def _serialize_metrics(self, metrics: PerformanceMetrics) -> Dict[str, Any]:
        """Serialize performance metrics"""
        return {
            "fps_actual": metrics.fps_actual,
            "fps_target": metrics.fps_target,
            "avg_latency_ms": metrics.avg_latency_ms,
            "max_latency_ms": metrics.max_latency_ms,
            "min_latency_ms": metrics.min_latency_ms,
            "capture_success_rate": metrics.capture_success_rate,
            "frame_drop_rate": metrics.frame_drop_rate,
            "test_duration_seconds": metrics.test_duration_seconds,
            "total_frames_captured": metrics.total_frames_captured,
            "total_frames_attempted": metrics.total_frames_attempted
        }

    def _serialize_quality_metrics(self, metrics: ImageQualityMetrics) -> Dict[str, Any]:
        """Serialize quality metrics"""
        return {
            "resolution_actual": metrics.resolution_actual,
            "resolution_target": metrics.resolution_target,
            "resolution_match": metrics.resolution_match,
            "overall_quality_score": metrics.overall_quality_score,
            "quality_grade": metrics.quality_grade,
            "sharpness_score": metrics.sharpness_score,
            "color_accuracy_score": metrics.color_accuracy_score,
            "brightness_score": metrics.brightness_score,
            "contrast_score": metrics.contrast_score,
            "noise_level": metrics.noise_level,
            "snr_db": metrics.snr_db
        }

    def _save_performance_data(self, metrics: PerformanceMetrics, test_name: str):
        """Save performance data"""
        data_file = self.session_dir / f"{test_name}_metrics.json"
        with open(data_file, 'w', encoding='utf-8') as f:
            json.dump(self._serialize_metrics(metrics), f, indent=2, ensure_ascii=False)

    def _save_quality_data(self, metrics: ImageQualityMetrics, test_name: str):
        """Save quality data"""
        data_file = self.session_dir / f"{test_name}_metrics.json"
        with open(data_file, 'w', encoding='utf-8') as f:
            json.dump(self._serialize_quality_metrics(metrics), f, indent=2, ensure_ascii=False)

    def _save_validation_results(self):
        """Save validation results"""
        results_file = self.session_dir / "validation_results.json"
        with open(results_file, 'w', encoding='utf-8') as f:
            json.dump(self.validation_results, f, indent=2, ensure_ascii=False)

    def _generate_html_report(self) -> Path:
        """Generate HTML report"""
        report_path = self.session_dir / "validation_report.html"

        # Simplified HTML report template
        html_content = f"""
<!DOCTYPE html>
<html>
<head>
    <title>Unitree Go2 Front Camera Validation Report</title>
    <style>
        body {{ font-family: Arial, sans-serif; margin: 20px; }}
        .header {{ background-color: #f0f0f0; padding: 10px; margin-bottom: 20px; }}
        .section {{ margin-bottom: 20px; border: 1px solid #ddd; padding: 10px; }}
        .pass {{ color: green; }} .fail {{ color: red; }} .warning {{ color: orange; }}
        table {{ border-collapse: collapse; width: 100%; }}
        th, td {{ border: 1px solid #ddd; padding: 8px; text-align: left; }}
        th {{ background-color: #f2f2f2; }}
    </style>
</head>
<body>
    <div class="header">
        <h1>Unitree Go2 Front Camera Validation Report</h1>
        <p>Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</p>
        <p>Overall status: <span class="{self.validation_results.get('summary', {}).get('overall_status', 'unknown').lower()}">{self.validation_results.get('summary', {}).get('overall_status', 'UNKNOWN')}</span></p>
    </div>

    <div class="section">
        <h2>Validation Results Overview</h2>
        {self._generate_results_table()}
    </div>

    <div class="section">
        <h2>Detailed Test Results</h2>
        {self._generate_detailed_results()}
    </div>
</body>
</html>
        """

        with open(report_path, 'w', encoding='utf-8') as f:
            f.write(html_content)

        return report_path

    def _generate_json_report(self) -> Path:
        """Generate JSON report"""
        report_path = self.session_dir / "validation_report.json"

        # Create simplified JSON report
        report_data = {
            "meta": {
                "test_type": "front_camera_validation",
                "timestamp": datetime.now().isoformat(),
                "version": "1.0.0"
            },
            "results": self.validation_results
        }

        with open(report_path, 'w', encoding='utf-8') as f:
            json.dump(report_data, f, indent=2, ensure_ascii=False)

        return report_path

    def _generate_results_table(self) -> str:
        """Generate results table HTML"""
        html = "<table><tr><th>Test Step</th><th>Status</th><th>Notes</th></tr>"

        for step, result in self.validation_results.items():
            if step != "summary" and isinstance(result, dict):
                status = result.get("status", "UNKNOWN")
                reason = result.get("reason", "")
                html += f"<tr><td>{step}</td><td class='{status.lower()}'>{status}</td><td>{reason}</td></tr>"

        html += "</table>"
        return html

    def _generate_detailed_results(self) -> str:
        """Generate detailed results HTML"""
        html = ""
        for step, result in self.validation_results.items():
            if step not in ["summary"] and isinstance(result, dict):
                html += f"<h3>{step}</h3>"
                html += f"<pre>{json.dumps(result, indent=2, ensure_ascii=False)}</pre>"
        return html

    def _cleanup(self):
        """Clean up resources"""
        if self.camera_config:
            self.camera_config.release()
            self.logger.info("Camera resources released")

def main():
    """Main function"""
    parser = argparse.ArgumentParser(description="Unitree Go2 Front Camera Validation")
    parser.add_argument("--config", help="Configuration file path")
    parser.add_argument("--output", help="Output directory path")
    parser.add_argument("--verbose", action="store_true", help="Verbose output")

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # Create validator and run
    validator = FrontCameraValidator(args.config, args.output)
    results = validator.run_full_validation()

    # Print summary
    summary = results.get("summary", {})
    print(f"\nValidation complete!")
    print(f"Overall status: {summary.get('overall_status', 'UNKNOWN')}")
    print(f"Duration: {summary.get('duration_seconds', 0):.1f}s")
    print(f"Session directory: {summary.get('session_directory', 'N/A')}")

    # Set exit code based on results
    exit_code = 0 if summary.get('overall_status') == 'PASS' else 1
    sys.exit(exit_code)

if __name__ == "__main__":
    main()
