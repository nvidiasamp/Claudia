#!/usr/bin/env python3
# scripts/validation/foot_force/foot_force_validation/basic_test.py
# Generated: 2025-06-27 14:09:30 CST
# Purpose: Foot force sensor data reading framework basic test

import os
import sys
import time
import json
import logging
import argparse
from pathlib import Path
from typing import Dict, Any
from datetime import datetime

# Add module path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from foot_force_config import FootForceConfig, FootForceReading, create_default_config, format_force_reading
from data_collector import FootForceDataCollector, FootForceData

class FootForceBasicTest:
    """Foot force sensor basic test class"""

    def __init__(self, config_file: str = "validation_config.json"):
        """
        Initialize test

        Args:
            config_file: Configuration file path
        """
        self.setup_logging()
        self.logger = logging.getLogger(__name__)

        # Load configuration
        self.config = self.load_config(config_file)

        # Initialize components
        self.foot_force_config: FootForceConfig = None
        self.data_collector: FootForceDataCollector = None

        # Output directory
        self.output_dir = Path("output") / datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.logger.info("Foot force sensor basic test initialization complete")

    def setup_logging(self):
        """Set up logging"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('logs/foot_force_basic_test.log'),
                logging.StreamHandler(sys.stdout)
            ]
        )

    def load_config(self, config_file: str) -> Dict[str, Any]:
        """Load configuration file"""
        try:
            config_path = Path(__file__).parent / config_file
            with open(config_path, 'r', encoding='utf-8') as f:
                config = json.load(f)
            self.logger.info(f"Configuration file loaded successfully: {config_path}")
            return config
        except Exception as e:
            self.logger.error(f"Configuration file loading failed: {e}")
            # Return default configuration
            return self.get_default_config()

    def get_default_config(self) -> Dict[str, Any]:
        """Get default configuration"""
        return {
            "general": {
                "network_interface": "eth0",
                "domain_id": 0
            },
            "foot_force_config": {
                "sampling_rate_hz": 500,
                "force_threshold": 5.0,
                "max_force_per_foot": 200.0
            },
            "data_collection": {
                "default_duration": 10.0,
                "quality_threshold": 0.75
            }
        }

    def initialize_foot_force_system(self) -> bool:
        """Initialize foot force sensor system"""
        try:
            self.logger.info("Initializing foot force sensor system...")

            # Create FootForceConfig
            network_interface = self.config["general"]["network_interface"]
            self.foot_force_config = create_default_config(network_interface)

            # Initialize connection
            domain_id = self.config["general"]["domain_id"]
            if not self.foot_force_config.initialize_connection(domain_id):
                self.logger.error("Foot force sensor connection initialization failed")
                return False

            # Create data collector
            self.data_collector = FootForceDataCollector(
                config=self.config,
                foot_force_config=self.foot_force_config
            )

            self.logger.info("Foot force sensor system initialization successful")
            return True

        except Exception as e:
            self.logger.error(f"Failed to initialize foot force sensor system: {e}")
            return False

    def test_connection(self) -> bool:
        """Test connection"""
        self.logger.info("Testing foot force sensor connection...")

        try:
            # Attempt to get one data reading
            reading = self.foot_force_config.get_latest_reading()
            if reading:
                self.logger.info("Connection test successful, data received:")
                self.logger.info(format_force_reading(reading))
                return True
            else:
                self.logger.warning("Connection test received no data")
                return False

        except Exception as e:
            self.logger.error(f"Connection test failed: {e}")
            return False

    def test_data_collection(self, duration: float = 10.0) -> bool:
        """Test data collection"""
        self.logger.info(f"Starting data collection test, duration: {duration}s")

        try:
            # Add real-time data display callback
            self.data_collector.add_data_callback(self.real_time_data_callback)

            # Start collection
            if not self.data_collector.start_collection(duration):
                self.logger.error("Data collection failed to start")
                return False

            self.logger.info("Data collection started, waiting for completion...")

            # Real-time monitoring
            start_time = time.time()
            last_report_time = start_time

            while self.data_collector.is_collecting:
                current_time = time.time()

                # Display real-time metrics every 5 seconds
                if current_time - last_report_time >= 5.0:
                    metrics = self.data_collector.get_real_time_metrics()
                    self.logger.info(f"Real-time metrics: FPS={metrics['current_fps']:.1f}, "
                                   f"samples={metrics['total_samples']}, "
                                   f"data_quality={metrics['data_quality']:.2f}, "
                                   f"contact_state={metrics['contact_summary']}")
                    last_report_time = current_time

                time.sleep(1.0)

            # Get final metrics
            final_metrics = self.data_collector.stop_collection()
            self.logger.info("Data collection complete")
            self.print_collection_metrics(final_metrics)

            return True

        except Exception as e:
            self.logger.error(f"Data collection test failed: {e}")
            return False

    def real_time_data_callback(self, data: FootForceData):
        """Real-time data callback (shows detailed info only in debug mode)"""
        # Simplified real-time display to avoid excessive logging
        if len(self.data_collector.raw_data) % 100 == 0:  # Display every 100 samples
            self.logger.debug(f"Sample #{len(self.data_collector.raw_data)}: "
                            f"total_force={data.total_force:.1f}N, "
                            f"stability={data.stability_index:.2f}, "
                            f"balance={data.force_balance:.2f}")

    def print_collection_metrics(self, metrics):
        """Print collection metrics"""
        self.logger.info("=== Data Collection Metrics ===")
        self.logger.info(f"Collection duration: {metrics.collection_duration:.2f}s")
        self.logger.info(f"Total samples: {metrics.total_samples}")
        self.logger.info(f"Average sampling rate: {metrics.avg_sampling_rate:.1f} Hz")
        self.logger.info(f"Data quality score: {metrics.data_quality_score:.2f}")

        if metrics.force_stats:
            self.logger.info("Foot force statistics:")
            for foot_label, stats in metrics.force_stats.items():
                self.logger.info(f"  {foot_label}: mean Fz={stats['mean_fz']:.2f}N, "
                               f"contact_events={stats['contact_events']}")

        if metrics.total_force_stats:
            self.logger.info(f"Total force statistics: mean={metrics.total_force_stats['mean']:.2f}N, "
                           f"range=[{metrics.total_force_stats['min']:.1f}, "
                           f"{metrics.total_force_stats['max']:.1f}]N")

    def save_test_results(self) -> bool:
        """Save test results"""
        try:
            self.logger.info("Saving test results...")

            # Save JSON format data
            json_path = self.output_dir / "foot_force_test_data.json"
            if self.data_collector.save_data(str(json_path)):
                self.logger.info(f"JSON data saved: {json_path}")

            # Save CSV format data
            csv_path = self.output_dir / "foot_force_test_data.csv"
            if self.data_collector.save_data_csv(str(csv_path)):
                self.logger.info(f"CSV data saved: {csv_path}")

            # Save statistics
            stats_path = self.output_dir / "test_statistics.json"
            stats = self.data_collector.get_statistics()
            with open(stats_path, 'w', encoding='utf-8') as f:
                json.dump(stats, f, indent=2, ensure_ascii=False)
            self.logger.info(f"Statistics saved: {stats_path}")

            # Save test configuration
            config_path = self.output_dir / "test_config.json"
            with open(config_path, 'w', encoding='utf-8') as f:
                json.dump(self.config, f, indent=2, ensure_ascii=False)

            return True

        except Exception as e:
            self.logger.error(f"Failed to save test results: {e}")
            return False

    def test_calibration(self) -> bool:
        """Test calibration functionality"""
        self.logger.info("Starting calibration test...")

        try:
            # Perform zero point calibration
            calibration_duration = self.config.get("static_validation", {}).get("calibration_duration", 5.0)

            if self.foot_force_config.zero_calibration(calibration_duration):
                self.logger.info("Zero point calibration successful")

                # Display calibration results
                foot_info = self.foot_force_config.get_foot_info()
                self.logger.info("Calibration offset values:")
                for foot_label, offset in foot_info['calibration_offset'].items():
                    self.logger.info(f"  {foot_label}: {offset}")

                return True
            else:
                self.logger.error("Zero point calibration failed")
                return False

        except Exception as e:
            self.logger.error(f"Calibration test failed: {e}")
            return False

    def run_basic_test(self, test_duration: float = None) -> bool:
        """Run basic test"""
        self.logger.info("=== Starting Foot Force Sensor Basic Test ===")

        try:
            # 1. Initialize system
            if not self.initialize_foot_force_system():
                return False

            # 2. Test connection
            if not self.test_connection():
                return False

            # 3. Test calibration (optional)
            calibration_test = input("Execute calibration test? (y/n): ").lower().strip() == 'y'
            if calibration_test:
                self.test_calibration()

            # 4. Data collection test
            if test_duration is None:
                test_duration = self.config["data_collection"]["default_duration"]

            if not self.test_data_collection(test_duration):
                return False

            # 5. Save results
            if not self.save_test_results():
                return False

            self.logger.info("=== Foot Force Sensor Basic Test Complete ===")
            self.logger.info(f"Test results saved to: {self.output_dir}")

            return True

        except KeyboardInterrupt:
            self.logger.info("User interrupted test")
            return False
        except Exception as e:
            self.logger.error(f"Basic test failed: {e}")
            return False
        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up resources"""
        try:
            if self.data_collector and self.data_collector.is_collecting:
                self.data_collector.stop_collection()

            if self.foot_force_config:
                self.foot_force_config.cleanup()

            self.logger.info("Resource cleanup complete")

        except Exception as e:
            self.logger.error(f"Error during resource cleanup: {e}")

def main():
    """Main function"""
    parser = argparse.ArgumentParser(description="Foot Force Sensor Basic Test")
    parser.add_argument('--duration', type=float, default=10.0,
                       help='Data collection duration (seconds)')
    parser.add_argument('--config', type=str, default='validation_config.json',
                       help='Configuration file path')
    parser.add_argument('--debug', action='store_true',
                       help='Enable debug mode')

    args = parser.parse_args()

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    # Create output directories
    os.makedirs('logs', exist_ok=True)
    os.makedirs('output', exist_ok=True)

    # Run test
    test = FootForceBasicTest(args.config)
    success = test.run_basic_test(args.duration)

    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()
