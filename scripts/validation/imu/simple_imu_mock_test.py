#!/usr/bin/env python3
# Simplified IMU validation test (no hardware dependency)

import time
import numpy as np
from dataclasses import dataclass
from typing import Tuple, List, Dict, Any

@dataclass
class MockIMUReading:
    """Simulated IMU reading"""
    timestamp: float
    accelerometer: Tuple[float, float, float]
    gyroscope: Tuple[float, float, float]
    quaternion: Tuple[float, float, float, float]
    temperature: int = 25

class MockIMUValidator:
    """Simulated IMU validator"""

    def __init__(self):
        self.readings = []

    def generate_mock_data(self, duration: float, sample_rate: float = 100) -> List[MockIMUReading]:
        """Generate simulated data"""
        readings = []
        num_samples = int(duration * sample_rate)

        for i in range(num_samples):
            t = time.time() + i / sample_rate

            # Simulate static data (mainly gravity)
            accel = (
                np.random.normal(0, 0.01),  # X-axis noise
                np.random.normal(0, 0.01),  # Y-axis noise
                np.random.normal(-9.81, 0.02)  # Z-axis gravity + noise
            )

            # Simulate static gyroscope (near zero)
            gyro = (
                np.random.normal(0, 0.001),
                np.random.normal(0, 0.001),
                np.random.normal(0, 0.001)
            )

            # Simulate quaternion (near unit quaternion)
            quat = (
                np.random.normal(1, 0.001),
                np.random.normal(0, 0.001),
                np.random.normal(0, 0.001),
                np.random.normal(0, 0.001)
            )

            reading = MockIMUReading(t, accel, gyro, quat)
            readings.append(reading)

        return readings

    def analyze_static_stability(self, readings: List[MockIMUReading]) -> Dict[str, Any]:
        """Analyze static stability"""
        if not readings:
            return {'status': 'FAIL', 'error': 'No data'}

        # Extract data
        accels = np.array([r.accelerometer for r in readings])
        gyros = np.array([r.gyroscope for r in readings])

        # Calculate statistics
        accel_std = np.std(accels, axis=0)
        gyro_std = np.std(gyros, axis=0)
        gravity_mag = np.mean(np.linalg.norm(accels, axis=1))

        # Evaluate
        accel_stable = np.max(accel_std) < 0.05
        gyro_stable = np.max(gyro_std) < 0.01
        gravity_accurate = abs(gravity_mag - 9.81) < 0.2

        return {
            'status': 'PASS' if all([accel_stable, gyro_stable, gravity_accurate]) else 'FAIL',
            'accelerometer_std': accel_std.tolist(),
            'gyroscope_std': gyro_std.tolist(),
            'gravity_magnitude': float(gravity_mag),
            'test_duration': readings[-1].timestamp - readings[0].timestamp,
            'sample_count': len(readings),
            'pass_criteria': {
                'accelerometer_stability': accel_stable,
                'gyroscope_stability': gyro_stable,
                'gravity_accuracy': gravity_accurate
            }
        }

    def run_mock_validation(self) -> Dict[str, Any]:
        """Run simulated validation"""
        print("Starting simulated IMU validation...")

        # Generate simulated data
        print("Generating simulated data...")
        readings = self.generate_mock_data(10.0)  # 10 seconds of data

        # Analyze
        print("Analyzing static stability...")
        static_results = self.analyze_static_stability(readings)

        # Generate report
        print("Generating validation report...")
        report = {
            'test_type': 'mock_validation',
            'test_timestamp': time.time(),
            'static_stability': static_results,
            'overall_status': static_results['status']
        }

        return report

def main():
    """Main function"""
    print("=" * 50)
    print("Simplified IMU Validation Test")
    print("=" * 50)

    validator = MockIMUValidator()
    report = validator.run_mock_validation()

    print("\nValidation results:")
    print(f"Status: {report['overall_status']}")
    print(f"Sample count: {report['static_stability']['sample_count']}")
    print(f"Gravity measurement: {report['static_stability']['gravity_magnitude']:.3f} m/s^2")
    print(f"Accelerometer standard deviation: {report['static_stability']['accelerometer_std']}")

    return 0 if report['overall_status'] == 'PASS' else 1

if __name__ == "__main__":
    exit(main())
