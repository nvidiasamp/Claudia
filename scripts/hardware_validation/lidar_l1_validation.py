#!/usr/bin/env python3
"""
4D LiDAR L1 Comprehensive Validation Script
Author: Claudia Robot Project
Date: 2025-01-27
Purpose: Validate the 4D LiDAR L1 sensor performance on the Unitree Go2 R&D Plus

Features include:
- Point cloud data stream acquisition and integrity checking
- Distance accuracy testing (comparison with known distances)
- 21600 points/second data rate verification
- 3D visualization and quality assessment
- Comprehensive validation report generation
"""

import time
import json
import numpy as np
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
# Commented out unitree_sdk2py, using ROS2 topics directly
# from unitree_sdk2py import Go2Robot
import matplotlib.pyplot as plt
from datetime import datetime
import os

# Try importing Open3D for 3D visualization
try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
    print("Open3D available for 3D visualization")
except ImportError:
    OPEN3D_AVAILABLE = False
    print("Open3D not available, will use matplotlib for 2D visualization")

def read_points_from_cloud(cloud_msg, field_names=None, skip_nans=True):
    """
    Read point cloud data from a PointCloud2 message.
    This is an alternative implementation of sensor_msgs_py.point_cloud2.read_points.
    """
    if field_names is None:
        field_names = [field.name for field in cloud_msg.fields]

    # Build field mapping
    field_map = {}
    for i, field in enumerate(cloud_msg.fields):
        if field.name in field_names:
            field_map[field.name] = (field.offset, field.datatype)

    # Parse point cloud data
    point_step = cloud_msg.point_step
    row_step = cloud_msg.row_step
    data = cloud_msg.data

    points = []
    for v in range(cloud_msg.height):
        for u in range(cloud_msg.width):
            # Calculate data offset
            offset = v * row_step + u * point_step
            point_data = {}

            for field_name, (field_offset, datatype) in field_map.items():
                idx = offset + field_offset

                # Parse based on data type
                if datatype == PointField.FLOAT32:
                    value = struct.unpack('f', data[idx:idx+4])[0]
                elif datatype == PointField.FLOAT64:
                    value = struct.unpack('d', data[idx:idx+8])[0]
                elif datatype == PointField.UINT32:
                    value = struct.unpack('I', data[idx:idx+4])[0]
                elif datatype == PointField.INT32:
                    value = struct.unpack('i', data[idx:idx+4])[0]
                elif datatype == PointField.UINT16:
                    value = struct.unpack('H', data[idx:idx+2])[0]
                elif datatype == PointField.INT16:
                    value = struct.unpack('h', data[idx:idx+2])[0]
                elif datatype == PointField.UINT8:
                    value = struct.unpack('B', data[idx:idx+1])[0]
                elif datatype == PointField.INT8:
                    value = struct.unpack('b', data[idx:idx+1])[0]
                else:
                    value = 0

                # Check for NaN
                if skip_nans and isinstance(value, float) and np.isnan(value):
                    continue

                point_data[field_name] = value

            # Only add point when all fields are present
            if len(point_data) == len(field_names):
                if field_names == ["x", "y", "z", "intensity"]:
                    points.append((point_data["x"], point_data["y"], point_data["z"], point_data["intensity"]))
                else:
                    points.append(tuple(point_data[name] for name in field_names))

    return points

class LiDARValidator(Node):
    """LiDAR L1 Validator class"""

    def __init__(self):
        super().__init__('lidar_l1_validator')

        # Initialize data storage
        self.point_count = 0
        self.start_time = time.time()
        self.point_cloud_data = []
        self.data_rate_history = []
        self.timestamps = []

        # Validation results storage
        self.validation_results = {
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'hardware': 'Unitree Go2 R&D Plus 4D LiDAR L1',
            'test_environment': {
                'indoor': True,
                'lighting': 'normal',
                'temperature': 'room_temp'
            }
        }

        print("LiDAR L1 validator initialization complete")
        print("=" * 60)

    def test_basic_connection(self):
        """Test basic connection"""
        print("Testing basic connection...")

        try:
            # Skip unitree_sdk2py connection, use ROS2 directly
            print("Using ROS2 topic mode, skipping SDK connection")

            # Create ROS2 topic subscriber for LiDAR data
            # Unitree robot LiDAR topic name
            lidar_topic = '/utlidar/cloud'

            self.lidar_subscription = self.create_subscription(
                PointCloud2,
                lidar_topic,
                self.lidar_callback,
                10
            )
            print(f"LiDAR topic subscription successful: {lidar_topic}")
            print("Starting to listen for Unitree LiDAR data...")

            return True

        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    def lidar_callback(self, msg):
        """LiDAR data callback function"""
        current_time = time.time()

        try:
            # Extract point cloud data using custom function
            points = read_points_from_cloud(msg, field_names=["x", "y", "z", "intensity"], skip_nans=True)
            point_count = len(points)

            # Record data
            self.point_count += point_count
            self.data_rate_history.append(point_count)
            self.timestamps.append(current_time)

            # Save latest point cloud data for analysis
            if len(self.point_cloud_data) < 100:  # Save latest 100 frames
                self.point_cloud_data.append({
                    'timestamp': current_time,
                    'points': points[:1000],  # Only save first 1000 points to conserve memory
                    'total_count': point_count
                })
            else:
                # Rolling update
                self.point_cloud_data.pop(0)
                self.point_cloud_data.append({
                    'timestamp': current_time,
                    'points': points[:1000],
                    'total_count': point_count
                })

            # Display data reception status in real time
            if len(self.data_rate_history) % 10 == 0:  # Display every 10 frames
                print(f"Receiving data: {point_count} points/frame, total: {self.point_count} points")

        except Exception as e:
            self.get_logger().error(f"Point cloud data processing error: {e}")

    def validate_data_rate(self, duration=10):
        """
        Validate whether LiDAR data rate reaches 21600 points/second.

        Args:
            duration: Test duration in seconds

        Returns:
            dict: Contains actual data rate, stability metrics, etc.
        """
        print(f"Starting {duration}-second data rate test...")

        # Clear previous data
        self.point_count = 0
        self.data_rate_history = []
        self.timestamps = []
        start_time = time.time()

        print("   Waiting for LiDAR data...")

        # Start ROS2 event loop for data collection
        while (time.time() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=0.1)

            # Display progress
            elapsed = time.time() - start_time
            if int(elapsed) % 2 == 0 and len(self.data_rate_history) > 0:
                current_rate = self.point_count / elapsed
                print(f"   Progress: {elapsed:.1f}/{duration}s, current rate: {current_rate:,.0f} points/sec")

        # Calculate statistics
        if len(self.data_rate_history) == 0:
            print("No LiDAR data received")
            print("Please check:")
            print("   1. Is the robot connected and powered on?")
            print("   2. Is the LiDAR topic name correct?")
            print("   3. Is the network configuration correct?")
            return None

        total_time = time.time() - start_time
        actual_rate = self.point_count / total_time
        target_rate = 21600

        # Calculate data stability
        rates_per_second = []
        if len(self.timestamps) > 1:
            for i in range(len(self.timestamps)-1):
                time_diff = self.timestamps[i+1] - self.timestamps[i]
                if time_diff > 0:
                    rate = self.data_rate_history[i+1] / time_diff
                    rates_per_second.append(rate)

        results = {
            'actual_rate': actual_rate,
            'target_rate': target_rate,
            'deviation_percent': abs(actual_rate - target_rate) / target_rate * 100,
            'stability_std': np.std(rates_per_second) if rates_per_second else 0,
            'total_samples': len(self.data_rate_history),
            'test_duration': total_time
        }

        print(f"Data rate test results:")
        print(f"   Target rate: {target_rate:,} points/sec")
        print(f"   Actual rate: {actual_rate:,.1f} points/sec")
        print(f"   Deviation: {results['deviation_percent']:.2f}%")
        print(f"   Stability (std dev): {results['stability_std']:.1f}")
        print(f"   Samples: {results['total_samples']}")

        return results

    def validate_distance_accuracy(self, test_distances=[0.5, 1.0, 2.0, 3.0]):
        """
        Test distance accuracy using standard targets at known distances.

        Args:
            test_distances: List of test distances in meters

        Returns:
            dict: Distance accuracy test results
        """
        print("Starting distance accuracy test...")
        print("Please place a high-reflectivity standard board (e.g., white cardboard) at the following distances:")

        accuracy_results = {}

        for target_distance in test_distances:
            print(f"\nTest distance: {target_distance}m")
            input("   Place the standard board at the specified distance, then press Enter to continue...")

            # Collect multiple measurements
            measurements = []
            print("   Collecting data...")

            for i in range(10):  # Take average of 10 measurements
                # Get latest point cloud data
                rclpy.spin_once(self, timeout_sec=0.5)

                if self.point_cloud_data:
                    latest_data = self.point_cloud_data[-1]
                    points = latest_data['points']

                    # Find the nearest high-reflectivity point (standard board)
                    distances = []
                    for point in points:
                        x, y, z, intensity = point
                        if intensity > 200:  # High reflectivity threshold
                            distance = np.sqrt(x**2 + y**2 + z**2)
                            if 0.1 < distance < 10:  # Reasonable distance range
                                distances.append(distance)

                    if distances:
                        measured_distance = min(distances)  # Nearest high-reflectivity point
                        measurements.append(measured_distance)
                        print(f"     Measurement {i+1}: {measured_distance:.3f}m")

                time.sleep(0.2)

            if measurements:
                mean_measured = np.mean(measurements)
                std_measured = np.std(measurements)
                error = abs(mean_measured - target_distance)
                error_percent = error / target_distance * 100

                accuracy_results[target_distance] = {
                    'measured_mean': mean_measured,
                    'measured_std': std_measured,
                    'absolute_error': error,
                    'relative_error_percent': error_percent,
                    'measurements': measurements
                }

                print(f"   Result: {mean_measured:.3f}+/-{std_measured:.3f}m")
                print(f"   Error: {error:.3f}m ({error_percent:.2f}%)")

                # Evaluate accuracy grade
                if error_percent < 2:
                    print("   Accuracy grade: Excellent")
                elif error_percent < 5:
                    print("   Accuracy grade: Good")
                elif error_percent < 10:
                    print("   Accuracy grade: Fair")
                else:
                    print("   Accuracy grade: Needs calibration")
            else:
                print("   Failed to detect standard board. Please check placement position and reflectivity.")

        return accuracy_results

    def setup_3d_visualization(self):
        """Set up 3D visualization"""
        if not OPEN3D_AVAILABLE:
            print("Open3D not available, skipping 3D visualization")
            return False

        print("Starting 3D point cloud visualization...")
        print("   Press ESC to exit visualization")

        try:
            # Create Open3D visualizer
            vis = o3d.visualization.Visualizer()
            vis.create_window("LiDAR L1 Real-time Point Cloud", width=1200, height=800)

            # Create point cloud object
            pcd = o3d.geometry.PointCloud()
            vis.add_geometry(pcd)

            # Set viewpoint
            view_control = vis.get_view_control()
            view_control.set_front([0, 0, 1])
            view_control.set_up([0, 1, 0])

            visualization_start = time.time()
            frame_count = 0

            while True:
                # Get latest point cloud data
                rclpy.spin_once(self, timeout_sec=0.1)

                if self.point_cloud_data:
                    latest_data = self.point_cloud_data[-1]
                    points = latest_data['points']

                    if points:
                        # Convert to numpy array
                        point_array = np.array([[p[0], p[1], p[2]] for p in points])
                        intensity_array = np.array([p[3] for p in points])

                        # Generate colors based on intensity
                        colors = np.zeros((len(point_array), 3))
                        normalized_intensity = intensity_array / 255.0
                        colors[:, 0] = normalized_intensity  # Red channel
                        colors[:, 1] = 0.5  # Green channel
                        colors[:, 2] = 1.0 - normalized_intensity  # Blue channel

                        # Update point cloud
                        pcd.points = o3d.utility.Vector3dVector(point_array)
                        pcd.colors = o3d.utility.Vector3dVector(colors)

                        # Refresh visualization
                        vis.update_geometry(pcd)

                        frame_count += 1

                if not vis.poll_events():
                    break
                vis.update_renderer()

                time.sleep(0.05)  # 20Hz refresh rate

                # Display runtime
                if frame_count % 60 == 0:
                    elapsed = time.time() - visualization_start
                    print(f"   Visualization runtime: {elapsed:.1f}s, frames: {frame_count}")

            vis.destroy_window()
            print("3D visualization closed")
            return True

        except Exception as e:
            print(f"3D visualization error: {e}")
            return False

    def setup_2d_visualization(self):
        """Set up 2D visualization as a fallback"""
        print("Starting 2D point cloud visualization...")

        try:
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

            visualization_start = time.time()
            frame_count = 0

            while frame_count < 100:  # Display 100 frames then stop
                # Get latest point cloud data
                rclpy.spin_once(self, timeout_sec=0.1)

                if self.point_cloud_data:
                    latest_data = self.point_cloud_data[-1]
                    points = latest_data['points']

                    if points and len(points) > 10:
                        # Extract coordinates and intensity
                        x = [p[0] for p in points]
                        y = [p[1] for p in points]
                        z = [p[2] for p in points]
                        intensity = [p[3] for p in points]

                        # Clear and plot XY plane
                        ax1.clear()
                        scatter1 = ax1.scatter(x, y, c=intensity, cmap='viridis', s=1)
                        ax1.set_xlabel('X (m)')
                        ax1.set_ylabel('Y (m)')
                        ax1.set_title(f'LiDAR XY Plane View (Frame {frame_count})')
                        ax1.grid(True)
                        ax1.axis('equal')

                        # Clear and plot XZ plane
                        ax2.clear()
                        scatter2 = ax2.scatter(x, z, c=intensity, cmap='viridis', s=1)
                        ax2.set_xlabel('X (m)')
                        ax2.set_ylabel('Z (m)')
                        ax2.set_title(f'LiDAR XZ Plane View (Frame {frame_count})')
                        ax2.grid(True)

                        plt.pause(0.1)
                        frame_count += 1

                time.sleep(0.1)

            plt.show()
            print("2D visualization complete")
            return True

        except Exception as e:
            print(f"2D visualization error: {e}")
            return False

    def generate_validation_report(self, data_rate_results, accuracy_results):
        """Generate comprehensive validation report"""
        print("Generating validation report...")

        self.validation_results.update({
            'data_rate_test': data_rate_results,
            'distance_accuracy_test': accuracy_results,
        })

        # Evaluate overall performance
        overall_status = "PASS"
        recommendations = []

        # Evaluate data rate
        if data_rate_results and data_rate_results['deviation_percent'] < 10:
            self.validation_results['data_rate_status'] = 'PASS'
        else:
            self.validation_results['data_rate_status'] = 'FAIL'
            overall_status = "FAIL"
            recommendations.append('Check data transmission bandwidth and network configuration')

        # Evaluate distance accuracy
        if accuracy_results:
            avg_error = np.mean([r['relative_error_percent'] for r in accuracy_results.values()])
            if avg_error < 5:
                self.validation_results['accuracy_status'] = 'PASS'
            else:
                self.validation_results['accuracy_status'] = 'FAIL'
                overall_status = "FAIL"
                recommendations.append('Distance calibration required')
        else:
            self.validation_results['accuracy_status'] = 'NOT_TESTED'
            recommendations.append('Recommended to complete distance accuracy testing')

        self.validation_results['overall_status'] = overall_status
        self.validation_results['recommendations'] = recommendations

        # Save report
        os.makedirs('logs', exist_ok=True)
        report_path = f"logs/lidar_l1_validation_{int(time.time())}.json"

        with open(report_path, 'w', encoding='utf-8') as f:
            json.dump(self.validation_results, f, indent=2, ensure_ascii=False)

        # Print report summary
        print("\n" + "="*60)
        print("LiDAR L1 Validation Report Summary")
        print("="*60)
        print(f"Test time: {self.validation_results['timestamp']}")
        print(f"Hardware: {self.validation_results['hardware']}")
        print(f"Overall status: {overall_status}")

        if data_rate_results:
            print(f"\nData rate test:")
            print(f"   Status: {self.validation_results['data_rate_status']}")
            print(f"   Actual rate: {data_rate_results['actual_rate']:,.1f} points/sec")
            print(f"   Target rate: {data_rate_results['target_rate']:,} points/sec")
            print(f"   Deviation: {data_rate_results['deviation_percent']:.2f}%")

        if accuracy_results:
            print(f"\nDistance accuracy test:")
            print(f"   Status: {self.validation_results['accuracy_status']}")
            for distance, result in accuracy_results.items():
                print(f"   {distance}m: error {result['relative_error_percent']:.2f}%")

        if recommendations:
            print(f"\nRecommendations:")
            for i, rec in enumerate(recommendations, 1):
                print(f"   {i}. {rec}")

        print(f"\nDetailed report saved: {report_path}")
        print("="*60)

        return report_path

def main():
    """Main function"""
    print("Unitree Go2 R&D Plus - 4D LiDAR L1 Validation Program")
    print("=" * 60)

    # Initialize ROS2
    rclpy.init()

    try:
        # Create validator
        validator = LiDARValidator()

        # Test basic connection
        if not validator.test_basic_connection():
            print("Basic connection test failed. Please check robot connection and ROS2 environment.")
            return

        print("\nStarting LiDAR L1 validation process...")

        # 1. Data rate validation
        print("\n" + "="*40)
        data_rate_results = validator.validate_data_rate(duration=15)

        # 2. Distance accuracy test
        print("\n" + "="*40)
        choice = input("Run distance accuracy test? (y/n): ").lower().strip()
        if choice == 'y':
            accuracy_results = validator.validate_distance_accuracy()
        else:
            accuracy_results = {}
            print("Skipping distance accuracy test")

        # 3. Visualization test
        print("\n" + "="*40)
        choice = input("Run 3D visualization test? (y/n): ").lower().strip()
        if choice == 'y':
            if not validator.setup_3d_visualization():
                print("Trying 2D visualization...")
                validator.setup_2d_visualization()
        else:
            print("Skipping visualization test")

        # 4. Generate report
        print("\n" + "="*40)
        report_path = validator.generate_validation_report(data_rate_results, accuracy_results)

        print("\nLiDAR L1 validation complete!")

    except KeyboardInterrupt:
        print("\n\nUser interrupted validation program")
    except Exception as e:
        print(f"\nValidation program error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Clean up ROS2
        rclpy.shutdown()
        print("Program exit")

if __name__ == "__main__":
    main()
