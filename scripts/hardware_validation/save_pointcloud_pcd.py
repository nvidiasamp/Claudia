#!/usr/bin/env python3
"""
Point Cloud PCD File Saver
Purpose: Save ROS2 point cloud data in PCD format for download and local viewing
Generated: 2025-06-27
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
from datetime import datetime
import os

def read_points_from_cloud(cloud_msg, field_names=None, skip_nans=True):
    """Read point data from a PointCloud2 message"""
    if field_names is None:
        field_names = ['x', 'y', 'z']

    points = []
    point_step = cloud_msg.point_step

    for i in range(0, len(cloud_msg.data), point_step):
        point_data = cloud_msg.data[i:i+point_step]
        point = {}

        for field in cloud_msg.fields:
            if field.name in field_names:
                offset = field.offset
                if field.datatype == PointField.FLOAT32:
                    value = struct.unpack('f', point_data[offset:offset+4])[0]
                else:
                    continue

                if skip_nans and isinstance(value, float) and np.isnan(value):
                    break

                point[field.name] = value

        if len(point) == len(field_names):
            points.append([point[name] for name in field_names])

    return np.array(points) if points else np.array([]).reshape(0, len(field_names))

def save_pcd_file(points, filename, frame_id="utlidar_lidar"):
    """Save point cloud data in PCD format"""
    if len(points) == 0:
        print("Empty point cloud data, skipping save")
        return False

    header = f"""# .PCD v0.7 - Point Cloud library
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {len(points)}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {len(points)}
DATA ascii
"""

    try:
        with open(filename, 'w') as f:
            f.write(header)
            for point in points:
                f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")

        return True
    except Exception as e:
        print(f"Failed to save PCD file: {e}")
        return False

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('pointcloud_saver')
        self.get_logger().info("Point cloud PCD saver started")

        # Ensure output directory exists
        self.output_dir = "logs/pointcloud_pcd"
        os.makedirs(self.output_dir, exist_ok=True)

        # Subscribe to point cloud topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/utlidar/cloud',
            self.pointcloud_callback,
            10
        )

        self.file_count = 0
        self.max_files = 3  # Save 3 PCD files

        print(f"Subscribed to topic: /utlidar/cloud")
        print(f"Output directory: {self.output_dir}")
        print(f"Will save {self.max_files} PCD files")

    def pointcloud_callback(self, msg):
        """Process point cloud data and save PCD files"""
        if self.file_count >= self.max_files:
            return

        try:
            # Parse point cloud data
            points = read_points_from_cloud(msg, ['x', 'y', 'z'])

            if len(points) == 0:
                self.get_logger().warn("Empty point cloud data")
                return

            # Generate filename
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{self.output_dir}/unitree_go2_lidar_{timestamp}_{self.file_count+1:03d}.pcd"

            # Save PCD file
            if save_pcd_file(points, filename):
                self.file_count += 1
                print(f"Saved PCD file {self.file_count}: {filename}")
                print(f"   Contains {len(points)} points")

                # Display statistics
                x, y, z = points[:, 0], points[:, 1], points[:, 2]
                distances = np.sqrt(x**2 + y**2 + z**2)
                print(f"   Range: X[{x.min():.2f}, {x.max():.2f}] "
                      f"Y[{y.min():.2f}, {y.max():.2f}] "
                      f"Z[{z.min():.2f}, {z.max():.2f}]")
                print(f"   Average distance: {distances.mean():.2f}m")

                if self.file_count >= self.max_files:
                    print(f"\nAll {self.max_files} PCD files saved!")
                    print(f"File location: {self.output_dir}/")
                    print(f"You can download these files and view them with:")
                    print(f"   - CloudCompare (recommended)")
                    print(f"   - PCL Viewer: pcl_viewer filename.pcd")
                    print(f"   - MeshLab")
                    print(f"   - Open3D Python")
                    rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Failed to process point cloud data: {e}")

def main():
    rclpy.init()

    print("Starting point cloud PCD saver...")
    print("Waiting for point cloud data...")

    saver = PointCloudSaver()

    try:
        rclpy.spin(saver)
    except KeyboardInterrupt:
        print("\nUser interrupted")
    finally:
        saver.destroy_node()
        rclpy.shutdown()
        print("Program exit")

if __name__ == '__main__':
    main()
