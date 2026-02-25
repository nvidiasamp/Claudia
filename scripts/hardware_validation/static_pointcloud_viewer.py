#!/usr/bin/env python3
"""
Static Point Cloud Image Generator
Purpose: Generate point cloud visualization images in SSH environments
Generated: 2025-06-27
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import struct
from datetime import datetime
import os

# Set font configuration
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'SimHei', 'Ubuntu']
plt.rcParams['axes.unicode_minus'] = False

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

class StaticPointCloudViewer(Node):
    def __init__(self):
        super().__init__('static_pointcloud_viewer')
        self.get_logger().info("Static point cloud image generator started")

        # Ensure output directory exists
        self.output_dir = "logs/pointcloud_images"
        os.makedirs(self.output_dir, exist_ok=True)

        # Subscribe to point cloud topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/utlidar/cloud',
            self.pointcloud_callback,
            10
        )

        self.image_count = 0
        self.max_images = 5  # Generate 5 images

        print(f"Subscribed to topic: /utlidar/cloud")
        print(f"Output directory: {self.output_dir}")
        print(f"Will generate {self.max_images} point cloud images")

    def pointcloud_callback(self, msg):
        """Process point cloud data and generate images"""
        if self.image_count >= self.max_images:
            return

        try:
            # Parse point cloud data
            points = read_points_from_cloud(msg, ['x', 'y', 'z'])

            if len(points) == 0:
                self.get_logger().warn("Empty point cloud data")
                return

            # Generate multi-view images
            self.generate_images(points, self.image_count + 1)
            self.image_count += 1

            print(f"Generated image {self.image_count} ({len(points)} points)")

            if self.image_count >= self.max_images:
                print(f"All {self.max_images} images generated!")
                print(f"Images saved in: {self.output_dir}/")
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Failed to generate image: {e}")

    def generate_images(self, points, frame_num):
        """Generate multi-view point cloud images"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Extract coordinates
        x, y, z = points[:, 0], points[:, 1], points[:, 2]

        # Calculate distances for color mapping
        distances = np.sqrt(x**2 + y**2 + z**2)

        # Create 2x2 subplot
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        fig.suptitle(f'Unitree Go2 LiDAR L1 Point Cloud - Frame {frame_num}\n'
                    f'{len(points)} points, Generated: {timestamp}',
                    fontsize=14, fontweight='bold')

        # Top view (XY plane)
        ax1 = axes[0, 0]
        scatter1 = ax1.scatter(x, y, c=distances, cmap='viridis', s=1, alpha=0.7)
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('Top View (XY Plane)')
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        plt.colorbar(scatter1, ax=ax1, label='Distance (m)', shrink=0.8)

        # Side view (XZ plane)
        ax2 = axes[0, 1]
        scatter2 = ax2.scatter(x, z, c=distances, cmap='plasma', s=1, alpha=0.7)
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Z (m)')
        ax2.set_title('Side View (XZ Plane)')
        ax2.grid(True, alpha=0.3)
        plt.colorbar(scatter2, ax=ax2, label='Distance (m)', shrink=0.8)

        # Front view (YZ plane)
        ax3 = axes[1, 0]
        scatter3 = ax3.scatter(y, z, c=distances, cmap='coolwarm', s=1, alpha=0.7)
        ax3.set_xlabel('Y (m)')
        ax3.set_ylabel('Z (m)')
        ax3.set_title('Front View (YZ Plane)')
        ax3.grid(True, alpha=0.3)
        plt.colorbar(scatter3, ax=ax3, label='Distance (m)', shrink=0.8)

        # Distance distribution histogram
        ax4 = axes[1, 1]
        ax4.hist(distances, bins=50, alpha=0.7, color='skyblue', edgecolor='black')
        ax4.set_xlabel('Distance (m)')
        ax4.set_ylabel('Point Count')
        ax4.set_title('Distance Distribution')
        ax4.grid(True, alpha=0.3)

        # Add statistics
        stats_text = f'Points: {len(points)}\n'
        stats_text += f'Range X: {x.min():.2f} ~ {x.max():.2f}m\n'
        stats_text += f'Range Y: {y.min():.2f} ~ {y.max():.2f}m\n'
        stats_text += f'Range Z: {z.min():.2f} ~ {z.max():.2f}m\n'
        stats_text += f'Avg Dist: {distances.mean():.2f}m'

        ax4.text(0.02, 0.98, stats_text, transform=ax4.transAxes,
                verticalalignment='top', bbox=dict(boxstyle='round',
                facecolor='wheat', alpha=0.8), fontsize=9)

        plt.tight_layout()

        # Save image
        filename = f"{self.output_dir}/pointcloud_frame_{frame_num:03d}_{timestamp}.png"
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close()

        print(f"Image saved: {filename}")

def main():
    rclpy.init()

    print("Starting static point cloud image generator...")
    print("Waiting for point cloud data...")

    viewer = StaticPointCloudViewer()

    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        print("\nUser interrupted")
    finally:
        viewer.destroy_node()
        rclpy.shutdown()
        print("Program exit")

if __name__ == '__main__':
    main()
