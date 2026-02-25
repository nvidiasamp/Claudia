#!/usr/bin/env python3
# scripts/validation/imu/imu_validation/visualizer.py
# Generated: 2025-06-27 11:54:45 CST
# Purpose: Unitree Go2 IMU real-time data visualization

import time
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import logging
from typing import Dict, List, Tuple, Any, Optional
from collections import deque
import queue

from imu_config import IMUConfig, IMUReading
from data_collector import IMUDataCollector

class IMUVisualizer:
    """IMU data real-time visualizer"""

    def __init__(self, imu_config: IMUConfig, data_collector: IMUDataCollector, config: Dict[str, Any]):
        """
        Initialize visualizer

        Args:
            imu_config: IMU configuration instance
            data_collector: Data collector instance
            config: Validation configuration
        """
        self.imu_config = imu_config
        self.data_collector = data_collector
        self.config = config
        self.logger = logging.getLogger(__name__)

        # Visualization configuration
        self.viz_config = config.get("visualization", {})
        self.enable_real_time = self.viz_config.get("real_time_plots", True)
        self.update_interval = self.viz_config.get("plot_update_interval_ms", 100)
        self.max_points = self.viz_config.get("max_plot_points", 500)
        self.enable_3d = self.viz_config.get("enable_3d_orientation", True)

        # Data buffer
        self.plot_data = {
            'timestamps': deque(maxlen=self.max_points),
            'accelerometer': {
                'x': deque(maxlen=self.max_points),
                'y': deque(maxlen=self.max_points),
                'z': deque(maxlen=self.max_points),
                'magnitude': deque(maxlen=self.max_points)
            },
            'gyroscope': {
                'x': deque(maxlen=self.max_points),
                'y': deque(maxlen=self.max_points),
                'z': deque(maxlen=self.max_points),
                'magnitude': deque(maxlen=self.max_points)
            },
            'orientation': {
                'roll': deque(maxlen=self.max_points),
                'pitch': deque(maxlen=self.max_points),
                'yaw': deque(maxlen=self.max_points)
            },
            'quaternion': {
                'w': deque(maxlen=self.max_points),
                'x': deque(maxlen=self.max_points),
                'y': deque(maxlen=self.max_points),
                'z': deque(maxlen=self.max_points)
            }
        }

        # Visualization state
        self.is_plotting = False
        self.fig = None
        self.axes = {}
        self.lines = {}
        self.animation = None

        # Data update queue
        self.data_queue = queue.Queue(maxsize=100)

        # 3D attitude visualization
        self.orientation_fig = None
        self.orientation_ax = None
        self.orientation_lines = {}

    def start_visualization(self, mode: str = "all") -> bool:
        """
        Start visualization

        Args:
            mode: Visualization mode ("all", "time_series", "3d_orientation")

        Returns:
            bool: Whether startup was successful
        """
        try:
            if not self.enable_real_time:
                self.logger.info("Real-time visualization is disabled")
                return False

            if self.is_plotting:
                self.logger.warning("Visualization is already running")
                return False

            # Register data callback
            self.data_collector.add_data_callback(self._data_callback)

            if mode == "all":
                success = self._start_time_series_plot() and (self._start_3d_orientation() if self.enable_3d else True)
            elif mode == "time_series":
                success = self._start_time_series_plot()
            elif mode == "3d_orientation":
                success = self._start_3d_orientation()
            else:
                self.logger.error(f"Unknown visualization mode: {mode}")
                return False

            if success:
                self.is_plotting = True
                self.logger.info(f"IMU visualization started, mode: {mode}")

            return success

        except Exception as e:
            self.logger.error(f"Failed to start visualization: {e}")
            return False

    def stop_visualization(self) -> bool:
        """Stop visualization"""
        try:
            self.is_plotting = False

            # Remove data callback
            self.data_collector.remove_data_callback(self._data_callback)

            # Stop animation
            if self.animation:
                self.animation.event_source.stop()
                self.animation = None

            # Close figure windows
            if self.fig:
                plt.close(self.fig)
                self.fig = None

            if self.orientation_fig:
                plt.close(self.orientation_fig)
                self.orientation_fig = None

            self.logger.info("IMU visualization stopped")

            return True

        except Exception as e:
            self.logger.error(f"Failed to stop visualization: {e}")
            return False

    def _data_callback(self, reading: IMUReading):
        """Data callback function"""
        try:
            # Put data into queue to avoid blocking data collection
            if not self.data_queue.full():
                self.data_queue.put(reading, block=False)
        except queue.Full:
            pass  # Discard data when queue is full
        except Exception as e:
            self.logger.error(f"Data callback processing failed: {e}")

    def _process_data_queue(self):
        """Process data queue"""
        try:
            while not self.data_queue.empty():
                reading = self.data_queue.get(block=False)
                self._update_plot_data(reading)
        except queue.Empty:
            pass
        except Exception as e:
            self.logger.error(f"Data queue processing failed: {e}")

    def _update_plot_data(self, reading: IMUReading):
        """Update plot data"""
        try:
            # Add timestamp
            self.plot_data['timestamps'].append(reading.timestamp)

            # Add accelerometer data
            self.plot_data['accelerometer']['x'].append(reading.accelerometer[0])
            self.plot_data['accelerometer']['y'].append(reading.accelerometer[1])
            self.plot_data['accelerometer']['z'].append(reading.accelerometer[2])
            self.plot_data['accelerometer']['magnitude'].append(np.linalg.norm(reading.accelerometer))

            # Add gyroscope data
            self.plot_data['gyroscope']['x'].append(reading.gyroscope[0])
            self.plot_data['gyroscope']['y'].append(reading.gyroscope[1])
            self.plot_data['gyroscope']['z'].append(reading.gyroscope[2])
            self.plot_data['gyroscope']['magnitude'].append(np.linalg.norm(reading.gyroscope))

            # Add quaternion data
            self.plot_data['quaternion']['w'].append(reading.quaternion[0])
            self.plot_data['quaternion']['x'].append(reading.quaternion[1])
            self.plot_data['quaternion']['y'].append(reading.quaternion[2])
            self.plot_data['quaternion']['z'].append(reading.quaternion[3])

            # Convert to Euler angles
            roll, pitch, yaw = self.imu_config.quaternion_to_euler(reading.quaternion)
            self.plot_data['orientation']['roll'].append(np.degrees(roll))
            self.plot_data['orientation']['pitch'].append(np.degrees(pitch))
            self.plot_data['orientation']['yaw'].append(np.degrees(yaw))

        except Exception as e:
            self.logger.error(f"Failed to update plot data: {e}")

    def _start_time_series_plot(self) -> bool:
        """Start time-series plot visualization"""
        try:
            # Create main figure
            self.fig, self.axes = plt.subplots(3, 2, figsize=(15, 10))
            self.fig.suptitle('Unitree Go2 IMU Real-time Data', fontsize=16)

            # Subplot layout:
            # [0,0]: Accelerometer XYZ    [0,1]: Accelerometer magnitude
            # [1,0]: Gyroscope XYZ        [1,1]: Gyroscope magnitude
            # [2,0]: Euler angles         [2,1]: Quaternion

            # Initialize empty data lines
            self.lines = {}

            # Accelerometer XYZ
            ax = self.axes[0, 0]
            ax.set_title('Accelerometer (m/s^2)')
            ax.set_ylabel('Acceleration')
            ax.grid(True, alpha=0.3)
            self.lines['accel_x'], = ax.plot([], [], 'r-', label='X', alpha=0.8)
            self.lines['accel_y'], = ax.plot([], [], 'g-', label='Y', alpha=0.8)
            self.lines['accel_z'], = ax.plot([], [], 'b-', label='Z', alpha=0.8)
            ax.legend(loc='upper right')

            # Accelerometer magnitude
            ax = self.axes[0, 1]
            ax.set_title('Accelerometer Magnitude')
            ax.set_ylabel('|Acceleration| (m/s^2)')
            ax.grid(True, alpha=0.3)
            self.lines['accel_mag'], = ax.plot([], [], 'k-', linewidth=2)
            ax.axhline(y=9.81, color='r', linestyle='--', alpha=0.7, label='Gravity')
            ax.legend(loc='upper right')

            # Gyroscope XYZ
            ax = self.axes[1, 0]
            ax.set_title('Gyroscope (rad/s)')
            ax.set_ylabel('Angular Velocity')
            ax.grid(True, alpha=0.3)
            self.lines['gyro_x'], = ax.plot([], [], 'r-', label='X', alpha=0.8)
            self.lines['gyro_y'], = ax.plot([], [], 'g-', label='Y', alpha=0.8)
            self.lines['gyro_z'], = ax.plot([], [], 'b-', label='Z', alpha=0.8)
            ax.legend(loc='upper right')

            # Gyroscope magnitude
            ax = self.axes[1, 1]
            ax.set_title('Gyroscope Magnitude')
            ax.set_ylabel('|Angular Velocity| (rad/s)')
            ax.grid(True, alpha=0.3)
            self.lines['gyro_mag'], = ax.plot([], [], 'k-', linewidth=2)

            # Euler angles
            ax = self.axes[2, 0]
            ax.set_title('Euler Angles (degrees)')
            ax.set_ylabel('Angle')
            ax.set_xlabel('Time (s)')
            ax.grid(True, alpha=0.3)
            self.lines['roll'], = ax.plot([], [], 'r-', label='Roll', alpha=0.8)
            self.lines['pitch'], = ax.plot([], [], 'g-', label='Pitch', alpha=0.8)
            self.lines['yaw'], = ax.plot([], [], 'b-', label='Yaw', alpha=0.8)
            ax.legend(loc='upper right')

            # Quaternion
            ax = self.axes[2, 1]
            ax.set_title('Quaternion')
            ax.set_ylabel('Value')
            ax.set_xlabel('Time (s)')
            ax.grid(True, alpha=0.3)
            self.lines['quat_w'], = ax.plot([], [], 'k-', label='W', alpha=0.8)
            self.lines['quat_x'], = ax.plot([], [], 'r-', label='X', alpha=0.8)
            self.lines['quat_y'], = ax.plot([], [], 'g-', label='Y', alpha=0.8)
            self.lines['quat_z'], = ax.plot([], [], 'b-', label='Z', alpha=0.8)
            ax.legend(loc='upper right')

            # Start animation
            self.animation = animation.FuncAnimation(
                self.fig,
                self._update_time_series_plot,
                interval=self.update_interval,
                blit=False,
                cache_frame_data=False
            )

            plt.tight_layout()
            plt.show(block=False)

            return True

        except Exception as e:
            self.logger.error(f"Failed to start time-series plot visualization: {e}")
            return False

    def _update_time_series_plot(self, frame):
        """Update time-series plot"""
        try:
            # Process data queue
            self._process_data_queue()

            if len(self.plot_data['timestamps']) < 2:
                return list(self.lines.values())

            # Calculate relative time (seconds)
            timestamps = list(self.plot_data['timestamps'])
            if timestamps:
                start_time = timestamps[0]
                times = [(t - start_time) for t in timestamps]
            else:
                times = []

            # Update accelerometer data
            self.lines['accel_x'].set_data(times, list(self.plot_data['accelerometer']['x']))
            self.lines['accel_y'].set_data(times, list(self.plot_data['accelerometer']['y']))
            self.lines['accel_z'].set_data(times, list(self.plot_data['accelerometer']['z']))
            self.lines['accel_mag'].set_data(times, list(self.plot_data['accelerometer']['magnitude']))

            # Update gyroscope data
            self.lines['gyro_x'].set_data(times, list(self.plot_data['gyroscope']['x']))
            self.lines['gyro_y'].set_data(times, list(self.plot_data['gyroscope']['y']))
            self.lines['gyro_z'].set_data(times, list(self.plot_data['gyroscope']['z']))
            self.lines['gyro_mag'].set_data(times, list(self.plot_data['gyroscope']['magnitude']))

            # Update Euler angle data
            self.lines['roll'].set_data(times, list(self.plot_data['orientation']['roll']))
            self.lines['pitch'].set_data(times, list(self.plot_data['orientation']['pitch']))
            self.lines['yaw'].set_data(times, list(self.plot_data['orientation']['yaw']))

            # Update quaternion data
            self.lines['quat_w'].set_data(times, list(self.plot_data['quaternion']['w']))
            self.lines['quat_x'].set_data(times, list(self.plot_data['quaternion']['x']))
            self.lines['quat_y'].set_data(times, list(self.plot_data['quaternion']['y']))
            self.lines['quat_z'].set_data(times, list(self.plot_data['quaternion']['z']))

            # Update axis ranges
            if times:
                time_range = max(times) - min(times)
                time_window = max(30.0, time_range)  # At least 30 second window

                for i in range(3):
                    for j in range(2):
                        ax = self.axes[i, j]
                        ax.set_xlim(max(times) - time_window, max(times))
                        ax.relim()
                        ax.autoscale_view(scalex=False, scaley=True)

            return list(self.lines.values())

        except Exception as e:
            self.logger.error(f"Failed to update time-series plot: {e}")
            return list(self.lines.values())

    def _start_3d_orientation(self) -> bool:
        """Start 3D attitude visualization"""
        try:
            if not self.enable_3d:
                return True

            # Create 3D figure
            self.orientation_fig = plt.figure(figsize=(10, 8))
            self.orientation_ax = self.orientation_fig.add_subplot(111, projection='3d')
            self.orientation_ax.set_title('IMU 3D Orientation (Robot Frame)', fontsize=14)

            # Set axis labels
            self.orientation_ax.set_xlabel('X (Forward)')
            self.orientation_ax.set_ylabel('Y (Left)')
            self.orientation_ax.set_zlabel('Z (Up)')

            # Set axis ranges
            self.orientation_ax.set_xlim([-1.5, 1.5])
            self.orientation_ax.set_ylim([-1.5, 1.5])
            self.orientation_ax.set_zlim([-1.5, 1.5])

            # Create axis lines (robot coordinate frame)
            origin = [0, 0, 0]

            # X-axis (forward direction, red)
            self.orientation_lines['x_axis'] = self.orientation_ax.plot([origin[0], 1], [origin[1], 0], [origin[2], 0], 'r-', linewidth=3, label='X (Forward)')[0]

            # Y-axis (left direction, green)
            self.orientation_lines['y_axis'] = self.orientation_ax.plot([origin[0], 0], [origin[1], 1], [origin[2], 0], 'g-', linewidth=3, label='Y (Left)')[0]

            # Z-axis (up direction, blue)
            self.orientation_lines['z_axis'] = self.orientation_ax.plot([origin[0], 0], [origin[1], 0], [origin[2], 1], 'b-', linewidth=3, label='Z (Up)')[0]

            # Add reference grid
            self.orientation_ax.plot([0, 0], [0, 0], [-1.5, 1.5], 'k--', alpha=0.3)
            self.orientation_ax.plot([0, 0], [-1.5, 1.5], [0, 0], 'k--', alpha=0.3)
            self.orientation_ax.plot([-1.5, 1.5], [0, 0], [0, 0], 'k--', alpha=0.3)

            self.orientation_ax.legend()

            # Create animation updater
            def update_3d_orientation(frame):
                return self._update_3d_orientation()

            self.orientation_animation = animation.FuncAnimation(
                self.orientation_fig,
                update_3d_orientation,
                interval=self.update_interval,
                blit=False,
                cache_frame_data=False
            )

            plt.show(block=False)

            return True

        except Exception as e:
            self.logger.error(f"Failed to start 3D attitude visualization: {e}")
            return False

    def _update_3d_orientation(self):
        """Update 3D attitude display"""
        try:
            # Get latest IMU reading
            latest_reading = self.imu_config.get_latest_reading()

            if not latest_reading:
                return list(self.orientation_lines.values())

            # Extract quaternion and normalize
            quat = np.array(latest_reading.quaternion)
            quat = quat / np.linalg.norm(quat)  # Ensure normalization

            # Quaternion to rotation matrix
            w, x, y, z = quat

            # Rotation matrix
            R = np.array([
                [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
            ])

            # Original axis vectors
            x_axis = np.array([1, 0, 0])
            y_axis = np.array([0, 1, 0])
            z_axis = np.array([0, 0, 1])

            # Rotated axes
            rotated_x = R @ x_axis
            rotated_y = R @ y_axis
            rotated_z = R @ z_axis

            # Update axis lines
            origin = [0, 0, 0]

            self.orientation_lines['x_axis'].set_data_3d([origin[0], rotated_x[0]], [origin[1], rotated_x[1]], [origin[2], rotated_x[2]])
            self.orientation_lines['y_axis'].set_data_3d([origin[0], rotated_y[0]], [origin[1], rotated_y[1]], [origin[2], rotated_y[2]])
            self.orientation_lines['z_axis'].set_data_3d([origin[0], rotated_z[0]], [origin[1], rotated_z[1]], [origin[2], rotated_z[2]])

            # Update title with current angles
            roll, pitch, yaw = self.imu_config.quaternion_to_euler(latest_reading.quaternion)
            self.orientation_ax.set_title(
                f'IMU 3D Orientation\nRoll: {np.degrees(roll):.1f} deg | Pitch: {np.degrees(pitch):.1f} deg | Yaw: {np.degrees(yaw):.1f} deg',
                fontsize=12
            )

            return list(self.orientation_lines.values())

        except Exception as e:
            self.logger.error(f"Failed to update 3D attitude: {e}")
            return list(self.orientation_lines.values())

    def get_plot_statistics(self) -> Dict[str, Any]:
        """Get plot statistics"""
        try:
            stats = {
                'data_points': len(self.plot_data['timestamps']),
                'plot_update_count': getattr(self, '_plot_update_count', 0),
                'active_plots': len(self.fig) if hasattr(self, 'fig') else 0,
                'visualization_duration': time.time() - self.start_time if hasattr(self, 'start_time') else 0,
                'last_update_time': getattr(self, '_last_update_time', 0)
            }

            if self.plot_data['timestamps']:
                stats['data_rate_hz'] = len(self.plot_data['timestamps']) / max(1, stats['visualization_duration'])
                stats['first_timestamp'] = self.plot_data['timestamps'][0]
                stats['last_timestamp'] = self.plot_data['timestamps'][-1]

            return stats

        except Exception as e:
            self.logger.error(f"Failed to get plot statistics: {e}")
            return {'data_points': 0, 'error': str(e)}

    def save_current_plots(self, output_dir: str) -> bool:
        """Save current plots to specified directory"""
        try:
            from pathlib import Path
            import matplotlib.pyplot as plt

            output_path = Path(output_dir)
            output_path.mkdir(parents=True, exist_ok=True)

            # Save all current figures
            saved_files = []

            if hasattr(self, 'fig') and self.fig:
                filename = output_path / f"imu_timeseries.png"
                self.fig.savefig(filename, dpi=300, bbox_inches='tight')
                saved_files.append(str(filename))

            if hasattr(self, 'orientation_fig') and self.orientation_fig:
                filename = output_path / f"imu_3d_orientation.png"
                self.orientation_fig.savefig(filename, dpi=300, bbox_inches='tight')
                saved_files.append(str(filename))

            self.logger.info(f"Plots saved to: {output_dir}")
            self.logger.info(f"Saved files: {saved_files}")
            return True

        except Exception as e:
            self.logger.error(f"Failed to save plots: {e}")
            return False
