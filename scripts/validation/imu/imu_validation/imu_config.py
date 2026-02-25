#!/usr/bin/env python3
# scripts/validation/imu/imu_validation/imu_config.py
# Generated: 2025-06-27 11:54:45 CST
# Purpose: Unitree Go2 IMU configuration and initialization management

import time
import json
import logging
import numpy as np
import threading
from typing import Dict, Tuple, Optional, Any, List
from dataclasses import dataclass
from collections import deque
import os

@dataclass
class IMUSpec:
    """IMU specification data class"""
    sampling_rate_hz: float
    accelerometer_range: str
    gyroscope_range: str
    orientation_format: str
    data_format: str
    noise_density: str

@dataclass
class IMUReading:
    """IMU reading data class"""
    timestamp: float
    quaternion: Tuple[float, float, float, float]  # w, x, y, z
    gyroscope: Tuple[float, float, float]          # rad/s
    accelerometer: Tuple[float, float, float]      # m/s^2
    temperature: Optional[float] = None            # deg C (if available)

class IMUConfig:
    """IMU configuration and management class"""

    def __init__(self, config_input = None):
        """
        Initialize IMU configuration

        Args:
            config_input: Configuration file path (str) or configuration dictionary (dict)
        """
        self.logger = logging.getLogger(__name__)

        # Load configuration
        if isinstance(config_input, dict):
            self.config = config_input
        else:
            self.config = self._load_config(config_input)

        # IMU connection state
        self.is_initialized = False
        self.connection_active = False

        # unitree_sdk2py related
        self.channel_factory = None
        self.lowstate_subscriber = None
        self.latest_reading = None

        # Data buffer
        self.data_buffer = deque(maxlen=self.config.get("imu_config", {}).get("data_buffer_size", 1000))
        self.buffer_lock = threading.Lock()

        # IMU specifications
        self.target_spec = IMUSpec(
            sampling_rate_hz=self.config.get("imu_config", {}).get("sampling_rate_hz", 100),
            accelerometer_range="+/-16g",
            gyroscope_range="+/-2000 deg/s",
            orientation_format="quaternion",
            data_format="float32",
            noise_density="typical robotics IMU"
        )

        self.actual_spec = None

    def _load_config(self, config_path: Optional[str]) -> Dict[str, Any]:
        """Load configuration file"""
        if config_path and os.path.exists(config_path):
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    return json.load(f)
            except Exception as e:
                self.logger.error(f"Configuration file loading failed: {e}")

        # Return default configuration
        return {
            "imu_config": {
                "sampling_rate_hz": 100,
                "timeout_seconds": 10,
                "network_interface": "eth0",
                "data_buffer_size": 1000
            }
        }

    def initialize_imu(self, method: str = "unitree_sdk2py") -> bool:
        """
        Initialize IMU connection

        Args:
            method: Initialization method ("unitree_sdk2py", "simulation")

        Returns:
            bool: Whether initialization was successful
        """
        try:
            if method == "unitree_sdk2py":
                return self._initialize_with_unitree_sdk()
            elif method == "simulation":
                return self._initialize_simulation_mode()
            else:
                self.logger.error(f"Unsupported initialization method: {method}")
                return False

        except Exception as e:
            self.logger.error(f"IMU initialization failed: {e}")
            # If unitree_sdk2py fails, try simulation mode
            if method == "unitree_sdk2py":
                self.logger.warning("unitree_sdk2py initialization failed, trying simulation mode...")
                return self._initialize_simulation_mode()
            return False

    def _initialize_with_unitree_sdk(self) -> bool:
        """Initialize IMU using unitree_sdk2py"""
        try:
            # Import unitree_sdk2py modules
            from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

            self.logger.info("unitree_sdk2py module import successful")

            # Initialize channel factory
            network_interface = self.config.get("imu_config", {}).get("network_interface", "eth0")
            self.logger.info(f"Initializing DDS channel factory, network interface: {network_interface}")

            ChannelFactoryInitialize(0, network_interface)
            self.logger.info("DDS channel factory initialization successful")

            # Create LowState subscriber
            self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
            self.lowstate_subscriber.Init(self._lowstate_callback, 10)

            self.logger.info("LowState subscriber created successfully")

            # Wait for first data reception
            timeout = self.config.get("imu_config", {}).get("timeout_seconds", 10)
            start_time = time.time()

            while not self.latest_reading and (time.time() - start_time) < timeout:
                time.sleep(0.1)

            if self.latest_reading:
                self.is_initialized = True
                self.connection_active = True

                # Create actual specifications
                self.actual_spec = IMUSpec(
                    sampling_rate_hz=self._estimate_sampling_rate(),
                    accelerometer_range="+/-16g (estimated)",
                    gyroscope_range="+/-2000 deg/s (estimated)",
                    orientation_format="quaternion",
                    data_format="float32",
                    noise_density="unitree_sdk2py"
                )

                self.logger.info("IMU initialization successful")
                return True
            else:
                self.logger.error("IMU initialization timed out, no data received")
                return False

        except ImportError as e:
            self.logger.error(f"unitree_sdk2py module import failed: {e}")
            return False
        except Exception as e:
            self.logger.error(f"IMU initialization exception: {e}")
            return False

    def _lowstate_callback(self, msg):
        """LowState message callback function"""
        try:
            timestamp = time.time()

            # Extract IMU data
            reading = IMUReading(
                timestamp=timestamp,
                quaternion=(
                    msg.imu_state.quaternion[0],  # w
                    msg.imu_state.quaternion[1],  # x
                    msg.imu_state.quaternion[2],  # y
                    msg.imu_state.quaternion[3]   # z
                ),
                gyroscope=(
                    msg.imu_state.gyroscope[0],   # x (rad/s)
                    msg.imu_state.gyroscope[1],   # y (rad/s)
                    msg.imu_state.gyroscope[2]    # z (rad/s)
                ),
                accelerometer=(
                    msg.imu_state.accelerometer[0],  # x (m/s^2)
                    msg.imu_state.accelerometer[1],  # y (m/s^2)
                    msg.imu_state.accelerometer[2]   # z (m/s^2)
                )
            )

            # Update latest reading
            self.latest_reading = reading

            # Add to buffer
            with self.buffer_lock:
                self.data_buffer.append(reading)

        except Exception as e:
            self.logger.error(f"IMU data processing error: {e}")

    def get_latest_reading(self) -> Optional[IMUReading]:
        """Get the latest IMU reading"""
        return self.latest_reading

    def get_buffered_data(self, num_samples: int = None) -> List[IMUReading]:
        """
        Get buffered IMU data

        Args:
            num_samples: Number of samples to retrieve, None means all

        Returns:
            List[IMUReading]: List of IMU readings
        """
        with self.buffer_lock:
            if num_samples is None:
                return list(self.data_buffer)
            else:
                return list(self.data_buffer)[-num_samples:]

    def clear_buffer(self):
        """Clear data buffer"""
        with self.buffer_lock:
            self.data_buffer.clear()

    def _estimate_sampling_rate(self) -> float:
        """Estimate actual sampling rate"""
        try:
            # Collect a short period of data to estimate sampling rate
            initial_count = len(self.data_buffer)
            time.sleep(2.0)  # Wait 2 seconds
            final_count = len(self.data_buffer)

            if final_count > initial_count:
                estimated_rate = (final_count - initial_count) / 2.0
                return estimated_rate
            else:
                return self.target_spec.sampling_rate_hz

        except Exception:
            return self.target_spec.sampling_rate_hz

    def get_imu_properties(self) -> Dict[str, Any]:
        """Get IMU property information"""
        if not self.is_initialized:
            return {"status": "not_initialized"}

        latest = self.latest_reading
        if not latest:
            return {"status": "no_data"}

        return {
            "status": "active",
            "connection_active": self.connection_active,
            "latest_timestamp": latest.timestamp,
            "buffer_size": len(self.data_buffer),
            "target_spec": self.target_spec.__dict__,
            "actual_spec": self.actual_spec.__dict__ if self.actual_spec else None,
            "latest_reading": {
                "quaternion": latest.quaternion,
                "gyroscope": latest.gyroscope,
                "accelerometer": latest.accelerometer,
                "magnitude_accel": np.linalg.norm(latest.accelerometer),
                "magnitude_gyro": np.linalg.norm(latest.gyroscope)
            }
        }

    def check_data_integrity(self) -> Dict[str, Any]:
        """Check data integrity"""
        if not self.is_initialized or len(self.data_buffer) < 10:
            return {"status": "insufficient_data"}

        data = self.get_buffered_data()
        timestamps = [reading.timestamp for reading in data]

        # Calculate data rate
        if len(timestamps) > 1:
            time_diffs = np.diff(timestamps)
            avg_interval = np.mean(time_diffs)
            actual_rate = 1.0 / avg_interval if avg_interval > 0 else 0

            # Check for dropped frames
            expected_interval = 1.0 / self.target_spec.sampling_rate_hz
            dropout_count = sum(1 for diff in time_diffs if diff > expected_interval * 1.5)
            dropout_rate = dropout_count / len(time_diffs)
        else:
            actual_rate = 0
            dropout_rate = 0

        # Check data validity
        valid_readings = 0
        for reading in data:
            if (all(np.isfinite(reading.quaternion)) and
                all(np.isfinite(reading.gyroscope)) and
                all(np.isfinite(reading.accelerometer))):
                valid_readings += 1

        data_validity = valid_readings / len(data) if data else 0

        return {
            "status": "analyzed",
            "sample_count": len(data),
            "actual_sampling_rate": actual_rate,
            "target_sampling_rate": self.target_spec.sampling_rate_hz,
            "dropout_rate": dropout_rate,
            "data_validity": data_validity,
            "time_span_seconds": timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 0
        }

    def quaternion_to_euler(self, q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
        """
        Convert quaternion to Euler angles (roll, pitch, yaw)

        Args:
            q: Quaternion (w, x, y, z)

        Returns:
            Tuple[float, float, float]: Euler angles (roll, pitch, yaw) in radians
        """
        w, x, y, z = q

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def release(self):
        """Release IMU resources"""
        try:
            self.connection_active = False
            self.is_initialized = False

            if self.lowstate_subscriber:
                # unitree_sdk2py subscribers typically clean up automatically
                self.lowstate_subscriber = None

            self.logger.info("IMU resources released")

        except Exception as e:
            self.logger.error(f"Error releasing IMU resources: {e}")
