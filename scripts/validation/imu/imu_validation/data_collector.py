#!/usr/bin/env python3
# scripts/validation/imu/imu_validation/data_collector.py
# Generated: 2025-06-27 11:54:45 CST
# Purpose: Unitree Go2 IMU real-time data collection and processing

import time
import threading
import statistics
import numpy as np
import logging
from typing import Dict, List, Tuple, Any, Optional, Callable
from dataclasses import dataclass, field, asdict
from collections import deque
import json
import csv
from pathlib import Path
from datetime import datetime

# Unitree SDK2 imports - using the correct import path
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

from imu_config import IMUConfig, IMUReading

@dataclass
class CollectionMetrics:
    """Data collection metrics"""
    start_time: float = 0.0
    end_time: float = 0.0
    total_samples: int = 0
    valid_samples: int = 0
    dropout_count: int = 0
    avg_sampling_rate: float = 0.0
    data_rate_std: float = 0.0
    collection_duration: float = 0.0

    # Data quality metrics
    accelerometer_stats: Dict[str, float] = field(default_factory=dict)
    gyroscope_stats: Dict[str, float] = field(default_factory=dict)
    quaternion_stats: Dict[str, float] = field(default_factory=dict)

    # Real-time statistics
    real_time_fps: List[float] = field(default_factory=list)
    latency_history: List[float] = field(default_factory=list)

@dataclass
class IMUData:
    """IMU data structure"""
    timestamp: float
    quaternion: List[float]  # [w, x, y, z]
    gyroscope: List[float]   # [x, y, z] rad/s
    accelerometer: List[float]  # [x, y, z] m/s^2
    temperature: int

class IMUDataCollector:
    """IMU data collector class"""

    def __init__(self, config: Dict, imu_config=None):
        """
        Initialize IMU data collector

        Args:
            config: Configuration dictionary
            imu_config: IMUConfig instance, uses its connection if provided
        """
        self.config = config
        self.logger = logging.getLogger(__name__)
        self.imu_config = imu_config

        # Data storage
        self.raw_data: List[IMUData] = []
        self.is_collecting = False
        self.collection_thread: Optional[threading.Thread] = None

        # Callback functions
        self.data_callbacks: List[Callable[[IMUData], None]] = []

        self.logger.info("IMU data collector initialization complete")

    def add_data_callback(self, callback: Callable[[IMUData], None]):
        """Add data callback function"""
        self.data_callbacks.append(callback)

    def _extract_imu_data_from_reading(self, reading) -> IMUData:
        """Extract data from IMUReading object"""
        return IMUData(
            timestamp=reading.timestamp,
            quaternion=list(reading.quaternion),
            gyroscope=list(reading.gyroscope),
            accelerometer=list(reading.accelerometer),
            temperature=getattr(reading, 'temperature', 0)
        )

    def _collection_worker(self):
        """Data collection worker thread - uses IMUConfig data"""
        self.logger.info("Starting IMU data collection")

        sampling_rate = self.config.get('imu_config', {}).get('sampling_rate_hz', 100)
        sleep_interval = 1.0 / sampling_rate

        while self.is_collecting:
            try:
                if self.imu_config:
                    # Get latest reading from IMUConfig
                    reading = self.imu_config.get_latest_reading()
                    if reading:
                        imu_data = self._extract_imu_data_from_reading(reading)

                        # Store data
                        self.raw_data.append(imu_data)

                        # Call callback functions
                        for callback in self.data_callbacks:
                            try:
                                callback(imu_data)
                            except Exception as e:
                                self.logger.error(f"Data callback error: {e}")
                else:
                    self.logger.warning("No IMUConfig instance, cannot collect data")

                # Control sampling frequency
                time.sleep(sleep_interval)

            except Exception as e:
                self.logger.error(f"Data collection error: {e}")
                time.sleep(0.1)

        self.logger.info("IMU data collection ended")

    def start_collection(self, duration_seconds: Optional[float] = None) -> bool:
        """
        Start data collection

        Args:
            duration_seconds: Optional collection duration (seconds); if not specified, collection continues until manually stopped
        """
        try:
            if not self.imu_config:
                self.logger.error("No IMUConfig instance, cannot start data collection")
                return False

            if not self.imu_config.is_initialized:
                self.logger.error("IMUConfig is not initialized, cannot start data collection")
                return False

            # Clear data
            self.raw_data.clear()

            # Start collection thread
            self.is_collecting = True
            self.collection_thread = threading.Thread(target=self._collection_worker)
            self.collection_thread.start()

            if duration_seconds is not None:
                self.logger.info(f"IMU data collection started, duration: {duration_seconds} seconds")

                # Start auto-stop timer thread
                def auto_stop():
                    time.sleep(duration_seconds)
                    if self.is_collecting:
                        self.stop_collection()
                        self.logger.info(f"Automatically stopped data collection after {duration_seconds} seconds")

                auto_stop_thread = threading.Thread(target=auto_stop)
                auto_stop_thread.daemon = True
                auto_stop_thread.start()
            else:
                self.logger.info("IMU data collection started, manual stop mode")

            return True

        except Exception as e:
            self.logger.error(f"Failed to start data collection: {e}")
            return False

    def stop_collection(self) -> CollectionMetrics:
        """Stop data collection and return collection metrics"""
        try:
            # Stop collection thread
            self.is_collecting = False
            if self.collection_thread:
                self.collection_thread.join(timeout=5.0)

            # Calculate collection metrics
            metrics = self._calculate_collection_metrics()

            self.logger.info("IMU data collection stopped")
            return metrics

        except Exception as e:
            self.logger.error(f"Failed to stop data collection: {e}")
            # Return empty metrics object
            return CollectionMetrics()

    def _calculate_collection_metrics(self) -> CollectionMetrics:
        """Calculate collection metrics"""
        metrics = CollectionMetrics()

        if self.raw_data:
            metrics.start_time = self.raw_data[0].timestamp
            metrics.end_time = self.raw_data[-1].timestamp
            metrics.total_samples = len(self.raw_data)
            metrics.valid_samples = len(self.raw_data)  # Assume all data is valid
            metrics.collection_duration = metrics.end_time - metrics.start_time

            if metrics.collection_duration > 0:
                metrics.avg_sampling_rate = metrics.total_samples / metrics.collection_duration

            # Simplified data quality statistics
            try:
                accelerometers = np.array([data.accelerometer for data in self.raw_data])
                gyroscopes = np.array([data.gyroscope for data in self.raw_data])
                quaternions = np.array([data.quaternion for data in self.raw_data])

                metrics.accelerometer_stats = {
                    'mean': accelerometers.mean(axis=0).tolist(),
                    'std': accelerometers.std(axis=0).tolist()
                }
                metrics.gyroscope_stats = {
                    'mean': gyroscopes.mean(axis=0).tolist(),
                    'std': gyroscopes.std(axis=0).tolist()
                }
                metrics.quaternion_stats = {
                    'mean': quaternions.mean(axis=0).tolist(),
                    'std': quaternions.std(axis=0).tolist()
                }
            except Exception as e:
                self.logger.error(f"Failed to calculate data quality statistics: {e}")

        return metrics

    def get_collected_data(self) -> List[IMUReading]:
        """Get collected data and convert to IMUReading format"""
        from imu_config import IMUReading

        readings = []
        for data in self.raw_data:
            # Ensure correct data length
            accel = data.accelerometer[:3] if len(data.accelerometer) >= 3 else data.accelerometer + [0.0] * (3 - len(data.accelerometer))
            gyro = data.gyroscope[:3] if len(data.gyroscope) >= 3 else data.gyroscope + [0.0] * (3 - len(data.gyroscope))
            quat = data.quaternion[:4] if len(data.quaternion) >= 4 else data.quaternion + [0.0] * (4 - len(data.quaternion))

            reading = IMUReading(
                timestamp=data.timestamp,
                accelerometer=(accel[0], accel[1], accel[2]),
                gyroscope=(gyro[0], gyro[1], gyro[2]),
                quaternion=(quat[0], quat[1], quat[2], quat[3]),
                temperature=data.temperature
            )
            readings.append(reading)

        return readings

    def get_real_time_metrics(self) -> Dict[str, float]:
        """Get real-time collection metrics"""
        if not self.raw_data:
            return {'current_fps': 0.0, 'total_samples': 0}

        # Calculate current sampling rate (based on recent samples)
        recent_count = min(100, len(self.raw_data))
        if recent_count > 1:
            recent_data = self.raw_data[-recent_count:]
            time_span = recent_data[-1].timestamp - recent_data[0].timestamp
            current_fps = (recent_count - 1) / time_span if time_span > 0 else 0
        else:
            current_fps = 0

        return {
            'current_fps': current_fps,
            'total_samples': len(self.raw_data),
            'last_timestamp': self.raw_data[-1].timestamp if self.raw_data else 0
        }

    def get_data(self) -> List[IMUData]:
        """Get collected data"""
        return self.raw_data.copy()

    def get_latest_data(self) -> Optional[IMUData]:
        """Get the latest IMU data"""
        if self.raw_data:
            return self.raw_data[-1]
        return None

    def save_data(self, filepath: str) -> bool:
        """Save data to file"""
        try:
            data_dict = [asdict(data) for data in self.raw_data]

            with open(filepath, 'w') as f:
                json.dump({
                    'metadata': {
                        'total_samples': len(self.raw_data),
                        'collection_config': self.config,
                        'timestamp': time.time()
                    },
                    'data': data_dict
                }, f, indent=2)

            self.logger.info(f"Data saved to: {filepath}")
            return True

        except Exception as e:
            self.logger.error(f"Failed to save data: {e}")
            return False

    def get_statistics(self) -> Dict:
        """Get data statistics"""
        if not self.raw_data:
            return {}

        # Extract data arrays
        quaternions = np.array([data.quaternion for data in self.raw_data])
        gyroscopes = np.array([data.gyroscope for data in self.raw_data])
        accelerometers = np.array([data.accelerometer for data in self.raw_data])
        temperatures = np.array([data.temperature for data in self.raw_data])

        return {
            'total_samples': len(self.raw_data),
            'duration_seconds': (self.raw_data[-1].timestamp - self.raw_data[0].timestamp) if len(self.raw_data) > 1 else 0,
            'quaternion_stats': {
                'mean': quaternions.mean(axis=0).tolist(),
                'std': quaternions.std(axis=0).tolist(),
                'min': quaternions.min(axis=0).tolist(),
                'max': quaternions.max(axis=0).tolist()
            },
            'gyroscope_stats': {
                'mean': gyroscopes.mean(axis=0).tolist(),
                'std': gyroscopes.std(axis=0).tolist(),
                'min': gyroscopes.min(axis=0).tolist(),
                'max': gyroscopes.max(axis=0).tolist()
            },
            'accelerometer_stats': {
                'mean': accelerometers.mean(axis=0).tolist(),
                'std': accelerometers.std(axis=0).tolist(),
                'min': accelerometers.min(axis=0).tolist(),
                'max': accelerometers.max(axis=0).tolist()
            },
            'temperature_stats': {
                'mean': float(temperatures.mean()),
                'std': float(temperatures.std()),
                'min': int(temperatures.min()),
                'max': int(temperatures.max())
            }
        }
