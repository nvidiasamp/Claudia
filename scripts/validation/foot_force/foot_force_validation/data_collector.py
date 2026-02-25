#!/usr/bin/env python3
# scripts/validation/foot_force/foot_force_validation/data_collector.py
# Generated: 2025-06-27 14:08:45 CST
# Purpose: Unitree Go2 foot force sensor real-time data collection and processing

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

from foot_force_config import FootForceConfig, FootForceReading

@dataclass
class FootForceCollectionMetrics:
    """Foot force data collection metrics"""
    start_time: float = 0.0
    end_time: float = 0.0
    total_samples: int = 0
    valid_samples: int = 0
    dropout_count: int = 0
    avg_sampling_rate: float = 0.0
    data_rate_std: float = 0.0
    collection_duration: float = 0.0

    # Foot force quality metrics
    force_stats: Dict[str, Dict[str, float]] = field(default_factory=dict)  # Force statistics per foot
    contact_rate: Dict[str, float] = field(default_factory=dict)  # Contact rate per foot
    total_force_stats: Dict[str, float] = field(default_factory=dict)  # Total force statistics
    cop_stats: Dict[str, float] = field(default_factory=dict)  # Center of pressure statistics

    # Real-time statistics
    real_time_fps: List[float] = field(default_factory=list)
    latency_history: List[float] = field(default_factory=list)

    # Data quality assessment
    data_quality_score: float = 0.0
    validation_results: Dict[str, float] = field(default_factory=dict)

@dataclass
class FootForceData:
    """Foot force data structure (for storage)"""
    timestamp: float
    foot_forces: List[List[float]]  # 4 feet, 3D force each [[Fx,Fy,Fz], ...]
    contact_states: List[bool]  # Contact state for 4 feet
    total_force: float
    center_of_pressure: List[float]  # [x, y]

    # Additional computed fields
    force_magnitude: List[float] = field(default_factory=list)  # Force magnitude per foot
    stability_index: float = 0.0  # Stability index
    force_balance: float = 0.0  # Force balance index

class FootForceDataCollector:
    """Foot force sensor data collector class"""

    def __init__(self, config: Dict, foot_force_config: Optional[FootForceConfig] = None):
        """
        Initialize foot force data collector

        Args:
            config: Configuration dictionary
            foot_force_config: FootForceConfig instance
        """
        self.config = config
        self.logger = logging.getLogger(__name__)
        self.foot_force_config = foot_force_config

        # Data storage
        self.raw_data: List[FootForceData] = []
        self.is_collecting = False
        self.collection_thread: Optional[threading.Thread] = None

        # Callback functions
        self.data_callbacks: List[Callable[[FootForceData], None]] = []

        # Data quality monitoring
        self.quality_window_size = 100  # Data quality assessment window size
        self.recent_validation_scores = deque(maxlen=self.quality_window_size)

        # Statistical data
        self.contact_event_counter = {i: 0 for i in range(4)}  # Contact event counter
        self.force_peak_detector = {i: [] for i in range(4)}  # Force peak detection

        self.logger.info("Foot force data collector initialization complete")

    def add_data_callback(self, callback: Callable[[FootForceData], None]):
        """Add data callback function"""
        self.data_callbacks.append(callback)

    def _extract_foot_force_data_from_reading(self, reading: FootForceReading) -> FootForceData:
        """Extract and compute data from FootForceReading object"""

        # Calculate force magnitude for each foot
        force_magnitude = [np.linalg.norm(force) for force in reading.foot_forces]

        # Calculate stability index (based on standard deviation of force distribution)
        vertical_forces = [abs(force[2]) for force in reading.foot_forces]
        if sum(vertical_forces) > 0:
            stability_index = 1.0 - (np.std(vertical_forces) / np.mean(vertical_forces))
            stability_index = max(0.0, min(1.0, stability_index))  # Clamp to 0-1
        else:
            stability_index = 0.0

        # Calculate force balance index (front-rear and left-right balance)
        front_force = vertical_forces[0] + vertical_forces[1]  # Front Left + Front Right
        rear_force = vertical_forces[2] + vertical_forces[3]   # Rear Left + Rear Right
        left_force = vertical_forces[0] + vertical_forces[2]   # Front Left + Rear Left
        right_force = vertical_forces[1] + vertical_forces[3]  # Front Right + Rear Right

        total_vertical = sum(vertical_forces)
        if total_vertical > 0:
            front_rear_balance = 1.0 - abs(front_force - rear_force) / total_vertical
            left_right_balance = 1.0 - abs(left_force - right_force) / total_vertical
            force_balance = (front_rear_balance + left_right_balance) / 2.0
        else:
            force_balance = 0.0

        return FootForceData(
            timestamp=reading.timestamp,
            foot_forces=[list(force) for force in reading.foot_forces],
            contact_states=list(reading.contact_states),
            total_force=reading.total_force,
            center_of_pressure=list(reading.center_of_pressure),
            force_magnitude=force_magnitude,
            stability_index=stability_index,
            force_balance=force_balance
        )

    def _collection_worker(self):
        """Data collection worker thread"""
        self.logger.info("Starting foot force data collection")

        sampling_rate = self.config.get('foot_force_config', {}).get('sampling_rate_hz', 500)
        sleep_interval = 1.0 / sampling_rate

        last_contact_states = [False] * 4  # Record previous contact states

        while self.is_collecting:
            try:
                if self.foot_force_config:
                    # Get latest reading from FootForceConfig
                    reading = self.foot_force_config.get_latest_reading()
                    if reading:
                        # Validate data quality
                        validation_results = self.foot_force_config.validate_force_data(reading)

                        # Calculate validation score
                        validation_score = sum(validation_results.values()) / len(validation_results)
                        self.recent_validation_scores.append(validation_score)

                        if validation_score >= 0.75:  # Only store high quality data
                            foot_force_data = self._extract_foot_force_data_from_reading(reading)

                            # Detect contact events
                            self._detect_contact_events(foot_force_data.contact_states, last_contact_states)
                            last_contact_states = foot_force_data.contact_states.copy()

                            # Record force peaks
                            self._record_force_peaks(foot_force_data.force_magnitude)

                            # Store data
                            self.raw_data.append(foot_force_data)

                            # Call callback functions
                            for callback in self.data_callbacks:
                                try:
                                    callback(foot_force_data)
                                except Exception as e:
                                    self.logger.error(f"Data callback error: {e}")
                        else:
                            self.logger.debug(f"Insufficient data quality, skipping storage. Score: {validation_score:.2f}")
                else:
                    self.logger.warning("No FootForceConfig instance, unable to collect data")

                # Control sampling rate
                time.sleep(sleep_interval)

            except Exception as e:
                self.logger.error(f"Data collection error: {e}")
                time.sleep(0.1)

        self.logger.info("Foot force data collection ended")

    def _detect_contact_events(self, current_contacts: List[bool], last_contacts: List[bool]):
        """Detect contact events (touchdown and liftoff)"""
        for i in range(4):
            if current_contacts[i] and not last_contacts[i]:
                # Touchdown event
                self.contact_event_counter[i] += 1
                self.logger.debug(f"Foot {i} touchdown event")

    def _record_force_peaks(self, force_magnitudes: List[float]):
        """Record force peaks"""
        for i, magnitude in enumerate(force_magnitudes):
            # Simple peak detection: keep recent peaks
            self.force_peak_detector[i].append(magnitude)
            if len(self.force_peak_detector[i]) > 50:  # Keep last 50 samples
                self.force_peak_detector[i].pop(0)

    def start_collection(self, duration_seconds: Optional[float] = None) -> bool:
        """
        Start data collection

        Args:
            duration_seconds: Optional collection duration (seconds)
        """
        try:
            if not self.foot_force_config:
                self.logger.error("No FootForceConfig instance, unable to start data collection")
                return False

            if not self.foot_force_config.is_initialized:
                self.logger.error("FootForceConfig not initialized, unable to start data collection")
                return False

            # Clear data
            self.raw_data.clear()
            self.recent_validation_scores.clear()
            self.contact_event_counter = {i: 0 for i in range(4)}
            self.force_peak_detector = {i: [] for i in range(4)}

            # Start collection thread
            self.is_collecting = True
            self.collection_thread = threading.Thread(target=self._collection_worker)
            self.collection_thread.daemon = True
            self.collection_thread.start()

            if duration_seconds is not None:
                self.logger.info(f"Foot force data collection started, duration: {duration_seconds}s")

                # Start auto-stop timer thread
                def auto_stop():
                    time.sleep(duration_seconds)
                    if self.is_collecting:
                        self.stop_collection()
                        self.logger.info(f"Auto-stopped data collection after {duration_seconds} seconds")

                auto_stop_thread = threading.Thread(target=auto_stop)
                auto_stop_thread.daemon = True
                auto_stop_thread.start()
            else:
                self.logger.info("Foot force data collection started, manual stop mode")

            return True

        except Exception as e:
            self.logger.error(f"Failed to start data collection: {e}")
            return False

    def stop_collection(self) -> FootForceCollectionMetrics:
        """Stop data collection and return collection metrics"""
        try:
            # Stop collection thread
            self.is_collecting = False
            if self.collection_thread:
                self.collection_thread.join(timeout=5.0)

            # Calculate collection metrics
            metrics = self._calculate_collection_metrics()

            self.logger.info("Foot force data collection stopped")
            return metrics

        except Exception as e:
            self.logger.error(f"Failed to stop data collection: {e}")
            return FootForceCollectionMetrics()

    def _calculate_collection_metrics(self) -> FootForceCollectionMetrics:
        """Calculate collection metrics"""
        metrics = FootForceCollectionMetrics()

        if self.raw_data:
            metrics.start_time = self.raw_data[0].timestamp
            metrics.end_time = self.raw_data[-1].timestamp
            metrics.total_samples = len(self.raw_data)
            metrics.valid_samples = len(self.raw_data)
            metrics.collection_duration = metrics.end_time - metrics.start_time

            if metrics.collection_duration > 0:
                metrics.avg_sampling_rate = metrics.total_samples / metrics.collection_duration

            # Calculate foot force statistics
            try:
                # Prepare data arrays
                all_forces = np.array([data.foot_forces for data in self.raw_data])  # (samples, 4_feet, 3_axes)
                all_contacts = np.array([data.contact_states for data in self.raw_data])  # (samples, 4_feet)
                total_forces = np.array([data.total_force for data in self.raw_data])
                cops = np.array([data.center_of_pressure for data in self.raw_data])  # (samples, 2)

                # Force statistics per foot
                for i in range(4):
                    foot_label = FootForceConfig.FOOT_LABELS[i]
                    foot_forces = all_forces[:, i, :]  # (samples, 3_axes)

                    metrics.force_stats[foot_label] = {
                        'mean_fx': float(np.mean(foot_forces[:, 0])),
                        'mean_fy': float(np.mean(foot_forces[:, 1])),
                        'mean_fz': float(np.mean(foot_forces[:, 2])),
                        'std_fx': float(np.std(foot_forces[:, 0])),
                        'std_fy': float(np.std(foot_forces[:, 1])),
                        'std_fz': float(np.std(foot_forces[:, 2])),
                        'max_magnitude': float(np.max([np.linalg.norm(force) for force in foot_forces])),
                        'contact_events': self.contact_event_counter[i]
                    }

                    # Contact rate
                    metrics.contact_rate[foot_label] = float(np.mean(all_contacts[:, i]))

                # Total force statistics
                metrics.total_force_stats = {
                    'mean': float(np.mean(total_forces)),
                    'std': float(np.std(total_forces)),
                    'min': float(np.min(total_forces)),
                    'max': float(np.max(total_forces))
                }

                # Center of pressure statistics
                metrics.cop_stats = {
                    'mean_x': float(np.mean(cops[:, 0])),
                    'mean_y': float(np.mean(cops[:, 1])),
                    'std_x': float(np.std(cops[:, 0])),
                    'std_y': float(np.std(cops[:, 1])),
                    'range_x': float(np.max(cops[:, 0]) - np.min(cops[:, 0])),
                    'range_y': float(np.max(cops[:, 1]) - np.min(cops[:, 1]))
                }

                # Data quality score
                if self.recent_validation_scores:
                    metrics.data_quality_score = float(np.mean(self.recent_validation_scores))
                    metrics.validation_results = {
                        'avg_quality': metrics.data_quality_score,
                        'min_quality': float(min(self.recent_validation_scores)),
                        'quality_std': float(np.std(self.recent_validation_scores))
                    }

            except Exception as e:
                self.logger.error(f"Failed to calculate foot force statistics: {e}")

        return metrics

    def get_collected_data(self) -> List[FootForceReading]:
        """Get collected data and convert to FootForceReading format"""
        readings = []
        for data in self.raw_data:
            reading = FootForceReading(
                timestamp=data.timestamp,
                foot_forces=tuple(tuple(force) for force in data.foot_forces),
                contact_states=tuple(data.contact_states),
                total_force=data.total_force,
                center_of_pressure=tuple(data.center_of_pressure)
            )
            readings.append(reading)

        return readings

    def get_real_time_metrics(self) -> Dict[str, Any]:
        """Get real-time collection metrics"""
        if not self.raw_data:
            return {
                'current_fps': 0.0,
                'total_samples': 0,
                'data_quality': 0.0,
                'contact_summary': [False] * 4
            }

        # Calculate current sampling rate
        recent_count = min(100, len(self.raw_data))
        if recent_count > 1:
            recent_data = self.raw_data[-recent_count:]
            time_span = recent_data[-1].timestamp - recent_data[0].timestamp
            current_fps = (recent_count - 1) / time_span if time_span > 0 else 0
        else:
            current_fps = 0

        # Latest contact state
        latest_contacts = self.raw_data[-1].contact_states if self.raw_data else [False] * 4

        # Latest data quality
        current_quality = self.recent_validation_scores[-1] if self.recent_validation_scores else 0.0

        return {
            'current_fps': current_fps,
            'total_samples': len(self.raw_data),
            'data_quality': current_quality,
            'contact_summary': latest_contacts,
            'last_timestamp': self.raw_data[-1].timestamp if self.raw_data else 0,
            'total_force': self.raw_data[-1].total_force if self.raw_data else 0.0,
            'cop': self.raw_data[-1].center_of_pressure if self.raw_data else [0.0, 0.0],
            'stability_index': self.raw_data[-1].stability_index if self.raw_data else 0.0
        }

    def get_data(self) -> List[FootForceData]:
        """Get collected data"""
        return self.raw_data.copy()

    def get_latest_data(self) -> Optional[FootForceData]:
        """Get latest foot force data"""
        if self.raw_data:
            return self.raw_data[-1]
        return None

    def save_data(self, filepath: str) -> bool:
        """Save data to file"""
        try:
            data_dict = [asdict(data) for data in self.raw_data]

            # Add metadata
            metadata = {
                'total_samples': len(self.raw_data),
                'collection_config': self.config,
                'timestamp': time.time(),
                'duration_seconds': (self.raw_data[-1].timestamp - self.raw_data[0].timestamp) if len(self.raw_data) > 1 else 0,
                'contact_events': self.contact_event_counter,
                'data_quality_avg': float(np.mean(self.recent_validation_scores)) if self.recent_validation_scores else 0.0
            }

            with open(filepath, 'w') as f:
                json.dump({
                    'metadata': metadata,
                    'data': data_dict
                }, f, indent=2)

            self.logger.info(f"Foot force data saved to: {filepath}")
            return True

        except Exception as e:
            self.logger.error(f"Failed to save data: {e}")
            return False

    def save_data_csv(self, filepath: str) -> bool:
        """Save data to CSV file"""
        try:
            with open(filepath, 'w', newline='') as csvfile:
                fieldnames = [
                    'timestamp', 'total_force', 'cop_x', 'cop_y', 'stability_index', 'force_balance'
                ]

                # Add fields for each foot
                for i, label in enumerate(FootForceConfig.FOOT_LABELS):
                    fieldnames.extend([
                        f'{label}_fx', f'{label}_fy', f'{label}_fz',
                        f'{label}_contact', f'{label}_magnitude'
                    ])

                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()

                for data in self.raw_data:
                    row = {
                        'timestamp': data.timestamp,
                        'total_force': data.total_force,
                        'cop_x': data.center_of_pressure[0],
                        'cop_y': data.center_of_pressure[1],
                        'stability_index': data.stability_index,
                        'force_balance': data.force_balance
                    }

                    # Add data for each foot
                    for i, label in enumerate(FootForceConfig.FOOT_LABELS):
                        row[f'{label}_fx'] = data.foot_forces[i][0]
                        row[f'{label}_fy'] = data.foot_forces[i][1]
                        row[f'{label}_fz'] = data.foot_forces[i][2]
                        row[f'{label}_contact'] = data.contact_states[i]
                        row[f'{label}_magnitude'] = data.force_magnitude[i]

                    writer.writerow(row)

            self.logger.info(f"Foot force data saved to CSV: {filepath}")
            return True

        except Exception as e:
            self.logger.error(f"Failed to save CSV data: {e}")
            return False

    def get_statistics(self) -> Dict:
        """Get data statistics"""
        if not self.raw_data:
            return {}

        # Prepare data arrays
        all_forces = np.array([data.foot_forces for data in self.raw_data])
        total_forces = np.array([data.total_force for data in self.raw_data])
        cops = np.array([data.center_of_pressure for data in self.raw_data])
        stability_indices = np.array([data.stability_index for data in self.raw_data])
        force_balances = np.array([data.force_balance for data in self.raw_data])

        stats = {
            'total_samples': len(self.raw_data),
            'duration_seconds': (self.raw_data[-1].timestamp - self.raw_data[0].timestamp) if len(self.raw_data) > 1 else 0,
            'avg_sampling_rate': len(self.raw_data) / ((self.raw_data[-1].timestamp - self.raw_data[0].timestamp) if len(self.raw_data) > 1 else 1),

            'total_force_stats': {
                'mean': float(np.mean(total_forces)),
                'std': float(np.std(total_forces)),
                'min': float(np.min(total_forces)),
                'max': float(np.max(total_forces))
            },

            'cop_stats': {
                'mean_x': float(np.mean(cops[:, 0])),
                'mean_y': float(np.mean(cops[:, 1])),
                'std_x': float(np.std(cops[:, 0])),
                'std_y': float(np.std(cops[:, 1]))
            },

            'stability_stats': {
                'mean': float(np.mean(stability_indices)),
                'std': float(np.std(stability_indices)),
                'min': float(np.min(stability_indices)),
                'max': float(np.max(stability_indices))
            },

            'balance_stats': {
                'mean': float(np.mean(force_balances)),
                'std': float(np.std(force_balances)),
                'min': float(np.min(force_balances)),
                'max': float(np.max(force_balances))
            },

            'contact_events': self.contact_event_counter,
            'data_quality': {
                'avg_score': float(np.mean(self.recent_validation_scores)) if self.recent_validation_scores else 0.0,
                'min_score': float(min(self.recent_validation_scores)) if self.recent_validation_scores else 0.0,
                'score_std': float(np.std(self.recent_validation_scores)) if self.recent_validation_scores else 0.0
            }
        }

        # Detailed statistics per foot
        stats['foot_stats'] = {}
        for i in range(4):
            foot_label = FootForceConfig.FOOT_LABELS[i]
            foot_forces = all_forces[:, i, :]

            stats['foot_stats'][foot_label] = {
                'force_mean': foot_forces.mean(axis=0).tolist(),
                'force_std': foot_forces.std(axis=0).tolist(),
                'force_min': foot_forces.min(axis=0).tolist(),
                'force_max': foot_forces.max(axis=0).tolist(),
                'contact_rate': float(np.mean([data.contact_states[i] for data in self.raw_data])),
                'contact_events': self.contact_event_counter[i],
                'peak_force': float(max(self.force_peak_detector[i])) if self.force_peak_detector[i] else 0.0
            }

        return stats
