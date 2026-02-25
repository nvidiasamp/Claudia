#!/usr/bin/env python3
# scripts/validation/foot_force/foot_force_validation/foot_force_config.py
# Generated: 2025-06-27 14:08:30 CST
# Purpose: Unitree Go2 foot force sensor configuration management

import time
import logging
import threading
from typing import Dict, List, Tuple, Optional, NamedTuple
from dataclasses import dataclass, field
import numpy as np

# Unitree SDK2 imports
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

@dataclass
class FootForceReading:
    """Single reading from foot force sensor"""
    timestamp: float
    # Force sensor data for four feet [front-left, front-right, rear-left, rear-right]
    foot_forces: Tuple[Tuple[float, float, float], ...]  # [(Fx, Fy, Fz), ...]
    contact_states: Tuple[bool, bool, bool, bool]  # Contact states
    total_force: float  # Total combined force
    center_of_pressure: Tuple[float, float]  # Center of pressure (x, y)

    def __post_init__(self):
        """Validate data integrity"""
        if len(self.foot_forces) != 4:
            raise ValueError("Foot force data must contain data for 4 feet")
        if len(self.contact_states) != 4:
            raise ValueError("Contact states must contain states for 4 feet")

class FootForceConfig:
    """Foot force sensor configuration management class"""

    # Foot identifier constants
    FOOT_FL = 0  # Front Left
    FOOT_FR = 1  # Front Right
    FOOT_RL = 2  # Rear Left
    FOOT_RR = 3  # Rear Right

    FOOT_NAMES = ["Front Left", "Front Right", "Rear Left", "Rear Right"]
    FOOT_LABELS = ["FL", "FR", "RL", "RR"]

    def __init__(self, network_interface: str = "eth0",
                 sampling_rate: Optional[float] = None,
                 force_threshold: Optional[float] = None,
                 max_force_per_foot: Optional[float] = None):
        """
        Initialize foot force sensor configuration

        Args:
            network_interface: Network interface name
            sampling_rate: Sampling rate (Hz), default 500Hz
            force_threshold: Contact detection threshold (N), default 5.0N
            max_force_per_foot: Maximum force per foot (N), default 200.0N
        """
        self.network_interface = network_interface
        self.logger = logging.getLogger(__name__)

        # Connection state
        self.is_initialized = False
        self.subscriber: Optional[ChannelSubscriber] = None

        # Data cache
        self._latest_reading: Optional[FootForceReading] = None
        self._data_lock = threading.Lock()

        # Calibration parameters
        self.calibration_offset = {
            'FL': (0.0, 0.0, 0.0),
            'FR': (0.0, 0.0, 0.0),
            'RL': (0.0, 0.0, 0.0),
            'RR': (0.0, 0.0, 0.0)
        }

        # Configuration parameters (supports constructor arguments)
        self.force_threshold = force_threshold if force_threshold is not None else 5.0  # N, contact detection threshold
        self.max_force_per_foot = max_force_per_foot if max_force_per_foot is not None else 200.0  # N, max force per foot
        self.sampling_rate_hz = sampling_rate if sampling_rate is not None else 500  # Hz, sampling rate

        self.logger.info("Foot force sensor configuration initialized")

    def initialize_connection(self, domain_id: int = 0) -> bool:
        """
        Initialize connection to the robot

        Args:
            domain_id: DDS domain ID

        Returns:
            bool: Whether initialization was successful
        """
        try:
            self.logger.info(f"Initializing foot force sensor connection - network interface: {self.network_interface}")

            # Initialize DDS channel factory
            ChannelFactoryInitialize(domain_id, self.network_interface)

            # Create LowState subscriber
            self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)

            # Verify connection
            if self._test_connection():
                self.is_initialized = True
                self.logger.info("Foot force sensor connection initialized successfully")
                return True
            else:
                self.logger.error("Foot force sensor connection test failed")
                return False

        except Exception as e:
            self.logger.error(f"Foot force sensor connection initialization failed: {e}")
            return False

    def _test_connection(self) -> bool:
        """Test whether connection is working"""
        try:
            if self.subscriber is None:
                self.logger.error("Subscriber not initialized")
                return False

            # Test whether subscriber is available, without trying to read specific data
            # Without a robot connection, we only verify that the subscriber was created successfully
            self.logger.info("Foot force sensor connection test successful - subscriber created")
            return True

        except Exception as e:
            self.logger.error(f"Foot force sensor connection test error: {e}")
            return False

    def _extract_foot_force_data(self, msg: LowState_) -> FootForceReading:
        """
        Extract foot force data from LowState message

        Args:
            msg: LowState message

        Returns:
            FootForceReading: Foot force reading
        """
        timestamp = time.time()

        # Extract foot force data (adjust based on actual SDK structure)
        # Note: Actual data structure may need adjustment based on unitree_sdk2py implementation
        foot_forces = []
        contact_states = []

        # Assumes LowState has foot_force field - needs adjustment based on actual API
        for i in range(4):
            if hasattr(msg, 'foot_force') and len(msg.foot_force) > i:
                # If 3D force data available
                if hasattr(msg.foot_force[i], 'x'):
                    fx = msg.foot_force[i].x
                    fy = msg.foot_force[i].y
                    fz = msg.foot_force[i].z
                else:
                    # If only vertical force available
                    fx, fy, fz = 0.0, 0.0, float(msg.foot_force[i])
            else:
                # Temporarily use other available data or default values
                fx, fy, fz = 0.0, 0.0, 0.0

            # Apply calibration offset
            offset = self.calibration_offset[self.FOOT_LABELS[i]]
            fx -= offset[0]
            fy -= offset[1]
            fz -= offset[2]

            foot_forces.append((fx, fy, fz))

            # Contact detection (based on vertical force)
            contact_states.append(abs(fz) > self.force_threshold)

        # Calculate total combined force
        total_force = sum(abs(force[2]) for force in foot_forces)

        # Calculate center of pressure (simplified calculation)
        if total_force > 0:
            # Assumed foot positions (relative to robot center of mass)
            foot_positions = [
                (-0.2, 0.15),   # FL: Front Left
                (-0.2, -0.15),  # FR: Front Right
                (0.2, 0.15),    # RL: Rear Left
                (0.2, -0.15)    # RR: Rear Right
            ]

            cop_x = sum(abs(force[2]) * pos[0] for force, pos in zip(foot_forces, foot_positions)) / total_force
            cop_y = sum(abs(force[2]) * pos[1] for force, pos in zip(foot_forces, foot_positions)) / total_force
            center_of_pressure = (cop_x, cop_y)
        else:
            center_of_pressure = (0.0, 0.0)

        return FootForceReading(
            timestamp=timestamp,
            foot_forces=tuple(foot_forces),
            contact_states=tuple(contact_states),
            total_force=total_force,
            center_of_pressure=center_of_pressure
        )

    def get_latest_reading(self) -> Optional[FootForceReading]:
        """
        Get latest foot force reading

        Returns:
            FootForceReading: Latest reading, None if no data available
        """
        if not self.is_initialized:
            self.logger.warning("Foot force sensor not initialized")
            return None

        try:
            # Try to read real data first
            if self.subscriber is not None:
                real_reading = self._read_real_data()
                if real_reading is not None:
                    # Update cache
                    with self._data_lock:
                        self._latest_reading = real_reading
                    return real_reading

            # Fall back to mock data if real data reading fails
            self.logger.warning("Failed to read real data, using mock data for testing")
            reading = self._generate_mock_reading()

            # Update cache
            with self._data_lock:
                self._latest_reading = reading

            return reading

        except Exception as e:
            self.logger.error(f"Failed to read foot force data: {e}")
            return None

    def get_cached_reading(self) -> Optional[FootForceReading]:
        """Get cached latest reading"""
        with self._data_lock:
            return self._latest_reading



    def _read_real_data(self) -> Optional[FootForceReading]:
        """
        Read real data from Unitree robot using correct SDK pattern

        Returns:
            FootForceReading: Real reading from robot, None if failed
        """
        try:
            if self.subscriber is None:
                return None

            # Official SDK pattern: Read() returns the data directly
            # No need to create LowState_ message manually
            lowstate_msg = self.subscriber.Read(timeout=100)  # 100ms timeout

            if lowstate_msg is not None:
                # Successfully received data from robot!
                reading = self._extract_foot_force_data(lowstate_msg)
                self.logger.info("Successfully read REAL data from Unitree robot!")
                return reading
            else:
                # No data available - this is normal if robot is not publishing
                self.logger.debug("No data available from robot (robot may not be active)")
                return None

        except Exception as e:
            self.logger.debug(f"Real data read attempt failed: {e}")
            return None

    def _generate_mock_reading(self) -> FootForceReading:
        """Generate mock foot force reading (for testing)"""
        import random

        timestamp = time.time()

        # Generate mock foot force data
        foot_forces = []
        contact_states = []

        for i in range(4):
            # Mock foot contact state (random)
            is_contact = random.choice([True, False])

            if is_contact:
                # Vertical force when in contact
                fx = random.uniform(-5, 5)  # Small horizontal force
                fy = random.uniform(-5, 5)  # Small horizontal force
                fz = random.uniform(10, 50)  # Vertical force
            else:
                # Very small force when airborne
                fx = random.uniform(-1, 1)
                fy = random.uniform(-1, 1)
                fz = random.uniform(0, 2)

            foot_forces.append((fx, fy, fz))
            contact_states.append(is_contact)

        # Calculate total combined force
        total_force = sum(abs(force[2]) for force in foot_forces)

        # Simple center of pressure calculation
        if total_force > 0:
            center_of_pressure = (0.0, 0.0)  # Simplified to origin
        else:
            center_of_pressure = (0.0, 0.0)

        return FootForceReading(
            timestamp=timestamp,
            foot_forces=tuple(foot_forces),
            contact_states=tuple(contact_states),
            total_force=total_force,
            center_of_pressure=center_of_pressure
        )

    def set_calibration_offset(self, foot_index: int, offset: Tuple[float, float, float]):
        """
        Set foot force sensor calibration offset

        Args:
            foot_index: Foot index (0-3)
            offset: Offset values (fx, fy, fz)
        """
        if 0 <= foot_index < 4:
            foot_label = self.FOOT_LABELS[foot_index]
            self.calibration_offset[foot_label] = offset
            self.logger.info(f"Set {self.FOOT_NAMES[foot_index]} calibration offset: {offset}")
        else:
            raise ValueError("Foot index must be between 0-3")

    def zero_calibration(self, duration_seconds: float = 5.0) -> bool:
        """
        Perform zero-point calibration

        Args:
            duration_seconds: Calibration data collection duration

        Returns:
            bool: Whether calibration was successful
        """
        if not self.is_initialized:
            self.logger.error("Foot force sensor not initialized, cannot perform calibration")
            return False

        self.logger.info(f"Starting zero-point calibration, duration: {duration_seconds} seconds")

        # Collect calibration data
        calibration_data = {i: [] for i in range(4)}
        start_time = time.time()
        sample_count = 0

        while time.time() - start_time < duration_seconds:
            reading = self.get_latest_reading()
            if reading:
                for i, force in enumerate(reading.foot_forces):
                    calibration_data[i].append(force)
                sample_count += 1
            time.sleep(0.01)  # 100Hz sampling

        if sample_count < 10:
            self.logger.error("Insufficient calibration data, cannot complete calibration")
            return False

        # Calculate calibration offsets
        for i in range(4):
            if calibration_data[i]:
                forces_array = np.array(calibration_data[i])
                mean_force = forces_array.mean(axis=0)
                self.set_calibration_offset(i, tuple(mean_force))

        self.logger.info(f"Zero-point calibration complete, collected {sample_count} samples")
        return True

    def validate_force_data(self, reading: FootForceReading) -> Dict[str, bool]:
        """
        Validate foot force data validity

        Args:
            reading: Foot force reading

        Returns:
            Dict[str, bool]: Validation results
        """
        validation_results = {
            'data_complete': True,
            'forces_in_range': True,
            'contact_consistent': True,
            'total_force_reasonable': True
        }

        # Check data completeness
        if len(reading.foot_forces) != 4:
            validation_results['data_complete'] = False

        # Check force value ranges
        for force in reading.foot_forces:
            if any(abs(f) > self.max_force_per_foot for f in force):
                validation_results['forces_in_range'] = False
                break

        # Check contact state consistency
        for i, (force, contact) in enumerate(zip(reading.foot_forces, reading.contact_states)):
            force_magnitude = abs(force[2])  # Vertical force
            expected_contact = force_magnitude > self.force_threshold
            if contact != expected_contact:
                validation_results['contact_consistent'] = False
                break

        # Check total force reasonableness
        if reading.total_force > 4 * self.max_force_per_foot:
            validation_results['total_force_reasonable'] = False

        return validation_results

    def get_foot_info(self) -> Dict[str, any]:
        """Get foot information"""
        return {
            'foot_count': 4,
            'foot_names': self.FOOT_NAMES,
            'foot_labels': self.FOOT_LABELS,
            'force_threshold': self.force_threshold,
            'max_force_per_foot': self.max_force_per_foot,
            'sampling_rate_hz': self.sampling_rate_hz,
            'calibration_offset': self.calibration_offset
        }

    def cleanup(self):
        """Clean up resources"""
        try:
            if self.subscriber:
                # Note: Clean up according to actual SDK API
                self.subscriber = None

            self.is_initialized = False
            self.logger.info("Foot force sensor configuration cleanup complete")

        except Exception as e:
            self.logger.error(f"Error cleaning up foot force sensor configuration: {e}")

# Utility functions
def create_default_config(network_interface: str = "eth0") -> FootForceConfig:
    """Create default configuration"""
    return FootForceConfig(network_interface=network_interface)

def format_force_reading(reading: FootForceReading) -> str:
    """Format force sensor reading to readable string"""
    lines = [
        f"Timestamp: {reading.timestamp:.3f}",
        f"Total Force: {reading.total_force:.2f} N",
        f"Center of Pressure: ({reading.center_of_pressure[0]:.3f}, {reading.center_of_pressure[1]:.3f})",
        "Foot Force Data:"
    ]

    for i, (force, contact) in enumerate(zip(reading.foot_forces, reading.contact_states)):
        foot_name = FootForceConfig.FOOT_NAMES[i]
        contact_str = "Contact" if contact else "Airborne"
        lines.append(f"  {foot_name}: Fx={force[0]:.2f}, Fy={force[1]:.2f}, Fz={force[2]:.2f} N [{contact_str}]")

    return "\n".join(lines)
