#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unitree Go2 LED Controller - Core Module
Implements LowCmd message publishing and basic LED control
Enhanced version: Integrated brightness-based flash system

Author: Claudia AI System
Generated: 2025-06-30
Enhanced: 2025-07-01 - Added flash mode support
Purpose: Subtask 6.1-6.4 - Complete LED control system implementation
"""

import os
import sys
import time
import threading
import logging
from typing import Tuple, Optional, List, Dict, Any
from dataclasses import dataclass
from enum import Enum
import struct

# Add project path (derived from module location, avoiding hardcoded paths)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

# Unitree SDK2 imports
try:
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, MotorCmd_, BmsCmd_
    from unitree_sdk2py.utils.crc import CRC
    UNITREE_SDK_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: Unitree SDK2 import failed: {e}")
    UNITREE_SDK_AVAILABLE = False

# VUI LED brightness control import
try:
    from unitree_sdk2py.go2.vui.vui_client import VuiClient
    VUI_CLIENT_AVAILABLE = True
except ImportError:
    VUI_CLIENT_AVAILABLE = False

@dataclass
class LEDState:
    """LED state data class"""
    timestamp: float
    led_data: List[int]  # uint8[12] LED data
    brightness: int      # Brightness level (0-255)
    is_active: bool     # Whether active

class LEDControlMode(Enum):
    """LED control mode enumeration"""
    OFF = 0
    SOLID = 1           # Steady on
    FLASH = 2           # Flash
    DOUBLE_FLASH = 3    # Double flash
    TRIPLE_FLASH = 4    # Triple flash
    PULSE = 5           # Pulse
    # Additional flash modes
    NORMAL = 10         # Normal operation - steady on
    WAITING = 11        # Waiting to process - single flash 1Hz
    WARNING = 12        # Warning state - double flash 2Hz
    ERROR = 13          # Fault state - fast flash 4Hz
    SPECIAL = 14        # Special state - breathing light

@dataclass
class FlashModeConfig:
    """Flash mode configuration"""
    mode: LEDControlMode
    type: str          # "steady", "single_flash", "double_flash", "fast_flash", "breathing"
    brightness: int    # Brightness level (0-10 for VUI, 0-255 for RGB)
    freq: float        # Frequency (Hz)
    brightness_range: Optional[Tuple[int, int]] = None  # Breathing light brightness range
    description: str = ""

class ClaudiaLEDController:
    """
    Claudia Robot LED Controller

    Controls the Unitree Go2 front LED array via LowCmd messages.
    Supports multiple LED modes and environmental adaptive brightness.
    Enhanced version: supports brightness-based flash system.
    """

    def __init__(self, network_interface: str = "eth0"):
        """
        Initialize LED controller

        Args:
            network_interface: Network interface name
        """
        self.logger = logging.getLogger(__name__)
        self.network_interface = network_interface

        # LED control state
        self.current_state = LEDState(
            timestamp=time.time(),
            led_data=[0] * 12,
            brightness=128,  # Default 50% brightness
            is_active=False
        )

        # Control parameters
        self.control_frequency = 50  # 50Hz control frequency
        self.control_dt = 1.0 / self.control_frequency
        self.max_response_time = 0.2  # 200ms max response time requirement

        # SDK components
        self.publisher = None
        self.crc_calculator = None
        self.vui_client = None
        self.is_initialized = False
        self.control_thread = None
        self.control_active = False

        # Flash system state
        self.flash_thread = None
        self.flash_stop_event = threading.Event()
        self.current_flash_mode = LEDControlMode.OFF
        self.use_vui_brightness = True  # Prefer VUI brightness control

        # Thread safety
        self.state_lock = threading.Lock()

        # Flash mode configuration
        self.flash_mode_configs = {
            LEDControlMode.NORMAL: FlashModeConfig(
                mode=LEDControlMode.NORMAL,
                type="steady",
                brightness=8,
                freq=0.0,
                description="Normal operation - steady on"
            ),
            LEDControlMode.WAITING: FlashModeConfig(
                mode=LEDControlMode.WAITING,
                type="single_flash",
                brightness=6,
                freq=1.0,
                description="Waiting to process - single flash 1Hz"
            ),
            LEDControlMode.WARNING: FlashModeConfig(
                mode=LEDControlMode.WARNING,
                type="double_flash",
                brightness=8,
                freq=2.0,
                description="Warning state - double flash 2Hz"
            ),
            LEDControlMode.ERROR: FlashModeConfig(
                mode=LEDControlMode.ERROR,
                type="fast_flash",
                brightness=10,
                freq=4.0,
                description="Fault state - fast flash 4Hz"
            ),
            LEDControlMode.SPECIAL: FlashModeConfig(
                mode=LEDControlMode.SPECIAL,
                type="breathing",
                brightness=6,
                freq=0.5,
                brightness_range=(2, 10),
                description="Special state - breathing light"
            ),
            LEDControlMode.OFF: FlashModeConfig(
                mode=LEDControlMode.OFF,
                type="steady",
                brightness=0,
                freq=0.0,
                description="Off state"
            )
        }

        # Performance monitoring
        self.performance_metrics = {
            'messages_sent': 0,
            'last_send_time': 0.0,
            'average_latency': 0.0,
            'max_latency': 0.0,
            'flash_mode_switches': 0
        }

        self.logger.info("LED controller initialization complete (with flash system)")

    def initialize_communication(self) -> bool:
        """
        Initialize communication with the robot

        Returns:
            bool: Whether initialization succeeded
        """
        success_count = 0
        total_methods = 2

        # Try initializing VUI client (preferred method)
        if self._initialize_vui_client():
            success_count += 1
            self.use_vui_brightness = True
            self.logger.info("VUI LED brightness control available")
        else:
            self.logger.warning("VUI LED brightness control not available")

        # Try initializing LowCmd control (fallback method)
        if self._initialize_lowcmd_control():
            success_count += 1
            self.logger.info("LowCmd LED control available")
        else:
            self.logger.warning("LowCmd LED control not available")

        # At least one method available is sufficient
        if success_count > 0:
            self.is_initialized = True
            self.logger.info(f"LED controller communication initialization complete ({success_count}/{total_methods} methods available)")
            return True
        else:
            self.logger.error("All LED control methods unavailable")
            return False

    def _initialize_vui_client(self) -> bool:
        """Initialize VUI client for brightness control"""
        if not VUI_CLIENT_AVAILABLE:
            return False

        try:
            ChannelFactoryInitialize(0, self.network_interface)
            self.vui_client = VuiClient()
            self.vui_client.SetTimeout(3.0)
            self.vui_client.Init()

            # Test VUI brightness control
            self.vui_client.SetBrightness(0)  # Safe test
            self.logger.info("VUI client initialized successfully")
            return True

        except Exception as e:
            self.logger.error(f"VUI client initialization failed: {e}")
            return False

    def _initialize_lowcmd_control(self) -> bool:
        """Initialize LowCmd control (original method)"""
        if not UNITREE_SDK_AVAILABLE:
            return False

        try:
            # Initialize DDS channel factory (if VUI not initialized)
            if self.vui_client is None:
                ChannelFactoryInitialize(0, self.network_interface)

            # Create LowCmd publisher
            lowcmd_topic = "rt/lowcmd"
            self.publisher = ChannelPublisher(lowcmd_topic, LowCmd_)
            self.publisher.Init()

            # Initialize CRC calculator
            self.crc_calculator = CRC()

            # Test basic communication
            return self._test_basic_communication()

        except Exception as e:
            self.logger.error(f"LowCmd control initialization failed: {e}")
            return False

    def set_vui_brightness(self, level: int) -> bool:
        """
        Set LED brightness using VUI client

        Args:
            level: Brightness level (0-10)

        Returns:
            bool: Whether setting succeeded
        """
        if not self.vui_client:
            return False

        level = max(0, min(10, level))  # Clamp range

        try:
            start_time = time.time()
            self.vui_client.SetBrightness(level)

            # Performance monitoring
            send_duration = time.time() - start_time
            self._update_performance_metrics(send_duration)

            self.logger.debug(f"VUI brightness set successfully: {level}, time: {send_duration*1000:.1f}ms")
            return True

        except Exception as e:
            self.logger.error(f"VUI brightness set failed: {e}")
            return False

    def start_flash_mode(self, mode: LEDControlMode) -> bool:
        """
        Start the specified flash mode

        Args:
            mode: Flash mode

        Returns:
            bool: Whether start succeeded
        """
        if mode not in self.flash_mode_configs:
            self.logger.error(f"Unknown flash mode: {mode}")
            return False

        if not self.is_initialized:
            self.logger.error("LED controller not initialized")
            return False

        # Stop current flash mode
        self.stop_flash_mode()

        config = self.flash_mode_configs[mode]
        self.current_flash_mode = mode

        self.logger.info(f"Starting flash mode: {config.description}")

        with self.state_lock:
            self.performance_metrics['flash_mode_switches'] += 1

        if config.type == "steady":
            # Steady mode (including off)
            if self.use_vui_brightness and self.vui_client:
                return self.set_vui_brightness(config.brightness)
            else:
                # Use RGB method to set white LED
                brightness_255 = int(config.brightness * 255 / 10)
                return self.set_led_color_simple(255, 255, 255, brightness_255)
        else:
            # Flash mode - start independent thread
            self.flash_stop_event.clear()
            self.flash_thread = threading.Thread(
                target=self._flash_worker,
                args=(config,),
                daemon=True
            )
            self.flash_thread.start()
            return True

    def stop_flash_mode(self) -> None:
        """Stop current flash mode"""
        if self.flash_thread and self.flash_thread.is_alive():
            self.flash_stop_event.set()
            self.flash_thread.join(timeout=2.0)
        self.current_flash_mode = LEDControlMode.OFF

    def _flash_worker(self, config: FlashModeConfig) -> None:
        """Flash worker thread"""
        flash_type = config.type

        try:
            if flash_type == "single_flash":
                self._single_flash(config)
            elif flash_type == "double_flash":
                self._double_flash(config)
            elif flash_type == "fast_flash":
                self._fast_flash(config)
            elif flash_type == "breathing":
                self._breathing_flash(config)
        except Exception as e:
            self.logger.error(f"Flash worker thread exception: {e}")

    def _single_flash(self, config: FlashModeConfig) -> None:
        """Single flash mode implementation"""
        period = 1.0 / config.freq
        on_time = period * 0.5
        off_time = period * 0.5

        while not self.flash_stop_event.is_set():
            # On
            if self.use_vui_brightness and self.vui_client:
                self.set_vui_brightness(config.brightness)
            else:
                brightness_255 = int(config.brightness * 255 / 10)
                self.set_led_color_simple(255, 255, 255, brightness_255)

            if self.flash_stop_event.wait(on_time):
                break

            # Off
            if self.use_vui_brightness and self.vui_client:
                self.set_vui_brightness(0)
            else:
                self.set_led_color_simple(0, 0, 0, 0)

            if self.flash_stop_event.wait(off_time):
                break

    def _double_flash(self, config: FlashModeConfig) -> None:
        """Double flash mode implementation"""
        period = 1.0 / config.freq
        flash_time = period * 0.2
        gap_time = period * 0.1
        pause_time = period * 0.5

        while not self.flash_stop_event.is_set():
            # First flash
            self._flash_once(config, flash_time)
            if self.flash_stop_event.wait(gap_time):
                break

            # Second flash
            self._flash_once(config, flash_time)
            if self.flash_stop_event.wait(pause_time):
                break

    def _fast_flash(self, config: FlashModeConfig) -> None:
        """Fast flash mode implementation"""
        period = 1.0 / config.freq
        on_time = period * 0.5
        off_time = period * 0.5

        while not self.flash_stop_event.is_set():
            # On
            if self.use_vui_brightness and self.vui_client:
                self.set_vui_brightness(config.brightness)
            else:
                brightness_255 = int(config.brightness * 255 / 10)
                self.set_led_color_simple(255, 255, 255, brightness_255)

            if self.flash_stop_event.wait(on_time):
                break

            # Off
            if self.use_vui_brightness and self.vui_client:
                self.set_vui_brightness(0)
            else:
                self.set_led_color_simple(0, 0, 0, 0)

            if self.flash_stop_event.wait(off_time):
                break

    def _breathing_flash(self, config: FlashModeConfig) -> None:
        """Breathing light mode implementation"""
        if not config.brightness_range:
            return

        min_brightness, max_brightness = config.brightness_range
        period = 1.0 / config.freq
        steps = 20
        step_time = period / (2 * steps)

        while not self.flash_stop_event.is_set():
            # Fade in
            for i in range(steps + 1):
                if self.flash_stop_event.is_set():
                    break
                brightness = min_brightness + (max_brightness - min_brightness) * (i / steps)

                if self.use_vui_brightness and self.vui_client:
                    self.set_vui_brightness(int(brightness))
                else:
                    brightness_255 = int(brightness * 255 / 10)
                    self.set_led_color_simple(255, 255, 255, brightness_255)

                if self.flash_stop_event.wait(step_time):
                    break

            # Fade out
            for i in range(steps + 1):
                if self.flash_stop_event.is_set():
                    break
                brightness = max_brightness - (max_brightness - min_brightness) * (i / steps)

                if self.use_vui_brightness and self.vui_client:
                    self.set_vui_brightness(int(brightness))
                else:
                    brightness_255 = int(brightness * 255 / 10)
                    self.set_led_color_simple(255, 255, 255, brightness_255)

                if self.flash_stop_event.wait(step_time):
                    break

    def _flash_once(self, config: FlashModeConfig, duration: float) -> None:
        """Execute a single flash"""
        # On
        if self.use_vui_brightness and self.vui_client:
            self.set_vui_brightness(config.brightness)
        else:
            brightness_255 = int(config.brightness * 255 / 10)
            self.set_led_color_simple(255, 255, 255, brightness_255)

        self.flash_stop_event.wait(duration / 2)

        # Off
        if self.use_vui_brightness and self.vui_client:
            self.set_vui_brightness(0)
        else:
            self.set_led_color_simple(0, 0, 0, 0)

        self.flash_stop_event.wait(duration / 2)

    def get_available_flash_modes(self) -> Dict[LEDControlMode, str]:
        """
        Get available flash modes

        Returns:
            Dict: Mode ID to description mapping
        """
        return {mode: config.description for mode, config in self.flash_mode_configs.items()}

    def get_current_flash_mode(self) -> LEDControlMode:
        """Get current flash mode"""
        return self.current_flash_mode

    def _test_basic_communication(self) -> bool:
        """
        Test basic communication functionality

        Returns:
            bool: Whether communication test succeeded
        """
        try:
            self.logger.info("Testing LED controller basic communication...")

            if self.publisher is None:
                self.logger.error("Publisher not initialized")
                return False

            # Create test LowCmd message
            test_msg = self._create_lowcmd_message()

            # Set safe test LED data (all off)
            test_msg.led = [0] * 12

            # Calculate and set CRC
            self._set_message_crc(test_msg)

            # Send test message
            start_time = time.time()
            self.publisher.Write(test_msg)
            send_duration = time.time() - start_time

            self.logger.info(f"Test message sent successfully, time: {send_duration*1000:.1f}ms")

            # Verify response time requirement
            if send_duration > self.max_response_time:
                self.logger.warning(f"Send time exceeds requirement: {send_duration*1000:.1f}ms > {self.max_response_time*1000}ms")

            return True

        except Exception as e:
            self.logger.error(f"Communication test failed: {e}")
            return False

    def _create_lowcmd_message(self) -> 'LowCmd_':
        """
        Create standard LowCmd message structure
        Uses positional parameter method based on debugging results

        Returns:
            LowCmd_: Initialized LowCmd message
        """
        try:
            # Based on debugging results, LowCmd_ requires 14 parameters in order

            # 1. head: uint8[2] - Message header
            head = [0xFE, 0xEF]

            # 2. level_flag: uint8 - Level flag
            level_flag = 0xFF

            # 3. frame_reserve: uint8 - Frame reserve
            frame_reserve = 0

            # 4. sn: uint32[2] - Sequence number
            sn = [0, 0]

            # 5. version: uint32[2] - Version
            version = [0, 0]

            # 6. bandwidth: uint16 - Bandwidth
            bandwidth = 0

            # 7. motor_cmd: MotorCmd_[20] - Motor command array (20 motors)
            motor_cmd = []
            for i in range(20):  # Go2 has 20 motors
                # Create safe motor command (stop mode)
                motor_cmd.append(MotorCmd_(
                    mode=0x00,      # Stop mode
                    q=0.0,          # Position
                    dq=0.0,         # Velocity
                    tau=0.0,        # Torque
                    kp=0.0,         # Position gain
                    kd=0.0,         # Velocity gain
                    reserve=[0, 0, 0]  # Reserved field uint32[3]
                ))

            # 8. bms_cmd: BmsCmd_ - Battery management system command
            bms_cmd = BmsCmd_(
                off=0,          # Off flag
                reserve=[0, 0, 0] # Reserved field uint8[3]
            )

            # 9. wireless_remote: uint8[40] - Wireless remote data
            wireless_remote = [0] * 40

            # 10. led: uint8[12] - LED data (the field we want to control!)
            led = [0] * 12

            # 11. fan: uint8[2] - Fan control
            fan = [0, 0]

            # 12. gpio: uint8 - GPIO state
            gpio = 0

            # 13. reserve: uint32 - Reserved field
            reserve = 0

            # 14. crc: uint32 - CRC checksum (calculated later)
            crc = 0

            # Create LowCmd message using positional parameters (confirmed working via debugging)
            msg = LowCmd_(
                head, level_flag, frame_reserve, sn, version, bandwidth,
                motor_cmd, bms_cmd, wireless_remote, led, fan, gpio, reserve, crc
            )

            self.logger.debug("LowCmd message created successfully (positional parameter method)")
            self.logger.debug(f"   LED field length: {len(msg.led)}")
            return msg

        except Exception as e:
            self.logger.error(f"Failed to create LowCmd message: {e}")
            # Provide more debug info
            try:
                self.logger.debug(f"LowCmd_ type: {type(LowCmd_)}")
                if hasattr(LowCmd_, '__doc__'):
                    self.logger.debug(f"LowCmd_ doc: {LowCmd_.__doc__}")
            except:
                pass
            raise

    def _set_message_crc(self, msg: 'LowCmd_') -> None:
        """
        Calculate and set the message CRC checksum

        Args:
            msg: LowCmd message
        """
        try:
            # CRC calculation method based on documentation example
            # crc = crc32_core((uint32_t *)&lowcmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1)

            if self.crc_calculator is None:
                self.logger.warning("CRC calculator not initialized, skipping CRC calculation")
                msg.crc = 0
                return

            # Use Unitree SDK provided CRC calculation method
            # Note: actual CRC calculation may need adjustment per SDK documentation
            crc_value = self.crc_calculator.Crc(msg)
            msg.crc = crc_value

            self.logger.debug(f"CRC calculation complete: 0x{crc_value:08X}")

        except Exception as e:
            self.logger.error(f"CRC calculation failed: {e}")
            msg.crc = 0  # Set to 0 as safe fallback

    def set_led_direct(self, led_data: List[int]) -> bool:
        """
        Set LED data directly

        Args:
            led_data: LED data array, length must be 12

        Returns:
            bool: Whether setting succeeded
        """
        if not self.is_initialized:
            self.logger.error("LED controller not initialized")
            return False

        if self.publisher is None:
            self.logger.error("Publisher not initialized")
            return False

        if len(led_data) != 12:
            self.logger.error(f"LED data length error: {len(led_data)} != 12")
            return False

        # Validate data range
        for i, value in enumerate(led_data):
            if not (0 <= value <= 255):
                self.logger.error(f"LED data[{i}] out of range: {value} (should be 0-255)")
                return False

        try:
            start_time = time.time()

            # Update internal state
            with self.state_lock:
                self.current_state.led_data = led_data.copy()
                self.current_state.timestamp = start_time
                self.current_state.is_active = any(x > 0 for x in led_data)

            # Create and send LowCmd message
            msg = self._create_lowcmd_message()
            msg.led = led_data
            self._set_message_crc(msg)

            # Send message
            self.publisher.Write(msg)

            # Performance monitoring
            send_duration = time.time() - start_time
            self._update_performance_metrics(send_duration)

            self.logger.debug(f"LED direct set successful, time: {send_duration*1000:.1f}ms")

            return True

        except Exception as e:
            self.logger.error(f"LED direct set failed: {e}")
            return False

    def set_led_color_simple(self, r: int, g: int, b: int, brightness: int = 255) -> bool:
        """
        Set simple RGB color (assuming first 4 LEDs are RGB controlled)

        Args:
            r: Red component (0-255)
            g: Green component (0-255)
            b: Blue component (0-255)
            brightness: Brightness (0-255)

        Returns:
            bool: Whether setting succeeded
        """
        # Apply brightness scaling, ensure result is integer
        r_scaled = int((r * brightness) // 255)
        g_scaled = int((g * brightness) // 255)
        b_scaled = int((b * brightness) // 255)

        # Create LED data - assuming format 1: [R1,G1,B1,R2,G2,B2,R3,G3,B3,R4,G4,B4]
        # This assumption needs experimental verification
        led_data = [r_scaled, g_scaled, b_scaled] * 4  # Copy to 4 LEDs

        self.logger.info(f"Setting LED color: RGB({r_scaled},{g_scaled},{b_scaled}) brightness={brightness}")
        return self.set_led_direct(led_data)

    def turn_off_all_leds(self) -> bool:
        """
        Turn off all LEDs

        Returns:
            bool: Whether operation succeeded
        """
        self.logger.info("Turning off all LEDs")
        return self.set_led_direct([0] * 12)

    def _update_performance_metrics(self, send_duration: float) -> None:
        """
        Update performance monitoring metrics

        Args:
            send_duration: Send duration (seconds)
        """
        self.performance_metrics['messages_sent'] += 1
        self.performance_metrics['last_send_time'] = send_duration

        # Calculate average latency
        count = self.performance_metrics['messages_sent']
        if count == 1:
            self.performance_metrics['average_latency'] = send_duration
        else:
            # Moving average
            alpha = 0.1  # Smoothing factor
            current_avg = self.performance_metrics['average_latency']
            self.performance_metrics['average_latency'] = (1 - alpha) * current_avg + alpha * send_duration

        # Update max latency
        if send_duration > self.performance_metrics['max_latency']:
            self.performance_metrics['max_latency'] = send_duration

        # Check performance requirement
        if send_duration > self.max_response_time:
            self.logger.warning(f"LED response time exceeded: {send_duration*1000:.1f}ms > {self.max_response_time*1000}ms")

    def get_performance_info(self) -> Dict[str, Any]:
        """
        Get performance information

        Returns:
            Dict: Performance monitoring data
        """
        with self.state_lock:
            return {
                'messages_sent': self.performance_metrics['messages_sent'],
                'last_send_time_ms': self.performance_metrics['last_send_time'] * 1000,
                'average_latency_ms': self.performance_metrics['average_latency'] * 1000,
                'max_latency_ms': self.performance_metrics['max_latency'] * 1000,
                'meets_requirement': self.performance_metrics['max_latency'] <= self.max_response_time,
                'flash_mode_switches': self.performance_metrics['flash_mode_switches'],
                'current_flash_mode': self.current_flash_mode.name,
                'vui_brightness_available': self.vui_client is not None,
                'lowcmd_control_available': self.publisher is not None,
                'current_state': self.current_state
            }

    def get_current_state(self) -> LEDState:
        """
        Get current LED state

        Returns:
            LEDState: Current LED state
        """
        with self.state_lock:
            return LEDState(
                timestamp=self.current_state.timestamp,
                led_data=self.current_state.led_data.copy(),
                brightness=self.current_state.brightness,
                is_active=self.current_state.is_active
            )

    def cleanup(self) -> None:
        """Clean up resources"""
        self.logger.info("Cleaning up LED controller resources...")

        try:
            # Stop flash mode
            self.stop_flash_mode()

            # Turn off all LEDs
            if self.is_initialized:
                if self.use_vui_brightness and self.vui_client:
                    self.set_vui_brightness(0)
                else:
                    self.turn_off_all_leds()
                time.sleep(0.1)  # Ensure last message is sent

            # Stop control thread
            if self.control_thread and self.control_thread.is_alive():
                self.control_active = False
                self.control_thread.join(timeout=1.0)

            self.is_initialized = False
            self.logger.info("LED controller resource cleanup complete")

        except Exception as e:
            self.logger.error(f"LED controller cleanup failed: {e}")

    def __enter__(self):
        """Context manager entry"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.cleanup()

# Factory function
def create_led_controller(network_interface: str = "eth0") -> ClaudiaLEDController:
    """
    Create LED controller instance

    Args:
        network_interface: Network interface name

    Returns:
        ClaudiaLEDController: LED controller instance
    """
    return ClaudiaLEDController(network_interface)

if __name__ == "__main__":
    # Basic test
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    print("LED Controller Basic Test")
    print("=" * 50)

    try:
        with create_led_controller() as controller:
            if controller.initialize_communication():
                print("LED controller initialized successfully")

                # Test basic LED control
                print("\nTesting red LED...")
                controller.set_led_color_simple(255, 0, 0, 128)
                time.sleep(2)

                print("Testing green LED...")
                controller.set_led_color_simple(0, 255, 0, 128)
                time.sleep(2)

                print("Testing blue LED...")
                controller.set_led_color_simple(0, 0, 255, 128)
                time.sleep(2)

                print("Turning off all LEDs...")
                controller.turn_off_all_leds()

                # Display performance info
                perf_info = controller.get_performance_info()
                print(f"\nPerformance Info:")
                print(f"   Messages sent: {perf_info['messages_sent']}")
                print(f"   Average latency: {perf_info['average_latency_ms']:.1f}ms")
                print(f"   Max latency: {perf_info['max_latency_ms']:.1f}ms")
                print(f"   Meets requirement: {'Yes' if perf_info['meets_requirement'] else 'No'}")

            else:
                print("LED controller initialization failed")

    except KeyboardInterrupt:
        print("\nUser interrupted test")
    except Exception as e:
        print(f"Test error: {e}")
        import traceback
        traceback.print_exc()
