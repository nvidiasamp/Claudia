#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Claudia LED Pattern Definition Module
Implements 5 dedicated LED status indicators based on VUI client

Author: Claudia AI System
Generated: 2025-06-30
Purpose: Subtask 6.2 - LED pattern definition and state machine implementation
"""

import os
import sys
import time
import threading
import logging
from typing import Tuple, Optional, List, Dict, Any, Callable
from dataclasses import dataclass
from enum import Enum
import math

# Add project path (derived from module location, avoiding hardcoded paths)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

# Unitree SDK2 VUI imports
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.vui.vui_client import VuiClient
    VUI_AVAILABLE = True
except ImportError as e:
    print(f"WARNING: VUI client import failed: {e}")
    VUI_AVAILABLE = False

class ClaudiaLEDMode(Enum):
    """
    Claudia dedicated LED mode enumeration

    Each mode corresponds to a specific interaction state,
    used to convey the robot's current status to the user
    """
    # System state modes
    OFF = "off"                          # Off state

    # Claudia dedicated status indicators
    WAKE_CONFIRM = "wake_confirm"        # Green double flash (wake confirmation)
    PROCESSING_VOICE = "processing"      # Blue steady (processing voice)
    EXECUTING_ACTION = "executing"       # Orange steady (executing action)
    ACTION_COMPLETE = "action_complete"  # White short flash x3 (action complete)
    ERROR_STATE = "error"                # Red triple flash (error/cannot understand)

    # System compatibility modes (avoid interfering with default states)
    SYSTEM_BOOT = "system_boot"          # Green steady (boot) - retained for compatibility
    SYSTEM_CALIBRATION = "calibration"   # Blue flashing (calibration) - retained for compatibility
    LOW_BATTERY = "low_battery"          # Yellow flashing (low battery) - retained for compatibility
    SEARCH_LIGHT = "search_light"        # White steady (search light) - retained for compatibility

@dataclass
class LEDPattern:
    """LED pattern parameters"""
    color: Tuple[int, int, int]          # RGB color (0-255)
    brightness: int                      # Brightness (0-10, VUI standard)
    flash_count: int                     # Flash count (0=steady)
    flash_interval: float                # Flash interval (seconds)
    duration: float                      # Pattern duration (seconds, 0=infinite)
    priority: int                        # Priority (1-10, 10 is highest)

class ClaudiaLEDModeDefinitions:
    """
    Claudia LED Mode Definition Class

    Defines all LED mode visual parameters and behavioral logic
    """

    # Claudia dedicated LED mode definitions
    PATTERNS = {
        ClaudiaLEDMode.OFF: LEDPattern(
            color=(0, 0, 0),
            brightness=0,
            flash_count=0,
            flash_interval=0.0,
            duration=0.0,
            priority=1
        ),

        # Green double flash (wake confirmation)
        ClaudiaLEDMode.WAKE_CONFIRM: LEDPattern(
            color=(0, 255, 0),               # Bright green
            brightness=8,                    # High brightness for visibility
            flash_count=2,                   # Double flash
            flash_interval=0.3,              # 300ms interval
            duration=2.0,                    # Auto-end after 2 seconds
            priority=7                       # High priority
        ),

        # Blue steady (processing voice)
        ClaudiaLEDMode.PROCESSING_VOICE: LEDPattern(
            color=(0, 100, 255),             # Soft blue
            brightness=6,                    # Medium brightness to avoid glare
            flash_count=0,                   # Steady
            flash_interval=0.0,
            duration=0.0,                    # Infinite until state change
            priority=6                       # Medium-high priority
        ),

        # Orange steady (executing action)
        ClaudiaLEDMode.EXECUTING_ACTION: LEDPattern(
            color=(255, 165, 0),             # Standard orange
            brightness=7,                    # Higher brightness for active state
            flash_count=0,                   # Steady
            flash_interval=0.0,
            duration=0.0,                    # Infinite until action complete
            priority=8                       # High priority
        ),

        # White short flash x3 (action complete)
        ClaudiaLEDMode.ACTION_COMPLETE: LEDPattern(
            color=(255, 255, 255),           # Pure white
            brightness=9,                    # High brightness for attention
            flash_count=3,                   # Triple flash
            flash_interval=0.2,              # 200ms rapid flash
            duration=1.5,                    # Complete in 1.5 seconds
            priority=9                       # Very high priority
        ),

        # Red triple flash (error/cannot understand)
        ClaudiaLEDMode.ERROR_STATE: LEDPattern(
            color=(255, 0, 0),               # Bright red
            brightness=10,                   # Maximum brightness for warning
            flash_count=3,                   # Triple flash
            flash_interval=0.4,              # 400ms slower for error indication
            duration=2.5,                    # 2.5 seconds to ensure user notice
            priority=10                      # Highest priority
        ),

        # System compatibility modes (retained but not actively used)
        ClaudiaLEDMode.SYSTEM_BOOT: LEDPattern(
            color=(0, 255, 0),
            brightness=5,
            flash_count=0,
            flash_interval=0.0,
            duration=0.0,
            priority=3
        ),

        ClaudiaLEDMode.SYSTEM_CALIBRATION: LEDPattern(
            color=(0, 0, 255),
            brightness=5,
            flash_count=10,                  # Continuous flashing
            flash_interval=0.5,
            duration=0.0,
            priority=4
        ),

        ClaudiaLEDMode.LOW_BATTERY: LEDPattern(
            color=(255, 255, 0),             # Yellow
            brightness=6,
            flash_count=10,                  # Continuous flashing
            flash_interval=1.0,              # Slow flash warning
            duration=0.0,
            priority=5
        ),

        ClaudiaLEDMode.SEARCH_LIGHT: LEDPattern(
            color=(255, 255, 255),
            brightness=10,                   # Maximum brightness
            flash_count=0,
            flash_interval=0.0,
            duration=0.0,
            priority=2
        )
    }

    @classmethod
    def get_pattern(cls, mode: ClaudiaLEDMode) -> LEDPattern:
        """
        Get LED pattern parameters for the specified mode

        Args:
            mode: LED mode

        Returns:
            LEDPattern: LED pattern parameters
        """
        return cls.PATTERNS.get(mode, cls.PATTERNS[ClaudiaLEDMode.OFF])

    @classmethod
    def get_all_modes(cls) -> List[ClaudiaLEDMode]:
        """Get all available LED modes"""
        return list(cls.PATTERNS.keys())

    @classmethod
    def get_claudia_modes(cls) -> List[ClaudiaLEDMode]:
        """Get Claudia-dedicated LED modes (excluding system compatibility modes)"""
        claudia_modes = [
            ClaudiaLEDMode.WAKE_CONFIRM,
            ClaudiaLEDMode.PROCESSING_VOICE,
            ClaudiaLEDMode.EXECUTING_ACTION,
            ClaudiaLEDMode.ACTION_COMPLETE,
            ClaudiaLEDMode.ERROR_STATE
        ]
        return claudia_modes

    @classmethod
    def validate_pattern(cls, pattern: LEDPattern) -> bool:
        """
        Validate LED pattern parameter validity

        Args:
            pattern: LED pattern parameters

        Returns:
            bool: Whether parameters are valid
        """
        # Validate color range
        r, g, b = pattern.color
        if not all(0 <= c <= 255 for c in [r, g, b]):
            return False

        # Validate brightness range (VUI standard: 0-10)
        if not (0 <= pattern.brightness <= 10):
            return False

        # Validate other parameters
        if pattern.flash_count < 0 or pattern.flash_interval < 0 or pattern.duration < 0:
            return False

        if not (1 <= pattern.priority <= 10):
            return False

        return True

class LEDModeRenderer:
    """
    LED Mode Renderer

    Responsible for converting LED modes into concrete VUI control commands
    """

    def __init__(self):
        """Initialize renderer"""
        self.logger = logging.getLogger(__name__)
        self.vui_client = None
        self.is_initialized = False

        # Rendering state
        self.current_mode = ClaudiaLEDMode.OFF
        self.current_pattern = None
        self.render_thread = None
        self.render_active = False
        self.render_lock = threading.Lock()

        # Environmental adaptation parameters
        self.environmental_brightness_factor = 1.0  # Environmental brightness adjustment factor
        self.auto_brightness_enabled = True

    def initialize_vui(self) -> bool:
        """
        Initialize VUI client

        Returns:
            bool: Whether initialization succeeded
        """
        if not VUI_AVAILABLE:
            self.logger.error("VUI client not available")
            return False

        try:
            self.logger.info("Initializing VUI client...")

            # Initialize channel
            ChannelFactoryInitialize(0)

            # Create VUI client
            self.vui_client = VuiClient()
            self.vui_client.SetTimeout(3.0)
            self.vui_client.Init()

            # Test connection
            code, current_brightness = self.vui_client.GetBrightness()
            if code == 0:
                self.logger.info(f"VUI client initialized successfully, current LED brightness: {current_brightness}")
                self.is_initialized = True
                return True
            else:
                self.logger.error(f"VUI client connection test failed, error code: {code}")
                return False

        except Exception as e:
            self.logger.error(f"VUI client initialization failed: {e}")
            return False

    def render_mode(self, mode: ClaudiaLEDMode, duration_override: Optional[float] = None) -> bool:
        """
        Render the specified LED mode

        Args:
            mode: LED mode to render
            duration_override: Optional duration override

        Returns:
            bool: Whether rendering started successfully
        """
        if not self.is_initialized:
            self.logger.error("VUI client not initialized")
            return False

        pattern = ClaudiaLEDModeDefinitions.get_pattern(mode)
        if not ClaudiaLEDModeDefinitions.validate_pattern(pattern):
            self.logger.error(f"Invalid LED pattern parameters: {mode}")
            return False

        with self.render_lock:
            # Stop current rendering
            self._stop_current_render()

            # Set new mode
            self.current_mode = mode
            self.current_pattern = pattern

            # Apply duration override
            if duration_override is not None:
                pattern = LEDPattern(
                    color=pattern.color,
                    brightness=pattern.brightness,
                    flash_count=pattern.flash_count,
                    flash_interval=pattern.flash_interval,
                    duration=duration_override,
                    priority=pattern.priority
                )
                self.current_pattern = pattern

            # Start rendering thread
            self.render_active = True
            self.render_thread = threading.Thread(
                target=self._render_pattern_worker,
                args=(pattern,),
                daemon=True
            )
            self.render_thread.start()

            self.logger.info(f"Started rendering LED mode: {mode.value}")
            return True

    def _render_pattern_worker(self, pattern: LEDPattern) -> None:
        """
        LED mode rendering worker thread

        Args:
            pattern: LED pattern to render
        """
        try:
            start_time = time.time()
            r, g, b = pattern.color

            # Apply environmental adaptive brightness
            effective_brightness = self._calculate_effective_brightness(pattern.brightness)

            if pattern.flash_count == 0:
                # Steady mode
                self._set_led_color_brightness(r, g, b, effective_brightness)

                # If there is a duration limit, wait then turn off
                if pattern.duration > 0:
                    elapsed = 0
                    while elapsed < pattern.duration and self.render_active:
                        time.sleep(0.1)
                        elapsed = time.time() - start_time

                    # Time expired, turn off LED
                    if self.render_active:
                        self._set_led_color_brightness(0, 0, 0, 0)

            else:
                # Flash mode
                for flash_num in range(pattern.flash_count):
                    if not self.render_active:
                        break

                    # On
                    self._set_led_color_brightness(r, g, b, effective_brightness)
                    time.sleep(pattern.flash_interval / 2)

                    if not self.render_active:
                        break

                    # Off
                    self._set_led_color_brightness(0, 0, 0, 0)

                    # Wait interval if not the last flash
                    if flash_num < pattern.flash_count - 1:
                        time.sleep(pattern.flash_interval / 2)

                # After flashing complete, decide whether to maintain state based on duration
                if pattern.duration > 0:
                    elapsed = time.time() - start_time
                    remaining = pattern.duration - elapsed
                    if remaining > 0 and self.render_active:
                        time.sleep(remaining)

                # Finally turn off LED
                if self.render_active:
                    self._set_led_color_brightness(0, 0, 0, 0)

        except Exception as e:
            self.logger.error(f"LED mode rendering failed: {e}")
        finally:
            with self.render_lock:
                self.render_active = False
                self.current_mode = ClaudiaLEDMode.OFF

    def _set_led_color_brightness(self, r: int, g: int, b: int, brightness: int) -> bool:
        """
        Set LED color and brightness

        Args:
            r, g, b: RGB color values (0-255)
            brightness: Brightness value (0-10)

        Returns:
            bool: Whether setting succeeded
        """
        try:
            if self.vui_client is None:
                return False

            # Set brightness
            brightness_code = self.vui_client.SetBrightness(brightness)

            # Note: VUI client may not have direct RGB control method
            # Needs adjustment based on actual SDK capabilities
            # Assuming use of AudioClient's LedControl method here

            if brightness_code != 0:
                self.logger.warning(f"Set brightness failed, error code: {brightness_code}")
                return False

            # TODO: Implement RGB color control
            # May need to use AudioClient or other interface

            return True

        except Exception as e:
            self.logger.error(f"Set LED color/brightness failed: {e}")
            return False

    def _calculate_effective_brightness(self, target_brightness: int) -> int:
        """
        Calculate environmentally adapted effective brightness

        Args:
            target_brightness: Target brightness (0-10)

        Returns:
            int: Effective brightness (0-10)
        """
        if not self.auto_brightness_enabled:
            return target_brightness

        # Apply environmental brightness adjustment factor
        effective = int(target_brightness * self.environmental_brightness_factor)
        return max(0, min(10, effective))

    def _stop_current_render(self) -> None:
        """Stop current rendering"""
        if self.render_thread and self.render_thread.is_alive():
            self.render_active = False
            self.render_thread.join(timeout=1.0)

    def stop_all_rendering(self) -> None:
        """Stop all LED rendering"""
        with self.render_lock:
            self._stop_current_render()
            # Turn off all LEDs
            if self.is_initialized:
                self._set_led_color_brightness(0, 0, 0, 0)
            self.current_mode = ClaudiaLEDMode.OFF

    def get_current_mode(self) -> ClaudiaLEDMode:
        """Get current LED mode"""
        with self.render_lock:
            return self.current_mode

    def set_environmental_brightness_factor(self, factor: float) -> None:
        """
        Set environmental brightness adjustment factor

        Args:
            factor: Adjustment factor (0.1-2.0, 1.0 is normal)
        """
        self.environmental_brightness_factor = max(0.1, min(2.0, factor))
        self.logger.info(f"Environmental brightness adjustment factor set to: {self.environmental_brightness_factor}")

    def cleanup(self) -> None:
        """Clean up resources"""
        self.logger.info("Cleaning up LED mode renderer...")
        self.stop_all_rendering()
        self.is_initialized = False


# Factory function
def create_led_mode_renderer() -> LEDModeRenderer:
    """
    Create LED mode renderer instance

    Returns:
        LEDModeRenderer: Renderer instance
    """
    return LEDModeRenderer()


if __name__ == "__main__":
    # Basic test
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    print("LED Pattern Definition Test")
    print("=" * 50)

    # Test pattern definitions
    print("\nClaudia Dedicated LED Modes:")
    for mode in ClaudiaLEDModeDefinitions.get_claudia_modes():
        pattern = ClaudiaLEDModeDefinitions.get_pattern(mode)
        print(f"   {mode.value}: RGB{pattern.color} brightness={pattern.brightness} priority={pattern.priority}")

    # Test renderer (requires real hardware)
    renderer = create_led_mode_renderer()

    try:
        if renderer.initialize_vui():
            print("VUI client initialized successfully")

            # Test wake confirmation mode
            print("\nTesting wake confirmation mode...")
            renderer.render_mode(ClaudiaLEDMode.WAKE_CONFIRM)
            time.sleep(3)

            print("Test complete!")
        else:
            print("VUI client initialization failed (robot may not be connected)")

    except KeyboardInterrupt:
        print("\nUser interrupted test")
    finally:
        renderer.cleanup()
