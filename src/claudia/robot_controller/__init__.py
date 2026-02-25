# src/claudia/robot_controller/__init__.py
"""
Claudia Robot Controller Module

Provides hardware control interface for the Unitree Go2 robot,
including LED control, sensor data reading, and more.

Author: Claudia AI System
"""

# Original LED controller (LowCmd infrastructure)
from .led_controller import ClaudiaLEDController, create_led_controller, LEDControlMode

# LED mode definitions and renderer
from .led_patterns import (
    ClaudiaLEDMode, LEDPattern, ClaudiaLEDModeDefinitions,
    LEDModeRenderer, create_led_mode_renderer
)

# LED state machine
from .led_state_machine import (
    LEDStateMachine, LEDStateRequest, LEDStateHistory,
    create_led_state_machine
)

# Unified LED controller (recommended)
from .unified_led_controller import (
    UnifiedLEDController, LEDControlMethod, AdvancedEnvironmentalLightInfo,
    EnvironmentalAdaptationProfile, create_unified_led_controller
)

__all__ = [
    # Original LED controller
    'ClaudiaLEDController',
    'create_led_controller',
    'LEDControlMode',

    # LED mode definitions
    'ClaudiaLEDMode',
    'LEDPattern',
    'ClaudiaLEDModeDefinitions',
    'LEDModeRenderer',
    'create_led_mode_renderer',

    # LED state machine
    'LEDStateMachine',
    'LEDStateRequest',
    'LEDStateHistory',
    'create_led_state_machine',

    # Unified LED controller (recommended)
    'UnifiedLEDController',
    'LEDControlMethod',
    'AdvancedEnvironmentalLightInfo',
    'EnvironmentalAdaptationProfile',
    'create_unified_led_controller'
]

__version__ = "0.2.0"  # Version upgrade: added LED mode definitions and state machine features

# Convenience factory function for higher-level applications
def create_claudia_led_system(preferred_method: str = "vui",
                             enable_environmental_adaptation: bool = True):
    """
    Create a complete Claudia LED control system.

    This is the recommended way to create the LED control system, integrating all features:
    - VUI/LowCmd dual control methods
    - 5 dedicated LED status indicators
    - Priority management and state machine
    - Environmental adaptive brightness adjustment
    - System compatibility protection

    Args:
        preferred_method: Preferred control method ("vui", "lowcmd", "auto")
        enable_environmental_adaptation: Whether to enable environmental adaptation

    Returns:
        UnifiedLEDController: Unified LED controller instance

    Example:
        >>> from claudia.robot_controller import create_claudia_led_system
        >>> led_system = create_claudia_led_system()
        >>> led_system.initialize()
        >>> led_system.wake_confirm()  # Green double flash
        >>> led_system.processing_voice()  # Blue solid
        >>> led_system.cleanup()
    """
    method_map = {
        "vui": LEDControlMethod.VUI_CLIENT,
        "lowcmd": LEDControlMethod.LOW_CMD,
        "auto": LEDControlMethod.AUTO_SELECT
    }

    control_method = method_map.get(preferred_method, LEDControlMethod.VUI_CLIENT)

    return create_unified_led_controller(
        preferred_method=control_method,
        enable_environmental_adaptation=enable_environmental_adaptation
    )