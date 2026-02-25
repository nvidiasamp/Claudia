#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Claudia System State Monitor - Real-time monitoring of Unitree Go2 system state
Monitors critical system states, providing priority decision basis for LED control

Author: Claudia AI System
Generated: 2025-06-30
Purpose: Subtask 6.4 - System default compatibility and safety mechanisms
Version: 0.4.0 (System Compatibility Enhancement)
"""

import sys
import time
import threading
import logging
import json
from typing import Dict, Optional, Callable, Any, List, Tuple
from dataclasses import dataclass, field
from enum import Enum, IntEnum
from collections import deque
import statistics

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from std_msgs.msg import String, Float32, Bool
    # Unitree message types (adjust based on actual SDK)
    try:
        from unitree_go.msg import LowState, SportModeState, BmsState
        UNITREE_MSGS_AVAILABLE = True
    except ImportError:
        # Fallback message definitions
        UNITREE_MSGS_AVAILABLE = False
        # Do not print warning -- hardware mode uses SDKStateProvider to get state,
        # does not depend on ROS2 message types. Only report error when ROS2 monitoring
        # is actually needed.
        logging.getLogger("SystemStateMonitor").debug(
            "Unitree ROS2 message types unavailable (LowState/SportModeState/BmsState), "
            "hardware mode will use direct SDK connection"
        )

    ROS2_AVAILABLE = True
except ImportError as e:
    logging.getLogger("SystemStateMonitor").debug("ROS2 import failed: %s", e)
    ROS2_AVAILABLE = False
    # Placeholder definitions when ROS2 is unavailable, preventing NameError in
    # SystemMonitorNode class definition
    Node = object
    String = Float32 = Bool = object
    UNITREE_MSGS_AVAILABLE = False

class SystemState(IntEnum):
    """System state enumeration (sorted by priority)"""
    UNKNOWN = 0                    # Unknown state
    NORMAL = 1                     # Normal operation
    IDLE = 2                       # Idle state
    ACTIVE = 3                     # Active state
    CALIBRATING = 5                # Calibrating
    LOW_BATTERY = 7                # Low battery
    ERROR = 8                      # System error
    EMERGENCY = 10                 # Emergency state

class SystemLEDPriority(IntEnum):
    """System LED priority mapping"""
    NORMAL = 1                     # Normal state
    USER_INTERACTION = 5           # User interaction
    SYSTEM_NOTIFICATION = 7        # System notification
    SAFETY_WARNING = 8             # Safety warning
    CRITICAL_ERROR = 10            # Critical error

@dataclass
class SystemStateInfo:
    """System state information"""
    state: SystemState
    priority: SystemLEDPriority
    battery_level: float           # Battery level (0-1)
    battery_voltage: float         # Battery voltage
    is_charging: bool              # Whether charging
    error_codes: List[int]         # Error code list
    temperature: float             # System temperature
    timestamp: float               # Timestamp

    # Motion state
    is_standing: bool = False      # Whether standing
    is_moving: bool = False        # Whether moving
    current_gait: str = "unknown"  # Current gait

    # Network and communication
    network_status: str = "unknown"  # Network status
    sdk_connection: bool = True      # SDK connection status

    # Metadata
    confidence: float = 1.0        # State confidence
    source: str = "system_monitor" # Data source

@dataclass
class LEDControlDecision:
    """LED control decision"""
    allow_custom_control: bool     # Whether custom LED control is allowed
    required_priority: SystemLEDPriority  # Required minimum priority
    system_override_active: bool   # Whether system is forcing override
    recommended_action: str        # Recommended action
    reason: str                   # Decision reason

class SystemStateMonitor:
    """System State Monitor"""

    def __init__(self,
                 node_name: str = "claudia_system_monitor",
                 history_size: int = 50,
                 update_rate: float = 10.0):
        """
        Initialize system state monitor

        Args:
            node_name: ROS2 node name
            history_size: History data cache size
            update_rate: Update frequency (Hz)
        """
        self.logger = logging.getLogger(__name__)
        self.node_name = node_name
        self.history_size = history_size
        self.update_rate = update_rate

        # ROS2 components
        self.node = None
        self.executor = None
        self.executor_thread = None
        self.is_ros_initialized = False

        # State data
        self.current_state_info = SystemStateInfo(
            state=SystemState.UNKNOWN,
            priority=SystemLEDPriority.NORMAL,
            battery_level=1.0,
            battery_voltage=25.0,
            is_charging=False,
            error_codes=[],
            temperature=30.0,
            timestamp=time.time()
        )

        # History data and analysis
        self.state_history = deque(maxlen=history_size)
        self.battery_history = deque(maxlen=history_size)
        self.error_history = deque(maxlen=history_size)

        # Callbacks and events
        self.state_change_callbacks = []
        self.critical_event_callbacks = []

        # Monitoring control
        self.monitoring_active = False
        self.monitor_thread = None
        self.monitor_lock = threading.Lock()

        # Performance statistics
        self.update_count = 0
        self.error_count = 0
        self.last_update_time = 0
        self.avg_update_interval = 0.1

        # State analysis configuration
        self.low_battery_threshold = 0.15      # 15% low battery threshold
        self.critical_battery_threshold = 0.05  # 5% critical battery threshold
        self.high_temperature_threshold = 70.0  # 70C high temperature threshold
        self.network_timeout = 5.0              # 5 second network timeout

    def initialize(self) -> bool:
        """Initialize monitor"""
        try:
            self.logger.info("Initializing system state monitor...")

            # Initialize ROS2
            if ROS2_AVAILABLE:
                success = self._initialize_ros2()
                if not success:
                    self.logger.warning("ROS2 initialization failed, enabling simulation mode")
                    return self._initialize_simulation_mode()
            else:
                self.logger.warning("ROS2 not available, enabling simulation mode")
                return self._initialize_simulation_mode()

            self.logger.info("System state monitor initialized successfully")
            return True

        except Exception as e:
            self.logger.error(f"System state monitor initialization failed: {e}")
            return False

    def _initialize_ros2(self) -> bool:
        """Initialize ROS2 components"""
        import os
        import contextlib

        try:
            # Suppress ROS2 error output (users should not see underlying RMW errors)
            with contextlib.redirect_stderr(open(os.devnull, 'w')):
                if not rclpy.ok():
                    # Set ROS2 log level to FATAL (suppress ERROR level)
                    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = ''
                    os.environ['RCUTILS_LOGGING_USE_STDOUT'] = '0'
                    rclpy.init()

                # Create node (may fail, but errors are suppressed)
                self.node = SystemMonitorNode(
                    node_name=self.node_name,
                    state_callback=self._on_state_update,
                    error_callback=self._on_system_error
                )

            # Create executor
            self.executor = MultiThreadedExecutor(num_threads=2)
            self.executor.add_node(self.node)

            # Start executor thread
            self.executor_thread = threading.Thread(
                target=self.executor.spin,
                daemon=True
            )
            self.executor_thread.start()

            self.is_ros_initialized = True
            self.logger.info("ROS2 system state monitor initialized successfully")
            return True

        except Exception as e:
            # Do not log detailed errors on ROS2 init failure (auto-fallback to simulation mode)
            return False

    def _initialize_simulation_mode(self) -> bool:
        """Initialize simulation mode"""
        try:
            self.is_ros_initialized = False
            self.logger.info("Enabling system state simulation mode")

            # Start simulation monitoring thread
            self.monitor_thread = threading.Thread(
                target=self._simulation_monitor_worker,
                daemon=True
            )
            self.monitor_thread.start()

            return True

        except Exception as e:
            self.logger.error(f"Simulation mode initialization failed: {e}")
            return False

    def start_monitoring(self) -> bool:
        """Start monitoring"""
        try:
            with self.monitor_lock:
                if self.monitoring_active:
                    self.logger.warning("System state monitoring is already running")
                    return True

                self.monitoring_active = True

                if not self.is_ros_initialized:
                    # Simulation mode already started in initialize
                    pass

                self.logger.info("System state monitoring started")
                return True

        except Exception as e:
            self.logger.error(f"Failed to start system state monitoring: {e}")
            return False

    def stop_monitoring(self) -> None:
        """Stop monitoring"""
        try:
            with self.monitor_lock:
                self.monitoring_active = False

                if self.executor:
                    self.executor.shutdown()

                if self.executor_thread and self.executor_thread.is_alive():
                    self.executor_thread.join(timeout=2.0)

                self.logger.info("System state monitoring stopped")

        except Exception as e:
            self.logger.error(f"Failed to stop system state monitoring: {e}")

    def _simulation_monitor_worker(self) -> None:
        """Simulation monitoring worker thread"""
        simulation_cycle = 0

        while self.monitoring_active:
            try:
                # Simulate system state changes
                simulation_cycle += 1
                current_time = time.time()

                # Basic state simulation
                if simulation_cycle % 100 == 0:  # Simulate low battery every 10 seconds
                    battery_level = 0.12  # Simulate low battery
                    state = SystemState.LOW_BATTERY
                    priority = SystemLEDPriority.SAFETY_WARNING
                elif simulation_cycle % 200 == 0:  # Simulate calibration every 20 seconds
                    battery_level = 0.85
                    state = SystemState.CALIBRATING
                    priority = SystemLEDPriority.SYSTEM_NOTIFICATION
                else:
                    battery_level = 0.85  # Normal battery
                    state = SystemState.NORMAL
                    priority = SystemLEDPriority.NORMAL

                # Create simulated state info
                sim_state_info = SystemStateInfo(
                    state=state,
                    priority=priority,
                    battery_level=battery_level,
                    battery_voltage=24.0 + battery_level * 4.0,
                    is_charging=False,
                    error_codes=[],
                    temperature=35.0,
                    timestamp=current_time,
                    is_standing=True,
                    current_gait="standing",
                    network_status="connected",
                    confidence=0.9,
                    source="simulation"
                )

                # Update state
                self._on_state_update(sim_state_info)

                time.sleep(1.0 / self.update_rate)

            except Exception as e:
                self.logger.error(f"Simulation monitoring loop error: {e}")
                time.sleep(1.0)

    def _on_state_update(self, state_info: SystemStateInfo) -> None:
        """Handle state update"""
        try:
            with self.monitor_lock:
                previous_state = self.current_state_info.state
                previous_priority = self.current_state_info.priority

                # Update current state
                self.current_state_info = state_info

                # Update history data
                self.state_history.append(state_info)
                self.battery_history.append(state_info.battery_level)
                if state_info.error_codes:
                    self.error_history.extend(state_info.error_codes)

                # Update performance statistics
                self.update_count += 1
                current_time = time.time()
                if self.last_update_time > 0:
                    interval = current_time - self.last_update_time
                    self.avg_update_interval = (self.avg_update_interval * 0.9 + interval * 0.1)
                self.last_update_time = current_time

                # Check state change
                if (previous_state != state_info.state or
                    previous_priority != state_info.priority):
                    self._notify_state_change(previous_state, state_info)

                # Check critical events
                self._check_critical_events(state_info)

        except Exception as e:
            self.logger.error(f"State update processing failed: {e}")
            self.error_count += 1

    def _on_system_error(self, error_code: int, error_message: str) -> None:
        """Handle system error"""
        try:
            self.logger.warning(f"System error: {error_code} - {error_message}")

            # Update error history
            self.error_history.append(error_code)

            # Create error state info
            error_state_info = SystemStateInfo(
                state=SystemState.ERROR,
                priority=SystemLEDPriority.CRITICAL_ERROR,
                battery_level=self.current_state_info.battery_level,
                battery_voltage=self.current_state_info.battery_voltage,
                is_charging=self.current_state_info.is_charging,
                error_codes=[error_code],
                temperature=self.current_state_info.temperature,
                timestamp=time.time(),
                source="error_handler"
            )

            # Trigger state update
            self._on_state_update(error_state_info)

        except Exception as e:
            self.logger.error(f"System error handling failed: {e}")

    def _notify_state_change(self, previous_state: SystemState, new_state_info: SystemStateInfo) -> None:
        """Notify state change"""
        try:
            self.logger.info(f"System state change: {previous_state.name} -> {new_state_info.state.name}")

            # Call all state change callbacks
            for callback in self.state_change_callbacks:
                try:
                    callback(previous_state, new_state_info)
                except Exception as e:
                    self.logger.error(f"State change callback failed: {e}")

        except Exception as e:
            self.logger.error(f"State change notification failed: {e}")

    def _check_critical_events(self, state_info: SystemStateInfo) -> None:
        """Check critical events"""
        try:
            critical_events = []

            # Check low battery
            if state_info.battery_level <= self.critical_battery_threshold:
                critical_events.append(("critical_battery", f"Battery critically low: {state_info.battery_level*100:.1f}%"))
            elif state_info.battery_level <= self.low_battery_threshold:
                critical_events.append(("low_battery", f"Battery low: {state_info.battery_level*100:.1f}%"))

            # Check high temperature
            if state_info.temperature >= self.high_temperature_threshold:
                critical_events.append(("high_temperature", f"System temperature too high: {state_info.temperature:.1f}C"))

            # Check error codes
            if state_info.error_codes:
                critical_events.append(("system_errors", f"System errors: {state_info.error_codes}"))

            # Trigger critical event callbacks
            for event_type, event_message in critical_events:
                self.logger.warning(f"Critical event: {event_message}")
                for callback in self.critical_event_callbacks:
                    try:
                        callback(event_type, event_message, state_info)
                    except Exception as e:
                        self.logger.error(f"Critical event callback failed: {e}")

        except Exception as e:
            self.logger.error(f"Critical event check failed: {e}")

    def get_current_state(self) -> SystemStateInfo:
        """Get current system state"""
        return self.current_state_info

    def get_led_control_decision(self, requested_priority: int = 5) -> LEDControlDecision:
        """
        Get LED control decision

        Args:
            requested_priority: Requested priority

        Returns:
            LEDControlDecision: LED control decision
        """
        try:
            current_state = self.current_state_info
            system_priority = current_state.priority.value

            # Basic rule: requested priority must be >= system state priority
            allow_control = requested_priority >= system_priority

            # Special state handling
            if current_state.state in [SystemState.EMERGENCY, SystemState.ERROR]:
                # Emergency and error states require higher priority
                required_priority = SystemLEDPriority.CRITICAL_ERROR
                allow_control = requested_priority >= SystemLEDPriority.CRITICAL_ERROR.value
                system_override = True
                reason = f"System in {current_state.state.name} state, requires high priority control"

            elif current_state.state == SystemState.LOW_BATTERY:
                required_priority = SystemLEDPriority.SAFETY_WARNING
                allow_control = requested_priority >= SystemLEDPriority.SAFETY_WARNING.value
                system_override = True
                reason = f"Battery low ({current_state.battery_level*100:.1f}%), prioritizing safety warning"

            elif current_state.state == SystemState.CALIBRATING:
                required_priority = SystemLEDPriority.SYSTEM_NOTIFICATION
                allow_control = requested_priority >= SystemLEDPriority.SYSTEM_NOTIFICATION.value
                system_override = False
                reason = "System calibrating, recommend avoiding LED control conflicts"

            else:
                # Normal state
                required_priority = SystemLEDPriority.NORMAL
                system_override = False
                reason = "System state normal, custom LED control allowed"

            # Recommended action
            if allow_control:
                recommended_action = "proceed"
            elif requested_priority < required_priority.value:
                recommended_action = "increase_priority"
            else:
                recommended_action = "wait_for_system_state_change"

            return LEDControlDecision(
                allow_custom_control=allow_control,
                required_priority=required_priority,
                system_override_active=system_override,
                recommended_action=recommended_action,
                reason=reason
            )

        except Exception as e:
            self.logger.error(f"LED control decision generation failed: {e}")
            # Safe fallback: deny control
            return LEDControlDecision(
                allow_custom_control=False,
                required_priority=SystemLEDPriority.CRITICAL_ERROR,
                system_override_active=True,
                recommended_action="wait",
                reason=f"Decision generation failed: {e}"
            )

    def register_state_change_callback(self, callback: Callable[[SystemState, SystemStateInfo], None]) -> None:
        """Register state change callback"""
        self.state_change_callbacks.append(callback)

    def register_critical_event_callback(self, callback: Callable[[str, str, SystemStateInfo], None]) -> None:
        """Register critical event callback"""
        self.critical_event_callbacks.append(callback)

    def get_system_statistics(self) -> Dict[str, Any]:
        """Get system statistics"""
        try:
            current_time = time.time()
            uptime = current_time - (self.last_update_time - self.avg_update_interval * self.update_count) if self.update_count > 0 else 0

            # Battery statistics
            battery_stats = {}
            if self.battery_history:
                battery_stats = {
                    "current": self.current_state_info.battery_level,
                    "average": statistics.mean(self.battery_history),
                    "min": min(self.battery_history),
                    "max": max(self.battery_history),
                    "trend": "stable"  # Simplified version, can compute actual trend
                }

            # State distribution statistics
            state_distribution = {}
            if self.state_history:
                for state_info in self.state_history:
                    state_name = state_info.state.name
                    state_distribution[state_name] = state_distribution.get(state_name, 0) + 1

            return {
                "uptime_seconds": uptime,
                "update_count": self.update_count,
                "error_count": self.error_count,
                "avg_update_interval": self.avg_update_interval,
                "current_state": self.current_state_info.state.name,
                "current_priority": self.current_state_info.priority.name,
                "battery_stats": battery_stats,
                "state_distribution": state_distribution,
                "error_history_count": len(self.error_history),
                "is_ros_initialized": self.is_ros_initialized,
                "monitoring_active": self.monitoring_active
            }

        except Exception as e:
            self.logger.error(f"Failed to get system statistics: {e}")
            return {"error": str(e)}

    def cleanup(self) -> None:
        """Clean up resources"""
        try:
            self.stop_monitoring()

            if self.node:
                self.node.destroy_node()

            if rclpy.ok():
                rclpy.shutdown()

            self.logger.info("System state monitor cleanup complete")

        except Exception as e:
            self.logger.error(f"System state monitor cleanup failed: {e}")

class SystemMonitorNode(Node):
    """ROS2 System Monitor Node"""

    def __init__(self,
                 node_name: str,
                 state_callback: Callable[[SystemStateInfo], None],
                 error_callback: Callable[[int, str], None]):
        """
        Initialize ROS2 monitor node

        Args:
            node_name: Node name
            state_callback: State update callback
            error_callback: Error callback
        """
        super().__init__(node_name)
        self.state_callback = state_callback
        self.error_callback = error_callback

        # Create subscribers
        self._create_subscribers()

        # State data
        self.last_low_state = None
        self.last_sport_state = None

        self.get_logger().info(f"System monitor node {node_name} started")

    def _create_subscribers(self) -> None:
        """Create ROS2 subscribers"""
        try:
            if UNITREE_MSGS_AVAILABLE:
                # Low-level state subscription
                self.low_state_sub = self.create_subscription(
                    LowState,
                    '/low_state',  # Adjust based on actual topic name
                    self._low_state_callback,
                    10
                )

                # Sport mode state subscription
                self.sport_state_sub = self.create_subscription(
                    SportModeState,
                    '/sportmodestate',  # Adjust based on actual topic name
                    self._sport_state_callback,
                    10
                )

                # Battery state subscription
                self.bms_state_sub = self.create_subscription(
                    BmsState,
                    '/bms_state',  # Adjust based on actual topic name
                    self._bms_state_callback,
                    10
                )
            else:
                # Simulation subscription - use standard message types
                self.sim_state_sub = self.create_subscription(
                    String,
                    '/simulation/system_state',
                    self._simulation_callback,
                    10
                )

            self.get_logger().info("ROS2 subscribers created successfully")

        except Exception as e:
            self.get_logger().error(f"Failed to create ROS2 subscribers: {e}")

    def _low_state_callback(self, msg) -> None:
        """Low-level state message callback"""
        try:
            self.last_low_state = msg

            # Parse system state (adjust based on actual message structure)
            battery_level = getattr(msg, 'battery_percentage', 0.85)
            battery_voltage = getattr(msg, 'battery_voltage', 24.0)
            temperature = getattr(msg, 'temperature', 30.0)
            error_codes = getattr(msg, 'error_codes', [])

            # Determine system state
            if error_codes:
                state = SystemState.ERROR
                priority = SystemLEDPriority.CRITICAL_ERROR
            elif battery_level <= 0.05:
                state = SystemState.EMERGENCY
                priority = SystemLEDPriority.CRITICAL_ERROR
            elif battery_level <= 0.15:
                state = SystemState.LOW_BATTERY
                priority = SystemLEDPriority.SAFETY_WARNING
            else:
                state = SystemState.NORMAL
                priority = SystemLEDPriority.NORMAL

            # Create state info
            state_info = SystemStateInfo(
                state=state,
                priority=priority,
                battery_level=battery_level,
                battery_voltage=battery_voltage,
                is_charging=getattr(msg, 'is_charging', False),
                error_codes=error_codes,
                temperature=temperature,
                timestamp=time.time(),
                source="low_state"
            )

            # Invoke callback
            self.state_callback(state_info)

        except Exception as e:
            self.get_logger().error(f"Low-level state callback failed: {e}")

    def _sport_state_callback(self, msg) -> None:
        """Sport state message callback"""
        try:
            self.last_sport_state = msg

            # Parse sport state (adjust based on actual message structure)
            gait_type = getattr(msg, 'gait_type', 0)
            is_standing = getattr(msg, 'mode', 0) == 1  # Assuming 1 is standing mode

            # If state info already exists, update motion-related fields
            if self.last_low_state:
                # Reuse latest low-level state info and update motion state
                pass  # Can be extended as needed

        except Exception as e:
            self.get_logger().error(f"Sport state callback failed: {e}")

    def _bms_state_callback(self, msg) -> None:
        """Battery management system state callback"""
        try:
            # Process detailed battery information
            battery_level = getattr(msg, 'soc', 0.85)  # State of Charge
            is_charging = getattr(msg, 'status', 0) == 2  # Assuming 2 is charging state

            # Can process more detailed battery information as needed

        except Exception as e:
            self.get_logger().error(f"Battery state callback failed: {e}")

    def _simulation_callback(self, msg: String) -> None:
        """Simulation state callback"""
        try:
            # Parse simulation data
            data = json.loads(msg.data)

            state_info = SystemStateInfo(
                state=SystemState[data.get('state', 'NORMAL')],
                priority=SystemLEDPriority[data.get('priority', 'NORMAL')],
                battery_level=data.get('battery_level', 0.85),
                battery_voltage=data.get('battery_voltage', 24.0),
                is_charging=data.get('is_charging', False),
                error_codes=data.get('error_codes', []),
                temperature=data.get('temperature', 30.0),
                timestamp=time.time(),
                source="simulation"
            )

            self.state_callback(state_info)

        except Exception as e:
            self.get_logger().error(f"Simulation callback failed: {e}")

# Factory function
def create_system_state_monitor(
    node_name: str = "claudia_system_monitor",
    history_size: int = 50,
    update_rate: float = 10.0) -> SystemStateMonitor:
    """
    Create system state monitor

    Args:
        node_name: ROS2 node name
        history_size: History data cache size
        update_rate: Update frequency (Hz)

    Returns:
        SystemStateMonitor: System state monitor instance
    """
    return SystemStateMonitor(
        node_name=node_name,
        history_size=history_size,
        update_rate=update_rate
    )

# Test and debug functionality
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    print("System State Monitor Test")

    monitor = create_system_state_monitor()

    def on_state_change(prev_state, new_state_info):
        print(f"State change: {prev_state.name} -> {new_state_info.state.name}")

    def on_critical_event(event_type, message, state_info):
        print(f"Critical event: {event_type} - {message}")

    monitor.register_state_change_callback(on_state_change)
    monitor.register_critical_event_callback(on_critical_event)

    if monitor.initialize():
        monitor.start_monitoring()

        try:
            # Run 5-minute test
            for i in range(30):
                time.sleep(10)
                decision = monitor.get_led_control_decision(5)
                print(f"LED control decision: {decision.allow_custom_control} - {decision.reason}")

                if i % 3 == 0:
                    stats = monitor.get_system_statistics()
                    print(f"Statistics: {stats}")

        except KeyboardInterrupt:
            print("Test interrupted")
        finally:
            monitor.cleanup()
    else:
        print("Monitor initialization failed")
