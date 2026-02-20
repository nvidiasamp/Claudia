# Robot Controller Module

Hardware interface layer for the Unitree Go2 robot.

## Modules

| File | Role |
|------|------|
| `system_state_monitor.py` | ROS2-based battery/posture monitoring at 5Hz |
| `unified_led_controller.py` | High-level LED mode API (thinking/success/error/listening) |
| `led_controller.py` | Low-level LED control via LowCmd DDS messages |
| `led_state_machine.py` | State-based LED transitions with priority |
| `led_patterns.py` | LED pattern definitions and VUI client integration |
| `unitree_messages.py` | Unitree DDS message type definitions |

## LED System

The LED system has three layers:
1. **unified_led_controller.py** — Mode API (`set_mode("thinking")`)
2. **led_state_machine.py** — Priority-based state transitions
3. **led_controller.py** — Direct DDS LowCmd writes

## State Monitoring

`SystemStateMonitor` subscribes to ROS2 topics for:
- Battery level (from LowState BMS)
- Robot posture (from SportModeState)
- Updates at 5Hz for real-time safety decisions
