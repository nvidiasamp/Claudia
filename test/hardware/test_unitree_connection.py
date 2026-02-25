#!/usr/bin/env python3
"""
Unitree Go2 Robot Basic Communication Test
Tests connection and basic functionality with the robot
Based on correct API usage from official example code
"""

import time
import sys

def test_robot_connection():
    """Test basic connection with Go2 robot"""
    print("Starting Unitree Go2 robot connection test...")

    try:
        # Use correct import method (based on official example)
        from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_

        print("Successfully imported unitree_sdk2py modules")

        print("Initializing DDS channel factory...")
        # Initialize channel factory, parameters: domain_id=0, interface="eth0"
        ChannelFactoryInitialize(0, "eth0")
        print("DDS channel factory initialized successfully")

        # Create state data container
        received_data = {'lowstate': False, 'sportstate': False}

        def low_state_handler(msg: LowState_):
            print(f"Received LowState data!")
            print(f"   IMU: x={msg.imu_state.quaternion[0]:.3f}, y={msg.imu_state.quaternion[1]:.3f}")
            print(f"   Battery: voltage={msg.power_v:.1f}V, current={msg.power_a:.1f}A")
            print(f"   Motor state (FR_0): position={msg.motor_state[0].q:.3f}, velocity={msg.motor_state[0].dq:.3f}")
            received_data['lowstate'] = True

        def sport_state_handler(msg: SportModeState_):
            print(f"Received SportModeState data!")
            print(f"   Mode: {msg.mode}")
            print(f"   Progress: {msg.progress}")
            if hasattr(msg, 'velocity') and len(msg.velocity) >= 3:
                print(f"   Velocity: x={msg.velocity[0]:.3f}, y={msg.velocity[1]:.3f}, yaw={msg.velocity[2]:.3f}")
            if hasattr(msg, 'position') and len(msg.position) >= 3:
                print(f"   Position: x={msg.position[0]:.3f}, y={msg.position[1]:.3f}, z={msg.position[2]:.3f}")
            received_data['sportstate'] = True

        # Create subscribers (using official API)
        print("Creating low-level state subscriber...")
        lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        lowstate_subscriber.Init(low_state_handler, 10)
        print("Low-level state subscriber created successfully")

        print("Creating sport state subscriber...")
        sportstate_subscriber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        sportstate_subscriber.Init(sport_state_handler, 10)
        print("Sport state subscriber created successfully")

        print("Waiting for robot state data...")
        print("(Note: If the robot is not connected, this will timeout after 10 seconds)")

        start_time = time.time()
        timeout = 10  # 10-second timeout

        while time.time() - start_time < timeout:
            # Use official example read method
            try:
                # Try to read low-level state
                lowstate_msg = lowstate_subscriber.Read()
                if lowstate_msg is not None:
                    low_state_handler(lowstate_msg)

                # Try to read sport state
                sportstate_msg = sportstate_subscriber.Read()
                if sportstate_msg is not None:
                    sport_state_handler(sportstate_msg)

                # If any data received, consider connection successful
                if received_data['lowstate'] or received_data['sportstate']:
                    return True

            except Exception as e:
                print(f"Error reading data: {e}")

            time.sleep(0.1)

        print("Timeout: No robot state data received")
        print("   Possible causes:")
        print("   1. Robot is not connected or not powered on")
        print("   2. Network configuration is incorrect (current NIC: eth0)")
        print("   3. Robot is not on the same subnet")
        print("   4. DDS domain configuration mismatch")
        print("   5. Firewall is blocking DDS communication")
        return False

    except ImportError as e:
        print(f"Import failed: {e}")
        print("Please confirm unitree_sdk2py is correctly installed")
        return False
    except Exception as e:
        print(f"Connection test failed: {e}")
        print(f"   Error type: {type(e).__name__}")
        import traceback
        traceback.print_exc()
        return False

def test_environment_setup():
    """Test environment configuration"""
    print("Testing environment configuration...")

    import os
    rmw_impl = os.environ.get('RMW_IMPLEMENTATION', 'not set')
    print(f"   RMW_IMPLEMENTATION: {rmw_impl}")

    if rmw_impl != 'rmw_cyclonedds_cpp':
        print("WARNING: RMW_IMPLEMENTATION is not set to rmw_cyclonedds_cpp")
        return False

    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        print("Successfully imported core channel module")
        return True
    except Exception as e:
        print(f"Environment configuration test failed: {e}")
        return False

def main():
    """Main function"""
    print("=" * 60)
    print("Unitree Go2 Robot Communication Test")
    print("Based on correct official API implementation")
    print("=" * 60)

    # Display system information
    print(f"Python version: {sys.version}")
    print(f"Test time: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print()

    # Environment configuration test
    env_success = test_environment_setup()
    print()

    if not env_success:
        print("Environment configuration is incorrect, cannot continue testing")
        return 1

    # Execute connection test
    print("Starting robot connection test...")
    success = test_robot_connection()

    print()
    print("=" * 60)
    if success:
        print("Test complete: Robot communication is working!")
        print("Successfully established DDS communication with Unitree Go2")
        print("Next steps:")
        print("  - Test basic control commands (stand/lie down)")
        print("  - Verify sensor data quality")
        print("  - Test real-time control responsiveness")
    else:
        print("Test failed: Unable to communicate with robot")
        print("Troubleshooting steps:")
        print("1. Confirm robot is powered on and in normal state")
        print("2. Check network connection: ping 192.168.123.xxx (robot IP)")
        print("3. Confirm same subnet: robot and development machine")
        print("4. Check firewall settings, allow DDS communication")
        print("5. Confirm environment variable: export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp")
    print("=" * 60)

    return 0 if success else 1

if __name__ == "__main__":
    exit(main())
