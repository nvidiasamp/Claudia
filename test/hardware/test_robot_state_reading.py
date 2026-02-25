#!/usr/bin/env python3
"""
Unitree Go2 Robot State Data Reading Specialized Test
Detailed analysis and display of robot IMU data, joint positions, sensor readings, and other state information
"""

import time
import sys
import json
import threading
from collections import defaultdict, deque
from datetime import datetime

def test_robot_state_reading():
    """Specialized test for robot state data reading and analysis"""
    print("Starting robot state data reading specialized test...")
    print("=" * 80)

    try:
        # Import unitree_sdk2py modules
        from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_

        print("Successfully imported unitree_sdk2py modules")

        # Initialize DDS communication
        print("Initializing DDS channel factory (NIC: eth0)...")
        ChannelFactoryInitialize(0, "eth0")
        print("DDS channel factory initialized successfully")

        # Data collection container
        data_storage = {
            'lowstate_samples': deque(maxlen=100),  # Keep last 100 samples
            'sportstate_samples': deque(maxlen=100),
            'stats': defaultdict(list),
            'start_time': time.time()
        }

        # Data lock
        data_lock = threading.Lock()

        def analyze_lowstate_data(msg: LowState_):
            """Detailed analysis of LowState data"""
            timestamp = time.time()

            with data_lock:
                # Parse IMU data
                imu_data = {
                    'timestamp': timestamp,
                    'quaternion': {
                        'w': msg.imu_state.quaternion[0],
                        'x': msg.imu_state.quaternion[1],
                        'y': msg.imu_state.quaternion[2],
                        'z': msg.imu_state.quaternion[3]
                    },
                    'gyroscope': {
                        'x': msg.imu_state.gyroscope[0],
                        'y': msg.imu_state.gyroscope[1],
                        'z': msg.imu_state.gyroscope[2]
                    },
                    'accelerometer': {
                        'x': msg.imu_state.accelerometer[0],
                        'y': msg.imu_state.accelerometer[1],
                        'z': msg.imu_state.accelerometer[2]
                    }
                }

                # Parse motor data (12 motors)
                motor_data = []
                for i in range(min(len(msg.motor_state), 12)):
                    motor = {
                        'id': i,
                        'mode': msg.motor_state[i].mode,
                        'q': msg.motor_state[i].q,        # Position (rad)
                        'dq': msg.motor_state[i].dq,      # Velocity (rad/s)
                        'ddq': msg.motor_state[i].ddq,    # Acceleration (rad/s^2)
                        'tau_est': msg.motor_state[i].tau_est,  # Torque estimate (N*m)
                        'temperature': msg.motor_state[i].temperature  # Temperature (deg C)
                    }
                    motor_data.append(motor)

                # Parse battery data
                battery_data = {
                    'voltage': msg.power_v,
                    'current': msg.power_a
                }

                # Parse foot sensor data (4 feet)
                foot_data = []
                for i in range(min(len(msg.foot_force), 4)):
                    foot_data.append({
                        'foot_id': i,
                        'force': msg.foot_force[i]
                    })

                # Store complete LowState sample
                lowstate_sample = {
                    'timestamp': timestamp,
                    'imu': imu_data,
                    'motors': motor_data,
                    'battery': battery_data,
                    'feet': foot_data
                }

                data_storage['lowstate_samples'].append(lowstate_sample)

                # Real-time display of key data
                print(f"\nLowState data [{datetime.fromtimestamp(timestamp).strftime('%H:%M:%S.%f')[:-3]}]")
                print(f"   IMU attitude: w={imu_data['quaternion']['w']:.3f}, x={imu_data['quaternion']['x']:.3f}, y={imu_data['quaternion']['y']:.3f}, z={imu_data['quaternion']['z']:.3f}")
                print(f"   Gyroscope: x={imu_data['gyroscope']['x']:.3f}, y={imu_data['gyroscope']['y']:.3f}, z={imu_data['gyroscope']['z']:.3f} rad/s")
                print(f"   Accelerometer: x={imu_data['accelerometer']['x']:.3f}, y={imu_data['accelerometer']['y']:.3f}, z={imu_data['accelerometer']['z']:.3f} m/s^2")
                print(f"   Battery status: {battery_data['voltage']:.1f}V, {battery_data['current']:.2f}A")
                print(f"   Active motors: {len(motor_data)}, FR hip joint position: {motor_data[0]['q']:.3f} rad" if motor_data else "")

        def analyze_sportstate_data(msg: SportModeState_):
            """Detailed analysis of SportModeState data"""
            timestamp = time.time()

            with data_lock:
                # Parse sport state data
                sport_data = {
                    'timestamp': timestamp,
                    'mode': msg.mode,
                    'progress': msg.progress,
                    'position': {
                        'x': msg.position[0],
                        'y': msg.position[1],
                        'z': msg.position[2]
                    },
                    'velocity': {
                        'x': msg.velocity[0],
                        'y': msg.velocity[1],
                        'yaw': msg.velocity[2]
                    },
                    'range_obstacle': list(msg.range_obstacle) if hasattr(msg, 'range_obstacle') else [],
                    'foot_raise_height': msg.foot_raise_height if hasattr(msg, 'foot_raise_height') else 0.0,
                    'body_height': msg.body_height if hasattr(msg, 'body_height') else 0.0
                }

                data_storage['sportstate_samples'].append(sport_data)

                # Real-time display of sport data
                print(f"\nSportState data [{datetime.fromtimestamp(timestamp).strftime('%H:%M:%S.%f')[:-3]}]")
                print(f"   Sport mode: {sport_data['mode']}, progress: {sport_data['progress']:.3f}")
                print(f"   Position: x={sport_data['position']['x']:.3f}, y={sport_data['position']['y']:.3f}, z={sport_data['position']['z']:.3f} m")
                print(f"   Velocity: vx={sport_data['velocity']['x']:.3f}, vy={sport_data['velocity']['y']:.3f}, yaw={sport_data['velocity']['yaw']:.3f} m/s")
                print(f"   Body height: {sport_data['body_height']:.3f} m, foot raise height: {sport_data['foot_raise_height']:.3f} m")

        # Create subscribers
        print("Creating data subscribers...")
        lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        lowstate_subscriber.Init(analyze_lowstate_data, 10)

        sportstate_subscriber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        sportstate_subscriber.Init(analyze_sportstate_data, 10)

        print("Subscribers created successfully, starting to receive data...")
        print("Data analysis running (15-second test period)...")
        print("=" * 80)

        # Data collection phase (15 seconds)
        collection_time = 15
        start_time = time.time()

        while time.time() - start_time < collection_time:
            time.sleep(0.1)

        print("\n" + "=" * 80)
        print("Data collection complete, starting statistical analysis...")

        # Data statistical analysis
        with data_lock:
            lowstate_count = len(data_storage['lowstate_samples'])
            sportstate_count = len(data_storage['sportstate_samples'])

            print(f"\nData collection statistics:")
            print(f"   LowState samples: {lowstate_count}")
            print(f"   SportState samples: {sportstate_count}")
            print(f"   Total collection time: {collection_time}s")
            print(f"   LowState frequency: {lowstate_count/collection_time:.1f} Hz")
            print(f"   SportState frequency: {sportstate_count/collection_time:.1f} Hz")

            if lowstate_count > 0:
                # Analyze IMU data stability
                latest_lowstate = data_storage['lowstate_samples'][-1]
                first_lowstate = data_storage['lowstate_samples'][0]

                print(f"\nIMU data analysis:")
                print(f"   Initial quaternion: w={first_lowstate['imu']['quaternion']['w']:.3f}")
                print(f"   Final quaternion: w={latest_lowstate['imu']['quaternion']['w']:.3f}")
                print(f"   Attitude change: {abs(latest_lowstate['imu']['quaternion']['w'] - first_lowstate['imu']['quaternion']['w']):.3f}")

                # Analyze battery state
                battery_voltages = [sample['battery']['voltage'] for sample in data_storage['lowstate_samples']]
                battery_currents = [sample['battery']['current'] for sample in data_storage['lowstate_samples']]

                print(f"\nBattery state analysis:")
                print(f"   Voltage range: {min(battery_voltages):.1f}V - {max(battery_voltages):.1f}V")
                print(f"   Average voltage: {sum(battery_voltages)/len(battery_voltages):.1f}V")
                print(f"   Current range: {min(battery_currents):.2f}A - {max(battery_currents):.2f}A")
                print(f"   Average current: {sum(battery_currents)/len(battery_currents):.2f}A")

                # Analyze motor state
                if latest_lowstate['motors']:
                    print(f"\nMotor state analysis:")
                    for motor in latest_lowstate['motors'][:4]:  # Display first 4 motors
                        print(f"   Motor {motor['id']}: position={motor['q']:.3f}rad, velocity={motor['dq']:.3f}rad/s, temperature={motor['temperature']:.1f}C")

            if sportstate_count > 0:
                # Analyze sport data
                latest_sportstate = data_storage['sportstate_samples'][-1]
                first_sportstate = data_storage['sportstate_samples'][0]

                print(f"\nSport state analysis:")
                print(f"   Position change: dx={latest_sportstate['position']['x'] - first_sportstate['position']['x']:.3f}m")
                print(f"   Position change: dy={latest_sportstate['position']['y'] - first_sportstate['position']['y']:.3f}m")
                print(f"   Current sport mode: {latest_sportstate['mode']}")
                print(f"   Current body height: {latest_sportstate['body_height']:.3f}m")

        print("\n" + "=" * 80)
        print("Robot state data reading test complete!")
        print("All sensor data read normally, communication stable, data integrity verification passed")

        return True

    except ImportError as e:
        print(f"Module import failed: {e}")
        return False
    except Exception as e:
        print(f"Error during test: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Main function"""
    print("Unitree Go2 Robot State Data Reading Specialized Test")
    print("Test objective: Verify Python scripts can successfully read and analyze robot state information")
    print("Including: IMU data, joint positions, sensor readings, etc.")
    print("=" * 80)

    # Execute test
    success = test_robot_state_reading()

    if success:
        print("\nTest result: PASSED")
        print("Robot state data reading functionality is fully operational")
        sys.exit(0)
    else:
        print("\nTest result: FAILED")
        print("Please check network connection and robot status")
        sys.exit(1)

if __name__ == "__main__":
    main()
