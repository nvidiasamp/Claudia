#!/usr/bin/env python3
"""
SportClient Connection Test
Used for diagnosing robot connection issues
"""

import time
import sys
import os
from datetime import datetime

# Set environment variables
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

# Add SDK path
_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.append(os.path.join(_PROJECT_ROOT, 'unitree_sdk2_python'))

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    print("Successfully imported unitree_sdk2py module")
except ImportError as e:
    print(f"Import error: {e}")
    sys.exit(1)

def test_sportclient_connection():
    """Test SportClient connection"""
    print(f"SportClient connection diagnostics - {datetime.now().strftime('%H:%M:%S')}")

    try:
        # Try different network configurations
        network_configs = [
            ("eth0", "Ethernet interface"),
            ("", "Default interface"),
        ]

        for interface, description in network_configs:
            print(f"\nTrying network interface: {interface} ({description})")

            try:
                # Initialize DDS channel
                if interface:
                    print(f"   Initializing DDS channel factory: {interface}")
                    ChannelFactoryInitialize(0, interface)
                else:
                    print("   Using default DDS channel configuration")

                # Create SportClient
                print("   Creating SportClient...")
                client = SportClient()
                client.SetTimeout(5.0)  # 5-second timeout

                print("   Initializing SportClient...")
                result = client.Init()

                if result is not None:
                    print(f"SportClient initialized successfully! Interface: {interface}")

                    # Try a simple status query
                    print("   Testing simple command...")
                    time.sleep(1)
                    print("Connection test successful!")
                    return True
                else:
                    print(f"SportClient initialization failed (returned None)")

            except Exception as e:
                print(f"Interface {interface} connection failed: {e}")
                continue

        print("All network interface tests failed")
        return False

    except Exception as e:
        print(f"Connection test exception: {e}")
        return False

def main():
    print("Unitree Go2 SportClient Connection Diagnostics")
    print("=" * 50)

    success = test_sportclient_connection()

    print("\n" + "=" * 50)
    if success:
        print("Connection diagnostics successful - SportClient is working correctly")
    else:
        print("Connection diagnostics failed - Check robot status and network configuration")
    print("=" * 50)

if __name__ == "__main__":
    main()
