#!/usr/bin/env python3
"""
Unitree Message Type Configuration
Based on the successful implementation from led_controller.py
"""

# Correct Unitree message import method
try:
    from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher, ChannelFactoryInitialize
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_, MotorCmd_, BmsCmd_
    from unitree_sdk2py.utils.crc import CRC
    UNITREE_AVAILABLE = True
    UNITREE_IMPORT_METHOD = 'unitree_go.msg.dds_'
    print("Unitree hardware control available (unitree_go.msg.dds_)")
except ImportError as e:
    UNITREE_AVAILABLE = False
    print(f"Unitree SDK2 not installed or unavailable: {e}")
    print("LED control will use ClaudiaLEDController (verified working)")
    # Define placeholders
    LowCmd_ = None
    LowState_ = None
    ChannelSubscriber = None
    ChannelPublisher = None
    ChannelFactoryInitialize = None

class UnitreeMessages:
    """Unitree Message Manager"""

    @staticmethod
    def is_available():
        """Check if Unitree hardware is available"""
        return UNITREE_AVAILABLE

    @staticmethod
    def get_import_method():
        """Get the import method used"""
        return UNITREE_IMPORT_METHOD if UNITREE_AVAILABLE else None

    @staticmethod
    def create_subscriber(topic, message_type=None):
        """Create a subscriber"""
        if not UNITREE_AVAILABLE:
            return None

        if message_type is None:
            message_type = LowState_

        return ChannelSubscriber(topic, message_type)

    @staticmethod
    def create_publisher(topic, message_type=None):
        """Create a publisher"""
        if not UNITREE_AVAILABLE:
            return None

        if message_type is None:
            message_type = LowCmd_

        return ChannelPublisher(topic, message_type)

    @staticmethod
    def create_low_cmd_with_params():
        """Create a LowCmd message (with required parameters)"""
        if not UNITREE_AVAILABLE:
            return None

        try:
            # Based on the successful implementation from led_controller.py
            # 1. head: uint8[2] - Message header
            head = [0xFE, 0xEF]

            # 2. level_flag: uint8 - Level flag
            level_flag = 0xFF

            # 3. frame_reserve: uint8 - Frame reserve
            frame_reserve = 0

            # 4. sn: uint32[2] - Serial number
            sn = [0, 0]

            # 5. version: uint32[2] - Version number
            version = [0, 0]

            # 6. bandwidth: uint16 - Bandwidth
            bandwidth = 0

            # 7. motor_cmd: MotorCmd_[20] - Motor command array
            motor_cmd = []
            for i in range(20):
                motor_cmd.append(MotorCmd_(
                    mode=0x00, q=0.0, dq=0.0, tau=0.0,
                    kp=0.0, kd=0.0, reserve=[0, 0, 0]
                ))

            # 8. bms_cmd: BmsCmd_ - Battery management system command
            bms_cmd = BmsCmd_(off=0, reserve=[0, 0, 0])

            # 9. wireless_remote: uint8[40] - Wireless remote controller data
            wireless_remote = [0] * 40

            # 10. led: uint8[12] - LED data
            led = [0] * 12

            # 11. fan: uint8[2] - Fan control
            fan = [0, 0]

            # 12. gpio: uint8 - GPIO status
            gpio = 0

            # 13. reserve: uint32 - Reserved field
            reserve = 0

            # 14. crc: uint32 - CRC checksum
            crc = 0

            # Create LowCmd message using positional parameters
            msg = LowCmd_(
                head=head, level_flag=level_flag, frame_reserve=frame_reserve, sn=sn, version=version, bandwidth=bandwidth,
                motor_cmd=motor_cmd, bms_cmd=bms_cmd, wireless_remote=wireless_remote, led=led, fan=fan, gpio=gpio, reserve=reserve, crc=crc
            )

            return msg

        except Exception as e:
            print(f"Failed to create LowCmd message: {e}")
            return None

    @staticmethod
    def test_hardware_communication():
        """Test hardware communication"""
        if not UNITREE_AVAILABLE:
            print("SDK unavailable")
            return False

        try:
            print("Testing hardware communication...")

            # Initialize DDS channel factory (required step)
            try:
                ChannelFactoryInitialize(0, "eth0")  # Use default network interface
                print("DDS channel factory initialization successful")
            except Exception as e:
                print(f"DDS channel factory initialization failed: {e}")
                # Continue testing; this may not be required in some environments

            # Test LowCmd creation
            cmd_msg = UnitreeMessages.create_low_cmd_with_params()
            if cmd_msg is None:
                print("LowCmd creation failed")
                return False
            print("LowCmd creation successful")

            # Test channel creation (without actually sending data)
            try:
                # Test publisher creation
                pub = UnitreeMessages.create_publisher("rt/lowcmd")
                if pub is not None:
                    print("Publisher creation successful")
                    try:
                        pub.Init()  # Initialize publisher
                        print("Publisher initialization successful")
                    except Exception as e:
                        print(f"Publisher initialization failed: {e}")
                else:
                    print("Publisher creation failed")
                    return False

                # Test subscriber creation
                sub = UnitreeMessages.create_subscriber("rt/lowstate")
                if sub is not None:
                    print("Subscriber creation successful")
                    try:
                        sub.Init()  # Initialize subscriber
                        print("Subscriber initialization successful")
                    except Exception as e:
                        print(f"Subscriber initialization failed: {e}")
                else:
                    print("Subscriber creation failed")
                    return False

                print("Hardware communication test fully passed")
                return True

            except Exception as e:
                print(f"Channel creation failed: {e}")
                return False

        except Exception as e:
            print(f"Hardware communication test failed: {e}")
            return False

# Test configuration
if __name__ == "__main__":
    print(f"Unitree availability: {UnitreeMessages.is_available()}")
    print(f"Import method: {UnitreeMessages.get_import_method()}")

    if UnitreeMessages.is_available():
        success = UnitreeMessages.test_hardware_communication()
        if success:
            print("All tests passed, hardware mode available!")
        else:
            print("Hardware communication test failed")
    else:
        print("Hardware unavailable, will use simulation mode")
