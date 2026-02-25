#!/usr/bin/env python3
"""
Unitree Go2 Robot Text-to-Speech (TTS) Test Script
Makes the robot actually produce sound for verification

Based on Unitree SDK2 AudioClient API
Includes TTS, volume control, and LED light synchronization tests

Author: Claudia AI System
Generated: 2025-06-30 13:30:00
Platform: Ubuntu 20.04 - aarch64
"""

import sys
import time
import json
from pathlib import Path
from datetime import datetime

# Add project path
sys.path.append(str(Path(__file__).parent.parent.parent))

def test_unitree_tts():
    """Test Unitree Go2 TTS functionality"""
    print("\nUnitree Go2 TTS Text-to-Speech Test")
    print("=" * 50)

    try:
        # Import Unitree SDK2
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.audio.audio_client import AudioClient

        print("[OK] Unitree SDK2 imported successfully")

        # Initialize channel
        ChannelFactoryInitialize(0, "")
        print("[OK] Channel Factory initialization complete")

        # Create audio client
        audio_client = AudioClient()
        print("[OK] AudioClient created successfully")

        # 1. Get current volume
        print("\n1. Volume Status Check")
        print("-" * 30)
        volume_result = audio_client.GetVolume()
        print(f"Current volume: {volume_result}")

        # 2. Set appropriate volume
        print("\n2. Setting volume to 80")
        print("-" * 30)
        audio_client.SetVolume(80)
        time.sleep(0.5)

        # Verify volume setting
        new_volume = audio_client.GetVolume()
        print(f"New volume: {new_volume}")

        # 3. TTS test - Chinese
        print("\n3. Chinese TTS Test")
        print("-" * 30)
        print("Robot will say: 'Hello! I am the Unitree Go2 robot, audio system validation successful!'")

        tts_result = audio_client.TtsMaker("Hello! I am the Unitree Go2 robot, audio system validation successful!", 0)
        print(f"TTS result: {tts_result}")
        time.sleep(3)  # Wait for playback to complete

        # 4. TTS test - English
        print("\n4. English TTS Test")
        print("-" * 30)
        print("Robot will say: 'Hello! I am Unitree Go2 robot. Audio validation successful!'")

        tts_result = audio_client.TtsMaker("Hello! I am Unitree Go2 robot. Audio validation successful!", 0)
        print(f"TTS result: {tts_result}")
        time.sleep(3)

        # 5. LED light synchronization test
        print("\n5. LED Light Synchronization Test")
        print("-" * 30)

        colors = [
            (255, 0, 0, "Red"),
            (0, 255, 0, "Green"),
            (0, 0, 255, "Blue"),
            (255, 255, 0, "Yellow"),
            (255, 0, 255, "Purple")
        ]

        for r, g, b, color_name in colors:
            print(f"Setting LED to {color_name}...")
            audio_client.LedControl(r, g, b)
            audio_client.TtsMaker(f"Current LED color is {color_name}", 0)
            time.sleep(2)

        # 6. Completion notification
        print("\n6. Test Completion Notification")
        print("-" * 30)
        audio_client.LedControl(0, 255, 0)  # Green indicates success
        audio_client.TtsMaker("Audio validation test complete! All functions working properly!", 0)
        time.sleep(3)

        # Turn off LED
        audio_client.LedControl(0, 0, 0)

        print("\n[OK] TTS test complete!")
        print("If you heard the robot's voice, the audio output function is working properly!")

        return True

    except ImportError as e:
        print(f"[FAIL] Unitree SDK2 import failed: {e}")
        print("Please ensure Unitree SDK2 is properly installed and configured")
        return False

    except Exception as e:
        print(f"[FAIL] TTS test failed: {e}")
        print("Possible causes:")
        print("  1. Robot is not connected or not powered on")
        print("  2. Network connection issue")
        print("  3. AudioClient service is not started")
        return False

def test_basic_audio_output():
    """Test basic audio output functionality (fallback)"""
    print("\nBasic Audio Output Test")
    print("=" * 50)

    try:
        import sounddevice as sd
        import numpy as np

        # Generate test tone
        duration = 2  # seconds
        sample_rate = 44100
        frequency = 1000  # 1kHz

        t = np.linspace(0, duration, int(sample_rate * duration))
        wave = 0.3 * np.sin(2 * np.pi * frequency * t)

        print(f"Playing {frequency}Hz test tone ({duration}s)...")
        print("You should hear a pure tone...")

        sd.play(wave, sample_rate)
        sd.wait()  # Wait for playback to complete

        print("[OK] Basic audio output test complete")
        return True

    except Exception as e:
        print(f"[FAIL] Basic audio output test failed: {e}")
        return False

def main():
    """Main function"""
    print("Unitree Go2 Audio Output Test")
    print("=" * 60)
    print(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print()

    # First try Unitree TTS
    print("Trying Option 1: Unitree AudioClient TTS")
    if test_unitree_tts():
        print("\nUnitree TTS test successful! You should have heard the robot's voice.")
    else:
        print("\n[WARN] Unitree TTS test failed, trying fallback option...")

        # Fallback: basic audio output
        print("\nTrying Option 2: Basic Audio Output")
        if test_basic_audio_output():
            print("\n[OK] Basic audio output successful!")
            print("Suggestions:")
            print("   - Ensure Go2 robot is powered on and connected")
            print("   - Check network connection status")
            print("   - Verify AudioClient service is started")
        else:
            print("\n[FAIL] All audio output tests failed")
            print("Suggestions:")
            print("   - Check audio device connections")
            print("   - Check speaker volume settings")
            print("   - Check audio driver status")

if __name__ == "__main__":
    main()
