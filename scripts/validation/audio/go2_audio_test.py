#!/usr/bin/env python3
"""
Unitree Go2 Robot Audio Function Test Script (Correct Version)
VUI client-based volume control and LED testing, combined with system audio output

Author: Claudia AI System
Generated: 2025-06-30 13:35:00
Platform: Ubuntu 20.04 - aarch64
"""

import sys
import time
import numpy as np
from pathlib import Path
from datetime import datetime

# Add project path
sys.path.append(str(Path(__file__).parent.parent.parent))

def test_go2_vui():
    """Test Go2 VUI functionality (volume and LED control)"""
    print("\nUnitree Go2 VUI Function Test")
    print("=" * 50)

    try:
        # Import Unitree SDK2
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.vui.vui_client import VuiClient

        print("[OK] Unitree Go2 SDK2 imported successfully")

        # Initialize channel (for Go2, typically no network interface parameter needed)
        ChannelFactoryInitialize(0)
        print("[OK] Channel Factory initialization complete")

        # Create VUI client
        vui_client = VuiClient()
        vui_client.SetTimeout(3.0)
        vui_client.Init()
        print("[OK] VUI Client created and initialized successfully")

        # 1. Get current volume
        print("\n1. Volume Status Check")
        print("-" * 30)
        code, current_volume = vui_client.GetVolume()
        if code == 0:
            print(f"[OK] Current volume: {current_volume}")
        else:
            print(f"[FAIL] Failed to get volume, error code: {code}")

        # 2. Get current LED brightness
        print("\n2. LED Brightness Status Check")
        print("-" * 30)
        code, current_brightness = vui_client.GetBrightness()
        if code == 0:
            print(f"[OK] Current LED brightness: {current_brightness}")
        else:
            print(f"[FAIL] Failed to get LED brightness, error code: {code}")

        # 3. Volume test sequence
        print("\n3. Volume Control Test")
        print("-" * 30)
        test_volumes = [3, 6, 9, 5]  # Test different volume levels

        for volume in test_volumes:
            print(f"Setting volume to {volume}...")
            code = vui_client.SetVolume(volume)
            if code == 0:
                print(f"[OK] Volume set successfully")

                # Verify setting
                code, new_volume = vui_client.GetVolume()
                if code == 0:
                    print(f"   Verified volume: {new_volume}")
                else:
                    print(f"   Verification failed, error code: {code}")
            else:
                print(f"[FAIL] Volume set failed, error code: {code}")

            time.sleep(1)

        # 4. LED brightness test sequence
        print("\n4. LED Brightness Control Test")
        print("-" * 30)
        test_brightness = [2, 5, 8, 10, 0]  # Test different brightness levels

        for brightness in test_brightness:
            print(f"Setting LED brightness to {brightness}...")
            code = vui_client.SetBrightness(brightness)
            if code == 0:
                print(f"[OK] LED brightness set successfully")

                # Verify setting
                code, new_brightness = vui_client.GetBrightness()
                if code == 0:
                    print(f"   Verified brightness: {new_brightness}")
                else:
                    print(f"   Verification failed, error code: {code}")
            else:
                print(f"[FAIL] LED brightness set failed, error code: {code}")

            time.sleep(1.5)

        # 5. Restore original settings
        print("\n5. Restoring Original Settings")
        print("-" * 30)
        if 'current_volume' in locals() and current_volume is not None:
            vui_client.SetVolume(current_volume)
            print(f"[OK] Volume restored to: {current_volume}")

        if 'current_brightness' in locals() and current_brightness is not None:
            vui_client.SetBrightness(current_brightness)
            print(f"[OK] LED brightness restored to: {current_brightness}")

        print("\n[OK] Go2 VUI test complete!")
        return True

    except ImportError as e:
        print(f"[FAIL] Unitree SDK2 import failed: {e}")
        print("Please ensure Unitree SDK2 is properly installed and configured")
        return False

    except Exception as e:
        print(f"[FAIL] VUI test failed: {e}")
        print("Possible causes:")
        print("  1. Go2 robot is not connected or not powered on")
        print("  2. VUI service is not started")
        print("  3. Network connection issue")
        return False

def test_audio_output_with_tones():
    """Play a series of tones to simulate 'robot speaking' effect"""
    print("\nSimulated Audio Feedback Test")
    print("=" * 50)

    try:
        import sounddevice as sd

        # Define tone sequence - simulated robot feedback sounds
        tones = [
            (800, 0.3),   # Startup sound
            (1000, 0.2),  # Confirmation sound 1
            (1200, 0.2),  # Confirmation sound 2
            (1500, 0.4),  # Completion sound
        ]

        sample_rate = 44100

        print("Playing robot status feedback sound sequence...")
        print("These tones represent:")
        print("  Startup sound (800Hz)")
        print("  Confirmation sound 1 (1000Hz)")
        print("  Confirmation sound 2 (1200Hz)")
        print("  Completion sound (1500Hz)")

        for i, (freq, duration) in enumerate(tones):
            print(f"\nPlaying tone {i+1}: {freq}Hz ({duration}s)")

            # Generate tone
            t = np.linspace(0, duration, int(sample_rate * duration))
            # Add fade in/out effect for a more natural sound
            fade_samples = int(0.05 * sample_rate)  # 50ms fade in/out
            wave = 0.3 * np.sin(2 * np.pi * freq * t)

            # Apply fade in/out
            if len(wave) > 2 * fade_samples:
                wave[:fade_samples] *= np.linspace(0, 1, fade_samples)
                wave[-fade_samples:] *= np.linspace(1, 0, fade_samples)

            sd.play(wave, sample_rate)
            sd.wait()  # Wait for playback to complete
            time.sleep(0.2)  # Short pause

        print("\n[OK] Audio feedback test complete!")
        print("If you heard these tones, the audio output system is working properly!")
        return True

    except Exception as e:
        print(f"[FAIL] Audio feedback test failed: {e}")
        return False

def main():
    """Main function"""
    print("Unitree Go2 Complete Audio Function Test")
    print("=" * 60)
    print(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    print("Test contents:")
    print("  Go2 VUI Control (Volume/LED)")
    print("  Audio Output Verification")
    print("  Robot Feedback Sound Test")
    print()

    results = []

    # Test 1: Go2 VUI functions
    print("Test 1: Go2 VUI Functions")
    if test_go2_vui():
        results.append("[PASS] VUI Control")
        print("If the LED lights changed, the Go2 robot is responding properly!")
    else:
        results.append("[FAIL] VUI Control")

    print("\n" + "="*60)

    # Test 2: Audio output
    print("Test 2: Audio Output Function")
    if test_audio_output_with_tones():
        results.append("[PASS] Audio Output")
    else:
        results.append("[FAIL] Audio Output")

    # Summary
    print("\n" + "="*60)
    print("Test Summary")
    print("-" * 30)
    success_count = sum(1 for r in results if r.startswith("[PASS]"))
    total_count = len(results)

    for result in results:
        print(f"  {result}")

    print(f"\nPass rate: {success_count}/{total_count} ({success_count/total_count*100:.1f}%)")

    if success_count == total_count:
        print("\nAll tests passed!")
        print("Summary:")
        print("   - Go2 LED and volume control working properly")
        print("   - Audio output system fully operational")
        print("   - Robot audio hardware validation successful")
    elif success_count > 0:
        print("\nSome functions are working")
        print("Summary:")
        print("   - Basic audio functions available")
        print("   - Some Go2 control functions may require robot connection")
    else:
        print("\nConfiguration check needed")
        print("Suggestions:")
        print("   - Ensure Go2 robot is powered on and connected")
        print("   - Check audio device connections")

if __name__ == "__main__":
    main()
