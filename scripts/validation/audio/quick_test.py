#!/usr/bin/env python3
"""
Unitree Go2 Audio I/O System Quick Test Script
Demonstrates the basic functionality of the audio validation system

Usage:
python3 quick_test.py

Author: Claudia AI System
Generated: 2025-06-30 13:12:00
Platform: Ubuntu 20.04 - aarch64
"""

import sys
import time
from pathlib import Path

# Add current directory to Python path
sys.path.append(str(Path(__file__).parent))

try:
    from audio_validation_main import AudioValidationMain
    print("[OK] Audio validation module imported successfully")
except ImportError as e:
    print(f"[FAIL] Import failed: {e}")
    print("Please ensure all dependencies are installed: pip install sounddevice scipy librosa matplotlib numpy")
    sys.exit(1)

def quick_device_check():
    """Quick audio device check"""
    print("\nQuick Audio Device Check")
    print("=" * 40)

    try:
        validator = AudioValidationMain()
        devices = validator.get_audio_devices()

        print(f"Input devices: {len(devices['input_devices'])}")
        for device in devices['input_devices'][:3]:  # Show first 3
            print(f"   - {device['name']} ({device['channels']} channels)")

        print(f"Output devices: {len(devices['output_devices'])}")
        for device in devices['output_devices'][:3]:  # Show first 3
            print(f"   - {device['name']} ({device['channels']} channels)")

        print(f"Default input: {devices['default_input']}")
        print(f"Default output: {devices['default_output']}")

        return True
    except Exception as e:
        print(f"[FAIL] Device check failed: {e}")
        return False

def quick_recording_test():
    """Quick recording test"""
    print("\nQuick Recording Test (3 seconds)")
    print("=" * 40)

    try:
        import sounddevice as sd
        import numpy as np

        # Record 3 seconds of audio
        print("Starting recording...")
        duration = 3.0
        sample_rate = 44100
        recording = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1)
        sd.wait()

        # Analyze recording
        max_amplitude = np.max(np.abs(recording))
        rms_level = np.sqrt(np.mean(recording**2))

        print(f"[OK] Recording complete")
        print(f"Max amplitude: {max_amplitude:.4f}")
        print(f"RMS level: {rms_level:.4f}")

        if max_amplitude > 0.001:
            print("Recording test passed - audio signal detected")
            return True
        else:
            print("[WARN] Recording test warning - weak signal")
            return False

    except Exception as e:
        print(f"[FAIL] Recording test failed: {e}")
        return False

def quick_playback_test():
    """Quick playback test"""
    print("\nQuick Playback Test (1kHz tone, 2 seconds)")
    print("=" * 40)

    try:
        import sounddevice as sd
        import numpy as np

        # Generate 1kHz sine wave
        duration = 2.0
        sample_rate = 44100
        frequency = 1000

        t = np.linspace(0, duration, int(duration * sample_rate), False)
        tone = 0.3 * np.sin(2 * np.pi * frequency * t)

        print(f"Playing {frequency}Hz tone...")
        sd.play(tone, sample_rate)
        sd.wait()

        print("[OK] Playback test complete")
        return True

    except Exception as e:
        print(f"[FAIL] Playback test failed: {e}")
        return False

def main():
    """Main function"""
    print("Unitree Go2 Audio I/O System Quick Test")
    print("=" * 50)
    print("Time:", time.strftime("%Y-%m-%d %H:%M:%S"))
    print("")

    # Test counter
    passed_tests = 0
    total_tests = 3

    # 1. Device check
    if quick_device_check():
        passed_tests += 1

    # 2. Recording test
    if quick_recording_test():
        passed_tests += 1

    # 3. Playback test
    if quick_playback_test():
        passed_tests += 1

    # Summary
    print("\nTest Summary")
    print("=" * 40)
    success_rate = passed_tests / total_tests
    print(f"Tests passed: {passed_tests}/{total_tests} ({success_rate:.1%})")

    if success_rate >= 0.67:
        print("Audio system is basically functional!")
        print("Recommended: run full validation:")
        print("   ./run_audio_validation.sh")
    else:
        print("[WARN] Audio system has issues")
        print("Suggestions:")
        print("   - Check audio device connections")
        print("   - Check system permission settings")
        print("   - Check driver installation")

    print("\nMore options:")
    print("   - Full validation: ./run_audio_validation.sh")
    print("   - Install dependencies: ./run_audio_validation.sh --install")
    print("   - Show help: ./run_audio_validation.sh --help")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[WARN] Test interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n[FAIL] An error occurred during testing: {e}")
        sys.exit(1)
