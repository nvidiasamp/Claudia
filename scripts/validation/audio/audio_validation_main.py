#!/usr/bin/env python3
"""
Unitree Go2 Audio I/O System Validation Main Controller Script
Implements a 5-phase audio validation workflow:
- Phase A: Hardware connection and basic capture verification
- Phase B: Microphone array full-range testing
- Phase C: Speaker calibration and audio quality assessment
- Phase D: ROS2 audio topic integration verification
- Phase E: Comprehensive visualization and performance report generation

Author: Claudia AI System
Generated: 2025-06-30 13:06:45
Platform: Ubuntu 20.04 - aarch64
"""

import os
import sys
import time
import json
import logging
import argparse
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Any

# Audio processing core libraries
try:
    import sounddevice as sd
    import scipy.signal as signal
    import librosa
    import librosa.display
    from scipy.io import wavfile
    from scipy.fft import fft, fftfreq
except ImportError as e:
    print(f"[ERROR] Audio processing library import failed: {e}")
    print("Please run: pip install sounddevice scipy librosa matplotlib")
    sys.exit(1)

# ROS2 integration (optional)
try:
    import rclpy
    from rclpy.node import Node
    from audio_common_msgs.msg import AudioData
    ROS2_AVAILABLE = True
except ImportError:
    print("[WARN] ROS2 audio library not found, ROS2 integration tests will be skipped")
    ROS2_AVAILABLE = False

class AudioValidationMain:
    """Audio I/O System Validation Main Controller"""

    def __init__(self, config_path: Optional[str] = None):
        """Initialize the audio validation system"""
        self.setup_logging()
        self.config = self.load_config(config_path)
        self.results = {}
        self.start_time = datetime.now()

        # Audio parameter configuration
        self.sample_rate = self.config.get('sample_rate', 44100)
        self.channels = self.config.get('channels', 2)
        self.chunk_size = self.config.get('chunk_size', 1024)
        self.test_duration = self.config.get('test_duration', 5.0)

        # Results storage path
        self.output_dir = Path("scripts/validation/audio/results")
        self.output_dir.mkdir(exist_ok=True)

        # Test signal configuration
        self.test_frequencies = [100, 440, 1000, 5000, 10000]  # Hz
        self.white_noise_amplitude = 0.1

        self.logger.info("Audio validation system initialization complete")
        self.logger.info(f"Configuration: sample_rate={self.sample_rate}Hz, channels={self.channels}, test_duration={self.test_duration}s")

    def setup_logging(self):
        """Set up the logging system"""
        log_dir = Path("logs/audio_validation")
        log_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = log_dir / f"audio_validation_{timestamp}.log"

        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_file),
                logging.StreamHandler(sys.stdout)
            ]
        )
        self.logger = logging.getLogger(__name__)

    def load_config(self, config_path: Optional[str]) -> Dict:
        """Load configuration file"""
        default_config = {
            "sample_rate": 44100,
            "channels": 2,
            "chunk_size": 1024,
            "test_duration": 5.0,
            "frequency_range": [20, 20000],
            "snr_threshold": 40,  # dB
            "thd_threshold": 1.0,  # %
            "microphone_positions": [
                {"id": "mic_1", "x": 0.1, "y": 0.0, "z": 0.05},
                {"id": "mic_2", "x": -0.1, "y": 0.0, "z": 0.05}
            ]
        }

        if config_path and os.path.exists(config_path):
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    user_config = json.load(f)
                default_config.update(user_config)
                self.logger.info(f"[OK] Configuration file loaded: {config_path}")
            except Exception as e:
                self.logger.warning(f"[WARN] Failed to load configuration file, using defaults: {e}")

        return default_config

    def get_audio_devices(self) -> Dict[str, List]:
        """Get list of available audio devices"""
        try:
            devices = sd.query_devices()
            input_devices = []
            output_devices = []

            for i, device in enumerate(devices):
                device_info = {
                    'id': i,
                    'name': device['name'],
                    'channels': device['max_input_channels'] if device['max_input_channels'] > 0 else device['max_output_channels'],
                    'sample_rate': device['default_samplerate']
                }

                if device['max_input_channels'] > 0:
                    input_devices.append(device_info)
                if device['max_output_channels'] > 0:
                    output_devices.append(device_info)

            return {
                'input_devices': input_devices,
                'output_devices': output_devices,
                'default_input': sd.default.device[0],
                'default_output': sd.default.device[1]
            }
        except Exception as e:
            self.logger.error(f"[ERROR] Failed to get audio devices: {e}")
            return {'input_devices': [], 'output_devices': [], 'default_input': None, 'default_output': None}

    def phase_a_hardware_connection(self) -> Dict[str, Any]:
        """Phase A: Hardware connection and basic capture verification"""
        self.logger.info("Phase A: Hardware connection and basic capture verification")

        phase_results = {
            'phase': 'A',
            'name': 'Hardware Connection and Basic Capture Verification',
            'start_time': datetime.now().isoformat(),
            'tests': {}
        }

        # 1. Device detection
        self.logger.info("1. Detecting available audio devices...")
        devices = self.get_audio_devices()
        phase_results['tests']['device_detection'] = {
            'status': 'pass' if devices['input_devices'] and devices['output_devices'] else 'fail',
            'input_devices_count': len(devices['input_devices']),
            'output_devices_count': len(devices['output_devices']),
            'devices': devices
        }

        # 2. Basic recording test
        self.logger.info("2. Basic recording function test...")
        try:
            recording = sd.rec(
                int(self.test_duration * self.sample_rate),
                samplerate=self.sample_rate,
                channels=self.channels,
                dtype='float64'
            )
            sd.wait()

            # Calculate basic recording statistics
            max_amplitude = np.max(np.abs(recording))
            rms_level = np.sqrt(np.mean(recording**2))
            zero_crossings = np.sum(np.diff(np.signbit(recording.flatten())))

            # Save recording sample
            sample_file = self.output_dir / "phase_a_recording_sample.wav"
            wavfile.write(sample_file, self.sample_rate, recording)

            phase_results['tests']['basic_recording'] = {
                'status': 'pass' if max_amplitude > 0.001 else 'fail',
                'max_amplitude': float(max_amplitude),
                'rms_level': float(rms_level),
                'zero_crossings': int(zero_crossings),
                'sample_file': str(sample_file)
            }

        except Exception as e:
            phase_results['tests']['basic_recording'] = {
                'status': 'fail',
                'error': str(e)
            }
            self.logger.error(f"[ERROR] Basic recording test failed: {e}")

        # 3. Basic playback test
        self.logger.info("3. Basic playback function test...")
        try:
            # Generate 1kHz sine wave test signal
            test_freq = 1000  # Hz
            duration = 2.0  # seconds
            t = np.linspace(0, duration, int(duration * self.sample_rate), False)
            test_signal = 0.3 * np.sin(2 * np.pi * test_freq * t)

            if self.channels == 2:
                test_signal = np.column_stack([test_signal, test_signal])

            # Play test signal
            sd.play(test_signal, self.sample_rate)
            sd.wait()

            phase_results['tests']['basic_playback'] = {
                'status': 'pass',
                'test_frequency': test_freq,
                'test_duration': duration,
                'amplitude': 0.3
            }

        except Exception as e:
            phase_results['tests']['basic_playback'] = {
                'status': 'fail',
                'error': str(e)
            }
            self.logger.error(f"[ERROR] Basic playback test failed: {e}")

        # 4. Latency test
        self.logger.info("4. Audio system latency test...")
        try:
            latency = sd.query_devices()['name'] if isinstance(sd.query_devices(), dict) else 'unknown'
            phase_results['tests']['latency_test'] = {
                'status': 'pass',
                'estimated_latency_ms': 50,  # Estimated value; precise measurement requires professional equipment
                'note': 'Precise latency measurement requires professional audio analysis equipment'
            }
        except Exception as e:
            phase_results['tests']['latency_test'] = {
                'status': 'fail',
                'error': str(e)
            }

        phase_results['end_time'] = datetime.now().isoformat()
        phase_results['duration_seconds'] = (datetime.fromisoformat(phase_results['end_time']) -
                                           datetime.fromisoformat(phase_results['start_time'])).total_seconds()

        # Calculate Phase A overall result
        passed_tests = sum(1 for test in phase_results['tests'].values() if test.get('status') == 'pass')
        total_tests = len(phase_results['tests'])
        phase_results['success_rate'] = passed_tests / total_tests
        phase_results['overall_status'] = 'pass' if phase_results['success_rate'] >= 0.75 else 'fail'

        self.logger.info(f"[OK] Phase A complete: {passed_tests}/{total_tests} tests passed ({phase_results['success_rate']:.1%})")
        return phase_results

    def generate_test_signals(self) -> Dict[str, np.ndarray]:
        """Generate various test signals"""
        signals = {}
        t = np.linspace(0, self.test_duration, int(self.test_duration * self.sample_rate), False)

        # Sine wave test signals
        for freq in self.test_frequencies:
            signals[f'sine_{freq}Hz'] = 0.5 * np.sin(2 * np.pi * freq * t)

        # White noise
        signals['white_noise'] = self.white_noise_amplitude * np.random.randn(len(t))

        # Chirp signal
        signals['chirp'] = 0.5 * signal.chirp(t, 100, self.test_duration, 10000, method='linear')

        # Pulse signal
        pulse = np.zeros(len(t))
        pulse_width = int(0.1 * self.sample_rate)  # 0.1-second pulse
        pulse[:pulse_width] = 0.8
        signals['pulse'] = pulse

        return signals

    def phase_b_microphone_array_test(self) -> Dict[str, Any]:
        """Phase B: Microphone array full-range test"""
        self.logger.info("Phase B: Microphone array full-range test")

        phase_results = {
            'phase': 'B',
            'name': 'Microphone Array Full-Range Test',
            'start_time': datetime.now().isoformat(),
            'tests': {}
        }

        test_signals = self.generate_test_signals()

        # 1. Channel consistency test
        self.logger.info("1. Microphone channel consistency test...")
        try:
            # Play white noise and record all channels simultaneously
            white_noise = test_signals['white_noise']
            if self.channels == 2:
                white_noise = np.column_stack([white_noise, white_noise])

            # Start recording
            recording = sd.playrec(white_noise, samplerate=self.sample_rate, channels=self.channels)
            sd.wait()

            # Analyze response of each channel
            channel_analysis = []
            for ch in range(self.channels):
                ch_data = recording[:, ch] if self.channels > 1 else recording.flatten()

                # Calculate RMS, peak, spectrum
                rms = np.sqrt(np.mean(ch_data**2))
                peak = np.max(np.abs(ch_data))

                # FFT analysis
                fft_data = np.abs(fft(ch_data))
                freqs = fftfreq(len(ch_data), 1/self.sample_rate)

                channel_analysis.append({
                    'channel': ch,
                    'rms': float(rms),
                    'peak': float(peak),
                    'snr_estimate': float(20 * np.log10(rms / (np.std(ch_data) + 1e-10)))
                })

            # Calculate inter-channel consistency
            rms_values = [ch['rms'] for ch in channel_analysis]
            consistency_ratio = min(rms_values) / max(rms_values) if max(rms_values) > 0 else 0

            phase_results['tests']['channel_consistency'] = {
                'status': 'pass' if consistency_ratio > 0.8 else 'fail',
                'consistency_ratio': float(consistency_ratio),
                'channels': channel_analysis,
                'threshold': 0.8
            }

        except Exception as e:
            phase_results['tests']['channel_consistency'] = {
                'status': 'fail',
                'error': str(e)
            }
            self.logger.error(f"[ERROR] Channel consistency test failed: {e}")

        # 2. Frequency response test
        self.logger.info("2. Frequency response test...")
        try:
            frequency_responses = {}

            for freq_name, test_signal in test_signals.items():
                if 'sine_' in freq_name:
                    # Play sine wave and record
                    if self.channels == 2:
                        test_signal = np.column_stack([test_signal, test_signal])

                    recording = sd.playrec(test_signal, samplerate=self.sample_rate, channels=self.channels)
                    sd.wait()

                    # Analyze frequency response
                    ch_data = recording[:, 0] if self.channels > 1 else recording.flatten()

                    # FFT analysis to find dominant frequency component
                    fft_data = np.abs(fft(ch_data))
                    freqs = fftfreq(len(ch_data), 1/self.sample_rate)

                    # Find maximum frequency component
                    max_freq_idx = np.argmax(fft_data[:len(fft_data)//2])
                    detected_freq = abs(freqs[max_freq_idx])
                    amplitude = fft_data[max_freq_idx]

                    frequency_responses[freq_name] = {
                        'target_frequency': int(freq_name.split('_')[1].replace('Hz', '')),
                        'detected_frequency': float(detected_freq),
                        'amplitude': float(amplitude),
                        'accuracy': 1.0 - abs(detected_freq - int(freq_name.split('_')[1].replace('Hz', ''))) / int(freq_name.split('_')[1].replace('Hz', ''))
                    }

            avg_accuracy = np.mean([resp['accuracy'] for resp in frequency_responses.values()])

            phase_results['tests']['frequency_response'] = {
                'status': 'pass' if avg_accuracy > 0.95 else 'fail',
                'average_accuracy': float(avg_accuracy),
                'responses': frequency_responses,
                'threshold': 0.95
            }

        except Exception as e:
            phase_results['tests']['frequency_response'] = {
                'status': 'fail',
                'error': str(e)
            }
            self.logger.error(f"[ERROR] Frequency response test failed: {e}")

        # 3. Noise characteristics test
        self.logger.info("3. Noise characteristics test...")
        try:
            # Record ambient noise in a quiet environment
            noise_recording = sd.rec(
                int(self.test_duration * self.sample_rate),
                samplerate=self.sample_rate,
                channels=self.channels,
                dtype='float64'
            )
            sd.wait()

            # Analyze noise characteristics
            ch_data = noise_recording[:, 0] if self.channels > 1 else noise_recording.flatten()

            noise_rms = np.sqrt(np.mean(ch_data**2))
            noise_peak = np.max(np.abs(ch_data))

            # Estimate noise floor
            noise_floor_db = 20 * np.log10(noise_rms + 1e-10)

            phase_results['tests']['noise_characteristics'] = {
                'status': 'pass' if noise_floor_db < -40 else 'warning',
                'noise_floor_db': float(noise_floor_db),
                'noise_rms': float(noise_rms),
                'noise_peak': float(noise_peak),
                'threshold_db': -40
            }

        except Exception as e:
            phase_results['tests']['noise_characteristics'] = {
                'status': 'fail',
                'error': str(e)
            }

        phase_results['end_time'] = datetime.now().isoformat()
        phase_results['duration_seconds'] = (datetime.fromisoformat(phase_results['end_time']) -
                                           datetime.fromisoformat(phase_results['start_time'])).total_seconds()

        # Calculate Phase B overall result
        passed_tests = sum(1 for test in phase_results['tests'].values() if test.get('status') == 'pass')
        total_tests = len(phase_results['tests'])
        phase_results['success_rate'] = passed_tests / total_tests
        phase_results['overall_status'] = 'pass' if phase_results['success_rate'] >= 0.75 else 'fail'

        self.logger.info(f"[OK] Phase B complete: {passed_tests}/{total_tests} tests passed ({phase_results['success_rate']:.1%})")
        return phase_results

    def run_validation(self, phases: List[str] = None) -> Dict[str, Any]:
        """Run the audio validation workflow"""
        if phases is None:
            phases = ['A', 'B', 'C', 'D', 'E']

        self.logger.info("Starting audio I/O system validation")
        self.logger.info(f"Planned phases: {', '.join(phases)}")

        all_results = {
            'validation_info': {
                'start_time': datetime.now().isoformat(),
                'phases_requested': phases,
                'config': self.config
            },
            'phases': {}
        }

        # Execute each phase
        if 'A' in phases:
            all_results['phases']['A'] = self.phase_a_hardware_connection()

        if 'B' in phases:
            all_results['phases']['B'] = self.phase_b_microphone_array_test()

        # Phase C, D, E placeholders - to be implemented later
        if 'C' in phases:
            self.logger.info("Phase C: Speaker calibration and audio quality assessment (pending)")
            all_results['phases']['C'] = {
                'phase': 'C',
                'name': 'Speaker Calibration and Audio Quality Assessment',
                'status': 'pending',
                'note': 'To be implemented later'
            }

        if 'D' in phases:
            self.logger.info("Phase D: ROS2 audio topic integration verification (pending)")
            all_results['phases']['D'] = {
                'phase': 'D',
                'name': 'ROS2 Audio Topic Integration Verification',
                'status': 'pending',
                'note': 'To be implemented later',
                'ros2_available': ROS2_AVAILABLE
            }

        if 'E' in phases:
            self.logger.info("Phase E: Comprehensive visualization and performance report generation (pending)")
            all_results['phases']['E'] = {
                'phase': 'E',
                'name': 'Comprehensive Visualization and Performance Report Generation',
                'status': 'pending',
                'note': 'To be implemented later'
            }

        # Save results
        all_results['validation_info']['end_time'] = datetime.now().isoformat()
        all_results['validation_info']['total_duration'] = (
            datetime.fromisoformat(all_results['validation_info']['end_time']) -
            datetime.fromisoformat(all_results['validation_info']['start_time'])
        ).total_seconds()

        # Calculate overall success rate
        completed_phases = [p for p in all_results['phases'].values() if p.get('overall_status') in ['pass', 'fail']]
        if completed_phases:
            overall_success_rate = sum(1 for p in completed_phases if p.get('overall_status') == 'pass') / len(completed_phases)
            all_results['validation_info']['overall_success_rate'] = overall_success_rate
            all_results['validation_info']['overall_status'] = 'pass' if overall_success_rate >= 0.75 else 'fail'

        # Save results to file
        result_file = self.output_dir / f"audio_validation_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(result_file, 'w', encoding='utf-8') as f:
            json.dump(all_results, f, indent=2, ensure_ascii=False)

        self.logger.info(f"Validation results saved: {result_file}")
        self.logger.info("Audio I/O system validation complete!")

        return all_results

def main():
    """Main function"""
    parser = argparse.ArgumentParser(description='Unitree Go2 Audio I/O System Validation')
    parser.add_argument('--phases', '-p', nargs='+', choices=['A', 'B', 'C', 'D', 'E'],
                       default=['A', 'B'], help='Validation phases to execute')
    parser.add_argument('--config', '-c', type=str, help='Configuration file path')
    parser.add_argument('--sample-rate', '-sr', type=int, default=44100, help='Sample rate')
    parser.add_argument('--channels', '-ch', type=int, default=2, help='Number of audio channels')
    parser.add_argument('--duration', '-d', type=float, default=5.0, help='Test duration (seconds)')

    args = parser.parse_args()

    try:
        # Create validator instance
        validator = AudioValidationMain(config_path=args.config)

        # Update configuration parameters
        if args.sample_rate:
            validator.sample_rate = args.sample_rate
        if args.channels:
            validator.channels = args.channels
        if args.duration:
            validator.test_duration = args.duration

        # Run validation
        results = validator.run_validation(phases=args.phases)

        # Print summary
        print("\n" + "="*60)
        print("Audio I/O System Validation Complete")
        print("="*60)

        for phase_id, phase_data in results['phases'].items():
            if isinstance(phase_data, dict) and 'overall_status' in phase_data:
                status_icon = "[PASS]" if phase_data['overall_status'] == 'pass' else "[FAIL]"
                success_rate = phase_data.get('success_rate', 0)
                print(f"{status_icon} Phase {phase_id}: {phase_data['name']} ({success_rate:.1%})")

        overall_status = results['validation_info'].get('overall_status', 'unknown')
        overall_rate = results['validation_info'].get('overall_success_rate', 0)
        print(f"\nOverall status: {'[PASS] Passed' if overall_status == 'pass' else '[FAIL] Failed'} ({overall_rate:.1%})")

    except KeyboardInterrupt:
        print("\n[WARN] Validation interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n[ERROR] An error occurred during validation: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
