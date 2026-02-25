# Unitree Go2 Audio I/O System Validation Report

## Project Information

| Item | Details |
|------|---------|
| **Validation System Name** | Unitree Go2 Audio I/O System Validation |
| **Development Status** | Phase A,B Complete / Phase C,D,E In Development |
| **Generated** | 2025-06-30 13:06:45 |
| **Platform** | Ubuntu 20.04 - aarch64 |
| **Developer** | Claudia AI System |
| **Version** | v1.0 |

---

## Validation Objectives

This audio validation system aims to comprehensively test the Unitree Go2 robot's audio input/output capabilities, ensuring:

1. **Hardware connection stability** - Audio devices correctly identified and connected
2. **Microphone array performance** - Multi-channel capture, spatial positioning, noise suppression
3. **Speaker audio quality** - Frequency response, distortion control, volume consistency
4. **ROS2 integration compatibility** - Audio topic publishing, real-time data streaming
5. **Overall system performance** - End-to-end latency, stability, visualization

---

## System Architecture

### Validation Phase Design

```
Audio I/O System Validation
+-- Phase A: Hardware Connection and Basic Capture Verification [DONE]
|   +-- 1. Audio device detection and enumeration
|   +-- 2. Basic recording function verification
|   +-- 3. Basic playback function verification
|   +-- 4. Audio system latency test
+-- Phase B: Microphone Array Full-Range Test [DONE]
|   +-- 1. Channel consistency test
|   +-- 2. Frequency response analysis
|   +-- 3. Noise characteristics assessment
+-- Phase C: Speaker Calibration and Audio Quality Assessment [IN PROGRESS]
|   +-- 1. Frequency response calibration
|   +-- 2. Maximum sound pressure level test
|   +-- 3. Total harmonic distortion (THD) analysis
|   +-- 4. Volume consistency verification
+-- Phase D: ROS2 Audio Topic Integration Verification [IN PROGRESS]
|   +-- 1. audio_common_msgs integration
|   +-- 2. Multi-channel data publishing
|   +-- 3. Real-time data stream testing
|   +-- 4. Timestamp synchronization verification
+-- Phase E: Comprehensive Visualization and Performance Report Generation [IN PROGRESS]
    +-- 1. Audio waveform visualization
    +-- 2. Spectrum analysis charts
    +-- 3. Performance metric dashboard
    +-- 4. Automated test report
```

### Technology Stack

| Layer | Components | Purpose |
|-------|-----------|---------|
| **Hardware Layer** | PortAudio, ALSA, PulseAudio | Low-level audio device access |
| **Signal Processing Layer** | SciPy, NumPy, LibROSA | Audio signal analysis and processing |
| **Visualization Layer** | Matplotlib, Plotly (planned) | Data visualization and chart generation |
| **ROS2 Integration Layer** | audio_common_msgs, rclpy | Robot system integration |
| **Application Layer** | SoundDevice, JSON configuration | User interface and configuration management |

---

## Installation and Configuration

### System Requirements

- **OS**: Ubuntu 20.04 LTS (aarch64)
- **Python**: 3.8+
- **Memory**: Minimum 2GB RAM
- **Storage**: 500MB available space
- **Audio Device**: Audio device supporting ALSA/PulseAudio

### Quick Installation

```bash
# 1. Navigate to audio validation directory
cd scripts/validation/audio/

# 2. Install all dependencies (automated)
./run_audio_validation.sh --install

# 3. Verify installation
./run_audio_validation.sh --help
```

### Manual Installation (Optional)

```bash
# System dependencies
sudo apt update
sudo apt install -y portaudio19-dev libasound2-dev libsndfile1-dev libfftw3-dev python3-pip python3-dev

# Python dependencies
pip3 install --user sounddevice scipy librosa matplotlib numpy

# ROS2 dependencies (optional)
pip3 install --user audio-common-msgs
```

---

## Usage Guide

### Basic Usage

```bash
# Run default validation (Phase A + B)
./run_audio_validation.sh

# Run specific phases
./run_audio_validation.sh --phases A,B

# Custom audio parameters
./run_audio_validation.sh --sample-rate 48000 --channels 2 --duration 10.0

# Use configuration file
./run_audio_validation.sh --config my_config.json
```

### Advanced Configuration

Create a custom configuration file `audio_config.json`:

```json
{
  "sample_rate": 44100,
  "channels": 2,
  "chunk_size": 1024,
  "test_duration": 5.0,
  "frequency_range": [20, 20000],
  "snr_threshold": 40,
  "thd_threshold": 1.0,
  "microphone_positions": [
    {"id": "mic_1", "x": 0.1, "y": 0.0, "z": 0.05},
    {"id": "mic_2", "x": -0.1, "y": 0.0, "z": 0.05}
  ]
}
```

### Command Line Arguments

| Argument | Short | Default | Description |
|----------|-------|---------|-------------|
| `--phases` | `-p` | A,B | Validation phases to execute |
| `--config` | `-c` | - | Configuration file path |
| `--sample-rate` | `-sr` | 44100 | Audio sample rate (Hz) |
| `--channels` | `-ch` | 2 | Number of audio channels |
| `--duration` | `-d` | 5.0 | Test duration (seconds) |
| `--install` | `-i` | - | Install dependencies |
| `--help` | `-h` | - | Show help information |

---

## Validation Phase Details

### Phase A: Hardware Connection and Basic Capture Verification [DONE]

**Objective**: Confirm audio hardware is working properly and basic I/O functions are available

#### Test Items

1. **Device Detection** - Enumerate and verify audio input/output devices
   - Checks: Available device count, device names, supported channels
   - Pass criteria: At least 1 input device and 1 output device

2. **Basic Recording Test** - Verify audio capture functionality
   - Checks: Recording amplitude, RMS level, zero-crossing rate
   - Pass criteria: Maximum amplitude > 0.001

3. **Basic Playback Test** - Verify audio playback functionality
   - Checks: 1kHz sine wave playback success
   - Pass criteria: No abnormal errors

4. **Latency Test** - Evaluate audio system latency
   - Checks: Estimated end-to-end latency
   - Pass criteria: Latency < 100ms (estimated)

#### Output Files

- `phase_a_recording_sample.wav` - Recording test sample
- `audio_validation_results_YYYYMMDD_HHMMSS.json` - Detailed test results

### Phase B: Microphone Array Full-Range Test [DONE]

**Objective**: Comprehensively test microphone array performance, including consistency, frequency response, noise characteristics

#### Test Items

1. **Channel Consistency Test** - Verify multi-channel microphone response consistency
   - Method: Play white noise and record all channels simultaneously
   - Analysis: RMS level, peak value, SNR estimation
   - Pass criteria: Inter-channel consistency ratio > 0.8

2. **Frequency Response Test** - Test microphone frequency characteristics
   - Method: Play sine wave signals at multiple frequencies
   - Test frequencies: 100Hz, 440Hz, 1kHz, 5kHz, 10kHz
   - Pass criteria: Average frequency detection accuracy > 95%

3. **Noise Characteristics Test** - Evaluate environmental and system noise
   - Method: Record in a quiet environment, analyze noise floor
   - Pass criteria: Noise floor < -40dB

#### Performance Metrics

| Metric | Target Value | Measurement Method |
|--------|-------------|-------------------|
| Channel consistency | > 0.8 | RMS ratio |
| Frequency accuracy | > 95% | FFT analysis |
| Noise floor | < -40dB | RMS power spectrum |
| Signal-to-noise ratio | > 40dB | Signal/noise ratio |

### Phase C: Speaker Calibration and Audio Quality Assessment [IN PROGRESS]

**Planned features**:
- Frequency response calibration
- Maximum sound pressure level test
- Total harmonic distortion (THD) analysis
- Volume consistency verification

### Phase D: ROS2 Audio Topic Integration Verification [IN PROGRESS]

**Planned features**:
- audio_common_msgs integration test
- Multi-channel audio data publishing
- Real-time data stream performance test
- Timestamp synchronization verification

### Phase E: Comprehensive Visualization and Performance Report Generation [IN PROGRESS]

**Planned features**:
- Interactive audio waveform visualization
- Real-time spectrum analysis charts
- Performance metric dashboard
- Automated HTML/PDF report generation

---

## Test Result Interpretation

### Result File Structure

```json
{
  "validation_info": {
    "start_time": "2025-06-30T13:06:45",
    "end_time": "2025-06-30T13:07:15",
    "total_duration": 30.5,
    "overall_success_rate": 0.85,
    "overall_status": "pass"
  },
  "phases": {
    "A": {
      "phase": "A",
      "name": "Hardware Connection and Basic Capture Verification",
      "success_rate": 1.0,
      "overall_status": "pass",
      "tests": { /* detailed test results */ }
    }
  }
}
```

### Status Descriptions

| Status | Meaning | Suggested Action |
|--------|---------|-----------------|
| `pass` | Test passed | Continue to next phase |
| `fail` | Test failed | Check error message, fix the issue |
| `warning` | Warning state | Can continue, but optimization recommended |
| `pending` | Not yet implemented | Wait for feature development |

### Common Troubleshooting

#### 1. Audio Devices Not Found

```bash
# Check audio devices
aplay -l    # Playback devices
arecord -l  # Recording devices

# Check user permissions
groups | grep audio

# Add user to audio group
sudo usermod -a -G audio $USER
```

#### 2. Dependency Installation Failure

```bash
# Update package manager
sudo apt update

# Install build tools
sudo apt install build-essential

# Reinstall dependencies
./run_audio_validation.sh --install
```

#### 3. Low Recording Amplitude

- Check if microphone is properly connected
- Adjust system volume settings
- Confirm microphone is not muted

---

## Development and Extension

### Adding New Test Items

1. Add a new test method in the `AudioValidationMain` class
2. Call the new method in `run_validation()`
3. Update configuration file to support new parameters
4. Add corresponding command line arguments

### Custom Signal Processing

```python
def custom_signal_analysis(self, audio_data, sample_rate):
    """Custom signal analysis function"""
    # Implement your analysis logic
    result = {}

    # Example: calculate spectral centroid
    fft_data = np.abs(np.fft.fft(audio_data))
    freqs = np.fft.fftfreq(len(audio_data), 1/sample_rate)
    spectral_centroid = np.sum(freqs * fft_data) / np.sum(fft_data)

    result['spectral_centroid'] = spectral_centroid
    return result
```

### Integrating New Visualizations

```python
import plotly.graph_objects as go

def create_interactive_spectrum(self, frequencies, magnitudes):
    """Create interactive spectrum chart"""
    fig = go.Figure()
    fig.add_trace(go.Scatter(
        x=frequencies,
        y=magnitudes,
        mode='lines',
        name='Spectrum'
    ))

    fig.update_layout(
        title='Audio Spectrum Analysis',
        xaxis_title='Frequency (Hz)',
        yaxis_title='Amplitude (dB)'
    )

    return fig
```

---

## File Structure

```
scripts/validation/audio/
+-- audio_validation_main.py      # Main validation script
+-- run_audio_validation.sh       # Startup script
+-- AUDIO_VALIDATION_REPORT.md    # This document
+-- results/                      # Test results directory
|   +-- audio_validation_results_*.json
|   +-- phase_a_recording_sample.wav
+-- logs/                         # Log directory (auto-created)
    +-- audio_validation_*.log
```

---

## Technical Reference

### Audio Processing Theory

1. **Sampling Theorem**: Sample rate must be greater than twice the highest signal frequency
2. **Window Functions**: Mathematical functions used to reduce spectral leakage
3. **FFT Analysis**: Fast Fourier Transform for frequency domain analysis
4. **Signal-to-Noise Ratio**: SNR = 20 * log10(Signal RMS / Noise RMS)

### Unitree Go2 Audio Specifications

| Specification | Value |
|--------------|-------|
| Microphone type | Omnidirectional condenser microphone array |
| Sample rate | 16kHz/44.1kHz |
| Bit depth | 16-bit |
| Channels | 2-channel (stereo) |
| Frequency response | 20Hz - 20kHz |
| Signal-to-noise ratio | > 60dB |

### Related Standards

- **IEC 61672**: Sound level meter standard
- **AES17**: Digital audio device measurement standard
- **ITU-T P.862**: PESQ voice quality assessment standard

---

## Changelog

### v1.0 (2025-06-30)

#### Completed
- Phase A: Hardware connection and basic capture verification
- Phase B: Microphone array full-range test
- Complete startup script and dependency management
- JSON format result output
- Detailed error handling and logging

#### In Development
- Phase C: Speaker calibration and audio quality assessment
- Phase D: ROS2 audio topic integration verification
- Phase E: Comprehensive visualization and performance report generation

#### Planned Features
- Real-time audio visualization interface
- Web-based monitoring dashboard
- Automated continuous integration testing
- Machine learning audio quality assessment

---

## Contributing

Contributions and improvement suggestions are welcome!

### Submission Process

1. Fork this project
2. Create a feature branch: `git checkout -b feature/new-audio-test`
3. Commit changes: `git commit -am 'Add new audio test'`
4. Push branch: `git push origin feature/new-audio-test`
5. Create Pull Request

### Code Standards

- Follow PEP 8 Python code style
- Add detailed docstrings
- Include unit tests
- Update related documentation

---

## Support and Feedback

### Technical Support

- **Repository**: [Claudia Robot Project](https://github.com/your-repo/claudia)
- **Issue Reporting**: GitHub Issues
- **Documentation Issues**: Please comment on the relevant documentation page

### FAQ

1. **Q**: Why do some dependencies fail to install?
   **A**: Please ensure the system is updated and has the appropriate compilation tools.

2. **Q**: How to resolve audio device permission errors?
   **A**: Add the user to the audio group: `sudo usermod -a -G audio $USER`

3. **Q**: How to customize test parameters?
   **A**: Create a JSON configuration file and specify it with the `--config` argument.

---

**Copyright 2025 Claudia AI System. This document will be continuously updated to reflect the latest development progress.**
