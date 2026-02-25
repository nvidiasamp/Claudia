# Unitree Go2 Audio Troubleshooting and Resolution Guide

## Current Issue

**Symptom**: Recording test fails, maximum amplitude and RMS level are 0.0000
**Device**: Unitree Go2 (NVIDIA Jetson Orin NX - aarch64)
**System**: Ubuntu 20.04

---

## Root Cause Analysis

### What is Working
- Permission configuration is correct (user is in audio group)
- PulseAudio is running properly
- ALSA device recognition is normal (26 playback devices, 22 recording devices)
- Audio device enumeration is successful

### What is Not Working
1. **Jetson Orin NX complex audio routing**: DMIC1-DMIC4 digital microphones are not properly configured
2. **Unitree AudioClient not being used**: Go2 robot needs to access audio through SDK2's AudioClient
3. **Audio routing misconfiguration**: Most mixer controls are set to 'None' state

---

## Solutions (Ordered by Priority)

### Option 1: Use Unitree AudioClient SDK (Recommended)

According to the latest Unitree SDK2 documentation, Go2 robot should access audio functionality through the dedicated AudioClient API:

```python
# Unitree Go2 dedicated audio interface
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.audio.audio_client import AudioClient

# Initialize
ChannelFactoryInitialize(0, "")
audio_client = AudioClient()

# Audio operations
volume = audio_client.GetVolume()
audio_client.SetVolume(85)
audio_client.TtsMaker("Test speech", 0)  # TTS test
# ASR and microphone recording via dedicated interface
```

**Advantages**:
- Bypasses Jetson complex audio routing
- Direct access to Go2 hardware microphone array
- Fully compatible with the robot system

---

### Option 2: Configure Jetson Audio Routing

#### 2.1 Enable DMIC Digital Microphones
```bash
# Try enabling DMIC
sudo amixer -c 1 set 'ADMAIF1 Mux' 'DMIC1'
sudo amixer -c 1 set 'ADMAIF2 Mux' 'DMIC2'

# Set recording volume
sudo amixer -c 1 set 'DMIC1' 100%
sudo amixer -c 1 set 'DMIC2' 100%
```

#### 2.2 Configure PulseAudio
```bash
# Restart PulseAudio
pulseaudio --kill
pulseaudio --start

# List available input sources
pactl list sources short

# Set default input source
pactl set-default-source <source_name>
```

---

### Option 3: Manual Device Testing

#### 3.1 Test ALSA Devices One by One
```bash
# List recording devices
arecord -l

# Test recording with a specific device
arecord -D hw:1,0 -f S16_LE -r 44100 -c 2 -d 5 test_record.wav

# Playback test
aplay test_record.wav
```

#### 3.2 Try Different Device IDs
Try different sounddevice device IDs in Python scripts:
```python
import sounddevice as sd

# Test all input devices
devices = sd.query_devices()
for i, device in enumerate(devices):
    if device['max_input_channels'] > 0:
        try:
            # Use lower sample rate and mono
            recording = sd.rec(
                int(1 * 8000),  # 1 second, 8kHz
                samplerate=8000,
                channels=1,
                device=i
            )
            sd.wait()
            print(f"Device {i} recording successful, max amplitude: {max(abs(recording))}")
        except Exception as e:
            print(f"Device {i} failed: {e}")
```

---

## Quick Fixes to Try Immediately

### Quick Fix 1: Reset Audio System
```bash
# Full audio system reset
sudo systemctl restart alsa-utils
pulseaudio --kill
pulseaudio --start

# Re-detect audio devices
sudo alsa force-reload
```

### Quick Fix 2: Try USB Audio Device
If you have a USB microphone or audio interface, try after plugging it in:
```bash
# Check for new device
lsusb | grep -i audio
arecord -l
```

### Quick Fix 3: Verify Unitree SDK2 Installation
```bash
# Verify Unitree SDK2
python3 -c "from unitree_sdk2py.core.channel import ChannelFactoryInitialize; print('SDK2 OK')"

# If it fails, reinstall
cd ~/unitree_sdk2_python
pip3 install -e .
```

---

## Recommended Resolution Path

### Phase 1: Verify Unitree AudioClient (2 hours)
1. Confirm unitree_sdk2py is installed correctly
2. Write AudioClient test script
3. Verify TTS and volume control functions
4. Test ASR and recording functionality

### Phase 2: If AudioClient Unavailable, Configure Jetson Audio (4 hours)
1. Configure DMIC per Option 2
2. Test PulseAudio configuration
3. Verify ALSA devices one by one

### Phase 3: Alternative Hardware Solution (1 hour)
1. Use USB audio device as fallback
2. Verify external microphone works

---

## Verification Checklist

Run this check after each solution attempt:

```bash
# Checklist script
echo "=== Audio System Status Check ==="
echo "1. User groups:" $(groups | grep audio)
echo "2. PulseAudio:" $(pgrep pulseaudio > /dev/null && echo "Running" || echo "Not running")
echo "3. ALSA playback devices:" $(aplay -l | grep "card" | wc -l)
echo "4. ALSA recording devices:" $(arecord -l | grep "card" | wc -l)
echo "5. Unitree SDK2:" $(python3 -c "from unitree_sdk2py.core.channel import ChannelFactoryInitialize; print('Available')" 2>/dev/null || echo "Not available")

# Quick recording test
echo "6. Quick recording test:"
arecord -D hw:1,0 -f S16_LE -r 16000 -c 1 -d 2 quick_test.wav 2>/dev/null && echo "Recording successful" || echo "Recording failed"
```

---

## Expert Recommendation

Based on documentation and your system configuration, I **strongly recommend trying Option 1 (Unitree AudioClient) first**, because:

1. **Designed for Go2**: Avoids Jetson's complex audio routing configuration
2. **Vendor supported**: Official audio interface provided by Unitree
3. **Full functionality**: Supports TTS, ASR, volume control, recording, and all other features
4. **System integration**: Fully compatible with the Go2 robot system

**Next steps**:
1. Run the checklist to confirm current status
2. Try the Unitree AudioClient interface
3. If successful, update the audio validation system to use AudioClient
4. If unsuccessful, follow Option 2 to configure Jetson audio routing

---

## Getting Help

If none of the above solutions work, please collect the following information:
- Checklist output results
- `dmesg | grep -i audio` output
- Unitree SDK2 version information
- Go2 robot firmware version

This will help with further diagnosis of the issue.
