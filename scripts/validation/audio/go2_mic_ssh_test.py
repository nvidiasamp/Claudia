#!/usr/bin/env python3
"""
Go2 Built-in Microphone SSH Recording Test (Phase 0)
Explores mic devices via arecord over SSH to Go2, records audio, and analyzes quality

Usage:
  python3 scripts/validation/audio/go2_mic_ssh_test.py
  python3 scripts/validation/audio/go2_mic_ssh_test.py --continuous

Author: Claudia AI System
Generated: 2026-02-16
Platform: Ubuntu 20.04 - aarch64 (Python 3.8, no numpy)
"""

import argparse
import array
import math
import os
import re
import struct
import subprocess
import sys
import time
import wave
from datetime import datetime
from pathlib import Path

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
GO2_HOST = "unitree@192.168.123.161"
SSH_OPTS = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=5"]
SAMPLE_RATE = 16000
CHANNELS = 1
SAMPLE_WIDTH = 2  # 16-bit = 2 bytes
RECORD_SECONDS = 5
CONTINUOUS_SECONDS = 30
AMPLITUDE_THRESHOLD = 0.001  # pass/fail threshold (normalized 0-1)

OUTPUT_DIR = Path(__file__).parent / "output"


# ---------------------------------------------------------------------------
# SSH helpers
# ---------------------------------------------------------------------------

def run_ssh_command(command, timeout=15):
    """Execute a command via SSH and return stdout/stderr"""
    cmd = ["ssh"] + SSH_OPTS + [GO2_HOST, command]
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            timeout=timeout,
        )
        return result
    except subprocess.TimeoutExpired:
        return None


def test_ssh_connection():
    """SSH connection test (BatchMode=yes, key authentication only)"""
    print("\n[1/4] SSH Connection Test")
    print("-" * 50)

    result = run_ssh_command("echo OK", timeout=10)

    if result is None:
        print("[FAIL] SSH connection timed out")
        _print_ssh_fix()
        return False

    if result.returncode != 0:
        stderr = result.stderr.decode("utf-8", errors="replace").strip()
        print("[FAIL] SSH connection failed: %s" % stderr)
        _print_ssh_fix()
        return False

    stdout = result.stdout.decode("utf-8", errors="replace").strip()
    if stdout == "OK":
        print("[OK] SSH connection successful (%s)" % GO2_HOST)
        return True

    print("[FAIL] SSH response abnormal: %s" % stdout)
    _print_ssh_fix()
    return False


def _print_ssh_fix():
    """Display SSH fix commands"""
    print("")
    print("SSH key authentication is not configured. Please run:")
    print("  ssh-copy-id %s" % GO2_HOST)
    print("")
    print("Verify connection:")
    print("  ssh %s 'echo OK'" % GO2_HOST)


# ---------------------------------------------------------------------------
# Device detection
# ---------------------------------------------------------------------------

def detect_recording_devices():
    """Parse arecord -l output and return a list of recording devices

    Uses regex matching on 'card N:' and 'device N:' patterns
    to avoid locale dependency.
    """
    print("\n[2/4] Recording Device Discovery")
    print("-" * 50)

    result = run_ssh_command("arecord -l", timeout=10)
    if result is None:
        print("[FAIL] arecord -l timed out")
        return []

    if result.returncode != 0:
        stderr = result.stderr.decode("utf-8", errors="replace").strip()
        print("[FAIL] arecord -l failed: %s" % stderr)
        return []

    output = result.stdout.decode("utf-8", errors="replace")
    print("arecord -l output:")
    for line in output.strip().split("\n"):
        print("  %s" % line)

    # Parse devices: match "card N:" and "device N:" patterns (locale tolerant)
    # Typical line: "card 0: xxx [yyy], device 0: zzz [www]"
    devices = []
    card_pattern = re.compile(r"card\s+(\d+):")
    device_pattern = re.compile(r"device\s+(\d+):")

    for line in output.split("\n"):
        card_match = card_pattern.search(line)
        device_match = device_pattern.search(line)
        if card_match and device_match:
            card_num = int(card_match.group(1))
            dev_num = int(device_match.group(1))
            hw_id = "hw:%d,%d" % (card_num, dev_num)
            desc = line.strip()
            devices.append({
                "card": card_num,
                "device": dev_num,
                "hw_id": hw_id,
                "description": desc,
            })

    if devices:
        print("\n[OK] Detected devices: %d" % len(devices))
        for dev in devices:
            print("  %s - %s" % (dev["hw_id"], dev["description"]))
    else:
        print("[FAIL] No recording devices found")

    return devices


# ---------------------------------------------------------------------------
# Audio recording via SSH
# ---------------------------------------------------------------------------

def record_from_device(hw_id, duration, label=""):
    """Record PCM audio from a specified device via SSH and save as WAV

    Returns:
        tuple: (wav_path, pcm_data_bytes) or (None, None) on failure
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    safe_id = hw_id.replace(":", "_").replace(",", "_")
    if label:
        filename = "%s_%s_%s.wav" % (label, safe_id, timestamp)
    else:
        filename = "go2_mic_%s_%s.wav" % (safe_id, timestamp)

    wav_path = OUTPUT_DIR / filename

    arecord_cmd = (
        "arecord -D %s -f S16_LE -r %d -c %d -d %d -t raw"
        % (hw_id, SAMPLE_RATE, CHANNELS, duration)
    )
    cmd = ["ssh"] + SSH_OPTS + [GO2_HOST, arecord_cmd]

    print("  Recording %ds from %s ..." % (duration, hw_id))
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            timeout=duration + 15,  # extra slack for SSH overhead
        )
    except subprocess.TimeoutExpired:
        print("  [FAIL] Recording timed out (%s)" % hw_id)
        return None, None

    if result.returncode != 0:
        stderr = result.stderr.decode("utf-8", errors="replace").strip()
        # arecord writes progress to stderr, only fail on empty stdout
        if not result.stdout:
            print("  [FAIL] arecord failed (%s): %s" % (hw_id, stderr))
            return None, None

    pcm_data = result.stdout
    if len(pcm_data) == 0:
        print("  [FAIL] PCM data is empty (%s)" % hw_id)
        return None, None

    expected_bytes = SAMPLE_RATE * CHANNELS * SAMPLE_WIDTH * duration
    actual_bytes = len(pcm_data)
    print("  Received %d bytes (expected ~%d)" % (actual_bytes, expected_bytes))

    # Write WAV
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    with wave.open(str(wav_path), "wb") as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(SAMPLE_WIDTH)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(pcm_data)

    print("  Saved: %s" % wav_path)
    return wav_path, pcm_data


# ---------------------------------------------------------------------------
# Audio analysis (no numpy - uses struct/array)
# ---------------------------------------------------------------------------

def analyze_pcm(pcm_data):
    """Calculate amplitude, RMS, and SNR of PCM S16_LE data (without numpy)

    Returns:
        dict with peak_amplitude, rms_amplitude, snr_estimate_db, num_samples
    """
    num_samples = len(pcm_data) // SAMPLE_WIDTH
    if num_samples == 0:
        return {
            "peak_amplitude": 0.0,
            "rms_amplitude": 0.0,
            "snr_estimate_db": 0.0,
            "num_samples": 0,
        }

    # Unpack 16-bit signed samples
    fmt = "<%dh" % num_samples
    samples = struct.unpack(fmt, pcm_data[:num_samples * SAMPLE_WIDTH])

    max_val = 32768.0  # 16-bit signed range

    # Peak amplitude (normalized 0-1)
    peak_raw = max(abs(s) for s in samples)
    peak_amplitude = peak_raw / max_val

    # RMS amplitude (normalized 0-1)
    sum_sq = sum(s * s for s in samples)
    rms_raw = math.sqrt(sum_sq / num_samples)
    rms_amplitude = rms_raw / max_val

    # SNR estimate: compare top 10% RMS (signal) vs bottom 10% RMS (noise)
    sorted_abs = sorted(abs(s) for s in samples)
    n10 = max(1, num_samples // 10)

    noise_samples = sorted_abs[:n10]
    signal_samples = sorted_abs[-n10:]

    noise_rms = math.sqrt(sum(s * s for s in noise_samples) / len(noise_samples))
    signal_rms = math.sqrt(sum(s * s for s in signal_samples) / len(signal_samples))

    if noise_rms > 0:
        snr_db = 20.0 * math.log10(signal_rms / noise_rms)
    else:
        snr_db = float("inf")

    return {
        "peak_amplitude": peak_amplitude,
        "rms_amplitude": rms_amplitude,
        "snr_estimate_db": snr_db,
        "num_samples": num_samples,
    }


# ---------------------------------------------------------------------------
# Device test
# ---------------------------------------------------------------------------

def test_devices(devices, duration=RECORD_SECONDS):
    """Record and analyze from each device, return results list"""
    print("\n[3/4] Device Recording Test (%ds)" % duration)
    print("-" * 50)

    results = []
    for dev in devices:
        hw_id = dev["hw_id"]
        print("\nTesting %s ..." % hw_id)

        wav_path, pcm_data = record_from_device(hw_id, duration)
        if pcm_data is None:
            results.append({
                "hw_id": hw_id,
                "description": dev["description"],
                "status": "FAIL",
                "reason": "recording failed",
            })
            continue

        analysis = analyze_pcm(pcm_data)
        passed = analysis["peak_amplitude"] > AMPLITUDE_THRESHOLD

        print("  Peak amplitude:  %.6f" % analysis["peak_amplitude"])
        print("  RMS amplitude:   %.6f" % analysis["rms_amplitude"])
        print("  SNR estimate:    %.1f dB" % analysis["snr_estimate_db"])
        print("  Samples:         %d" % analysis["num_samples"])
        print("  Status:          %s" % ("PASS" if passed else "FAIL"))

        results.append({
            "hw_id": hw_id,
            "description": dev["description"],
            "wav_path": str(wav_path) if wav_path else None,
            "status": "PASS" if passed else "FAIL",
            "peak_amplitude": analysis["peak_amplitude"],
            "rms_amplitude": analysis["rms_amplitude"],
            "snr_estimate_db": analysis["snr_estimate_db"],
            "num_samples": analysis["num_samples"],
        })

    return results


# ---------------------------------------------------------------------------
# Continuous stream test
# ---------------------------------------------------------------------------

def test_continuous_stream(hw_id, duration=CONTINUOUS_SECONDS):
    """Long-duration continuous streaming test"""
    print("\n[Continuous] %ds continuous stream test (%s)" % (duration, hw_id))
    print("-" * 50)

    wav_path, pcm_data = record_from_device(
        hw_id, duration, label="continuous"
    )
    if pcm_data is None:
        print("[FAIL] Continuous stream failed")
        return False

    analysis = analyze_pcm(pcm_data)
    passed = analysis["peak_amplitude"] > AMPLITUDE_THRESHOLD

    # Chunk analysis: split into 5s segments to check consistency
    chunk_size = SAMPLE_RATE * CHANNELS * SAMPLE_WIDTH * 5
    num_chunks = max(1, len(pcm_data) // chunk_size)
    print("\nChunk analysis (%d x 5s segments):" % num_chunks)

    for i in range(num_chunks):
        start = i * chunk_size
        end = min(start + chunk_size, len(pcm_data))
        chunk_analysis = analyze_pcm(pcm_data[start:end])
        chunk_ok = chunk_analysis["peak_amplitude"] > AMPLITUDE_THRESHOLD
        print(
            "  Chunk %d: peak=%.6f rms=%.6f %s"
            % (
                i + 1,
                chunk_analysis["peak_amplitude"],
                chunk_analysis["rms_amplitude"],
                "OK" if chunk_ok else "WEAK",
            )
        )

    print("\nOverall:")
    print("  Peak amplitude:  %.6f" % analysis["peak_amplitude"])
    print("  RMS amplitude:   %.6f" % analysis["rms_amplitude"])
    print("  SNR estimate:    %.1f dB" % analysis["snr_estimate_db"])
    print("  Duration:        %.1fs" % (analysis["num_samples"] / SAMPLE_RATE))
    print("  Status:          %s" % ("PASS" if passed else "FAIL"))

    return passed


# ---------------------------------------------------------------------------
# Latency measurement
# ---------------------------------------------------------------------------

def measure_ssh_audio_latency(hw_id, iterations=5):
    """Measure SSH recording latency (command issued -> first byte received)

    Starts SSH arecord via Popen and measures time until first PCM chunk arrives.
    Repeats for the given number of iterations and reports average/min/max.
    """
    print("\n[Latency] SSH recording latency measurement (%s, %d iterations)" % (hw_id, iterations))
    print("-" * 50)

    arecord_cmd = (
        "arecord -D %s -f S16_LE -r %d -c %d -d 2 -t raw"
        % (hw_id, SAMPLE_RATE, CHANNELS)
    )
    cmd = ["ssh"] + SSH_OPTS + [GO2_HOST, arecord_cmd]

    latencies = []
    for i in range(iterations):
        try:
            t_start = time.monotonic()
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            # Read first chunk (any bytes arriving = first audio data)
            first_chunk = proc.stdout.read(SAMPLE_WIDTH * 16)  # 16 samples
            t_first_byte = time.monotonic()

            latency_ms = (t_first_byte - t_start) * 1000.0

            # Clean up the process
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()

            if first_chunk:
                latencies.append(latency_ms)
                print("  Run %d: %.1f ms" % (i + 1, latency_ms))
            else:
                print("  Run %d: no data received" % (i + 1))

        except Exception as e:
            print("  Run %d: error - %s" % (i + 1, e))

    if not latencies:
        print("\n[FAIL] Latency measurement failed")
        return None

    avg_ms = sum(latencies) / len(latencies)
    min_ms = min(latencies)
    max_ms = max(latencies)

    print("\nLatency results:")
    print("  Average: %.1f ms" % avg_ms)
    print("  Min:     %.1f ms" % min_ms)
    print("  Max:     %.1f ms" % max_ms)
    print("  Target:  < 200 ms")
    print("  Status:  %s" % ("PASS" if avg_ms < 200 else "WARN (high latency)"))

    return {
        "avg_ms": avg_ms,
        "min_ms": min_ms,
        "max_ms": max_ms,
        "iterations": len(latencies),
    }


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def print_report(results, continuous_result=None):
    """Display test results summary"""
    print("\n[4/4] Test Results Summary")
    print("=" * 60)

    if not results:
        print("[FAIL] No test results available")
        return False

    pass_count = sum(1 for r in results if r["status"] == "PASS")
    total_count = len(results)

    for r in results:
        status_mark = "[PASS]" if r["status"] == "PASS" else "[FAIL]"
        if "peak_amplitude" in r:
            print(
                "  %s %s  peak=%.6f  rms=%.6f  snr=%.1fdB"
                % (
                    status_mark,
                    r["hw_id"],
                    r["peak_amplitude"],
                    r["rms_amplitude"],
                    r["snr_estimate_db"],
                )
            )
        else:
            print("  %s %s  %s" % (status_mark, r["hw_id"], r.get("reason", "")))

    print("")
    print("Devices tested: %d" % total_count)
    print("Devices passed: %d" % pass_count)

    # Best device
    passed_devices = [r for r in results if r["status"] == "PASS"]
    if passed_devices:
        best = max(passed_devices, key=lambda r: r.get("peak_amplitude", 0))
        print("\nBest device: %s" % best["hw_id"])
        print("  Peak amplitude: %.6f" % best["peak_amplitude"])
        print("  SNR estimate:   %.1f dB" % best["snr_estimate_db"])
        if best.get("wav_path"):
            print("  WAV file:       %s" % best["wav_path"])

    if continuous_result is not None:
        print(
            "\nContinuous stream: %s"
            % ("PASS" if continuous_result else "FAIL")
        )

    # Overall verdict
    overall = pass_count > 0
    print("")
    if overall:
        print("=" * 60)
        print("OVERALL: PASS - Go2 microphone recording via SSH is working")
        print("=" * 60)
    else:
        print("=" * 60)
        print("OVERALL: FAIL - No devices capable of recording found")
        print("=" * 60)
        print("")
        print("Troubleshooting:")
        print("  1. Verify Go2 is powered on")
        print("  2. SSH connection: ssh %s" % GO2_HOST)
        print("  3. Run arecord -l manually on Go2")
        print("  4. Verify microphone is physically enabled")

    return overall


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    """Main execution"""
    parser = argparse.ArgumentParser(
        description="Go2 Built-in Microphone SSH Recording Test (Phase 0)"
    )
    parser.add_argument(
        "--continuous",
        action="store_true",
        help="Also run %ds continuous stream test on best device" % CONTINUOUS_SECONDS,
    )
    parser.add_argument(
        "--device",
        type=str,
        default=None,
        help="Test a specific device only (e.g., hw:0,0)",
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=RECORD_SECONDS,
        help="Recording duration in seconds (default: %d)" % RECORD_SECONDS,
    )
    parser.add_argument(
        "--latency",
        action="store_true",
        help="Measure SSH recording latency on best device (5 iterations)",
    )
    args = parser.parse_args()

    print("Go2 Built-in Microphone SSH Recording Test (Phase 0)")
    print("=" * 60)
    print("Time: %s" % datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    print("Host: %s" % GO2_HOST)
    print("Output: %s" % OUTPUT_DIR)
    print("")

    # Step 1: SSH connection
    if not test_ssh_connection():
        sys.exit(1)

    # Step 2: Device detection
    if args.device:
        # User specified a device directly
        devices = [{
            "card": -1,
            "device": -1,
            "hw_id": args.device,
            "description": "user-specified",
        }]
        print("\n[2/4] Device specified: %s" % args.device)
    else:
        devices = detect_recording_devices()
        if not devices:
            print("\n[FAIL] No devices detected")
            sys.exit(1)

    # Step 3: Record and analyze
    results = test_devices(devices, duration=args.duration)

    # Step 3.5: Continuous test (if requested)
    continuous_result = None
    if args.continuous:
        passed_devices = [r for r in results if r["status"] == "PASS"]
        if passed_devices:
            best = max(passed_devices, key=lambda r: r.get("peak_amplitude", 0))
            continuous_result = test_continuous_stream(
                best["hw_id"], duration=CONTINUOUS_SECONDS
            )
        else:
            print("\n[SKIP] Continuous test: no PASS devices")
            continuous_result = False

    # Step 3.6: Latency measurement (if requested)
    latency_result = None
    if args.latency:
        passed_devices = [r for r in results if r["status"] == "PASS"]
        if passed_devices:
            best = max(passed_devices, key=lambda r: r.get("peak_amplitude", 0))
            latency_result = measure_ssh_audio_latency(best["hw_id"])
        else:
            print("\n[SKIP] Latency measurement: no PASS devices")

    # Step 4: Report
    overall = print_report(results, continuous_result)

    if latency_result is not None:
        print("\nSSH Audio Latency: avg=%.1fms min=%.1fms max=%.1fms"
              % (latency_result["avg_ms"], latency_result["min_ms"],
                 latency_result["max_ms"]))

    sys.exit(0 if overall else 1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(1)
    except Exception as e:
        print("\n[ERROR] An error occurred during testing: %s" % e)
        import traceback
        traceback.print_exc()
        sys.exit(1)
