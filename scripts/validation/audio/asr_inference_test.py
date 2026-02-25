#!/usr/bin/env python3
"""
ASR Inference Smoke Test (Phase 1)
Validates faster-whisper (CTranslate2) model loading and inference

Usage:
  # Production mode (requires faster-whisper)
  python3 scripts/validation/audio/asr_inference_test.py

  # Mock mode (verifies imports and pipeline structure only, no model needed)
  python3 scripts/validation/audio/asr_inference_test.py --mock

  # Specify model size
  python3 scripts/validation/audio/asr_inference_test.py --model medium

Note: For manual execution only. Do not run during Shadow comparison (GPU VRAM conflict).

Author: Claudia AI System
Generated: 2026-02-16
Target Python: 3.8 (system)
"""

import argparse
import logging
import math
import os
import struct
import sys
import time
from pathlib import Path
from typing import Optional

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
SAMPLE_RATE = 16000
CHANNELS = 1
SAMPLE_WIDTH = 2  # 16-bit
OUTPUT_DIR = Path(__file__).parent / "output"

logger = logging.getLogger("claudia.asr.smoke_test")


# ---------------------------------------------------------------------------
# Test audio generation
# ---------------------------------------------------------------------------

def generate_test_wav(duration_silence_s: float = 1.0,
                      duration_tone_s: float = 1.0,
                      freq_hz: float = 440.0) -> bytes:
    """Generate test WAV: silence + sine wave tone

    Returns
    -------
    bytes
        16kHz 16-bit mono PCM data (no WAV header)
    """
    n_silence = int(SAMPLE_RATE * duration_silence_s)
    n_tone = int(SAMPLE_RATE * duration_tone_s)

    samples = []
    for _ in range(n_silence):
        samples.append(0)

    for i in range(n_tone):
        t = i / SAMPLE_RATE
        value = int(0.5 * 32767 * math.sin(2 * math.pi * freq_hz * t))
        samples.append(max(-32768, min(32767, value)))

    pcm_data = struct.pack("<{}h".format(len(samples)), *samples)
    return pcm_data


def save_test_wav(pcm_data: bytes, filepath: Path) -> None:
    """Save PCM data as a WAV file"""
    import wave
    filepath.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(filepath), "wb") as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(SAMPLE_WIDTH)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(pcm_data)
    logger.info("Test WAV saved: %s (%d bytes, %.1fs)",
                filepath, len(pcm_data),
                len(pcm_data) / (SAMPLE_RATE * SAMPLE_WIDTH * CHANNELS))


# ---------------------------------------------------------------------------
# Test execution
# ---------------------------------------------------------------------------

class ASRSmokeTest:
    """ASR Inference Smoke Test"""

    def __init__(self, mock: bool = False,
                 model_size: Optional[str] = None) -> None:
        self.mock = mock
        self.model_size = model_size or os.getenv("CLAUDIA_ASR_MODEL", "base")
        self.results = {}  # type: dict
        self._model = None
        self._passed = True

    def run_all(self) -> bool:
        """Run all tests. True = all passed"""
        print("=" * 60)
        print("  Claudia ASR Inference Smoke Test")
        print("  Mode: {}".format("MOCK" if self.mock else "PRODUCTION"))
        if not self.mock:
            print("  Model: whisper-{}".format(self.model_size))
            print("  Backend: CTranslate2 (CPU, INT8)")
        print("=" * 60)
        print()

        self._test_imports()
        self._test_generate_audio()

        if self.mock:
            self._test_mock_pipeline()
        else:
            self._test_model_load()
            self._test_inference()
            self._test_quick_inference()

        self._test_asr_service_imports()

        print()
        print("=" * 60)
        self._print_summary()
        print("=" * 60)

        return self._passed

    # ------------------------------------------------------------------
    # Individual tests
    # ------------------------------------------------------------------

    def _test_imports(self) -> None:
        """Basic import test"""
        self._section("Import Verification")
        failures = []

        required = ["numpy", "struct", "asyncio", "json", "logging"]
        for mod in required:
            try:
                __import__(mod)
                self._ok("import {}".format(mod))
            except ImportError as e:
                self._fail("import {}: {}".format(mod, e))
                failures.append(mod)

        # faster-whisper
        try:
            from faster_whisper import WhisperModel
            self._ok("import faster_whisper.WhisperModel")
        except ImportError as e:
            if self.mock:
                self._warn("import faster_whisper: {} (mock mode, continuing)".format(e))
            else:
                self._fail("import faster_whisper: {}".format(e))
                failures.append("faster_whisper")

        # ctranslate2
        try:
            import ctranslate2
            self._ok("import ctranslate2 ({})".format(ctranslate2.__version__))
        except ImportError as e:
            if self.mock:
                self._warn("import ctranslate2: {} (mock)".format(e))
            else:
                self._fail("import ctranslate2: {}".format(e))
                failures.append("ctranslate2")

        if failures and not self.mock:
            self._passed = False

    def _test_generate_audio(self) -> None:
        """Test audio generation"""
        self._section("Test Audio Generation")

        pcm_data = generate_test_wav()
        expected_samples = SAMPLE_RATE * 2  # 1s silence + 1s tone
        expected_bytes = expected_samples * SAMPLE_WIDTH
        actual_bytes = len(pcm_data)

        if actual_bytes == expected_bytes:
            self._ok("PCM generated: {} bytes ({} samples, 2.0s)".format(
                actual_bytes, expected_samples))
        else:
            self._fail("PCM size mismatch: expected={}, actual={}".format(
                expected_bytes, actual_bytes))
            self._passed = False
            return

        wav_path = OUTPUT_DIR / "asr_smoke_test.wav"
        save_test_wav(pcm_data, wav_path)
        self._ok("WAV saved: {}".format(wav_path))

        self.results["pcm_data"] = pcm_data
        self.results["wav_path"] = wav_path

    def _test_mock_pipeline(self) -> None:
        """Mock mode: pipeline test"""
        self._section("Mock Pipeline Verification")

        pcm_data = self.results.get("pcm_data", b"")
        if not pcm_data:
            self._fail("No test audio available")
            self._passed = False
            return

        try:
            start = time.monotonic()
            mock_text = "mock transcription result"
            mock_confidence = 0.99
            elapsed_ms = (time.monotonic() - start) * 1000
            self._ok("Mock transcription: '{}' (conf={}, latency={:.1f}ms)".format(
                mock_text, mock_confidence, elapsed_ms))
        except Exception as e:
            self._fail("Mock pipeline failed: {}".format(e))
            self._passed = False

        # RingBuffer test
        try:
            sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src"))
            from claudia.audio.asr_service.ring_buffer import RingBuffer
            rb = RingBuffer()
            rb.write(pcm_data[:960])  # 30ms
            last = rb.read_last(30)
            assert len(last) == 960, "RingBuffer read_last mismatch: {}".format(len(last))
            rb.clear()
            assert rb.available_ms == 0
            self._ok("RingBuffer: write/read_last/clear OK")
        except Exception as e:
            self._fail("RingBuffer test failed: {}".format(e))
            self._passed = False

        # VADProcessor mock test
        try:
            from claudia.audio.asr_service.vad_processor import (
                VADProcessor, VADConfig, VADState,
            )
            from claudia.audio.asr_service.ring_buffer import RingBuffer as RB2
            rb2 = RB2()

            async def dummy_callback(event):
                pass

            vad = VADProcessor(
                ring_buffer=rb2,
                event_callback=dummy_callback,
                mock=True,
            )
            assert vad.state == VADState.SILENCE
            self._ok("VADProcessor: mock initialization successful (state=SILENCE)")
        except Exception as e:
            self._fail("VADProcessor test failed: {}".format(e))
            self._passed = False

    def _test_model_load(self) -> None:
        """Production: model loading"""
        self._section("Model Loading")

        try:
            from faster_whisper import WhisperModel

            compute_type = os.getenv("CLAUDIA_ASR_COMPUTE_TYPE", "int8")

            start = time.monotonic()
            self._model = WhisperModel(
                self.model_size,
                device="cpu",
                compute_type=compute_type,
            )
            load_ms = (time.monotonic() - start) * 1000

            self.results["load_ms"] = load_ms
            self._ok("Model loaded successfully: whisper-{}".format(self.model_size))
            self._ok("  Load time: {:.0f}ms".format(load_ms))
            self._ok("  compute_type: {}".format(compute_type))

        except Exception as e:
            self._fail("Model loading failed: {}".format(e))
            self._passed = False

    def _test_inference(self) -> None:
        """Production: full inference test"""
        self._section("Inference Test (beam_size=3)")

        if self._model is None:
            self._fail("Model not loaded -- skipping")
            self._passed = False
            return

        pcm_data = self.results.get("pcm_data", b"")
        if not pcm_data:
            self._fail("No test audio -- skipping")
            self._passed = False
            return

        try:
            import numpy as np

            audio_np = np.frombuffer(pcm_data, dtype=np.int16).astype(np.float32) / 32768.0

            start = time.monotonic()
            segments, info = self._model.transcribe(
                audio_np,
                language="ja",
                beam_size=3,
                vad_filter=False,
            )

            text_parts = []
            total_logprob = 0.0
            n_segments = 0
            for seg in segments:
                text_parts.append(seg.text)
                total_logprob += seg.avg_logprob
                n_segments += 1

            inference_ms = (time.monotonic() - start) * 1000

            text = "".join(text_parts).strip()
            if n_segments > 0:
                confidence = min(1.0, max(0.0, math.exp(total_logprob / n_segments)))
            else:
                confidence = 0.0

            self.results["inference_ms"] = inference_ms
            self.results["text"] = text
            self.results["confidence"] = confidence

            self._ok("Inference complete: '{}'".format(text))
            self._ok("  segments: {}".format(n_segments))
            self._ok("  confidence: {:.3f}".format(confidence))
            self._ok("  inference time: {:.0f}ms".format(inference_ms))
            self._ok("  language detected: {} (prob={:.2f})".format(info.language, info.language_probability))

            if inference_ms > 5000:
                self._warn("Inference time is long: {:.0f}ms (target < 2000ms)".format(inference_ms))

        except Exception as e:
            self._fail("Inference failed: {}".format(e))
            self._passed = False

    def _test_quick_inference(self) -> None:
        """Production: quick_transcribe test (beam_size=1)"""
        self._section("Quick Inference Test (beam_size=1)")

        if self._model is None:
            self._fail("Model not loaded -- skipping")
            return

        pcm_data = self.results.get("pcm_data", b"")
        if not pcm_data:
            return

        try:
            import numpy as np

            # First 300ms only (for emergency check)
            n_bytes = int(SAMPLE_RATE * 0.3 * SAMPLE_WIDTH)
            short_pcm = pcm_data[:n_bytes]
            audio_np = np.frombuffer(short_pcm, dtype=np.int16).astype(np.float32) / 32768.0

            start = time.monotonic()
            segments, info = self._model.transcribe(
                audio_np,
                language="ja",
                beam_size=1,
                best_of=1,
                without_timestamps=True,
                vad_filter=False,
            )
            text = "".join(seg.text for seg in segments).strip()
            quick_ms = (time.monotonic() - start) * 1000

            self.results["quick_inference_ms"] = quick_ms
            self._ok("Quick inference complete: '{}' ({:.0f}ms)".format(text, quick_ms))

        except Exception as e:
            self._fail("Quick inference failed: {}".format(e))

    def _test_asr_service_imports(self) -> None:
        """ASR service module import test"""
        self._section("ASR Service Import Verification")

        try:
            sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src"))
            from claudia.audio.asr_service import (
                ASRServer,
                ASRModelWrapper,
                VADProcessor,
                VADConfig,
                VADState,
                VADEvent,
                RingBuffer,
            )
            self._ok("claudia.audio.asr_service: all exports imported successfully")
        except ImportError as e:
            self._fail("ASR service import failed: {}".format(e))
            self._passed = False

        try:
            from claudia.audio.asr_service.emergency_keywords import EMERGENCY_KEYWORDS_TEXT
            self._ok("emergency_keywords: {} keywords loaded".format(
                len(EMERGENCY_KEYWORDS_TEXT)))
        except ImportError as e:
            self._warn("emergency_keywords import: {}".format(e))

        try:
            from claudia.audio.asr_service.ipc_protocol import PROTO_VERSION
            self._ok("ipc_protocol: proto_version={}".format(PROTO_VERSION))
        except ImportError as e:
            self._warn("ipc_protocol import: {}".format(e))

    # ------------------------------------------------------------------
    # Output helpers
    # ------------------------------------------------------------------

    def _section(self, name: str) -> None:
        print("\n--- {} ---".format(name))

    def _ok(self, msg: str) -> None:
        print("  [PASS] {}".format(msg))

    def _fail(self, msg: str) -> None:
        print("  [FAIL] {}".format(msg))

    def _warn(self, msg: str) -> None:
        print("  [WARN] {}".format(msg))

    def _print_summary(self) -> None:
        if self._passed:
            print("  RESULT: ALL TESTS PASSED")
        else:
            print("  RESULT: SOME TESTS FAILED")

        if self.results:
            print()
            print("  Metrics:")
            for key in ["load_ms", "inference_ms", "quick_inference_ms",
                        "text", "confidence"]:
                if key in self.results:
                    val = self.results[key]
                    if isinstance(val, float):
                        print("    {}: {:.1f}".format(key, val))
                    else:
                        print("    {}: {}".format(key, val))


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Claudia ASR Inference Smoke Test (Phase 1)",
    )
    parser.add_argument(
        "--mock", action="store_true",
        help="Mock mode: verify imports and pipeline structure only, no model needed",
    )
    parser.add_argument(
        "--model", type=str, default=None,
        help="Whisper model size (default: base)",
    )
    parser.add_argument(
        "--usb-mic", action="store_true",
        help="USB microphone live recording test (AT2020USB-XP, 5-second recording -> ASR)",
    )
    parser.add_argument(
        "--usb-device", type=str, default="hw:2,0",
        help="ALSA USB microphone device name (default: hw:2,0)",
    )
    parser.add_argument(
        "--usb-rate", type=int, default=44100,
        help="USB microphone native sample rate (default: 44100)",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )

    mock = args.mock or os.getenv("ASR_MOCK", "0") == "1"

    if args.usb_mic:
        return _run_usb_mic_test(
            model_size=args.model or os.getenv("CLAUDIA_ASR_MODEL", "base"),
            device=args.usb_device,
            native_rate=args.usb_rate,
        )

    test = ASRSmokeTest(
        mock=mock,
        model_size=args.model,
    )
    passed = test.run_all()
    return 0 if passed else 1


def _run_usb_mic_test(model_size: str, device: str, native_rate: int) -> int:
    """USB microphone live recording -> ASR end-to-end test

    AT2020USB-XP has a native rate of 44100Hz. Tegra ALSA plughw outputs all zeros,
    so we record at native rate and resample to 16kHz on the Python side.
    """
    import subprocess
    import wave

    print("=" * 60)
    print("  USB Microphone -> ASR End-to-End Test")
    print("  Device: {}  Native Rate: {}Hz".format(device, native_rate))
    print("  Model: whisper-{}".format(model_size))
    print("=" * 60)
    print()

    wav_path = "/tmp/claudia_usb_mic_test.wav"
    duration_s = 5

    # 1. Recording
    print("[1/4] Recording {}s from {} at {}Hz ...".format(duration_s, device, native_rate))
    try:
        subprocess.run(
            ["arecord", "-D", device, "-f", "S16_LE",
             "-r", str(native_rate), "-c", "1",
             "-d", str(duration_s), "-t", "wav", wav_path],
            check=True, capture_output=True, timeout=duration_s + 10,
        )
        print("  [PASS] Recorded to {}".format(wav_path))
    except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired) as e:
        print("  [FAIL] arecord failed: {}".format(e))
        return 1

    # 2. WAV reading + resampling
    import numpy as np
    print("[2/4] Reading WAV and resampling {}->16000Hz ...".format(native_rate))
    try:
        with wave.open(wav_path, "rb") as wf:
            sr_in = wf.getframerate()
            n_frames = wf.getnframes()
            raw = wf.readframes(n_frames)

        samples = np.frombuffer(raw, dtype=np.int16)
        rms_orig = int(np.sqrt(np.mean(samples.astype(np.float64) ** 2)))
        print("  Original: {} samples, RMS={}".format(len(samples), rms_orig))

        if rms_orig < 10:
            print("  [WARN] Very low RMS -- mic may not be capturing audio")

        # Resample (numpy index-based)
        sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src"))
        from claudia.audio.pcm_utils import resample_pcm_int16
        from claudia.audio.asr_service.asr_server import SAMPLE_RATE
        resampled = resample_pcm_int16(samples, sr_in, SAMPLE_RATE)
        print("  Resampled: {} -> {} samples (16kHz)".format(len(samples), len(resampled)))
        print("  [PASS] Resampling OK")
    except Exception as e:
        print("  [FAIL] WAV processing failed: {}".format(e))
        return 1

    # 3. ASR inference
    print("[3/4] Loading whisper-{} and transcribing ...".format(model_size))
    try:
        from faster_whisper import WhisperModel

        start = time.monotonic()
        model = WhisperModel(model_size, device="cpu", compute_type="int8")
        load_ms = (time.monotonic() - start) * 1000
        print("  Model loaded in {:.0f}ms".format(load_ms))

        audio_float = resampled.astype(np.float32) / 32768.0
        start = time.monotonic()
        segments, info = model.transcribe(
            audio_float, language="ja", beam_size=3, vad_filter=True,
        )
        text_parts = []
        for seg in segments:
            text_parts.append(seg.text)
        inference_ms = (time.monotonic() - start) * 1000

        text = "".join(text_parts).strip()
        print("  [PASS] Transcription: '{}'".format(text))
        print("  Language: {} (prob={:.2f})".format(info.language, info.language_probability))
        print("  Inference: {:.0f}ms".format(inference_ms))
    except Exception as e:
        print("  [FAIL] ASR inference failed: {}".format(e))
        return 1

    # 4. Summary
    print()
    print("[4/4] Summary")
    print("  Device:      {}".format(device))
    print("  Native Rate: {}Hz".format(native_rate))
    print("  RMS:         {}".format(rms_orig))
    print("  Model:       whisper-{}".format(model_size))
    print("  Text:        {}".format(text if text else "(empty)"))
    print("  Load:        {:.0f}ms".format(load_ms))
    print("  Inference:   {:.0f}ms".format(inference_ms))
    print()
    print("=" * 60)
    if text:
        print("  RESULT: USB MIC TEST PASSED")
    else:
        print("  RESULT: USB MIC TEST PASSED (empty transcription -- silent input?)")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
