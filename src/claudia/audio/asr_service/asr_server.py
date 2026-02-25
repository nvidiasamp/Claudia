#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ASR Service Main Entry -- Python 3.8 system process

3-way unidirectional UDS (under $XDG_RUNTIME_DIR/claudia/):
- audio.sock  (receives PCM audio stream)
- result.sock (sends JSON Lines: transcript/emergency/heartbeat)
- ctrl.sock   (receives JSON Lines: tts_start/tts_end/shutdown)

After startup, loads faster-whisper model (CTranslate2 backend) and sends handshake ready message.
Supports --mock flag or ASR_MOCK=1 environment variable; mock mode does not load model.
"""

import argparse
import asyncio
import json
import logging
import math
import os
import signal
import stat
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

from .ring_buffer import RingBuffer, BYTES_PER_MS
from .vad_processor import VADProcessor, VADConfig, VADEvent

logger = logging.getLogger("claudia.asr.server")

# ======================================================================
# Constants
# ======================================================================

# Socket paths obtained from ipc_protocol's single definition point (DRY)
# Supports both standalone process (python3 asr_server.py) and package import
try:
    from ipc_protocol import (
        AUDIO_SOCKET, ASR_RESULT_SOCKET as RESULT_SOCKET, ASR_CTRL_SOCKET as CTRL_SOCKET,
        SESSION_TOKEN_FILE, generate_session_token, validate_session_token,
    )
except ImportError:
    from .ipc_protocol import (
        AUDIO_SOCKET, ASR_RESULT_SOCKET as RESULT_SOCKET, ASR_CTRL_SOCKET as CTRL_SOCKET,
        SESSION_TOKEN_FILE, generate_session_token, validate_session_token,
    )

HEARTBEAT_INTERVAL_S = 5
TTS_GATE_TIMEOUT_S = 30
PROTO_VERSION = "1.0"

# PCM parameters: 16kHz, 16-bit, mono (internal processing standard)
SAMPLE_RATE = 16000
FRAME_MS = 30
FRAME_BYTES = FRAME_MS * BYTES_PER_MS  # 960 bytes = 30ms


# ======================================================================
# Audio resampling utility (shared module)
# ======================================================================

from ..pcm_utils import resample_pcm_int16  # noqa: F401 -- extracted to pcm_utils.py


# ======================================================================
# ASR Model Wrapper
# ======================================================================

class ASRModelWrapper:
    """faster-whisper (CTranslate2) model wrapper

    Encapsulates model loading and inference, supports mock mode.
    Uses CTranslate2 backend, no PyTorch dependency for inference.

    Environment variables:
    - CLAUDIA_ASR_MODEL: whisper model name (tiny/base/small/medium/large-v3), default base
    - CLAUDIA_ASR_DEVICE: cpu or cuda (default cpu)
    - CLAUDIA_ASR_COMPUTE_TYPE: int8/float16/float32 (default int8)
    """

    def __init__(self, mock: bool = False) -> None:
        self._mock = mock
        self._model: Optional[Any] = None
        self._model_size: str = ""
        self._compute_type: str = ""
        self._ram_mb: int = 0

    def load(self) -> None:
        """Load faster-whisper model"""
        if self._mock:
            logger.info("ASR mock mode, skipping model loading")
            self._ram_mb = 0
            return

        try:
            from faster_whisper import WhisperModel

            model_size = os.getenv("CLAUDIA_ASR_MODEL", "base")
            device = os.getenv("CLAUDIA_ASR_DEVICE", "cpu")
            compute_type = os.getenv("CLAUDIA_ASR_COMPUTE_TYPE", "int8")

            logger.info("ASR model loading: whisper-%s (device=%s, compute=%s)",
                        model_size, device, compute_type)

            self._model = WhisperModel(
                model_size,
                device=device,
                compute_type=compute_type,
            )
            self._model_size = model_size
            self._compute_type = compute_type

            # CTranslate2 does not use PyTorch VRAM, estimate RAM usage
            size_ram_map = {
                "tiny": 75, "base": 150, "small": 500,
                "medium": 1500, "large-v3": 3000,
            }
            self._ram_mb = size_ram_map.get(model_size, 500)

            logger.info("ASR model loaded: whisper-%s (RAM ~%dMB)",
                        model_size, self._ram_mb)

        except Exception as e:
            logger.error("ASR model loading failed: %s", e)
            logger.warning("Falling back to mock mode")
            self._mock = True
            self._ram_mb = 0

    def transcribe(self, audio_data: bytes,
                   sample_rate: int = SAMPLE_RATE) -> Tuple[str, float]:
        """Full speech segment ASR transcription

        Parameters
        ----------
        audio_data : bytes
            16-bit mono PCM (any sample rate, auto-resampled if not 16kHz)
        sample_rate : int
            Input audio sample rate (default 16000). USB microphones like AT2020USB-XP
            natively at 44100Hz; ALSA plughw on Tegra produces all-zeros during resampling,
            so raw 44100Hz data must be passed for resampling by this method.

        Returns
        -------
        (text, confidence) : Tuple[str, float]
        """
        if self._mock:
            return ("mock transcription result", 0.99)

        try:
            import numpy as np

            audio_int16 = np.frombuffer(audio_data, dtype=np.int16)
            if sample_rate != SAMPLE_RATE:
                audio_int16 = resample_pcm_int16(audio_int16, sample_rate, SAMPLE_RATE)
            audio_np = audio_int16.astype(np.float32) / 32768.0

            # beam_size: accuracy vs speed trade-off
            # beam=1 (default): greedy decoder, fastest for short commands
            # beam=3+: beam search, accuracy priority for long text
            # CLAUDIA_ASR_BEAM_SIZE allows runtime switching
            beam = int(os.getenv("CLAUDIA_ASR_BEAM_SIZE", "1"))
            segments, info = self._model.transcribe(
                audio_np,
                language="ja",
                beam_size=beam,
                best_of=1,
                without_timestamps=True,
                vad_filter=False,  # External VAD already processed
            )

            text_parts = []
            total_logprob = 0.0
            n_segments = 0
            for seg in segments:
                text_parts.append(seg.text)
                total_logprob += seg.avg_logprob
                n_segments += 1

            text = "".join(text_parts).strip()

            # avg_logprob -> confidence (0-1)
            if n_segments > 0:
                avg_logprob = total_logprob / n_segments
                confidence = min(1.0, max(0.0, math.exp(avg_logprob)))
            else:
                confidence = 0.0

            return (text, confidence)

        except Exception as e:
            logger.error("ASR inference failed: %s", e)
            return ("", 0.0)

    def quick_transcribe(self, audio_data: bytes,
                         sample_rate: int = SAMPLE_RATE) -> Tuple[str, float]:
        """Short segment quick transcription (for Emergency fast detector)

        beam_size=1 + best_of=1 to maximize speed.

        Parameters
        ----------
        audio_data : bytes
            16-bit mono PCM (any sample rate)
        sample_rate : int
            Input audio sample rate (default 16000)
        """
        if self._mock:
            return ("mock transcription result", 0.99)

        try:
            import numpy as np

            audio_int16 = np.frombuffer(audio_data, dtype=np.int16)
            if sample_rate != SAMPLE_RATE:
                audio_int16 = resample_pcm_int16(audio_int16, sample_rate, SAMPLE_RATE)
            audio_np = audio_int16.astype(np.float32) / 32768.0

            segments, info = self._model.transcribe(
                audio_np,
                language="ja",
                beam_size=1,
                best_of=1,
                without_timestamps=True,
                vad_filter=False,
            )

            text_parts = []
            total_logprob = 0.0
            n_segments = 0
            for seg in segments:
                text_parts.append(seg.text)
                total_logprob += seg.avg_logprob
                n_segments += 1

            text = "".join(text_parts).strip()

            if n_segments > 0:
                avg_logprob = total_logprob / n_segments
                confidence = min(1.0, max(0.0, math.exp(avg_logprob)))
            else:
                confidence = 0.0

            return (text, confidence)

        except Exception as e:
            logger.error("Emergency ASR quick_transcribe failed: %s", e)
            return ("", 0.0)

    @property
    def vram_mb(self) -> int:
        """RAM estimate (CTranslate2 CPU mode has no VRAM usage)"""
        return self._ram_mb

    @property
    def model_name(self) -> str:
        return f"whisper-{self._model_size}" if self._model_size else "mock"

    @property
    def is_mock(self) -> bool:
        return self._mock


# ======================================================================
# ASR Server
# ======================================================================

class ASRServer:
    """ASR main service

    Manages 3-way UDS, VAD processing, ASR inference, TTS echo gating.
    """

    def __init__(self, mock: bool = False) -> None:
        self._mock = mock
        self._running = False

        # ctrl socket session token (regenerated at each startup)
        self._session_token = generate_session_token()

        # ASR model
        self._asr = ASRModelWrapper(mock=mock)

        # Audio ring buffer
        self._ring = RingBuffer()

        # Result writer (assigned after result socket connection)
        self._result_writer: Optional[asyncio.StreamWriter] = None
        self._result_lock = asyncio.Lock()

        # TTS echo gate
        self._tts_gate = False
        self._tts_gate_timer: Optional[asyncio.TimerHandle] = None

        # VAD processor (initialized after model loading)
        self._vad: Optional[VADProcessor] = None

        # Heartbeat task
        self._heartbeat_task: Optional[asyncio.Task] = None  # type: ignore[type-arg]

        # Server references (for shutdown)
        self._audio_server: Optional[asyncio.AbstractServer] = None
        self._ctrl_server: Optional[asyncio.AbstractServer] = None
        self._result_server: Optional[asyncio.AbstractServer] = None

    # ------------------------------------------------------------------
    # Startup / Shutdown
    # ------------------------------------------------------------------

    async def start(self) -> None:
        """Start ASR service"""
        logger.info("ASR service starting (mock=%s)...", self._mock)

        # 1. Load ASR model
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self._asr.load)

        # 2. Initialize VAD
        self._vad = VADProcessor(
            ring_buffer=self._ring,
            event_callback=self._handle_vad_event,
            quick_transcriber=self._asr,
            mock=self._mock,
        )

        # 3. Clean up old socket files
        for sock_path in (AUDIO_SOCKET, RESULT_SOCKET, CTRL_SOCKET):
            if os.path.exists(sock_path):
                os.unlink(sock_path)

        # 3.5. Write session token to file (0o600)
        try:
            fd = os.open(SESSION_TOKEN_FILE, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
            with os.fdopen(fd, "w") as f:
                f.write(self._session_token)
            logger.info("Session token written: %s", SESSION_TOKEN_FILE)
        except OSError as e:
            logger.warning("Session token write failed: %s", e)

        # 4. Start 3-way UDS servers (restrict permissions to owner-only after creation)
        self._result_server = await asyncio.start_unix_server(
            self._handle_result_connection, path=RESULT_SOCKET,
        )
        self._audio_server = await asyncio.start_unix_server(
            self._handle_audio_connection, path=AUDIO_SOCKET,
        )
        self._ctrl_server = await asyncio.start_unix_server(
            self._handle_ctrl_connection, path=CTRL_SOCKET,
        )
        for sock_path in (AUDIO_SOCKET, RESULT_SOCKET, CTRL_SOCKET):
            try:
                os.chmod(sock_path, stat.S_IRUSR | stat.S_IWUSR)  # 0o600
            except OSError as e:
                logger.warning("Socket permission setting failed: %s: %s", sock_path, e)

        self._running = True

        # 5. Start heartbeat
        self._heartbeat_task = asyncio.ensure_future(self._heartbeat_loop())

        logger.info("ASR service ready (audio=%s, result=%s, ctrl=%s)",
                     AUDIO_SOCKET, RESULT_SOCKET, CTRL_SOCKET)

    async def shutdown(self) -> None:
        """Graceful shutdown"""
        if not self._running:
            return

        logger.info("ASR service shutting down...")
        self._running = False

        # Cancel heartbeat
        if self._heartbeat_task and not self._heartbeat_task.done():
            self._heartbeat_task.cancel()
            try:
                await self._heartbeat_task
            except asyncio.CancelledError:
                pass

        # Cancel TTS gate timer
        if self._tts_gate_timer:
            self._tts_gate_timer.cancel()

        # Close servers
        for server in (self._audio_server, self._result_server, self._ctrl_server):
            if server:
                server.close()
                await server.wait_closed()

        # Close result writer
        if self._result_writer:
            try:
                self._result_writer.close()
                await self._result_writer.wait_closed()
            except Exception:
                pass

        # Clean up socket files + token file
        for sock_path in (AUDIO_SOCKET, RESULT_SOCKET, CTRL_SOCKET, SESSION_TOKEN_FILE):
            if os.path.exists(sock_path):
                try:
                    os.unlink(sock_path)
                except OSError:
                    pass

        logger.info("ASR service shutdown complete")

    # ------------------------------------------------------------------
    # UDS connection handling
    # ------------------------------------------------------------------

    async def _handle_result_connection(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        """Result socket connection handler: save writer reference, send handshake"""
        logger.info("Result socket client connected")
        async with self._result_lock:
            self._result_writer = writer

        # Send handshake ready message
        await self._emit_result({
            "type": "ready",
            "model": self._asr.model_name,
            "vram_mb": self._asr.vram_mb,
            "proto_version": PROTO_VERSION,
        })

        # Keep connection until close
        try:
            while self._running:
                # result socket is unidirectional ASR->Main, no reading needed
                # but need to detect disconnection
                data = await reader.read(1)
                if not data:
                    break
                await asyncio.sleep(0.1)
        except (asyncio.CancelledError, ConnectionError):
            pass
        finally:
            logger.info("Result socket client disconnected")
            writer.close()
            async with self._result_lock:
                # Only clear current writer to prevent accidentally clearing a new connection's writer
                if self._result_writer is writer:
                    self._result_writer = None

    async def _handle_audio_connection(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        """Audio socket connection handler: receive PCM stream -> ring buffer -> VAD"""
        logger.info("Audio socket client connected")
        loop = asyncio.get_event_loop()

        try:
            while self._running:
                # Read one PCM frame (30ms = 960 bytes)
                data = await reader.readexactly(FRAME_BYTES)

                # Write to ring buffer (threading.Lock protected, 960 bytes write <1us)
                # Don't use executor: overhead (~1-2ms) far exceeds actual lock hold time
                self._ring.write(data)

                # TTS echo gate: skip VAD during playback
                if self._tts_gate:
                    continue

                # VAD processing (silero-vad CPU inference, needs executor to avoid blocking)
                if self._vad:
                    events = await loop.run_in_executor(None, self._vad.process_frame, data)
                    for event in events:
                        await self._handle_vad_event(event)

        except asyncio.IncompleteReadError:
            logger.info("Audio stream ended (incomplete frame)")
        except (asyncio.CancelledError, ConnectionError):
            pass
        finally:
            logger.info("Audio socket client disconnected")
            writer.close()

    async def _handle_ctrl_connection(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        """Control socket connection handler: receive JSON Lines control messages"""
        logger.info("Control socket client connected")

        try:
            while self._running:
                line = await reader.readline()
                if not line:
                    break

                try:
                    msg = json.loads(line.decode("utf-8").strip())
                except (json.JSONDecodeError, UnicodeDecodeError) as e:
                    logger.warning("Invalid control message: %s", e)
                    continue

                # Session token verification
                if not validate_session_token(msg, self._session_token):
                    logger.warning("Invalid session token: control message rejected (type=%s)",
                                   msg.get("type", "?"))
                    continue

                msg_type = msg.get("type", "")
                await self._handle_ctrl_message(msg_type, msg)

        except (asyncio.CancelledError, ConnectionError):
            pass
        finally:
            logger.info("Control socket client disconnected")
            writer.close()

    # ------------------------------------------------------------------
    # Control message handling
    # ------------------------------------------------------------------

    async def _handle_ctrl_message(self, msg_type: str, msg: Dict[str, Any]) -> None:
        """Handle control messages"""
        if msg_type == "tts_start":
            logger.info("TTS echo gate: enabled")
            self._tts_gate = True
            # Reset VAD (discard current speech segment)
            if self._vad:
                self._vad.reset()
            # Set timeout protection
            loop = asyncio.get_event_loop()
            if self._tts_gate_timer:
                self._tts_gate_timer.cancel()
            self._tts_gate_timer = loop.call_later(
                TTS_GATE_TIMEOUT_S, self._force_gate_open,
            )

        elif msg_type == "tts_end":
            logger.info("TTS echo gate: disabled")
            self._tts_gate = False
            if self._tts_gate_timer:
                self._tts_gate_timer.cancel()
                self._tts_gate_timer = None

        elif msg_type == "shutdown":
            reason = msg.get("reason", "requested")
            logger.info("Shutdown control message received (reason=%s)", reason)
            await self.shutdown()

        else:
            logger.warning("Unknown control message type: %s", msg_type)

    def _force_gate_open(self) -> None:
        """TTS gate timeout protection: force restore VAD + audit after 30s"""
        logger.warning("TTS echo gate timeout (%ds), forcing restore", TTS_GATE_TIMEOUT_S)
        self._tts_gate = False
        self._tts_gate_timer = None
        # Async send audit event (catch exceptions to avoid silent loss)
        task = asyncio.ensure_future(self._emit_result({
            "type": "gate_timeout_audit",
            "ts": time.time(),
        }))
        task.add_done_callback(self._log_task_exception)

    @staticmethod
    def _log_task_exception(task: "asyncio.Task") -> None:
        """ensure_future callback: log uncaught exceptions to prevent silent loss"""
        if not task.cancelled() and task.exception() is not None:
            logger.error("Async task exception: %s", task.exception())

    # ------------------------------------------------------------------
    # VAD event handling
    # ------------------------------------------------------------------

    async def _handle_vad_event(self, event: VADEvent) -> None:
        """Handle events emitted by VAD state machine"""
        if event.event_type == "vad_start":
            await self._emit_result({
                "type": "vad_start",
                "utterance_id": event.utterance_id,
            })

        elif event.event_type == "vad_end":
            await self._emit_result({
                "type": "vad_end",
                "duration_ms": event.duration_ms,
                "utterance_id": event.utterance_id,
            })

        elif event.event_type == "emergency":
            logger.warning("Emergency event: keyword='%s' (conf=%.2f, utt=%s)",
                           event.keyword, event.confidence, event.utterance_id)
            await self._emit_result({
                "type": "emergency",
                "keyword": event.keyword,
                "confidence": event.confidence,
                "utterance_id": event.utterance_id,
            })

        elif event.event_type == "transcript_request":
            # Full speech segment ASR transcription (run in thread pool to avoid blocking event loop)
            await self._run_full_transcription(event)

    async def _run_full_transcription(self, event: VADEvent) -> None:
        """Full speech segment ASR transcription"""
        utterance_id = event.utterance_id
        audio_data = event.audio_data
        duration_ms = event.duration_ms

        if not audio_data:
            return

        logger.info("ASR transcription started: utt=%s, duration=%dms, audio=%d bytes",
                     utterance_id, duration_ms, len(audio_data))

        start_time = time.monotonic()

        # Run ASR inference in thread pool (30s timeout protection to prevent Whisper stalling)
        loop = asyncio.get_event_loop()
        try:
            text, confidence = await asyncio.wait_for(
                loop.run_in_executor(None, self._asr.transcribe, audio_data),
                timeout=30.0,
            )
        except asyncio.TimeoutError:
            asr_latency_ms = int((time.monotonic() - start_time) * 1000)
            logger.error("ASR transcription timeout (30s): utt=%s, duration=%dms, latency=%dms",
                         utterance_id, duration_ms, asr_latency_ms)
            return

        asr_latency_ms = int((time.monotonic() - start_time) * 1000)

        if text:
            logger.info("ASR result: '%s' (conf=%.2f, latency=%dms, utt=%s)",
                         text, confidence, asr_latency_ms, utterance_id)
            await self._emit_result({
                "type": "transcript",
                "text": text,
                "confidence": confidence,
                "duration_ms": duration_ms,
                "asr_latency_ms": asr_latency_ms,
                "utterance_id": utterance_id,
            })
        else:
            logger.warning("ASR transcription returned no result (utt=%s, latency=%dms)",
                           utterance_id, asr_latency_ms)

    # ------------------------------------------------------------------
    # Result sending
    # ------------------------------------------------------------------

    async def _emit_result(self, msg: Dict[str, Any]) -> None:
        """Send JSON Lines message via result socket"""
        async with self._result_lock:
            if self._result_writer is None:
                return
            try:
                line = json.dumps(msg, ensure_ascii=False) + "\n"
                self._result_writer.write(line.encode("utf-8"))
                await self._result_writer.drain()
            except (ConnectionError, OSError) as e:
                logger.warning("Result socket write failed: %s", e)
                self._result_writer = None

    # ------------------------------------------------------------------
    # Heartbeat
    # ------------------------------------------------------------------

    async def _heartbeat_loop(self) -> None:
        """Periodically send heartbeat messages"""
        try:
            while self._running:
                await asyncio.sleep(HEARTBEAT_INTERVAL_S)
                if self._running:
                    await self._emit_result({
                        "type": "heartbeat",
                        "ts": time.time(),
                    })
        except asyncio.CancelledError:
            pass


# ======================================================================
# Entry point
# ======================================================================

def _setup_logging() -> None:
    """Configure logging"""
    log_level = os.getenv("CLAUDIA_ASR_LOG_LEVEL", "INFO").upper()
    logging.basicConfig(
        level=getattr(logging, log_level, logging.INFO),
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments"""
    parser = argparse.ArgumentParser(description="Claudia ASR Service")
    parser.add_argument(
        "--mock", action="store_true",
        help="Mock mode: does not load CUDA model, for testing",
    )
    return parser.parse_args()


async def _async_main(mock: bool) -> None:
    """Async main function"""
    server = ASRServer(mock=mock)

    # Signal handling
    loop = asyncio.get_event_loop()
    for sig in (signal.SIGTERM, signal.SIGINT):
        def _shutdown_handler():
            t = asyncio.ensure_future(server.shutdown())
            t.add_done_callback(ASRServer._log_task_exception)
        loop.add_signal_handler(sig, _shutdown_handler)

    await server.start()

    # Wait until shutdown
    try:
        while server._running:
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        pass
    finally:
        await server.shutdown()


def main() -> None:
    """Synchronous entry point"""
    _setup_logging()
    args = _parse_args()
    mock = args.mock or os.getenv("ASR_MOCK", "0") == "1"

    if mock:
        logger.info("ASR service starting in mock mode")
    else:
        logger.info("ASR service starting in production mode")

    asyncio.run(_async_main(mock))


if __name__ == "__main__":
    main()
