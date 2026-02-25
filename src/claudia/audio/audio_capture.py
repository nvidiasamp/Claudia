#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
USB Microphone Audio Capture -- arecord async subprocess

AT2020USB-XP (hw:2,0, 44100Hz) -> resample 16kHz -> UDS audio.sock
Sends frames (960 bytes = 30ms @ 16kHz 16bit mono) to the ASR server.
"""

import asyncio
import logging
import os
import re
import time
from typing import Optional

from .pcm_utils import resample_pcm_int16
from .asr_service.ipc_protocol import AUDIO_SOCKET, connect_uds

logger = logging.getLogger("claudia.audio.capture")

# PCM constants
SRC_RATE = 44100       # USB microphone native rate
DST_RATE = 16000       # ASR server input rate
FRAME_SAMPLES = 480    # 30ms @ 16kHz
FRAME_BYTES = FRAME_SAMPLES * 2  # 960 bytes (16-bit mono)

# Chunk size to read from arecord at once: 100ms @ 44100Hz 16bit mono = 8820 bytes
ARECORD_CHUNK_SAMPLES = 4410  # 100ms
ARECORD_CHUNK_BYTES = ARECORD_CHUNK_SAMPLES * 2  # 8820 bytes

# Retry settings
RETRY_DELAY_INIT = 1.0
RETRY_DELAY_MAX = 30.0


class AudioCapture:
    """USB Microphone Audio Capture

    Captures PCM via an arecord async subprocess, resamples to 16kHz,
    and sends frame-by-frame to the UDS audio socket.

    Args:
        device: ALSA device name (e.g., "hw:2,0"). None for auto-detection.
        mock: If True, generates silent frames (no microphone needed)
        socket_path: Audio socket path (default: ipc_protocol.AUDIO_SOCKET)
    """

    def __init__(
        self,
        device: Optional[str] = None,
        mock: bool = False,
        socket_path: str = AUDIO_SOCKET,
    ) -> None:
        self._explicit_device = device or os.environ.get("CLAUDIA_AUDIO_DEVICE")
        self._device = self._explicit_device
        self._mock = mock
        self._socket_path = socket_path
        self._running = False
        self._process: Optional[asyncio.subprocess.Process] = None
        self._writer: Optional[asyncio.StreamWriter] = None
        self._frame_buffer = b""

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    async def run(self) -> None:
        """Main loop: device detection -> socket connection -> capture

        Retries with exponential backoff on USB disconnect/reconnect.
        """
        self._running = True
        retry_delay = RETRY_DELAY_INIT

        while self._running:
            try:
                # 1. Device detection (skipped in mock mode)
                if not self._mock and not self._device:
                    self._device = await self._discover_device()

                # 2. Socket connection
                await self._connect_socket()

                # 3. Capture loop
                await self._capture_loop()

                # Normal exit (running=False)
                break

            except (ConnectionError, OSError) as e:
                if not self._running:
                    break
                logger.warning("Capture error: %s -- retrying in %.1fs", e, retry_delay)
                await asyncio.sleep(retry_delay)
                retry_delay = min(retry_delay * 2, RETRY_DELAY_MAX)
                # Re-detect if no explicit device specified (keep explicit device)
                if not self._explicit_device:
                    self._device = None

            except asyncio.CancelledError:
                break

            except Exception as e:
                if not self._running:
                    break
                logger.error("Unexpected capture error: %s", e, exc_info=True)
                await asyncio.sleep(retry_delay)
                retry_delay = min(retry_delay * 2, RETRY_DELAY_MAX)

        await self._cleanup()

    async def shutdown(self) -> None:
        """Graceful shutdown"""
        logger.info("AudioCapture shutting down...")
        self._running = False
        await self._cleanup()

    # ------------------------------------------------------------------
    # Device Detection
    # ------------------------------------------------------------------

    async def _discover_device(self) -> str:
        """Auto-detect AT2020 USB microphone by parsing arecord -l output

        Returns:
            ALSA device name (e.g., "hw:2,0")

        Raises:
            RuntimeError: Device not found
        """
        try:
            proc = await asyncio.create_subprocess_exec(
                "arecord", "-l",
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.DEVNULL,
            )
            stdout, _ = await proc.communicate()
            output = stdout.decode("utf-8", errors="replace")
        except FileNotFoundError:
            raise RuntimeError("arecord command not found (alsa-utils not installed?)")

        # Look for lines matching "card N: ... [AT2020...]"
        pattern = re.compile(r"card\s+(\d+):.*AT2020", re.IGNORECASE)
        for line in output.splitlines():
            m = pattern.search(line)
            if m:
                card_num = m.group(1)
                device = "hw:{},0".format(card_num)
                logger.info("AT2020 USB microphone detected: %s", device)
                return device

        raise RuntimeError(
            "AT2020 USB microphone not found. "
            "Please specify the device via CLAUDIA_AUDIO_DEVICE environment variable."
        )

    # ------------------------------------------------------------------
    # Socket Connection
    # ------------------------------------------------------------------

    async def _connect_socket(self) -> None:
        """Connect to audio.sock"""
        if self._writer:
            try:
                self._writer.close()
            except Exception:
                pass
            self._writer = None

        _, writer = await connect_uds(self._socket_path, retries=10, delay=1.0)
        self._writer = writer
        self._frame_buffer = b""
        logger.info("audio.sock connection established: %s", self._socket_path)

    # ------------------------------------------------------------------
    # Capture Loop
    # ------------------------------------------------------------------

    async def _capture_loop(self) -> None:
        """arecord -> read -> resample -> send frames"""
        if self._mock:
            await self._mock_capture_loop()
            return

        # Start arecord
        self._process = await asyncio.create_subprocess_exec(
            "arecord",
            "-D", self._device,
            "-f", "S16_LE",
            "-r", str(SRC_RATE),
            "-c", "1",
            "-t", "raw",
            "--buffer-size", "8192",
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.DEVNULL,
        )
        logger.info("arecord started: device=%s, rate=%d", self._device, SRC_RATE)

        loop = asyncio.get_event_loop()

        try:
            while self._running and self._process.returncode is None:
                # Async read 100ms of data
                data = await self._process.stdout.readexactly(ARECORD_CHUNK_BYTES)

                # Resample (CPU bound -> thread pool)
                import numpy as np
                samples_in = np.frombuffer(data, dtype=np.int16)
                samples_out = await loop.run_in_executor(
                    None, resample_pcm_int16, samples_in, SRC_RATE, DST_RATE,
                )
                resampled_bytes = samples_out.tobytes()

                # Add to frame buffer -> send complete frames
                await self._send_frames(resampled_bytes)

        except asyncio.IncompleteReadError:
            if self._running:
                logger.warning("arecord terminated unexpectedly (USB disconnected?)")
                raise ConnectionError("arecord process terminated")
        finally:
            await self._kill_process()

    async def _mock_capture_loop(self) -> None:
        """Mock mode: send silent frames at 30ms intervals"""
        silent_frame = b"\x00" * FRAME_BYTES
        logger.info("Mock capture mode started")

        while self._running:
            try:
                self._writer.write(silent_frame)
                await self._writer.drain()
            except (ConnectionError, OSError):
                if self._running:
                    raise
                break
            await asyncio.sleep(0.03)  # 30ms

    # ------------------------------------------------------------------
    # Frame Sending
    # ------------------------------------------------------------------

    async def _send_frames(self, resampled_bytes: bytes) -> None:
        """Combine with residual buffer and send complete 960 byte frames"""
        self._frame_buffer += resampled_bytes

        while len(self._frame_buffer) >= FRAME_BYTES:
            frame = self._frame_buffer[:FRAME_BYTES]
            self._frame_buffer = self._frame_buffer[FRAME_BYTES:]
            try:
                self._writer.write(frame)
                await self._writer.drain()
            except (ConnectionError, OSError) as e:
                if self._running:
                    raise ConnectionError("audio.sock write failed: {}".format(e))
                break

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    async def _kill_process(self) -> None:
        """Terminate the arecord process"""
        if self._process and self._process.returncode is None:
            try:
                self._process.terminate()
                try:
                    await asyncio.wait_for(self._process.wait(), timeout=3.0)
                except asyncio.TimeoutError:
                    self._process.kill()
                    await self._process.wait()
            except ProcessLookupError:
                pass
            self._process = None

    async def _cleanup(self) -> None:
        """Release all resources"""
        await self._kill_process()

        if self._writer:
            try:
                self._writer.close()
            except Exception:
                pass
            self._writer = None

        self._frame_buffer = b""
