#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AudioCapture unit tests"""

import asyncio
import os
import sys
import unittest
from unittest.mock import MagicMock, patch, AsyncMock

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_PROJECT_ROOT, "src"))

from claudia.audio.audio_capture import (
    AudioCapture,
    FRAME_BYTES,
    SRC_RATE,
    DST_RATE,
    ARECORD_CHUNK_BYTES,
)


def _run(coro):
    """Python 3.8 compatible asyncio test helper"""
    loop = asyncio.new_event_loop()
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


class TestAudioCaptureInit(unittest.TestCase):
    """Initialization tests"""

    def test_default_init(self):
        cap = AudioCapture()
        self.assertFalse(cap._mock)
        self.assertIsNone(cap._device)
        self.assertFalse(cap._running)

    def test_mock_mode(self):
        cap = AudioCapture(mock=True)
        self.assertTrue(cap._mock)

    def test_explicit_device(self):
        cap = AudioCapture(device="hw:3,0")
        self.assertEqual(cap._device, "hw:3,0")

    def test_env_device_override(self):
        with patch.dict(os.environ, {"CLAUDIA_AUDIO_DEVICE": "hw:5,0"}):
            cap = AudioCapture()
            self.assertEqual(cap._device, "hw:5,0")


class TestDeviceDiscovery(unittest.TestCase):
    """Device discovery tests"""

    def test_discover_at2020(self):
        """Correctly parses arecord -l output containing AT2020"""
        mock_output = (
            "**** List of CAPTURE Hardware Devices ****\n"
            "card 0: tegrahda [tegra-hda], device 3: HDMI 0 [HDMI 0]\n"
            "card 2: XPLUS [AT2020USB-XPLUS], device 0: USB Audio [USB Audio]\n"
        )

        async def _test():
            cap = AudioCapture()
            mock_proc = MagicMock()
            mock_proc.communicate = asyncio.coroutine(
                lambda: (mock_output.encode("utf-8"), b"")
            )
            with patch("asyncio.create_subprocess_exec", return_value=mock_proc):
                device = await cap._discover_device()
            self.assertEqual(device, "hw:2,0")

        _run(_test())

    def test_discover_no_device_raises(self):
        """Raises RuntimeError when AT2020 is not found"""
        mock_output = (
            "**** List of CAPTURE Hardware Devices ****\n"
            "card 0: tegrahda [tegra-hda], device 3: HDMI 0 [HDMI 0]\n"
        )

        async def _test():
            cap = AudioCapture()
            mock_proc = MagicMock()
            mock_proc.communicate = asyncio.coroutine(
                lambda: (mock_output.encode("utf-8"), b"")
            )
            with patch("asyncio.create_subprocess_exec", return_value=mock_proc):
                with self.assertRaises(RuntimeError):
                    await cap._discover_device()

        _run(_test())


class TestFrameBuffer(unittest.TestCase):
    """Frame buffer (residual accumulator) tests"""

    def test_exact_frame_sends(self):
        """Data exactly equal to FRAME_BYTES sends 1 frame"""
        sent_frames = []

        async def _test():
            cap = AudioCapture(mock=True)
            cap._running = True
            mock_writer = MagicMock()
            mock_writer.write = MagicMock()
            mock_writer.drain = asyncio.coroutine(lambda: None)
            cap._writer = mock_writer

            data = b"\x01" * FRAME_BYTES
            await cap._send_frames(data)
            self.assertEqual(mock_writer.write.call_count, 1)
            self.assertEqual(len(mock_writer.write.call_args[0][0]), FRAME_BYTES)
            self.assertEqual(len(cap._frame_buffer), 0)

        _run(_test())

    def test_partial_frame_buffered(self):
        """Data less than FRAME_BYTES accumulates in buffer"""
        async def _test():
            cap = AudioCapture(mock=True)
            cap._running = True
            mock_writer = MagicMock()
            mock_writer.write = MagicMock()
            mock_writer.drain = asyncio.coroutine(lambda: None)
            cap._writer = mock_writer

            data = b"\x01" * (FRAME_BYTES - 100)
            await cap._send_frames(data)
            self.assertEqual(mock_writer.write.call_count, 0)
            self.assertEqual(len(cap._frame_buffer), FRAME_BYTES - 100)

        _run(_test())

    def test_multiple_frames_plus_residual(self):
        """2.5 frames of data -> 2 frames sent + 0.5 frame residual"""
        async def _test():
            cap = AudioCapture(mock=True)
            cap._running = True
            mock_writer = MagicMock()
            mock_writer.write = MagicMock()
            mock_writer.drain = asyncio.coroutine(lambda: None)
            cap._writer = mock_writer

            data = b"\x02" * int(FRAME_BYTES * 2.5)
            await cap._send_frames(data)
            self.assertEqual(mock_writer.write.call_count, 2)
            expected_residual = int(FRAME_BYTES * 2.5) - FRAME_BYTES * 2
            self.assertEqual(len(cap._frame_buffer), expected_residual)

        _run(_test())


class TestShutdown(unittest.TestCase):
    """Shutdown tests"""

    def test_shutdown_sets_running_false(self):
        async def _test():
            cap = AudioCapture()
            cap._running = True
            await cap.shutdown()
            self.assertFalse(cap._running)

        _run(_test())

    def test_shutdown_closes_writer(self):
        async def _test():
            cap = AudioCapture()
            cap._running = True
            mock_writer = MagicMock()
            cap._writer = mock_writer
            await cap.shutdown()
            mock_writer.close.assert_called_once()
            self.assertIsNone(cap._writer)

        _run(_test())


if __name__ == "__main__":
    unittest.main()
