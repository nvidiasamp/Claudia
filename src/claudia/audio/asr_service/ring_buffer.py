#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ring Audio Buffer -- stores the most recent 30 seconds of 16kHz/16bit/mono PCM data.

Thread-safe: uses threading.Lock to protect all read/write operations.
Can be safely called from asyncio contexts via run_in_executor.
"""

import threading
from typing import Optional

# 16kHz, 16-bit mono: 32 bytes per millisecond
BYTES_PER_MS = 16000 * 2 // 1000  # = 32
DEFAULT_DURATION_S = 30
DEFAULT_BUFFER_SIZE = DEFAULT_DURATION_S * 16000 * 2  # 960,000 bytes


class RingBuffer:
    """Fixed-size ring buffer for continuous PCM audio streams.

    Parameters
    ----------
    capacity : int
        Buffer capacity in bytes. Default 960,000 (30 seconds @16kHz/16bit/mono).
    pre_speech_buffer_ms : int
        Pre-speech buffer in milliseconds. read_last / read_range always guarantee
        this interval is available.
    """

    def __init__(
        self,
        capacity: int = DEFAULT_BUFFER_SIZE,
        pre_speech_buffer_ms: int = 300,
    ) -> None:
        self._capacity = capacity
        self._buf = bytearray(capacity)
        self._write_pos = 0       # Next write position (circular)
        self._total_written = 0   # Cumulative bytes written (non-wrapping)
        self._lock = threading.Lock()
        self._pre_speech_bytes = pre_speech_buffer_ms * BYTES_PER_MS

    # ------------------------------------------------------------------
    # Write
    # ------------------------------------------------------------------

    def write(self, data: bytes) -> None:
        """Append PCM data to the buffer. Overwrites oldest data when capacity is exceeded."""
        if not data:
            return

        n = len(data)
        with self._lock:
            if n >= self._capacity:
                # Data size >= entire buffer: keep only the last capacity bytes
                tail = data[-self._capacity:]
                self._buf[:] = tail
                self._write_pos = 0
                self._total_written += n
                return

            end = self._write_pos + n
            if end <= self._capacity:
                self._buf[self._write_pos:end] = data
            else:
                # Wrap-around needed
                first_part = self._capacity - self._write_pos
                self._buf[self._write_pos:] = data[:first_part]
                self._buf[:n - first_part] = data[first_part:]

            self._write_pos = end % self._capacity
            self._total_written += n

    # ------------------------------------------------------------------
    # Read
    # ------------------------------------------------------------------

    def read_last(self, ms: int) -> bytes:
        """Read the most recent *ms* milliseconds of audio data.

        Returns the actually available portion if the buffer has insufficient data.
        """
        request_bytes = ms * BYTES_PER_MS
        with self._lock:
            available = min(self._total_written, self._capacity)
            nbytes = min(request_bytes, available)
            if nbytes == 0:
                return b""
            return self._read_tail(nbytes)

    def read_range(self, start_ms: int, end_ms: int) -> bytes:
        """Read audio from the [start_ms, end_ms) interval measured back from the current write position.

        start_ms / end_ms are both "millisecond offsets from the current moment", i.e., 0 = now.
        Example: read_range(500, 200) reads data from 500ms ago to 200ms ago.

        Automatically truncates if the requested range exceeds available data.
        """
        if start_ms <= end_ms:
            return b""

        start_bytes = start_ms * BYTES_PER_MS
        end_bytes = end_ms * BYTES_PER_MS
        range_bytes = start_bytes - end_bytes

        with self._lock:
            available = min(self._total_written, self._capacity)
            # Truncate to available range
            actual_start = min(start_bytes, available)
            actual_end = min(end_bytes, available)
            if actual_start <= actual_end:
                return b""

            nbytes = actual_start - actual_end
            # Read nbytes starting from actual_start bytes before write_pos
            read_start = (self._write_pos - actual_start) % self._capacity
            return self._read_from(read_start, nbytes)

    # ------------------------------------------------------------------
    # Utility methods
    # ------------------------------------------------------------------

    def clear(self) -> None:
        """Clear the buffer."""
        with self._lock:
            self._write_pos = 0
            self._total_written = 0
            # No need to zero out the bytearray; write_pos/total_written control the valid range

    @property
    def available_ms(self) -> int:
        """Number of milliseconds of audio currently available in the buffer."""
        with self._lock:
            available = min(self._total_written, self._capacity)
        return available // BYTES_PER_MS

    @property
    def capacity_ms(self) -> int:
        """Total buffer capacity in milliseconds."""
        return self._capacity // BYTES_PER_MS

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _read_tail(self, nbytes: int) -> bytes:
        """Read nbytes from the current write position backwards. Caller must hold _lock."""
        start = (self._write_pos - nbytes) % self._capacity
        return self._read_from(start, nbytes)

    def _read_from(self, start: int, nbytes: int) -> bytes:
        """Read nbytes from the given start position. Caller must hold _lock."""
        end = start + nbytes
        if end <= self._capacity:
            return bytes(self._buf[start:end])
        else:
            first_part = self._capacity - start
            return bytes(self._buf[start:]) + bytes(self._buf[:nbytes - first_part])
