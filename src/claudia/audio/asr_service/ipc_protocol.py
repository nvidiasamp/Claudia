#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IPC Protocol Layer: Message Factories + Unix Domain Socket Utilities

3-way unidirectional socket architecture (under $XDG_RUNTIME_DIR/claudia/):
  - audio.sock  (AudioCapture -> ASR, raw PCM)
  - result.sock (ASR -> Main, JSON Lines)
  - ctrl.sock   (Main -> ASR, JSON Lines)

Protocol version: major-compatible (1.0 and 1.1 interoperable, 2.0 incompatible)
Message format: JSON Lines (one JSON per line, \n separated)
"""

import asyncio
import json
import logging
import os
import stat
import time
from typing import Any, AsyncIterator, Callable, Optional, Tuple

logger = logging.getLogger(__name__)

# === Protocol version ===
PROTO_VERSION = "1.0"

# === Socket paths ===
# Security: use $XDG_RUNTIME_DIR (typically /run/user/<uid>, 0700) instead of /tmp.
# Blocks access from other users at the OS level.
# Falls back to /tmp/claudia_ipc_<uid> if $XDG_RUNTIME_DIR is unset/unwritable.
# UID suffix prevents directory collisions in multi-user environments.
def _socket_dir():
    # type: () -> str
    candidates = []

    # Preferred: $XDG_RUNTIME_DIR/claudia (OS-level 0700 guarantee)
    runtime = os.environ.get("XDG_RUNTIME_DIR")
    if runtime and os.path.isdir(runtime):
        candidates.append(os.path.join(runtime, "claudia"))

    # Fallback: /tmp/claudia_ipc_<uid> (UID isolation)
    candidates.append("/tmp/claudia_ipc_{}".format(os.getuid()))

    for d in candidates:
        try:
            os.makedirs(d, mode=0o700, exist_ok=True)
            # Symlink rejection: os.stat() follows symlinks, so an attacker could
            # set up /tmp/claudia_ipc_<uid> -> /target to bypass the owner check.
            # Use lstat to inspect the symlink itself and reject it.
            dir_lstat = os.lstat(d)
            if stat.S_ISLNK(dir_lstat.st_mode):
                logger.warning("Socket directory %s is a symlink, rejecting", d)
                continue
            if dir_lstat.st_uid != os.getuid():
                logger.warning("Socket directory %s is owned by another user, skipping", d)
                continue
            if stat.S_IMODE(dir_lstat.st_mode) != 0o700:
                os.chmod(d, 0o700)
            return d
        except OSError as e:
            logger.warning("Socket directory creation failed: %s: %s", d, e)
            continue

    # All candidates failed -- raise exception as last resort
    raise RuntimeError(
        "Cannot create IPC socket directory: {}".format(candidates)
    )

_SOCK_DIR = _socket_dir()
AUDIO_SOCKET = os.path.join(_SOCK_DIR, "audio.sock")
ASR_RESULT_SOCKET = os.path.join(_SOCK_DIR, "result.sock")
ASR_CTRL_SOCKET = os.path.join(_SOCK_DIR, "ctrl.sock")
SESSION_TOKEN_FILE = os.path.join(_SOCK_DIR, ".ctrl_token")

# === Message types (Result: ASR -> Main) ===
MSG_READY = "ready"
MSG_HEARTBEAT = "heartbeat"
MSG_TRANSCRIPT = "transcript"
MSG_EMERGENCY = "emergency"
MSG_VAD_START = "vad_start"
MSG_VAD_END = "vad_end"
MSG_ERROR = "error"
MSG_GATE_TIMEOUT_AUDIT = "gate_timeout_audit"

# === Control types (Control: Main -> ASR) ===
CTRL_TTS_START = "tts_start"
CTRL_TTS_END = "tts_end"
CTRL_SHUTDOWN = "shutdown"


# === ctrl socket session authentication ===
# Protection level: prevention of accidental interference (leftover scripts, debug tools, etc.).
# A deliberate attacker with the same UID can read the token file (0o600), so this method
# cannot prevent such attacks (SO_PEERCRED + PID whitelist would be needed).
# Accepted risk in a Jetson single-user embedded environment.
#
# Mechanism: ASR server generates a 16-byte nonce at startup and writes it to the token file.
# ctrl clients read the token and include it in messages. Token mismatches are rejected.


def generate_session_token():
    # type: () -> str
    """Generate a session token for ctrl socket authentication (16-byte hex)"""
    return os.urandom(16).hex()


def create_ctrl_message(msg_type, token, **kwargs):
    # type: (str, str, **Any) -> dict
    """Generate a ctrl message with authentication token"""
    msg = {"type": msg_type, "token": token}
    msg.update(kwargs)
    return msg


def validate_session_token(msg, expected_token):
    # type: (dict, str) -> bool
    """Validate the session token in a ctrl message"""
    return msg.get("token") == expected_token


def read_session_token():
    # type: () -> Optional[str]
    """Read session token from the token file"""
    try:
        with open(SESSION_TOKEN_FILE, "r") as f:
            return f.read().strip()
    except (OSError, IOError):
        return None


# ============================================================
# Message factories (pure functions, no side effects)
# ============================================================

def create_handshake(model: str, vram_mb: int) -> dict:
    """Ready message sent at ASR service startup"""
    return {
        "type": MSG_READY,
        "model": model,
        "vram_mb": vram_mb,
        "proto_version": PROTO_VERSION,
    }


def create_transcript_msg(
    text: str,
    confidence: float,
    duration_ms: int,
    asr_latency_ms: int,
    utterance_id: str,
) -> dict:
    """ASR transcription result message"""
    return {
        "type": MSG_TRANSCRIPT,
        "text": text,
        "confidence": confidence,
        "duration_ms": duration_ms,
        "asr_latency_ms": asr_latency_ms,
        "utterance_id": utterance_id,
    }


def create_emergency_msg(
    keyword: str,
    confidence: float,
    utterance_id: str,
) -> dict:
    """Emergency stop detection message"""
    return {
        "type": MSG_EMERGENCY,
        "keyword": keyword,
        "confidence": confidence,
        "utterance_id": utterance_id,
    }


def create_heartbeat() -> dict:
    """Heartbeat message (for watchdog)"""
    return {
        "type": MSG_HEARTBEAT,
        "ts": time.time(),
    }


def create_vad_event(event_type: str, **kwargs) -> dict:
    """VAD event message (vad_start / vad_end)

    Args:
        event_type: "vad_start" or "vad_end"
        **kwargs: for vad_end, includes duration_ms
    """
    msg: dict = {"type": event_type}
    if event_type == MSG_VAD_END and "duration_ms" in kwargs:
        msg["duration_ms"] = kwargs["duration_ms"]
    return msg


def create_error_msg(msg: str) -> dict:
    """Error message"""
    return {
        "type": MSG_ERROR,
        "msg": msg,
    }


def create_gate_timeout_audit() -> dict:
    """TTS gate timeout audit message (30s protection)"""
    return {
        "type": MSG_GATE_TIMEOUT_AUDIT,
        "ts": time.time(),
    }


# ============================================================
# Protocol validation
# ============================================================

def validate_proto_version(version: str) -> bool:
    """Protocol version compatibility check

    Compatible if major versions match.
    Example: "1.0" and "1.1" -> True, "1.0" and "2.0" -> False
    """
    try:
        remote_major = version.split(".")[0]
        local_major = PROTO_VERSION.split(".")[0]
        return remote_major == local_major
    except (AttributeError, IndexError):
        return False


# ============================================================
# JSON Lines encode/decode
# ============================================================

def encode_message(msg: dict) -> bytes:
    """dict -> JSON bytes + newline (JSON Lines format)"""
    return json.dumps(msg, ensure_ascii=False).encode("utf-8") + b"\n"


def decode_message(line: bytes) -> dict:
    """JSON bytes line -> dict

    Raises:
        json.JSONDecodeError: Invalid JSON
        UnicodeDecodeError: Invalid encoding
    """
    return json.loads(line.decode("utf-8").strip())


# ============================================================
# Socket Utilities (asyncio-based)
# ============================================================

def cleanup_socket(path: str) -> None:
    """Delete stale socket files"""
    try:
        if os.path.exists(path):
            os.unlink(path)
            logger.debug("Deleted old socket file: %s", path)
    except OSError as e:
        logger.warning("Failed to delete socket file: %s: %s", path, e)


async def create_uds_server(
    path: str,
    handler: Callable,
) -> asyncio.AbstractServer:
    """Create a Unix Domain Socket server

    Deletes any existing socket file first.

    Args:
        path: Socket file path
        handler: client_connected_cb for asyncio.start_unix_server
                 Callback receiving (reader, writer)

    Returns:
        asyncio.Server instance
    """
    cleanup_socket(path)
    server = await asyncio.start_unix_server(handler, path=path)
    # Restrict socket file permissions to owner-only
    # (prevents unauthorized connections from other users on the same machine)
    try:
        os.chmod(path, stat.S_IRUSR | stat.S_IWUSR)  # 0o600
    except OSError as e:
        logger.warning("Failed to set socket permissions: %s: %s", path, e)
    logger.info("UDS server started: %s", path)
    return server


async def connect_uds(
    path: str,
    retries: int = 5,
    delay: float = 0.5,
) -> Tuple[asyncio.StreamReader, asyncio.StreamWriter]:
    """Connect to a Unix Domain Socket (with retries)

    Args:
        path: Socket file path
        retries: Maximum number of retry attempts
        delay: Retry interval in seconds (exponential backoff)

    Returns:
        (StreamReader, StreamWriter) tuple

    Raises:
        ConnectionError: All retries failed
    """
    last_error: Optional[Exception] = None
    current_delay = delay

    for attempt in range(1, retries + 1):
        try:
            reader, writer = await asyncio.open_unix_connection(path)
            logger.debug("UDS connection successful: %s (attempt %d)", path, attempt)
            return reader, writer
        except (FileNotFoundError, ConnectionRefusedError, OSError) as e:
            last_error = e
            if attempt < retries:
                logger.debug(
                    "UDS connection failed (attempt %d/%d): %s: %s, retrying in %.1fs",
                    attempt, retries, path, e, current_delay,
                )
                await asyncio.sleep(current_delay)
                current_delay = min(current_delay * 2, 10.0)

    raise ConnectionError(
        f"UDS connection failed (after {retries} retries): {path}: {last_error}"
    )


async def read_json_lines(
    reader: asyncio.StreamReader,
) -> AsyncIterator[dict]:
    """Asynchronously iterate JSON Lines from a StreamReader

    Terminates on connection disconnect or empty line. Invalid JSON lines are skipped with logging.

    Yields:
        Parsed dict
    """
    while True:
        try:
            line = await reader.readline()
        except (ConnectionResetError, BrokenPipeError):
            logger.warning("Connection disconnected while reading JSON Lines")
            break

        if not line:
            # EOF
            break

        line = line.strip()
        if not line:
            continue

        try:
            yield decode_message(line)
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            logger.warning("Skipping invalid JSON line: %s (error: %s)", line[:100], e)


async def write_json_line(
    writer: asyncio.StreamWriter,
    msg: dict,
) -> None:
    """Write a single JSON message to a StreamWriter

    Logs and propagates exceptions on write failure.
    """
    data = encode_message(msg)
    writer.write(data)
    await writer.drain()
