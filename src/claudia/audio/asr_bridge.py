#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ASR Bridge -- Consumes ASR server results and bridges to ProductionBrain

Reads JSON Lines from result.sock, branching by message type:
  - emergency -> Immediate brain call (queue bypass) + queue flush + cooldown
  - transcript -> Confidence filter + deduplication -> command queue
  - heartbeat -> Watchdog update
  - ready -> Protocol version verification + ready signal

Command queue (bounded, maxsize=3) -> Worker serially consumes -> brain.process_and_execute
"""

import asyncio
import json
import logging
import os
import time
from datetime import datetime
from typing import Any, Dict, Optional, Tuple

from .asr_service.ipc_protocol import (
    ASR_RESULT_SOCKET,
    PROTO_VERSION,
    connect_uds,
    read_json_lines,
    validate_proto_version,
)

logger = logging.getLogger("claudia.asr.bridge")

# Configuration constants
QUEUE_MAXSIZE = 3
MIN_CONFIDENCE = 0.35
DEDUP_TTL_S = 10.0
HEARTBEAT_TIMEOUT_S = 15.0
EMERGENCY_COOLDOWN_S = 0.5
RECONNECT_DELAY_S = 2.0

# E2E timing log
_E2E_LOG_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "..", "logs",
)
_E2E_LOG_PATH = os.path.join(_E2E_LOG_DIR, "e2e_timing.jsonl")


class ASRBridge:
    """ASR result consumer + command queue management

    Args:
        brain: ProductionBrain instance (calls process_and_execute)
        socket_path: Result socket path
        on_result: Result callback (for debug display, Optional)
    """

    def __init__(
        self,
        brain: Any,
        socket_path: str = ASR_RESULT_SOCKET,
        on_result: Any = None,
        on_wake: Any = None,
    ) -> None:
        self._brain = brain
        self._socket_path = socket_path
        self._on_result = on_result

        # Wake word gate
        from .wake_word import WakeWordGate
        self._wake_gate = WakeWordGate(
            on_wake=self._make_wake_callback(on_wake),
        )

        # Ready event (set upon receiving ready message)
        self.ready_event = asyncio.Event()

        # Command queue (bounded, drop-oldest)
        self._queue: asyncio.Queue = asyncio.Queue(maxsize=QUEUE_MAXSIZE)

        # Deduplication: utterance_id -> processing timestamp
        self._processed_ids: Dict[str, float] = {}

        # Heartbeat watchdog
        self._last_heartbeat: float = time.monotonic()

        # Emergency cooldown window
        self._cooldown_until: float = 0.0

        # Task references
        self._consumer_task: Optional[asyncio.Task] = None  # type: ignore[type-arg]
        self._worker_task: Optional[asyncio.Task] = None  # type: ignore[type-arg]
        self._watchdog_task: Optional[asyncio.Task] = None  # type: ignore[type-arg]

        self._running = False

    def _make_wake_callback(self, external_on_wake):
        # type: (Any) -> Any
        """Generate wake word callback: external callback + "wake" event notification"""
        def _cb():
            if external_on_wake:
                try:
                    external_on_wake()
                except Exception:
                    pass
            if self._on_result:
                try:
                    self._on_result("wake", "", 0)
                except Exception:
                    pass
        return _cb

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    async def start(self) -> None:
        """Start bridge: launch result consumer + command worker

        Watchdog starts after ready handshake (prevents false detection during ASR model loading).
        """
        self._running = True
        self._last_heartbeat = time.monotonic()

        self._consumer_task = asyncio.ensure_future(self._result_consumer_loop())
        self._worker_task = asyncio.ensure_future(self._command_worker())
        # Watchdog starts inside _handle_ready()

        logger.info("ASRBridge started")

    async def stop(self) -> None:
        """Stop bridge"""
        logger.info("ASRBridge stopping...")
        self._running = False

        for task in (self._consumer_task, self._worker_task, self._watchdog_task):
            if task and not task.done():
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass

        self._consumer_task = None
        self._worker_task = None
        self._watchdog_task = None

    # ------------------------------------------------------------------
    # Result consumer loop (auto-reconnect)
    # ------------------------------------------------------------------

    async def _result_consumer_loop(self) -> None:
        """Outer loop: disconnect -> retry"""
        while self._running:
            try:
                reader, writer = await connect_uds(
                    self._socket_path, retries=10, delay=1.0,
                )
                logger.info("result.sock connection established")

                try:
                    await self._consume_results(reader)
                finally:
                    try:
                        writer.close()
                    except Exception:
                        pass

            except asyncio.CancelledError:
                break
            except ConnectionError as e:
                if not self._running:
                    break
                logger.warning("result.sock connection failed: %s -- retrying in %.1fs",
                               e, RECONNECT_DELAY_S)
                await asyncio.sleep(RECONNECT_DELAY_S)
            except Exception as e:
                if not self._running:
                    break
                logger.error("Result consumer error: %s", e, exc_info=True)
                await asyncio.sleep(RECONNECT_DELAY_S)

    async def _consume_results(self, reader: asyncio.StreamReader) -> None:
        """Read JSON Lines -> message dispatch"""
        async for msg in read_json_lines(reader):
            if not self._running:
                break
            msg_type = msg.get("type", "")
            try:
                if msg_type == "ready":
                    await self._handle_ready(msg)
                elif msg_type == "transcript":
                    await self._handle_transcript(msg)
                elif msg_type == "emergency":
                    await self._handle_emergency(msg)
                elif msg_type == "heartbeat":
                    self._handle_heartbeat(msg)
                elif msg_type == "vad_start":
                    if self._on_result:
                        try:
                            self._on_result("vad_start", "", 0)
                        except Exception:
                            pass
                elif msg_type in ("vad_end", "error", "gate_timeout_audit"):
                    logger.debug("ASR event: %s", msg_type)
                else:
                    logger.warning("Unknown message type: %s", msg_type)
            except Exception as e:
                logger.error("Message processing error (%s): %s", msg_type, e)

    # ------------------------------------------------------------------
    # Message handlers
    # ------------------------------------------------------------------

    async def _handle_ready(self, msg: dict) -> None:
        """Ready handshake: protocol version verification + watchdog start"""
        version = msg.get("proto_version", "")
        model = msg.get("model", "unknown")

        if not validate_proto_version(version):
            logger.error(
                "Protocol version mismatch: remote=%s, local=%s",
                version, PROTO_VERSION,
            )
            return

        logger.info("ASR ready: model=%s, version=%s", model, version)

        # Start heartbeat watchdog after receiving ready
        # (prevents false detection during ASR model loading)
        self._last_heartbeat = time.monotonic()
        if self._watchdog_task is None or self._watchdog_task.done():
            self._watchdog_task = asyncio.ensure_future(self._heartbeat_watchdog())

        self.ready_event.set()

    async def _handle_transcript(self, msg: dict) -> None:
        """Transcript processing: emergency fallback recheck + confidence filter + deduplication + queue insertion"""
        text = msg.get("text", "").strip()
        confidence = msg.get("confidence", 0.0)
        utterance_id = msg.get("utterance_id", "")

        if not text:
            return

        # Fallback: check emergency keywords even in transcript
        try:
            from .asr_service.emergency_keywords import match_emergency
            emg = match_emergency(text)
            if emg is not None:
                logger.warning("Emergency detected via transcript: '%s'", text)
                await self._handle_emergency_action(utterance_id, text)
                return
        except ImportError:
            pass  # Skip if emergency_keywords unavailable

        # Confidence filter
        if confidence < MIN_CONFIDENCE:
            logger.info("Low confidence skip: '%s' (conf=%.2f < %.2f)", text, confidence, MIN_CONFIDENCE)
            return

        # Wake word gate (emergency handled above -- won't reach here)
        filtered_text = self._wake_gate.filter(text, confidence)
        if filtered_text is None:
            return
        text = filtered_text

        # Deduplication (skip if emergency already processed same ID)
        if self._is_processed(utterance_id):
            logger.debug("Deduplication: utt=%s", utterance_id)
            return

        # Enqueue (mark_processed done at worker execution time)
        asr_latency_ms = msg.get("asr_latency_ms", 0)
        await self._enqueue_command(text, utterance_id, asr_latency_ms)

        if self._on_result:
            try:
                self._on_result("transcript", text, confidence)
            except Exception:
                pass

    async def _handle_emergency(self, msg: dict) -> None:
        """Emergency immediate processing"""
        keyword = msg.get("keyword", "")
        utterance_id = msg.get("utterance_id", "")

        logger.warning("Emergency received: keyword='%s', utt=%s", keyword, utterance_id)

        await self._handle_emergency_action(utterance_id, keyword)

        if self._on_result:
            try:
                self._on_result("emergency", keyword, 1.0)
            except Exception:
                pass

    async def _handle_emergency_action(self, utterance_id: str, keyword: str) -> None:
        """Emergency common processing: queue flush -> cooldown setup -> brain call

        Order matters: queue flush and cooldown before brain call to prevent
        the worker from executing old commands during brain's await.
        (emergency does not acquire _command_lock, so it runs in parallel with worker)
        """
        # Deduplication check
        if self._is_processed(utterance_id):
            logger.debug("Emergency deduplication: utt=%s", utterance_id)
            return

        self._mark_processed(utterance_id)

        # Flush queue + set cooldown first (prevent worker execution during brain await)
        flushed = 0
        while not self._queue.empty():
            try:
                self._queue.get_nowait()
                flushed += 1
            except asyncio.QueueEmpty:
                break
        if flushed:
            logger.info("Emergency queue flush: %d items discarded", flushed)

        self._cooldown_until = time.monotonic() + EMERGENCY_COOLDOWN_S

        # Reset wake word gate (close listening window during emergency)
        self._wake_gate.reset()

        # Brain call (queue bypass) + E2E measurement
        logger.warning("Emergency execution: '%s'", keyword)
        brain_start = time.monotonic()
        try:
            result = await self._brain.process_and_execute("止まれ")
            brain_ms = (time.monotonic() - brain_start) * 1000

            self._write_e2e_log(
                command=keyword,
                route="emergency",
                asr_ms=0,  # Emergency is keyword detection, no ASR full inference
                queue_ms=0,
                brain_ms=brain_ms,
                e2e_ms=brain_ms,
                api_code=result.api_code if result else None,
                execution_status=result.execution_status or "" if result else "",
                utterance_id=utterance_id,
            )

            logger.info("Emergency result: %s (brain=%.0fms)", result, brain_ms)
        except Exception as e:
            brain_ms = (time.monotonic() - brain_start) * 1000
            logger.error("Emergency brain call failed: %s (brain=%.0fms)", e, brain_ms)

    def _handle_heartbeat(self, msg: dict) -> None:
        """Heartbeat update"""
        self._last_heartbeat = time.monotonic()

    # ------------------------------------------------------------------
    # Deduplication (utterance_id)
    # ------------------------------------------------------------------

    def _is_processed(self, utterance_id: str) -> bool:
        """Check if utterance_id has been processed (empty string always treated as unprocessed)"""
        if not utterance_id:
            return False
        self._cleanup_expired_ids()
        return utterance_id in self._processed_ids

    def _mark_processed(self, utterance_id: str) -> None:
        """Mark utterance_id as processed (+ periodic cleanup)"""
        if utterance_id:
            self._processed_ids[utterance_id] = time.monotonic()
            # Prevent memory accumulation even on paths where _is_processed is not called
            if len(self._processed_ids) > 100:
                self._cleanup_expired_ids()

    def _cleanup_expired_ids(self) -> None:
        """Remove utterance_ids that have exceeded TTL"""
        now = time.monotonic()
        expired = [
            uid for uid, ts in self._processed_ids.items()
            if now - ts > DEDUP_TTL_S
        ]
        for uid in expired:
            del self._processed_ids[uid]

    # ------------------------------------------------------------------
    # Command queue
    # ------------------------------------------------------------------

    async def _enqueue_command(
        self, text: str, utterance_id: str, asr_latency_ms: int = 0,
    ) -> None:
        """Enqueue command to bounded queue (discard oldest when full)"""
        if self._queue.full():
            try:
                discarded = self._queue.get_nowait()
                logger.warning("Queue full: oldest command discarded '%s'", discarded[0])
            except asyncio.QueueEmpty:
                pass
        enqueue_ts = time.monotonic()
        await self._queue.put((text, utterance_id, asr_latency_ms, enqueue_ts))
        logger.debug("Command enqueued: '%s' (size=%d)", text, self._queue.qsize())

    async def _command_worker(self) -> None:
        """Command worker: serially consume from queue -> brain.process_and_execute + E2E measurement"""
        while self._running:
            try:
                item = await asyncio.wait_for(
                    self._queue.get(), timeout=1.0,
                )
            except asyncio.TimeoutError:
                continue
            except asyncio.CancelledError:
                break

            text, utterance_id, asr_latency_ms, enqueue_ts = item

            # Cooldown window check
            now = time.monotonic()
            if now < self._cooldown_until:
                remaining = self._cooldown_until - now
                logger.debug("In cooldown: %.0fms remaining, skipping '%s'",
                             remaining * 1000, text)
                continue

            # Deduplication (in case same ID was processed between enqueue and execution)
            if self._is_processed(utterance_id):
                logger.debug("Worker deduplication: utt=%s", utterance_id)
                continue

            self._mark_processed(utterance_id)

            # Queue wait time
            queue_wait_ms = (now - enqueue_ts) * 1000

            # Brain call + E2E measurement
            logger.info("Command execution: '%s'", text)
            brain_start = time.monotonic()
            try:
                result = await self._brain.process_and_execute(text)
                brain_ms = (time.monotonic() - brain_start) * 1000

                # E2E = ASR inference + queue wait + Brain processing+execution
                e2e_ms = asr_latency_ms + queue_wait_ms + brain_ms

                # E2E timing log output
                self._write_e2e_log(
                    command=text,
                    route=result.reasoning or "",
                    asr_ms=asr_latency_ms,
                    queue_ms=queue_wait_ms,
                    brain_ms=brain_ms,
                    e2e_ms=e2e_ms,
                    api_code=result.api_code,
                    execution_status=result.execution_status or "",
                    utterance_id=utterance_id,
                )

                logger.info(
                    "E2E measurement: '%s' -> e2e=%.0fms (asr=%dms + queue=%.0fms + brain=%.0fms)",
                    text, e2e_ms, asr_latency_ms, queue_wait_ms, brain_ms,
                )

                # Result display (with E2E data)
                if self._on_result:
                    try:
                        self._on_result("result", text, result)
                        self._on_result("e2e_timing", text, {
                            "asr_ms": asr_latency_ms,
                            "brain_ms": brain_ms,
                            "e2e_ms": e2e_ms,
                        })
                    except Exception:
                        pass

                logger.info("Command result: response='%s', api=%s, status=%s",
                            result.response[:50] if result.response else "",
                            result.api_code,
                            result.execution_status)

            except Exception as e:
                brain_ms = (time.monotonic() - brain_start) * 1000
                logger.error("Command execution failed: '%s': %s (brain=%.0fms)", text, e, brain_ms)

    # ------------------------------------------------------------------
    # Heartbeat watchdog
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # E2E timing log
    # ------------------------------------------------------------------

    def _write_e2e_log(
        self,
        command,        # type: str
        route,          # type: str
        asr_ms,         # type: float
        queue_ms,       # type: float
        brain_ms,       # type: float
        e2e_ms,         # type: float
        api_code,       # type: Optional[int]
        execution_status,  # type: str
        utterance_id,   # type: str
    ):
        # type: (...) -> None
        """Record E2E timing to JSONL file"""
        try:
            entry = {
                "ts": datetime.now().isoformat(),
                "command": command,
                "route": route,
                "asr_ms": round(asr_ms, 1),
                "queue_ms": round(queue_ms, 1),
                "brain_ms": round(brain_ms, 1),
                "e2e_ms": round(e2e_ms, 1),
                "api_code": api_code,
                "execution_status": execution_status,
                "utterance_id": utterance_id,
            }
            log_path = os.path.normpath(_E2E_LOG_PATH)
            log_dir = os.path.dirname(log_path)
            if not os.path.isdir(log_dir):
                os.makedirs(log_dir, exist_ok=True)
            with open(log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(entry, ensure_ascii=False) + "\n")
        except Exception as e:
            logger.debug("E2E log write failed: %s", e)

    # ------------------------------------------------------------------
    # Heartbeat watchdog
    # ------------------------------------------------------------------

    async def _heartbeat_watchdog(self) -> None:
        """Heartbeat monitor: prompts reconnection on timeout"""
        while self._running:
            try:
                await asyncio.sleep(5.0)
            except asyncio.CancelledError:
                break

            elapsed = time.monotonic() - self._last_heartbeat
            if elapsed > HEARTBEAT_TIMEOUT_S:
                logger.warning(
                    "Heartbeat timeout: %.0fs elapsed (threshold=%ds)",
                    elapsed, HEARTBEAT_TIMEOUT_S,
                )
                # Cancel consumer_task -> enters reconnection loop
                if self._consumer_task and not self._consumer_task.done():
                    self._consumer_task.cancel()
                    # The outer while in consumer_loop handles reconnection
                    self._consumer_task = asyncio.ensure_future(
                        self._result_consumer_loop()
                    )
                self._last_heartbeat = time.monotonic()  # Reset
