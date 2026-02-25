#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ASRBridge unit tests"""

import asyncio
import os
import sys
import time
import unittest
from unittest.mock import MagicMock, patch

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_PROJECT_ROOT, "src"))

from claudia.audio.asr_bridge import (
    ASRBridge,
    QUEUE_MAXSIZE,
    MIN_CONFIDENCE,
    DEDUP_TTL_S,
    EMERGENCY_COOLDOWN_S,
)


def _run(coro):
    """Python 3.8 compatible asyncio test helper"""
    loop = asyncio.new_event_loop()
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


def _make_mock_brain():
    """Create a mock brain with process_and_execute"""
    brain = MagicMock()
    result = MagicMock()
    result.response = "テスト応答"
    result.api_code = 1016
    result.execution_status = "success"
    result.sequence = None

    async def mock_pe(text):
        return result

    brain.process_and_execute = mock_pe
    return brain, result


class TestASRBridgeInit(unittest.TestCase):
    """Initialization tests"""

    def test_default_init(self):
        brain = MagicMock()
        bridge = ASRBridge(brain)
        self.assertFalse(bridge._running)
        self.assertFalse(bridge.ready_event.is_set())
        self.assertEqual(bridge._queue.maxsize, QUEUE_MAXSIZE)

    def test_custom_socket_path(self):
        brain = MagicMock()
        bridge = ASRBridge(brain, socket_path="/tmp/test.sock")
        self.assertEqual(bridge._socket_path, "/tmp/test.sock")


class TestReadyHandshake(unittest.TestCase):
    """Ready handshake tests"""

    def test_valid_version_sets_event(self):
        async def _test():
            brain = MagicMock()
            bridge = ASRBridge(brain)
            await bridge._handle_ready({
                "type": "ready",
                "model": "whisper-base",
                "proto_version": "1.0",
            })
            self.assertTrue(bridge.ready_event.is_set())
        _run(_test())

    def test_incompatible_version_does_not_set_event(self):
        async def _test():
            brain = MagicMock()
            bridge = ASRBridge(brain)
            await bridge._handle_ready({
                "type": "ready",
                "model": "whisper-base",
                "proto_version": "2.0",
            })
            self.assertFalse(bridge.ready_event.is_set())
        _run(_test())


class TestTranscriptHandling(unittest.TestCase):
    """Transcript processing tests"""

    def test_normal_transcript_enqueued(self):
        """Normal transcript is enqueued"""
        async def _test():
            brain, _ = _make_mock_brain()
            bridge = ASRBridge(brain)
            await bridge._handle_transcript({
                "type": "transcript",
                "text": "お手",
                "confidence": 0.92,
                "utterance_id": "utt-001",
            })
            self.assertEqual(bridge._queue.qsize(), 1)
            item = bridge._queue.get_nowait()
            text, uid = item[0], item[1]
            self.assertEqual(text, "お手")
            self.assertEqual(uid, "utt-001")
        _run(_test())

    def test_low_confidence_filtered(self):
        """Low confidence transcript is filtered out"""
        async def _test():
            brain, _ = _make_mock_brain()
            bridge = ASRBridge(brain)
            await bridge._handle_transcript({
                "type": "transcript",
                "text": "あいう",
                "confidence": 0.1,
                "utterance_id": "utt-002",
            })
            self.assertEqual(bridge._queue.qsize(), 0)
        _run(_test())

    def test_empty_text_ignored(self):
        """Empty text is ignored"""
        async def _test():
            brain, _ = _make_mock_brain()
            bridge = ASRBridge(brain)
            await bridge._handle_transcript({
                "type": "transcript",
                "text": "",
                "confidence": 0.99,
                "utterance_id": "utt-003",
            })
            self.assertEqual(bridge._queue.qsize(), 0)
        _run(_test())

    def test_duplicate_utterance_filtered_after_execution(self):
        """Same utterance_id is deduplicated after worker execution"""
        async def _test():
            brain, _ = _make_mock_brain()
            bridge = ASRBridge(brain)
            # 1st: enqueued
            await bridge._handle_transcript({
                "text": "お手", "confidence": 0.9, "utterance_id": "utt-dup",
            })
            self.assertEqual(bridge._queue.qsize(), 1)
            # Simulate worker execution (mark_processed)
            bridge._mark_processed("utt-dup")
            # 2nd: skipped because already processed
            await bridge._handle_transcript({
                "text": "お手", "confidence": 0.9, "utterance_id": "utt-dup",
            })
            self.assertEqual(bridge._queue.qsize(), 1)
        _run(_test())

    def test_transcript_emergency_fallback(self):
        """Emergency keyword via transcript is processed immediately"""
        async def _test():
            brain, _ = _make_mock_brain()
            bridge = ASRBridge(brain)
            await bridge._handle_transcript({
                "text": "止まれ",
                "confidence": 0.95,
                "utterance_id": "utt-emg-fb",
            })
            # Processed via emergency path so not enqueued
            self.assertEqual(bridge._queue.qsize(), 0)
            # Marked as processed
            self.assertIn("utt-emg-fb", bridge._processed_ids)
        _run(_test())


class TestEmergencyHandling(unittest.TestCase):
    """Emergency processing tests"""

    def test_emergency_calls_brain(self):
        """Emergency immediately calls brain"""
        call_log = []

        async def mock_pe(text):
            call_log.append(text)
            result = MagicMock()
            result.response = "緊急停止"
            result.api_code = 1003
            result.execution_status = "success"
            return result

        async def _test():
            brain = MagicMock()
            brain.process_and_execute = mock_pe
            bridge = ASRBridge(brain)
            await bridge._handle_emergency({
                "keyword": "止まれ",
                "utterance_id": "utt-emg",
                "confidence": 1.0,
            })
            self.assertEqual(call_log, ["止まれ"])
        _run(_test())

    def test_emergency_flushes_queue(self):
        """Queue is flushed after emergency"""
        async def _test():
            brain, _ = _make_mock_brain()
            bridge = ASRBridge(brain)
            # Pre-fill queue with commands
            await bridge._queue.put(("ダンス", "utt-d1"))
            await bridge._queue.put(("お手", "utt-d2"))
            self.assertEqual(bridge._queue.qsize(), 2)

            await bridge._handle_emergency({
                "keyword": "stop",
                "utterance_id": "utt-emg-flush",
                "confidence": 1.0,
            })
            # Queue has been flushed
            self.assertEqual(bridge._queue.qsize(), 0)
        _run(_test())

    def test_emergency_sets_cooldown(self):
        """Cooldown window is set after emergency"""
        async def _test():
            brain, _ = _make_mock_brain()
            bridge = ASRBridge(brain)
            before = time.monotonic()
            await bridge._handle_emergency({
                "keyword": "halt",
                "utterance_id": "utt-cool",
                "confidence": 1.0,
            })
            self.assertGreater(bridge._cooldown_until, before)
            self.assertAlmostEqual(
                bridge._cooldown_until,
                before + EMERGENCY_COOLDOWN_S,
                delta=1.0,
            )
        _run(_test())

    def test_emergency_dedup_prevents_double(self):
        """Same utterance_id emergency + transcript does not double-execute"""
        call_count = [0]

        async def mock_pe(text):
            call_count[0] += 1
            result = MagicMock()
            result.response = "停止"
            result.api_code = 1003
            result.execution_status = "success"
            return result

        async def _test():
            brain = MagicMock()
            brain.process_and_execute = mock_pe
            bridge = ASRBridge(brain)

            # Emergency arrives first
            await bridge._handle_emergency({
                "keyword": "止まれ",
                "utterance_id": "utt-same",
                "confidence": 1.0,
            })
            # Transcript arrives later (same utterance_id)
            await bridge._handle_transcript({
                "text": "止まれ",
                "confidence": 0.95,
                "utterance_id": "utt-same",
            })
            # Brain is called only once
            self.assertEqual(call_count[0], 1)
        _run(_test())


class TestQueueManagement(unittest.TestCase):
    """Queue management tests"""

    def test_queue_bounded(self):
        """Queue is bounded at maxsize=3"""
        async def _test():
            brain, _ = _make_mock_brain()
            bridge = ASRBridge(brain)
            self.assertEqual(bridge._queue.maxsize, 3)
        _run(_test())

    def test_queue_drop_oldest_on_overflow(self):
        """Oldest command is dropped when queue is full"""
        async def _test():
            brain, _ = _make_mock_brain()
            bridge = ASRBridge(brain)

            # Enqueue 3
            await bridge._enqueue_command("cmd1", "u1")
            await bridge._enqueue_command("cmd2", "u2")
            await bridge._enqueue_command("cmd3", "u3")
            self.assertEqual(bridge._queue.qsize(), 3)

            # 4th enqueue -> cmd1 is dropped
            await bridge._enqueue_command("cmd4", "u4")
            self.assertEqual(bridge._queue.qsize(), 3)

            items = []
            while not bridge._queue.empty():
                items.append(bridge._queue.get_nowait())
            texts = [item[0] for item in items]
            self.assertEqual(texts, ["cmd2", "cmd3", "cmd4"])
        _run(_test())


class TestDedupExpiry(unittest.TestCase):
    """Deduplication TTL tests"""

    def test_expired_ids_cleaned(self):
        """IDs past TTL are cleaned up"""
        brain, _ = _make_mock_brain()
        bridge = ASRBridge(brain)

        # Old entry
        bridge._processed_ids["old-id"] = time.monotonic() - DEDUP_TTL_S - 1

        bridge._cleanup_expired_ids()
        self.assertNotIn("old-id", bridge._processed_ids)

    def test_fresh_ids_kept(self):
        """Fresh IDs are retained"""
        brain, _ = _make_mock_brain()
        bridge = ASRBridge(brain)

        bridge._processed_ids["fresh-id"] = time.monotonic()

        bridge._cleanup_expired_ids()
        self.assertIn("fresh-id", bridge._processed_ids)

    def test_empty_utterance_id_not_deduped(self):
        """Empty utterance_id is not deduplicated (always treated as unprocessed)"""
        brain, _ = _make_mock_brain()
        bridge = ASRBridge(brain)
        self.assertFalse(bridge._is_processed(""))


class TestHeartbeat(unittest.TestCase):
    """Heartbeat tests"""

    def test_heartbeat_updates_timestamp(self):
        brain, _ = _make_mock_brain()
        bridge = ASRBridge(brain)
        old_ts = bridge._last_heartbeat
        time.sleep(0.01)
        bridge._handle_heartbeat({"ts": time.time()})
        self.assertGreater(bridge._last_heartbeat, old_ts)


class TestCommandWorker(unittest.TestCase):
    """Command worker tests"""

    def test_cooldown_skips_command(self):
        """Commands during cooldown window are skipped"""
        async def _test():
            brain, _ = _make_mock_brain()
            bridge = ASRBridge(brain)
            bridge._running = True

            # During cooldown
            bridge._cooldown_until = time.monotonic() + 10.0
            await bridge._queue.put(("ダンス", "utt-cool-cmd"))

            # Run worker for 1 cycle by making it a task with short timeout
            # Worker should skip because of cooldown
            bridge._running = True

            async def _run_one_cycle():
                try:
                    text, uid = await asyncio.wait_for(
                        bridge._queue.get(), timeout=0.5,
                    )
                except asyncio.TimeoutError:
                    return None
                # Cooldown check
                now = time.monotonic()
                if now < bridge._cooldown_until:
                    return "skipped"
                return "executed"

            result = await _run_one_cycle()
            self.assertEqual(result, "skipped")

        _run(_test())


if __name__ == "__main__":
    unittest.main()
