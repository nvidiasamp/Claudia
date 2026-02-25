#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Wake word detection — unit tests

WakeWordMatcher: Pure string matching tests
WakeWordGate:    State machine + filtering tests
"""

import os
import sys
import time
import unittest
from unittest.mock import MagicMock, patch

# Test target
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))
from claudia.audio.wake_word import WakeWordMatcher, WakeWordGate


class TestWakeWordMatcher(unittest.TestCase):
    """WakeWordMatcher unit tests"""

    def setUp(self):
        self.matcher = WakeWordMatcher()

    # ------------------------------------------------------------------
    # Standalone detection (exact match with known prefixes)
    # ------------------------------------------------------------------

    def test_ideal_wake_word(self):
        """Ideal form "クラちゃん" -> standalone"""
        result = self.matcher.match("クラちゃん")
        self.assertIsNotNone(result)
        self.assertEqual(result[0], "")

    def test_observed_variant_kracchon(self):
        """ASR observed: "クラッチョン" -> standalone"""
        result = self.matcher.match("クラッチョン")
        self.assertIsNotNone(result)
        self.assertEqual(result[0], "")

    def test_observed_variant_kurachon(self):
        """ASR observed: "クラチョン" -> standalone"""
        result = self.matcher.match("クラチョン")
        self.assertIsNotNone(result)
        self.assertEqual(result[0], "")

    def test_observed_variant_kuracha(self):
        """ASR observed: "クラチャ" (truncated) -> standalone"""
        result = self.matcher.match("クラチャ")
        self.assertIsNotNone(result)
        self.assertEqual(result[0], "")

    def test_hiragana_kurachan(self):
        """Hiragana "くらちゃん" -> standalone"""
        result = self.matcher.match("くらちゃん")
        self.assertIsNotNone(result)
        self.assertEqual(result[0], "")

    def test_kurasu_rejected(self):
        """"クラス" is a common word -> rejected (false positive prevention)"""
        result = self.matcher.match("クラス")
        self.assertIsNone(result)

    def test_kura_alone_rejected(self):
        """"クラ" alone is too short to distinguish from common words -> rejected"""
        result = self.matcher.match("クラ")
        self.assertIsNone(result)

    def test_with_trailing_punctuation(self):
        """Trailing punctuation is stripped: "クラちゃん！" -> standalone"""
        result = self.matcher.match("クラちゃん！")
        self.assertIsNotNone(result)
        self.assertEqual(result[0], "")

    # ------------------------------------------------------------------
    # Inline detection (known prefix + remainder)
    # ------------------------------------------------------------------

    def test_inline_kurachan_odotte(self):
        """inline: "クラちゃん踊って" -> ("踊って", "クラちゃん")"""
        result = self.matcher.match("クラちゃん踊って")
        self.assertIsNotNone(result)
        self.assertEqual(result[0], "踊って")
        self.assertEqual(result[1], "クラちゃん")

    def test_inline_kracchon_odotte(self):
        """inline: "クラッチョン踊って" -> ("踊って", "クラッチョン")"""
        result = self.matcher.match("クラッチョン踊って")
        self.assertIsNotNone(result)
        self.assertEqual(result[0], "踊って")
        self.assertEqual(result[1], "クラッチョン")

    def test_inline_kurachon_odotte(self):
        """inline: "クラチョン踊って" -> ("踊って", "クラチョン")"""
        result = self.matcher.match("クラチョン踊って")
        self.assertIsNotNone(result)
        self.assertEqual(result[0], "踊って")

    def test_inline_kurasu_rejected(self):
        """inline: "クラス踊って" -> None (クラス is not a known prefix)"""
        result = self.matcher.match("クラス踊って")
        self.assertIsNone(result)

    def test_inline_kurabu_rejected(self):
        """inline: "クラブ踊って" -> None (クラブ is a common word)"""
        result = self.matcher.match("クラブ踊って")
        self.assertIsNone(result)

    def test_inline_hiragana(self):
        """inline hiragana: "くらちゃんおどって" -> ("おどって", ...)"""
        result = self.matcher.match("くらちゃんおどって")
        self.assertIsNotNone(result)
        self.assertIn("おどって", result[0])

    def test_inline_with_filler(self):
        """inline filler removal: "クラちゃんさ踊って" -> ("踊って", ...)"""
        result = self.matcher.match("クラちゃんさ踊って")
        self.assertIsNotNone(result)
        self.assertEqual(result[0], "踊って")

    def test_inline_claudia_full(self):
        """inline long name: "クローディア踊って" -> ("踊って", "クローディア")"""
        result = self.matcher.match("クローディア踊って")
        self.assertIsNotNone(result)
        self.assertEqual(result[0], "踊って")
        self.assertEqual(result[1], "クローディア")

    # ------------------------------------------------------------------
    # No detection
    # ------------------------------------------------------------------

    def test_no_wake_word_odotte(self):
        """No wake word: "踊って" -> None"""
        result = self.matcher.match("踊って")
        self.assertIsNone(result)

    def test_no_wake_word_dance(self):
        """No wake word: "ダンス" -> None"""
        result = self.matcher.match("ダンス")
        self.assertIsNone(result)

    def test_empty_string(self):
        """Empty string -> None"""
        result = self.matcher.match("")
        self.assertIsNone(result)

    def test_whitespace_only(self):
        """Whitespace only -> None"""
        result = self.matcher.match("   ")
        self.assertIsNone(result)

    def test_no_wake_word_stop(self):
        """Emergency command: "止まれ" -> None"""
        result = self.matcher.match("止まれ")
        self.assertIsNone(result)


class TestWakeWordGate(unittest.TestCase):
    """WakeWordGate unit tests"""

    # ------------------------------------------------------------------
    # Disabled mode
    # ------------------------------------------------------------------

    @patch.dict(os.environ, {"CLAUDIA_WAKE_WORD_ENABLED": "0"}, clear=False)
    def test_disabled_passes_all(self):
        """When disabled: all text passes through as-is"""
        gate = WakeWordGate()
        self.assertFalse(gate.enabled)
        self.assertEqual(gate.filter("踊って", 0.8), "踊って")
        self.assertEqual(gate.filter("座って", 0.6), "座って")

    @patch.dict(os.environ, {}, clear=False)
    def test_default_disabled(self):
        """Default (env var not set) -> disabled"""
        # Remove CLAUDIA_WAKE_WORD_ENABLED
        env = os.environ.copy()
        env.pop("CLAUDIA_WAKE_WORD_ENABLED", None)
        with patch.dict(os.environ, env, clear=True):
            gate = WakeWordGate()
            self.assertFalse(gate.enabled)

    # ------------------------------------------------------------------
    # Enabled mode: basic filtering
    # ------------------------------------------------------------------

    @patch.dict(os.environ, {"CLAUDIA_WAKE_WORD_ENABLED": "1"}, clear=False)
    def test_enabled_rejects_no_wake_word(self):
        """When enabled: no wake word -> None (rejected)"""
        gate = WakeWordGate()
        self.assertTrue(gate.enabled)
        self.assertIsNone(gate.filter("踊って", 0.8))

    @patch.dict(os.environ, {"CLAUDIA_WAKE_WORD_ENABLED": "1"}, clear=False)
    def test_enabled_inline(self):
        """When enabled: inline "クラちゃん踊って" -> "踊って" """
        gate = WakeWordGate()
        result = gate.filter("クラちゃん踊って", 0.8)
        self.assertEqual(result, "踊って")

    @patch.dict(os.environ, {"CLAUDIA_WAKE_WORD_ENABLED": "1"}, clear=False)
    def test_inline_does_not_change_state(self):
        """After inline detection, idle state is maintained"""
        gate = WakeWordGate()
        gate.filter("クラちゃん踊って", 0.8)
        self.assertEqual(gate.state, "idle")

    # ------------------------------------------------------------------
    # 2-step mode
    # ------------------------------------------------------------------

    @patch.dict(os.environ, {"CLAUDIA_WAKE_WORD_ENABLED": "1"}, clear=False)
    def test_two_step_wake_then_command(self):
        """2-step: standalone -> listening; next text -> passes through"""
        gate = WakeWordGate()

        # Step 1: wake word only -> None + listening
        result = gate.filter("クラチョン", 0.6)
        self.assertIsNone(result)
        self.assertEqual(gate.state, "listening")

        # Step 2: command -> passes through
        result = gate.filter("踊って", 0.8)
        self.assertEqual(result, "踊って")
        self.assertEqual(gate.state, "idle")

    @patch.dict(os.environ, {"CLAUDIA_WAKE_WORD_ENABLED": "1"}, clear=False)
    def test_window_consumed_after_one_command(self):
        """Window is consumed after one command"""
        gate = WakeWordGate()

        gate.filter("クラちゃん", 0.6)
        self.assertEqual(gate.state, "listening")

        gate.filter("踊って", 0.8)
        self.assertEqual(gate.state, "idle")

        # 3rd call: no wake word -> rejected
        self.assertIsNone(gate.filter("座って", 0.8))

    @patch.dict(os.environ, {
        "CLAUDIA_WAKE_WORD_ENABLED": "1",
        "CLAUDIA_WAKE_WORD_TIMEOUT": "0.1",
    }, clear=False)
    def test_window_expiry(self):
        """Window expiry: rejected after timeout"""
        gate = WakeWordGate()

        gate.filter("クラちゃん", 0.6)
        self.assertEqual(gate.state, "listening")

        # Exceed timeout
        time.sleep(0.15)

        result = gate.filter("踊って", 0.8)
        self.assertIsNone(result)
        self.assertEqual(gate.state, "idle")

    # ------------------------------------------------------------------
    # on_wake callback
    # ------------------------------------------------------------------

    @patch.dict(os.environ, {"CLAUDIA_WAKE_WORD_ENABLED": "1"}, clear=False)
    def test_on_wake_called_on_standalone(self):
        """on_wake callback is called on standalone detection"""
        cb = MagicMock()
        gate = WakeWordGate(on_wake=cb)

        gate.filter("クラちゃん", 0.6)
        cb.assert_called_once()

    @patch.dict(os.environ, {"CLAUDIA_WAKE_WORD_ENABLED": "1"}, clear=False)
    def test_on_wake_not_called_on_inline(self):
        """on_wake is not called on inline detection"""
        cb = MagicMock()
        gate = WakeWordGate(on_wake=cb)

        gate.filter("クラちゃん踊って", 0.8)
        cb.assert_not_called()

    @patch.dict(os.environ, {"CLAUDIA_WAKE_WORD_ENABLED": "1"}, clear=False)
    def test_on_wake_exception_handled(self):
        """on_wake callback exceptions are silently caught"""
        cb = MagicMock(side_effect=RuntimeError("test"))
        gate = WakeWordGate(on_wake=cb)

        # Confirm exception does not propagate
        gate.filter("クラちゃん", 0.6)
        cb.assert_called_once()
        self.assertEqual(gate.state, "listening")

    # ------------------------------------------------------------------
    # reset
    # ------------------------------------------------------------------

    @patch.dict(os.environ, {"CLAUDIA_WAKE_WORD_ENABLED": "1"}, clear=False)
    def test_reset_closes_window(self):
        """reset() closes the listening window"""
        gate = WakeWordGate()

        gate.filter("クラちゃん", 0.6)
        self.assertEqual(gate.state, "listening")

        gate.reset()
        self.assertEqual(gate.state, "idle")

        # After reset, rejected without wake word
        self.assertIsNone(gate.filter("踊って", 0.8))

    @patch.dict(os.environ, {"CLAUDIA_WAKE_WORD_ENABLED": "1"}, clear=False)
    def test_reset_from_idle_is_noop(self):
        """reset from idle state is a safe no-op"""
        gate = WakeWordGate()
        gate.reset()
        self.assertEqual(gate.state, "idle")


if __name__ == "__main__":
    unittest.main()
