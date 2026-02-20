#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
唤醒词 (ウェイクワード) 検出 — ユニットテスト

WakeWordMatcher: 純粋文字列マッチングのテスト
WakeWordGate:    状態マシン + フィルタリングのテスト
"""

import os
import sys
import time
import unittest
from unittest.mock import MagicMock, patch

# テスト対象
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))
from claudia.audio.wake_word import WakeWordMatcher, WakeWordGate


class TestWakeWordMatcher(unittest.TestCase):
    """WakeWordMatcher のユニットテスト"""

    def setUp(self):
        self.matcher = WakeWordMatcher()

    # ------------------------------------------------------------------
    # Standalone 検出 (既知プレフィクス完全一致)
    # ------------------------------------------------------------------

    def test_ideal_wake_word(self):
        """理想形 "クラちゃん" → standalone"""
        result = self.matcher.match("クラちゃん")
        self.assertIsNotNone(result)
        self.assertEqual(result[0], "")

    def test_observed_variant_kracchon(self):
        """ASR 観測: "クラッチョン" → standalone"""
        result = self.matcher.match("クラッチョン")
        self.assertIsNotNone(result)
        self.assertEqual(result[0], "")

    def test_observed_variant_kurachon(self):
        """ASR 観測: "クラチョン" → standalone"""
        result = self.matcher.match("クラチョン")
        self.assertIsNotNone(result)
        self.assertEqual(result[0], "")

    def test_observed_variant_kuracha(self):
        """ASR 観測: "クラチャ" (切断) → standalone"""
        result = self.matcher.match("クラチャ")
        self.assertIsNotNone(result)
        self.assertEqual(result[0], "")

    def test_hiragana_kurachan(self):
        """ひらがな "くらちゃん" → standalone"""
        result = self.matcher.match("くらちゃん")
        self.assertIsNotNone(result)
        self.assertEqual(result[0], "")

    def test_kurasu_rejected(self):
        """"クラス" は一般語 → 拒否 (false positive 防止)"""
        result = self.matcher.match("クラス")
        self.assertIsNone(result)

    def test_kura_alone_rejected(self):
        """"クラ" のみは短すぎて一般語と区別不能 → 拒否"""
        result = self.matcher.match("クラ")
        self.assertIsNone(result)

    def test_with_trailing_punctuation(self):
        """末尾句読点は除去される: "クラちゃん！" → standalone"""
        result = self.matcher.match("クラちゃん！")
        self.assertIsNotNone(result)
        self.assertEqual(result[0], "")

    # ------------------------------------------------------------------
    # Inline 検出 (既知プレフィクス + remainder)
    # ------------------------------------------------------------------

    def test_inline_kurachan_odotte(self):
        """inline: "クラちゃん踊って" → ("踊って", "クラちゃん")"""
        result = self.matcher.match("クラちゃん踊って")
        self.assertIsNotNone(result)
        self.assertEqual(result[0], "踊って")
        self.assertEqual(result[1], "クラちゃん")

    def test_inline_kracchon_odotte(self):
        """inline: "クラッチョン踊って" → ("踊って", "クラッチョン")"""
        result = self.matcher.match("クラッチョン踊って")
        self.assertIsNotNone(result)
        self.assertEqual(result[0], "踊って")
        self.assertEqual(result[1], "クラッチョン")

    def test_inline_kurachon_odotte(self):
        """inline: "クラチョン踊って" → ("踊って", "クラチョン")"""
        result = self.matcher.match("クラチョン踊って")
        self.assertIsNotNone(result)
        self.assertEqual(result[0], "踊って")

    def test_inline_kurasu_rejected(self):
        """inline: "クラス踊って" → None (クラス は既知プレフィクスではない)"""
        result = self.matcher.match("クラス踊って")
        self.assertIsNone(result)

    def test_inline_kurabu_rejected(self):
        """inline: "クラブ踊って" → None (クラブ は一般語)"""
        result = self.matcher.match("クラブ踊って")
        self.assertIsNone(result)

    def test_inline_hiragana(self):
        """inline ひらがな: "くらちゃんおどって" → ("おどって", ...)"""
        result = self.matcher.match("くらちゃんおどって")
        self.assertIsNotNone(result)
        self.assertIn("おどって", result[0])

    def test_inline_with_filler(self):
        """inline フィラー除去: "クラちゃんさ踊って" → ("踊って", ...)"""
        result = self.matcher.match("クラちゃんさ踊って")
        self.assertIsNotNone(result)
        self.assertEqual(result[0], "踊って")

    def test_inline_claudia_full(self):
        """inline 長い名前: "クローディア踊って" → ("踊って", "クローディア")"""
        result = self.matcher.match("クローディア踊って")
        self.assertIsNotNone(result)
        self.assertEqual(result[0], "踊って")
        self.assertEqual(result[1], "クローディア")

    # ------------------------------------------------------------------
    # 非検出
    # ------------------------------------------------------------------

    def test_no_wake_word_odotte(self):
        """ウェイクワードなし: "踊って" → None"""
        result = self.matcher.match("踊って")
        self.assertIsNone(result)

    def test_no_wake_word_dance(self):
        """ウェイクワードなし: "ダンス" → None"""
        result = self.matcher.match("ダンス")
        self.assertIsNone(result)

    def test_empty_string(self):
        """空文字列 → None"""
        result = self.matcher.match("")
        self.assertIsNone(result)

    def test_whitespace_only(self):
        """空白のみ → None"""
        result = self.matcher.match("   ")
        self.assertIsNone(result)

    def test_no_wake_word_stop(self):
        """緊急コマンド: "止まれ" → None"""
        result = self.matcher.match("止まれ")
        self.assertIsNone(result)


class TestWakeWordGate(unittest.TestCase):
    """WakeWordGate のユニットテスト"""

    # ------------------------------------------------------------------
    # 無効モード
    # ------------------------------------------------------------------

    @patch.dict(os.environ, {"CLAUDIA_WAKE_WORD_ENABLED": "0"}, clear=False)
    def test_disabled_passes_all(self):
        """無効時: 全テキストがそのまま通過"""
        gate = WakeWordGate()
        self.assertFalse(gate.enabled)
        self.assertEqual(gate.filter("踊って", 0.8), "踊って")
        self.assertEqual(gate.filter("座って", 0.6), "座って")

    @patch.dict(os.environ, {}, clear=False)
    def test_default_disabled(self):
        """デフォルト (環境変数未設定) → 無効"""
        # CLAUDIA_WAKE_WORD_ENABLED を除去
        env = os.environ.copy()
        env.pop("CLAUDIA_WAKE_WORD_ENABLED", None)
        with patch.dict(os.environ, env, clear=True):
            gate = WakeWordGate()
            self.assertFalse(gate.enabled)

    # ------------------------------------------------------------------
    # 有効モード: 基本フィルタリング
    # ------------------------------------------------------------------

    @patch.dict(os.environ, {"CLAUDIA_WAKE_WORD_ENABLED": "1"}, clear=False)
    def test_enabled_rejects_no_wake_word(self):
        """有効時: ウェイクワードなし → None (拒否)"""
        gate = WakeWordGate()
        self.assertTrue(gate.enabled)
        self.assertIsNone(gate.filter("踊って", 0.8))

    @patch.dict(os.environ, {"CLAUDIA_WAKE_WORD_ENABLED": "1"}, clear=False)
    def test_enabled_inline(self):
        """有効時: inline "クラちゃん踊って" → "踊って" """
        gate = WakeWordGate()
        result = gate.filter("クラちゃん踊って", 0.8)
        self.assertEqual(result, "踊って")

    @patch.dict(os.environ, {"CLAUDIA_WAKE_WORD_ENABLED": "1"}, clear=False)
    def test_inline_does_not_change_state(self):
        """inline 検出後も idle 状態を維持"""
        gate = WakeWordGate()
        gate.filter("クラちゃん踊って", 0.8)
        self.assertEqual(gate.state, "idle")

    # ------------------------------------------------------------------
    # 2-step モード
    # ------------------------------------------------------------------

    @patch.dict(os.environ, {"CLAUDIA_WAKE_WORD_ENABLED": "1"}, clear=False)
    def test_two_step_wake_then_command(self):
        """2-step: standalone → listening; 次のテキスト → 通過"""
        gate = WakeWordGate()

        # ステップ 1: ウェイクワードのみ → None + listening
        result = gate.filter("クラチョン", 0.6)
        self.assertIsNone(result)
        self.assertEqual(gate.state, "listening")

        # ステップ 2: コマンド → 通過
        result = gate.filter("踊って", 0.8)
        self.assertEqual(result, "踊って")
        self.assertEqual(gate.state, "idle")

    @patch.dict(os.environ, {"CLAUDIA_WAKE_WORD_ENABLED": "1"}, clear=False)
    def test_window_consumed_after_one_command(self):
        """ウィンドウはコマンド 1 回で消費される"""
        gate = WakeWordGate()

        gate.filter("クラちゃん", 0.6)
        self.assertEqual(gate.state, "listening")

        gate.filter("踊って", 0.8)
        self.assertEqual(gate.state, "idle")

        # 3 回目: ウェイクワードなし → 拒否
        self.assertIsNone(gate.filter("座って", 0.8))

    @patch.dict(os.environ, {
        "CLAUDIA_WAKE_WORD_ENABLED": "1",
        "CLAUDIA_WAKE_WORD_TIMEOUT": "0.1",
    }, clear=False)
    def test_window_expiry(self):
        """ウィンドウ期限切れ: timeout 後は拒否"""
        gate = WakeWordGate()

        gate.filter("クラちゃん", 0.6)
        self.assertEqual(gate.state, "listening")

        # timeout 超過
        time.sleep(0.15)

        result = gate.filter("踊って", 0.8)
        self.assertIsNone(result)
        self.assertEqual(gate.state, "idle")

    # ------------------------------------------------------------------
    # on_wake コールバック
    # ------------------------------------------------------------------

    @patch.dict(os.environ, {"CLAUDIA_WAKE_WORD_ENABLED": "1"}, clear=False)
    def test_on_wake_called_on_standalone(self):
        """standalone 検出時に on_wake コールバックが呼ばれる"""
        cb = MagicMock()
        gate = WakeWordGate(on_wake=cb)

        gate.filter("クラちゃん", 0.6)
        cb.assert_called_once()

    @patch.dict(os.environ, {"CLAUDIA_WAKE_WORD_ENABLED": "1"}, clear=False)
    def test_on_wake_not_called_on_inline(self):
        """inline 検出時に on_wake は呼ばれない"""
        cb = MagicMock()
        gate = WakeWordGate(on_wake=cb)

        gate.filter("クラちゃん踊って", 0.8)
        cb.assert_not_called()

    @patch.dict(os.environ, {"CLAUDIA_WAKE_WORD_ENABLED": "1"}, clear=False)
    def test_on_wake_exception_handled(self):
        """on_wake コールバックの例外は握り潰される"""
        cb = MagicMock(side_effect=RuntimeError("test"))
        gate = WakeWordGate(on_wake=cb)

        # 例外が伝播しないことを確認
        gate.filter("クラちゃん", 0.6)
        cb.assert_called_once()
        self.assertEqual(gate.state, "listening")

    # ------------------------------------------------------------------
    # reset
    # ------------------------------------------------------------------

    @patch.dict(os.environ, {"CLAUDIA_WAKE_WORD_ENABLED": "1"}, clear=False)
    def test_reset_closes_window(self):
        """reset() でリスニングウィンドウが閉じる"""
        gate = WakeWordGate()

        gate.filter("クラちゃん", 0.6)
        self.assertEqual(gate.state, "listening")

        gate.reset()
        self.assertEqual(gate.state, "idle")

        # reset 後はウェイクワードなしで拒否
        self.assertIsNone(gate.filter("踊って", 0.8))

    @patch.dict(os.environ, {"CLAUDIA_WAKE_WORD_ENABLED": "1"}, clear=False)
    def test_reset_from_idle_is_noop(self):
        """idle 状態からの reset は安全に無操作"""
        gate = WakeWordGate()
        gate.reset()
        self.assertEqual(gate.state, "idle")


if __name__ == "__main__":
    unittest.main()
