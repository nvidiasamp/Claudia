#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""VoiceCommander ユニットテスト"""

import asyncio
import os
import sys
import unittest
from unittest.mock import MagicMock, patch

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, _PROJECT_ROOT)
sys.path.insert(0, os.path.join(_PROJECT_ROOT, "src"))


def _run(coro):
    """Python 3.8 互換 asyncio テストヘルパー"""
    loop = asyncio.new_event_loop()
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


class TestVoiceCommanderInit(unittest.TestCase):
    """初期化テスト"""

    @patch("voice_commander.ProductionBrain")
    def test_default_init(self, _mock_brain):
        from voice_commander import VoiceCommander
        vc = VoiceCommander()
        self.assertFalse(vc._use_real_hardware)
        self.assertFalse(vc._asr_mock)
        self.assertEqual(vc._command_count, 0)

    @patch("voice_commander.ProductionBrain")
    def test_hardware_mode(self, _mock_brain):
        from voice_commander import VoiceCommander
        vc = VoiceCommander(use_real_hardware=True)
        self.assertTrue(vc._use_real_hardware)

    @patch("voice_commander.ProductionBrain")
    def test_asr_mock_mode(self, _mock_brain):
        from voice_commander import VoiceCommander
        vc = VoiceCommander(asr_mock=True)
        self.assertTrue(vc._asr_mock)


class TestDisplayResult(unittest.TestCase):
    """結果表示テスト"""

    @patch("voice_commander.ProductionBrain")
    def test_display_transcript(self, _mock_brain):
        from voice_commander import VoiceCommander
        vc = VoiceCommander()
        # 例外を投げない (stdout 出力のみ)
        vc._display_result("transcript", "お手", 0.92)

    @patch("voice_commander.ProductionBrain")
    def test_display_emergency(self, _mock_brain):
        from voice_commander import VoiceCommander
        vc = VoiceCommander()
        vc._display_result("emergency", "止まれ", 1.0)

    @patch("voice_commander.ProductionBrain")
    def test_display_result_increments_count(self, _mock_brain):
        from voice_commander import VoiceCommander
        vc = VoiceCommander()
        mock_result = MagicMock()
        mock_result.response = "テスト"
        mock_result.api_code = 1016
        mock_result.sequence = None
        mock_result.execution_status = "success"
        vc._display_result("result", "お手", mock_result)
        self.assertEqual(vc._command_count, 1)


class TestShutdownSequence(unittest.TestCase):
    """シャットダウンシーケンステスト"""

    @patch("voice_commander.ProductionBrain")
    def test_shutdown_without_components(self, _mock_brain):
        """コンポーネント未初期化でも shutdown が安全に完了する"""
        from voice_commander import VoiceCommander

        async def _test():
            vc = VoiceCommander()
            await vc._shutdown()

        _run(_test())

    @patch("voice_commander.ProductionBrain")
    def test_signal_handler_sets_event(self, _mock_brain):
        from voice_commander import VoiceCommander
        vc = VoiceCommander()
        self.assertFalse(vc._shutdown_event.is_set())
        vc._signal_handler()
        self.assertTrue(vc._shutdown_event.is_set())


class TestCLIParsing(unittest.TestCase):
    """CLI 引数パースのテスト"""

    def test_parse_default(self):
        from voice_commander import _parse_args
        with patch("sys.argv", ["voice_commander.py"]):
            args = _parse_args()
        self.assertFalse(args.hardware)
        self.assertFalse(args.asr_mock)

    def test_parse_hardware(self):
        from voice_commander import _parse_args
        with patch("sys.argv", ["voice_commander.py", "--hardware"]):
            args = _parse_args()
        self.assertTrue(args.hardware)

    def test_parse_asr_mock(self):
        from voice_commander import _parse_args
        with patch("sys.argv", ["voice_commander.py", "--asr-mock"]):
            args = _parse_args()
        self.assertTrue(args.asr_mock)


if __name__ == "__main__":
    unittest.main()
