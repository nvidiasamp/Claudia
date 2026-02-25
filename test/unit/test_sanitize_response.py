#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_sanitize_response.py — _sanitize_response() unit tests

Validates ProductionBrain._sanitize_response() filtering logic:
  - Empty/None input -> default Japanese response
  - Pure English/non-Japanese -> default Japanese response
  - Contains nonsense words (godee/pong etc.) -> default Japanese response
  - Valid Japanese output -> returned as-is
  - Mixed text containing Japanese -> returned as-is
  - Word boundary matching does not incorrectly filter valid substrings
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from unittest.mock import MagicMock, patch


# Default response constant (consistent with production_brain.py)
DEFAULT_RESPONSE = "すみません、よく分かりません"


def _make_brain_for_sanitize():
    """Create minimal ProductionBrain instance for testing _sanitize_response only

    Skips heavyweight initialization in __init__ (Ollama, DDS, ROS2),
    directly constructs an object with logger and _sanitize_response method.
    """
    from claudia.brain.production_brain import ProductionBrain

    # Bypass __init__, create instance directly
    brain = object.__new__(ProductionBrain)
    brain.logger = MagicMock()
    return brain


class TestSanitizeResponseEmpty(unittest.TestCase):
    """Empty values and boundary inputs"""

    def setUp(self):
        self.brain = _make_brain_for_sanitize()

    def test_none_input(self):
        """None -> default response"""
        result = self.brain._sanitize_response(None)
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_empty_string(self):
        """Empty string -> default response"""
        result = self.brain._sanitize_response("")
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_whitespace_only(self):
        """Whitespace only -> default response"""
        result = self.brain._sanitize_response("   \n\t  ")
        self.assertEqual(result, DEFAULT_RESPONSE)


class TestSanitizeResponseNonJapanese(unittest.TestCase):
    """Non-Japanese output filtering"""

    def setUp(self):
        self.brain = _make_brain_for_sanitize()

    def test_pure_english(self):
        """Pure English -> default response"""
        result = self.brain._sanitize_response("Hello world")
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_pure_numbers(self):
        """Pure numbers -> default response"""
        result = self.brain._sanitize_response("12345")
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_pure_punctuation(self):
        """Pure punctuation -> default response"""
        result = self.brain._sanitize_response("!!??...")
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_korean_text(self):
        """Korean (non-Japanese) -> default response"""
        result = self.brain._sanitize_response("안녕하세요")
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_chinese_only(self):
        """Pure kanji (within CJK Unified Ideographs range) -> returned as-is
        Note: kanji in \\u4e00-\\u9faf range, has_kanji = True"""
        result = self.brain._sanitize_response("你好世界")
        self.assertEqual(result, "你好世界")


class TestSanitizeResponseNonsense(unittest.TestCase):
    """Nonsense word filtering (word-boundary matching)"""

    def setUp(self):
        self.brain = _make_brain_for_sanitize()

    def test_godee_mixed(self):
        """Mixed text containing godee -> default response"""
        result = self.brain._sanitize_response("今日は godee ですね")
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_pong_mixed(self):
        """Mixed text containing pong -> default response"""
        result = self.brain._sanitize_response("ちんちん pong")
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_hi_word_boundary(self):
        """Standalone 'hi' -> default response"""
        result = self.brain._sanitize_response("こんにちは hi")
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_hello_word_boundary(self):
        """Standalone 'hello' -> default response"""
        result = self.brain._sanitize_response("元気です hello")
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_ok_word_boundary(self):
        """Standalone 'ok' -> default response"""
        result = self.brain._sanitize_response("はい ok")
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_yes_word_boundary(self):
        """Standalone 'yes' -> default response"""
        result = self.brain._sanitize_response("分かった yes")
        self.assertEqual(result, DEFAULT_RESPONSE)

    def test_no_word_boundary(self):
        """Standalone 'no' -> default response"""
        result = self.brain._sanitize_response("いいえ no")
        self.assertEqual(result, DEFAULT_RESPONSE)


class TestSanitizeResponseWordBoundary(unittest.TestCase):
    """Word boundary matching does not incorrectly filter valid substrings"""

    def setUp(self):
        self.brain = _make_brain_for_sanitize()

    def test_tokyo_not_matched_by_ok(self):
        """'tokyo' contains 'ok' substring but should not be incorrectly filtered"""
        result = self.brain._sanitize_response("東京はtokyoです")
        self.assertEqual(result, "東京はtokyoです")

    def test_hi_in_longer_word(self):
        """'high' contains 'hi' substring but should not be incorrectly filtered"""
        result = self.brain._sanitize_response("テンションがhighです")
        self.assertEqual(result, "テンションがhighです")

    def test_pong_in_longer_word(self):
        """'pingpong' contains 'pong' substring but should not be incorrectly filtered"""
        result = self.brain._sanitize_response("ピンポン pingpong")
        self.assertEqual(result, "ピンポン pingpong")

    def test_nobody_not_matched(self):
        """'nobody' contains 'no' substring but should not be incorrectly filtered"""
        result = self.brain._sanitize_response("誰もnobodyいない")
        self.assertEqual(result, "誰もnobodyいない")


class TestSanitizeResponseValidJapanese(unittest.TestCase):
    """Valid Japanese output returned as-is"""

    def setUp(self):
        self.brain = _make_brain_for_sanitize()

    def test_hiragana(self):
        """Pure hiragana -> returned as-is"""
        result = self.brain._sanitize_response("こんにちは")
        self.assertEqual(result, "こんにちは")

    def test_katakana(self):
        """Pure katakana -> returned as-is"""
        result = self.brain._sanitize_response("ロボット")
        self.assertEqual(result, "ロボット")

    def test_mixed_japanese(self):
        """Mixed Japanese -> returned as-is"""
        result = self.brain._sanitize_response("お散歩しましょう！ワンワン！")
        self.assertEqual(result, "お散歩しましょう！ワンワン！")

    def test_japanese_with_numbers(self):
        """Japanese + numbers -> returned as-is"""
        result = self.brain._sanitize_response("バッテリーは80%です")
        self.assertEqual(result, "バッテリーは80%です")

    def test_whitespace_stripped(self):
        """Leading/trailing whitespace is stripped"""
        result = self.brain._sanitize_response("  こんにちは  ")
        self.assertEqual(result, "こんにちは")

    def test_action_response_template(self):
        """Action template response -> returned as-is"""
        result = self.brain._sanitize_response("立ちます！")
        self.assertEqual(result, "立ちます！")

    def test_long_japanese_response(self):
        """Long Japanese response -> returned as-is"""
        long_resp = "今日はとても良い天気ですね。お散歩に行きましょうか？"
        result = self.brain._sanitize_response(long_resp)
        self.assertEqual(result, long_resp)


if __name__ == "__main__":
    unittest.main()
