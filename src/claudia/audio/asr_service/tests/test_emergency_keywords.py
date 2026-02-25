#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""emergency_keywords unit tests"""

import time
import pytest

from claudia.audio.asr_service.emergency_keywords import (
    EMERGENCY_KEYWORDS_TEXT,
    normalize_for_emergency,
    match_emergency,
    _NORMALIZED_KEYWORDS,
)


# ============================================================
# normalize_for_emergency
# ============================================================

class TestNormalize:

    def test_strips_whitespace(self):
        assert normalize_for_emergency("  stop  ") == "stop"

    def test_lowercase(self):
        assert normalize_for_emergency("STOP") == "stop"
        assert normalize_for_emergency("Stop") == "stop"

    def test_nfkc_normalization(self):
        # Half-width katakana -> full-width katakana
        assert normalize_for_emergency("ｽﾄｯﾌﾟ") == "ストップ"

    def test_strips_punctuation(self):
        assert normalize_for_emergency("止まれ！") == "止まれ"
        assert normalize_for_emergency("止まれ。") == "止まれ"
        assert normalize_for_emergency("「止まれ」") == "止まれ"
        assert normalize_for_emergency("stop!") == "stop"
        assert normalize_for_emergency("stop?") == "stop"

    def test_strips_mixed_whitespace_and_punctuation(self):
        assert normalize_for_emergency("　止まれ ！　") == "止まれ"

    def test_empty_string(self):
        assert normalize_for_emergency("") == ""

    def test_only_punctuation(self):
        assert normalize_for_emergency("！？。") == ""


# ============================================================
# match_emergency: Exact match (confidence = 1.0)
# ============================================================

class TestExactMatch:

    def test_kanji_exact(self):
        result = match_emergency("止まれ")
        assert result is not None
        kw, conf = result
        assert conf == 1.0

    def test_kana_exact(self):
        result = match_emergency("とまれ")
        assert result is not None
        _, conf = result
        assert conf == 1.0

    def test_katakana_exact(self):
        result = match_emergency("ストップ")
        assert result is not None
        _, conf = result
        assert conf == 1.0

    def test_english_exact(self):
        result = match_emergency("stop")
        assert result is not None
        _, conf = result
        assert conf == 1.0

    def test_chinese_exact(self):
        result = match_emergency("停止")
        assert result is not None
        _, conf = result
        assert conf == 1.0

    def test_case_insensitive(self):
        result = match_emergency("STOP")
        assert result is not None
        _, conf = result
        assert conf == 1.0

    def test_with_whitespace_and_punctuation(self):
        """Exact match after normalization"""
        result = match_emergency("  止まれ！ ")
        assert result is not None
        _, conf = result
        assert conf == 1.0


# ============================================================
# match_emergency: Substring match (confidence = 0.85)
# ============================================================

class TestSubstringMatch:

    def test_keyword_embedded_in_sentence(self):
        result = match_emergency("今すぐ止まれ！")
        assert result is not None
        kw, conf = result
        assert conf == 0.85

    def test_keyword_at_end_of_sentence(self):
        result = match_emergency("ロボットstop")
        assert result is not None
        _, conf = result
        assert conf == 0.85

    def test_long_sentence_with_emergency(self):
        result = match_emergency("お願いだからやめてください")
        assert result is not None
        kw, conf = result
        assert conf == 0.85


# ============================================================
# match_emergency: Partial/prefix match (confidence = 0.7)
# ============================================================

class TestPartialMatch:

    def test_prefix_of_keyword(self):
        """ASR only captured "とま" -> prefix of "とまれ" """
        result = match_emergency("とま")
        assert result is not None
        _, conf = result
        assert conf == 0.7

    def test_single_char_no_match(self):
        """Single character does not match to prevent false positives"""
        result = match_emergency("と")
        assert result is None

    def test_two_char_partial(self):
        result = match_emergency("停下")  # This is an exact match
        assert result is not None
        # "停下" is one of the keywords, so it should be 1.0
        _, conf = result
        assert conf == 1.0


# ============================================================
# match_emergency: Non-emergency text
# ============================================================

class TestNonEmergency:

    def test_normal_command(self):
        assert match_emergency("座って") is None

    def test_greeting(self):
        assert match_emergency("こんにちは") is None

    def test_random_text(self):
        assert match_emergency("今日はいい天気ですね") is None

    def test_empty_string(self):
        assert match_emergency("") is None

    def test_only_punctuation(self):
        assert match_emergency("！？") is None


# ============================================================
# Full keyword coverage
# ============================================================

class TestAllKeywordsCovered:
    """Every entry in EMERGENCY_KEYWORDS_TEXT must be recognized"""

    @pytest.mark.parametrize("keyword", EMERGENCY_KEYWORDS_TEXT)
    def test_each_keyword_matches(self, keyword):
        result = match_emergency(keyword)
        assert result is not None, f"Keyword not matched: {keyword}"
        _, conf = result
        assert conf == 1.0, f"Keyword {keyword} should be exact match (1.0)"


# ============================================================
# Performance: pure string operations, no model dependency
# ============================================================

class TestPerformance:

    def test_matching_is_fast(self):
        """1000 iterations of matching should complete within 50ms"""
        start = time.monotonic()
        for _ in range(1000):
            match_emergency("止まれ")
            match_emergency("今すぐストップして")
            match_emergency("座って")
        elapsed_ms = (time.monotonic() - start) * 1000
        assert elapsed_ms < 50, f"Too slow: {elapsed_ms:.1f}ms for 3000 calls"


# ============================================================
# _NORMALIZED_KEYWORDS initialization
# ============================================================

class TestNormalizedKeywordsInit:

    def test_initialized_at_import(self):
        assert len(_NORMALIZED_KEYWORDS) > 0

    def test_count_matches_source(self):
        """After normalization there may be merges (e.g., kanji and kana normalize to the same form),
        but the count should never exceed the source list"""
        assert len(_NORMALIZED_KEYWORDS) <= len(EMERGENCY_KEYWORDS_TEXT)
