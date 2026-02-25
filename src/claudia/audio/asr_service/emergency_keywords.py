#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Emergency Stop Keyword Matching Module

Kept in sync with production_brain.py EMERGENCY_COMMANDS (lines 611-632).
Pure string operations, no model dependency, matching speed < 1ms.
Prefer false stops over missed stops -- all emergency events execute regardless of confidence.
"""

import re
import unicodedata
from typing import Optional, Set, Tuple

# === Emergency keyword list ===
# Fully synchronized with ProductionBrain.EMERGENCY_COMMANDS keys
# Includes: kanji, ASR kana variants, katakana, English, Chinese
EMERGENCY_KEYWORDS_TEXT = [
    # Japanese (kanji)
    "止まれ",
    "止めて",
    "止まって",
    "緊急停止",
    "やめて",
    # Japanese (ASR kana variants)
    "とまれ",
    "とめて",
    "とまって",
    "きんきゅうていし",
    # Katakana
    "ストップ",
    # English
    "stop",
    "halt",
    "emergency",
    # Chinese
    "停止",
    "停下",
]

# Pre-compiled: normalized keyword set (for exact matching)
_NORMALIZED_KEYWORDS: Set[str] = set()

# Pre-compiled: regex for punctuation stripping
_PUNCTUATION_RE = re.compile(
    r"[\s\u3000\u3001\u3002\uff01\uff0c\uff0e\uff1f"
    r"\uff1a\uff1b!?,.:;\-\u300c\u300d\u300e\u300f"
    r"\u3010\u3011\u2018\u2019\u201c\u201d()（）]"
)


def normalize_for_emergency(text: str) -> str:
    """Normalize text for emergency matching: strip whitespace -> lowercase -> NFKC -> remove punctuation

    Similar to brain's _kana_to_kanji but more simplified,
    focused on letting ASR output variants hit the keyword list.
    """
    # strip + lowercase
    text = text.strip().lower()
    # Unicode NFKC normalization (half-width -> full-width unification, composed char decomposition, etc.)
    text = unicodedata.normalize("NFKC", text)
    # Remove punctuation and whitespace
    text = _PUNCTUATION_RE.sub("", text)
    return text


def _init_normalized_keywords() -> None:
    """Pre-normalize all keywords at startup"""
    global _NORMALIZED_KEYWORDS
    _NORMALIZED_KEYWORDS = {
        normalize_for_emergency(kw) for kw in EMERGENCY_KEYWORDS_TEXT
    }


# Initialize at module load time
_init_normalized_keywords()


def match_emergency(text: str) -> Optional[Tuple[str, float]]:
    """Emergency keyword matching

    Args:
        text: ASR transcription text (raw or already normalized)

    Returns:
        (matched_keyword, confidence_estimate) or None
        confidence: 1.0 = exact match, 0.7-0.9 = partial/substring match
    """
    normalized = normalize_for_emergency(text)
    if not normalized:
        return None

    # Layer 1: Exact match (confidence = 1.0)
    if normalized in _NORMALIZED_KEYWORDS:
        return (normalized, 1.0)

    # Layer 2: Keyword is a substring of the input text (confidence = 0.85)
    # Example: "今すぐ止まれ！" -> contains "止まれ"
    for kw in _NORMALIZED_KEYWORDS:
        if kw in normalized:
            return (kw, 0.85)

    # Layer 3: Input text is a substring of a keyword (confidence = 0.7)
    # Example: ASR only captured "とま" -> prefix of "とまれ"
    # Require at least 2-character match (1 character causes too many false positives)
    if len(normalized) >= 2:
        for kw in _NORMALIZED_KEYWORDS:
            if normalized in kw:
                return (kw, 0.7)

    return None
