#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Wake Word Detection -- "Kura-chan" Gating

Composed of 2 classes:
  WakeWordMatcher -- Pure string matching (stateless)
  WakeWordGate    -- Listening window state machine

Detection strategy:
  Longest match from known prefix list (WAKE_PREFIXES).
  Exact match -> standalone (2-step), remainder present -> inline.
  Fuzzy matching (old Layer 1/3) was removed to prevent false positives
  from common words like "kurasu" (class).

Configuration:
  CLAUDIA_WAKE_WORD_ENABLED=1   Enable wake word detection (default OFF)
  CLAUDIA_WAKE_WORD_TIMEOUT=5   Listening window seconds (default 5)
"""

import logging
import os
import re
import time
import unicodedata
from typing import Callable, Optional, Tuple

logger = logging.getLogger("claudia.audio.wake_word")

# Known wake word prefixes (sorted by length descending -- longest match first)
# Covers variations that ASR (whisper-base) may output.
# Do not include common words (kurasu, kurabu, etc.) -- causes false positives.
# Maintain character count descending order when adding new entries.
WAKE_PREFIXES = [
    # 6 characters
    "クローディア", "くろーでぃあ",
    "クラウディア", "くらうでぃあ",
    "クラッチョン",
    "クラーちゃん", "くらーちゃん",
    # 5 characters
    "クロディア", "くろでぃあ",
    "クラちゃん", "くらちゃん",
    "クラチャン",
    "クラチョン",
    # 4 characters
    "クラチャ",
]

# "Kura/kuro" prefixes -- for unknown variant detection logging
_KURA_PREFIXES = ("クラ", "くら", "クロ", "くろ")

# Filler characters to remove from inline remainder
_FILLER_CHARS = re.compile(r"^[さねー、,\s]+")


def _normalize(text):
    # type: (str) -> str
    """NFKC normalization + strip whitespace + remove punctuation"""
    t = unicodedata.normalize("NFKC", text).strip()
    # Remove trailing punctuation (for standalone detection)
    t = t.rstrip("。、！？!?,.")
    return t


class WakeWordMatcher:
    """Pure wake word matching (stateless)

    match(text) -> Optional[Tuple[str, str]]
      - ("", matched)     -- standalone (2-step trigger)
      - (remainder, matched) -- inline (with command)
      - None              -- no wake word detected
    """

    def match(self, text):
        # type: (str) -> Optional[Tuple[str, str]]
        """Detect wake word in text (known prefix exact match only)

        Longest match against WAKE_PREFIXES list.
        Common words (kurasu, etc.) are not in WAKE_PREFIXES,
        so false positives do not occur.

        Returns:
            ("", matched_word) -- standalone wake word (2-step)
            (remainder, matched_prefix) -- inline with command
            None -- wake word not detected
        """
        norm = _normalize(text)
        if not norm:
            return None

        # Longest match from known prefixes
        for prefix in WAKE_PREFIXES:
            if norm.startswith(prefix):
                if len(norm) == len(prefix):
                    return ("", norm)
                remainder = norm[len(prefix):]
                remainder = _FILLER_CHARS.sub("", remainder)
                if remainder:
                    return (remainder, prefix)
                return ("", norm)

        # Unknown variant detection: text starts with "kura/kuro" but
        # does not match any WAKE_PREFIXES entry -- log as WARNING.
        # This logging helps discover new ASR variants.
        for kp in _KURA_PREFIXES:
            if norm.startswith(kp):
                logger.warning(
                    "Wake word candidate mismatch: '%s' (prefix='%s', not registered in WAKE_PREFIXES)",
                    norm, kp,
                )
                break

        return None


class WakeWordGate:
    """Wake Word Gate -- Listening Window State Machine

    States:
      idle      -- waiting for wake word (default)
      listening -- listening window open (timeout seconds)

    filter(text, confidence) -> Optional[str]:
      - Disabled: returns text as-is (transparent)
      - Inline detected: returns remainder
      - Standalone detected: opens window + returns None
      - Within window: returns text as-is (consumed)
      - Otherwise: None (rejected)
    """

    def __init__(
        self,
        on_wake=None,  # type: Optional[Callable[[], None]]
    ):
        # type: (...) -> None
        env_enabled = os.getenv("CLAUDIA_WAKE_WORD_ENABLED", "0")
        self._enabled = env_enabled == "1"
        self._timeout = float(os.getenv("CLAUDIA_WAKE_WORD_TIMEOUT", "5"))

        self._matcher = WakeWordMatcher()
        self._on_wake = on_wake

        # State
        self._state = "idle"  # "idle" | "listening"
        self._window_deadline = 0.0  # monotonic timestamp

        if self._enabled:
            logger.info(
                "Wake word gate enabled: timeout=%.0fs", self._timeout
            )
        else:
            logger.debug("Wake word gate disabled (CLAUDIA_WAKE_WORD_ENABLED != 1)")

    @property
    def enabled(self):
        # type: () -> bool
        return self._enabled

    @property
    def state(self):
        # type: () -> str
        return self._state

    def filter(self, text, confidence):
        # type: (str, float) -> Optional[str]
        """Gate text through wake word filter

        Returns:
            str  -- text to pass to brain
            None -- rejected (no wake word or window opened only)
        """
        if not self._enabled:
            return text

        # Check listening state
        if self._state == "listening":
            now = time.monotonic()
            if now <= self._window_deadline:
                # Within window: accept command and consume window
                self._state = "idle"
                logger.info("Listening window consumed: '%s'", text)
                return text
            else:
                # Window expired: return to idle
                logger.info("Listening window expired")
                self._state = "idle"
                # Fall through: also check this text for wake word

        # Wake word match
        result = self._matcher.match(text)
        if result is None:
            logger.debug("Wake word gate rejected: '%s'", text)
            return None

        remainder, matched = result

        if remainder:
            # Inline: command attached -> return remainder
            logger.info(
                "Wake word inline detected: matched='%s', command='%s'",
                matched, remainder,
            )
            return remainder
        else:
            # Standalone: open listening window
            self._state = "listening"
            self._window_deadline = time.monotonic() + self._timeout
            logger.info(
                "Wake word standalone detected: '%s' -> listening window %.0fs",
                matched, self._timeout,
            )
            if self._on_wake:
                try:
                    self._on_wake()
                except Exception as e:
                    logger.debug("on_wake callback error: %s", e)
            return None

    def reset(self):
        # type: () -> None
        """Reset state to idle (called on emergency)"""
        if self._state != "idle":
            logger.info("Wake word gate reset (emergency)")
        self._state = "idle"
        self._window_deadline = 0.0
