#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
唤醒词 (ウェイクワード) 検出 — "クラちゃん" ゲーティング

2 つのクラスで構成:
  WakeWordMatcher — 純粋な文字列マッチング (状態なし)
  WakeWordGate    — 監聴ウィンドウ状態マシン

検出戦略:
  既知プレフィックスリスト (WAKE_PREFIXES) から最長一致。
  完全一致 → standalone (2-step)、remainder あり → inline。
  "クラス" 等の一般語による false positive を防ぐため、
  fuzzy マッチング (旧 Layer 1/3) は廃止。

設定:
  CLAUDIA_WAKE_WORD_ENABLED=1   唤醒词検出有効化 (デフォルト OFF)
  CLAUDIA_WAKE_WORD_TIMEOUT=5   監聴窓秒数 (デフォルト 5)
"""

import logging
import os
import re
import time
import unicodedata
from typing import Callable, Optional, Tuple

logger = logging.getLogger("claudia.audio.wake_word")

# 既知ウェイクワードプレフィクス (長い順にソート — 最長一致優先)
# ASR (whisper-base) が出力しうるバリエーションを網羅。
# 一般語 (クラス, クラブ等) は含めないこと — false positive の原因になる。
WAKE_PREFIXES = [
    "クローディア", "くろーでぃあ",
    "クラウディア", "くらうでぃあ",
    "クラッチョン",
    "クロディア", "くろでぃあ",
    "クラちゃん", "くらちゃん",
    "クラチャン", "くらちゃん",
    "クラチョン",
    "クラチャ",
    # ASR 観測バリアント (whisper-base 特有)
    "クラーちゃん",
    "くらーちゃん",
]

# inline remainder から除去するフィラー文字
_FILLER_CHARS = re.compile(r"^[さねー、,\s]+")


def _normalize(text):
    # type: (str) -> str
    """NFKC 正規化 + 前後空白除去 + 句読点除去"""
    t = unicodedata.normalize("NFKC", text).strip()
    # 末尾の句読点を除去 (standalone 判定用)
    t = t.rstrip("。、！？!?,.")
    return t


class WakeWordMatcher:
    """純粋なウェイクワードマッチング (状態なし)

    match(text) -> Optional[Tuple[str, str]]
      - ("", matched)     — standalone (2-step トリガー)
      - (remainder, matched) — inline (コマンド付き)
      - None              — ウェイクワードなし
    """

    def match(self, text):
        # type: (str) -> Optional[Tuple[str, str]]
        """テキストからウェイクワードを検出 (既知プレフィクス完全一致のみ)

        WAKE_PREFIXES リストとの最長一致。
        "クラス" 等の一般語は WAKE_PREFIXES に含まれないため
        false positive が発生しない。

        Returns:
            ("", matched_word) — standalone ウェイクワード (2-step)
            (remainder, matched_prefix) — inline コマンド付き
            None — ウェイクワード未検出
        """
        norm = _normalize(text)
        if not norm:
            return None

        # 既知プレフィクスから最長一致
        for prefix in WAKE_PREFIXES:
            if norm.startswith(prefix):
                if len(norm) == len(prefix):
                    return ("", norm)
                remainder = norm[len(prefix):]
                remainder = _FILLER_CHARS.sub("", remainder)
                if remainder:
                    return (remainder, prefix)
                return ("", norm)

        return None


class WakeWordGate:
    """ウェイクワードゲート — 監聴ウィンドウ状態マシン

    状態:
      idle      — ウェイクワード待ち (デフォルト)
      listening — 監聴ウィンドウ開放中 (timeout 秒)

    filter(text, confidence) -> Optional[str]:
      - 無効時: text をそのまま返す (透過)
      - inline 検出: remainder を返す
      - standalone 検出: ウィンドウ開放 + None 返す
      - ウィンドウ内: text をそのまま返す (消費)
      - それ以外: None (拒否)
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

        # 状態
        self._state = "idle"  # "idle" | "listening"
        self._window_deadline = 0.0  # monotonic timestamp

        if self._enabled:
            logger.info(
                "唤醒词ゲート有効: timeout=%.0fs", self._timeout
            )
        else:
            logger.debug("唤醒词ゲート無効 (CLAUDIA_WAKE_WORD_ENABLED != 1)")

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
        """テキストをゲーティング

        Returns:
            str  — brain に渡すテキスト
            None — 拒否 (ウェイクワードなし or ウィンドウ開放のみ)
        """
        if not self._enabled:
            return text

        # listening 状態チェック
        if self._state == "listening":
            now = time.monotonic()
            if now <= self._window_deadline:
                # ウィンドウ内: コマンドを受理し、ウィンドウを消費
                self._state = "idle"
                logger.info("監聴ウィンドウ消費: '%s'", text)
                return text
            else:
                # ウィンドウ期限切れ: idle に戻る
                logger.info("監聴ウィンドウ期限切れ")
                self._state = "idle"
                # fall through: 今回のテキストもウェイクワードチェック

        # ウェイクワードマッチ
        result = self._matcher.match(text)
        if result is None:
            logger.debug("唤醒词ゲート拒否: '%s'", text)
            return None

        remainder, matched = result

        if remainder:
            # inline: コマンド付き → remainder を返す
            logger.info(
                "唤醒词 inline 検出: matched='%s', command='%s'",
                matched, remainder,
            )
            return remainder
        else:
            # standalone: ウィンドウ開放
            self._state = "listening"
            self._window_deadline = time.monotonic() + self._timeout
            logger.info(
                "唤醒詞 standalone 検出: '%s' → 監聴ウィンドウ %.0fs",
                matched, self._timeout,
            )
            if self._on_wake:
                try:
                    self._on_wake()
                except Exception as e:
                    logger.debug("on_wake コールバックエラー: %s", e)
            return None

    def reset(self):
        # type: () -> None
        """状態を idle にリセット (emergency 時に呼出)"""
        if self._state != "idle":
            logger.info("唤醒詞ゲートリセット (emergency)")
        self._state = "idle"
        self._window_deadline = 0.0
