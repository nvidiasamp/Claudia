#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
E2E タイミング分析スクリプト — ポスター掲載用統計値の算出

logs/e2e_timing.jsonl を読み取り、ルート別の P50/Mean/P95 を算出する。
voice_commander.py 実行中に自動生成される E2E ログを分析対象とする。

使い方:
    python3 scripts/e2e_timing_analysis.py
    python3 scripts/e2e_timing_analysis.py --file logs/e2e_timing.jsonl
"""

import argparse
import json
import os
import sys
from typing import Dict, List

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_LOG = os.path.join(_PROJECT_ROOT, "logs", "e2e_timing.jsonl")


def load_entries(path):
    # type: (str) -> List[Dict]
    """JSONL ファイルから全エントリを読み込む"""
    entries = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                entries.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return entries


def percentile(values, p):
    # type: (List[float], float) -> float
    """簡易パーセンタイル計算 (numpy 不要)"""
    if not values:
        return 0.0
    sorted_v = sorted(values)
    k = (len(sorted_v) - 1) * (p / 100.0)
    f = int(k)
    c = f + 1
    if c >= len(sorted_v):
        return sorted_v[-1]
    d = k - f
    return sorted_v[f] + d * (sorted_v[c] - sorted_v[f])


def analyze(entries):
    # type: (List[Dict]) -> None
    """ルート別に E2E 統計を算出して表示"""
    # 全体統計
    all_e2e = [e["e2e_ms"] for e in entries if "e2e_ms" in e]
    all_asr = [e["asr_ms"] for e in entries if e.get("asr_ms", 0) > 0]
    all_brain = [e["brain_ms"] for e in entries if "brain_ms" in e]

    print("=" * 65)
    print("E2E タイミング分析 — ポスター掲載用")
    print("=" * 65)
    print("データ数: {} エントリ".format(len(entries)))
    print()

    # 全体
    if all_e2e:
        mean_e2e = sum(all_e2e) / len(all_e2e)
        print("--- 全体 E2E (speech_end -> action_complete) ---")
        print("  N      = {}".format(len(all_e2e)))
        print("  Mean   = {:.0f} ms".format(mean_e2e))
        print("  P50    = {:.0f} ms".format(percentile(all_e2e, 50)))
        print("  P95    = {:.0f} ms".format(percentile(all_e2e, 95)))
        print("  Min    = {:.0f} ms".format(min(all_e2e)))
        print("  Max    = {:.0f} ms".format(max(all_e2e)))
        print()

    # ASR のみ (emergency 除外: asr_ms > 0)
    if all_asr:
        print("--- ASR 推論時間 ---")
        print("  N      = {}".format(len(all_asr)))
        print("  Mean   = {:.0f} ms".format(sum(all_asr) / len(all_asr)))
        print("  P50    = {:.0f} ms".format(percentile(all_asr, 50)))
        print("  P95    = {:.0f} ms".format(percentile(all_asr, 95)))
        print()

    # Brain のみ
    if all_brain:
        print("--- Brain 処理+実行時間 ---")
        print("  N      = {}".format(len(all_brain)))
        print("  Mean   = {:.0f} ms".format(sum(all_brain) / len(all_brain)))
        print("  P50    = {:.0f} ms".format(percentile(all_brain, 50)))
        print("  P95    = {:.0f} ms".format(percentile(all_brain, 95)))
        print()

    # ルート別
    routes = {}  # type: Dict[str, List[Dict]]
    for e in entries:
        r = e.get("route", "unknown")
        # route から主要カテゴリを抽出
        if "emergency" in r:
            cat = "emergency"
        elif "hotpath" in r or "hot_cache" in r:
            cat = "hotpath"
        elif "7B" in r or "llm" in r.lower() or "action" in r:
            cat = "llm"
        elif "conversational" in r:
            cat = "conversational"
        else:
            cat = r or "other"
        routes.setdefault(cat, []).append(e)

    print("--- ルート別 E2E ---")
    print("{:<16} {:>5} {:>8} {:>8} {:>8}".format(
        "Route", "N", "Mean", "P50", "P95"))
    print("-" * 50)

    for cat in ["emergency", "hotpath", "llm", "conversational", "other"]:
        if cat not in routes:
            continue
        e2e_vals = [e["e2e_ms"] for e in routes[cat] if "e2e_ms" in e]
        if not e2e_vals:
            continue
        print("{:<16} {:>5} {:>7.0f}ms {:>7.0f}ms {:>7.0f}ms".format(
            cat,
            len(e2e_vals),
            sum(e2e_vals) / len(e2e_vals),
            percentile(e2e_vals, 50),
            percentile(e2e_vals, 95),
        ))

    print()
    print("--- ポスター掲載用 (推奨フォーマット) ---")
    print()

    # ポスター掲載用サマリー
    hotpath_e2e = [e["e2e_ms"] for e in entries
                   if "e2e_ms" in e and ("hotpath" in e.get("route", "") or "hot_cache" in e.get("route", ""))]
    llm_e2e = [e["e2e_ms"] for e in entries
               if "e2e_ms" in e and ("7B" in e.get("route", "") or "action" in e.get("route", "") or "llm" in e.get("route", "").lower())]
    emg_e2e = [e["e2e_ms"] for e in entries
               if "e2e_ms" in e and "emergency" in e.get("route", "")]

    if hotpath_e2e:
        print("  ホットパス E2E: 中央値 {:.1f}s (N={})".format(
            percentile(hotpath_e2e, 50) / 1000, len(hotpath_e2e)))
    if llm_e2e:
        print("  LLM経路 E2E: 中央値 {:.1f}s (N={})".format(
            percentile(llm_e2e, 50) / 1000, len(llm_e2e)))
    if emg_e2e:
        print("  緊急停止 E2E: 中央値 {:.1f}ms (N={})".format(
            percentile(emg_e2e, 50), len(emg_e2e)))

    # コマンド別詳細
    print()
    print("--- コマンド別 E2E (全エントリ) ---")
    print("{:<20} {:>8} {:>10} {:>8}".format(
        "Command", "E2E(ms)", "Route", "Status"))
    print("-" * 50)
    for e in entries:
        cmd = e.get("command", "?")
        if len(cmd) > 18:
            cmd = cmd[:18] + ".."
        print("{:<20} {:>7.0f}ms {:>10} {:>8}".format(
            cmd,
            e.get("e2e_ms", 0),
            (e.get("route", "?"))[:10],
            e.get("execution_status", "?")[:8],
        ))


def main():
    # type: () -> None
    parser = argparse.ArgumentParser(description="E2E Timing Analysis")
    parser.add_argument(
        "--file", "-f",
        default=DEFAULT_LOG,
        help="E2E timing JSONL path (default: logs/e2e_timing.jsonl)",
    )
    args = parser.parse_args()

    if not os.path.exists(args.file):
        print("E2E ログファイルが見つかりません: {}".format(args.file))
        print()
        print("voice_commander.py を実行してコマンドを入力すると自動生成されます:")
        print("  python3 voice_commander.py --asr-mock")
        sys.exit(1)

    entries = load_entries(args.file)
    if not entries:
        print("E2E ログが空です: {}".format(args.file))
        sys.exit(1)

    analyze(entries)


if __name__ == "__main__":
    main()
