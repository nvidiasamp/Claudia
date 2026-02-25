#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
E2E Timing Analysis Script -- Compute statistics for poster presentation

Reads logs/e2e_timing.jsonl and computes P50/Mean/P95 by route.
Analyzes the E2E logs automatically generated during voice_commander.py execution.

Usage:
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
    """Load all entries from a JSONL file"""
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
    """Simple percentile calculation (no numpy required)"""
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
    """Compute and display E2E statistics by route"""
    # Overall statistics
    all_e2e = [e["e2e_ms"] for e in entries if "e2e_ms" in e]
    all_asr = [e["asr_ms"] for e in entries if e.get("asr_ms", 0) > 0]
    all_brain = [e["brain_ms"] for e in entries if "brain_ms" in e]

    print("=" * 65)
    print("E2E Timing Analysis -- For Poster Presentation")
    print("=" * 65)
    print("Data count: {} entries".format(len(entries)))
    print()

    # Overall
    if all_e2e:
        mean_e2e = sum(all_e2e) / len(all_e2e)
        print("--- Overall E2E (speech_end -> action_complete) ---")
        print("  N      = {}".format(len(all_e2e)))
        print("  Mean   = {:.0f} ms".format(mean_e2e))
        print("  P50    = {:.0f} ms".format(percentile(all_e2e, 50)))
        print("  P95    = {:.0f} ms".format(percentile(all_e2e, 95)))
        print("  Min    = {:.0f} ms".format(min(all_e2e)))
        print("  Max    = {:.0f} ms".format(max(all_e2e)))
        print()

    # ASR only (excluding emergency: asr_ms > 0)
    if all_asr:
        print("--- ASR Inference Time ---")
        print("  N      = {}".format(len(all_asr)))
        print("  Mean   = {:.0f} ms".format(sum(all_asr) / len(all_asr)))
        print("  P50    = {:.0f} ms".format(percentile(all_asr, 50)))
        print("  P95    = {:.0f} ms".format(percentile(all_asr, 95)))
        print()

    # Brain only
    if all_brain:
        print("--- Brain Processing + Execution Time ---")
        print("  N      = {}".format(len(all_brain)))
        print("  Mean   = {:.0f} ms".format(sum(all_brain) / len(all_brain)))
        print("  P50    = {:.0f} ms".format(percentile(all_brain, 50)))
        print("  P95    = {:.0f} ms".format(percentile(all_brain, 95)))
        print()

    # By route
    routes = {}  # type: Dict[str, List[Dict]]
    for e in entries:
        r = e.get("route", "unknown")
        # Extract main category from route
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

    print("--- E2E by Route ---")
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
    print("--- Poster Presentation Format (Recommended) ---")
    print()

    # Poster presentation summary
    hotpath_e2e = [e["e2e_ms"] for e in entries
                   if "e2e_ms" in e and ("hotpath" in e.get("route", "") or "hot_cache" in e.get("route", ""))]
    llm_e2e = [e["e2e_ms"] for e in entries
               if "e2e_ms" in e and ("7B" in e.get("route", "") or "action" in e.get("route", "") or "llm" in e.get("route", "").lower())]
    emg_e2e = [e["e2e_ms"] for e in entries
               if "e2e_ms" in e and "emergency" in e.get("route", "")]

    if hotpath_e2e:
        print("  Hot path E2E: median {:.1f}s (N={})".format(
            percentile(hotpath_e2e, 50) / 1000, len(hotpath_e2e)))
    if llm_e2e:
        print("  LLM route E2E: median {:.1f}s (N={})".format(
            percentile(llm_e2e, 50) / 1000, len(llm_e2e)))
    if emg_e2e:
        print("  Emergency stop E2E: median {:.1f}ms (N={})".format(
            percentile(emg_e2e, 50), len(emg_e2e)))

    # Per-command details
    print()
    print("--- E2E by Command (All Entries) ---")
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
        print("E2E log file not found: {}".format(args.file))
        print()
        print("Run voice_commander.py and enter commands to auto-generate:")
        print("  python3 voice_commander.py --asr-mock")
        sys.exit(1)

    entries = load_entries(args.file)
    if not entries:
        print("E2E log is empty: {}".format(args.file))
        sys.exit(1)

    analyze(entries)


if __name__ == "__main__":
    main()
