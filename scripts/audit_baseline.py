#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
audit_baseline.py -- PR2 Audit Log Baseline Analysis

Reads logs/audit/*.jsonl and outputs:
  - Route distribution (emergency/hotpath/7B/action_channel/...)
  - Action success rate / unknown rate / safety rejection rate
  - P50/P95 latency (grouped by route)
  - PR2 extension: grouped by router_mode, shadow agreement rate

Usage:
  python3 scripts/audit_baseline.py              # Default logs/audit/
  python3 scripts/audit_baseline.py --dir /path   # Specify directory
  python3 scripts/audit_baseline.py --min-n 100   # Minimum sample size
"""

import argparse
import json
import sys
from pathlib import Path
from collections import Counter, defaultdict


def load_entries(audit_dir):
    """Load all JSONL files"""
    entries = []
    audit_path = Path(audit_dir)
    if not audit_path.exists():
        print("Error: Audit directory does not exist: {}".format(audit_dir))
        sys.exit(1)

    for f in sorted(audit_path.glob("audit_*.jsonl")):
        with f.open('r', encoding='utf-8') as fh:
            for line_num, line in enumerate(fh, 1):
                line = line.strip()
                if not line:
                    continue
                try:
                    entries.append(json.loads(line))
                except json.JSONDecodeError:
                    print("Warning: {}:{} JSON parse failed".format(f.name, line_num))
    return entries


def percentile(sorted_values, p):
    """Calculate percentile"""
    if not sorted_values:
        return 0.0
    idx = int(len(sorted_values) * p / 100)
    idx = min(idx, len(sorted_values) - 1)
    return sorted_values[idx]


def analyze(entries, min_n):
    """Analyze audit logs"""
    if len(entries) < min_n:
        print("Insufficient samples: {} entries (minimum required {})".format(len(entries), min_n))
        print("Continuing analysis, but results may not be statistically significant.\n")

    print("=" * 60)
    print("Audit Log Baseline Analysis")
    print("Total entries: {}".format(len(entries)))
    print("=" * 60)

    # --- Route distribution ---
    route_counts = Counter(e.get("route", "unknown") for e in entries)
    print("\n--- Route Distribution ---")
    for route, count in route_counts.most_common():
        pct = count / len(entries) * 100
        print("  {:30s} {:5d} ({:5.1f}%)".format(route, count, pct))

    # --- Latency statistics by route ---
    print("\n--- Latency Statistics (ms) ---")
    route_latencies = defaultdict(list)
    for e in entries:
        route = e.get("route", "unknown")
        ms = e.get("elapsed_ms", 0)
        if ms > 0:
            route_latencies[route].append(ms)

    print("  {:30s} {:>8s} {:>8s} {:>8s} {:>6s}".format(
        "Route", "P50", "P95", "P99", "N"))
    for route in sorted(route_latencies.keys()):
        vals = sorted(route_latencies[route])
        p50 = percentile(vals, 50)
        p95 = percentile(vals, 95)
        p99 = percentile(vals, 99)
        print("  {:30s} {:8.1f} {:8.1f} {:8.1f} {:6d}".format(
            route, p50, p95, p99, len(vals)))

    # --- Safety verdicts ---
    verdicts = Counter(e.get("safety_verdict", "unknown") for e in entries)
    print("\n--- Safety Verdicts ---")
    for v, count in verdicts.most_common():
        pct = count / len(entries) * 100
        print("  {:30s} {:5d} ({:5.1f}%)".format(v, count, pct))

    # --- Pipeline success rate + action rate ---
    pipeline_ok = sum(1 for e in entries if e.get("success", False))
    action_entries = [e for e in entries
                      if e.get("api_code") is not None or e.get("sequence")]
    conversational = len(entries) - len(action_entries)
    rejected = sum(1 for e in entries
                   if str(e.get("safety_verdict", "")).startswith("rejected"))
    print("\n--- Pipeline Overview ---")
    print("  Total entries:       {}".format(len(entries)))
    print("  Pipeline success:    {} ({:.1f}%)".format(
        pipeline_ok, pipeline_ok / len(entries) * 100 if entries else 0))
    print("  With action:         {} ({:.1f}%)".format(
        len(action_entries),
        len(action_entries) / len(entries) * 100 if entries else 0))
    print("  Conversation only:   {} ({:.1f}%)".format(
        conversational,
        conversational / len(entries) * 100 if entries else 0))
    print("  Safety rejected:     {} ({:.1f}%)".format(
        rejected, rejected / len(entries) * 100 if entries else 0))

    # --- PR2: router_mode grouping ---
    mode_counts = Counter(e.get("router_mode", "none") for e in entries)
    has_pr2 = any(e.get("router_mode") is not None for e in entries)
    if has_pr2:
        print("\n--- PR2: Router Mode Distribution ---")
        for mode, count in mode_counts.most_common():
            pct = count / len(entries) * 100
            print("  {:30s} {:5d} ({:5.1f}%)".format(mode, count, pct))

    # --- PR2: Shadow agreement rate ---
    shadow_entries = [e for e in entries if e.get("shadow_comparison")]
    if shadow_entries:
        agreements = sum(
            1 for e in shadow_entries
            if e["shadow_comparison"].get("raw_agreement", False))
        high_risk = sum(
            1 for e in shadow_entries
            if e["shadow_comparison"].get("high_risk_divergence", False))
        # dual_status semantics: "ok"=normal, "timeout"=timed out, "error"=error, "invalid_output"=invalid output
        # Backward compatible with old logs: fall back to dual_api_code when dual_status is absent
        status_counts = Counter()
        for e in shadow_entries:
            sc = e["shadow_comparison"]
            ds = sc.get("dual_status")
            if ds is None:
                # Old log compatibility: infer from dual_api_code
                dac = sc.get("dual_api_code")
                if dac == "timeout":
                    ds = "timeout"
                elif dac == "error":
                    ds = "error"
                else:
                    ds = "ok"
            status_counts[ds] += 1

        print("\n--- PR2: Shadow Comparison ---")
        print("  Shadow entries: {}".format(len(shadow_entries)))
        print("  Raw agreement rate: {:.1f}%".format(
            agreements / len(shadow_entries) * 100))
        print("  High-risk divergences: {}".format(high_risk))
        print("  Dual status distribution:")
        for st, cnt in status_counts.most_common():
            print("    {:20s} {:5d} ({:5.1f}%)".format(
                st, cnt, cnt / len(shadow_entries) * 100))

        # PR2 Action channel latency (only for samples with dual_status=ok)
        action_ms = [
            e["shadow_comparison"].get("dual_ms", 0)
            for e in shadow_entries
            if isinstance(e["shadow_comparison"].get("dual_ms"), (int, float))
            and e["shadow_comparison"].get("dual_status", "ok") == "ok"
        ]
        if action_ms:
            action_ms.sort()
            print("\n  Action channel latency (dual_status=ok):")
            print("    P50: {:.1f}ms  P95: {:.1f}ms  N={}".format(
                percentile(action_ms, 50),
                percentile(action_ms, 95),
                len(action_ms)))

    print("\n" + "=" * 60)


def main():
    parser = argparse.ArgumentParser(description="Audit log baseline analysis")
    parser.add_argument("--dir", default="logs/audit",
                        help="Audit log directory")
    parser.add_argument("--min-n", type=int, default=100,
                        help="Minimum sample size for statistical significance")
    args = parser.parse_args()

    entries = load_entries(args.dir)
    if not entries:
        print("No audit log entries found")
        sys.exit(0)

    analyze(entries, args.min_n)


if __name__ == "__main__":
    main()
