#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
smoke_test_three_modes.py -- PR2 Three-Mode Smoke Test

Runs a set of test commands under legacy/dual/shadow routing modes,
verifying hot_cache mapping, LLM routing, safety compiler, and other critical paths.

Fixes:
  - LLM timeout is judged as FAIL (not OK)
  - Uses process_and_execute() (atomic entry point, no deprecation warning)
  - Shadow comparison data read from audit log (not BrainOutput)
  - 'dance' classified as hot_cache (not LLM)

Usage:
  python3 scripts/smoke_test_three_modes.py
"""

import os
import sys
import asyncio
import time
import json
import glob as glob_mod
from pathlib import Path

# Ensure project path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

# Disable ROS2 state monitoring (avoid Jetson OOM / DDS initialization)
import claudia.brain.production_brain as pb_mod
pb_mod.STATE_MONITOR_AVAILABLE = False

# LLM timeout threshold (exceeding this value is considered a timeout FAIL)
LLM_TIMEOUT_THRESHOLD_MS = 28000


def create_brain(mode):
    """Create a brain instance with the specified routing mode"""
    os.environ["BRAIN_ROUTER_MODE"] = mode
    from claudia.brain.production_brain import ProductionBrain
    brain = ProductionBrain(use_real_hardware=False)

    # Finding #4 fix: use named attributes (not MagicMock) to avoid log source=<MagicMock...>
    from types import SimpleNamespace
    mock_state = SimpleNamespace(
        battery_level=0.80,
        is_standing=True,
        is_moving=False,
        temperature=40.0,
        timestamp=time.monotonic(),
        source="smoke_test",       # Explicitly identify data source
        confidence=1.0,
        current_gait="unknown",
        network_status="unknown",
        sdk_connection=False,
    )

    def _get_current_state():
        # Prevent safety compiler from misjudging simulated state as stale
        mock_state.timestamp = time.monotonic()
        return mock_state

    mock_monitor = SimpleNamespace(
        get_current_state=_get_current_state,
        is_ros_initialized=True,
    )
    brain.state_monitor = mock_monitor
    return brain


def run_async(coro):
    loop = asyncio.new_event_loop()
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


# === Test case definitions ===
# (command, expected_api_code, description, is_hotcache)
# expected_api_code: exact expected value (hot_cache), or "any_action" / "conversational" / "any" (LLM)
# Note: Japanese/Chinese command strings are intentional robot command test inputs
TEST_CASES = [
    # --- hot_cache bow phrases (all -> 1029) ---
    ("ちんちん", 1029, "chintin -> Scrape", True),
    ("チンチン", 1029, "chintin (katakana) -> Scrape", True),
    ("拜年", 1029, "bainian (Chinese) -> Scrape", True),
    ("お辞儀", 1029, "ojigi -> Scrape", True),
    ("礼", 1029, "rei -> Scrape", True),
    ("ちんちん！", 1029, "chintin! (punctuation) -> Scrape", True),

    # --- hot_cache basic commands ---
    ("座って", 1009, "suwatte -> Sit", True),
    ("立って", 1004, "tatte -> StandUp", True),
    ("止まれ", 1003, "tomare -> Stop", True),
    ("かわいい", 1036, "kawaii -> Heart", True),
    ("踊って", 1022, "odotte -> Dance1", True),  # 'dance' is in hot_cache, not LLM

    # --- LLM routed commands (depends on Ollama inference) ---
    # expected: "any_action"=expect api_code, "conversational"=expect no action, "any"=lenient
    ("座りなさい", "any_action", "suwarinasai -> LLM (expect Sit)", False),
    ("ストレッチして", "any_action", "stretch shite -> LLM (expect Stretch)", False),
    ("元気を出して", "any", "genki wo dashite -> LLM (expect action/conv)", False),
    ("今日の天気は？", "conversational", "kyou no tenki wa? -> LLM (expect conv)", False),

    # --- PR2: Modelfile alignment verification (new actions + parameterized rejection) ---
    ("転がって", "any_action", "korogatte -> LLM (expect 1021 Wallow)", False),
    ("腰を振って", "any_action", "koshi wo futte -> LLM (expect 1033 WiggleHips)", False),
    ("歩いて", "conversational", "aruite -> LLM (expect a:null parameterized rejection)", False),
]


def format_result(output, elapsed_ms):
    """Format a single result"""
    api = output.api_code
    seq = getattr(output, 'sequence', None)
    route = getattr(output, 'reasoning', '?')
    resp = output.response[:30] if output.response else ""
    exec_status = getattr(output, 'execution_status', None)
    if seq:
        return "seq={} route={} exec={} resp='{}' {:.0f}ms".format(
            seq, route, exec_status, resp, elapsed_ms)
    return "api={} route={} exec={} resp='{}' {:.0f}ms".format(
        api, route, exec_status, resp, elapsed_ms)


def judge_llm_result(output, expected, elapsed_ms):
    """Judge LLM routed command result

    Returns (passed, status_label)
    """
    # Timeout detection: exceeding threshold = FAIL
    if elapsed_ms > LLM_TIMEOUT_THRESHOLD_MS:
        return False, "TIMEOUT"

    # Crash detection: empty response
    if not output.response:
        return False, "EMPTY"

    # Semantic judgment
    if expected == "any_action":
        passed = output.api_code is not None or getattr(output, 'sequence', None)
        return passed, "PASS" if passed else "FAIL"
    elif expected == "conversational":
        passed = output.api_code is None
        return passed, "PASS" if passed else "WARN"
    else:  # "any"
        return True, "PASS"


def read_recent_audit_entries(n=50, after_ts=None):
    """Read the most recent n shadow_comparison entries from audit log

    Finding #3 fix: after_ts is an ISO timestamp string, only entries after this
    time are read to isolate this run's data and avoid historical session contamination.
    """
    audit_dir = Path("logs/audit")
    if not audit_dir.exists():
        return []
    entries = []
    for f in sorted(audit_dir.glob("audit_*.jsonl"), reverse=True):
        with f.open('r', encoding='utf-8') as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    entry = json.loads(line)
                    if not entry.get("shadow_comparison"):
                        continue
                    # Finding #3: time window filter
                    if after_ts and entry.get("timestamp", "") < after_ts:
                        continue
                    entries.append(entry)
                except json.JSONDecodeError:
                    pass
        if len(entries) >= n:
            break
    return entries[-n:]  # Last n entries


def run_smoke_test():
    modes = ["legacy", "dual", "shadow"]
    results = {}  # mode -> [(cmd, pass, detail)]

    # Finding #3: record the start time of this run to isolate audit entries
    from datetime import datetime
    run_start_ts = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")

    for mode in modes:
        print("\n" + "=" * 70)
        print("  MODE: {}".format(mode.upper()))
        print("=" * 70)

        brain = create_brain(mode)
        mode_results = []

        for cmd, expected, desc, is_hotcache in TEST_CASES:
            t0 = time.monotonic()
            try:
                # Use atomic entry point process_and_execute() (no deprecation warning)
                output = run_async(brain.process_and_execute(cmd))
                elapsed = (time.monotonic() - t0) * 1000
                detail = format_result(output, elapsed)

                if is_hotcache:
                    # hot_cache command: strict api_code check
                    passed = (output.api_code == expected)
                    # Special case: SafetyCompiler may insert StandUp
                    if not passed and output.sequence:
                        passed = (expected in output.sequence)
                    status = "PASS" if passed else "FAIL"
                else:
                    # LLM command: judge based on expected type and timeout
                    passed, status = judge_llm_result(output, expected, elapsed)

                mode_results.append((cmd, passed, detail))
                mark = "  [{}]".format(status)
                print("  {} {:25s} -> {}".format(mark, desc, detail))

            except Exception as e:
                elapsed = (time.monotonic() - t0) * 1000
                mode_results.append((cmd, False, "ERROR: {}".format(e)))
                print("  [ERROR] {:25s} -> {} ({:.0f}ms)".format(desc, e, elapsed))

        results[mode] = mode_results

    # === Shadow comparison: read from audit log ===
    print("\n" + "=" * 70)
    print("  SHADOW COMPARISON (from audit log)")
    print("=" * 70)

    shadow_entries = read_recent_audit_entries(50, after_ts=run_start_ts)
    if not shadow_entries:
        print("  (No shadow_comparison audit entries found)")
    else:
        agreements = sum(1 for e in shadow_entries
                         if e["shadow_comparison"].get("raw_agreement"))
        divergences = sum(1 for e in shadow_entries
                          if e["shadow_comparison"].get("high_risk_divergence"))
        # dual_status semantics: "ok"=normal, "timeout"=timed out, "error"=error, "invalid_output"=invalid output
        timeouts = sum(1 for e in shadow_entries
                       if e["shadow_comparison"].get("dual_status") == "timeout")
        errors = sum(1 for e in shadow_entries
                     if e["shadow_comparison"].get("dual_status") == "error")
        invalids = sum(1 for e in shadow_entries
                       if e["shadow_comparison"].get("dual_status") == "invalid_output")

        print("  Entries: {}".format(len(shadow_entries)))
        print("  Agreement rate: {:.1f}% ({}/{})".format(
            agreements / len(shadow_entries) * 100,
            agreements, len(shadow_entries)))
        print("  High-risk divergences: {}".format(divergences))
        print("  Action timeouts: {}".format(timeouts))
        print("  Action errors: {}".format(errors))
        print("  Invalid outputs:   {}".format(invalids))

        # Per-entry details (last 5 entries)
        print("\n  Last {} entries:".format(min(5, len(shadow_entries))))
        for e in shadow_entries[-5:]:
            sc = e["shadow_comparison"]
            print("    cmd='{}' legacy={} dual={} status={} agree={} diverge={}".format(
                e.get("input_command", "?")[:15],
                sc.get("legacy_api_code"),
                sc.get("dual_api_code"),
                sc.get("dual_status", "?"),
                sc.get("raw_agreement"),
                sc.get("high_risk_divergence"),
            ))

    # === Summary ===
    print("\n" + "=" * 70)
    print("  SUMMARY")
    print("=" * 70)
    for mode in modes:
        total = len(results[mode])
        passed = sum(1 for _, p, _ in results[mode] if p)
        failed = total - passed
        status = "ALL PASS" if failed == 0 else "{} FAILED".format(failed)
        print("  {:8s}: {}/{} ({})".format(mode.upper(), passed, total, status))

    # All passed?
    all_pass = all(
        p for mode in modes for _, p, _ in results[mode]
    )
    print("\n  Overall: {}".format("ALL PASS" if all_pass else "HAS FAILURES"))
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(run_smoke_test())
