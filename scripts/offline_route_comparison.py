#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
offline_route_comparison.py -- Offline Model Decision Consistency Comparison

Directly compares decision consistency between Action model and 7B model
outside the execution pipeline. Since it does not go through SafetyCompiler /
state snapshots / execution chain, only pure model decision quality is evaluated.

How it works:
  1. Call the Action model for each command (load into VRAM)
  2. Call the 7B model for the same command (VRAM swap)
  3. Compare api_code / sequence from both
  4. Log results to JSONL

Features:
  - No execution chain used (bypasses SafetyCompiler / BrainOutput)
  - Sequential execution assuming single GPU (avoids VRAM contention)
  - Records measured latency for each call (including VRAM swap)
  - Skips hot_cache commands as they are irrelevant for LLM comparison

Usage:
  python3 scripts/offline_route_comparison.py
  python3 scripts/offline_route_comparison.py --timeout 60
  python3 scripts/offline_route_comparison.py --output results/comparison.jsonl
  python3 scripts/offline_route_comparison.py --batch 2
  python3 scripts/offline_route_comparison.py --dry-run
"""

import os
import sys
import json
import time
import argparse
import subprocess
from datetime import datetime

# === Command list ===
# Share non-hot_cache commands from shadow_observation_commands.py
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from claudia.brain.action_registry import VALID_API_CODES, HIGH_ENERGY_ACTIONS, ACTION_SCHEMA

# Import commands directly from shadow_observation_commands.py
# Excluding hot_cache (not subject to LLM comparison)
try:
    from scripts.shadow_observation_commands import COMMANDS as ALL_COMMANDS, BATCH_RANGES
except ImportError:
    # Fallback: load directly from file path
    _script_dir = os.path.dirname(os.path.abspath(__file__))
    _obs_path = os.path.join(_script_dir, 'shadow_observation_commands.py')
    _ns = {}
    with open(_obs_path) as _f:
        _code = compile(_f.read(), _obs_path, 'exec')
        _globals = {"__file__": _obs_path, "__name__": "_shadow_obs"}
        exec(_code, _globals)  # noqa: S102 -- loading sibling script, not user input
    ALL_COMMANDS = _globals['COMMANDS']
    BATCH_RANGES = _globals['BATCH_RANGES']

# LLM target commands only, excluding hot_cache
LLM_COMMANDS = [(cmd, cat, exp) for cmd, cat, exp in ALL_COMMANDS if cat != "hot_cache"]

# Model configuration
MODEL_ACTION = os.environ.get("BRAIN_MODEL_ACTION", "claudia-action-v1")
MODEL_7B = os.environ.get("BRAIN_MODEL_7B", "claudia-7b:v2.0")


def call_ollama(model, command, num_predict=100, num_ctx=2048, timeout=60,
                output_format='json'):
    """Direct call to Ollama (using Python ollama library)

    Returns:
        (parsed_dict, latency_ms, status)
        status: "ok" | "timeout" | "error" | "parse_error"

    Args:
        output_format: 'json' = arbitrary JSON, dict = JSON Schema structured output
    """
    try:
        import ollama as ollama_lib
    except ImportError:
        return _call_ollama_subprocess(model, command, num_predict, num_ctx, timeout)

    t0 = time.monotonic()
    try:
        response = ollama_lib.chat(
            model=model,
            messages=[{'role': 'user', 'content': command}],
            format=output_format,
            options={
                'temperature': 0.0,
                'num_predict': num_predict,
                'num_ctx': num_ctx,
                'top_p': 0.9,
            }
        )
        latency = (time.monotonic() - t0) * 1000
        content = response['message']['content']
        parsed = json.loads(content)
        return (parsed, latency, "ok")

    except Exception as e:
        latency = (time.monotonic() - t0) * 1000
        err_type = type(e).__name__
        if "timeout" in err_type.lower() or latency > timeout * 1000:
            return (None, latency, "timeout")
        if "json" in err_type.lower() or "JSON" in str(e):
            return (None, latency, "parse_error")
        return (None, latency, "error")


def _call_ollama_subprocess(model, command, num_predict, num_ctx, timeout):
    """Fallback: Ollama call via subprocess"""
    t0 = time.monotonic()
    try:
        result = subprocess.run(
            ["ollama", "run", model, command],
            capture_output=True, text=True, timeout=timeout,
        )
        latency = (time.monotonic() - t0) * 1000
        if result.returncode != 0:
            return (None, latency, "error")
        parsed = json.loads(result.stdout.strip())
        return (parsed, latency, "ok")
    except subprocess.TimeoutExpired:
        latency = (time.monotonic() - t0) * 1000
        return (None, latency, "timeout")
    except json.JSONDecodeError:
        latency = (time.monotonic() - t0) * 1000
        return (None, latency, "parse_error")
    except Exception:
        latency = (time.monotonic() - t0) * 1000
        return (None, latency, "error")


def extract_decision(parsed, model_type="7b"):
    """Extract decision (api_code, sequence) from model output

    Action model: {"a": N} or {"s": [...]} or {"a": null}
    7B model: {"r": "...", "a": N} or {"r": "...", "s": [...]}
    """
    if parsed is None:
        return (None, None)

    api_code = parsed.get("api_code") or parsed.get("a")
    sequence = parsed.get("sequence") or parsed.get("s")

    # a=null means "conversation only"
    if api_code is None and sequence is None:
        return (None, None)

    return (api_code, sequence)


def compare_decisions(action_decision, legacy_decision):
    """Compare two decisions

    Returns:
        dict with comparison results
    """
    action_api, action_seq = action_decision
    legacy_api, legacy_seq = legacy_decision

    # raw_agreement: exact match of api_code and sequence
    raw_agreement = (action_api == legacy_api and action_seq == legacy_seq)

    # semantic_agreement: the "meaning" of actions matches
    # (same api_code, or same actions within sequence)
    action_codes = set()
    if action_api is not None:
        action_codes.add(action_api)
    if action_seq:
        action_codes.update(action_seq)

    legacy_codes = set()
    if legacy_api is not None:
        legacy_codes.add(legacy_api)
    if legacy_seq:
        legacy_codes.update(legacy_seq)

    # Both null = both conversational = agreement
    if not action_codes and not legacy_codes:
        semantic_agreement = True
    elif action_codes == legacy_codes:
        semantic_agreement = True
    else:
        semantic_agreement = False

    # high_risk_divergence: one side has high-risk action, the other does not
    action_high = action_codes & HIGH_ENERGY_ACTIONS
    legacy_high = legacy_codes & HIGH_ENERGY_ACTIONS
    high_risk_divergence = (action_high != legacy_high)

    # null agreement: both null (judged as conversation)
    both_null = (action_api is None and action_seq is None
                 and legacy_api is None and legacy_seq is None)

    return {
        "raw_agreement": raw_agreement,
        "semantic_agreement": semantic_agreement,
        "high_risk_divergence": high_risk_divergence,
        "both_null": both_null,
        "action_api": action_api,
        "action_seq": action_seq,
        "legacy_api": legacy_api,
        "legacy_seq": legacy_seq,
    }


def warmup_models(timeout=60):
    """Preload models into VRAM (Action then 7B, so 7B remains in VRAM last)"""
    models = [
        (MODEL_ACTION, 1024, "Action"),
        (MODEL_7B, 2048, "7B"),
    ]
    for model_name, num_ctx, label in models:
        t0 = time.monotonic()
        print("  Warmup: {} ({})...".format(label, model_name), end="", flush=True)
        try:
            result = subprocess.run(
                ["ollama", "run", model_name, '{"a":null}'],
                capture_output=True, text=True, timeout=timeout,
            )
            elapsed = (time.monotonic() - t0) * 1000
            if result.returncode == 0:
                print(" OK ({:.0f}ms)".format(elapsed))
            else:
                print(" WARN: rc={} ({:.0f}ms)".format(result.returncode, elapsed))
        except subprocess.TimeoutExpired:
            elapsed = (time.monotonic() - t0) * 1000
            print(" TIMEOUT ({:.0f}ms)".format(elapsed))
        except Exception as e:
            elapsed = (time.monotonic() - t0) * 1000
            print(" ERROR: {} ({:.0f}ms)".format(e, elapsed))


def run_comparison(commands, output_path, timeout=60, delay=2.0):
    """Run offline comparison for all commands

    Order: For each command, Action then 7B (7B called last to remain in VRAM)
    """
    results = []
    n = len(commands)

    for i, (cmd, cat, expected) in enumerate(commands):
        idx = i + 1
        # Skip empty commands (edge case: empty string may be rejected by Ollama)
        if not cmd.strip():
            print("  [{}/{}] [{}] (empty) -> SKIP".format(idx, n, cat[0].upper()))
            results.append({
                "index": idx,
                "command": cmd,
                "category": cat,
                "expected": expected,
                "action_status": "skipped",
                "legacy_status": "skipped",
                "comparison": {"raw_agreement": True, "both_null": True,
                               "semantic_agreement": True, "high_risk_divergence": False},
            })
            continue

        # Step 1: Action model (VRAM swap in, ACTION_SCHEMA structured output)
        action_raw, action_ms, action_status = call_ollama(
            MODEL_ACTION, cmd,
            num_predict=30, num_ctx=1024, timeout=timeout,
            output_format=ACTION_SCHEMA,
        )

        # Step 2: 7B model (VRAM swap in, remains in VRAM)
        legacy_raw, legacy_ms, legacy_status = call_ollama(
            MODEL_7B, cmd,
            num_predict=100, num_ctx=2048, timeout=timeout,
        )

        # Extract decisions
        action_decision = extract_decision(action_raw, "action")
        legacy_decision = extract_decision(legacy_raw, "7b")

        # Compare
        if action_status == "ok" and legacy_status == "ok":
            comparison = compare_decisions(action_decision, legacy_decision)
        else:
            comparison = {
                "raw_agreement": False,
                "semantic_agreement": False,
                "high_risk_divergence": False,
                "both_null": False,
                "action_api": action_decision[0],
                "action_seq": action_decision[1],
                "legacy_api": legacy_decision[0],
                "legacy_seq": legacy_decision[1],
            }

        # Record result
        entry = {
            "timestamp": datetime.now().isoformat(),
            "index": idx,
            "command": cmd,
            "category": cat,
            "expected": expected,
            "action_raw": action_raw,
            "action_status": action_status,
            "action_ms": round(action_ms, 1),
            "legacy_raw": legacy_raw,
            "legacy_status": legacy_status,
            "legacy_ms": round(legacy_ms, 1),
            "comparison": comparison,
        }
        results.append(entry)

        # Append to JSONL immediately (crash resilience)
        with open(output_path, 'a') as f:
            f.write(json.dumps(entry, ensure_ascii=False) + '\n')

        # Display
        agree_mark = "=" if comparison["raw_agreement"] else "!"
        action_api_str = str(action_decision[0]) if action_decision[0] is not None else (
            str(action_decision[1]) if action_decision[1] else "null")
        legacy_api_str = str(legacy_decision[0]) if legacy_decision[0] is not None else (
            str(legacy_decision[1]) if legacy_decision[1] else "null")

        print("  [{}/{}] [{}] {} -> A:{} L:{} [{}] {:.0f}ms/{:.0f}ms".format(
            idx, n, cat[0].upper(),
            cmd[:20].ljust(20),
            action_api_str.ljust(6), legacy_api_str.ljust(6),
            agree_mark,
            action_ms, legacy_ms,
        ))

        # Delay (Ollama GPU memory cooldown)
        if i < n - 1:
            time.sleep(delay)

    return results


def print_summary(results):
    """Display results summary"""
    total = len(results)
    if total == 0:
        print("\n  No results.")
        return

    # Aggregate by status
    action_ok = sum(1 for r in results if r["action_status"] == "ok")
    legacy_ok = sum(1 for r in results if r["legacy_status"] == "ok")
    both_ok = sum(1 for r in results
                  if r["action_status"] == "ok" and r["legacy_status"] == "ok")

    # Aggregate comparison results (both_ok only)
    ok_results = [r for r in results
                  if r["action_status"] == "ok" and r["legacy_status"] == "ok"]
    n_ok = len(ok_results)

    raw_agree = sum(1 for r in ok_results if r["comparison"]["raw_agreement"])
    semantic_agree = sum(1 for r in ok_results if r["comparison"]["semantic_agreement"])
    both_null = sum(1 for r in ok_results if r["comparison"]["both_null"])
    high_risk_div = sum(1 for r in ok_results if r["comparison"]["high_risk_divergence"])

    # Aggregate by category
    by_cat = {}
    for r in ok_results:
        cat = r["category"]
        if cat not in by_cat:
            by_cat[cat] = {"total": 0, "raw_agree": 0, "semantic_agree": 0}
        by_cat[cat]["total"] += 1
        if r["comparison"]["raw_agreement"]:
            by_cat[cat]["raw_agree"] += 1
        if r["comparison"]["semantic_agreement"]:
            by_cat[cat]["semantic_agree"] += 1

    # Aggregate latency
    action_latencies = sorted([r["action_ms"] for r in ok_results])
    legacy_latencies = sorted([r["legacy_ms"] for r in ok_results])

    def percentile(arr, p):
        if not arr:
            return 0.0
        k = (len(arr) - 1) * p / 100
        f = int(k)
        c = f + 1 if f + 1 < len(arr) else f
        return arr[f] + (k - f) * (arr[c] - arr[f])

    print("\n" + "=" * 60)
    print("  Offline Route Comparison -- Summary")
    print("=" * 60)

    print("\n  Status:")
    print("    Total commands:     {}".format(total))
    print("    Action model OK:    {}/{}".format(action_ok, total))
    print("    Legacy model OK:    {}/{}".format(legacy_ok, total))
    print("    Both OK:            {}/{}".format(both_ok, total))

    if n_ok > 0:
        print("\n  Decision Agreement (N={}):".format(n_ok))
        print("    Raw agreement:      {}/{} ({:.1f}%)".format(
            raw_agree, n_ok, raw_agree / n_ok * 100))
        print("    Semantic agreement: {}/{} ({:.1f}%)".format(
            semantic_agree, n_ok, semantic_agree / n_ok * 100))
        print("    Both null (conv):   {}/{} ({:.1f}%)".format(
            both_null, n_ok, both_null / n_ok * 100))
        print("    High-risk diverge:  {}/{} ({:.1f}%)".format(
            high_risk_div, n_ok, high_risk_div / n_ok * 100))

        print("\n  By Category:")
        for cat in sorted(by_cat.keys()):
            d = by_cat[cat]
            print("    {:<22} raw={}/{} ({:.0f}%)  semantic={}/{} ({:.0f}%)".format(
                cat,
                d["raw_agree"], d["total"],
                d["raw_agree"] / d["total"] * 100 if d["total"] else 0,
                d["semantic_agree"], d["total"],
                d["semantic_agree"] / d["total"] * 100 if d["total"] else 0,
            ))

        print("\n  Latency (ms) -- includes VRAM swap:")
        print("    Action:  P50={:.0f}  P95={:.0f}  max={:.0f}".format(
            percentile(action_latencies, 50),
            percentile(action_latencies, 95),
            max(action_latencies) if action_latencies else 0,
        ))
        print("    Legacy:  P50={:.0f}  P95={:.0f}  max={:.0f}".format(
            percentile(legacy_latencies, 50),
            percentile(legacy_latencies, 95),
            max(legacy_latencies) if legacy_latencies else 0,
        ))

    # Disagreement list
    disagree = [r for r in ok_results if not r["comparison"]["raw_agreement"]]
    if disagree:
        print("\n  Disagreements ({})".format(len(disagree)))
        for r in disagree[:20]:  # Max 20 entries
            a_str = str(r["comparison"].get("action_api") or r["comparison"].get("action_seq") or "null")
            l_str = str(r["comparison"].get("legacy_api") or r["comparison"].get("legacy_seq") or "null")
            sem = "sem_ok" if r["comparison"]["semantic_agreement"] else "sem_NG"
            hr = " HIGH_RISK" if r["comparison"]["high_risk_divergence"] else ""
            print("    {} -> A:{} L:{} [{}]{}".format(
                r["command"][:25].ljust(25), a_str, l_str, sem, hr))

    print()


def main():
    parser = argparse.ArgumentParser(description="Offline route comparison: Action vs 7B")
    parser.add_argument("--timeout", type=int, default=60,
                        help="Per-model call timeout in seconds (default: 60)")
    parser.add_argument("--delay", type=float, default=2.0,
                        help="Delay between commands in seconds (default: 2.0)")
    parser.add_argument("--output", type=str, default=None,
                        help="Output JSONL path (default: logs/offline_comparison_YYYYMMDD_HHMMSS.jsonl)")
    parser.add_argument("--batch", type=int, default=None,
                        help="Run only this batch number (2-7, batch 1=hot_cache excluded)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Show commands without executing")
    parser.add_argument("--skip-warmup", action="store_true",
                        help="Skip model warmup")
    args = parser.parse_args()

    # Output destination
    if args.output:
        output_path = args.output
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        os.makedirs("logs", exist_ok=True)
        output_path = "logs/offline_comparison_{}.jsonl".format(ts)

    # Command filter
    if args.batch:
        if args.batch < 2 or args.batch > 7:
            print("Error: --batch must be 2-7 (batch 1 = hot_cache, excluded)")
            sys.exit(1)
        start, end = BATCH_RANGES[args.batch]
        commands = ALL_COMMANDS[start:end]
        # Re-exclude hot_cache (batch 1 is already excluded, but just in case)
        commands = [(c, cat, e) for c, cat, e in commands if cat != "hot_cache"]
    else:
        commands = LLM_COMMANDS

    # Header
    print("=" * 60)
    print("  Offline Route Comparison")
    print("=" * 60)
    print("  Action model: {}".format(MODEL_ACTION))
    print("  Legacy model: {}".format(MODEL_7B))
    print("  Commands:     {} (LLM-routed only)".format(len(commands)))
    print("  Timeout:      {}s per model call".format(args.timeout))
    print("  Delay:        {}s between commands".format(args.delay))
    print("  Output:       {}".format(output_path))

    # Category breakdown
    cats = {}
    for _, cat, _ in commands:
        cats[cat] = cats.get(cat, 0) + 1
    for cat in sorted(cats.keys()):
        print("    {:<22} {}".format(cat, cats[cat]))

    # Time estimate (each command: Action VRAM swap+inference + 7B VRAM swap+inference + delay)
    # Measured: Action ~15-25s, 7B ~15-25s = ~30-50s/command
    est_sec = len(commands) * (40 + args.delay)
    est_min = est_sec / 60
    print("  Est. time:    {:.0f} min".format(est_min))
    print()

    if args.dry_run:
        print("  [DRY RUN] Commands:")
        for i, (cmd, cat, exp) in enumerate(commands):
            print("    [{}/{}] [{}] {} -> {}".format(
                i + 1, len(commands), cat, cmd, exp))
        return

    # Warmup
    if not args.skip_warmup:
        warmup_models(timeout=args.timeout)

    print("\n  Start: {}\n".format(datetime.now().strftime("%Y-%m-%d %H:%M:%S")))

    results = run_comparison(commands, output_path, args.timeout, args.delay)

    print("\n  End: {}".format(datetime.now().strftime("%Y-%m-%d %H:%M:%S")))

    print_summary(results)

    print("  Results saved to: {}".format(output_path))


if __name__ == "__main__":
    main()
