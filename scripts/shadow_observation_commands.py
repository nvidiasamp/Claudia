#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
shadow_observation_commands.py -- Shadow Observation Batch Command Script

Sends commands in batch under BRAIN_ROUTER_MODE=shadow to accumulate audit data.
Target: 100+ LLM-routed commands (hot_cache commands are excluded from LLM routing statistics).

Usage:
  BRAIN_ROUTER_MODE=shadow python3 scripts/shadow_observation_commands.py
  BRAIN_ROUTER_MODE=shadow python3 scripts/shadow_observation_commands.py --hardware
  BRAIN_ROUTER_MODE=shadow python3 scripts/shadow_observation_commands.py --batch 1  # Run only batch 1
  BRAIN_ROUTER_MODE=shadow python3 scripts/shadow_observation_commands.py --dry-run   # Print only, do not execute

3 second interval between commands (LLM inference + VRAM swap cooldown), approx. 10-12 minutes total.
"""

import os
import sys
import asyncio
import time
import json
import argparse
from datetime import datetime

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

# Disable ROS2 state monitoring (avoid Jetson OOM / DDS initialization)
import claudia.brain.production_brain as pb_mod
pb_mod.STATE_MONITOR_AVAILABLE = False


# === Command list ===
# Format: (command, category, expected_behavior)
# category: "hot_cache" | "llm_action" | "llm_conversational" | "llm_sequence"
# expected_behavior: brief description of expected behavior
# Note: All Japanese/Chinese command strings are intentional robot command test inputs

COMMANDS = [
    # ============================================================
    # Batch 1: Hot cache control group (20 commands, ~0ms each)
    # These are excluded from LLM routing statistics, but verify
    # that hot_cache still works correctly under shadow mode
    # ============================================================
    ("座って", "hot_cache", "Sit(1009)"),
    ("立って", "hot_cache", "StandUp(1004)"),
    ("止まれ", "hot_cache", "Stop(1003)"),
    ("かわいい", "hot_cache", "Heart(1036)"),
    ("踊って", "hot_cache", "Dance1(1022)"),
    ("ちんちん", "hot_cache", "Scrape(1029)"),
    ("お辞儀", "hot_cache", "Scrape(1029)"),
    ("おすわり", "hot_cache", "Sit(1009)"),
    ("ストレッチ", "hot_cache", "Stretch(1017)"),
    ("ハート", "hot_cache", "Heart(1036)"),
    ("こんにちは", "hot_cache", "Hello(1016)"),
    ("おはよう", "hot_cache", "Hello(1016)"),
    ("バランス", "hot_cache", "Balance(1002)"),
    ("伏せる", "hot_cache", "StandDown(1005)"),
    ("チンチン", "hot_cache", "Scrape(1029)"),
    ("拜年", "hot_cache", "Scrape(1029)"),
    ("すごい", "hot_cache", "Heart(1036)"),
    ("停止", "hot_cache", "Stop(1003)"),
    ("ジャンプ", "hot_cache", "FrontJump(1031)"),
    ("比心", "hot_cache", "Heart(1036)"),

    # ============================================================
    # Batch 2: LLM action commands -- Basic actions (25 commands)
    # Natural expressions not in hot_cache, expecting LLM to map
    # to the correct api_code
    # ============================================================
    ("座りなさい", "llm_action", "expect Sit(1009)"),
    ("お座りして", "llm_action", "expect Sit(1009)"),
    ("しゃがんで", "llm_action", "expect Sit(1009)"),
    ("立ちなさい", "llm_action", "expect StandUp(1004)"),
    ("起きて", "llm_action", "expect StandUp(1004)"),
    ("立ち上がって", "llm_action", "expect StandUp(1004)"),
    ("止まりなさい", "llm_action", "expect Stop(1003)"),
    ("動かないで", "llm_action", "expect Stop(1003)"),
    ("ストップ", "llm_action", "expect Stop(1003)"),
    ("横になって", "llm_action", "expect StandDown(1005)"),
    ("寝てください", "llm_action", "expect StandDown(1005)"),
    ("伏せて", "llm_action", "expect StandDown(1005)"),
    ("休んで", "llm_action", "expect Sit(1009)/StandDown(1005)"),
    ("疲れたでしょう", "llm_action", "expect Sit(1009)/StandDown(1005)"),
    ("ゆっくりして", "llm_action", "expect Sit(1009)"),
    ("リラックスして", "llm_action", "expect Sit(1009)/StandDown(1005)"),
    ("体を起こして", "llm_action", "expect Recovery(1006)/StandUp(1004)"),
    ("回復して", "llm_action", "expect Recovery(1006)"),
    ("元に戻って", "llm_action", "expect Recovery(1006)/StandUp(1004)"),
    ("正しい姿勢に", "llm_action", "expect StandUp(1004)/Balance(1002)"),
    ("安定して", "llm_action", "expect Balance(1002)"),
    ("バランスを取って", "llm_action", "expect Balance(1002)"),
    ("しっかり立って", "llm_action", "expect StandUp(1004)"),
    ("ちゃんと座って", "llm_action", "expect Sit(1009)"),
    ("ゆっくり座って", "llm_action", "expect Sit(1009)"),

    # ============================================================
    # Batch 3: LLM action commands -- Performance & emotion (25 commands)
    # ============================================================
    ("ストレッチして", "llm_action", "expect Stretch(1017)"),
    ("体を伸ばして", "llm_action", "expect Stretch(1017)"),
    ("伸びをして", "llm_action", "expect Stretch(1017)"),
    ("踊りなさい", "llm_action", "expect Dance1(1022)/Dance2(1023)"),
    ("ダンスして", "llm_action", "expect Dance1(1022)/Dance2(1023)"),
    ("かっこよく踊って", "llm_action", "expect Dance2(1023)"),
    ("楽しく踊って", "llm_action", "expect Dance1(1022)"),
    ("挨拶して", "llm_action", "expect Hello(1016)"),
    ("手を振って", "llm_action", "expect Hello(1016)"),
    ("ハートして", "llm_action", "expect Heart(1036)"),
    ("ハートを見せて", "llm_action", "expect Heart(1036)"),
    ("愛してる", "llm_action", "expect Heart(1036)"),
    ("大好き", "llm_action", "expect Heart(1036)"),
    ("いい子だね", "llm_action", "expect Heart(1036)"),
    ("元気を出して", "llm_action", "expect Dance/Jump"),
    ("かっこいいポーズして", "llm_action", "expect Pose(1028)"),
    ("ポーズを取って", "llm_action", "expect Pose(1028)"),
    ("飛んで", "llm_action", "expect FrontJump(1031)/FrontFlip(1030)"),
    ("ジャンプして", "llm_action", "expect FrontJump(1031)"),
    ("前に跳んで", "llm_action", "expect FrontJump(1031)/FrontPounce(1032)"),
    ("転がって", "llm_action", "expect RiseSit(1010)/RollOver"),
    ("ゴロンして", "llm_action", "expect RiseSit(1010)"),
    ("お辞儀して", "llm_action", "expect Scrape(1029)"),
    ("頭を下げて", "llm_action", "expect Scrape(1029)"),
    ("腰振って", "llm_action", "expect WiggleHips(1033)"),

    # ============================================================
    # Batch 4: LLM action commands -- Chinese variants (15 commands)
    # ============================================================
    ("坐下", "llm_action", "expect Sit(1009)"),
    ("站起来", "llm_action", "expect StandUp(1004)"),
    ("趴下", "llm_action", "expect StandDown(1005)"),
    ("跳舞", "llm_action", "expect Dance1(1022)/Dance2(1023)"),
    ("伸个懒腰", "llm_action", "expect Stretch(1017)"),
    ("打个招呼", "llm_action", "expect Hello(1016)"),
    ("做个爱心", "llm_action", "expect Heart(1036)"),
    ("跳一下", "llm_action", "expect FrontJump(1031)"),
    ("翻个跟头", "llm_action", "expect FrontFlip(1030)"),
    ("恢复正常", "llm_action", "expect Recovery(1006)"),
    ("不要动", "llm_action", "expect Stop(1003)"),
    ("摆个姿势", "llm_action", "expect Pose(1028)"),
    ("你好棒", "llm_action", "expect Heart(1036)"),
    ("真厉害", "llm_action", "expect Heart(1036)"),
    ("休息一下", "llm_action", "expect Sit(1009)/StandDown(1005)"),

    # ============================================================
    # Batch 5: LLM conversational commands -- a=null path (20 commands)
    # ============================================================
    ("今日の天気は？", "llm_conversational", "expect a=null, conversational reply"),
    ("何時ですか？", "llm_conversational", "expect a=null, conversational reply"),
    ("あなたは誰？", "llm_conversational", "expect a=null, conversational reply"),
    ("名前は何ですか？", "llm_conversational", "expect a=null, self-introduction"),
    ("元気？", "llm_conversational", "expect a=null, acknowledgment"),
    ("今日は何をしますか？", "llm_conversational", "expect a=null, conversation"),
    ("好きな食べ物は？", "llm_conversational", "expect a=null, conversation"),
    ("何歳ですか？", "llm_conversational", "expect a=null, conversation"),
    ("お腹すいた？", "llm_conversational", "expect a=null, acknowledgment"),
    ("どこに住んでる？", "llm_conversational", "expect a=null, conversation"),
    ("Claudiaちゃん", "llm_conversational", "expect a=null, name call"),
    ("今日は暑いね", "llm_conversational", "expect a=null, weather small talk"),
    ("疲れた？", "llm_conversational", "expect a=null or Sit"),
    ("楽しい？", "llm_conversational", "expect a=null, acknowledgment"),
    ("ありがとう", "llm_conversational", "expect a=null or Heart"),
    ("你是谁？", "llm_conversational", "expect a=null, self-introduction"),
    ("今天天气怎么样？", "llm_conversational", "expect a=null, weather"),
    ("你叫什么名字？", "llm_conversational", "expect a=null, name"),
    ("你多大了？", "llm_conversational", "expect a=null, age"),
    ("你喜欢什么？", "llm_conversational", "expect a=null, conversation"),

    # ============================================================
    # Batch 6: LLM sequence commands -- Multiple actions (10 commands)
    # ============================================================
    ("立ってから座って", "llm_sequence", "expect [1004, 1009]"),
    ("座ってから立って", "llm_sequence", "expect [1009, 1004]"),
    ("立ってから挨拶して", "llm_sequence", "expect [1004, 1016]"),
    ("ストレッチしてからダンスして", "llm_sequence", "expect [1017, 1022/1023]"),
    ("起きてから踊って", "llm_sequence", "expect [1004, 1022]"),
    ("先にストレッチして、その後ダンスして", "llm_sequence", "expect [1017, 1022]"),
    ("立って、それからお辞儀して", "llm_sequence", "expect [1004, 1029]"),
    ("站起来然后跳舞", "llm_sequence", "expect [1004, 1022]"),
    ("先坐下再站起来", "llm_sequence", "expect [1009, 1004]"),
    ("伸展然后跳", "llm_sequence", "expect [1017, 1031]"),

    # ============================================================
    # Batch 7: Edge cases + noise (5 commands)
    # ============================================================
    ("", "edge_case", "empty input -> conversation or error"),
    ("あいうえお", "edge_case", "nonsensical -> a=null"),
    ("1234", "edge_case", "numeric input -> a=null"),
    ("asdfjkl", "edge_case", "random alphanumeric -> a=null"),
    ("ちんちんして立ってから踊って", "edge_case", "mix of hot_cache word + sequence command"),
]

# Batch ranges
BATCH_RANGES = {
    1: (0, 20),     # hot_cache control group
    2: (20, 45),    # LLM basic actions
    3: (45, 70),    # LLM performance
    4: (70, 85),    # LLM Chinese
    5: (85, 105),   # LLM conversational
    6: (105, 115),  # LLM sequence
    7: (115, 120),  # Edge cases
}


def count_by_category():
    """Statistics by category"""
    from collections import Counter
    cats = Counter(cat for _, cat, _ in COMMANDS)
    return cats


def create_brain(hardware=False):
    """Create brain instance for shadow mode"""
    os.environ["BRAIN_ROUTER_MODE"] = "shadow"
    from claudia.brain.production_brain import ProductionBrain
    brain = ProductionBrain(use_real_hardware=hardware)

    if not hardware:
        from types import SimpleNamespace
        mock_state = SimpleNamespace(
            battery_level=0.80,
            is_standing=True,
            is_moving=False,
            temperature=40.0,
            timestamp=time.monotonic(),
            source="shadow_observation",
            confidence=1.0,
            current_gait="unknown",
            network_status="unknown",
            sdk_connection=False,
        )

        def _get_current_state():
            # Avoid triggering stale-state safety rejection during long batch runs
            mock_state.timestamp = time.monotonic()
            return mock_state

        mock_monitor = SimpleNamespace(
            get_current_state=_get_current_state,
            is_ros_initialized=True,
        )
        brain.state_monitor = mock_monitor

    # In shadow mode, single GPU model switching takes ~30-45s/command
    # Default snapshot_max_age=5s would cause excessive false stale-state safety downgrades
    # Relaxed to 90s (still safe compared to real hardware failure threshold)
    if hasattr(brain, 'safety_compiler') and brain.safety_compiler:
        brain.safety_compiler.snapshot_max_age = 90.0

    return brain


def run_async(coro):
    loop = asyncio.new_event_loop()
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


def warmup_models(brain, router_mode="shadow"):
    """Preload models into VRAM (prevent cold start latency contamination on first command)

    In shadow mode, warmup in order: Action -> 7B.
    The last loaded model remains in VRAM, so warm up the
    primary path 7B last to maximize hit rate.
    """
    try:
        import subprocess
    except ImportError:
        print("  (subprocess not available, skipping warmup)")
        return

    models = []
    if router_mode == "shadow":
        # Shadow: Action (for observation) -> 7B (primary path, loaded last)
        action_model = getattr(brain, '_channel_router', None)
        if action_model:
            action_model = action_model._action_model
        else:
            action_model = os.environ.get("BRAIN_MODEL_ACTION", "claudia-action-v1")
        models = [
            (action_model, 1024, "Action"),
            (brain.model_7b, 2048, "7B"),
        ]
    else:
        models = [(brain.model_7b, 2048, "7B")]

    for model_name, num_ctx, label in models:
        t0 = time.monotonic()
        print("  Warmup: {} ({})...".format(label, model_name), end="", flush=True)
        try:
            result = subprocess.run(
                ["ollama", "run", model_name, '{"a":null}'],
                capture_output=True, text=True, timeout=60,
            )
            elapsed = (time.monotonic() - t0) * 1000
            if result.returncode == 0:
                print(" OK ({:.0f}ms)".format(elapsed))
            else:
                print(" WARN: returncode={} ({:.0f}ms)".format(
                    result.returncode, elapsed))
        except subprocess.TimeoutExpired:
            elapsed = (time.monotonic() - t0) * 1000
            print(" TIMEOUT ({:.0f}ms), continuing".format(elapsed))
        except Exception as e:
            print(" ERROR: {}, continuing".format(e))

    print()


def run_observation(brain, commands, delay=3.0):
    """Execute commands sequentially and record results"""
    results = []
    total = len(commands)

    for i, (cmd, cat, expected) in enumerate(commands, 1):
        if not cmd:
            # Skip empty input
            print("  [{}/{}] (skip empty)".format(i, total))
            continue

        t0 = time.monotonic()
        try:
            output = run_async(brain.process_and_execute(cmd))
            elapsed = (time.monotonic() - t0) * 1000

            api = output.api_code
            seq = getattr(output, 'sequence', None)
            resp = (output.response or "")[:30]
            exec_st = getattr(output, 'execution_status', None)

            status_icon = "A" if api else ("S" if seq else "C")
            result_str = "api={} seq={} exec={} resp='{}' {:.0f}ms".format(
                api, seq, exec_st, resp, elapsed)
            results.append((cmd, cat, True, result_str))

            print("  [{}/{}] [{}] {:20s} -> {}".format(
                i, total, status_icon, cmd[:20], result_str))

        except Exception as e:
            elapsed = (time.monotonic() - t0) * 1000
            results.append((cmd, cat, False, str(e)))
            print("  [{}/{}] [E] {:20s} -> ERROR: {} ({:.0f}ms)".format(
                i, total, cmd[:20], e, elapsed))

        # Inference interval (VRAM swap cooldown)
        if delay > 0:
            time.sleep(delay)

    return results


def main():
    parser = argparse.ArgumentParser(
        description="Shadow observation batch: 120+ commands for audit data accumulation")
    parser.add_argument("--hardware", action="store_true",
                        help="Real hardware connection mode")
    parser.add_argument("--batch", type=int, default=0,
                        help="Run only a specific batch (1-7, 0=all)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Display command list (do not execute)")
    parser.add_argument("--delay", type=float, default=3.0,
                        help="Wait seconds between commands (default: 3.0)")
    args = parser.parse_args()

    # Display statistics
    cats = count_by_category()
    llm_total = sum(v for k, v in cats.items() if k != "hot_cache")
    print("=" * 60)
    print("  Shadow Observation Batch")
    print("=" * 60)
    print("  Total commands: {}".format(len(COMMANDS)))
    print("  Non-hot_cache: {} (some may be caught at conversational/sequence stage)".format(
        llm_total))
    for cat, count in sorted(cats.items()):
        print("    {:25s} {}".format(cat, count))
    print("  * Actual router-routed count is determined from audit logs after execution".format())
    # Shadow single GPU: ~45s/LLM command (VRAM swap x2) + delay
    # Hot cache: ~0.5s/command + delay
    hot_count_all = sum(1 for _, c, _ in COMMANDS if c == "hot_cache")
    llm_count_all = len(COMMANDS) - hot_count_all
    est_min = (hot_count_all * (0.5 + args.delay) + llm_count_all * (45 + args.delay)) / 60
    print("  Estimated time: {:.0f} min (hot={}, llm={}, delay={}s)".format(
        est_min, hot_count_all, llm_count_all, args.delay))
    print("  Mode: {}".format("hardware" if args.hardware else "simulation"))
    print("  BRAIN_ROUTER_MODE: {}".format(
        os.environ.get("BRAIN_ROUTER_MODE", "(not set)")))
    print()

    # Batch selection
    if args.batch > 0:
        if args.batch not in BATCH_RANGES:
            print("ERROR: Batch {} does not exist (1-7)".format(args.batch))
            return 1
        start, end = BATCH_RANGES[args.batch]
        commands = COMMANDS[start:end]
        print("  Batch {}: commands[{}:{}] ({} commands)".format(
            args.batch, start, end, len(commands)))
    else:
        commands = COMMANDS
        print("  Running all batches ({} commands)".format(len(commands)))

    # Dry run
    if args.dry_run:
        print("\n--- Dry Run: Command List ---")
        for i, (cmd, cat, exp) in enumerate(commands, 1):
            print("  {:3d}. [{:20s}] {:25s}  -> {}".format(i, cat, cmd, exp))
        return 0

    # Execute
    print()
    brain = create_brain(hardware=args.hardware)

    # Cold start prevention: preload both models into VRAM
    router_mode = os.environ.get("BRAIN_ROUTER_MODE", "legacy")
    warmup_models(brain, router_mode=router_mode)

    start_time = datetime.now()
    print("  Start: {}".format(start_time.strftime("%Y-%m-%d %H:%M:%S")))
    print()

    results = run_observation(brain, commands, delay=args.delay)

    end_time = datetime.now()
    elapsed_total = (end_time - start_time).total_seconds()

    # Summary
    print("\n" + "=" * 60)
    print("  Completed: {} ({:.0f} seconds)".format(
        end_time.strftime("%H:%M:%S"), elapsed_total))
    print("=" * 60)

    # Finding #4 fix: execution_status classification
    no_exception = sum(1 for _, _, ok, _ in results if ok)
    exceptions = len(results) - no_exception
    print("  Completed (no exceptions): {}/{}".format(no_exception, len(results)))
    if exceptions > 0:
        print("  Exceptions occurred: {}".format(exceptions))

    # By category
    from collections import defaultdict, Counter as Ctr
    cat_stats = defaultdict(lambda: [0, 0])  # [no_exception, total]
    for _, cat, ok, _ in results:
        cat_stats[cat][1] += 1
        if ok:
            cat_stats[cat][0] += 1
    print("\n  By category:")
    for cat in sorted(cat_stats):
        s, t = cat_stats[cat]
        print("    {:25s} {}/{} ({:.0f}%)".format(cat, s, t, s / t * 100))

    # Finding #1/#3: read actual router-routed sample count from audit log
    run_start_iso = start_time.strftime("%Y-%m-%dT%H:%M:%S")
    print("\n  --- Audit Log Check (after this run) ---")
    from pathlib import Path
    audit_dir = Path("logs/audit")
    if audit_dir.exists():
        run_entries = []
        shadow_entries = []
        route_counts = Ctr()
        status_counts = Ctr()
        for f in sorted(audit_dir.glob("audit_*.jsonl"), reverse=True):
            with f.open('r', encoding='utf-8') as fh:
                for line in fh:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        entry = json.loads(line)
                        ts = entry.get("timestamp", "")
                        if ts < run_start_iso:
                            continue
                        run_entries.append(entry)
                        route_counts[entry.get("route", "?")] += 1
                        if entry.get("shadow_comparison"):
                            shadow_entries.append(entry)
                            ds = entry["shadow_comparison"].get("dual_status", "?")
                            status_counts[ds] += 1
                    except (json.JSONDecodeError, KeyError):
                        pass

        # Actual router-routed count = entries with shadow_comparison
        # hot_cache / emergency / conversational_detect have no shadow_comparison
        print("  Audit total entries: {}".format(len(run_entries)))
        print("  Route distribution:")
        for r, cnt in route_counts.most_common():
            print("    {:25s} {}".format(r, cnt))
        print("  Shadow comparison entries: {} (= actual router-routed count)".format(
            len(shadow_entries)))
        if shadow_entries:
            agreements = sum(
                1 for e in shadow_entries
                if e["shadow_comparison"].get("raw_agreement", False))
            print("  Agreement rate: {:.1f}% ({}/{})".format(
                agreements / len(shadow_entries) * 100,
                agreements, len(shadow_entries)))
            print("  Dual status distribution:")
            for st, cnt in status_counts.most_common():
                print("    {:20s} {}".format(st, cnt))

        # N>=100 check
        print("\n  N>=100 check: {} (router-routed shadow entries)".format(
            "PASS" if len(shadow_entries) >= 100 else
            "NOT YET ({}/100)".format(len(shadow_entries))))
    else:
        print("  (audit directory does not exist)")

    print("\n  Next step:")
    print("  python3 scripts/audit_baseline.py --min-n 100")

    return 1 if exceptions > 0 else 0


if __name__ == "__main__":
    sys.exit(main())
