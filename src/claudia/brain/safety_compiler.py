#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Safety Compiler -- Single entry point for all safety checks

Input: Candidate action (single or sequence) + robot state
Output: SafetyVerdict (executable sequence / rejected / downgraded)

Pipeline:
  1. Whitelist validation -- illegal api_code directly rejected
  2. High-risk policy gate -- unconditionally reject HIGH_ENERGY_ACTIONS when allow_high_risk=False
  3. Battery gate -- <=0.10: safe actions only, <=0.20: no high-energy, <=0.30: downgrade high-energy to Dance
  4. Standing prerequisite -- requires standing but not standing -> insert StandUp(1004) as needed (supports mid-sequence insertion)
  5. Sequence truncation -- first rejected=entire rejection, mid rejected=truncate, downgrade=replace

Notes:
  - Battery level uses 0.0-1.0 normalized format throughout, not 0-100 percentage
  - Clock domain: snapshot_timestamp uses time.monotonic() (unaffected by NTP jumps)
"""

import time
from dataclasses import dataclass, field
from typing import List, Optional

from .action_registry import (
    EXECUTABLE_API_CODES, REQUIRE_STANDING, HIGH_ENERGY_ACTIONS,
)


@dataclass
class SafetyVerdict:
    """Safety compilation result"""
    executable_sequence: List[int] = field(default_factory=list)
    auto_prepend: List[int] = field(default_factory=list)  # Auto-prepended actions (e.g., StandUp)
    rejected: List[int] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    is_blocked: bool = False           # Entire request rejected
    block_reason: str = ""
    response_override: Optional[str] = None  # Safe response when rejected


class SafetyCompiler:
    """Unified safety pipeline -- single entry point for all safety checks

    Replaces the old SafetyValidator + _quick_safety_precheck + _final_safety_gate scattered logic.
    """

    # Safe action whitelist (allowed at very low battery)
    SAFE_ACTIONS = frozenset([1001, 1002, 1003, 1004, 1005, 1006, 1009, 1010])
    # Actions that change posture state (for in-sequence virtual posture advancement)
    # True=standing after execution, False=not standing after execution
    POSTURE_TRANSITIONS = {
        1004: True,   # StandUp
        1006: True,   # RecoveryStand
        1010: True,   # RiseSit
        1005: False,  # StandDown
        1009: False,  # Sit
    }

    def __init__(self, downgrade_target=1023, snapshot_max_age=5.0,
                 allow_high_risk=False):
        # type: (int, float, bool) -> None
        """Initialize safety compiler

        Args:
            downgrade_target: Downgrade target for high-energy actions (default Dance2=1023, tunable)
            snapshot_max_age: Maximum state snapshot age (seconds), fail-safe for high-risk actions if exceeded
            allow_high_risk: Whether to allow high-risk actions (FrontFlip/FrontJump/FrontPounce).
                Default False, consistent with old SafetyValidator(enable_high_risk_actions=False) behavior.
                Can be enabled via SAFETY_ALLOW_HIGH_RISK=1 environment variable.
        """
        self.downgrade_target = downgrade_target
        self.snapshot_max_age = snapshot_max_age
        self.allow_high_risk = allow_high_risk

    def compile(
        self,
        actions,           # type: List[int]
        battery_level,     # type: float
        is_standing,       # type: bool
        snapshot_timestamp=None,  # type: Optional[float]
    ):
        # type: (...) -> SafetyVerdict
        """Compile action sequence into safe executable version

        Args:
            actions: Candidate action list (single action also wrapped as [code])
            battery_level: Current battery level (0.0-1.0 normalized)
            is_standing: Whether currently standing
            snapshot_timestamp: State snapshot timestamp (time.monotonic()), None = skip freshness check
        """
        verdict = SafetyVerdict()

        # === Step 0a: Input contract -- battery hard validation ===
        if battery_level is None or not isinstance(battery_level, (int, float)):
            verdict.is_blocked = True
            verdict.block_reason = "battery_level missing or type error"
            verdict.response_override = "State data is invalid. Action stopped for safety."
            verdict.warnings.append("battery_level={!r} is invalid".format(battery_level))
            return verdict

        if battery_level > 1.0:
            # No auto /100 correction -- auto-correction would mask upstream normalization bugs
            verdict.is_blocked = True
            verdict.block_reason = (
                "battery_level={} > 1.0, exceeds valid range [0.0, 1.0]".format(battery_level)
            )
            verdict.response_override = "Battery data is abnormal. Action stopped for safety."
            verdict.warnings.append(
                "battery_level={} not in [0.0, 1.0] range, "
                "check _normalize_battery() or state_monitor output".format(battery_level)
            )
            return verdict

        if battery_level < 0.0:
            battery_level = 0.0
            verdict.warnings.append("battery_level < 0, corrected to 0.0")

        # === Step 0b: State freshness check ===
        if snapshot_timestamp is not None:
            age = time.monotonic() - snapshot_timestamp
            if age > self.snapshot_max_age:
                verdict.warnings.append(
                    "State snapshot expired ({:.1f}s > {:.1f}s)".format(age, self.snapshot_max_age)
                )
                safe_only = [a for a in actions if a in self.SAFE_ACTIONS]
                if safe_only != actions:
                    verdict.warnings.append("Only safe actions allowed with expired state")
                    actions = safe_only
                    if not actions:
                        verdict.is_blocked = True
                        verdict.block_reason = "State data expired, high-risk actions rejected"
                        verdict.response_override = (
                            "State data is stale, only safe actions can be executed."
                        )
                        return verdict

        # === Steps 1-5: Per-item compilation ===
        # Key: posture changes within a sequence, cannot use input is_standing once.
        # virtual_standing advances after each action via POSTURE_TRANSITIONS.
        virtual_standing = bool(is_standing)
        for api_code in actions:
            # Step 1: Whitelist
            if api_code not in EXECUTABLE_API_CODES:
                verdict.rejected.append(api_code)
                verdict.warnings.append("API {} not in whitelist".format(api_code))
                if not verdict.executable_sequence:
                    # First action illegal -> entire request rejected
                    verdict.is_blocked = True
                    verdict.block_reason = "First action {} is illegal".format(api_code)
                    verdict.response_override = "Action stopped for safety."
                    return verdict
                # Mid-sequence illegal -> truncate
                verdict.warnings.append(
                    "Sequence truncated: keeping {}".format(verdict.executable_sequence)
                )
                break

            # Step 2: High-risk policy gate (consistent with old SafetyValidator behavior)
            if not self.allow_high_risk and api_code in HIGH_ENERGY_ACTIONS:
                verdict.rejected.append(api_code)
                verdict.warnings.append(
                    "High-risk action {} blocked by policy (allow_high_risk=False)".format(api_code)
                )
                if not verdict.executable_sequence:
                    verdict.is_blocked = True
                    verdict.block_reason = (
                        "High-risk actions are currently disabled (api_code={})".format(
                            api_code
                        )
                    )
                    verdict.response_override = (
                        "High-risk actions are currently disabled."
                    )
                    return verdict
                # Mid-sequence hit -> truncate
                verdict.warnings.append(
                    "Sequence truncated: high-risk action {} blocked by policy".format(api_code)
                )
                break

            # Step 3: Battery gate
            gate_result = self._battery_gate(api_code, battery_level)
            if gate_result == "reject":
                verdict.rejected.append(api_code)
                if not verdict.executable_sequence:
                    verdict.is_blocked = True
                    verdict.block_reason = (
                        "Battery {:.0%} insufficient, {} rejected".format(battery_level, api_code)
                    )
                    verdict.response_override = (
                        "Battery is low, only safe actions can be executed."
                    )
                    return verdict
                verdict.warnings.append(
                    "Sequence truncated: {} rejected by battery gate".format(api_code)
                )
                break
            elif gate_result == "downgrade":
                verdict.warnings.append(
                    "API {} downgraded to {} (battery {:.0%})".format(
                        api_code, self.downgrade_target, battery_level
                    )
                )
                api_code = self.downgrade_target

            # Step 4: Standing prerequisite (in-sequence insertion as needed)
            if api_code in REQUIRE_STANDING and not virtual_standing:
                # Avoid adjacent duplicate StandUp insertion
                if not verdict.executable_sequence or verdict.executable_sequence[-1] != 1004:
                    insert_at_head = len(verdict.executable_sequence) == 0
                    verdict.executable_sequence.append(1004)
                    # Backward compatibility: only head insertion counts as auto_prepend
                    if insert_at_head and 1004 not in verdict.auto_prepend:
                        verdict.auto_prepend.append(1004)
                        verdict.warnings.append(
                            "Auto-prepended StandUp(1004): {} requires standing".format(api_code)
                        )
                    elif not insert_at_head:
                        verdict.warnings.append(
                            "Mid-sequence StandUp(1004) inserted: {} requires standing".format(api_code)
                        )
                virtual_standing = True

            verdict.executable_sequence.append(api_code)

            # Sequence posture advancement: let subsequent actions use "post-execution" posture
            if api_code in self.POSTURE_TRANSITIONS:
                virtual_standing = self.POSTURE_TRANSITIONS[api_code]

        return verdict

    def _battery_gate(self, api_code, battery_level):
        # type: (int, float) -> str
        """Battery gate

        Args:
            api_code: Action code to check
            battery_level: 0.0-1.0 normalized battery level

        Returns: "pass" | "reject" | "downgrade"
        """
        if battery_level <= 0.10:
            if api_code not in self.SAFE_ACTIONS:
                return "reject"
        elif battery_level <= 0.20:
            if api_code in HIGH_ENERGY_ACTIONS:
                return "reject"
        elif battery_level <= 0.30:
            if api_code in HIGH_ENERGY_ACTIONS:
                return "downgrade"
        return "pass"
