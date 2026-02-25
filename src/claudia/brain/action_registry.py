#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Action Registry -- Single source of truth for robot action definitions

All action properties are defined here; downstream collections (whitelist,
standing requirements, method mappings, etc.) are derived automatically.
To add/remove/modify actions, simply edit the _ACTIONS list.
"""

from dataclasses import dataclass
from typing import Dict, FrozenSet, Optional, Tuple


@dataclass(frozen=True)
class ActionDef:
    """Robot action definition -- single source of truth for all properties"""
    api_code: int
    method: str             # SportClient method name
    name_ja: str            # Japanese display name (for TTS/response templates)
    requires_standing: bool = False
    risk_level: str = "safe"   # "safe" | "medium" | "high"
    has_params: bool = False   # True = requires parameters, excluded from LLM whitelist
    enabled: bool = True       # False = globally disabled (SDK not implemented, etc.)
    safe_default_params: Optional[Tuple] = None  # Safe default params (for hot cache/direct execution only)


# === Action Registry (single definition point) ===
_ACTIONS = [
    # --- Basic Postures (no parameters) ---
    ActionDef(1001, "Damp",          "ダンプモード"),
    ActionDef(1002, "BalanceStand",   "バランスします"),
    ActionDef(1003, "StopMove",       "止まります"),
    ActionDef(1004, "StandUp",        "立ちます"),
    # StandDown can only execute from Standing (returns -1 / ignored from Sit/Damp)
    ActionDef(1005, "StandDown",      "伏せます",    requires_standing=True),
    ActionDef(1006, "RecoveryStand",  "回復します"),
    # Sit requires standing prerequisite (policy constraint):
    # Automatically prepends StandUp from untrusted states to avoid executing Sit from unknown postures.
    ActionDef(1009, "Sit",            "座ります",     requires_standing=True),
    ActionDef(1010, "RiseSit",        "起き上がります"),

    # --- Performance Actions (no parameters, standing requirement annotated) ---
    ActionDef(1016, "Hello",      "挨拶します",     requires_standing=True),
    ActionDef(1017, "Stretch",    "伸びをします",   requires_standing=True),
    ActionDef(1021, "Wallow",     "転がります", enabled=False),  # Go2 firmware unsupported (3203)
    ActionDef(1022, "Dance1",     "ダンス1します",   requires_standing=True),
    ActionDef(1023, "Dance2",     "ダンス2します",   requires_standing=True),
    ActionDef(1029, "Scrape",     "刮ります",       requires_standing=True),
    ActionDef(1033, "WiggleHips", "腰を振ります",   requires_standing=True),
    ActionDef(1036, "Heart",      "ハートします",   requires_standing=True),

    # --- High-Risk Performance (no parameters, high energy consumption) ---
    ActionDef(1030, "FrontFlip",   "前転します",       requires_standing=True, risk_level="high"),
    ActionDef(1031, "FrontJump",   "ジャンプします",   requires_standing=True, risk_level="high"),
    ActionDef(1032, "FrontPounce", "飛びかかります",   requires_standing=True, risk_level="high"),

    # --- Parameterized Actions (enabled=True but has_params=True excludes from LLM whitelist) ---
    ActionDef(1007, "Euler",           "姿勢を調整します", has_params=True),
    ActionDef(1008, "Move",            "歩きます",         has_params=True, risk_level="medium"),
    ActionDef(1015, "SpeedLevel",      "速度を変えます",   has_params=True),
    ActionDef(1019, "ContinuousGait",  "歩行モード変更",   has_params=True),
    ActionDef(1027, "SwitchJoystick",  "操作切り替え",     has_params=True),
    ActionDef(1028, "Pose",            "ポーズします",     has_params=True, safe_default_params=(True,)),

    # --- High-Risk / Unverified (disabled) ---
    ActionDef(1042, "LeftFlip",  "横回転します", risk_level="high", enabled=False),
    ActionDef(1044, "BackFlip",  "バク転します", risk_level="high", enabled=False),
]

# === Indexes ===
ACTION_REGISTRY: Dict[int, ActionDef] = {a.api_code: a for a in _ACTIONS}

# === Auto-derived collections (downstream code uses these, not _ACTIONS directly) ===

# LLM-decidable action whitelist (no parameters + enabled)
# Purpose: LLM output parsing layer filter, blocks model from outputting parameterized actions
VALID_API_CODES: FrozenSet[int] = frozenset(
    a.api_code for a in _ACTIONS if a.enabled and not a.has_params
)

# Executable whitelist (includes parameterized actions with safe default params)
# Purpose: SafetyCompiler whitelist check, hot cache entry validation
# Rule: Non-parameterized actions + parameterized actions with safe_default_params
EXECUTABLE_API_CODES: FrozenSet[int] = frozenset(
    a.api_code for a in _ACTIONS
    if a.enabled and (not a.has_params or a.safe_default_params is not None)
)

# Safe default parameters for parameterized actions (only those with declared safe_default_params)
# Used by executor: method(*SAFE_DEFAULT_PARAMS[api_code])
SAFE_DEFAULT_PARAMS: Dict[int, Tuple] = {
    a.api_code: a.safe_default_params
    for a in _ACTIONS
    if a.enabled and a.has_params and a.safe_default_params is not None
}

# Actions requiring standing prerequisite
REQUIRE_STANDING: FrozenSet[int] = frozenset(
    a.api_code for a in _ACTIONS if a.requires_standing
)

# High energy consumption actions (for battery gating)
HIGH_ENERGY_ACTIONS: FrozenSet[int] = frozenset(
    a.api_code for a in _ACTIONS if a.risk_level == "high"
)

# Action model visible actions (excludes high-risk by default -- prevents decision noise)
# When allow_high_risk=False, SafetyCompiler rejects high-risk actions.
# If the modelfile includes these actions, the model "learns" it can select them,
# causing a select->reject->ineffective fallback loop.
ACTION_MODEL_API_CODES: FrozenSet[int] = frozenset(
    a.api_code for a in _ACTIONS
    if a.enabled and not a.has_params and a.risk_level != "high"
)

# api_code -> SportClient method name
METHOD_MAP: Dict[int, str] = {
    a.api_code: a.method for a in _ACTIONS if a.enabled
}

# api_code -> Japanese response template
ACTION_RESPONSES: Dict[int, str] = {
    a.api_code: a.name_ja for a in _ACTIONS if a.enabled
}


def get_response_for_action(api_code: int) -> str:
    """Get the Japanese response text for an action"""
    return ACTION_RESPONSES.get(api_code, "はい、わかりました")


def get_response_for_sequence(sequence: list) -> str:
    """Get the combined response for a sequence of actions"""
    responses = [ACTION_RESPONSES.get(c, "動作") for c in sequence]
    return "、".join(responses)


# === JSON Schema (for Action channel structured output) ===
ACTION_SCHEMA = {
    "type": "object",
    "oneOf": [
        {"properties": {"a": {"type": ["integer", "null"]}},
         "required": ["a"], "additionalProperties": False},
        {"properties": {"s": {"type": "array", "items": {"type": "integer"}, "minItems": 1}},
         "required": ["s"], "additionalProperties": False}
    ]
}


def _remove_suffix(s: str, suffix: str) -> str:
    """Python 3.8 compatible removesuffix (str.removesuffix is 3.9+)"""
    if s.endswith(suffix):
        return s[:-len(suffix)]
    return s


def generate_modelfile_action_block(include_high_risk=True) -> str:
    """Auto-generate the action list block for Modelfile from registry

    Purpose: Modelfile auto-generation / consistency validation
    Ensures the Modelfile action list always matches the registry

    Args:
        include_high_risk: True=VALID_API_CODES (for 7B model),
                          False=ACTION_MODEL_API_CODES (for Action model, excludes high-risk)
    """
    lines = []
    for a in sorted(_ACTIONS, key=lambda x: x.api_code):
        if not a.enabled or a.has_params:
            continue
        if not include_high_risk and a.risk_level == "high":
            continue
        short_name = _remove_suffix(a.name_ja, "します")
        lines.append(f"{a.api_code}={short_name}")
    return " ".join(lines)
