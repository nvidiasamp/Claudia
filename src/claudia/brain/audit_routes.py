#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Canonical route field names for audit logs -- finalized once, no ad-hoc additions

All _log_audit() calls must use constants from this module for the route= parameter.
Enforced by runtime ValueError + AST-level CI test dual guarantee.
"""

# === PR1 Routes ===
ROUTE_EMERGENCY = "emergency"                      # Emergency stop
ROUTE_HOTPATH = "hotpath"                          # Hot cache hit
ROUTE_HOTPATH_REJECTED = "hotpath_safety_rejected"  # Hot cache safety rejection
ROUTE_SEQUENCE = "sequence_predefined"             # Predefined sequence
ROUTE_DANCE = "dance"                              # Dance random branch
ROUTE_CONVERSATIONAL = "conversational"            # Conversational detection
ROUTE_PRECHECK_REJECTED = "precheck_rejected"      # Precheck rejection
ROUTE_LLM_7B = "7B"                               # Legacy 7B single-channel

# === PR2 Routes (when BRAIN_ROUTER_MODE != legacy) ===
ROUTE_ACTION_CHANNEL = "action_channel"            # Dual-channel action channel execution
ROUTE_VOICE_CHANNEL = "voice_channel"              # Dual-channel voice channel (text-only)
ROUTE_SHADOW = "shadow"                            # Shadow mode logging
ROUTE_ACTION_FALLBACK = "action_fallback"          # Action channel failure fallback to legacy
ROUTE_STARTUP = "startup"                          # Wake animation (Commander startup)

# === All valid route values (for Go/No-Go statistics scripts) ===
ALL_ROUTES = frozenset([
    ROUTE_EMERGENCY, ROUTE_HOTPATH, ROUTE_HOTPATH_REJECTED,
    ROUTE_SEQUENCE, ROUTE_DANCE, ROUTE_CONVERSATIONAL,
    ROUTE_PRECHECK_REJECTED, ROUTE_LLM_7B,
    ROUTE_ACTION_CHANNEL, ROUTE_VOICE_CHANNEL,
    ROUTE_SHADOW, ROUTE_ACTION_FALLBACK,
    ROUTE_STARTUP,
])
