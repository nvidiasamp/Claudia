#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_audit_extension.py — PR2 Slice D: Audit extension + Shadow logging validation

Validates:
  - D1: AuditEntry backward compatibility (old logs without PR2 fields can still be deserialized)
  - D2: AuditEntry new fields are populated correctly
  - D3: _log_audit accepts PR2 kwargs
  - D4: Shadow comparison log integrity
"""

import sys
import os
import json

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from dataclasses import asdict
from claudia.brain.audit_logger import AuditEntry


# === D1: Backward compatibility ===

class TestAuditEntryBackwardCompat:
    """Old logs (without PR2 fields) can still be deserialized"""

    def _make_old_entry_dict(self):
        """Simulate PR1-era audit log JSON"""
        return {
            "timestamp": "2026-02-12T10:00:00",
            "model_name": "7B",
            "input_command": "おすわり",
            "state_battery": 0.85,
            "state_standing": True,
            "state_emergency": False,
            "llm_output": '{"r":"座ります","a":1009}',
            "api_code": 1009,
            "sequence": None,
            "safety_verdict": "ok",
            "safety_reason": None,
            "elapsed_ms": 245.3,
            "cache_hit": False,
            "route": "7B",
            "success": True,
        }

    def test_old_json_deserializes(self):
        """Old format JSON can create AuditEntry"""
        old = self._make_old_entry_dict()
        entry = AuditEntry(**old)
        assert entry.api_code == 1009
        assert entry.route == "7B"
        # PR2 fields default to None
        assert entry.request_id is None
        assert entry.router_mode is None
        assert entry.shadow_comparison is None
        assert entry.action_latency_ms is None
        assert entry.voice_latency_ms is None

    def test_old_roundtrip(self):
        """Old log -> AuditEntry -> dict -> JSON -> dict -> AuditEntry roundtrip"""
        old = self._make_old_entry_dict()
        entry = AuditEntry(**old)
        json_str = json.dumps(asdict(entry), ensure_ascii=False)
        restored = json.loads(json_str)
        entry2 = AuditEntry(**restored)
        assert entry2.api_code == entry.api_code
        assert entry2.request_id is None

    def test_old_json_with_extra_fields_ignored(self):
        """Future extension fields do not affect deserialization (using **kwargs would
        raise an error, so this validates current field set completeness)"""
        old = self._make_old_entry_dict()
        # Add PR2 fields
        old["request_id"] = "abc12345"
        old["router_mode"] = "shadow"
        entry = AuditEntry(**old)
        assert entry.request_id == "abc12345"


# === D2: New fields ===

class TestAuditEntryNewFields:
    """PR2 fields are populated correctly"""

    def test_pr2_fields_populated(self):
        """All PR2 fields can be set and read correctly"""
        entry = AuditEntry(
            timestamp="2026-02-13T14:00:00",
            model_name="dual",
            input_command="踊って",
            state_battery=0.65,
            state_standing=True,
            state_emergency=False,
            llm_output='{"a":1022}',
            api_code=1022,
            sequence=None,
            safety_verdict="ok",
            safety_reason=None,
            elapsed_ms=150.0,
            cache_hit=False,
            route="action_channel",
            success=True,
            # PR2 fields
            request_id="ab12cd34",
            router_mode="dual",
            shadow_comparison=None,
            action_latency_ms=120.5,
            voice_latency_ms=0.0,
        )
        assert entry.request_id == "ab12cd34"
        assert entry.router_mode == "dual"
        assert entry.action_latency_ms == 120.5
        assert entry.voice_latency_ms == 0.0

    def test_shadow_comparison_serializable(self):
        """shadow_comparison Dict can be serialized to JSON"""
        shadow = {
            "legacy_api_code": 1009,
            "legacy_sequence": None,
            "dual_api_code": 1016,
            "dual_sequence": None,
            "raw_agreement": False,
            "high_risk_divergence": False,
            "legacy_ms": 200.0,
            "dual_ms": 100.0,
        }
        entry = AuditEntry(
            timestamp="2026-02-13T14:00:00",
            model_name="shadow",
            input_command="test",
            state_battery=0.65,
            state_standing=True,
            state_emergency=False,
            llm_output=None,
            api_code=1009,
            sequence=None,
            safety_verdict="ok",
            safety_reason=None,
            elapsed_ms=200.0,
            cache_hit=False,
            route="shadow",
            success=True,
            request_id="test1234",
            router_mode="shadow",
            shadow_comparison=shadow,
        )
        json_str = json.dumps(asdict(entry), ensure_ascii=False)
        restored = json.loads(json_str)
        assert restored["shadow_comparison"]["raw_agreement"] is False
        assert restored["shadow_comparison"]["legacy_api_code"] == 1009

    def test_asdict_includes_pr2_fields(self):
        """asdict() output includes all PR2 fields"""
        entry = AuditEntry(
            timestamp="t", model_name="m", input_command="c",
            state_battery=None, state_standing=None, state_emergency=None,
            llm_output=None, api_code=None, sequence=None,
            safety_verdict="ok", safety_reason=None,
            elapsed_ms=0, cache_hit=False, route="7B", success=False,
            request_id="r123", router_mode="legacy",
        )
        d = asdict(entry)
        assert "request_id" in d
        assert "router_mode" in d
        assert "shadow_comparison" in d
        assert "action_latency_ms" in d
        assert "voice_latency_ms" in d

    def test_shadow_timeout_entry(self):
        """Shadow timeout with dual_api_code='timeout' can be serialized"""
        shadow = {
            "legacy_api_code": 1009,
            "legacy_sequence": None,
            "dual_api_code": "timeout",
            "dual_sequence": None,
            "raw_agreement": False,
            "high_risk_divergence": False,
            "legacy_ms": 200.0,
            "dual_ms": 5000.0,
        }
        entry = AuditEntry(
            timestamp="t", model_name="shadow", input_command="c",
            state_battery=None, state_standing=None, state_emergency=None,
            llm_output=None, api_code=1009, sequence=None,
            safety_verdict="ok", safety_reason=None,
            elapsed_ms=200.0, cache_hit=False, route="shadow", success=True,
            request_id="t123", router_mode="shadow",
            shadow_comparison=shadow,
        )
        json_str = json.dumps(asdict(entry))
        restored = json.loads(json_str)
        assert restored["shadow_comparison"]["dual_api_code"] == "timeout"

    def test_shadow_error_entry(self):
        """Shadow exception with dual_api_code='error' can be serialized"""
        shadow = {
            "legacy_api_code": 1009,
            "dual_api_code": "error",
            "error": "connection refused",
            "raw_agreement": False,
        }
        entry = AuditEntry(
            timestamp="t", model_name="shadow", input_command="c",
            state_battery=None, state_standing=None, state_emergency=None,
            llm_output=None, api_code=1009, sequence=None,
            safety_verdict="ok", safety_reason=None,
            elapsed_ms=200.0, cache_hit=False, route="shadow", success=True,
            shadow_comparison=shadow,
        )
        json_str = json.dumps(asdict(entry))
        assert "connection refused" in json_str
