#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_channel_router.py — PR2 Slice B: ChannelRouter skeleton validation

Validates:
  - B1: RouterMode enum and BRAIN_ROUTER_MODE env var
  - B2: Legacy mode passthrough to existing LLM path
  - B3: Dual mode: action -> template / a=null -> voice
  - B4: Shadow mode: legacy as primary, dual parallel observation
  - B5: SafetyCompiler still validates in all modes (Invariant 1)
  - B6: Invalid BRAIN_ROUTER_MODE -> fallback to legacy
  - B7: RouterResult data integrity
"""

import sys
import os
import asyncio
import logging
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from unittest.mock import AsyncMock, MagicMock, patch

from claudia.brain.channel_router import (
    ChannelRouter, RouterMode, RouterResult, MAX_SEQUENCE_LENGTH,
)
from claudia.brain.production_brain import ProductionBrain, BrainOutput, _pae_depth
from claudia.brain.audit_routes import (
    ROUTE_LLM_7B, ROUTE_ACTION_CHANNEL, ROUTE_VOICE_CHANNEL,
    ROUTE_SHADOW, ROUTE_ACTION_FALLBACK,
)


# === Helpers ===

def _make_brain():
    """Create a minimal mock brain"""
    with patch.object(ProductionBrain, '__init__', lambda self, **kw: None):
        brain = ProductionBrain.__new__(ProductionBrain)
    brain.logger = logging.getLogger("test_router")
    brain._command_lock = asyncio.Lock()
    brain.sport_client = None
    brain.use_real_hardware = False
    brain.model_7b = "test-model-7b"
    brain.safety_compiler = MagicMock()
    brain.state_monitor = None
    brain.audit_logger = None
    brain.last_posture_standing = False
    brain.EMERGENCY_COMMANDS = {}
    brain._generate_conversational_response = lambda cmd: "はい、何でしょうか？"

    # _ensure_model_loaded calls real Ollama API, so mock it in tests
    async def _mock_ensure(model, **kwargs):
        return True
    brain._ensure_model_loaded = _mock_ensure

    return brain


# === B1: RouterMode enum ===

class TestRouterMode:
    """Validate RouterMode enum values"""

    def test_legacy_mode(self):
        assert RouterMode("legacy") == RouterMode.LEGACY

    def test_dual_mode(self):
        assert RouterMode("dual") == RouterMode.DUAL

    def test_shadow_mode(self):
        assert RouterMode("shadow") == RouterMode.SHADOW

    def test_invalid_mode_raises(self):
        import pytest
        with pytest.raises(ValueError):
            RouterMode("invalid")


# === B2: Legacy mode ===

class TestLegacyRoute:
    """Validate Legacy mode passthrough to 7B LLM"""

    def test_legacy_delegates_to_ollama_v2(self):
        """Legacy mode calls brain._call_ollama_v2"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.LEGACY)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            return {"r": "座ります", "a": 1009}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(
                router.route("おすわり"))
            assert isinstance(result, RouterResult)
            assert result.api_code == 1009
            assert result.response == "座ります"
            assert result.route == ROUTE_LLM_7B
        finally:
            loop.close()

    def test_legacy_none_response(self):
        """Legacy mode returns default text when LLM has no response"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.LEGACY)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            return None

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            assert result.api_code is None
            assert result.sequence is None
            assert "理解できませんでした" in result.response
        finally:
            loop.close()

    def test_legacy_returns_sequence(self):
        """Legacy mode returns sequence"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.LEGACY)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            return {"r": "立ってから座ります", "s": [1004, 1009]}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("立ってから座って"))
            assert result.sequence == [1004, 1009]
            assert result.api_code is None or result.api_code == result.sequence
        finally:
            loop.close()


# === B3: Dual mode ===

class TestDualRoute:
    """Validate Dual mode: action channel + voice fallback"""

    def test_dual_action_returns_template(self):
        """Dual: action channel returns valid api_code -> template response"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        call_count = {"action": 0, "legacy": 0}

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v3":
                call_count["action"] += 1
                return {"a": 1009}
            else:
                call_count["legacy"] += 1
                return {"r": "座ります", "a": 1009}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("おすわり"))
            assert result.api_code == 1009
            assert result.route == ROUTE_ACTION_CHANNEL
            assert call_count["action"] == 1
            # Should not call legacy (action succeeded)
            assert call_count["legacy"] == 0
            # Response should be template (not LLM generated)
            assert len(result.response) > 0
        finally:
            loop.close()

    def test_dual_a_null_goes_voice(self):
        """Dual: a=null -> template conversational response (Invariant 2: not empty)"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            return {"a": None}  # Action model: no action

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("お名前は？"))
            assert result.api_code is None
            assert result.route == ROUTE_VOICE_CHANNEL
            # Invariant 2: response is not empty (template generated)
            assert len(result.response) > 0
        finally:
            loop.close()

    def test_dual_action_timeout_fallback(self):
        """Dual: action channel timeout -> template fallback (no 7B call)"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v3":
                return None  # Timeout
            else:
                return {"r": "座ります", "a": 1009}

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("おすわり"))
            # Template fallback: no action, template text response
            assert result.api_code is None
            assert result.route == ROUTE_ACTION_FALLBACK
            assert len(result.response) > 0
        finally:
            loop.close()

    def test_dual_invalid_api_code_fallback(self):
        """Dual: action channel returns invalid api_code -> template fallback"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            return {"a": 9999}  # Invalid code (only action model path)

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("おすわり"))
            assert result.route == ROUTE_ACTION_FALLBACK
            assert result.api_code is None
            assert len(result.response) > 0
        finally:
            loop.close()

    def test_dual_sequence_valid(self):
        """Dual: action channel returns valid sequence"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v3":
                return {"s": [1004, 1016]}
            return None

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("立ってから挨拶"))
            assert result.sequence == [1004, 1016]
            assert result.route == ROUTE_ACTION_CHANNEL
        finally:
            loop.close()


# === B4: Shadow mode ===

class TestShadowRoute:
    """Validate Shadow mode: legacy as primary, dual parallel"""

    def test_shadow_returns_legacy_result(self):
        """Shadow mode always returns legacy result"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.SHADOW)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v3":
                return {"a": 1016}  # dual picks hello
            else:
                return {"r": "座ります", "a": 1009}  # legacy picks sit

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            # Primary path is legacy
            assert result.api_code == 1009
            assert result.route == ROUTE_SHADOW
        finally:
            loop.close()

    def test_shadow_has_comparison(self):
        """Shadow mode includes shadow_comparison data"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.SHADOW)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v3":
                return {"a": 1016}
            else:
                return {"r": "座ります", "a": 1009}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            assert result.shadow_comparison is not None
            sc = result.shadow_comparison
            assert sc["legacy_api_code"] == 1009
            assert sc["dual_api_code"] == 1016
            assert sc["raw_agreement"] is False  # Disagreement
        finally:
            loop.close()

    def test_shadow_agreement_when_same(self):
        """Shadow: legacy and dual agree -> raw_agreement=True"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.SHADOW)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            return {"r": "座ります", "a": 1009}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("おすわり"))
            sc = result.shadow_comparison
            assert sc["raw_agreement"] is True
        finally:
            loop.close()

    def test_shadow_timeout_recorded(self):
        """Shadow: _action_channel_shadow timeout -> dual_status='timeout' (Invariant 4)

        Single GPU sequential design: _action_channel_shadow wraps action channel call,
        returns RouterResult with _action_status="timeout" on timeout.
        """
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.SHADOW)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v3":
                await asyncio.sleep(10)  # Timeout
                return {"a": 1009}
            else:
                return {"r": "座ります", "a": 1009}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            # Monkey-patch _action_channel_shadow with short timeout (avoid waiting 10s)
            original = router._action_channel_shadow

            async def fast_action_shadow(command, request_id, timeout=20):
                return await original(command, request_id, timeout=0.1)

            router._action_channel_shadow = fast_action_shadow
            result = loop.run_until_complete(router.route("test"))
            sc = result.shadow_comparison
            assert sc["dual_status"] == "timeout"
            assert sc["raw_agreement"] is False
        finally:
            loop.close()

    def test_shadow_action_channel_internal_timeout_encoded(self):
        """Action channel internal timeout (raw=None) -> dual_status='timeout'

        When _call_ollama_v2 returns None (Ollama internal timeout) and allow_fallback=False,
        _action_channel returns _action_status="timeout".
        _build_shadow_comparison should read _action_status and encode dual_status="timeout".
        """
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.SHADOW)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v3":
                return None  # Ollama internal timeout
            else:
                return {"r": "座ります", "a": 1009}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("座って"))
            sc = result.shadow_comparison
            assert sc is not None, "shadow_comparison should exist"
            assert sc["dual_status"] == "timeout", \
                "Action channel internal timeout: dual_status='timeout', got: {}".format(
                    sc["dual_status"])
            assert sc["raw_agreement"] is False
        finally:
            loop.close()

    def test_shadow_invalid_output_encoded(self):
        """Action channel invalid api_code -> dual_status='invalid_output'

        Finding #2: When model outputs invalid api_code, shadow should record
        dual_status="invalid_output" instead of confusing zeroed None with normal a=null.
        """
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.SHADOW)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v3":
                return {"a": 9999}  # Invalid api_code
            else:
                return {"r": "やります", "a": None}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("何かして"))
            sc = result.shadow_comparison
            assert sc is not None
            assert sc["dual_status"] == "invalid_output", \
                "Invalid api_code: dual_status='invalid_output', got: {}".format(
                    sc["dual_status"])
            assert sc["raw_agreement"] is False
        finally:
            loop.close()

    def test_shadow_ok_status_on_normal_response(self):
        """Normal action channel return -> dual_status='ok'"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.SHADOW)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v3":
                return {"a": 1009}  # Valid
            else:
                return {"r": "座ります", "a": 1009}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("座って"))
            sc = result.shadow_comparison
            assert sc is not None
            assert sc["dual_status"] == "ok"
        finally:
            loop.close()


# === B5: SafetyCompiler validates in all modes (Invariant 1) ===

class TestSafetyCompilerIntegration:
    """Validate SafetyCompiler still works in Dual/Shadow paths"""

    def test_router_result_feeds_safety_compiler(self):
        """Dual mode RouterResult is verified by SafetyCompiler"""
        brain = _make_brain()
        brain._router_mode = RouterMode.DUAL
        brain._channel_router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v3":
                return {"a": 1030}  # FrontFlip (high-risk)
            return {"r": "やります", "a": 1030}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        # Mock SafetyCompiler rejecting high-risk action
        mock_verdict = MagicMock()
        mock_verdict.is_blocked = True
        mock_verdict.block_reason = "high_energy_blocked"
        mock_verdict.response_override = "バッテリー不足です"
        brain.safety_compiler.compile.return_value = mock_verdict

        # Mock _log_audit
        brain._log_audit = MagicMock()

        result = brain._apply_safety_to_router_result(
            "前方宙返り",
            RouterResult(
                api_code=1030, sequence=None,
                response="フロントフリップします",
                route=ROUTE_ACTION_CHANNEL,
                request_id="test123",
            ),
            state_snapshot=None,
            snapshot_monotonic_ts=None,
            start_time=time.time(),
        )
        # SafetyCompiler rejected
        assert result.api_code is None
        assert "バッテリー" in result.response
        brain.safety_compiler.compile.assert_called_once()


# === B6: Invalid BRAIN_ROUTER_MODE ===

class TestInvalidRouterMode:
    """Validate invalid mode fallback to legacy"""

    def test_invalid_mode_in_brain_init(self):
        """Invalid BRAIN_ROUTER_MODE -> fallback to legacy"""
        with patch.dict(os.environ, {"BRAIN_ROUTER_MODE": "invalid_mode"}):
            with patch.object(ProductionBrain, '__init__', lambda self, **kw: None):
                brain = ProductionBrain.__new__(ProductionBrain)
            brain.logger = logging.getLogger("test")

            # Simulate router initialization logic from __init__
            router_mode_str = os.getenv("BRAIN_ROUTER_MODE", "legacy")
            try:
                mode = RouterMode(router_mode_str)
            except ValueError:
                mode = RouterMode.LEGACY
            assert mode == RouterMode.LEGACY


# === B7: RouterResult properties ===

class TestRouterResultProperties:
    """Validate RouterResult data integrity"""

    def test_has_action_with_api_code(self):
        r = RouterResult(api_code=1009, sequence=None, response="",
                         route=ROUTE_LLM_7B)
        assert r.has_action is True

    def test_has_action_with_sequence(self):
        r = RouterResult(api_code=None, sequence=[1004, 1009], response="",
                         route=ROUTE_LLM_7B)
        assert r.has_action is True

    def test_no_action_null(self):
        r = RouterResult(api_code=None, sequence=None, response="test",
                         route=ROUTE_LLM_7B)
        assert r.has_action is False

    def test_no_action_empty_sequence(self):
        r = RouterResult(api_code=None, sequence=[], response="test",
                         route=ROUTE_LLM_7B)
        assert r.has_action is False

    def test_shadow_comparison_optional(self):
        r = RouterResult(api_code=None, sequence=None, response="",
                         route=ROUTE_LLM_7B)
        assert r.shadow_comparison is None

    def test_request_id_default_empty(self):
        r = RouterResult(api_code=None, sequence=None, response="",
                         route=ROUTE_LLM_7B)
        assert r.request_id == ""


# === Fix #4: Sequence validation ===

class TestSequenceValidation:
    """Validate action channel sequence validation logic"""

    def test_sequence_items_validated(self):
        """Each item in sequence is checked against VALID_API_CODES"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v3":
                return {"s": [1004, 1009]}  # All valid
            return None

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            assert result.sequence == [1004, 1009]
        finally:
            loop.close()

    def test_sequence_invalid_items_filtered(self):
        """Invalid items are filtered, valid items retained"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v3":
                return {"s": [1004, 9999, 1009]}  # 9999 is invalid
            return None

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            assert result.sequence == [1004, 1009]
            assert result.route == ROUTE_ACTION_CHANNEL
        finally:
            loop.close()

    def test_sequence_all_invalid_fallback(self):
        """All items invalid -> fallback"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v3":
                return {"s": [9999, 8888]}
            else:
                return {"r": "座ります", "a": 1009}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            assert result.route == ROUTE_ACTION_FALLBACK
        finally:
            loop.close()

    def test_sequence_length_limit(self):
        """Sequence exceeding MAX_SEQUENCE_LENGTH is truncated"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        long_seq = [1004, 1009, 1016, 1017, 1022]  # 5 items
        assert len(long_seq) > MAX_SEQUENCE_LENGTH

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v3":
                return {"s": long_seq}
            return None

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            assert len(result.sequence) == MAX_SEQUENCE_LENGTH
            assert result.sequence == long_seq[:MAX_SEQUENCE_LENGTH]
        finally:
            loop.close()

    def test_sequence_empty_fallback(self):
        """Empty sequence [] -> template fallback"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            return {"s": []}  # Empty sequence

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            assert result.route == ROUTE_ACTION_FALLBACK
            assert result.api_code is None
            assert len(result.response) > 0
        finally:
            loop.close()

    def test_sequence_non_list_fallback(self):
        """s is not a list -> template fallback"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            return {"s": "not_a_list"}

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            assert result.route == ROUTE_ACTION_FALLBACK
            assert result.api_code is None
        finally:
            loop.close()


# === Shadow high-risk divergence detection (Fix #3) ===

class TestShadowHighRiskDivergence:
    """Validate shadow_comparison high_risk_divergence detection"""

    def test_high_risk_divergence_flagged(self):
        """Legacy picks high-risk action, dual does not -> high_risk_divergence=True"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.SHADOW)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v3":
                return {"a": 1009}  # dual: Sit (safe)
            else:
                return {"r": "フリップ!", "a": 1030}  # legacy: FrontFlip (high-risk)

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            sc = result.shadow_comparison
            assert sc["high_risk_divergence"] is True
        finally:
            loop.close()

    def test_no_divergence_both_safe(self):
        """Both sides pick safe actions -> high_risk_divergence=False"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.SHADOW)

        async def mock_ollama(model, command, timeout=10, **kwargs):
            if model == "claudia-action-v3":
                return {"a": 1009}  # Sit
            else:
                return {"r": "立ちます", "a": 1004}  # StandUp

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            sc = result.shadow_comparison
            assert sc["high_risk_divergence"] is False
        finally:
            loop.close()


# === Review fix: Fix #1 -- Action channel token budget ===

class TestActionChannelTokenBudget:
    """Fix #1: Action channel uses num_predict=30, num_ctx=1024"""

    def test_action_channel_passes_low_token_budget(self):
        """_action_channel() passes compact parameters to _call_ollama_v2"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        captured_kwargs = {}

        async def mock_ollama(model, command, timeout=10,
                              num_predict=100, num_ctx=2048,
                              output_format='json'):
            captured_kwargs['num_predict'] = num_predict
            captured_kwargs['num_ctx'] = num_ctx
            return {"a": 1009}

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            loop.run_until_complete(router._action_channel("おすわり", "test1"))
            assert captured_kwargs['num_predict'] == 30, (
                "Action channel should pass num_predict=30, actual={}".format(
                    captured_kwargs['num_predict']))
            assert captured_kwargs['num_ctx'] == 1024, (
                "Action channel should pass num_ctx=1024, actual={}".format(
                    captured_kwargs['num_ctx']))
        finally:
            loop.close()

    def test_legacy_route_uses_default_budget(self):
        """_legacy_route() uses default parameters for _call_ollama_v2"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.LEGACY)

        captured_kwargs = {}

        async def mock_ollama(model, command, timeout=10,
                              num_predict=100, num_ctx=2048,
                              output_format='json'):
            captured_kwargs['num_predict'] = num_predict
            captured_kwargs['num_ctx'] = num_ctx
            return {"r": "座ります", "a": 1009}

        brain._call_ollama_v2 = mock_ollama
        brain._sanitize_response = lambda x: x

        loop = asyncio.new_event_loop()
        try:
            loop.run_until_complete(router._legacy_route("おすわり", "test1"))
            assert captured_kwargs['num_predict'] == 100, (
                "Legacy route should use default num_predict=100, actual={}".format(
                    captured_kwargs['num_predict']))
            assert captured_kwargs['num_ctx'] == 2048, (
                "Legacy route should use default num_ctx=2048, actual={}".format(
                    captured_kwargs['num_ctx']))
        finally:
            loop.close()


# === Review fix: Fix #2 -- a+s conflict normalization ===

class TestActionSequenceConflict:
    """Fix #2: When both a and s are present, s takes priority"""

    def test_both_a_and_s_present_s_wins(self):
        """a=1009 + s=[1004,1016] both present -> s takes priority, a is cleared"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10,
                              num_predict=100, num_ctx=2048,
                              output_format='json'):
            if model == "claudia-action-v3":
                return {"a": 1009, "s": [1004, 1016]}  # Conflict!
            return {"r": "ok", "a": None}

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            # s takes priority: sequence retained, api_code cleared
            assert result.sequence == [1004, 1016]
            assert result.api_code is None
            # Template response should be based on sequence, not api_code
            assert "立ちます" in result.response
            assert "挨拶します" in result.response
        finally:
            loop.close()

    def test_only_a_present_works_normally(self):
        """Only a present, no s -> normal processing"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)

        async def mock_ollama(model, command, timeout=10,
                              num_predict=100, num_ctx=2048,
                              output_format='json'):
            if model == "claudia-action-v3":
                return {"a": 1009}
            return {"r": "ok", "a": None}

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("test"))
            assert result.api_code == 1009
            assert result.sequence is None
        finally:
            loop.close()


# === Review fix: Fix #3 -- Avoid double legacy call ===

class TestNoDoubleLegacyCall:
    """Fix #3: No duplicate legacy call after action fallback"""

    def test_action_fail_template_fallback_no_7b(self):
        """Action model failure -> template response, no 7B call"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)
        call_count = {"legacy": 0}

        async def mock_ollama(model, command, timeout=10,
                              num_predict=100, num_ctx=2048,
                              output_format='json'):
            if model == "claudia-action-v3":
                return None  # Action model failure
            else:
                call_count["legacy"] += 1
                return {"r": "fallback", "a": None}

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("天気について"))
            # Action-primary: on failure use template response, 7B is not called
            assert call_count["legacy"] == 0, (
                "7B should not be called, actual {} times".format(call_count["legacy"]))
            assert result.route == ROUTE_ACTION_FALLBACK
            assert result.api_code is None
            assert result.response  # Template response is non-empty
        finally:
            loop.close()

    def test_action_null_template_response_no_7b(self):
        """Action returns a=null normally -> template conversational response, no 7B call"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)
        call_count = {"total": 0}

        async def mock_ollama(model, command, timeout=10,
                              num_predict=100, num_ctx=2048,
                              output_format='json'):
            call_count["total"] += 1
            if model == "claudia-action-v3":
                return {"a": None}  # Normal: no action
            else:
                return {"r": "不応到達", "a": None}

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("天気について"))
            # Action-primary: a=null uses template, only calls action model once
            assert call_count["total"] == 1
            assert result.route == ROUTE_VOICE_CHANNEL
            assert result.response  # Template response is non-empty
        finally:
            loop.close()

    def test_action_success_no_7b_call(self):
        """Action returns valid action -> template response, 7B is not called"""
        brain = _make_brain()
        router = ChannelRouter(brain, RouterMode.DUAL)
        call_count = {"legacy": 0}

        async def mock_ollama(model, command, timeout=10,
                              num_predict=100, num_ctx=2048,
                              output_format='json'):
            if model == "claudia-action-v3":
                return {"a": 1009}  # Sit
            else:
                call_count["legacy"] += 1
                return {"r": "不応到達", "a": 1009}

        brain._call_ollama_v2 = mock_ollama

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(router.route("おすわり"))
            # 7B is not called at all
            assert call_count["legacy"] == 0
            assert result.api_code == 1009
            assert result.response  # ACTION_RESPONSES template
        finally:
            loop.close()


# === Review fix: Fix #4 -- Audit success semantics ===

class TestAuditSuccessSemantics:
    """Fix #4: success reflects pipeline success, not 'whether there is an action'"""

    def test_conversational_is_success(self):
        """Pure conversational command (no action) -> success=True"""
        brain = _make_brain()
        brain._log_audit = MagicMock()
        brain.audit_logger = MagicMock()

        from claudia.brain.audit_logger import AuditEntry
        from dataclasses import asdict

        # Construct pure conversational output (no api_code/sequence)
        output = BrainOutput(response="今日はいい天気ですね", api_code=None)

        # Manually call _log_audit and check success calculation
        # Re-enable _log_audit (remove mock)
        brain._log_audit = ProductionBrain._log_audit.__get__(brain)

        brain._log_audit(
            "天気について", output, route=ROUTE_LLM_7B,
            elapsed_ms=100.0, cache_hit=False,
            model_used="7b", current_state=None,
            llm_output='{"r":"今日はいい天気ですね","a":null}',
            safety_verdict="ok",
        )

        # Verify audit_logger.log_entry was called
        brain.audit_logger.log_entry.assert_called_once()
        entry = brain.audit_logger.log_entry.call_args[0][0]
        # Key: pure conversational should be success=True (pipeline completed normally)
        assert entry.success is True, (
            "Pure conversational command should be success=True, actual={}".format(entry.success))

    def test_action_is_success(self):
        """Action command -> success=True"""
        brain = _make_brain()
        brain.audit_logger = MagicMock()
        brain._log_audit = ProductionBrain._log_audit.__get__(brain)

        output = BrainOutput(response="座ります", api_code=1009)
        brain._log_audit(
            "おすわり", output, route=ROUTE_LLM_7B,
            elapsed_ms=100.0, cache_hit=False,
            model_used="7b", current_state=None,
            llm_output='{"r":"座ります","a":1009}',
            safety_verdict="ok",
        )

        entry = brain.audit_logger.log_entry.call_args[0][0]
        assert entry.success is True

    def test_safety_rejected_is_success(self):
        """Safety rejection -> success=True (system correctly rejected dangerous action)"""
        brain = _make_brain()
        brain.audit_logger = MagicMock()
        brain._log_audit = ProductionBrain._log_audit.__get__(brain)

        output = BrainOutput(
            response="バッテリー不足です", api_code=None,
            reasoning="router_safety_rejected")
        brain._log_audit(
            "フリップして", output, route=ROUTE_ACTION_CHANNEL,
            elapsed_ms=100.0, cache_hit=False,
            model_used="dual", current_state=None,
            llm_output='{"a":1030}',
            safety_verdict="rejected:high_energy_blocked",
        )

        entry = brain.audit_logger.log_entry.call_args[0][0]
        # Safety rejection is also success=True (pipeline ran correctly, properly rejected)
        assert entry.success is True, (
            "Safety rejection should be success=True, actual={}".format(entry.success))
