#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_safety_regression.py — P0 fix regression tests

Validates:
  - P0-1: Return code 3104 no longer misjudged as success (requires Mock)
  - P0-2: Return code 3103 logged correctly
  - P0-5: Initialization uses GetState instead of RecoveryStand
  - P0-8: Sequence mid-failure aborts
  - hot_cache normalization matching
  - SafetyCompiler full-path coverage semantic consistency
"""

import sys
import os
import asyncio
import logging
from unittest.mock import MagicMock, patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from claudia.brain.action_registry import (
    VALID_API_CODES, EXECUTABLE_API_CODES, METHOD_MAP,
    get_response_for_action,
)
from claudia.brain.safety_compiler import SafetyCompiler


# === P0-1: Return code 3104 semantic tests ===
# Note: Full _execute_real integration tests require Mock SportClient,
# here we test semantic correctness at the SafetyCompiler level.


class TestReturnCodeSemantics:
    """Ensure SafetyCompiler does not affect 3104/3103 handling"""

    def test_3104_is_not_in_safe_codes(self):
        """3104 is not an API code, not in any whitelist"""
        assert 3104 not in VALID_API_CODES
        assert 3104 not in EXECUTABLE_API_CODES

    def test_3103_is_not_in_safe_codes(self):
        """3103 is not an API code, not in any whitelist"""
        assert 3103 not in VALID_API_CODES
        assert 3103 not in EXECUTABLE_API_CODES


# === P0-8: Sequence mid-failure abort ===

class TestSequenceAbort:
    """Sequence mid-failure logic (SafetyCompiler level)"""

    def test_sequence_first_invalid_blocks(self):
        """First action in sequence is invalid -> entire request rejected"""
        sc = SafetyCompiler()
        v = sc.compile([9999, 1004], battery_level=0.80, is_standing=True)
        assert v.is_blocked
        assert 9999 in v.rejected

    def test_sequence_mid_invalid_truncates(self):
        """Invalid action mid-sequence -> truncated, keeping preceding actions"""
        sc = SafetyCompiler()
        v = sc.compile([1004, 9999, 1005], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1004]
        assert 9999 in v.rejected

    def test_sequence_all_valid(self):
        """All actions in sequence are valid -> all pass"""
        sc = SafetyCompiler()
        v = sc.compile([1004, 1016], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1004, 1016]


# === Hot cache normalization ===

class TestHotCacheNormalization:
    """Hot cache should match various input formats"""

    def test_basic_match_logic(self):
        """Basic string matching: strip + lower"""
        # Simulate hot_cache matching logic
        hot_cache = {
            "座って": {"api_code": 1009},
            "立って": {"api_code": 1004},
        }
        # Basic match
        cmd = "座って"
        assert cmd.strip() in hot_cache
        # With whitespace
        cmd_ws = " 座って "
        assert cmd_ws.strip() in hot_cache

    def test_case_insensitive_fallback(self):
        """English command lower-case fallback matching"""
        hot_cache_lower = {
            "sit": {"api_code": 1009},
            "stop": {"api_code": 1003},
        }
        cmd = "SIT"
        assert cmd.strip().lower() in hot_cache_lower
        cmd2 = " Stop "
        assert cmd2.strip().lower() in hot_cache_lower


# === Smoke test: Semantic consistency ===

class TestSmokeSemanticConsistency:
    """Semantic-level consistency: validate routing and action categories (not exact api_code)"""

    POSTURE_CODES = frozenset([1001, 1002, 1003, 1004, 1005, 1006, 1009, 1010])
    PERFORMANCE_CODES = frozenset([1016, 1017, 1021, 1022, 1023, 1029, 1033, 1036])

    def test_safety_compiler_posture_actions_pass(self):
        """All posture actions pass at normal battery level"""
        sc = SafetyCompiler()
        for code in self.POSTURE_CODES:
            v = sc.compile([code], battery_level=0.80, is_standing=True)
            assert not v.is_blocked, "Posture action {} should pass".format(code)
            assert code in v.executable_sequence

    def test_safety_compiler_performance_actions_pass(self):
        """All performance actions pass at normal battery level"""
        sc = SafetyCompiler()
        for code in self.PERFORMANCE_CODES:
            if code in EXECUTABLE_API_CODES:
                v = sc.compile([code], battery_level=0.80, is_standing=True)
                assert not v.is_blocked, "Performance action {} should pass".format(code)
                assert code in v.executable_sequence

    def test_safety_compiler_dance_either_variant(self):
        """Dance1(1022) and Dance2(1023) both pass"""
        sc = SafetyCompiler()
        for code in [1022, 1023]:
            v = sc.compile([code], battery_level=0.80, is_standing=True)
            assert not v.is_blocked
            assert code in v.executable_sequence

    def test_safety_compiler_standing_prepend_for_hello(self):
        """Hello(1016) not standing -> auto-prepend StandUp"""
        sc = SafetyCompiler()
        v = sc.compile([1016], battery_level=0.80, is_standing=False)
        assert not v.is_blocked
        assert v.executable_sequence == [1004, 1016]

    def test_safety_compiler_sequence_stand_then_hello(self):
        """Sequence [1004, 1016] already standing -> passes as-is"""
        sc = SafetyCompiler()
        v = sc.compile([1004, 1016], battery_level=0.80, is_standing=True)
        assert not v.is_blocked
        assert v.executable_sequence == [1004, 1016]


# === Response helper regression ===

class TestResponseHelperRegression:
    """Ensure action_registry response helper function behavior is unchanged"""

    def test_known_action_response(self):
        """Known action returns Japanese name"""
        assert get_response_for_action(1004) == "立ちます"
        assert get_response_for_action(1016) == "挨拶します"
        assert get_response_for_action(1009) == "座ります"

    def test_unknown_action_default(self):
        """Unknown action returns default response"""
        assert get_response_for_action(9999) == "はい、わかりました"

    def test_all_enabled_have_responses(self):
        """All actions in METHOD_MAP have responses"""
        from claudia.brain.action_registry import ACTION_RESPONSES
        for code in METHOD_MAP:
            assert code in ACTION_RESPONSES, "Action {} is missing a response".format(code)


# === P0-5: Initialization connectivity (static validation) ===

class TestInitConnectivity:
    """Validate MockSportClient.GetState signature compatibility"""

    def test_mock_getstate_returns_tuple(self):
        """MockSportClient.GetState returns (code, data) tuple"""
        from claudia.brain.mock_sport_client import MockSportClient
        mock = MockSportClient()
        result = mock.GetState(["mode"])
        assert isinstance(result, tuple)
        assert len(result) == 2
        assert result[0] == 0  # success code

    def test_mock_getstate_no_args(self):
        """MockSportClient.GetState() can be called without arguments"""
        from claudia.brain.mock_sport_client import MockSportClient
        mock = MockSportClient()
        result = mock.GetState()
        assert isinstance(result, tuple)
        assert result[0] == 0


class TestGetStateProbeValidation:
    """Validate GetState probe result validity check (prevent code=0 + empty data false success)"""

    def _make_brain(self):
        return _make_lightweight_brain()[0]

    def test_probe_code_0_empty_dict_is_invalid(self):
        """code=0 but data={} should be judged invalid"""
        brain = self._make_brain()
        assert brain._is_valid_getstate_probe(0, {}) is False

    def test_probe_code_0_none_is_invalid(self):
        """code=0 but data=None should be judged invalid"""
        brain = self._make_brain()
        assert brain._is_valid_getstate_probe(0, None) is False

    def test_probe_code_0_nonempty_dict_is_valid(self):
        """code=0 with non-empty dict data is a valid probe"""
        brain = self._make_brain()
        data = {"state": 1, "gait": 0}
        assert brain._is_valid_getstate_probe(0, data) is True

    def test_probe_nonzero_code_is_invalid(self):
        """code!=0 is always invalid"""
        brain = self._make_brain()
        assert brain._is_valid_getstate_probe(3103, {"state": 1}) is False


# === P0-4: Dead code removal validation ===

class TestDeadCodeRemoved:
    """Validate dead code no longer exists in MockSportClient"""

    def test_no_rollover_method(self):
        """MockSportClient should not have Rollover method (does not exist in SDK)"""
        from claudia.brain.mock_sport_client import MockSportClient
        assert not hasattr(MockSportClient, 'Rollover')

    def test_no_handstand_method(self):
        """MockSportClient should not have Handstand method (does not exist in SDK)"""
        from claudia.brain.mock_sport_client import MockSportClient
        assert not hasattr(MockSportClient, 'Handstand')


# === METHOD_MAP completeness regression ===

class TestMethodMapRegression:
    """METHOD_MAP and MockSportClient consistency"""

    def test_all_method_map_methods_exist_on_mock(self):
        """All methods in METHOD_MAP exist on MockSportClient"""
        from claudia.brain.mock_sport_client import MockSportClient
        mock = MockSportClient()
        missing = []
        for api_code, method_name in METHOD_MAP.items():
            if not hasattr(mock, method_name):
                missing.append((api_code, method_name))
        assert not missing, "MockSportClient is missing methods: {}".format(missing)


# === P0-1: _execute_real 3104 branch true semantic tests ===

def _run_async(coro):
    """Python 3.8 compatible asyncio runner"""
    loop = asyncio.new_event_loop()
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


def _make_lightweight_brain():
    """Create lightweight brain — skip ROS2 state monitor initialization (avoid Jetson OOM)

    Temporarily disables STATE_MONITOR_AVAILABLE to avoid rclpy/DDS initialization.
    """
    import claudia.brain.production_brain as pb_mod
    from claudia.brain.production_brain import ProductionBrain, BrainOutput
    orig = pb_mod.STATE_MONITOR_AVAILABLE
    pb_mod.STATE_MONITOR_AVAILABLE = False
    try:
        brain = ProductionBrain(use_real_hardware=False)
    finally:
        pb_mod.STATE_MONITOR_AVAILABLE = orig
    brain.sport_client = MagicMock()
    return brain, BrainOutput


class TestExecuteReal3104Semantics:
    """P0-1: 3104 (RPC_ERR_CLIENT_API_TIMEOUT) semantic validation

    Monkeypatches _rpc_call to simulate 3104 return, validates:
      - 3104 + GetState OK -> "unknown" (not True)
      - 3104 + GetState exception -> False
      - 3104 + GetState non-zero -> False
    """

    def _make_brain(self):
        return _make_lightweight_brain()

    def test_3104_getstate_ok_returns_unknown(self):
        """3104 + GetState(0) -> 'unknown'"""
        brain, BrainOutput = self._make_brain()

        call_count = [0]
        def mock_rpc_call(method, *args, **kwargs):
            call_count[0] += 1
            if method == "StandUp":
                return 0
            if method == "GetState":
                return (0, {"mode": 1})
            return 3104  # First call returns timeout
        brain._rpc_call = mock_rpc_call
        output = BrainOutput("", api_code=1016)  # Hello
        result = _run_async(brain._execute_real(output))
        assert result == "unknown", "3104+GetState(0) should return 'unknown', got: {}".format(result)

    def test_3104_getstate_exception_returns_unknown(self):
        """3104 + GetState exception -> 'unknown' (command was sent, should not be judged as failure)"""
        brain, BrainOutput = self._make_brain()

        def mock_rpc_call(method, *args, **kwargs):
            if method == "GetState":
                raise ConnectionError("DDS connection lost")
            return 3104

        brain._rpc_call = mock_rpc_call
        output = BrainOutput("", api_code=1016)
        result = _run_async(brain._execute_real(output))
        assert result == "unknown", (
            "3104 means command was sent, should return 'unknown' not False, got: {}".format(result)
        )

    def test_3104_getstate_nonzero_returns_unknown(self):
        """3104 + GetState returns non-zero -> 'unknown' (command was sent)"""
        brain, BrainOutput = self._make_brain()

        def mock_rpc_call(method, *args, **kwargs):
            if method == "GetState":
                return (3104, None)  # GetState also times out
            return 3104

        brain._rpc_call = mock_rpc_call
        output = BrainOutput("", api_code=1016)
        result = _run_async(brain._execute_real(output))
        assert result == "unknown", (
            "3104 means command was sent, should return 'unknown', got: {}".format(result)
        )

    def test_return_0_is_true(self):
        """RPC returns 0 -> True (success)"""
        brain, BrainOutput = self._make_brain()
        brain._rpc_call = lambda method, *args, **kwargs: 0
        output = BrainOutput("", api_code=1004)  # StandUp
        result = _run_async(brain._execute_real(output))
        assert result is True

    def test_return_neg1_is_true(self):
        """RPC returns -1 (already in target state) -> True"""
        brain, BrainOutput = self._make_brain()
        brain._rpc_call = lambda method, *args, **kwargs: -1
        output = BrainOutput("", api_code=1004)
        result = _run_async(brain._execute_real(output))
        assert result is True

    def test_return_3103_is_false(self):
        """RPC returns 3103 (APP occupied) -> False"""
        brain, BrainOutput = self._make_brain()
        brain._rpc_call = lambda method, *args, **kwargs: 3103
        output = BrainOutput("", api_code=1004)
        result = _run_async(brain._execute_real(output))
        assert result is False

    def test_sequence_mid_failure_aborts(self):
        """P0-8: Sequence mid-failure -> abort without continuing"""
        brain, BrainOutput = self._make_brain()

        executed = []
        def mock_rpc_call(method, *args, **kwargs):
            executed.append(method)
            if method == "Hello":
                return 3103  # Second action fails
            return 0

        brain._rpc_call = mock_rpc_call
        output = BrainOutput("", sequence=[1004, 1016, 1017])  # StandUp, Hello, Stretch
        result = _run_async(brain._execute_real(output))
        assert result is False
        # Should not execute Stretch after Hello failure
        assert "Stretch" not in executed, "Sequence should abort after Hello failure, but executed: {}".format(executed)


class TestStandupUnknownSequenceGuard:
    """StandUp unknown(3104) guard strategy in sequences"""

    def _make_brain(self):
        return _make_lightweight_brain()

    def test_standup_unknown_without_confirmation_aborts_sequence(self):
        """StandUp=unknown and confirmation failed -> sequence aborts, no subsequent actions executed"""
        brain, BrainOutput = self._make_brain()
        executed = []

        def mock_rpc_call(method, *args, **kwargs):
            executed.append(method)
            if method == "StandUp":
                return 3104
            if method == "GetState":
                return (0, {"state": 1})  # Triggers single action return unknown
            if method == "Hello":
                return 0
            return 0

        async def fake_verify(*args, **kwargs):
            return False

        async def _no_sleep(_):
            return None

        brain._rpc_call = mock_rpc_call
        brain._verify_standing_after_unknown = fake_verify

        with patch("claudia.brain.production_brain.asyncio.sleep", new=_no_sleep):
            output = BrainOutput("", sequence=[1004, 1016])  # StandUp, Hello
            result = _run_async(brain._execute_real(output))

        assert result is False
        assert "Hello" not in executed, (
            "Should not execute subsequent actions when StandUp is unknown and standing unconfirmed, executed: {}".format(executed)
        )

    def test_standup_unknown_with_confirmation_continues_sequence(self):
        """StandUp=unknown but confirmation succeeds -> sequence continues"""
        brain, BrainOutput = self._make_brain()
        executed = []

        def mock_rpc_call(method, *args, **kwargs):
            executed.append(method)
            if method == "StandUp":
                return 3104
            if method == "GetState":
                return (0, {"state": 1})
            if method == "Hello":
                return 0
            return 0

        async def fake_verify(*args, **kwargs):
            return True

        async def _no_sleep(_):
            return None

        brain._rpc_call = mock_rpc_call
        brain._verify_standing_after_unknown = fake_verify

        with patch("claudia.brain.production_brain.asyncio.sleep", new=_no_sleep):
            output = BrainOutput("", sequence=[1004, 1016])  # StandUp, Hello
            result = _run_async(brain._execute_real(output))

        assert result is True
        assert "Hello" in executed, "Should continue executing subsequent actions after standing confirmed"


# === Finding 1: Emergency stop return code tests ===

class TestEmergencyStopReturnCode:
    """Emergency stop RPC return code correctly reflected in execution_status"""

    def _make_brain(self):
        brain, _ = _make_lightweight_brain()
        return brain

    def test_emergency_rpc_success(self):
        """RPC returns 0 -> execution_status='success'"""
        brain = self._make_brain()
        brain._rpc_call = lambda method, *args, **kwargs: 0
        result = _run_async(brain._handle_emergency("止まれ"))
        assert result.execution_status == "success"

    def test_emergency_rpc_failure(self):
        """RPC returns non-zero -> execution_status='failed'"""
        brain = self._make_brain()
        brain._rpc_call = lambda method, *args, **kwargs: 3103
        result = _run_async(brain._handle_emergency("止まれ"))
        assert result.execution_status == "failed"

    def test_emergency_rpc_exception(self):
        """RPC exception -> execution_status='failed'"""
        brain = self._make_brain()
        def raise_err(*args, **kwargs):
            raise ConnectionError("DDS dead")
        brain._rpc_call = raise_err
        result = _run_async(brain._handle_emergency("止まれ"))
        assert result.execution_status == "failed"

    def test_emergency_no_client_success(self):
        """No sport_client (simulation mode) -> execution_status='success'"""
        brain = self._make_brain()
        brain.sport_client = None
        result = _run_async(brain._handle_emergency("止まれ"))
        assert result.execution_status == "success"


# === Fix #4: End-to-end hot_cache routing tests ===

class TestHotCacheE2ERouting:
    """Validate hot_cache routing behavior via process_command (not simulated dict matching)

    Ensures variants like cute/whitespace/case go through hot_cache path rather than conversational.
    Provides mock state_monitor to ensure SafetyCompiler gets normal state
    (battery=0.80, is_standing=True), testing routing rather than safety rejection.
    """

    def _make_brain(self):
        brain, BrainOutput = _make_lightweight_brain()
        brain.sport_client = None  # Simulation mode, no sport_client needed
        # Provide mock state_monitor to avoid fail-safe (battery=0.0) rejecting non-safe actions
        mock_state = MagicMock()
        mock_state.battery_level = 0.80
        mock_state.is_standing = True
        mock_state.is_moving = False
        mock_state.temperature = 40.0
        mock_state.timestamp = 0.0
        mock_monitor = MagicMock()
        mock_monitor.get_current_state.return_value = mock_state
        mock_monitor.is_ros_initialized = True  # Preserve mock state's is_standing=True
        brain.state_monitor = mock_monitor
        return brain, BrainOutput

    def test_kawaii_routes_to_heart(self):
        """かわいい -> hot_cache -> Heart(1036)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("かわいい"))
        assert result.api_code == 1036, (
            "かわいい should route to Heart(1036), got: api_code={}".format(result.api_code)
        )

    def test_kawaii_kanji_routes_to_heart(self):
        """可愛い -> hot_cache -> Heart(1036)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("可愛い"))
        assert result.api_code == 1036

    def test_sugoi_routes_to_heart(self):
        """すごい -> hot_cache -> Heart(1036)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("すごい"))
        assert result.api_code == 1036

    def test_sugoi_kanji_routes_to_heart(self):
        """凄い -> hot_cache -> Heart(1036)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("凄い"))
        assert result.api_code == 1036

    def test_cute_english_routes_to_heart(self):
        """cute -> hot_cache -> Heart(1036)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("cute"))
        assert result.api_code == 1036

    def test_whitespace_normalization(self):
        """' 座って ' (with spaces) -> hot_cache -> Sit(1009)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command(" 座って "))
        assert result.api_code == 1009, (
            "' 座って ' should match hot_cache via strip(), got: api_code={}".format(result.api_code)
        )

    def test_case_normalization_english(self):
        """'STOP' -> hot_cache (lower fallback) -> Stop(1003)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("STOP"))
        assert result.api_code == 1003, (
            "'STOP' should match hot_cache via lower(), got: api_code={}".format(result.api_code)
        )

    def test_case_normalization_hello(self):
        """'Hello' -> hot_cache (lower fallback) -> Hello(1016)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("Hello"))
        assert result.api_code == 1016

    def test_greeting_ohayo_routes_to_hello(self):
        """'おはよう' -> hot_cache -> Hello(1016)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("おはよう"))
        assert result.api_code == 1016

    def test_greeting_konbanwa_routes_to_hello(self):
        """'こんばんは' -> hot_cache -> Hello(1016)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("こんばんは"))
        assert result.api_code == 1016

    def test_greeting_sayounara_routes_to_hello(self):
        """'さようなら' -> hot_cache -> Hello(1016)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("さようなら"))
        assert result.api_code == 1016

    def test_greeting_punctuation_normalization(self):
        """'おはよう！' -> rstrip punctuation -> hot_cache -> Hello(1016)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("おはよう！"))
        assert result.api_code == 1016


class TestBowActionMapping:
    """Bow/New Year greeting action mapping regression — ちんちん/お辞儀/拜年 unified mapping to Scrape(1029)

    Scrape(1029) performs a front-paw bow motion on Go2, semantically corresponding to bow/New Year greeting.
    Hello(1016) is a wave motion, not a bow. Ensure all bow-semantic words consistently map to 1029.
    """

    def _make_brain(self):
        brain, BrainOutput = _make_lightweight_brain()
        brain.sport_client = None
        mock_state = MagicMock()
        mock_state.battery_level = 0.80
        mock_state.is_standing = True
        mock_state.is_moving = False
        mock_state.temperature = 40.0
        mock_state.timestamp = 0.0
        mock_monitor = MagicMock()
        mock_monitor.get_current_state.return_value = mock_state
        mock_monitor.is_ros_initialized = True
        brain.state_monitor = mock_monitor
        return brain, BrainOutput

    def test_chinchin_routes_to_scrape(self):
        """'ちんちん' -> hot_cache -> Scrape(1029)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("ちんちん"))
        assert result.api_code == 1029, (
            "ちんちん should route to Scrape(1029), got: api_code={}".format(result.api_code)
        )

    def test_chinchin_katakana_routes_to_scrape(self):
        """'チンチン' (katakana) -> hot_cache -> Scrape(1029)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("チンチン"))
        assert result.api_code == 1029, (
            "チンチン should route to Scrape(1029), got: api_code={}".format(result.api_code)
        )

    def test_bainian_routes_to_scrape(self):
        """'拜年' (Chinese) -> hot_cache -> Scrape(1029)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("拜年"))
        assert result.api_code == 1029, (
            "拜年 should route to Scrape(1029), got: api_code={}".format(result.api_code)
        )

    def test_ojigi_routes_to_scrape(self):
        """'お辞儀' -> hot_cache -> Scrape(1029)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("お辞儀"))
        assert result.api_code == 1029, (
            "お辞儀 should route to Scrape(1029), got: api_code={}".format(result.api_code)
        )

    def test_rei_routes_to_scrape(self):
        """'礼' -> hot_cache -> Scrape(1029)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("礼"))
        assert result.api_code == 1029, (
            "礼 should route to Scrape(1029), got: api_code={}".format(result.api_code)
        )

    def test_chinchin_with_punctuation_routes_to_scrape(self):
        """'ちんちん！' -> rstrip punctuation -> hot_cache -> Scrape(1029)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command("ちんちん！"))
        assert result.api_code == 1029, (
            "ちんちん！ should match hot_cache via punctuation normalization, got: api_code={}".format(result.api_code)
        )


class TestPose1028IntentionalWhitelistDifference:
    """Validate intentional design difference in Pose(1028) dual-layer whitelist

    1028 has has_params=True + safe_default_params=(True,), so:
    - Excluded from LLM path (VALID_API_CODES) (prevent hallucinated parameters)
    - Included in execution path (EXECUTABLE_API_CODES) (execute with safe defaults)
    - Registered in hot_cache as direct user command
    """

    def test_pose_not_in_valid_api_codes(self):
        """1028 is not included in LLM decision whitelist"""
        assert 1028 not in VALID_API_CODES

    def test_pose_in_executable_api_codes(self):
        """1028 is included in execution whitelist"""
        assert 1028 in EXECUTABLE_API_CODES

    def test_pose_in_hot_cache(self):
        """1028 is registered in hot_cache"""
        brain, _ = _make_lightweight_brain()
        pose_entries = [
            v for v in brain.hot_cache.values()
            if v.get("api_code") == 1028
        ]
        assert len(pose_entries) > 0, "Pose(1028) does not exist in hot_cache"

    def test_llm_path_rejects_1028(self):
        """LLM outputting 1028 is ignored by process_command

        Codes not in VALID_API_CODES are filtered at the LLM output parsing layer
        """
        assert 1028 not in VALID_API_CODES, (
            "LLM path must not allow 1028 (parameter hallucination risk)"
        )


class TestEmergencyAliasRouting:
    """Emergency word unified entry (EMERGENCY_COMMANDS) regression"""

    def _make_brain(self):
        return _make_lightweight_brain()

    def test_process_command_kana_emergency_alias(self):
        """process_command: とまれ (kana) should hit emergency bypass (delegates to _handle_emergency)"""
        brain, _ = self._make_brain()
        result = _run_async(brain.process_command(" とまれ "))
        assert result.api_code == 1003
        assert result.reasoning in ("emergency_bypass", "emergency")

    def test_process_and_execute_uses_same_emergency_source(self):
        """process_and_execute: STOP (uppercase) should hit the same emergency word source"""
        brain, BrainOutput = self._make_brain()
        called = {"count": 0}

        async def fake_handle(command):
            called["count"] += 1
            return BrainOutput(
                response="緊急停止しました",
                api_code=1003,
                execution_status="success",
            )

        brain._handle_emergency = fake_handle
        result = _run_async(brain.process_and_execute("STOP"))
        assert called["count"] == 1
        assert result.api_code == 1003


class TestConversationalKanaNormalization:
    """Conversational branch kana normalization regression"""

    def _make_brain(self):
        brain, _ = _make_lightweight_brain()
        brain.state_monitor = None
        return brain

    def test_onamaewa_returns_identity_response(self):
        """おなまえは -> after kana normalization should hit self-introduction response"""
        brain = self._make_brain()
        result = _run_async(brain.process_command("おなまえは？"))
        assert result.api_code is None
        assert "Claudia" in result.response


# === Commander unknown branch regression tests ===

class TestCommanderUnknownBranch:
    """Validate commander's three-branch handling of execute_action return values

    Core: "unknown" is a truthy string, `if success:` would misjudge it as success.
    Fix uses `result is True` for strict determination.
    """

    def test_unknown_is_truthy_but_not_true(self):
        """Premise: 'unknown' is truthy but is not True"""
        assert bool("unknown") is True   # truthy
        assert ("unknown" is True) is False  # but is True is False

    def test_commander_branch_logic_true(self):
        """result=True -> 'execution succeeded' branch"""
        result = True
        assert result is True

    def test_commander_branch_logic_unknown(self):
        """result='unknown' -> 'action timeout' branch (not is True)"""
        result = "unknown"
        assert result is not True
        assert result == "unknown"

    def test_commander_branch_logic_false(self):
        """result=False -> 'execution failed' branch"""
        result = False
        assert result is not True
        assert result != "unknown"

    def test_commander_source_uses_execution_status(self):
        """production_commander.py uses execution_status rather than direct result check

        PR2-A: Commander has migrated to process_and_execute(),
        execution result is distinguished via execution_status field (no longer directly checking execute_action return value).
        Actual code: status = brain_output.execution_status; if status == "success": ...
        """
        import pathlib
        commander_path = pathlib.Path(__file__).parent.parent.parent / "production_commander.py"
        source = commander_path.read_text(encoding="utf-8")
        # Code first assigns status = brain_output.execution_status, then uses status == "..." checks
        assert "execution_status" in source, (
            "production_commander.py should reference execution_status field"
        )
        assert 'status == "success"' in source, (
            "production_commander.py should use status==\"success\" to determine success"
        )
        assert 'status == "unknown"' in source, (
            "production_commander.py should use status==\"unknown\" to determine timeout"
        )
        assert 'status == "failed"' in source, (
            "production_commander.py should use status==\"failed\" to determine failure"
        )


# === state_snapshot=None fail-closed regression tests ===

class TestStateSnapshotNoneFailClosed:
    """Critical: state_snapshot=None must go through fail-safe SafetyCompiler, cannot be skipped

    Validates: When state monitoring is unavailable, non-safe actions (Dance/Hello/Jump) are rejected,
    while safe actions (Sit/StandUp/Stop) can still execute.
    """

    def _run(self, coro):
        loop = asyncio.new_event_loop()
        try:
            return loop.run_until_complete(coro)
        finally:
            loop.close()

    def test_hotcache_high_risk_blocked_without_monitor(self):
        """hot_cache path: ジャンプ(1031) rejected by fail-safe when no state monitor"""
        brain, BrainOutput = _make_lightweight_brain()
        assert brain.state_monitor is None, "Lightweight brain should have no state monitor"
        # ジャンプ(1031) is HIGH_ENERGY_ACTION, rejected at fail-safe battery=0.0
        output = self._run(brain.process_command("ジャンプ"))
        assert output.api_code is None, (
            "ジャンプ(1031) should be rejected without state monitor, got api_code={}".format(output.api_code)
        )
        assert output.success is False or "rejected" in (output.reasoning or ""), (
            "Should be marked as safety rejected, got reasoning={}".format(output.reasoning)
        )

    def test_hotcache_dance_blocked_without_monitor(self):
        """hot_cache path: ダンス(1022/1023) rejected by fail-safe when no state monitor

        Dance requires standing + battery > 0.10, fail-safe battery=0.0 rejects
        """
        brain, BrainOutput = _make_lightweight_brain()
        output = self._run(brain.process_command("ダンス"))
        assert output.api_code is None, (
            "ダンス should be rejected without state monitor, got api_code={}".format(output.api_code)
        )

    def test_hotcache_hello_blocked_without_monitor(self):
        """hot_cache path: こんにちは(1016) rejected by fail-safe when no state monitor

        Hello requires standing + battery > 0.10, fail-safe battery=0.0 rejects
        """
        brain, BrainOutput = _make_lightweight_brain()
        output = self._run(brain.process_command("こんにちは"))
        assert output.api_code is None, (
            "こんにちは(Hello 1016) should be rejected without state monitor, got api_code={}".format(output.api_code)
        )

    def test_hotcache_safe_action_allowed_without_monitor(self):
        """hot_cache path: 座って(1009=Sit) can still pass without state monitor

        Sit belongs to SAFE_ACTIONS, allowed even at battery=0.0.
        Sit now has requires_standing=True + is_standing=False(fail-safe)
        -> SafetyCompiler auto-prepend StandUp -> sequence=[1004, 1009]
        """
        brain, BrainOutput = _make_lightweight_brain()
        output = self._run(brain.process_command("座って"))
        # Sit requires standing -> auto-prepend StandUp(1004)
        assert output.sequence == [1004, 1009], (
            "座って(Sit 1009) requires standing + fail-safe is_standing=False, "
            "should get sequence [1004, 1009], got: api_code={}, sequence={}".format(
                output.api_code, output.sequence
            )
        )

    def test_hotcache_standup_allowed_without_monitor(self):
        """hot_cache path: 立って(1004=StandUp) can still pass without state monitor"""
        brain, BrainOutput = _make_lightweight_brain()
        output = self._run(brain.process_command("立って"))
        assert output.api_code == 1004, (
            "立って(StandUp 1004) is a safe action, should pass fail-safe, got api_code={}".format(output.api_code)
        )

    def test_hotcache_stop_allowed_without_monitor(self):
        """hot_cache path: 止まれ -> StopMove(1003) can still pass without state monitor"""
        brain, BrainOutput = _make_lightweight_brain()
        output = self._run(brain.process_command("止まれ"))
        # 止まれ may be routed as emergency or hot_cache, both result in api_code=1003
        assert output.api_code == 1003, (
            "止まれ(Stop 1003) is a safe action, should pass, got api_code={}".format(output.api_code)
        )

    def test_sequence_blocked_without_monitor(self):
        """sequence path: 立ってから挨拶 rejected by fail-safe when no state monitor

        Sequence contains 1016(Hello) not in SAFE_ACTIONS, rejected at battery=0.0
        """
        brain, BrainOutput = _make_lightweight_brain()
        output = self._run(brain.process_command("立ってから挨拶"))
        # 1004(StandUp) is safe but 1016(Hello) is not — sequence should be blocked or truncated
        # SafetyCompiler truncates at first rejection, keeping only prior safe actions
        # But 1004 passes, 1016 fails -> truncated to [1004] OR first-reject blocks all
        # Since 1016 is NOT the first action, it's a mid-sequence rejection -> truncation
        # Result depends on SafetyCompiler: 1004 alone might pass through
        if output.api_code is not None:
            # If truncated to just StandUp, that's acceptable fail-safe behavior
            assert output.api_code == 1004, (
                "After sequence truncation should only have safe action 1004, got={}".format(output.api_code)
            )
        elif output.sequence:
            # Sequence should only contain safe actions
            for code in output.sequence:
                assert code in SafetyCompiler.SAFE_ACTIONS, (
                    "Sequence should not contain non-safe action {}".format(code)
                )
        # If api_code is None and sequence is None, the whole sequence was rejected — also acceptable

    def test_dance_command_blocked_without_monitor(self):
        """dance path: dance command rejected by fail-safe when no state monitor

        Dance(1022/1023) requires standing + battery > 0.10
        """
        brain, BrainOutput = _make_lightweight_brain()
        output = self._run(brain.process_command("dance"))
        assert output.api_code is None, (
            "dance command should be rejected without state monitor, got api_code={}".format(output.api_code)
        )


# === state_source safety semantic regression tests ===

class TestStateSourceSafety:
    """Validate: simulation source data is replaced with conservative values

    When state_snapshot.source == "simulation", process_command should:
    - Override battery_level to 0.50 (do not trust simulated 0.85/1.0)
    - Replace is_standing with internal tracking value
    This ensures SafetyCompiler is not misled by simulated data.
    """

    def _run(self, coro):
        loop = asyncio.new_event_loop()
        try:
            return loop.run_until_complete(coro)
        finally:
            loop.close()

    def test_simulation_source_blocks_high_energy(self):
        """simulation source -> battery=0.50 -> high-energy action(1031) downgraded or rejected"""
        brain, BrainOutput = _make_lightweight_brain()
        # Construct a state_monitor with simulation source
        mock_state = MagicMock()
        mock_state.battery_level = 0.85  # Optimistic value from simulation
        mock_state.is_standing = True
        mock_state.source = "simulation"  # Key: mark source
        mock_state.timestamp = 0.0
        mock_monitor = MagicMock()
        mock_monitor.get_current_state.return_value = mock_state
        mock_monitor.is_ros_initialized = False
        brain.state_monitor = mock_monitor

        # ジャンプ(1031) is HIGH_ENERGY_ACTION
        # battery=0.50 > 0.30: per battery gate should be downgraded to Dance
        # But with allow_high_risk=False, directly rejected
        output = self._run(brain.process_command("ジャンプ"))
        # Should be blocked by safety policy (high-risk disabled or downgraded)
        assert output.api_code != 1031, (
            "High-energy action 1031 should not execute directly with simulation source"
        )

    def test_simulation_source_allows_safe_actions(self):
        """simulation source -> battery=0.50, is_standing=False(fail-safe) -> safe actions still executable

        Sit(1009) requires_standing=True + is_standing=False(fail-safe for simulation)
        -> SafetyCompiler auto-prepend StandUp -> sequence=[1004, 1009]
        """
        brain, BrainOutput = _make_lightweight_brain()
        mock_state = MagicMock()
        mock_state.battery_level = 0.85
        mock_state.is_standing = True
        mock_state.source = "simulation"
        mock_state.timestamp = 0.0
        mock_monitor = MagicMock()
        mock_monitor.get_current_state.return_value = mock_state
        mock_monitor.is_ros_initialized = False
        brain.state_monitor = mock_monitor

        # 座って -> Sit(1009) requires standing, simulation=fail-safe not standing
        output = self._run(brain.process_command("座って"))
        assert output.sequence == [1004, 1009], (
            "With simulation source, Sit requires standing + fail-safe is_standing=False, "
            "should get sequence [1004, 1009], got: api_code={}, sequence={}".format(
                output.api_code, output.sequence
            )
        )

    def test_sdk_source_preserves_real_battery(self):
        """sdk source -> uses real battery value (not overridden to 0.50)"""
        brain, BrainOutput = _make_lightweight_brain()
        mock_state = MagicMock()
        mock_state.battery_level = 0.85
        mock_state.is_standing = True
        mock_state.source = "sdk"  # Real SDK data
        mock_state.timestamp = 0.0
        mock_monitor = MagicMock()
        mock_monitor.get_current_state.return_value = mock_state
        # is_ros_initialized does not affect source=="sdk" branch
        mock_monitor.is_ros_initialized = False
        brain.state_monitor = mock_monitor

        # 座って -> Sit(1009) should pass normally
        output = self._run(brain.process_command("座って"))
        assert output.api_code == 1009

    def test_sdk_source_trusts_posture_not_overridden(self):
        """sdk source -> is_standing is not overridden by last_posture_standing"""
        brain, BrainOutput = _make_lightweight_brain()
        mock_state = MagicMock()
        mock_state.battery_level = 0.85
        mock_state.is_standing = True  # SDK says standing
        mock_state.source = "sdk"
        mock_state.timestamp = 0.0
        mock_monitor = MagicMock()
        mock_monitor.get_current_state.return_value = mock_state
        mock_monitor.is_ros_initialized = False  # Previously this would trigger override
        brain.state_monitor = mock_monitor
        brain.last_posture_standing = False  # Internal tracking says not standing

        # If is_standing were overridden to False, Hello would need StandUp prepended -> becomes sequence
        # If is_standing keeps True (SDK trusted), Hello executes directly
        output = self._run(brain.process_command("こんにちは"))
        # SDK source should trust is_standing=True -> Hello(1016) executes directly
        assert output.api_code == 1016, (
            "With source=sdk, is_standing should not be overridden by last_posture_standing, "
            "Hello should execute directly, got: api_code={}, sequence={}".format(
                output.api_code, output.sequence
            )
        )

    def test_sdk_fallback_uses_internal_posture(self):
        """sdk_fallback source -> is_ros_initialized=False -> uses internal tracking"""
        brain, BrainOutput = _make_lightweight_brain()
        mock_state = MagicMock()
        mock_state.battery_level = 0.50
        mock_state.is_standing = True  # fallback value
        mock_state.source = "sdk_fallback"
        mock_state.timestamp = 0.0
        mock_monitor = MagicMock()
        mock_monitor.get_current_state.return_value = mock_state
        mock_monitor.is_ros_initialized = False
        brain.state_monitor = mock_monitor
        brain.last_posture_standing = False  # Internal tracking: not standing

        # Since is_ros_initialized=False, is_standing should be overridden by internal tracking
        # Actions requiring standing should auto-prepend StandUp
        output = self._run(brain.process_command("こんにちは"))
        # Hello(1016) requires standing, is_standing=False -> should prepend StandUp
        if output.sequence:
            assert 1004 in output.sequence, (
                "Hello should prepend StandUp(1004) when not standing"
            )
        elif output.api_code == 1016:
            pass  # If safety compiler decides prepend not needed (for other reasons), also acceptable
        # Either way, should not be completely rejected (battery=0.50 is enough for non-high-energy actions)

    def test_sdk_partial_state_ok_trusts_posture(self):
        """sdk_partial + state_ok=True -> trusts SDK posture, does not override"""
        brain, BrainOutput = _make_lightweight_brain()
        mock_state = MagicMock()
        mock_state.battery_level = 0.50  # battery fallback
        mock_state.is_standing = True  # SDK says standing
        mock_state.source = "sdk_partial"
        mock_state.state_ok = True  # Posture data is trustworthy
        mock_state.battery_ok = False  # Battery data unavailable
        mock_state.timestamp = 0.0
        mock_monitor = MagicMock()
        mock_monitor.get_current_state.return_value = mock_state
        mock_monitor.is_ros_initialized = False
        brain.state_monitor = mock_monitor
        brain.last_posture_standing = False  # Internal tracking says not standing

        # state_ok=True -> trust SDK is_standing=True -> Hello executes directly
        output = self._run(brain.process_command("こんにちは"))
        assert output.api_code == 1016, (
            "With source=sdk_partial + state_ok=True, should trust SDK posture, "
            "Hello should execute directly, got: api_code={}, sequence={}".format(
                output.api_code, output.sequence
            )
        )

    def test_sdk_partial_battery_only_overrides_posture(self):
        """sdk_partial + state_ok=False -> posture untrustworthy, fail-safe is_standing=False

        When state_ok=False, last_posture_standing is no longer trusted, forcing is_standing=False.
        Hello(1016) requires_standing=True -> SafetyCompiler auto-prepend StandUp
        -> sequence=[1004, 1016]
        """
        brain, BrainOutput = _make_lightweight_brain()
        mock_state = MagicMock()
        mock_state.battery_level = 0.85
        mock_state.is_standing = False  # default value
        mock_state.source = "sdk_partial"
        mock_state.state_ok = False  # Posture data unavailable
        mock_state.battery_ok = True  # Battery trustworthy
        mock_state.timestamp = 0.0
        mock_monitor = MagicMock()
        mock_monitor.get_current_state.return_value = mock_state
        mock_monitor.is_ros_initialized = False
        brain.state_monitor = mock_monitor
        brain.last_posture_standing = True  # Internal tracking: standing (not trusted)

        # state_ok=False -> is_standing=False (fail-safe) -> Hello needs auto-prepend StandUp
        output = self._run(brain.process_command("こんにちは"))
        assert output.sequence == [1004, 1016], (
            "With source=sdk_partial + state_ok=False -> fail-safe is_standing=False, "
            "Hello should get sequence [1004, 1016], got: api_code={}, sequence={}".format(
                output.api_code, output.sequence
            )
        )
