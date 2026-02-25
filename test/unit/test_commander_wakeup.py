#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_commander_wakeup.py — Wakeup animation unit tests

Validates:
  - Environment variable gating (disabled by default, enabled with COMMANDER_WAKEUP_ANIMATION=1)
  - Non-hardware mode skip
  - StandUp success (code=0) -> Stretch -> posture update
  - StandUp already standing (code=-1) -> Stretch -> posture update
  - StandUp timeout (code=3104) -> _verify_standing_after_unknown -> conditional continue
  - StandUp failure (code=3103) -> skip Stretch
  - Exceptions do not block startup
  - Audit log recording
"""

import sys
import os
import asyncio
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from unittest.mock import MagicMock, AsyncMock, patch, call
from types import SimpleNamespace


def _run(coro):
    loop = asyncio.new_event_loop()
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


def _make_commander(use_real_hardware=True, sport_client=True, router_mode="legacy"):
    """Build mock Commander instance"""
    from production_commander import ProductionCommander

    with patch.object(ProductionCommander, '__init__', lambda self, **kw: None):
        cmd = ProductionCommander()

    cmd.brain = MagicMock()
    cmd.brain.use_real_hardware = use_real_hardware
    cmd.brain.sport_client = MagicMock() if sport_client else None
    cmd.brain._update_posture_tracking = MagicMock()
    cmd.brain._verify_standing_after_unknown = AsyncMock(return_value=True)
    cmd.brain.model_7b = "test-model"

    # router mode
    from claudia.brain.channel_router import RouterMode
    cmd.brain._router_mode = RouterMode(router_mode) if router_mode != "legacy" else RouterMode.LEGACY
    cmd.brain._channel_router = MagicMock()
    cmd.brain._channel_router._action_model = "test-action-model"

    return cmd


# === Gating tests ===

class TestWakeupGating:
    """Environment variable and hardware mode gating"""

    def test_skipped_when_env_not_set(self):
        """Skips animation when COMMANDER_WAKEUP_ANIMATION is not set (default)"""
        cmd = _make_commander()
        os.environ.pop("COMMANDER_WAKEUP_ANIMATION", None)
        _run(cmd._wakeup_animation())
        cmd.brain._rpc_call.assert_not_called()

    def test_skipped_when_env_is_zero(self):
        """Skips animation when COMMANDER_WAKEUP_ANIMATION=0"""
        cmd = _make_commander()
        os.environ["COMMANDER_WAKEUP_ANIMATION"] = "0"
        try:
            _run(cmd._wakeup_animation())
            cmd.brain._rpc_call.assert_not_called()
        finally:
            os.environ.pop("COMMANDER_WAKEUP_ANIMATION", None)

    def test_skipped_when_not_hardware(self):
        """Skips animation in simulation mode"""
        cmd = _make_commander(use_real_hardware=False)
        os.environ["COMMANDER_WAKEUP_ANIMATION"] = "1"
        try:
            _run(cmd._wakeup_animation())
            cmd.brain._rpc_call.assert_not_called()
        finally:
            os.environ.pop("COMMANDER_WAKEUP_ANIMATION", None)

    def test_skipped_when_no_sport_client(self):
        """Skips animation when SportClient is not available"""
        cmd = _make_commander(sport_client=False)
        os.environ["COMMANDER_WAKEUP_ANIMATION"] = "1"
        try:
            _run(cmd._wakeup_animation())
            cmd.brain._rpc_call.assert_not_called()
        finally:
            os.environ.pop("COMMANDER_WAKEUP_ANIMATION", None)

    def test_enabled_when_env_is_one(self):
        """Executes animation when COMMANDER_WAKEUP_ANIMATION=1"""
        cmd = _make_commander()
        cmd.brain._rpc_call.side_effect = [0, 0]  # StandUp, Stretch
        os.environ["COMMANDER_WAKEUP_ANIMATION"] = "1"
        try:
            with patch('production_commander.asyncio.sleep', new_callable=AsyncMock):
                with patch.object(cmd, '_log_wakeup_audit'):
                    _run(cmd._wakeup_animation())
            assert cmd.brain._rpc_call.call_count == 2
        finally:
            os.environ.pop("COMMANDER_WAKEUP_ANIMATION", None)


# === StandUp return code branches ===

class TestWakeupStandUpBranches:
    """Different handling paths for StandUp return codes"""

    def setup_method(self):
        os.environ["COMMANDER_WAKEUP_ANIMATION"] = "1"

    def teardown_method(self):
        os.environ.pop("COMMANDER_WAKEUP_ANIMATION", None)

    def test_standup_success_updates_posture(self):
        """StandUp code=0 -> update posture + execute Stretch"""
        cmd = _make_commander()
        cmd.brain._rpc_call.side_effect = [0, 0]  # StandUp=0, Stretch=0
        with patch('production_commander.asyncio.sleep', new_callable=AsyncMock):
            with patch.object(cmd, '_log_wakeup_audit'):
                _run(cmd._wakeup_animation())
        cmd.brain._update_posture_tracking.assert_called_once_with(1004)
        assert cmd.brain._rpc_call.call_count == 2

    def test_standup_already_standing_updates_posture(self):
        """StandUp code=-1 (already standing) -> update posture + execute Stretch"""
        cmd = _make_commander()
        cmd.brain._rpc_call.side_effect = [-1, 0]
        with patch('production_commander.asyncio.sleep', new_callable=AsyncMock):
            with patch.object(cmd, '_log_wakeup_audit'):
                _run(cmd._wakeup_animation())
        cmd.brain._update_posture_tracking.assert_called_once_with(1004)
        assert cmd.brain._rpc_call.call_count == 2

    def test_standup_3104_verifies_then_continues(self):
        """StandUp code=3104 -> _verify_standing_after_unknown -> continues after confirmation"""
        cmd = _make_commander()
        cmd.brain._rpc_call.side_effect = [3104, 0]  # StandUp=3104, Stretch=0
        cmd.brain._verify_standing_after_unknown = AsyncMock(return_value=True)
        with patch('production_commander.asyncio.sleep', new_callable=AsyncMock):
            with patch.object(cmd, '_log_wakeup_audit'):
                _run(cmd._wakeup_animation())
        cmd.brain._verify_standing_after_unknown.assert_called_once()
        cmd.brain._update_posture_tracking.assert_called_once_with(1004)
        assert cmd.brain._rpc_call.call_count == 2

    def test_standup_3104_unconfirmed_skips_stretch(self):
        """StandUp code=3104 + standing unconfirmed -> skip Stretch"""
        cmd = _make_commander()
        cmd.brain._rpc_call.side_effect = [3104]
        cmd.brain._verify_standing_after_unknown = AsyncMock(return_value=False)
        with patch('production_commander.asyncio.sleep', new_callable=AsyncMock):
            with patch.object(cmd, '_log_wakeup_audit'):
                _run(cmd._wakeup_animation())
        cmd.brain._update_posture_tracking.assert_not_called()
        assert cmd.brain._rpc_call.call_count == 1  # Only called StandUp

    def test_standup_failure_skips_stretch(self):
        """StandUp code=3103 (failure) -> skip Stretch"""
        cmd = _make_commander()
        cmd.brain._rpc_call.side_effect = [3103]
        with patch('production_commander.asyncio.sleep', new_callable=AsyncMock):
            with patch.object(cmd, '_log_wakeup_audit'):
                _run(cmd._wakeup_animation())
        cmd.brain._update_posture_tracking.assert_not_called()
        assert cmd.brain._rpc_call.call_count == 1

    def test_standup_tuple_return(self):
        """_rpc_call returns tuple (code, data) -> correctly extracts code"""
        cmd = _make_commander()
        cmd.brain._rpc_call.side_effect = [(0, {}), (0, {})]
        with patch('production_commander.asyncio.sleep', new_callable=AsyncMock):
            with patch.object(cmd, '_log_wakeup_audit'):
                _run(cmd._wakeup_animation())
        cmd.brain._update_posture_tracking.assert_called_once_with(1004)


# === Exception safety ===

class TestWakeupExceptionSafety:
    """Exceptions do not block startup"""

    def setup_method(self):
        os.environ["COMMANDER_WAKEUP_ANIMATION"] = "1"

    def teardown_method(self):
        os.environ.pop("COMMANDER_WAKEUP_ANIMATION", None)

    def test_rpc_exception_does_not_propagate(self):
        """_rpc_call exception is caught, does not block startup"""
        cmd = _make_commander()
        cmd.brain._rpc_call.side_effect = RuntimeError("DDS disconnected")
        with patch.object(cmd, '_log_wakeup_audit'):
            _run(cmd._wakeup_animation())  # Should not raise


# === Audit logging ===

class TestWakeupAudit:
    """Audit log recording"""

    def setup_method(self):
        os.environ["COMMANDER_WAKEUP_ANIMATION"] = "1"

    def teardown_method(self):
        os.environ.pop("COMMANDER_WAKEUP_ANIMATION", None)

    def test_audit_logged_on_success(self):
        """Audit is recorded on successful completion"""
        cmd = _make_commander()
        cmd.brain._rpc_call.side_effect = [0, 0]
        with patch('production_commander.asyncio.sleep', new_callable=AsyncMock):
            with patch(
                'claudia.brain.audit_logger.get_audit_logger'
            ) as mock_get:
                mock_audit = MagicMock()
                mock_get.return_value = mock_audit
                _run(cmd._wakeup_animation())
                mock_audit.log_entry.assert_called_once()
                entry = mock_audit.log_entry.call_args[0][0]
                assert entry.route == "startup"
                assert entry.input_command == "__wakeup__"
                assert entry.success is True
                assert entry.sequence == [1004, 1017]
                assert entry.safety_verdict == "ok"

    def test_audit_logged_on_standup_failure(self):
        """Audit is recorded even on StandUp failure"""
        cmd = _make_commander()
        cmd.brain._rpc_call.side_effect = [3103]
        with patch('production_commander.asyncio.sleep', new_callable=AsyncMock):
            with patch(
                'claudia.brain.audit_logger.get_audit_logger'
            ) as mock_get:
                mock_audit = MagicMock()
                mock_get.return_value = mock_audit
                _run(cmd._wakeup_animation())
                mock_audit.log_entry.assert_called_once()
                entry = mock_audit.log_entry.call_args[0][0]
                assert entry.success is False
                assert entry.safety_verdict == "standup_failed"
                assert "3103" in entry.safety_reason

    def test_audit_3104_confirmed_standing_is_success(self):
        """3104 + _verify_standing_after_unknown=True -> audit success=True

        Core regression protection: validation net for standup_confirmed semantic fix.
        """
        cmd = _make_commander()
        cmd.brain._rpc_call.side_effect = [3104, 0]  # StandUp=3104, Stretch=0
        cmd.brain._verify_standing_after_unknown = AsyncMock(return_value=True)
        with patch('production_commander.asyncio.sleep', new_callable=AsyncMock):
            with patch(
                'claudia.brain.audit_logger.get_audit_logger'
            ) as mock_get:
                mock_audit = MagicMock()
                mock_get.return_value = mock_audit
                _run(cmd._wakeup_animation())
                mock_audit.log_entry.assert_called_once()
                entry = mock_audit.log_entry.call_args[0][0]
                assert entry.success is True
                assert entry.safety_verdict == "ok"
                assert entry.sequence == [1004, 1017]

    def test_audit_3104_unconfirmed_is_failure(self):
        """3104 + _verify_standing_after_unknown=False -> audit success=False"""
        cmd = _make_commander()
        cmd.brain._rpc_call.side_effect = [3104]
        cmd.brain._verify_standing_after_unknown = AsyncMock(return_value=False)
        with patch('production_commander.asyncio.sleep', new_callable=AsyncMock):
            with patch(
                'claudia.brain.audit_logger.get_audit_logger'
            ) as mock_get:
                mock_audit = MagicMock()
                mock_get.return_value = mock_audit
                _run(cmd._wakeup_animation())
                mock_audit.log_entry.assert_called_once()
                entry = mock_audit.log_entry.call_args[0][0]
                assert entry.success is False
                assert entry.safety_verdict == "standup_failed"

    def test_audit_standup_ok_stretch_fail_is_consistent(self):
        """StandUp success + Stretch failure -> success=False + safety_verdict='stretch_failed'

        Prevents inconsistency between success and safety_verdict:
        Previously had contradictory state of success=False + safety_verdict='ok'.
        """
        cmd = _make_commander()
        cmd.brain._rpc_call.side_effect = [0, 3103]  # StandUp=0, Stretch=3103(failure)
        with patch('production_commander.asyncio.sleep', new_callable=AsyncMock):
            with patch(
                'claudia.brain.audit_logger.get_audit_logger'
            ) as mock_get:
                mock_audit = MagicMock()
                mock_get.return_value = mock_audit
                _run(cmd._wakeup_animation())
                mock_audit.log_entry.assert_called_once()
                entry = mock_audit.log_entry.call_args[0][0]
                assert entry.success is False
                assert entry.safety_verdict == "stretch_failed"
                assert "3103" in entry.safety_reason


# === Action model warmup ===

class TestActionModelWarmup:
    """Warm up Action model in Dual/Shadow mode"""

    def test_legacy_mode_no_action_warmup(self):
        """Legacy mode only warms up 7B model"""
        cmd = _make_commander(router_mode="legacy")
        call_count = 0

        def mock_chat(**kwargs):
            nonlocal call_count
            call_count += 1
            return {'message': {'content': '{}'}}

        with patch('production_commander.asyncio.sleep', new_callable=AsyncMock):
            with patch.dict('sys.modules', {'ollama': MagicMock(chat=mock_chat)}):
                _run(cmd._warmup_model())

        assert call_count == 1  # Only warmed up 7B

    def test_shadow_mode_warms_action_first_7b_last(self):
        """Shadow mode: Action warms up first -> 7B last (7B stays in VRAM, primary path for first command)"""
        cmd = _make_commander(router_mode="shadow")
        warmed_models = []

        def mock_chat(**kwargs):
            warmed_models.append(kwargs.get('model'))
            return {'message': {'content': '{}'}}

        with patch.dict('sys.modules', {'ollama': MagicMock(chat=mock_chat)}):
            _run(cmd._warmup_model())

        assert len(warmed_models) == 2
        assert warmed_models[0] == "test-action-model"  # Action first
        assert warmed_models[1] == "test-model"          # 7B last (stays in VRAM)

    def test_dual_mode_warms_action_only(self):
        """Dual mode (Action-primary): Only warms up Action model, not 7B"""
        cmd = _make_commander(router_mode="dual")
        warmed_models = []

        def mock_chat(**kwargs):
            warmed_models.append(kwargs.get('model'))
            return {'message': {'content': '{}'}}

        with patch.dict('sys.modules', {'ollama': MagicMock(chat=mock_chat)}):
            _run(cmd._warmup_model())

        assert len(warmed_models) == 1
        assert warmed_models[0] == "test-action-model"   # Action is the only warmup

    def test_shadow_first_timeout_still_warms_second(self):
        """Shadow mode: Action timeout -> 7B still warms up (timeout isolation)"""
        cmd = _make_commander(router_mode="shadow")
        warmed_models = []
        call_idx = [0]

        def mock_chat(**kwargs):
            idx = call_idx[0]
            call_idx[0] += 1
            if idx == 0:
                # Action model timeout: block long enough to trigger wait_for timeout
                import time as _time
                _time.sleep(999)
            warmed_models.append(kwargs.get('model'))
            return {'message': {'content': '{}'}}

        # wait_for timeout cancels first task, loop continues to second
        with patch.dict('sys.modules', {'ollama': MagicMock(chat=mock_chat)}):
            _run(cmd._warmup_model())

        # First timed out and was skipped, second (7B) should warm up normally
        assert len(warmed_models) == 1
        assert warmed_models[0] == "test-model"  # 7B still gets warmed up
