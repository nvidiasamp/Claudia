#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_commander_migration.py — PR2 Slice A: Commander migration validation

Validates:
  - A1: Commander uses process_and_execute (no longer directly calls process_command + execute_action)
  - A2: execution_status completeness (success/unknown/failed/skipped)
  - A3: Soft deprecation warning (contextvars coroutine safe)
"""

import sys
import os
import asyncio
import logging
import re

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from unittest.mock import AsyncMock, MagicMock, patch
from claudia.brain.production_brain import ProductionBrain, BrainOutput, _pae_depth


# === A1: Commander source-level check ===

class TestCommanderSourceMigration:
    """Confirm commander no longer directly calls process_command + execute_action"""

    def test_commander_uses_process_and_execute(self):
        """production_commander.py should not have direct calls to self.brain.process_command"""
        commander_path = os.path.join(
            os.path.dirname(__file__), '..', '..', 'production_commander.py'
        )
        with open(commander_path, 'r', encoding='utf-8') as f:
            source = f.read()

        # Should not have direct call to self.brain.process_command (excluding comment references in warmup)
        # Exclude comment lines and explanatory comments in warmup method
        lines = source.split('\n')
        violations = []
        for i, line in enumerate(lines, 1):
            stripped = line.strip()
            # Skip comments
            if stripped.startswith('#') or stripped.startswith('//'):
                continue
            # Check for direct calls (not method definitions)
            if 'self.brain.process_command(' in stripped:
                violations.append(f"line {i}: {stripped}")

        assert not violations, (
            "Commander still directly calls brain.process_command:\n"
            + "\n".join(violations)
        )

    def test_commander_no_direct_execute_action(self):
        """production_commander.py should not have direct calls to self.brain.execute_action"""
        commander_path = os.path.join(
            os.path.dirname(__file__), '..', '..', 'production_commander.py'
        )
        with open(commander_path, 'r', encoding='utf-8') as f:
            source = f.read()

        lines = source.split('\n')
        violations = []
        for i, line in enumerate(lines, 1):
            stripped = line.strip()
            if stripped.startswith('#'):
                continue
            if 'self.brain.execute_action(' in stripped:
                violations.append(f"line {i}: {stripped}")

        assert not violations, (
            "Commander still directly calls brain.execute_action:\n"
            + "\n".join(violations)
        )


# === A2: execution_status completeness ===

class TestExecutionStatus:
    """Validate process_and_execute execution_status semantics"""

    def _make_brain(self):
        """Create a minimal mock brain for testing"""
        with patch.object(ProductionBrain, '__init__', lambda self, **kw: None):
            brain = ProductionBrain.__new__(ProductionBrain)
        brain.logger = logging.getLogger("test")
        brain._command_lock = asyncio.Lock()
        brain.sport_client = None
        brain.use_real_hardware = False
        brain.EMERGENCY_COMMANDS = {
            "止まれ": "止まります",
            "stop": "止まります",
        }
        return brain

    def test_execution_status_skipped_for_text_only(self):
        """Text-only response (no api_code/sequence) -> execution_status='skipped'"""
        brain = self._make_brain()
        text_output = BrainOutput(response="こんにちは")

        async def mock_process_command(cmd):
            return text_output

        brain.process_command = mock_process_command

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(brain.process_and_execute("hello"))
            assert result.execution_status == "skipped", (
                f"Expected 'skipped', got '{result.execution_status}'"
            )
        finally:
            loop.close()

    def test_execution_status_success(self):
        """Action execution succeeded -> execution_status='success'"""
        brain = self._make_brain()
        action_output = BrainOutput(response="座ります", api_code=1009)

        async def mock_process_command(cmd):
            return action_output

        async def mock_execute_action(output):
            return True

        brain.process_command = mock_process_command
        brain.execute_action = mock_execute_action

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(brain.process_and_execute("おすわり"))
            assert result.execution_status == "success"
        finally:
            loop.close()

    def test_execution_status_unknown(self):
        """Action timeout -> execution_status='unknown'"""
        brain = self._make_brain()
        action_output = BrainOutput(response="ダンスします", api_code=1022)

        async def mock_process_command(cmd):
            return action_output

        async def mock_execute_action(output):
            return "unknown"

        brain.process_command = mock_process_command
        brain.execute_action = mock_execute_action

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(brain.process_and_execute("踊って"))
            assert result.execution_status == "unknown"
        finally:
            loop.close()

    def test_execution_status_failed(self):
        """Action execution failed -> execution_status='failed'"""
        brain = self._make_brain()
        action_output = BrainOutput(response="立ちます", api_code=1004)

        async def mock_process_command(cmd):
            return action_output

        async def mock_execute_action(output):
            return False

        brain.process_command = mock_process_command
        brain.execute_action = mock_execute_action

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(brain.process_and_execute("立って"))
            assert result.execution_status == "failed"
        finally:
            loop.close()

    def test_emergency_sets_execution_status(self):
        """Emergency stop -> execution_status is not None"""
        brain = self._make_brain()

        async def mock_handle_emergency(cmd):
            return BrainOutput(
                response="緊急停止しました",
                api_code=1003,
                execution_status="success",
            )

        brain._handle_emergency = mock_handle_emergency

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(brain.process_and_execute("止まれ"))
            assert result.execution_status is not None
        finally:
            loop.close()


# === A3: Soft deprecation warning (contextvars safe) ===

class TestDeprecationWarning:
    """Validate process_command produces warning when called outside process_and_execute"""

    def _make_brain(self):
        with patch.object(ProductionBrain, '__init__', lambda self, **kw: None):
            brain = ProductionBrain.__new__(ProductionBrain)
        brain.logger = logging.getLogger("test_deprecation")
        brain._command_lock = asyncio.Lock()
        brain.sport_client = None
        brain.use_real_hardware = False
        brain.safety_compiler = None
        brain.state_monitor = None
        brain.audit_logger = None
        brain.model_7b = "test-model"
        brain.last_posture_standing = False
        brain.EMERGENCY_COMMANDS = {}
        return brain

    def test_deprecation_warning_logged(self):
        """Directly calling process_command should produce a warning log"""
        # Ensure _pae_depth is 0 (not in process_and_execute context)
        assert _pae_depth.get(0) == 0, "Test precondition: _pae_depth should be 0"

        brain = self._make_brain()
        with patch.object(brain.logger, 'warning') as mock_warn:
            # Since process_command has complex internal logic, mock the rest
            # Only need to verify the first line warning is triggered
            try:
                loop = asyncio.new_event_loop()
                loop.run_until_complete(brain.process_command("test"))
            except Exception:
                pass  # process_command will fail due to missing other attributes
            finally:
                loop.close()

            # Check for warning
            warn_calls = [
                str(c) for c in mock_warn.call_args_list
                if 'process_and_execute' in str(c)
            ]
            assert len(warn_calls) > 0, (
                "Expected deprecation warning about process_and_execute, "
                f"got calls: {mock_warn.call_args_list}"
            )

    def test_no_warning_inside_pae_context(self):
        """Calling process_command inside process_and_execute should not produce warning"""
        brain = self._make_brain()

        # Simulate process_and_execute context
        token = _pae_depth.set(1)
        try:
            with patch.object(brain.logger, 'warning') as mock_warn:
                try:
                    loop = asyncio.new_event_loop()
                    loop.run_until_complete(brain.process_command("test"))
                except Exception:
                    pass
                finally:
                    loop.close()

                # Should not have process_and_execute related warnings
                pae_warns = [
                    str(c) for c in mock_warn.call_args_list
                    if 'process_and_execute' in str(c)
                ]
                assert len(pae_warns) == 0, (
                    f"Unexpected deprecation warning inside PAE context: {pae_warns}"
                )
        finally:
            _pae_depth.reset(token)

    def test_contextvars_coroutine_isolation(self):
        """Different coroutines' _pae_depth do not interfere with each other"""
        results = {}

        async def check_depth(name, expected):
            results[name] = _pae_depth.get(0) == expected

        async def run():
            # Main coroutine sets depth=1
            token = _pae_depth.set(1)
            try:
                # Child task should inherit depth=1
                task1 = asyncio.ensure_future(check_depth("inherited", 1))
                await task1

                # Child task internal modification does not affect parent
                async def modify_and_check():
                    inner_token = _pae_depth.set(99)
                    results["inner"] = _pae_depth.get(0) == 99
                    _pae_depth.reset(inner_token)

                task2 = asyncio.ensure_future(modify_and_check())
                await task2

                # Parent coroutine depth still 1
                results["parent_unchanged"] = _pae_depth.get(0) == 1
            finally:
                _pae_depth.reset(token)

        loop = asyncio.new_event_loop()
        try:
            loop.run_until_complete(run())
        finally:
            loop.close()

        assert results.get("inherited"), "Child task should inherit parent's _pae_depth"
        assert results.get("inner"), "Inner task should see its own modified _pae_depth"
        assert results.get("parent_unchanged"), "Parent's _pae_depth should be unchanged"
