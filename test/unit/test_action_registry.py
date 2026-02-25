#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_action_registry.py — action_registry unit tests

Validates:
  - Derived sets are consistent with ActionDef attributes
  - VALID_API_CODES does not contain parameterized actions
  - EXECUTABLE_API_CODES includes actions with safe_default_params
  - METHOD_MAP only contains enabled actions
  - generate_modelfile_action_block() output is consistent with VALID_API_CODES
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from claudia.brain.action_registry import (
    _ACTIONS, ACTION_REGISTRY,
    VALID_API_CODES, EXECUTABLE_API_CODES, SAFE_DEFAULT_PARAMS,
    REQUIRE_STANDING, HIGH_ENERGY_ACTIONS, ACTION_MODEL_API_CODES,
    METHOD_MAP, ACTION_RESPONSES,
    get_response_for_action, get_response_for_sequence,
    generate_modelfile_action_block,
)


class TestDerivedSets:
    """Derived set consistency"""

    def test_valid_api_codes_no_params(self):
        """VALID_API_CODES does not include has_params actions"""
        for a in _ACTIONS:
            if a.has_params:
                assert a.api_code not in VALID_API_CODES, (
                    "has_params=True action {} should not be in VALID_API_CODES".format(a.api_code)
                )

    def test_valid_api_codes_only_enabled(self):
        """VALID_API_CODES only includes enabled actions"""
        for a in _ACTIONS:
            if not a.enabled:
                assert a.api_code not in VALID_API_CODES, (
                    "enabled=False action {} should not be in VALID_API_CODES".format(a.api_code)
                )

    def test_valid_api_codes_completeness(self):
        """All non-parameterized + enabled actions are in VALID_API_CODES"""
        expected = {a.api_code for a in _ACTIONS if a.enabled and not a.has_params}
        assert VALID_API_CODES == expected

    def test_executable_includes_safe_default_params(self):
        """EXECUTABLE_API_CODES includes parameterized actions with safe_default_params"""
        for a in _ACTIONS:
            if a.enabled and a.has_params and a.safe_default_params is not None:
                assert a.api_code in EXECUTABLE_API_CODES, (
                    "Action {} with safe_default_params should be in EXECUTABLE_API_CODES".format(a.api_code)
                )

    def test_executable_excludes_params_without_defaults(self):
        """EXECUTABLE_API_CODES does not include parameterized actions without safe_default_params"""
        for a in _ACTIONS:
            if a.enabled and a.has_params and a.safe_default_params is None:
                assert a.api_code not in EXECUTABLE_API_CODES, (
                    "Action {} without safe_default_params should not be in EXECUTABLE_API_CODES".format(a.api_code)
                )

    def test_executable_superset_of_valid(self):
        """EXECUTABLE_API_CODES is a superset of VALID_API_CODES"""
        assert VALID_API_CODES.issubset(EXECUTABLE_API_CODES)

    def test_pose_1028_in_executable_not_valid(self):
        """Pose(1028) is in EXECUTABLE but not in VALID (core dual-layer whitelist validation)"""
        assert 1028 in EXECUTABLE_API_CODES
        assert 1028 not in VALID_API_CODES

    def test_safe_default_params_dict(self):
        """SAFE_DEFAULT_PARAMS only contains enabled parameterized actions with safe_default_params"""
        for api_code, params in SAFE_DEFAULT_PARAMS.items():
            a = ACTION_REGISTRY[api_code]
            assert a.enabled
            assert a.has_params
            assert a.safe_default_params is not None
            assert params == a.safe_default_params

    def test_require_standing_matches(self):
        """REQUIRE_STANDING matches requires_standing=True"""
        expected = {a.api_code for a in _ACTIONS if a.requires_standing}
        assert REQUIRE_STANDING == expected

    def test_sit_1009_explicitly_requires_standing(self):
        """Policy constraint: Sit(1009) explicitly requires standing prerequisite"""
        assert 1009 in REQUIRE_STANDING, (
            "Sit(1009) should be in REQUIRE_STANDING "
            "to trigger SafetyCompiler auto-prepend of StandUp when state is untrusted"
        )

    def test_high_energy_matches(self):
        """HIGH_ENERGY_ACTIONS matches risk_level='high'"""
        expected = {a.api_code for a in _ACTIONS if a.risk_level == "high"}
        assert HIGH_ENERGY_ACTIONS == expected

    def test_method_map_only_enabled(self):
        """METHOD_MAP only includes enabled actions"""
        for a in _ACTIONS:
            if a.enabled:
                assert a.api_code in METHOD_MAP
                assert METHOD_MAP[a.api_code] == a.method
            else:
                assert a.api_code not in METHOD_MAP

    def test_action_responses_only_enabled(self):
        """ACTION_RESPONSES only includes enabled actions"""
        for a in _ACTIONS:
            if a.enabled:
                assert a.api_code in ACTION_RESPONSES
            else:
                assert a.api_code not in ACTION_RESPONSES


class TestResponseHelpers:
    """Response helper functions"""

    def test_get_response_known(self):
        assert get_response_for_action(1004) == "立ちます"

    def test_get_response_unknown(self):
        assert get_response_for_action(9999) == "はい、わかりました"

    def test_get_response_for_sequence(self):
        result = get_response_for_sequence([1004, 1016])
        assert "立ちます" in result
        assert "挨拶します" in result


class TestModelfileGeneration:
    """Modelfile generation"""

    def test_generate_nonempty(self):
        block = generate_modelfile_action_block()
        assert len(block) > 0

    def test_generate_contains_all_valid(self):
        """Generated action list contains all VALID_API_CODES"""
        block = generate_modelfile_action_block()
        for code in VALID_API_CODES:
            assert str(code) in block, (
                "Code {} from VALID_API_CODES not found in generated Modelfile action list".format(code)
            )

    def test_generate_no_params_actions(self):
        """Generated action list does not contain parameterized actions"""
        block = generate_modelfile_action_block()
        for a in _ACTIONS:
            if a.has_params:
                # Parameterized action api_code should not appear as a standalone entry
                entry = "{}=".format(a.api_code)
                assert entry not in block, (
                    "Parameterized action {} should not appear in Modelfile generation".format(a.api_code)
                )


class TestRegistryIntegrity:
    """Registry integrity"""

    def test_no_duplicate_api_codes(self):
        codes = [a.api_code for a in _ACTIONS]
        assert len(codes) == len(set(codes)), "Duplicate api_code found"

    def test_no_duplicate_methods(self):
        methods = [a.method for a in _ACTIONS]
        assert len(methods) == len(set(methods)), "Duplicate method found"

    def test_all_enabled_have_method_name(self):
        for a in _ACTIONS:
            if a.enabled:
                assert a.method, "Enabled action {} is missing method".format(a.api_code)

    def test_all_have_name_ja(self):
        for a in _ACTIONS:
            assert a.name_ja, "Action {} is missing name_ja".format(a.api_code)

    def test_disabled_not_in_executable(self):
        """Disabled actions are not in any executable whitelist"""
        for a in _ACTIONS:
            if not a.enabled:
                assert a.api_code not in VALID_API_CODES
                assert a.api_code not in EXECUTABLE_API_CODES
                assert a.api_code not in METHOD_MAP


class TestActionModelApiCodes:
    """PR2: ACTION_MODEL_API_CODES consistency"""

    def test_subset_of_valid(self):
        """ACTION_MODEL_API_CODES is a subset of VALID_API_CODES"""
        assert ACTION_MODEL_API_CODES <= VALID_API_CODES

    def test_no_high_risk(self):
        """ACTION_MODEL_API_CODES does not contain high-risk actions"""
        overlap = ACTION_MODEL_API_CODES & HIGH_ENERGY_ACTIONS
        assert not overlap, (
            "ACTION_MODEL_API_CODES should not contain high-risk actions: {}".format(overlap))

    def test_excludes_exactly_high_risk(self):
        """VALID - ACTION_MODEL = only high-risk actions"""
        diff = VALID_API_CODES - ACTION_MODEL_API_CODES
        expected_high = frozenset(
            a.api_code for a in _ACTIONS
            if a.enabled and not a.has_params and a.risk_level == "high"
        )
        assert diff == expected_high, (
            "VALID - ACTION_MODEL should be exactly the high-risk action set, "
            "diff: {}".format(diff ^ expected_high))

    def test_generate_no_high_risk(self):
        """generate_modelfile_action_block(include_high_risk=False) does not contain high-risk codes"""
        block = generate_modelfile_action_block(include_high_risk=False)
        for code in HIGH_ENERGY_ACTIONS:
            # Ensure high-risk codes do not appear as standalone entries
            # Note: 1030 should not appear as "1030="
            assert "{}=".format(code) not in block, (
                "High-risk action {} should not appear in Action model list".format(code))

    def test_generate_includes_safe_actions(self):
        """generate_modelfile_action_block(include_high_risk=False) includes safe actions"""
        block = generate_modelfile_action_block(include_high_risk=False)
        for code in ACTION_MODEL_API_CODES:
            assert str(code) in block, (
                "Safe action {} not found in Action model list".format(code))

    def test_modelfile_action_list_consistent(self):
        """ClaudiaAction modelfile action list is consistent with ACTION_MODEL_API_CODES"""
        import re
        modelfile_path = os.path.join(
            os.path.dirname(__file__), '..', '..', 'models',
            'ClaudiaAction_v3.0')
        with open(modelfile_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Match "NNNN=" format action definitions only within SYSTEM section
        # Avoid false matches on PARAMETER values (e.g., num_ctx=1024)
        codes_in_file = set()
        for match in re.finditer(r'(\d{4})=', content):
            code = int(match.group(1))
            if 1000 <= code <= 1099:
                codes_in_file.add(code)

        # Codes in modelfile should be a subset of ACTION_MODEL_API_CODES
        unexpected = codes_in_file - ACTION_MODEL_API_CODES
        assert not unexpected, (
            "Modelfile contains codes not in ACTION_MODEL_API_CODES: {}. "
            "If these are high-risk actions, please remove them from the modelfile.".format(unexpected))

        # Codes in ACTION_MODEL_API_CODES should appear in modelfile
        missing = ACTION_MODEL_API_CODES - codes_in_file
        assert not missing, (
            "Codes {} from ACTION_MODEL_API_CODES not found in modelfile. "
            "Please update the modelfile or registry.".format(missing))

    def test_7b_modelfile_action_list_consistent(self):
        """ClaudiaIntelligent 7B modelfile action list is consistent with VALID_API_CODES

        The 7B model can output any code in VALID_API_CODES (including high-risk).
        The modelfile should not contain parameterized action codes (they are filtered
        at runtime, wasting inference tokens).
        """
        import re
        modelfile_path = os.path.join(
            os.path.dirname(__file__), '..', '..', 'models',
            'ClaudiaIntelligent_7B_v2.0')
        with open(modelfile_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Extract "NNNN=" format action definitions in SYSTEM block (exclude PARAMETER line values)
        codes_in_file = set()
        for match in re.finditer(r'(\d{4})=', content):
            code = int(match.group(1))
            if 1000 <= code <= 1099:
                codes_in_file.add(code)

        # Codes in modelfile should be a subset of VALID_API_CODES (should not contain parameterized actions)
        unexpected = codes_in_file - VALID_API_CODES
        assert not unexpected, (
            "7B Modelfile contains codes not in VALID_API_CODES: {}. "
            "Parameterized actions (1008/1015/1028 etc.) are filtered at runtime "
            "and should not appear in the modelfile."
            .format(unexpected))

        # Codes in VALID_API_CODES should appear in modelfile (ensure the model can learn all valid actions)
        missing = VALID_API_CODES - codes_in_file
        assert not missing, (
            "Codes {} from VALID_API_CODES not found in 7B modelfile. "
            "Please update the modelfile so the model can learn these actions.".format(missing))
