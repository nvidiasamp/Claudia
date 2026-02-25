#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mock SportClient -- Simulated Robot Controller (V2)

Used for testing the system without a real robot.
Returns deterministic values (P0-6 requirement), semantically consistent with real SDK return codes:
  0    = Success
  -1   = Already in target state
  3104 = RPC_ERR_CLIENT_API_TIMEOUT (async action "triggered but completion not confirmed")
  3103 = App occupied
  3203 = Not supported
"""

import logging
import time


class MockSportClient:
    """Simulated SportClient that returns deterministic responses

    All parameterless actions are aligned with METHOD_MAP in ACTION_REGISTRY.
    """

    def __init__(self):
        self.logger = logging.getLogger("MockSportClient")
        self.timeout = 10.0
        self.is_connected = False
        self.current_state = "standing"

    def SetTimeout(self, timeout):
        # type: (float) -> None
        """Set timeout"""
        self.timeout = timeout

    def Init(self):
        """Initialize (simulated)"""
        time.sleep(0.1)  # Shortened simulated delay (test-friendly)
        self.is_connected = True
        self.logger.info("MockSportClient initialized successfully")

    def _simulate_action(self, action_name, duration=0.1):
        # type: (str, float) -> int
        """Simulate action execution (deterministic return)"""
        self.logger.info("[Sim] Action executed: {}".format(action_name))
        time.sleep(duration)
        return 0

    # === Basic Postures ===

    def Damp(self):
        # type: () -> int
        """Damp mode (1001)"""
        self.current_state = "damped"
        return self._simulate_action("Damp", 0.1)

    def BalanceStand(self):
        # type: () -> int
        """Balance stand (1002)"""
        self.current_state = "standing"
        return self._simulate_action("BalanceStand", 0.1)

    def StopMove(self):
        # type: () -> int
        """Stop movement (1003)"""
        return self._simulate_action("StopMove", 0.05)

    def StandUp(self):
        # type: () -> int
        """Stand up (1004)"""
        if self.current_state == "standing":
            return -1  # Already standing
        self.current_state = "standing"
        return self._simulate_action("StandUp", 0.2)

    def StandDown(self):
        # type: () -> int
        """Lie down (1005)"""
        self.current_state = "lying"
        return self._simulate_action("StandDown", 0.2)

    def RecoveryStand(self):
        # type: () -> int
        """Recovery stand (1006)"""
        self.current_state = "standing"
        return self._simulate_action("RecoveryStand", 0.2)

    def Sit(self):
        # type: () -> int
        """Sit (1009)"""
        self.current_state = "sitting"
        return self._simulate_action("Sit", 0.2)

    def RiseSit(self):
        # type: () -> int
        """Rise from sitting (1010)"""
        self.current_state = "standing"
        return self._simulate_action("RiseSit", 0.2)

    # === Performance Actions ===

    def Hello(self):
        # type: () -> int
        """Greeting (1016)"""
        return self._simulate_action("Hello", 0.3)

    def Stretch(self):
        # type: () -> int
        """Stretch (1017)"""
        return self._simulate_action("Stretch", 0.3)

    def Wallow(self):
        # type: () -> int
        """Roll over (1021)"""
        return self._simulate_action("Wallow", 0.3)

    def Dance1(self):
        # type: () -> int
        """Dance 1 (1022) -- deterministic return 3104 (async action trigger code)"""
        self.logger.info("[Sim] Action executed: Dance1")
        time.sleep(0.3)
        return 3104

    def Dance2(self):
        # type: () -> int
        """Dance 2 (1023) -- deterministic return 3104 (async action trigger code)"""
        self.logger.info("[Sim] Action executed: Dance2")
        time.sleep(0.3)
        return 3104

    def Scrape(self):
        # type: () -> int
        """Scrape (1029)"""
        return self._simulate_action("Scrape", 0.3)

    def WiggleHips(self):
        # type: () -> int
        """Wiggle hips (1033)"""
        return self._simulate_action("WiggleHips", 0.3)

    def Heart(self):
        # type: () -> int
        """Heart gesture (1036)"""
        return self._simulate_action("Heart", 0.3)

    # === High-Risk Actions ===

    def FrontFlip(self):
        # type: () -> int
        """Front flip (1030)"""
        return self._simulate_action("FrontFlip", 0.5)

    def FrontJump(self):
        # type: () -> int
        """Front jump (1031)"""
        return self._simulate_action("FrontJump", 0.5)

    def FrontPounce(self):
        # type: () -> int
        """Front pounce (1032)"""
        return self._simulate_action("FrontPounce", 0.5)

    # === Parameterized Actions ===

    def Euler(self, roll=0, pitch=0, yaw=0):
        # type: (float, float, float) -> int
        """Posture control (1007)"""
        return self._simulate_action("Euler({},{},{})".format(roll, pitch, yaw), 0.1)

    def Move(self, x=0, y=0, z=0):
        # type: (float, float, float) -> int
        """Movement (1008)"""
        return self._simulate_action("Move({},{},{})".format(x, y, z), 0.1)

    def SpeedLevel(self, level=1):
        # type: (int,) -> int
        """Speed level (1015)"""
        return self._simulate_action("SpeedLevel({})".format(level), 0.05)

    def ContinuousGait(self, flag=1):
        # type: (int,) -> int
        """Walking mode (1019)"""
        return self._simulate_action("ContinuousGait({})".format(flag), 0.05)

    def SwitchJoystick(self, enable=True):
        # type: (bool,) -> int
        """Joystick switch (1027)"""
        return self._simulate_action("SwitchJoystick({})".format(enable), 0.05)

    def Pose(self, enable=True):
        # type: (bool,) -> int
        """Pose (1028)"""
        return self._simulate_action("Pose({})".format(enable), 0.1)

    # === State Query ===

    def GetState(self, keys=None):
        # type: (list) -> tuple
        """Get current state (compatible with SDK GetState signature)

        Go2 firmware requires all 5 keys: ["state","bodyHeight","footRaiseHeight","speedLevel","gait"].
        Simulation mode returns reasonable defaults.

        Returns:
            (return_code, state_dict) tuple, matching real SDK return format.
        """
        # Map internal current_state string to integer mode code
        state_mode_map = {"standing": 1, "idle": 0, "walking": 2, "damped": 0}
        mode_int = state_mode_map.get(self.current_state, 0)

        # Simulate full GetState return (matching real Go2 firmware format)
        full_state = {
            "state": mode_int,
            "bodyHeight": 0.0,
            "footRaiseHeight": 0.09,
            "speedLevel": 0,
            "gait": 0,
        }
        if keys:
            state = {k: full_state.get(k) for k in keys}
        else:
            state = full_state
        return 0, state
