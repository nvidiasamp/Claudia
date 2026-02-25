# Task 11: Preset Action Mapping & Execution Architecture

## Overall Architecture Design

### System Goal
Convert Japanese intents parsed by the LLM into specific action execution for the Unitree Go2 robot, achieving the mapping from natural language to robot control.

### Core Component Architecture
```
Japanese command input -> LLM intent parsing -> Mapping decision engine -> SportClient execution -> Action feedback
     |              |              |              |           |
  "お座り"     {intent:"sit"}  -> SIT(1009)  -> client.Sit() -> LED feedback
```

---

## Track 1: Complete Go2 Sport API Analysis (Zero Risk - Execute Immediately)

### 1.1 Tier Classification of 27 Go2 Sport APIs

#### High-Frequency Basic Actions (6)
| API Code | Function Name | Japanese Mapping | Safety Level | Execution Time | Priority |
|---------|----------|----------|----------|----------|---------|
| 1009 | SIT | お座り, 座って | Safe | 2-3s | High |
| 1004 | STANDUP | 立って, 起立 | Safe | 3-4s | High |
| 1016 | HELLO | お手, 握手, 挨拶 | Safe | 3-4s | High |
| 1017 | STRETCH | 伸展, ストレッチ | Safe | 4-5s | High |
| 1002 | BALANCESTAND | バランス, 安定 | Safe | 2-3s | Medium |
| 1001 | DAMP | 停止, ダンプ | Safe | 1s | Emergency |

#### Movement Control Actions (4)
| API Code | Function Name | Japanese Mapping | Safety Level | Execution Time | Priority |
|---------|----------|----------|----------|----------|---------|
| 1008 | MOVE | 移動, 歩いて | Medium | Continuous | High |
| 1007 | EULER | 姿勢, 角度 | Medium | 2-3s | Medium |
| 1005 | STANDDOWN | 伏せ, 下へ | Safe | 3-4s | Medium |
| 1010 | RISESIT | 座りから立つ | Safe | 3-4s | Medium |

#### Advanced Performance Actions (4)
| API Code | Function Name | Japanese Mapping | Safety Level | Execution Time | Priority |
|---------|----------|----------|----------|----------|---------|
| 1022 | DANCE1 | ダンス, 踊って | Medium | 5-8s | High |
| 1023 | DANCE2 | サークル, 回って | Medium | 5-8s | High |
| 1024 | FRONTFLIP | フリップ, 宙返り | Dangerous | 3-5s | Low |
| 1025 | FRONTJUMP | ジャンプ, 跳んで | Dangerous | 2-3s | Low |

#### Control and Recovery (4)
| API Code | Function Name | Japanese Mapping | Safety Level | Execution Time | Priority |
|---------|----------|----------|----------|----------|---------|
| 1003 | STOPMOVE | 移動停止 | Safe | Immediate | Emergency |
| 1006 | RECOVERYSTAND | 回復, 立て直し | Safe | 4-6s | Medium |
| 1026 | FRONTPOUNCE | 前扑, アタック | Dangerous | 2-3s | Low |

### 1.2 API Safety Classification and Execution Strategy

#### Safety Classification Definitions
- **Safe (Green)**: Can be executed safely in any environment
- **Medium (Yellow)**: Requires confirmation that the surrounding environment is safe
- **Dangerous (Red)**: Execute only in open, safe areas
- **Emergency**: Execute immediately, used for emergency stops

#### Execution Priority Strategy
```python
Execution priority order:
1. Emergency level (DAMP, STOPMOVE) - Execute immediately
2. High-priority safe actions (SIT, STANDUP, HELLO) - Prioritize processing
3. High-priority performance actions (DANCE1, DANCE2) - Prioritize in demo scenarios
4. Medium-priority (movement control) - Decide based on environment
5. Low-priority dangerous actions - Require special permission confirmation
```

---

## Track 2: LLM Output Interface Design

### 2.1 Standard LLM Output Format Definition
```json
{
  "intent": "Basic intent type",
  "action": "Specific action name",
  "confidence": 0.85,
  "parameters": {
    "direction": "forward/backward/left/right",
    "speed": "slow/normal/fast",
    "duration": 5.0
  },
  "safety_check": true,
  "composite": false
}
```

### 2.2 Intent Classification Mapping Table
| LLM Intent Type | Target API | Japanese Example | Safety Verification |
|-------------|---------|----------|----------|
| greet | HELLO(1016) | お手, 挨拶して | Yes |
| sit | SIT(1009) | お座り, 座って | Yes |
| stand | STANDUP(1004) | 立って, 起立 | Yes |
| dance | DANCE1(1022) | ダンス, 踊って | Yes |
| stretch | STRETCH(1017) | 伸展, ストレッチ | Yes |
| move | MOVE(1008) | 歩いて, 移動 | Environment check |
| stop | DAMP(1001) | 停止, 止まって | Emergency |

### 2.3 Composite Action Support
```json
{
  "intent": "composite",
  "sequence": [
    {"action": "sit", "wait": 3.0},
    {"action": "dance", "wait": 5.0},
    {"action": "hello", "wait": 2.0}
  ],
  "safety_check": true,
  "total_duration": 10.0
}
```

---

## Track 3: Mapping Decision Engine Architecture

### 3.1 Core Mapping Engine Design
```python
class ActionMappingEngine:
    def __init__(self):
        self.api_registry = self._load_api_registry()
        self.safety_checker = SafetyChecker()
        self.execution_controller = ExecutionController()

    def map_intent_to_action(self, llm_output):
        """LLM output -> SportClient API call"""

        # 1. Parse LLM output
        parsed = self.parse_llm_output(llm_output)

        # 2. Safety check
        if not self.safety_checker.validate(parsed):
            return self.safe_fallback()

        # 3. Mapping decision
        api_call = self.resolve_api_mapping(parsed)

        # 4. Execution control
        return self.execution_controller.execute(api_call)
```

### 3.2 Decision Flow Diagram
```
LLM Output -> Format Validation -> Intent Recognition -> Safety Check -> API Mapping -> Execution Control
    |         |         |         |         |         |
  JSON    Valid     sit     Safe   SIT(1009)  client.Sit()
    |         |         |         |         |         |
 Exception   Format    Unknown    Dangerous  API       Execution
 handling    error     intent     action     error     failure
    |         |         |         |         |         |
  Logging    Default   Ask user  Permission  Retry    Error
             action              confirmation mechanism recovery
```

---

## SportClient Integration Architecture

### 4.1 SportClient Wrapper Design
```python
class ClaudiaRobotController:
    def __init__(self):
        self.client = None
        self.initialized = False
        self.safety_mode = True

    def initialize(self):
        """Initialize SportClient following official examples"""
        ChannelFactoryInitialize(0, "eth0")
        self.client = SportClient()
        self.client.SetTimeout(10.0)
        self.client.Init()
        self.initialized = True

    async def execute_action(self, api_code, parameters=None):
        """Execute specific robot action"""
        if not self.initialized:
            raise RuntimeError("SportClient not initialized")

        action_method = self.get_action_method(api_code)
        return await action_method(parameters)
```

### 4.2 API Call Mapping
```python
API_MAPPING = {
    1001: ("damp", lambda client: client.Damp()),
    1002: ("balance_stand", lambda client: client.BalanceStand()),
    1003: ("stop_move", lambda client: client.StopMove()),
    1004: ("stand_up", lambda client: client.StandUp()),
    1005: ("stand_down", lambda client: client.StandDown()),
    1006: ("recovery_stand", lambda client: client.RecoveryStand()),
    1007: ("euler", lambda client, params: client.Euler(params)),
    1008: ("move", lambda client, params: client.Move(params)),
    1009: ("sit", lambda client: client.Sit()),
    1010: ("rise_sit", lambda client: client.RiseSit()),
    1016: ("hello", lambda client: client.Hello()),
    1017: ("stretch", lambda client: client.Stretch()),
    1022: ("dance1", lambda client: client.Dance1()),
    1023: ("dance2", lambda client: client.Dance2()),
    1024: ("front_flip", lambda client: client.FrontFlip()),
    1025: ("front_jump", lambda client: client.FrontJump()),
    1026: ("front_pounce", lambda client: client.FrontPounce()),
}
```

---

## Safety and Error Handling Mechanism

### 5.1 Multi-Layer Safety Checks
```python
class SafetyChecker:
    def validate_intent(self, intent):
        """Intent safety validation"""
        dangerous_actions = ['front_flip', 'front_jump', 'front_pounce']
        return intent not in dangerous_actions or self.has_permission()

    def validate_environment(self):
        """Environment safety check"""
        # Check sensor data, confirm surroundings are safe
        return self.check_obstacles() and self.check_space()

    def validate_robot_state(self):
        """Robot state check"""
        return self.check_battery() and self.check_temperature()
```

### 5.2 Error Recovery Strategy
```python
Error type handling:
1. LLM output format error -> Request re-input
2. Unknown intent -> Provide list of available actions
3. Safety check failure -> Execute safe alternative action
4. API call failure -> Retry mechanism (max 3 times)
5. Robot anomaly -> Emergency stop + recovery process
```

---

## Mock Data and Testing Strategy

### 6.1 LLM Output Mock Data
```json
// Basic action test
{
  "intent": "sit",
  "confidence": 0.92,
  "parameters": {},
  "safety_check": true
}

// Composite action test
{
  "intent": "composite",
  "sequence": [
    {"action": "sit", "wait": 3.0},
    {"action": "hello", "wait": 2.0}
  ],
  "safety_check": true
}

// Parameterized action test
{
  "intent": "move",
  "parameters": {
    "direction": "forward",
    "speed": "slow",
    "duration": 5.0
  },
  "safety_check": true
}
```

### 6.2 Test Case Design
```python
TEST_CASES = [
    # Basic action tests
    ("お座り", "sit", "SIT(1009)"),
    ("お手", "greet", "HELLO(1016)"),
    ("踊って", "dance", "DANCE1(1022)"),

    # Composite action tests
    ("座ってから手を振って", "composite", ["SIT", "HELLO"]),

    # Abnormal case tests
    ("不明な指令", "unknown", "error_handling"),
    ("危険な動作", "dangerous", "safety_block"),
]
```

---

## Implementation Path and Milestones

### Phase 1: Core Mapping System (Zero Risk)
- [x] API analysis and classification
- [ ] Mapping engine framework design
- [ ] Mock data format definition
- [ ] Basic test case preparation

### Phase 2: SportClient Integration (Low Risk)
- [ ] SportClient wrapper class implementation
- [ ] API call method mapping
- [ ] Basic action test verification

### Phase 3: Safety and Error Handling (Medium Risk)
- [ ] Safety check mechanism
- [ ] Error recovery process
- [ ] Exception handling tests

### Phase 4: Composite Action Support (Medium Risk)
- [ ] Serialized execution engine
- [ ] State machine design
- [ ] Composite action testing

### Phase 5: End-to-End Integration (Pending Task 10 Fix)
- [ ] Interface with Task 10 LLM output
- [ ] Full pipeline testing
- [ ] Performance optimization

---

## Performance Requirements

| Metric | Target Value | Measurement Method |
|------|--------|----------|
| Mapping response time | <50ms | Processing time from LLM output to API call |
| Safety check time | <20ms | Safety verification process duration |
| API call success rate | >95% | Proportion of valid API calls |
| Error recovery time | <2 seconds | Time from anomaly to normal recovery |
| Composite action accuracy | >90% | Proportion of sequence actions executed as expected |

---

**Summary**: This architecture design provides a complete zero-risk implementation path for Task 11. It can be developed and tested independently while Task 10 is being fixed, ensuring parallel progress on both tracks without blocking each other.
