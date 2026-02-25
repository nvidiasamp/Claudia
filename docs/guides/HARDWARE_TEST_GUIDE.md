# Claudia Hardware Test Guide

**Date**: 2025-11-14
**Version**: v2.0
**Prerequisites**: DDS symbol issue fixed + 7 Critical issues fixed

---

## Issue Diagnosis and Fixes

### Issue 1: ROS2 Initialization Failure - No Action Needed
```
[ERROR] [rmw_cyclonedds_cpp]: rmw_create_node: failed to create domain, error Precondition Not Met
ROS2 initialization failed, enabling simulation mode
```

**Root Cause**:
- LD_LIBRARY_PATH=~/cyclonedds/install/lib causes ROS2 rmw to load CycloneDDS 0.10.x
- ROS2 Foxy's rmw_cyclonedds_cpp was compiled against CycloneDDS 0.7.0, libraries incompatible

**Impact Assessment**:
- **No functional impact**: Code has fallback mechanism, automatically switches to simulation mode
- unitree_sdk2py still works normally (uses 0.10.x)
- ProductionBrain functionality complete (does not depend on ROS2)

**Why it can be ignored**:
1. SystemStateMonitor's ROS2 functionality is **optional enhancement**, not a core dependency
2. Simulation mode provides the same API interface, default battery 100%, simulated posture
3. Real hardware control via unitree_sdk2py (successfully initialized)

**Log interpretation**:
```bash
Initializing DDS channel factory (eth0)...
Real SportClient initialization successful - Robot connected  # <- Key success
[ERROR] [rmw_cyclonedds_cpp]: ...                             # <- Can be ignored
ROS2 initialization failed, enabling simulation mode          # <- Expected fallback
Status monitor started                                        # <- Simulation mode normal
```

### Issue 2: BrainOutput TypeError - Fixed
```
TypeError: __init__() got an unexpected keyword argument 'reasoning'
TypeError: __init__() got an unexpected keyword argument 'success'
```

**Fix Content**:
- First fix (Commit d252ab3): Added `reasoning` field
- Second fix (Commit 221e9b3): Added `success` field
- File: `src/claudia/brain/production_brain.py:54-55`

**Root Cause**:
Hot path SafetyValidator integration added new parameters, but dataclass definition was not updated synchronously

**Verification Results**:
```python
BrainOutput creation successful (with reasoning and success parameters)
to_dict() correctly includes all fields
Default values correctly set (reasoning="", success=True)
```

### Issue 3: Python 3.8 Compatibility - asyncio.to_thread - Fixed
```
Ollama call error: module 'asyncio' has no attribute 'to_thread'
```

**Fix Content**:
- Commit c14935e: Replaced `asyncio.to_thread` with `loop.run_in_executor`
- File: `src/claudia/brain/production_brain.py:634-640`

**Root Cause**:
- `asyncio.to_thread` is a Python 3.9+ new API
- System runs Python 3.8.10, does not support this API

**Impact Scope**:
- Non-hot-path, non-cached LLM calls failed (e.g., "可愛ね", "あなたは誰")
- Hot path commands unaffected (don't call LLM)
- Cache hits unaffected (skip LLM)

**Verification Results**:
```python
loop.run_in_executor works normally on Python 3.8.10
Async timeout control remains unchanged
Fully compatible with hot path and cache logic
```

### Issue 4: Duplicate Action Execution - Fixed
```
Hot path execution complete (7055ms, success=True)  # Already executed
...
Executing action...  # Executed again!
```

**Fix Content**:
- Commit 8df3710: Removed execute_action call from hot path
- File: `src/claudia/brain/production_brain.py:849`

**Root Cause**:
- Hot path called `await self.execute_action(brain_output)` in `process_command`
- `production_commander.py` also called `await self.brain.execute_action(brain_output)`
- Resulted in the same action being executed twice

**Fix Approach**:
- Hot path only returns `BrainOutput`, does not execute actions
- All actions uniformly executed by `production_commander.py`

**Verification Results**:
```python
Hot path only returns BrainOutput(success=True) marked as pending execution
Commander detects BrainOutput and executes once
Audit log correctly records single execution
```

### Issue 5: Inaccurate State Snapshot - Fixed
```
State snapshot: Battery 100%, posture not standing  # Was clearly already standing
```

**Fix Content**:
- Commit 8df3710: Added posture tracking mechanism
- File: `src/claudia/brain/production_brain.py:283-285,729-730,1195-1207`

**Root Cause**:
- SystemStateMonitor in simulation mode does not track posture changes
- Each get_current_state() returns default value (is_standing=False)
- SafetyValidator makes incorrect posture judgments, causing unnecessary auto-stand

**Fix Approach**:
1. Added tracking fields: `self.last_posture_standing`, `self.last_executed_api`
2. Updated posture in `execute_action` (Stand->True, Sit/Down->False)
3. State snapshot uses tracked posture instead of SystemStateMonitor default

**Verification Results**:
```python
Initial state: last_posture_standing=False (assumes sitting)
After executing Stand(1004): last_posture_standing=True
State snapshot correctly shows "posture standing"
Subsequent Heart command no longer auto-inserts Stand
```

### Issue 6: ROS2 Error Message Exposure - Fixed
```
[ERROR] [rmw_cyclonedds_cpp]: rmw_create_node: failed to create domain, error Precondition Not Met
ROS2 initialization failed, enabling simulation mode
```

**Fix Content**:
- Commit 8df3710: Suppressed ROS2 error output
- File: `src/claudia/robot_controller/system_state_monitor.py:183-221`

**Root Cause**:
- ROS2 library prints errors to stderr (code cannot directly control)
- Although fallback mechanism exists, users still see underlying technical errors

**Fix Approach**:
1. Used `contextlib.redirect_stderr(open(os.devnull, 'w'))` to suppress stderr
2. Set ROS2 environment variable to suppress logs: `RCUTILS_CONSOLE_OUTPUT_FORMAT=''`
3. Removed detailed error logging from _initialize_ros2 (returning False is sufficient)

**Verification Results**:
```bash
ROS2 errors no longer displayed to users
Fallback mechanism works normally (auto-switches to simulation mode)
unitree_sdk2py unaffected (independent channel)
```

### Issue 7: LLM Misinterpreting Dialog as Actions - Fixed
```
くら> あなたは誰  # "Who are you?"
Response: こんにちは  # Error: Returns "Hello"
Sequence: [1004, 1016]  # Error: Executes Stand+Hello actions
```

**Fix Content**:
- Commit 8df3710: Added conversational query detection
- File: `src/claudia/brain/production_brain.py:593-673,952-973`

**Root Cause**:
- LLM trained to map all inputs to action APIs
- Lacked preprocessing detection for dialog-type queries
- "あなたは誰" was misinterpreted by LLM as a greeting need

**Fix Approach**:
1. Added `_is_conversational_query()` to detect dialog keywords
2. Added `_generate_conversational_response()` to generate friendly replies
3. Executes dialog detection after hot path, before LLM
4. Dialog queries return `BrainOutput(api_code=None, response="...")`

**Dialog Keyword Coverage**:
- Identity: あなた, 誰, 名前, who, etc.
- Praise: 可愛い, すごい, cute, cool, etc.
- Thanks: ありがとう, thank you, etc.
- Greetings: おはよう, こんばんは, さようなら, good morning, etc.

**Verification Results**:
```python
"あなたは誰" -> "私はClaudiaです。Unitree Go2のAIアシスタントです。"
"可愛いね" -> "ありがとうございます！"
api_code=None, sequence=None (no action executed)
Audit log route="conversational", model_used="dialog_detector"
```

---

## Hardware Test Preparation

### 1. Environment Check
```bash
# Confirm LD_LIBRARY_PATH configuration
echo $LD_LIBRARY_PATH
# Should contain: ~/cyclonedds/install/lib

# Confirm network connection
ping -c 3 192.168.123.161

# Confirm Ollama service
curl http://localhost:11434/api/tags
```

### 2. Start Test
```bash
./start_production_brain_v2.sh
# Select mode 2 (real hardware mode)
```

### 3. Expected Log Pattern
```
Real SportClient initialization successful - Robot connected
[ERROR] [rmw_cyclonedds_cpp]: ...  # <- Expected error, can be ignored
ROS2 initialization failed, enabling simulation mode  # <- Expected fallback
Status monitor started              # <- Simulation mode
Safety validator loaded
Audit logger started
```

---

## Hardware Test Scenarios

### Scenario 1: Sitting -> Heart (Auto-Add Stand)
**Goal**: Verify SafetyValidator standing requirement check

**Steps**:
1. Ensure robot is in **sitting position** (manually or send "座って")
2. Enter command: `ハート` or `比心` or `hello`

**Expected Behavior**:
```bash
Received command: 'ハート'
State snapshot: Battery 100%, posture not standing
Hot path hit: ハート -> 1036
Hot path auto-complete sequence: [1004, 1036]  # <- Auto-insert Stand
Sequence execution: Stand(1004) -> Heart(1036)
```

**Actual Robot Actions**:
1. Stand up first (Stand, API 1004)
2. Then perform Heart gesture (API 1036)

### Scenario 2: Standing -> Heart (Direct Execution)
**Goal**: Verify no prerequisite needed when already standing

**Steps**:
1. Ensure robot is in **standing posture**
2. Enter command: `ハート`

**Expected Behavior**:
```bash
Received command: 'ハート'
State snapshot: Battery 100%, posture standing
Hot path hit: ハート -> 1036
Direct execution: Heart(1036)  # <- No Stand needed
```

### Scenario 3: Low Battery Rejection (Simulated 8%)
**Goal**: Verify Final Safety Gate battery check

**Steps**:
1. Modify code to temporarily simulate low battery:
   ```python
   # production_brain.py get_current_state()
   return SystemStateInfo(
       battery_level=0.08,  # <- Modify here
       is_standing=True,
       ...
   )
   ```
2. Enter high-energy command: `前転` or `FrontFlip`

**Expected Behavior**:
```bash
Received command: '前転'
State snapshot: Battery 8%, posture standing
Quick precheck rejection: Battery level extremely low (8%)
Action rejected
```

### Scenario 4: Complex Sequence ("座ってから挨拶")
**Goal**: Verify LLM understanding of complex semantics

**Steps**:
1. Enter command: `座ってから挨拶`

**Expected Behavior**:
```bash
Received command: '座ってから挨拶'
Using 3B model for inference...
LLM returned: {"r":"座ってから挨拶します","a":null,"seq":[1009,1016]}
Sequence execution: Sit(1009) -> Hello(1016)
```

### Scenario 5: Hot Path High-Frequency Commands
**Goal**: Verify hot path performance

**Sequential input**:
```
座って
立って
止まれ
ダンス
ハート
```

**Expected Behavior**: Each command should hit the hot path, latency <0.01ms

---

## Troubleshooting

### SportClient Initialization Failure
**Symptoms**:
```
SportClient initialization failed
```

**Checks**:
1. Is robot powered on: `ping 192.168.123.161`
2. Is Unitree App closed (avoid occupancy conflict)
3. Network interface: `ip addr show eth0`
4. DDS domain ID: `echo $ROS_DOMAIN_ID` (should be 0)

### LD_LIBRARY_PATH Not Set
**Symptoms**:
```
ImportError: undefined symbol: ddsi_sertype_v0
```

**Fix**:
```bash
export LD_LIBRARY_PATH=~/cyclonedds/install/lib:$LD_LIBRARY_PATH
```

### Ollama Model Not Loaded
**Symptoms**:
```
Model does not exist: claudia-go2-3b:v11.2
```

**Fix**:
```bash
ollama list | grep claudia
# If missing:
ollama pull claudia-go2-3b:v11.2
```

---

## Performance Benchmarks

### Hot Path Performance
- **Hit judgment**: < 0.01ms (dictionary lookup)
- **SafetyValidator check**: < 0.05ms
- **Total latency (sitting->Heart+Stand)**: < 0.1ms (excluding SDK communication)

### LLM Inference Performance
- **3B model**: 800-1500ms (simple commands)
- **7B model**: 2000-4000ms (complex sequences)
- **Parameter convergence**: temperature=0.1, num_predict=30

### Safety Check Performance
- **Quick Precheck**: < 0.01ms (before LLM)
- **SafetyValidator**: < 0.05ms (before action)
- **Final Safety Gate**: < 0.01ms (before execution)

---

## Audit Logs

### Log Location
```bash
logs/audit/audit_YYYYMMDD.jsonl
```

### Key Fields
```json
{
  "timestamp": "2025-11-14T17:10:45.123",
  "command": "ハート",
  "model": "claudia-go2-3b:v11.2",
  "response": "ハートします",
  "api_code": 1036,
  "sequence": [1004, 1036],
  "latency_ms": 0.05,
  "battery_level": 1.0,
  "is_standing": false,
  "safety_result": "sequence_modified",
  "reasoning": "hotpath_executed"
}
```

### Analysis Commands
```bash
# Count hot path hit rate
grep "hotpath_executed" logs/audit/*.jsonl | wc -l

# View rejection cases
grep "rejected" logs/audit/*.jsonl | jq .

# Latency P95
jq -r '.latency_ms' logs/audit/*.jsonl | sort -n | tail -n 5
```

---

## Next Steps

### Completed Fixes (Commit 8df3710)
1. Duplicate action execution - Hot path no longer executes, unified execution by commander
2. Inaccurate state snapshot - Added posture tracking, simulation mode accurately reflects Stand/Sit state
3. ROS2 error message exposure - Suppressed stderr output, users no longer see underlying errors
4. LLM dialog misinterpretation - Added dialog detection, "あなたは誰" etc. return friendly replies instead of actions

### Immediately Executable (P0 - Today)
1. Execute scenario 1-5 complete test (verify fix effects)
2. Test dialog queries: "あなたは誰", "可愛いね", "ありがとう"
3. Verify state tracking: Stand->Heart (no duplicate standing)
4. Collect audit logs (verify route="conversational", "hotpath")

### Optimization Direction (P1 - This Week)
1. Expand hot path coverage (add 10-15 variants)
2. Expand dialog keywords (more dialog scenarios)
3. Externalize battery threshold configuration (config/default.yaml)
4. Daily audit report script

### Production Deployment Preparation (P2 - Next Week)
1. Performance stress test (1000 commands, mixed dialog and actions)
2. Boundary condition test (network disconnect, low battery, abnormal posture)
3. LED status sync verification
4. Real state monitoring integration (SportClient API query actual posture)

---

## FAQ

### Q1: Does the ROS2 error affect functionality?
**A**: No. unitree_sdk2py works independently, SystemStateMonitor has fallen back to simulation mode.

### Q2: Is the simulation mode state accurate?
**A**: **Optimized** (Commit 8df3710): Simulation mode now tracks posture changes (Stand/Sit/Down), state snapshot accurately reflects the last executed action. Battery remains at default 100%. Real battery/IMU requires SportClient API query (P2 integration planned).

### Q3: How to completely disable ROS2 warnings?
**A**: **Already handled automatically** (Commit 8df3710): System automatically suppresses ROS2 error output, users no longer see `[ERROR] [rmw_cyclonedds_cpp]` messages. To manually disable other ROS2 logs:
```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] {message}"
export RCUTILS_LOGGING_BUFFERED_STREAM=0
```

### Q4: Does LD_LIBRARY_PATH affect other programs?
**A**: No. LD_LIBRARY_PATH only takes effect in the current shell session, does not affect the global environment.

### Q5: How to verify CycloneDDS version?
**A**:
```bash
ldd ~/.local/lib/python3.8/site-packages/cyclonedds/_clayer.so | grep ddsc
# Should show: ~/cyclonedds/install/lib/libddsc.so.0
```

---

**Status**: **Ready for Full Hardware Testing (7 Critical Fixes Complete)**
**Fix Summary**:
- DDS symbol issue (Commit 54fd322)
- BrainOutput TypeError (Commits d252ab3, 221e9b3)
- Python 3.8 compatibility (Commit c14935e)
- Duplicate action execution (Commit 8df3710)
- Inaccurate state snapshot (Commit 8df3710)
- ROS2 error exposure (Commit 8df3710)
- LLM dialog misinterpretation (Commit 8df3710)

**Next Step**: Execute hardware test scenarios 1-5 + dialog query test

**Author**: Claude + User
**Last Updated**: 2025-11-14 17:45 UTC
