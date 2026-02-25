# Claudia Robot Complete API Action Analysis

## Complete List of 27 Go2 Sport APIs

Based on `/usr/local/include/unitree/robot/go2/sport/sport_api.hpp` and project implementation:

### **Basic Control Actions (6)**

| API | Function Name | Japanese Command | English Command | Safety Level | Execution Time | State Requirement |
|-----|---------------|------------------|-----------------|--------------|----------------|-------------------|
| 1001 | Damp | ダンプ、制動、停止、止まれ | stop, halt, emergency | Emergency | 0.5s | Any |
| 1002 | BalanceStand | バランス、平衡、調整 | balance, stabilize | Safe | 1.0s | Any |
| 1003 | StopMove | 移動停止、動き止める | stop_move, freeze | Emergency | 0.5s | Any |
| 1004 | StandUp | 立つ、立って、起立 | stand, stand_up, rise | Safe | 2.0s | Sitting/Lying -> Standing |
| 1005 | StandDown | 伏せる、横になる、趴下、躺下 | lie_down, down | Safe | 2.0s | Standing -> Lying |
| 1006 | RecoveryStand | 回復、復帰、リカバリー | recover, recovery | Safe | 2.0s | Abnormal -> Standing |

### **Motion Control Actions (9)**

| API | Function Name | Japanese Command | English Command | Safety Level | Execution Time | State Requirement |
|-----|---------------|------------------|-----------------|--------------|----------------|-------------------|
| 1007 | Euler | 傾く、角度、オイラー | euler, tilt, angle | Medium | 1.0s | Requires parameters |
| 1008 | Move | 歩く、歩いて、前進、前进 | walk, move, forward | Safe | 3.0s | Requires parameters |
| 1009 | Sit | 座る、座って、お座り、坐下 | sit, sit_down | Safe | 1.5s | Standing -> Sitting |
| 1010 | RiseSit | 起き上がる、座り直す | rise_sit, sit_up | Safe | 2.0s | Sitting -> Standing |
| 1011 | SwitchGait | 歩調、ゲート、切替 | switch_gait, gait | Medium | 1.0s | Requires parameters |
| 1012 | Trigger | トリガー、実行 | trigger, execute | Medium | 0.5s | Any |
| 1013 | BodyHeight | 高さ、身長、ハイト | height, body_height | Safe | 1.0s | Requires parameters |
| 1014 | FootRaiseHeight | 足上げ、リフト | foot_raise, lift | Safe | 1.0s | Requires parameters |
| 1015 | SpeedLevel | 速度、スピード | speed, velocity | Medium | 1.0s | Requires parameters |

### **Performance/Entertainment Actions (7)**

| API | Function Name | Japanese Command | English Command | Safety Level | Execution Time | State Requirement |
|-----|---------------|------------------|-----------------|--------------|----------------|-------------------|
| 1016 | Hello | お手、握手、挨拶、こんにちは | hello, greet, wave | Safe | 2.0s | **Requires standing** |
| 1017 | Stretch | ストレッチ、伸び、体操 | stretch, exercise | Safe | 3.0s | **Requires standing** |
| 1021 | Wallow | 比心、ハート、love、愛 | heart, love, wallow | Safe | 2.0s | **Requires standing** |
| 1022 | Dance1 | ダンス、踊る、踊って | dance, dance1 | Safe | 5.0s | **Requires standing** |
| 1023 | Dance2 | ダンス2、踊る2 | dance2 | Safe | 5.0s | **Requires standing** |
| 1025 | ShakeHands | 握手、手を振る | shake_hands, handshake | Safe | 2.0s | **Requires standing** |
| 1026 | Cheer | ちんちん、拜年、おめでとう | cheer, celebrate | Safe | 3.0s | **Requires standing** |

### **High-Risk Actions (5)**

| API | Function Name | Japanese Command | English Command | Safety Level | Execution Time | State Requirement |
|-----|---------------|------------------|-----------------|--------------|----------------|-------------------|
| 1024 | FrontFlip | 前転、フリップ | front_flip, flip | Dangerous | 3.0s | **Requires standing** |
| 1027 | BackFlip | 後転、バックフリップ | back_flip | Dangerous | 3.0s | **Requires standing** |
| 1028 | Jump | ジャンプ、跳ぶ | jump | Medium | 2.0s | **Requires standing** |
| 1029 | Pounce | 飛びかかる、パウンス | pounce | Medium | 2.0s | **Requires standing** |
| 1030 | Bow | お辞儀、鞠躬 | bow | Safe | 2.0s | **Requires standing** |

**Total: 27 complete APIs**

## Current Issue Analysis

### **1. Incomplete LLM Fine-Tuning Coverage**

From the ClaudiaOptimizedModelfile_v2_3B file, only the following actions are covered:

#### Covered Actions (~10-12)
- 座る|立つ|歩く|回る (4 basic)
- ダンス|挨拶|ストレッチ (3 performance)
- 停止|ダンプ (2 emergency)
- 状態|バランス (2 status)

#### Missing Actions (~15-17)
- **Heart gesture actions**: 1021 (Wallow)
- **Advanced performance**: 1025 (ShakeHands), 1026 (Cheer), 1030 (Bow)
- **Motion control**: 1007-1015 (9 motion parameter control APIs)
- **Dangerous actions**: 1024, 1027-1029 (4 flip/jump types)

### **2. Action Sequence Issues**

**Key Issue**: Performance actions require transitioning from sitting/lying to standing state first

#### Problem Scenario
```
User: 座って → Robot sits down
User: お手 → Cannot execute, because the robot cannot wave while sitting
```

#### Correct Flow Should Be
```
User: 座って → Robot sits down
User: お手 → Auto stand up → Wave action
```

### **3. LLM Response Analysis**

Some interesting phenomena observed from test logs:
- `趴下` (lie down) → LLM response: 'バランススタンド' (incorrect)
- `比心` (heart gesture) → LLM response: 'TIMEOUT' (cache issue)
- `你好` (hello) → LLM response: 'こんにちは' (language conversion but mapping failed)

## State Dependency Issues

Many performance actions are marked with `requires_standing=True`, but the current system does not properly handle this state transition!
