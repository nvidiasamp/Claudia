# Real Action to API Mapping Complete Analysis

## User-Provided Real Remote Controller Actions (26)

### Performance Actions (9)
| Chinese | Japanese Romanization | Corresponding API | API Name | State Requirement |
|---------|----------------------|-------------------|----------|-------------------|
| Roll over | ゴロン(goron) | **1029** | Pounce/Rollover | Requires standing |
| Stretch | ノビ(nobi) | **1017** | Stretch | Requires standing |
| Shake hands | お手(ote) | **1025** | ShakeHands | Requires standing |
| Heart gesture | ハート(haato) | **1021** | Wallow | Requires standing |
| Pounce | トンデ(tonde) | **1029** | Pounce | Requires standing |
| Front jump | ジャンプ(janpu) | **1028** | Jump | Requires standing |
| Celebratory bow | おじぎ(ojigi)/ちんちん | **1026/1030** | Cheer/Bow | Requires standing |
| Dance 1 | ダンス(kururin) | **1022** | Dance1 | Requires standing |
| Dance 2 | ダンス(dansu) | **1023** | Dance2 | Requires standing |

### Posture Control (5)
| Chinese | Japanese Romanization | Corresponding API | API Name | State Requirement |
|---------|----------------------|-------------------|----------|-------------------|
| Damping | スロー(suro) | **1001** | Damp | Any state |
| Sit down | おすわり(osuwari) | **1009** | Sit | Any state |
| Pose | ポーズ(pozu) | **1002** | BalanceStand | Requires standing |
| Stand up | タッテ(tatte) | **1004** | StandUp | Any state |
| Normal | ノーマル(nomaru) | **1002** | BalanceStand | Any state |

### Movement Modes (12)
| Chinese | Japanese Romanization | Corresponding API | API Name | Parameter Requirement |
|---------|----------------------|-------------------|----------|-----------------------|
| Agile | リズム(rizumu) | **1011** | SwitchGait | gait parameter |
| Run | ハシレ(hashire) | **1008** | Move | speed parameter |
| Classic | クラシック(kurashikku) | **1011** | SwitchGait | gait=0 |
| Lock | ロック(rokku) | **1003** | StopMove | - |
| Endurance | キープ(kipu) | **1008** | Move | sustained parameter |
| Follow | ツイテ(tsuite) | **1008** | Move | follow mode |
| Handstand | サカダチ(sakadachi) | **Special sequence** | Multi-step | Requires permission |
| Dodge | カワセ(kawase) | **1008** | Move | Quick lateral movement |
| Parallel legs run | ナカヨク(nakayoku) | **1011** | SwitchGait | gait=1 |
| Jump run | ジャンプラン(janpu ran) | **1011** | SwitchGait | gait=2 |
| Upright | タッチ(tacchi) | **Special sequence** | Multi-step | Requires permission |
| Cross step | クロス(kurosu) | **1011** | SwitchGait | gait=3 |

## Key Findings

### 1. **API Mapping Must Be Explicit**
The LLM must output specific API codes, not abstract action_types!

### 2. **Complex Actions Require Sequencing**
Certain actions require multi-API call sequences:
- Handstand: StandUp(1004) -> BalanceStand(1002) -> Special sequence
- Upright walking: StandUp(1004) -> SwitchGait(1011, mode=bipedal)

### 3. **Parameterized Action Handling**
Many actions require parameters:
- Move(1008): Requires speed, direction parameters
- SwitchGait(1011): Requires gait parameters
- BodyHeight(1013): Requires height parameters

## Solution Design

### Recommended LLM Output Format
```json
{
  "response": "はい、座ります",     // Short Japanese reply for TTS
  "api_code": 1009,                  // Direct API code!
  "params": {},                      // API parameters (if needed)
  "sequence": [1004, 1009]          // Action sequence (if needed)
}
```

### Why This Design?
1. **response**: Short Japanese reply, directly used for TTS
2. **api_code**: Directly maps to SportClient methods, no secondary lookup needed
3. **params**: Supports parameterized actions
4. **sequence**: Supports complex action sequences

## 3B Model Optimization Direction

### Prompt Design Principles
1. **Minimal**: Reduce token usage, improve response speed
2. **Direct**: Output API codes rather than abstract types
3. **Structured**: Strict JSON format
4. **Comprehensive coverage**: Include all 26 real actions

### Performance Optimization Suggestions
- temperature: 0.0 (deterministic output)
- top_p: 0.5 (reduce randomness)
- num_predict: 30 (sufficient for JSON output)
- num_ctx: 256 (include necessary context)
