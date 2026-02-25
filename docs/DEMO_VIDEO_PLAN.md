# Demo Video Recording Plan

Recording plan for demo videos for the Claudia project GitHub repository.

---

## Video List

### 1. Hero Demo (Main Demo) -- P0 Required

**Purpose**: The most important demo placed at the top of the README. Determines the project's first impression.

**Content**:
- Execute several commands in Japanese (こんにちは, 踊って, 座って, 可愛いね)
- Demonstrate semantic understanding: "疲れた" (tired) -> sit down (not keyword matching)
- Command sequence: "立ってから挨拶して" -> StandUp(1004) -> Hello(1016)
- Terminal screen showing LLM inference process

**Recording Method**: Terminal screen + robot physical unit PiP (Picture-in-Picture)

**Duration**: 60-90 seconds

---

### 2. Voice Pipeline -- P1

**Purpose**: Complete chain from USB mic -> ASR -> LLM -> robot execution.

**Content**:
- Full pipeline flow (ASR recognition -> LLM inference -> Safety verification -> execution)
- Wake word feature demonstration ("クラちゃん踊って")
- Emergency stop voice command ("止まれ！")
- Terminal logs showing timing for each stage

**Recording Method**: Terminal screen + audio input visualization + robot physical unit

**Duration**: 45-60 seconds

---

### 3. Multilingual -- P1

**Purpose**: Control the same robot in three languages: Japanese, Chinese, and English.

**Content**:
- Japanese: "挨拶して" -> Hello(1016)
- Chinese: "跳个舞" -> Dance(1022/1023)
- English: "sit down" -> Sit(1009)

**Recording Method**: Continuous cuts showing input in each language and robot response

**Duration**: 30-45 seconds

---

### 4. Safety System -- P2

**Purpose**: Live demonstration of the safety mechanism provided by SafetyCompiler.

**Content**:
- High-energy action request during low battery -> automatic downgrade to dance
- Automatic StandUp insertion for actions requiring standing
- Zero-latency emergency stop response
- Terminal logs showing SafetyCompiler verdict results

**Recording Method**: Terminal screen (focusing on safety logs) + robot physical unit

**Duration**: 30-45 seconds

---

### 5. Action Showcase -- P2

**Purpose**: Visual introduction of all performance actions.

**Content**:
- Heart(1036), Dance1/Dance2(1022/1023), Hello(1016), Stretch(1017)
- High-risk actions such as FrontFlip(1030), FrontJump(1031)
- WiggleHips(1033), Scrape(1029)

**Recording Method**: Quick-cut editing with background music

**Duration**: 30-60 seconds

---

### 6. Startup & Config Panel -- P3

**Purpose**: How to use the `start_production_brain.sh` interactive launcher.

**Content**:
- Mode selection (keyboard/voice x simulation/hardware)
- Config panel (routing mode, wake word, model selection)
- tmux background execution + SSH disconnect recovery

**Recording Method**: Terminal screen only

**Duration**: 20-30 seconds

---

### 7. Edge AI on Jetson -- P3

**Purpose**: Highlight fully local inference on Jetson Orin NX.

**Content**:
- Hardware setup (physical connection of Jetson + Go2)
- Ollama model load + inference latency (~3-5 seconds)
- No cloud required, fully on-device operation

**Recording Method**: Hardware physical unit + terminal screen

**Duration**: 20-30 seconds

---

## Priority

| Priority | Video | Reason |
|:------:|------|------|
| **P0** | Hero Demo | Top of README, determines first impression |
| **P1** | Voice Pipeline | Biggest technical highlight of the project |
| **P1** | Multilingual | Differentiating factor, high visual impact |
| **P2** | Safety System | Demonstrates engineering quality |
| **P2** | Action Showcase | High visual appeal |
| **P3** | Startup & Config | Developer-oriented, supplementary content |
| **P3** | Edge AI on Jetson | Edge AI / local inference features |

---

## Recording Guidelines

### Screen Layout
- **Hero / Voice / Safety**: Terminal screen + robot physical unit PiP (Picture-in-Picture)
- **Multilingual / Action Showcase**: Robot physical unit as main, terminal as secondary
- **Startup / Jetson**: Terminal screen or hardware physical unit

### Hosting
- Upload to YouTube
- Embed in README using `[![Demo](thumbnail.jpg)](youtube-url)` format
- Place GIF version (5-10 second loop) at top of README, full video link below

### Terminal Settings
- Font size: 16pt or larger (for readability)
- Background: Dark theme (for contrast)
- Window size: 120x30 or larger

### Notes
- Film the robot in a well-lit indoor environment
- Keep the emergency stop remote control at hand
- Ensure sufficient space for high-risk actions (FrontFlip, etc.)
- Check battery level in advance (also prepare a low-battery state for Safety demo)
