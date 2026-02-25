# Claudia Robot: Correct LLM Architecture Design

## LLM as the Robot Brain: Correct Architecture

### Design Philosophy
> "The LLM is Claudia robot's brain, responsible for understanding, reasoning, and decision-making. Keyword mapping is only an emergency fallback mechanism."

### Correct Execution Flow
```
User Input: "Please do a cute action for me"
    |
LLM Brain Analysis: Understands user wants a "cute action" -> Output: "heart"
    |
Standardization: "heart" -> API 1021 (Wallow)
    |
State Check: Currently sitting -> Need to execute stand_up first
    |
Action Sequence: StandUp(1004) -> Wallow(1021)
    |
Execution Result: Robot stands up and makes a heart gesture
```

### LLM Core Responsibilities

#### 1. Natural Language Understanding (Primary)
- Understand complex Japanese expressions
- Support multiple phrasings and variations
- Understand context and emotion

#### 2. Action Intent Recognition (Critical)
- Output standardized action types
- Handle ambiguous commands
- Support compound action sequences

#### 3. Intelligent Reasoning (Advanced)
- "Cute action" -> "Heart gesture"
- "Greet" -> "Wave" + "Hello"
- "Perform something" -> Select a dance based on context

### New Role of Keyword Mapping
- **Fallback only**: Emergency mechanism when LLM fails
- **Debugging tool**: Quick testing during development
- **Performance optimization**: Fast path for common commands

## New Architecture Design

### Layer 1: LLM Intelligence Layer
```python
class ClaudiaLLMBrain:
    """Claudia robot's brain"""

    def analyze_intent(self, user_input: str) -> ActionIntent:
        """Analyze user intent - core functionality"""
        # LLM understands complex commands
        # Outputs standardized action intent

    def plan_action_sequence(self, intent: ActionIntent, current_state: RobotState) -> ActionPlan:
        """Plan action sequence - intelligent decision making"""
        # Consider current state
        # Plan optimal execution path
```

### Layer 2: Action Execution Layer
```python
class ActionExecutionEngine:
    """Action execution engine"""

    def execute_plan(self, plan: ActionPlan) -> ExecutionResult:
        """Execute action plan designed by LLM"""
        # Execute based on LLM decisions
        # Not based on keywords
```

### Layer 3: Hardware Control Layer
```python
class RobotController:
    """Hardware control"""
    # Specific API calls
    # State monitoring
    # Safety checks
```

## Correct Design Goals for LLM Prompts

### What NOT to Do (Previous Mistakes)
- Simple keyword confirmation
- Rigid one-to-one mapping
- Only returning fixed phrases

### What to Do (Correct Goals)
- Truly understand natural language
- Output standardized action intents
- Support complex reasoning and context
- Handle ambiguous and creative commands

### LLM Output Standard Format
```json
{
  "action_type": "heart",           // Standardized action type
  "confidence": 0.9,                // Understanding confidence
  "reasoning": "User wants a cute action",   // Reasoning process
  "requires_state_check": true,     // Whether state check is needed
  "alternative_actions": ["wave"],  // Alternative actions
  "emotional_context": "playful"    // Emotional context
}
```

## Refactoring Action Plan

1. **Redesign the LLM intelligence layer**
2. **Create truly intelligent prompts**
3. **Modify action mapping logic**
4. **Test complex command understanding**

Would you like me to start over immediately and design an architecture truly centered on the LLM?
