---
sidebar_position: 5
title: "Exercises and Challenges"
description: "Hands-on exercises to reinforce cognitive planning concepts"
---

# Exercises and Challenges

## Overview

These exercises build on Chapter 9's cognitive planning concepts. Complete them to create a production-ready LLM-based task planner.

---

## Exercise 1: Basic Task Decomposition

**Objective**: Implement task decomposition without an LLM.

### Task

Create a rule-based task decomposer that:
1. Parses simple commands into action sequences
2. Handles common robotics tasks
3. Returns structured TaskPlan objects

### Starter Code

```python
#!/usr/bin/env python3
"""Exercise 1: Rule-based task decomposition."""

from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional
from enum import Enum
import re


class ActionType(Enum):
    NAVIGATE = "navigate"
    PICK = "pick"
    PLACE = "place"
    SPEAK = "speak"
    WAIT = "wait"


@dataclass
class Action:
    action_type: ActionType
    parameters: Dict[str, Any]
    description: str


@dataclass
class TaskPlan:
    description: str
    actions: List[Action]


class RuleBasedDecomposer:
    """
    Decomposes commands using pattern matching rules.
    """

    def __init__(self):
        # Define decomposition rules
        # TODO: Add more patterns
        self.patterns = [
            (r"get (?:me )?(?:a |the )?(.+) from (?:the )?(.+)",
             self._decompose_fetch),
            (r"go to (?:the )?(.+)",
             self._decompose_navigate),
            (r"put (?:the )?(.+) on (?:the )?(.+)",
             self._decompose_place),
        ]

    def decompose(self, command: str) -> Optional[TaskPlan]:
        """
        Decompose a command into actions.

        Args:
            command: Natural language command

        Returns:
            TaskPlan or None if not recognized
        """
        command_lower = command.lower().strip()

        for pattern, handler in self.patterns:
            match = re.match(pattern, command_lower)
            if match:
                return handler(match.groups())

        return None

    def _decompose_fetch(self, groups: tuple) -> TaskPlan:
        """Decompose a fetch command."""
        obj, location = groups

        # TODO: Implement fetch decomposition
        # Should return: navigate → find → pick → return → handover
        pass

    def _decompose_navigate(self, groups: tuple) -> TaskPlan:
        """Decompose a navigation command."""
        location = groups[0]

        # TODO: Implement navigate decomposition
        pass

    def _decompose_place(self, groups: tuple) -> TaskPlan:
        """Decompose a place command."""
        obj, surface = groups

        # TODO: Implement place decomposition
        pass


# Test the decomposer
if __name__ == "__main__":
    decomposer = RuleBasedDecomposer()

    test_commands = [
        "Get me a cup from the kitchen",
        "Go to the bedroom",
        "Put the book on the table",
        "Bring me water",  # Should fail (no pattern)
    ]

    for cmd in test_commands:
        print(f"\nCommand: '{cmd}'")
        plan = decomposer.decompose(cmd)

        if plan:
            print(f"  Description: {plan.description}")
            for i, action in enumerate(plan.actions):
                print(f"  {i+1}. [{action.action_type.value}] {action.description}")
        else:
            print("  Not recognized")
```

### Expected Output

```
Command: 'Get me a cup from the kitchen'
  Description: Fetch cup from kitchen
  1. [navigate] Go to kitchen
  2. [speak] Looking for cup
  3. [pick] Pick up cup
  4. [navigate] Return to user
  5. [speak] Here is your cup

Command: 'Go to the bedroom'
  Description: Navigate to bedroom
  1. [navigate] Go to bedroom
  2. [speak] Arrived at bedroom
```

### Solution Hints

<details>
<summary>Click to reveal hints</summary>

- Each handler should return a TaskPlan with a list of Actions
- Use f-strings to include extracted values in descriptions
- Consider edge cases (missing articles, variations in phrasing)
- Add more patterns for common commands

</details>

---

## Exercise 2: Safety Validation Implementation

**Objective**: Build a comprehensive safety validation system.

### Task

Extend the SafetyValidator to:
1. Check for collision risks
2. Validate manipulation forces
3. Ensure path safety
4. Handle emergency stop scenarios

### Starter Code

```python
#!/usr/bin/env python3
"""Exercise 2: Comprehensive safety validation."""

from dataclasses import dataclass
from typing import List, Dict, Any, Set, Tuple
from enum import Enum


class RiskLevel(Enum):
    NONE = 0
    LOW = 1
    MEDIUM = 2
    HIGH = 3
    CRITICAL = 4


@dataclass
class SafetyIssue:
    risk_level: RiskLevel
    category: str
    description: str
    mitigation: str


@dataclass
class ValidationResult:
    is_safe: bool
    issues: List[SafetyIssue]
    risk_score: float  # 0.0 to 1.0


class ComprehensiveSafetyValidator:
    """
    Comprehensive safety validation for robot actions.
    """

    def __init__(self):
        # Define safety parameters
        self.max_speed_near_human = 0.3  # m/s
        self.max_grip_force = 50.0  # N
        self.max_reach_height = 2.0  # m
        self.min_obstacle_distance = 0.5  # m

        # Dangerous zones
        self.danger_zones = {
            "stairs": RiskLevel.HIGH,
            "kitchen_stove": RiskLevel.HIGH,
            "balcony": RiskLevel.CRITICAL,
        }

        # Fragile objects
        self.fragile_objects = {
            "glass", "cup", "plate", "vase", "mirror"
        }

    def validate_action(
        self,
        action: Dict[str, Any],
        context: Dict[str, Any]
    ) -> ValidationResult:
        """
        Validate a single action.

        Args:
            action: Action to validate
            context: Environmental context (obstacles, humans, etc.)

        Returns:
            ValidationResult
        """
        issues = []

        # TODO: Implement validation checks
        # 1. Check speed limits
        # 2. Check force limits
        # 3. Check collision risks
        # 4. Check zone restrictions
        # 5. Check object handling

        pass

    def validate_plan(
        self,
        plan: List[Dict[str, Any]],
        context: Dict[str, Any]
    ) -> ValidationResult:
        """Validate entire plan."""
        all_issues = []
        max_risk = RiskLevel.NONE

        for action in plan:
            result = self.validate_action(action, context)
            all_issues.extend(result.issues)

            for issue in result.issues:
                if issue.risk_level.value > max_risk.value:
                    max_risk = issue.risk_level

        is_safe = max_risk.value < RiskLevel.HIGH.value
        risk_score = max_risk.value / RiskLevel.CRITICAL.value

        return ValidationResult(
            is_safe=is_safe,
            issues=all_issues,
            risk_score=risk_score
        )

    def _check_speed_safety(
        self,
        speed: float,
        context: Dict[str, Any]
    ) -> List[SafetyIssue]:
        """Check if speed is safe given context."""
        # TODO: Implement
        pass

    def _check_manipulation_safety(
        self,
        obj: str,
        force: float
    ) -> List[SafetyIssue]:
        """Check manipulation safety."""
        # TODO: Implement
        pass

    def _check_zone_safety(
        self,
        target: str
    ) -> List[SafetyIssue]:
        """Check if target zone is safe."""
        # TODO: Implement
        pass

    def _check_collision_risk(
        self,
        path: List[Tuple[float, float]],
        obstacles: List[Dict[str, Any]]
    ) -> List[SafetyIssue]:
        """Check for collision risks along path."""
        # TODO: Implement
        pass


# Test safety validator
if __name__ == "__main__":
    validator = ComprehensiveSafetyValidator()

    # Test scenarios
    test_actions = [
        {
            "type": "navigate",
            "target": "kitchen",
            "speed": 0.5
        },
        {
            "type": "navigate",
            "target": "balcony",
            "speed": 0.3
        },
        {
            "type": "pick",
            "object": "glass",
            "force": 30.0
        },
        {
            "type": "pick",
            "object": "heavy_box",
            "force": 100.0
        }
    ]

    context = {
        "humans_nearby": True,
        "obstacles": [
            {"position": (1.0, 1.0), "radius": 0.3}
        ]
    }

    for action in test_actions:
        print(f"\nAction: {action}")
        result = validator.validate_action(action, context)
        print(f"  Safe: {result.is_safe}")
        print(f"  Risk Score: {result.risk_score:.2f}")
        for issue in result.issues:
            print(f"  Issue: [{issue.risk_level.name}] {issue.description}")
```

### Expected Behavior

- Navigate to balcony should return CRITICAL risk
- High force on fragile objects should warn
- Speed near humans should be validated

---

## Exercise 3: Multi-Step Command Handling

**Objective**: Handle commands with multiple sequential tasks.

### Task

Build a system that:
1. Parses multi-step commands ("First X, then Y, finally Z")
2. Maintains context between steps
3. Handles dependencies between actions

### Starter Code

```python
#!/usr/bin/env python3
"""Exercise 3: Multi-step command handling."""

from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional
import re


@dataclass
class CommandStep:
    """A single step in a multi-step command."""
    order: int
    command: str
    dependencies: List[int] = field(default_factory=list)
    context_updates: Dict[str, Any] = field(default_factory=dict)


@dataclass
class MultiStepCommand:
    """Parsed multi-step command."""
    original: str
    steps: List[CommandStep]
    shared_context: Dict[str, Any]


class MultiStepParser:
    """
    Parse and manage multi-step commands.
    """

    # Sequence indicators
    SEQUENCE_PATTERNS = [
        (r"first[,]?\s*(.+?)[,.]?\s*then\s+(.+)", ["first", "then"]),
        (r"(.+?)\s+and\s+then\s+(.+)", ["step1", "step2"]),
        (r"(.+?)[,]\s*after\s+that\s+(.+)", ["before", "after"]),
        (r"(.+?)[,]\s*finally\s+(.+)", ["main", "final"]),
    ]

    def parse(self, command: str) -> MultiStepCommand:
        """
        Parse a multi-step command.

        Args:
            command: Natural language command

        Returns:
            MultiStepCommand with ordered steps
        """
        # TODO: Implement parsing
        # 1. Identify sequence patterns
        # 2. Extract individual steps
        # 3. Determine dependencies
        # 4. Build context sharing

        pass

    def _extract_steps(self, command: str) -> List[str]:
        """Extract individual steps from command."""
        # TODO: Implement
        pass

    def _determine_dependencies(
        self,
        steps: List[str]
    ) -> Dict[int, List[int]]:
        """Determine which steps depend on others."""
        # TODO: Implement
        pass

    def _build_context_updates(
        self,
        steps: List[str]
    ) -> List[Dict[str, Any]]:
        """
        Determine what context each step provides.

        E.g., "pick up the cup" → {"holding": "cup"}
        """
        # TODO: Implement
        pass


class MultiStepExecutor:
    """Execute multi-step commands with context management."""

    def __init__(self, planner):
        self.planner = planner
        self.context = {}

    async def execute(self, multi_cmd: MultiStepCommand) -> bool:
        """
        Execute multi-step command.

        Args:
            multi_cmd: Parsed multi-step command

        Returns:
            True if all steps succeeded
        """
        self.context = multi_cmd.shared_context.copy()

        for step in multi_cmd.steps:
            # Check dependencies
            # TODO: Verify dependencies are met

            # Execute step with current context
            # TODO: Generate plan for step
            # TODO: Execute plan
            # TODO: Update context

            pass

        return True


# Test multi-step handling
if __name__ == "__main__":
    parser = MultiStepParser()

    test_commands = [
        "First go to the kitchen, then pick up a glass",
        "Get the book from the shelf and then bring it to me",
        "Go to the bedroom, pick up my phone, and finally bring it here",
        "Clean the table after you put away the dishes",
    ]

    for cmd in test_commands:
        print(f"\nCommand: '{cmd}'")
        parsed = parser.parse(cmd)

        if parsed:
            print(f"  Steps: {len(parsed.steps)}")
            for step in parsed.steps:
                print(f"    {step.order}. '{step.command}'")
                if step.dependencies:
                    print(f"       Depends on: {step.dependencies}")
```

---

## Challenge: Custom Action Types

**Objective**: Extend the system with custom robot-specific actions.

### Requirements

Build a plugin system for custom actions:

1. **Define Action Interface**: Abstract base class for actions
2. **Register Actions**: Dynamic action registration
3. **Custom Parameters**: Action-specific parameter validation
4. **Execution Logic**: Pluggable execution handlers

### Starter Code

```python
#!/usr/bin/env python3
"""Challenge: Custom action type plugin system."""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, Any, Type, Optional, Callable
import asyncio


@dataclass
class ActionResult:
    """Result of action execution."""
    success: bool
    message: str
    data: Dict[str, Any] = None


class ActionPlugin(ABC):
    """Base class for action plugins."""

    @property
    @abstractmethod
    def action_name(self) -> str:
        """Unique action identifier."""
        pass

    @property
    @abstractmethod
    def required_params(self) -> list:
        """List of required parameters."""
        pass

    @property
    def optional_params(self) -> dict:
        """Optional parameters with defaults."""
        return {}

    @abstractmethod
    async def execute(
        self,
        params: Dict[str, Any],
        context: Dict[str, Any]
    ) -> ActionResult:
        """Execute the action."""
        pass

    def validate_params(self, params: Dict[str, Any]) -> bool:
        """Validate action parameters."""
        for req in self.required_params:
            if req not in params:
                return False
        return True


class ActionRegistry:
    """Registry for action plugins."""

    def __init__(self):
        self._plugins: Dict[str, ActionPlugin] = {}

    def register(self, plugin: ActionPlugin):
        """Register an action plugin."""
        # TODO: Implement
        pass

    def get(self, action_name: str) -> Optional[ActionPlugin]:
        """Get plugin by name."""
        # TODO: Implement
        pass

    def list_actions(self) -> list:
        """List all registered actions."""
        # TODO: Implement
        pass


# Example custom action: Dance
class DanceAction(ActionPlugin):
    """Custom dance action for entertainment."""

    @property
    def action_name(self) -> str:
        return "dance"

    @property
    def required_params(self) -> list:
        return ["style"]

    @property
    def optional_params(self) -> dict:
        return {"duration_seconds": 10, "music": None}

    async def execute(
        self,
        params: Dict[str, Any],
        context: Dict[str, Any]
    ) -> ActionResult:
        style = params["style"]
        duration = params.get("duration_seconds", 10)

        # Simulate dance execution
        print(f"Dancing {style} style for {duration} seconds!")
        await asyncio.sleep(min(duration, 2))  # Simulate

        return ActionResult(
            success=True,
            message=f"Completed {style} dance",
            data={"moves_performed": 5}
        )


# Example custom action: Photo
class TakePhotoAction(ActionPlugin):
    """Take a photo of something."""

    @property
    def action_name(self) -> str:
        return "take_photo"

    @property
    def required_params(self) -> list:
        return ["subject"]

    async def execute(
        self,
        params: Dict[str, Any],
        context: Dict[str, Any]
    ) -> ActionResult:
        subject = params["subject"]

        # Simulate photo capture
        print(f"Taking photo of {subject}...")

        return ActionResult(
            success=True,
            message=f"Photo of {subject} captured",
            data={"image_path": f"/tmp/{subject}.jpg"}
        )


# Test the plugin system
if __name__ == "__main__":
    registry = ActionRegistry()

    # Register custom actions
    registry.register(DanceAction())
    registry.register(TakePhotoAction())

    # List available actions
    print("Available actions:", registry.list_actions())

    # Execute custom action
    async def test():
        dance = registry.get("dance")
        if dance:
            result = await dance.execute(
                {"style": "robot", "duration_seconds": 5},
                {}
            )
            print(f"Result: {result}")

    asyncio.run(test())
```

### Evaluation Criteria

Your solution should:

- [ ] Allow registration of new action types
- [ ] Validate parameters before execution
- [ ] Provide clear error messages for invalid actions
- [ ] Support async execution
- [ ] Include at least 3 custom action types

---

## Summary

### What You've Learned

Chapter 9 covered the cognitive planning layer:

| Concept | Technology | Purpose |
|---------|------------|---------|
| Task Planning | LLMs (GPT, Claude, Ollama) | Natural language to plans |
| Decomposition | Rule-based + ML | Break tasks into steps |
| Safety | Validation rules | Ensure safe execution |
| Integration | ROS 2 Actions | Execute on robot |

### Key Takeaways

1. **LLMs** enable natural language understanding for robots
2. **Task decomposition** bridges high-level goals and low-level actions
3. **Safety validation** is essential before execution
4. **ROS 2 actions** provide the execution interface

### Next Steps

With cognitive planning complete, you're ready for **Chapter 10: Capstone Project**, where you'll:

- Build a complete autonomous humanoid demo
- Integrate all modules (voice, vision, planning, control)
- Create a working "fetch an object" scenario

---

## Additional Resources

### Papers

- "Language Models as Zero-Shot Planners" (2022)
- "SayCan: Grounding Language in Robotic Affordances" (2022)
- "PaLM-E: An Embodied Multimodal Language Model" (2023)
- "RT-2: Vision-Language-Action Models" (2023)

### Projects

- [LangChain for Robotics](https://github.com/langchain-ai/langchain)
- [ROS 2 LLM Integration](https://github.com/ros2)
- [OpenAI Function Calling](https://platform.openai.com/docs/guides/function-calling)

### Documentation

- [Nav2 Documentation](https://docs.nav2.org/)
- [MoveIt 2 Documentation](https://moveit.picknik.ai/)
- [ROS 2 Actions Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
