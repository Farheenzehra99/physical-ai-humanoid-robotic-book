---
sidebar_position: 2
title: "LLM-Based Task Planning"
description: "Comparing LLM providers and implementing multi-backend task planning for robotics"
---

# LLM-Based Task Planning

## Learning Objectives

By the end of this section, you will be able to:

- Compare different LLM providers for robotics applications
- Implement prompt engineering techniques for task planning
- Build a multi-backend TaskPlanner class
- Understand cost and latency tradeoffs

---

## Choosing an LLM Provider

### Provider Comparison

| Provider | Model | Latency | Cost | Best For |
|----------|-------|---------|------|----------|
| **OpenAI** | GPT-4o | 1-3s | $5-15/M tokens | Production, reliability |
| **Anthropic** | Claude 3.5 | 1-3s | $3-15/M tokens | Safety, long context |
| **Ollama** | Llama 3 | 0.5-2s | Free (local) | Privacy, offline |
| **Google** | Gemini | 1-3s | $1-7/M tokens | Multimodal |

### Selection Criteria for Robotics

1. **Latency**: Real-time interaction requires < 3s response
2. **Reliability**: Critical systems need consistent uptime
3. **Safety**: Built-in refusals for dangerous actions
4. **Cost**: High-frequency use can be expensive
5. **Privacy**: Some applications require local inference

:::tip Recommendation
Start with **OpenAI GPT-4o** for prototyping (best documentation, most examples), then evaluate **Ollama** for production if privacy or cost is a concern.
:::

---

## Prompt Engineering for Robotics

### The System Prompt

The system prompt defines the LLM's role and capabilities:

```python
ROBOTICS_SYSTEM_PROMPT = """You are a task planner for a humanoid robot.

CAPABILITIES:
- Navigation: Move to named locations (kitchen, bedroom, living_room)
- Manipulation: Pick up, place, and hand over objects
- Perception: Detect and locate objects using vision
- Communication: Speak to the user

CONSTRAINTS:
- Only plan actions the robot can physically perform
- Always ensure safety (no dangerous actions)
- Break complex tasks into atomic steps
- Each step must have clear success criteria

OUTPUT FORMAT:
Return a JSON object with the following structure:
{
  "task_description": "Brief description of the task",
  "steps": [
    {
      "step_id": 1,
      "action": "action_name",
      "parameters": {"key": "value"},
      "description": "Human-readable description"
    }
  ],
  "estimated_duration_seconds": 30
}

AVAILABLE ACTIONS:
- navigate: Move to a location. Parameters: target (string), speed (slow/normal/fast)
- pick: Grasp an object. Parameters: object (string), hand (left/right)
- place: Put down held object. Parameters: location (string), surface (string)
- find: Locate an object. Parameters: object (string), search_area (string)
- speak: Say something. Parameters: message (string)
- wait: Pause execution. Parameters: duration_seconds (number)
- handover: Give object to human. Parameters: object (string)
"""
```

### Few-Shot Examples

Provide examples to guide the output format:

```python
FEW_SHOT_EXAMPLES = """
EXAMPLE 1:
User: "Get me a glass of water"

Response:
{
  "task_description": "Fetch a glass of water for the user",
  "steps": [
    {"step_id": 1, "action": "navigate", "parameters": {"target": "kitchen", "speed": "normal"}, "description": "Go to the kitchen"},
    {"step_id": 2, "action": "find", "parameters": {"object": "glass", "search_area": "cabinet"}, "description": "Locate a glass"},
    {"step_id": 3, "action": "pick", "parameters": {"object": "glass", "hand": "right"}, "description": "Pick up the glass"},
    {"step_id": 4, "action": "navigate", "parameters": {"target": "sink", "speed": "slow"}, "description": "Go to the sink"},
    {"step_id": 5, "action": "speak", "parameters": {"message": "Filling the glass with water"}, "description": "Announce action"},
    {"step_id": 6, "action": "wait", "parameters": {"duration_seconds": 5}, "description": "Fill the glass"},
    {"step_id": 7, "action": "navigate", "parameters": {"target": "user_location", "speed": "slow"}, "description": "Return to user"},
    {"step_id": 8, "action": "handover", "parameters": {"object": "glass"}, "description": "Hand the glass to user"}
  ],
  "estimated_duration_seconds": 120
}

EXAMPLE 2:
User: "Turn off the lights in the bedroom"

Response:
{
  "task_description": "Turn off bedroom lights",
  "steps": [
    {"step_id": 1, "action": "navigate", "parameters": {"target": "bedroom", "speed": "normal"}, "description": "Go to the bedroom"},
    {"step_id": 2, "action": "find", "parameters": {"object": "light_switch", "search_area": "wall"}, "description": "Locate the light switch"},
    {"step_id": 3, "action": "speak", "parameters": {"message": "Turning off the lights"}, "description": "Announce action"},
    {"step_id": 4, "action": "interact", "parameters": {"object": "light_switch", "action": "toggle"}, "description": "Toggle the switch"}
  ],
  "estimated_duration_seconds": 45
}
"""
```

---

## Multi-Backend TaskPlanner

### Implementation

```python
#!/usr/bin/env python3
"""
Multi-backend LLM Task Planner.

Supports OpenAI, Anthropic Claude, and Ollama backends.
"""

import json
import os
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, List, Dict, Any
from enum import Enum


class LLMBackend(Enum):
    """Supported LLM backends."""
    OPENAI = "openai"
    ANTHROPIC = "anthropic"
    OLLAMA = "ollama"


@dataclass
class TaskStep:
    """A single step in a task plan."""
    step_id: int
    action: str
    parameters: Dict[str, Any]
    description: str


@dataclass
class TaskPlan:
    """Complete task plan from LLM."""
    task_description: str
    steps: List[TaskStep]
    estimated_duration_seconds: int
    raw_response: str


class LLMClient(ABC):
    """Abstract base class for LLM clients."""

    @abstractmethod
    def generate(self, system_prompt: str, user_prompt: str) -> str:
        """Generate a response from the LLM."""
        pass


class OpenAIClient(LLMClient):
    """OpenAI GPT client."""

    def __init__(self, model: str = "gpt-4o"):
        from openai import OpenAI
        self.client = OpenAI()
        self.model = model

    def generate(self, system_prompt: str, user_prompt: str) -> str:
        response = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.3,  # Lower temperature for more consistent planning
            response_format={"type": "json_object"}
        )
        return response.choices[0].message.content


class AnthropicClient(LLMClient):
    """Anthropic Claude client."""

    def __init__(self, model: str = "claude-3-5-sonnet-20241022"):
        from anthropic import Anthropic
        self.client = Anthropic()
        self.model = model

    def generate(self, system_prompt: str, user_prompt: str) -> str:
        response = self.client.messages.create(
            model=self.model,
            max_tokens=2048,
            system=system_prompt,
            messages=[
                {"role": "user", "content": user_prompt}
            ]
        )
        return response.content[0].text


class OllamaClient(LLMClient):
    """Ollama local LLM client."""

    def __init__(self, model: str = "llama3"):
        import ollama
        self.model = model
        # Verify model is available
        try:
            ollama.show(model)
        except Exception:
            print(f"Model {model} not found. Pulling...")
            ollama.pull(model)

    def generate(self, system_prompt: str, user_prompt: str) -> str:
        import ollama
        response = ollama.chat(
            model=self.model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            options={"temperature": 0.3}
        )
        return response["message"]["content"]


# System prompt (from earlier)
SYSTEM_PROMPT = """You are a task planner for a humanoid robot.

CAPABILITIES:
- Navigation: Move to named locations (kitchen, bedroom, living_room, bathroom)
- Manipulation: Pick up, place, and hand over objects
- Perception: Detect and locate objects using vision
- Communication: Speak to the user

CONSTRAINTS:
- Only plan actions the robot can physically perform
- Always ensure safety (no dangerous actions)
- Break complex tasks into atomic steps
- Each step must have clear success criteria

OUTPUT FORMAT:
Return ONLY a valid JSON object with this structure:
{
  "task_description": "Brief description",
  "steps": [
    {
      "step_id": 1,
      "action": "action_name",
      "parameters": {"key": "value"},
      "description": "Human-readable description"
    }
  ],
  "estimated_duration_seconds": 30
}

AVAILABLE ACTIONS:
- navigate: target (string), speed (slow/normal/fast)
- pick: object (string), hand (left/right)
- place: location (string), surface (string)
- find: object (string), search_area (string)
- speak: message (string)
- wait: duration_seconds (number)
- handover: object (string)
- interact: object (string), action (string)
"""


class TaskPlanner:
    """
    Multi-backend task planner for robotics.

    Converts natural language commands to structured task plans.
    """

    def __init__(
        self,
        backend: LLMBackend = LLMBackend.OPENAI,
        model: Optional[str] = None
    ):
        """
        Initialize task planner.

        Args:
            backend: Which LLM backend to use
            model: Specific model name (uses default if None)
        """
        self.backend = backend
        self.client = self._create_client(backend, model)

    def _create_client(
        self,
        backend: LLMBackend,
        model: Optional[str]
    ) -> LLMClient:
        """Create the appropriate LLM client."""
        if backend == LLMBackend.OPENAI:
            return OpenAIClient(model or "gpt-4o")
        elif backend == LLMBackend.ANTHROPIC:
            return AnthropicClient(model or "claude-3-5-sonnet-20241022")
        elif backend == LLMBackend.OLLAMA:
            return OllamaClient(model or "llama3")
        else:
            raise ValueError(f"Unknown backend: {backend}")

    def plan(self, command: str, context: Optional[str] = None) -> TaskPlan:
        """
        Generate a task plan from natural language command.

        Args:
            command: Natural language command from user
            context: Optional context (robot state, environment)

        Returns:
            TaskPlan with steps to execute
        """
        # Build user prompt
        user_prompt = f"Plan the following task: {command}"
        if context:
            user_prompt = f"Context: {context}\n\n{user_prompt}"

        # Generate response
        response = self.client.generate(SYSTEM_PROMPT, user_prompt)

        # Parse response
        return self._parse_response(response)

    def _parse_response(self, response: str) -> TaskPlan:
        """Parse LLM response into TaskPlan."""
        # Extract JSON from response (handle markdown code blocks)
        json_str = response.strip()
        if json_str.startswith("```"):
            # Remove markdown code block
            lines = json_str.split("\n")
            json_str = "\n".join(lines[1:-1])

        try:
            data = json.loads(json_str)
        except json.JSONDecodeError as e:
            raise ValueError(f"Invalid JSON response: {e}\nResponse: {response}")

        # Build TaskPlan
        steps = [
            TaskStep(
                step_id=s["step_id"],
                action=s["action"],
                parameters=s.get("parameters", {}),
                description=s.get("description", "")
            )
            for s in data.get("steps", [])
        ]

        return TaskPlan(
            task_description=data.get("task_description", ""),
            steps=steps,
            estimated_duration_seconds=data.get("estimated_duration_seconds", 0),
            raw_response=response
        )


# Usage example
if __name__ == "__main__":
    # Try different backends
    backends_to_try = [
        (LLMBackend.OPENAI, "OpenAI GPT-4o"),
        # (LLMBackend.ANTHROPIC, "Anthropic Claude"),
        # (LLMBackend.OLLAMA, "Ollama Llama3"),
    ]

    command = "Get me a glass of water from the kitchen"

    for backend, name in backends_to_try:
        print(f"\n{'='*50}")
        print(f"Testing: {name}")
        print(f"{'='*50}")

        try:
            planner = TaskPlanner(backend=backend)
            plan = planner.plan(command)

            print(f"\nTask: {plan.task_description}")
            print(f"Estimated Duration: {plan.estimated_duration_seconds}s")
            print(f"\nSteps:")
            for step in plan.steps:
                print(f"  {step.step_id}. [{step.action}] {step.description}")
                print(f"      Parameters: {step.parameters}")

        except Exception as e:
            print(f"Error: {e}")
```

---

## Cost and Latency Analysis

### Cost Estimation

```python
def estimate_cost(
    tokens_per_request: int = 1000,
    requests_per_day: int = 100,
    provider: str = "openai"
) -> dict:
    """Estimate monthly LLM costs."""

    # Approximate costs per 1M tokens (input + output)
    costs = {
        "openai_gpt4o": 10.0,      # ~$5 input, $15 output averaged
        "openai_gpt4o_mini": 0.60,  # Much cheaper
        "anthropic_claude": 9.0,    # ~$3 input, $15 output averaged
        "ollama": 0.0,              # Free (hardware costs not included)
    }

    daily_tokens = tokens_per_request * requests_per_day
    monthly_tokens = daily_tokens * 30

    cost_per_token = costs.get(provider, 10.0) / 1_000_000

    return {
        "daily_tokens": daily_tokens,
        "monthly_tokens": monthly_tokens,
        "monthly_cost": monthly_tokens * cost_per_token
    }

# Example
print(estimate_cost(1000, 100, "openai_gpt4o"))
# {'daily_tokens': 100000, 'monthly_tokens': 3000000, 'monthly_cost': 30.0}
```

### Latency Optimization

```python
import time
import asyncio
from functools import lru_cache


class OptimizedTaskPlanner(TaskPlanner):
    """Task planner with latency optimizations."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._cache = {}

    @lru_cache(maxsize=100)
    def plan_cached(self, command: str) -> TaskPlan:
        """Cache frequent commands."""
        return self.plan(command)

    async def plan_async(self, command: str) -> TaskPlan:
        """Async planning for non-blocking operation."""
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(None, self.plan, command)

    def plan_with_timeout(
        self,
        command: str,
        timeout_seconds: float = 5.0
    ) -> Optional[TaskPlan]:
        """Plan with timeout for real-time requirements."""
        import concurrent.futures

        with concurrent.futures.ThreadPoolExecutor() as executor:
            future = executor.submit(self.plan, command)
            try:
                return future.result(timeout=timeout_seconds)
            except concurrent.futures.TimeoutError:
                print(f"Planning timed out after {timeout_seconds}s")
                return None
```

---

## Handling Edge Cases

### Ambiguous Commands

```python
def handle_ambiguous_command(planner: TaskPlanner, command: str) -> TaskPlan:
    """Handle ambiguous commands by asking for clarification."""

    clarification_prompt = f"""
    The user said: "{command}"

    This command is ambiguous. Generate a clarification question.
    Return JSON with:
    {{
      "is_ambiguous": true,
      "clarification_question": "What specific object should I pick up?",
      "possible_interpretations": ["Pick up the cup", "Pick up the book"]
    }}

    If the command is clear, return:
    {{
      "is_ambiguous": false,
      "task_description": "...",
      "steps": [...]
    }}
    """

    response = planner.client.generate(SYSTEM_PROMPT, clarification_prompt)
    data = json.loads(response)

    if data.get("is_ambiguous"):
        print(f"Clarification needed: {data['clarification_question']}")
        return None

    return planner._parse_response(response)
```

### Unsafe Commands

```python
SAFETY_RULES = """
SAFETY RULES (NEVER VIOLATE):
1. Never plan actions that could harm humans
2. Never plan actions that could damage property
3. Never plan actions near edges, stairs, or heights without safety
4. Always verify object stability before manipulation
5. Never exceed speed limits near humans

If a command would violate safety rules, respond with:
{
  "task_description": "UNSAFE - Command rejected",
  "safety_violation": "Description of the safety concern",
  "alternative_suggestion": "A safe alternative if possible",
  "steps": []
}
"""


def validate_plan_safety(plan: TaskPlan) -> bool:
    """Validate that a plan doesn't violate safety rules."""

    dangerous_actions = ["jump", "throw", "run", "climb"]
    dangerous_locations = ["stairs", "balcony", "roof", "window"]

    for step in plan.steps:
        # Check action
        if step.action.lower() in dangerous_actions:
            print(f"Safety violation: Dangerous action '{step.action}'")
            return False

        # Check parameters
        for key, value in step.parameters.items():
            if isinstance(value, str):
                if value.lower() in dangerous_locations:
                    print(f"Safety violation: Dangerous location '{value}'")
                    return False

    return True
```

---

## Summary

- **Multiple backends**: OpenAI, Claude, and Ollama each have tradeoffs
- **Prompt engineering**: System prompts and few-shot examples guide output
- **TaskPlanner class**: Abstracts backend differences
- **Cost management**: Cache common commands, use smaller models when possible
- **Safety**: Always validate plans before execution

**Next**: We'll dive deeper into natural language to action conversion, including task decomposition and parameter extraction.
