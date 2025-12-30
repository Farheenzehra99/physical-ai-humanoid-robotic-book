# Data Model: Module 4 - Vision-Language-Action (VLA)

**Branch**: `001-vla-module` | **Date**: 2025-12-30 | **Spec**: [spec.md](./spec.md)

## Overview

This document defines the data entities, message types, and state machines for the VLA module book content. These models will be used in code examples throughout Chapters 8-10.

---

## 1. Core Entities

### 1.1 VoiceCommand

Represents a spoken command captured from the user.

| Field | Type | Description |
|-------|------|-------------|
| `id` | `string` | Unique identifier (UUID) |
| `timestamp` | `datetime` | When command was captured |
| `audio_data` | `bytes` | Raw audio buffer (optional, for debugging) |
| `transcription` | `string` | Text output from Whisper |
| `confidence` | `float` | Transcription confidence (0.0-1.0) |
| `language` | `string` | Detected language code (e.g., "en") |
| `status` | `CommandStatus` | Processing status |

**Status Enum**:
```python
class CommandStatus(Enum):
    RECEIVED = "received"      # Audio captured
    TRANSCRIBING = "transcribing"  # Whisper processing
    TRANSCRIBED = "transcribed"    # Text available
    PLANNING = "planning"      # Sent to LLM
    EXECUTING = "executing"    # Actions running
    COMPLETED = "completed"    # All actions done
    FAILED = "failed"         # Error occurred
    REJECTED = "rejected"     # Unsafe/impossible
```

---

### 1.2 TaskPlan

Represents an LLM-generated action sequence.

| Field | Type | Description |
|-------|------|-------------|
| `id` | `string` | Unique identifier (UUID) |
| `source_command_id` | `string` | Reference to VoiceCommand |
| `original_text` | `string` | Natural language input |
| `actions` | `List[RobotAction]` | Ordered action sequence |
| `created_at` | `datetime` | Plan generation time |
| `llm_provider` | `string` | Which LLM generated this (openai/claude/ollama) |
| `reasoning` | `string` | LLM explanation (optional) |
| `is_valid` | `bool` | Passed safety validation |
| `validation_errors` | `List[str]` | Why invalid (if any) |

---

### 1.3 RobotAction

Represents a single executable robot action.

| Field | Type | Description |
|-------|------|-------------|
| `id` | `string` | Unique identifier (UUID) |
| `action_type` | `ActionType` | Type of action |
| `parameters` | `Dict[str, Any]` | Action-specific parameters |
| `preconditions` | `List[str]` | What must be true before execution |
| `expected_duration` | `float` | Estimated time in seconds |
| `status` | `ActionStatus` | Execution status |
| `started_at` | `datetime` | When execution began |
| `completed_at` | `datetime` | When execution finished |
| `error_message` | `string` | Error details (if failed) |

**ActionType Enum**:
```python
class ActionType(Enum):
    NAVIGATE_TO = "navigate_to"      # Move to location
    IDENTIFY_OBJECT = "identify_object"  # Find object with vision
    PICK_OBJECT = "pick_object"      # Grasp object
    PLACE_OBJECT = "place_object"    # Place object
    WAIT = "wait"                    # Pause execution
    SPEAK = "speak"                  # Text-to-speech output
    LOOK_AT = "look_at"              # Point camera at target
```

**ActionStatus Enum**:
```python
class ActionStatus(Enum):
    PENDING = "pending"      # Not started
    ACTIVE = "active"        # Currently executing
    SUCCEEDED = "succeeded"  # Completed successfully
    FAILED = "failed"        # Error occurred
    CANCELLED = "cancelled"  # User/system stopped
```

---

### 1.4 DetectedObject

Represents an object identified by computer vision.

| Field | Type | Description |
|-------|------|-------------|
| `id` | `int` | Unique ID in current scene |
| `class_name` | `string` | Object category (cup, ball, etc.) |
| `confidence` | `float` | Detection confidence (0.0-1.0) |
| `bounding_box` | `BoundingBox` | 2D image coordinates |
| `position_3d` | `Point3D` | World coordinates (if depth available) |
| `color` | `string` | Dominant color (optional) |
| `timestamp` | `datetime` | When detected |

**BoundingBox**:
```python
@dataclass
class BoundingBox:
    x_min: float  # Left edge (pixels)
    y_min: float  # Top edge (pixels)
    x_max: float  # Right edge (pixels)
    y_max: float  # Bottom edge (pixels)
```

**Point3D**:
```python
@dataclass
class Point3D:
    x: float  # Forward (meters)
    y: float  # Left (meters)
    z: float  # Up (meters)
    frame_id: str  # Reference frame (e.g., "base_link")
```

---

### 1.5 SemanticLocation

Represents a named location in the environment.

| Field | Type | Description |
|-------|------|-------------|
| `name` | `string` | Human-readable name (kitchen, table) |
| `position` | `Point3D` | World coordinates |
| `orientation` | `Quaternion` | Facing direction |
| `radius` | `float` | Acceptable arrival distance (meters) |

---

### 1.6 RobotState

Represents the current state of the robot.

| Field | Type | Description |
|-------|------|-------------|
| `position` | `Point3D` | Current world position |
| `orientation` | `Quaternion` | Current facing direction |
| `battery_level` | `float` | Battery percentage (0.0-1.0) |
| `is_holding_object` | `bool` | Whether gripper has object |
| `held_object_id` | `int` | ID of held object (if any) |
| `current_action` | `string` | What robot is doing |
| `error_state` | `string` | Active error (if any) |
| `timestamp` | `datetime` | State update time |

---

## 2. ROS 2 Message Types

### 2.1 Custom Messages

**VoiceCommand.msg**:
```
# Voice command message
string id
builtin_interfaces/Time timestamp
string transcription
float32 confidence
string language
string status
```

**TaskPlan.msg**:
```
# Task plan with action sequence
string id
string source_command_id
string original_text
RobotAction[] actions
builtin_interfaces/Time created_at
string llm_provider
bool is_valid
string[] validation_errors
```

**RobotAction.msg**:
```
# Single robot action
string id
string action_type
string parameters_json  # JSON-encoded parameters
string[] preconditions
float32 expected_duration
string status
```

### 2.2 Service Definitions

**GeneratePlan.srv**:
```
# Request
string natural_language_command
---
# Response
bool success
TaskPlan plan
string error_message
```

**ValidatePlan.srv**:
```
# Request
TaskPlan plan
---
# Response
bool is_valid
string[] errors
string[] warnings
```

---

## 3. State Machines

### 3.1 Voice Command Processing

```
                  ┌─────────┐
                  │ IDLE    │
                  └────┬────┘
                       │ audio_detected
                       ▼
                  ┌─────────┐
                  │LISTENING│
                  └────┬────┘
                       │ silence_detected
                       ▼
               ┌───────────────┐
               │ TRANSCRIBING  │
               └───────┬───────┘
                       │ transcription_complete
                       ▼
                ┌─────────────┐
                │  PLANNING   │
                └──────┬──────┘
           ┌───────────┼───────────┐
           │           │           │
           ▼           ▼           ▼
      ┌────────┐ ┌──────────┐ ┌─────────┐
      │REJECTED│ │EXECUTING │ │ FAILED  │
      └────────┘ └─────┬────┘ └─────────┘
                       │ all_actions_complete
                       ▼
                  ┌──────────┐
                  │COMPLETED │
                  └──────────┘
```

### 3.2 Action Execution

```
        ┌─────────┐
        │ PENDING │
        └────┬────┘
             │ start_action
             ▼
        ┌─────────┐
        │ ACTIVE  │
        └────┬────┘
    ┌────────┼────────┐
    │        │        │
    ▼        ▼        ▼
┌────────┐ ┌────────┐ ┌──────────┐
│SUCCEEDED│ │ FAILED │ │CANCELLED │
└─────────┘ └────────┘ └──────────┘
```

---

## 4. Validation Rules

### 4.1 Voice Command Validation

- `transcription` must not be empty
- `confidence` must be >= 0.5 for processing
- `language` must be in supported list (initially ["en"])

### 4.2 Task Plan Validation

- `actions` must not be empty
- Each action must have valid `action_type`
- Navigation actions must reference known locations
- Pick actions require prior identify action
- Place actions require robot to be holding object

### 4.3 Safety Constraints

- No navigation to restricted zones
- Maximum single action duration: 60 seconds
- Maximum total plan duration: 300 seconds
- Emergency stop cancels all pending actions
- Unknown objects cannot be manipulated

---

## 5. Integration Points

### 5.1 From Spec to Data Model

| Spec Entity | Data Model |
|-------------|------------|
| Voice Command | `VoiceCommand` |
| Transcription | `VoiceCommand.transcription` |
| Task Plan | `TaskPlan` |
| ROS 2 Action | `RobotAction` |
| Scene Understanding | `DetectedObject` + `SemanticLocation` |
| Robot State | `RobotState` |

### 5.2 Chapter Code Examples

| Chapter | Primary Models Used |
|---------|---------------------|
| Ch 8: Voice | `VoiceCommand`, `CommandStatus` |
| Ch 9: Planning | `TaskPlan`, `RobotAction`, `ActionType` |
| Ch 10: Capstone | All models integrated |
