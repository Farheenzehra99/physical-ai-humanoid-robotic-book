# ROS 2 Interface Contracts: VLA Module

**Branch**: `001-vla-module` | **Date**: 2025-12-30

## Overview

This document defines the ROS 2 communication contracts for the VLA module, including topics, services, and actions used in code examples.

---

## 1. Topics

### 1.1 Voice Input Topic

**Topic**: `/voice_commands`
**Type**: `std_msgs/String`
**Publisher**: Voice command node (Chapter 8)
**Subscribers**: Task planner node (Chapter 9)

```yaml
# Message: std_msgs/String
data: "go to the kitchen and pick up the blue cup"
```

### 1.2 Transcription Status Topic

**Topic**: `/voice_status`
**Type**: `vla_msgs/VoiceStatus`
**Publisher**: Voice command node
**Subscribers**: UI feedback, task planner

```yaml
# Custom message: vla_msgs/VoiceStatus
string status  # "listening", "transcribing", "ready", "error"
float32 confidence
string error_message
```

### 1.3 Detected Objects Topic

**Topic**: `/detected_objects`
**Type**: `vision_msgs/Detection2DArray`
**Publisher**: Vision node (Isaac ROS or OpenCV)
**Subscribers**: Task executor, visualization

```yaml
# Standard message: vision_msgs/Detection2DArray
header:
  stamp: <timestamp>
  frame_id: "camera_frame"
detections:
  - bbox:
      center:
        position:
          x: 320.0
          y: 240.0
      size_x: 100.0
      size_y: 80.0
    results:
      - hypothesis:
          class_id: "cup"
          score: 0.95
```

### 1.4 Robot State Topic

**Topic**: `/robot_state`
**Type**: `vla_msgs/RobotState`
**Publisher**: State aggregator node
**Subscribers**: Task planner, UI

```yaml
# Custom message: vla_msgs/RobotState
geometry_msgs/PoseStamped pose
float32 battery_level
bool is_holding_object
int32 held_object_id
string current_action
string error_state
```

---

## 2. Services

### 2.1 Generate Task Plan Service

**Service**: `/generate_plan`
**Type**: `vla_msgs/GeneratePlan`

**Request**:
```yaml
string natural_language_command
string llm_provider  # "openai", "claude", "ollama"
```

**Response**:
```yaml
bool success
vla_msgs/TaskPlan plan
string error_message
```

### 2.2 Validate Plan Service

**Service**: `/validate_plan`
**Type**: `vla_msgs/ValidatePlan`

**Request**:
```yaml
vla_msgs/TaskPlan plan
```

**Response**:
```yaml
bool is_valid
string[] errors
string[] warnings
```

### 2.3 Get Semantic Location Service

**Service**: `/get_location`
**Type**: `vla_msgs/GetLocation`

**Request**:
```yaml
string location_name  # e.g., "kitchen", "table"
```

**Response**:
```yaml
bool found
geometry_msgs/PoseStamped pose
float32 radius
```

---

## 3. Actions

### 3.1 Execute Task Plan Action

**Action**: `/execute_plan`
**Type**: `vla_msgs/ExecutePlan`

**Goal**:
```yaml
vla_msgs/TaskPlan plan
bool skip_validation  # default false
```

**Feedback**:
```yaml
int32 current_action_index
int32 total_actions
string current_action_type
string current_action_status
float32 progress_percentage
```

**Result**:
```yaml
bool success
int32 actions_completed
string[] failed_actions
string error_message
```

### 3.2 Navigate To Location Action

**Action**: `/navigate_to_pose`
**Type**: `nav2_msgs/NavigateToPose` (standard Nav2)

**Goal**:
```yaml
geometry_msgs/PoseStamped pose
string behavior_tree  # optional
```

**Feedback**:
```yaml
geometry_msgs/PoseStamped current_pose
nav_msgs/Path current_path
float32 distance_remaining
builtin_interfaces/Duration estimated_time_remaining
int16 number_of_recoveries
```

**Result**:
```yaml
std_msgs/Empty  # Success indicated by action state
```

### 3.3 Identify Object Action

**Action**: `/identify_object`
**Type**: `vla_msgs/IdentifyObject`

**Goal**:
```yaml
string object_class  # e.g., "cup"
string color_filter  # optional, e.g., "blue"
float32 timeout_seconds
```

**Feedback**:
```yaml
int32 candidates_found
float32 best_confidence
```

**Result**:
```yaml
bool found
vla_msgs/DetectedObject object
string error_message
```

### 3.4 Pick Object Action

**Action**: `/pick_object`
**Type**: `vla_msgs/PickObject`

**Goal**:
```yaml
int32 object_id  # From detection
geometry_msgs/PoseStamped object_pose
string grasp_type  # "top", "side", "pinch"
```

**Feedback**:
```yaml
string phase  # "approaching", "grasping", "lifting", "complete"
float32 progress
```

**Result**:
```yaml
bool success
string error_message
```

---

## 4. TF Frames

### Required Frames

| Frame | Parent | Description |
|-------|--------|-------------|
| `base_link` | `odom` | Robot base |
| `camera_link` | `base_link` | RGB camera |
| `camera_depth_link` | `base_link` | Depth camera |
| `gripper_link` | `arm_link` | End effector |
| `map` | - | World frame |
| `odom` | `map` | Odometry frame |

### Example TF Tree

```
map
 └── odom
      └── base_link
           ├── camera_link
           │    └── camera_depth_link
           └── torso_link
                └── arm_link
                     └── gripper_link
```

---

## 5. Parameters

### Voice Node Parameters

```yaml
voice_command_node:
  ros__parameters:
    whisper_model: "base"  # tiny, base, small, medium, large
    use_api: true  # true for OpenAI API, false for local
    api_key_env: "OPENAI_API_KEY"
    sample_rate: 16000
    chunk_duration: 2.0  # seconds
    silence_threshold: 0.1
    language: "en"
```

### Task Planner Parameters

```yaml
task_planner_node:
  ros__parameters:
    llm_provider: "openai"  # openai, claude, ollama
    model_name: "gpt-4"
    api_key_env: "OPENAI_API_KEY"
    max_actions: 10
    timeout_seconds: 30.0
    safety_check_enabled: true
```

### Object Detection Parameters

```yaml
object_detection_node:
  ros__parameters:
    model_path: "/models/yolov8n.pt"
    confidence_threshold: 0.5
    nms_threshold: 0.4
    use_gpu: true
    input_topic: "/camera/image_raw"
    output_topic: "/detected_objects"
```

---

## 6. Message Package Structure

```
vla_msgs/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── VoiceStatus.msg
│   ├── RobotState.msg
│   ├── TaskPlan.msg
│   ├── RobotAction.msg
│   └── DetectedObject.msg
├── srv/
│   ├── GeneratePlan.srv
│   ├── ValidatePlan.srv
│   └── GetLocation.srv
└── action/
    ├── ExecutePlan.action
    ├── IdentifyObject.action
    └── PickObject.action
```

---

## 7. Error Codes

| Code | Name | Description |
|------|------|-------------|
| 0 | SUCCESS | Operation completed successfully |
| 1 | TRANSCRIPTION_FAILED | Whisper could not process audio |
| 2 | PLANNING_FAILED | LLM did not return valid plan |
| 3 | VALIDATION_FAILED | Plan failed safety checks |
| 4 | NAVIGATION_FAILED | Could not reach goal |
| 5 | DETECTION_FAILED | Object not found |
| 6 | MANIPULATION_FAILED | Could not grasp/place object |
| 7 | TIMEOUT | Operation exceeded time limit |
| 8 | CANCELLED | User cancelled operation |
| 9 | UNKNOWN_ERROR | Unexpected failure |
