# Research: Module 4 - Vision-Language-Action (VLA)

**Branch**: `001-vla-module` | **Date**: 2025-12-30 | **Spec**: [spec.md](./spec.md)

## Research Summary

This document consolidates research findings for implementing Module 4: Vision-Language-Action (VLA) book content for the Physical AI Humanoid Robotics educational resource.

---

## 1. OpenAI Whisper Integration

### Decision: Use OpenAI Whisper API with local whisper.cpp fallback

**Rationale**:
- OpenAI Whisper API provides production-grade speech recognition with minimal setup
- Local whisper.cpp option ensures offline capability and no API costs for students
- Both options maintain compatibility with ROS 2 Python nodes

**Alternatives Considered**:
| Alternative | Rejected Because |
|-------------|------------------|
| Google Cloud Speech-to-Text | Requires GCP account, more complex setup, API costs |
| Mozilla DeepSpeech | Discontinued, outdated models |
| Vosk | Good offline option but lower accuracy than Whisper |
| Azure Speech Services | Microsoft ecosystem lock-in, API costs |

**Implementation Details**:
- Primary: `openai` Python package with Whisper API (`whisper-1`)
- Fallback: `whisper.cpp` via Python bindings for local deployment
- Audio capture: `pyaudio` or `sounddevice` for cross-platform microphone access
- ROS 2 integration: Custom node publishing to `/voice_commands` topic

**Best Practices**:
1. Use 16kHz sample rate for optimal Whisper performance
2. Implement Voice Activity Detection (VAD) to reduce API calls
3. Buffer audio in chunks (1-3 seconds) for streaming transcription
4. Provide clear feedback when command is recognized/rejected

---

## 2. LLM Task Planning Integration

### Decision: Support multiple LLM backends (OpenAI GPT-4, Claude, Ollama)

**Rationale**:
- Multiple options accommodate different student budgets and API access
- Ollama provides free local option with Llama/Mistral models
- Unified interface pattern allows swapping backends without code changes

**Alternatives Considered**:
| Alternative | Rejected Because |
|-------------|------------------|
| Single vendor lock-in | Excludes students without API access |
| Custom fine-tuned model | Too complex for educational context |
| Rule-based planner only | Misses opportunity to teach LLM integration |

**Implementation Details**:
- Abstract `TaskPlanner` interface with backend-specific implementations
- Prompt engineering for robotics task decomposition
- Output format: JSON action sequence compatible with ROS 2 actions
- Safety validation layer before action execution

**Prompt Engineering Pattern**:
```
You are a robot task planner. Given a natural language command,
decompose it into a sequence of executable robot actions.

Available actions:
- navigate_to(location: str) - Move to a named location
- identify_object(object_name: str, color: str) - Find object using vision
- pick_object(object_id: int) - Grasp identified object
- place_object(location: str) - Place held object at location

Command: "{user_command}"
Output: JSON array of actions with parameters
```

---

## 3. ROS 2 Integration Patterns

### Decision: Use ROS 2 action servers for long-running tasks

**Rationale**:
- Actions provide feedback during execution (progress updates)
- Actions are cancellable (safety requirement)
- Actions are the standard ROS 2 pattern for navigation and manipulation

**Key Interfaces**:
| Component | ROS 2 Interface | Message Type |
|-----------|-----------------|--------------|
| Voice Input | Topic | `std_msgs/String` on `/voice_commands` |
| Task Plan | Service | Custom `TaskPlan.srv` |
| Navigation | Action | `nav2_msgs/NavigateToPose` |
| Manipulation | Action | `moveit_msgs/MoveGroup` or custom |
| Object Detection | Topic | `vision_msgs/Detection2DArray` |

**Integration Architecture**:
```
Microphone → Whisper Node → /voice_commands (String)
                              ↓
                        Task Planner Node
                              ↓
                        /task_plan (TaskPlan)
                              ↓
                    Task Executor Node
                         ↓     ↓     ↓
                       Nav2  Vision  Manipulation
```

---

## 4. Computer Vision for Object Detection

### Decision: Use Isaac ROS for GPU-accelerated detection with OpenCV fallback

**Rationale**:
- Isaac ROS provides optimized DNN inference on NVIDIA GPUs
- OpenCV fallback ensures students without NVIDIA hardware can participate
- YOLOv8 provides good accuracy for common object detection

**Alternatives Considered**:
| Alternative | Rejected Because |
|-------------|------------------|
| TensorFlow Object Detection | Heavier dependencies, slower inference |
| Custom model training | Out of scope for this module |
| Cloud-based detection | Latency issues for real-time robotics |

**Implementation Details**:
- Primary: Isaac ROS with YOLOv8 or RT-DETR
- Fallback: OpenCV DNN module with pre-trained YOLO
- Output: Bounding boxes, class labels, confidence scores
- Localization: Depth camera integration for 3D position estimation

---

## 5. Navigation with Nav2

### Decision: Use Nav2 with simple waypoint navigation

**Rationale**:
- Nav2 is the standard ROS 2 navigation stack
- Students learned basics in Module 1-2, this builds on that foundation
- Waypoint navigation is sufficient for capstone demo complexity

**Key Components**:
| Component | Purpose |
|-----------|---------|
| `nav2_bt_navigator` | Behavior tree execution |
| `nav2_planner` | Global path planning |
| `nav2_controller` | Local trajectory following |
| `nav2_costmap_2d` | Obstacle representation |

**Integration Points**:
- Send goals via `NavigateToPose` action
- Monitor feedback for progress updates
- Handle cancellation for safety stops
- Use semantic locations (kitchen, table) mapped to coordinates

---

## 6. Manipulation Strategy

### Decision: Simplified manipulation with MoveIt2 for arm control

**Rationale**:
- Full manipulation is complex; focus on educational value
- MoveIt2 provides motion planning abstraction
- Capstone demo requires pick-and-place, not dexterous manipulation

**Scope Limitation**:
- Pre-defined grasp poses for common objects
- Simplified collision checking
- No force feedback (simulation only for capstone)

**Alternative for Students Without Arms**:
- Simulated manipulation in Gazebo/Isaac Sim
- Focus on planning and vision integration
- Manipulation becomes "virtual" action in demo

---

## 7. Book Content Structure

### Decision: Follow existing book patterns with page-based organization

**Rationale**:
- Consistency with Modules 1-3 structure
- Frontmatter with sidebar_position for navigation
- MDX format for interactive components

**File Naming Convention**:
```
docs/module-04-vision-language-action/
├── _category_.json
├── chapter-08-voice-to-action/
│   ├── _category_.json
│   ├── page-01-intro-speech-robotics.md
│   ├── page-02-whisper-architecture.md
│   ├── page-03-whisper-setup.md
│   ├── page-04-voice-command-pipeline.md
│   ├── page-05-ros2-integration.md
│   └── page-06-exercises.md
├── chapter-09-cognitive-planning/
│   ├── _category_.json
│   ├── page-01-intro-cognitive-robotics.md
│   ├── page-02-llm-task-planning.md
│   ├── page-03-natural-language-actions.md
│   ├── page-04-ros2-action-generation.md
│   └── page-05-exercises.md
└── chapter-10-capstone-autonomous-humanoid/
    ├── _category_.json
    ├── page-01-project-overview.md
    ├── page-02-system-architecture.md
    ├── page-03-implementation-guide.md
    ├── page-04-testing-validation.md
    └── page-05-demo-presentation.md
```

---

## 8. Dependencies and Versions

### Confirmed Compatible Versions

| Package | Version | Purpose |
|---------|---------|---------|
| Ubuntu | 22.04 LTS | Base OS |
| ROS 2 | Jazzy/Iron | Middleware |
| Python | 3.10+ | Primary language |
| openai | 1.0+ | Whisper API |
| whisper.cpp | Latest | Local STT |
| nav2 | Jazzy release | Navigation |
| moveit2 | Jazzy release | Manipulation |
| isaac_ros | 3.0+ | GPU vision |
| ultralytics | 8.0+ | YOLOv8 |

### New Dependencies for VLA Module

```bash
# Python packages
pip install openai anthropic sounddevice webrtcvad

# ROS 2 packages
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-moveit
```

---

## 9. Simulation Environment

### Decision: Use Gazebo for basic testing, Isaac Sim for capstone

**Rationale**:
- Gazebo is accessible to all students (no NVIDIA requirement)
- Isaac Sim provides realistic physics for final demo
- Consistent with existing book modules

**Environment Setup**:
- Kitchen/office environment with table, chairs, objects
- Pre-placed objects (cups, books, balls) with known labels
- Navigation waypoints mapped to semantic locations
- Camera and depth sensor simulation

---

## 10. Risk Assessment

| Risk | Mitigation |
|------|------------|
| Students lack OpenAI API access | Provide Ollama/local alternatives |
| Audio capture issues cross-platform | Test on Ubuntu, provide troubleshooting |
| Navigation complexity | Simplified waypoint-based navigation |
| Manipulation hardware dependency | Simulation-first approach |
| Content length exceeds target | Prioritize core concepts, exercises optional |

---

## Resolved Clarifications

All technical unknowns have been resolved through this research phase:

1. **STT Provider**: OpenAI Whisper with local fallback ✓
2. **LLM Provider**: Multi-backend support ✓
3. **Vision Pipeline**: Isaac ROS with OpenCV fallback ✓
4. **Navigation Stack**: Nav2 waypoint navigation ✓
5. **Manipulation**: MoveIt2 simplified pick-and-place ✓
6. **Content Structure**: Page-based MDX following existing patterns ✓
