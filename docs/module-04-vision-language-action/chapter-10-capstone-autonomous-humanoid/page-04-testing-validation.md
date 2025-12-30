---
sidebar_position: 4
title: "Testing and Validation"
description: "Procedures for testing each subsystem and the integrated capstone"
---

# Testing and Validation

## Overview

Thorough testing is critical for a reliable demo. This section provides testing procedures for each subsystem and the integrated system.

---

## Subsystem Testing

### 1. Voice Recognition Tests

#### Test 1.1: Microphone Check

```bash
# List available audio devices
python3 -c "import sounddevice as sd; print(sd.query_devices())"

# Record and playback
arecord -d 3 test.wav && aplay test.wav
```

**Expected**: Clear audio playback with your voice.

#### Test 1.2: VAD Functionality

```python
#!/usr/bin/env python3
"""Test Voice Activity Detection."""

import sounddevice as sd
import numpy as np
import webrtcvad

vad = webrtcvad.Vad(2)
SAMPLE_RATE = 16000
CHUNK_SIZE = 480  # 30ms

def test_vad():
    print("Testing VAD. Speak and check detection...")
    print("Press Ctrl+C to stop\n")

    def callback(indata, frames, time, status):
        chunk = indata.flatten().astype(np.int16)
        is_speech = vad.is_speech(chunk.tobytes(), SAMPLE_RATE)
        status = "SPEECH" if is_speech else "silence"
        print(f"\r{status}      ", end="")

    with sd.InputStream(
        samplerate=SAMPLE_RATE,
        channels=1,
        dtype=np.int16,
        blocksize=CHUNK_SIZE,
        callback=callback
    ):
        try:
            while True:
                sd.sleep(100)
        except KeyboardInterrupt:
            print("\nDone")

if __name__ == "__main__":
    test_vad()
```

**Expected**: "SPEECH" when talking, "silence" when quiet.

#### Test 1.3: Whisper Transcription

```python
#!/usr/bin/env python3
"""Test Whisper transcription."""

from openai import OpenAI
import tempfile
import wave
import numpy as np
import sounddevice as sd

def test_whisper():
    client = OpenAI()

    print("Recording 3 seconds... Speak now!")
    audio = sd.rec(
        int(3 * 16000),
        samplerate=16000,
        channels=1,
        dtype=np.int16
    )
    sd.wait()
    print("Processing...")

    # Save to file
    with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
        with wave.open(f.name, 'w') as wav:
            wav.setnchannels(1)
            wav.setsampwidth(2)
            wav.setframerate(16000)
            wav.writeframes(audio.tobytes())
        path = f.name

    # Transcribe
    with open(path, 'rb') as audio_file:
        result = client.audio.transcriptions.create(
            model='whisper-1',
            file=audio_file
        )

    print(f"\nTranscription: {result.text}")
    return result.text

if __name__ == "__main__":
    test_whisper()
```

**Expected**: Accurate transcription of spoken text.

#### Test 1.4: ROS Integration

```bash
# Terminal 1: Run voice node
ros2 run voice_commands voice_node

# Terminal 2: Monitor topic
ros2 topic echo /voice_commands

# Speak "hello world"
# Expected output: data: 'hello world'
```

---

### 2. Planning Tests

#### Test 2.1: LLM Connectivity

```python
#!/usr/bin/env python3
"""Test LLM connection."""

from openai import OpenAI

def test_llm():
    client = OpenAI()

    response = client.chat.completions.create(
        model="gpt-4o",
        messages=[
            {"role": "user", "content": "Say 'LLM test successful'"}
        ],
        max_tokens=50
    )

    print(response.choices[0].message.content)

if __name__ == "__main__":
    test_llm()
```

**Expected**: "LLM test successful"

#### Test 2.2: Plan Generation

```python
#!/usr/bin/env python3
"""Test task plan generation."""

from openai import OpenAI
import json

SYSTEM_PROMPT = """You are a robot task planner. Return JSON:
{"task": "description", "steps": [{"action": "...", "params": {...}}]}

Actions: navigate, find, pick, place, speak, handover
Locations: kitchen, bedroom, living_room"""

def test_planning(command: str):
    client = OpenAI()

    response = client.chat.completions.create(
        model="gpt-4o",
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": command}
        ],
        response_format={"type": "json_object"}
    )

    plan = json.loads(response.choices[0].message.content)
    print(json.dumps(plan, indent=2))
    return plan

if __name__ == "__main__":
    test_commands = [
        "Go to the kitchen",
        "Get me a cup from the kitchen",
        "Find my phone",
    ]

    for cmd in test_commands:
        print(f"\n{'='*50}")
        print(f"Command: {cmd}")
        print(f"{'='*50}")
        test_planning(cmd)
```

**Expected**: Valid JSON plans with appropriate steps.

#### Test 2.3: ROS Planner Node

```bash
# Terminal 1: Run planner
ros2 run cognitive_planning planner_node

# Terminal 2: Monitor plan output
ros2 topic echo /task_plan

# Terminal 3: Publish test command
ros2 topic pub /voice_commands std_msgs/String "data: 'go to the kitchen'" --once
```

**Expected**: JSON plan published to `/task_plan`.

---

### 3. Navigation Tests

#### Test 3.1: Nav2 Availability

```bash
# Check Nav2 action server
ros2 action list

# Expected: /navigate_to_pose should be listed
```

#### Test 3.2: Simple Navigation

```bash
# Send navigation goal via CLI
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}}}}"
```

**Expected**: Robot moves to (1.0, 1.0) or feedback about the attempt.

#### Test 3.3: Semantic Navigation

```python
#!/usr/bin/env python3
"""Test semantic navigation."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

LOCATIONS = {
    "kitchen": (3.0, 2.0),
    "bedroom": (5.0, 4.0),
    "living_room": (1.0, 1.0),
}

class NavTest(Node):
    def __init__(self):
        super().__init__('nav_test')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def navigate(self, location):
        if location not in LOCATIONS:
            print(f"Unknown: {location}")
            return

        x, y = LOCATIONS[location]
        print(f"Navigating to {location} ({x}, {y})")

        if not self.client.wait_for_server(timeout_sec=5.0):
            print("Nav2 not available")
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        print("Goal sent")

def main():
    rclpy.init()
    node = NavTest()
    node.navigate("kitchen")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

---

### 4. Vision Tests

#### Test 4.1: Camera Feed

```bash
# Check camera topic
ros2 topic list | grep camera

# View camera feed
ros2 run rqt_image_view rqt_image_view
```

**Expected**: Live camera feed visible.

#### Test 4.2: Object Detection

```bash
# Run detector node
ros2 run robot_vision detector_node

# Monitor detections
ros2 topic echo /detected_objects
```

**Expected**: JSON with detected objects when visible.

#### Test 4.3: Target Object Detection

Place a cup in camera view and verify detection:

```bash
# Should see output like:
# data: '[{"label": "cup", "confidence": 0.92, "bbox": [...]}]'
```

---

## Integration Testing

### Test Scenario 1: Voice to Plan

```bash
# Terminal 1: Voice node
ros2 run voice_commands voice_node

# Terminal 2: Planner node
ros2 run cognitive_planning planner_node

# Terminal 3: Monitor
ros2 topic echo /voice_commands &
ros2 topic echo /task_plan

# Speak: "Go to the kitchen"
# Expected: Voice transcribed → Plan generated
```

### Test Scenario 2: Plan to Execution

```bash
# Terminal 1: Executor node
ros2 run cognitive_planning executor_node

# Terminal 2: Monitor
ros2 topic echo /executor_status &
ros2 topic echo /executor_feedback

# Terminal 3: Publish test plan
ros2 topic pub /task_plan std_msgs/String "data: '{\"task\": \"test\", \"steps\": [{\"action\": \"navigate\", \"params\": {\"target\": \"kitchen\"}}]}'" --once

# Expected: Executor attempts navigation
```

### Test Scenario 3: Full Pipeline

```bash
# Launch full system
ros2 launch capstone_bringup capstone.launch.py

# Monitor all topics
ros2 topic echo /voice_commands &
ros2 topic echo /task_plan &
ros2 topic echo /executor_status &
ros2 topic echo /executor_feedback &

# Speak: "Get me a cup from the kitchen"
# Expected: Full execution pipeline
```

---

## Validation Checklist

### Pre-Demo Validation

| # | Check | Status |
|---|-------|--------|
| 1 | Microphone captures clear audio | [ ] |
| 2 | VAD correctly detects speech/silence | [ ] |
| 3 | Whisper transcribes with >90% accuracy | [ ] |
| 4 | LLM generates valid JSON plans | [ ] |
| 5 | Plans include correct actions | [ ] |
| 6 | Navigator accepts goals | [ ] |
| 7 | Robot reaches target locations | [ ] |
| 8 | Camera feed is live | [ ] |
| 9 | Detector finds target objects | [ ] |
| 10 | Full pipeline works end-to-end | [ ] |

### Success Criteria Validation

| Criterion | Target | Actual | Pass |
|-----------|--------|--------|------|
| Voice recognition accuracy | >90% | | [ ] |
| Planning latency | &lt;5s | | [ ] |
| Navigation success rate | >80% | | [ ] |
| Object detection accuracy | >70% | | [ ] |
| Full task completion | >70% | | [ ] |

---

## Troubleshooting Guide

### Voice Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| No audio detected | Wrong device | Check `sd.query_devices()` |
| Poor transcription | Background noise | Use directional mic, reduce noise |
| Slow transcription | Network latency | Check internet, consider local Whisper |
| VAD too sensitive | Aggressiveness | Increase VAD level (2→3) |

### Planning Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| Invalid JSON | LLM response | Retry, improve prompt |
| Wrong actions | Missing context | Add examples to prompt |
| Timeout | API overload | Implement retry logic |
| No plan | Unclear command | Provide clarification |

### Navigation Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| Nav2 not available | Not running | Launch Nav2 first |
| Goal rejected | Invalid pose | Check map bounds |
| Robot stuck | Obstacle | Clear path, check costmap |
| Wrong destination | Bad coordinates | Verify semantic map |

### Vision Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| No detections | Camera not publishing | Check camera node |
| Wrong detections | Low confidence | Adjust threshold |
| Missing objects | Not in training set | Use different model |
| Slow detection | CPU inference | Use GPU if available |

---

## Performance Benchmarks

### Timing Breakdown

Run timing tests:

```python
#!/usr/bin/env python3
"""Benchmark pipeline timing."""

import time
from openai import OpenAI

def benchmark():
    client = OpenAI()

    # Voice processing (simulated)
    voice_time = 0.5  # VAD + buffer

    # Transcription
    start = time.time()
    # Simulate with a simple API call
    client.chat.completions.create(
        model="gpt-4o",
        messages=[{"role": "user", "content": "test"}],
        max_tokens=10
    )
    transcription_time = time.time() - start

    # Planning
    start = time.time()
    client.chat.completions.create(
        model="gpt-4o",
        messages=[{"role": "user", "content": "Plan: go to kitchen"}],
        max_tokens=500
    )
    planning_time = time.time() - start

    print(f"Voice processing: {voice_time:.2f}s")
    print(f"Transcription: {transcription_time:.2f}s")
    print(f"Planning: {planning_time:.2f}s")
    print(f"Total (before nav): {voice_time + transcription_time + planning_time:.2f}s")

if __name__ == "__main__":
    benchmark()
```

### Expected Benchmarks

| Stage | Target | Acceptable |
|-------|--------|------------|
| Voice capture + VAD | &lt;1s | &lt;2s |
| Transcription | &lt;2s | &lt;3s |
| Planning | &lt;3s | &lt;5s |
| Navigation | Varies | &lt;60s |
| Object detection | &lt;1s | &lt;2s |
| Manipulation | Varies | &lt;30s |

---

## Summary

Testing ensures reliability:

1. **Unit tests**: Each component individually
2. **Integration tests**: Components working together
3. **End-to-end tests**: Full pipeline validation
4. **Benchmarks**: Performance meets requirements

**Next**: Demo presentation guidelines.
