# Quickstart: Module 4 - Vision-Language-Action (VLA)

**Branch**: `001-vla-module` | **Date**: 2025-12-30

## Prerequisites

Before starting Module 4, ensure you have completed:

- [ ] Modules 1-3 of the Physical AI Humanoid Robotics book
- [ ] Ubuntu 22.04 LTS installed
- [ ] ROS 2 Jazzy or Iron installed and sourced
- [ ] Python 3.10+ with pip
- [ ] Working microphone for voice input
- [ ] (Optional) NVIDIA GPU for accelerated inference

---

## Quick Setup (10 minutes)

### Step 1: Install Python Dependencies

```bash
# Create virtual environment (recommended)
python3 -m venv ~/vla_env
source ~/vla_env/bin/activate

# Install core packages
pip install openai sounddevice webrtcvad numpy

# Install optional local Whisper (if not using API)
pip install openai-whisper
```

### Step 2: Install ROS 2 Packages

```bash
# Navigation stack
sudo apt update
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-nav2-bt-navigator

# Vision (optional - for Isaac ROS)
# See Chapter 10 for Isaac ROS setup
```

### Step 3: Set API Keys

```bash
# Add to ~/.bashrc
export OPENAI_API_KEY="your-api-key-here"  # For Whisper + GPT
export ANTHROPIC_API_KEY="your-api-key-here"  # Optional: For Claude

# Reload
source ~/.bashrc
```

### Step 4: Test Voice Input

```bash
# Quick microphone test
python3 -c "
import sounddevice as sd
print('Recording 3 seconds...')
audio = sd.rec(int(3 * 16000), samplerate=16000, channels=1)
sd.wait()
print(f'Captured {len(audio)} samples')
print('Microphone working!')
"
```

---

## Verify Installation

Run this verification script:

```bash
python3 << 'EOF'
import sys

checks = []

# Check Python version
checks.append(("Python 3.10+", sys.version_info >= (3, 10)))

# Check OpenAI
try:
    import openai
    checks.append(("openai package", True))
except ImportError:
    checks.append(("openai package", False))

# Check sounddevice
try:
    import sounddevice as sd
    checks.append(("sounddevice package", True))
except ImportError:
    checks.append(("sounddevice package", False))

# Check webrtcvad
try:
    import webrtcvad
    checks.append(("webrtcvad package", True))
except ImportError:
    checks.append(("webrtcvad package", False))

# Check API key
import os
checks.append(("OPENAI_API_KEY set", bool(os.getenv("OPENAI_API_KEY"))))

# Results
print("\n=== VLA Module Verification ===\n")
all_pass = True
for name, passed in checks:
    status = "PASS" if passed else "FAIL"
    print(f"  [{status}] {name}")
    if not passed:
        all_pass = False

print()
if all_pass:
    print("All checks passed! Ready for Module 4.")
else:
    print("Some checks failed. Review installation steps.")
EOF
```

---

## Chapter Overview

### Chapter 8: Voice-to-Action (Weeks 7-8)

Learn to capture voice commands and convert them to robot instructions using OpenAI Whisper.

**Key Files**:
- `voice_command_node.py` - ROS 2 node for voice capture
- `whisper_client.py` - Whisper API/local integration

**Test Command**:
```bash
# After completing Chapter 8
ros2 run vla_nodes voice_command_node
# Speak: "move forward"
# Should see: Published command: move_forward
```

### Chapter 9: Cognitive Planning (Weeks 8-9)

Learn to use LLMs for task decomposition and action planning.

**Key Files**:
- `task_planner_node.py` - LLM-based planner
- `action_validator.py` - Safety validation

**Test Command**:
```bash
# After completing Chapter 9
ros2 service call /generate_plan vla_msgs/GeneratePlan \
  "{natural_language_command: 'go to the kitchen and pick up the cup'}"
# Should return: TaskPlan with navigate_to + identify_object + pick_object
```

### Chapter 10: Capstone Project (Weeks 10-13)

Integrate all components into an autonomous humanoid demo.

**Demo Command**:
```bash
# After completing Chapter 10
ros2 launch vla_bringup capstone_demo.launch.py
# Speak: "Go to the kitchen table and pick up the blue cup"
# Robot navigates, identifies, and picks up the cup
```

---

## Troubleshooting

### Microphone Not Working

```bash
# List audio devices
python3 -c "import sounddevice; print(sounddevice.query_devices())"

# Set specific device
export SDL_AUDIODRIVER=alsa
```

### OpenAI API Errors

```bash
# Test API key
python3 -c "
import openai
client = openai.OpenAI()
print(client.models.list().data[0].id)
"
```

### ROS 2 Package Not Found

```bash
# Ensure ROS 2 is sourced
source /opt/ros/jazzy/setup.bash

# Rebuild workspace
cd ~/ros2_ws
colcon build --packages-select vla_msgs vla_nodes
source install/setup.bash
```

---

## Alternative: Local-Only Setup (No API Keys)

If you don't have OpenAI API access:

### 1. Install Local Whisper

```bash
pip install openai-whisper
# Download model (first run will be slow)
python3 -c "import whisper; whisper.load_model('base')"
```

### 2. Install Ollama for LLM

```bash
# Install Ollama
curl -fsSL https://ollama.ai/install.sh | sh

# Pull a model
ollama pull llama2

# Test
ollama run llama2 "What is 2+2?"
```

### 3. Configure for Local Mode

```yaml
# In your launch file or params
voice_command_node:
  ros__parameters:
    use_api: false  # Use local Whisper

task_planner_node:
  ros__parameters:
    llm_provider: "ollama"
    model_name: "llama2"
```

---

## Next Steps

1. **Read Chapter 8** to understand voice-to-action pipeline
2. **Complete exercises** at the end of each chapter
3. **Build incrementally** - each chapter builds on the previous
4. **Join community** for help: [GitHub Discussions]

Happy building!
