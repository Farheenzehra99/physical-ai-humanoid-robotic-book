---
sidebar_position: 3
title: "Setting Up Whisper"
description: "Step-by-step setup for OpenAI Whisper API and local deployment options"
---

# Setting Up Whisper

## Learning Objectives

By the end of this section, you will be able to:

- Configure the OpenAI Whisper API for cloud transcription
- Install and run Whisper locally using Python
- Set up whisper.cpp for optimized local inference
- Verify your installation with test commands

---

## Prerequisites

Before starting, ensure you have:

- Ubuntu 22.04 LTS
- Python 3.10 or higher
- pip package manager
- (Optional) NVIDIA GPU with CUDA for faster local inference
- (Optional) OpenAI API key for cloud transcription

---

## Option 1: OpenAI Whisper API

The fastest way to get started is using OpenAI's cloud API.

### Step 1: Get an API Key

1. Visit [platform.openai.com](https://platform.openai.com)
2. Create an account or sign in
3. Navigate to **API Keys** section
4. Click **Create new secret key**
5. Copy and save the key securely

### Step 2: Install the OpenAI Package

```bash
# Create a virtual environment (recommended)
python3 -m venv ~/vla_env
source ~/vla_env/bin/activate

# Install the OpenAI Python package
pip install openai
```

### Step 3: Set Your API Key

```bash
# Add to your ~/.bashrc or ~/.zshrc
export OPENAI_API_KEY="sk-your-api-key-here"

# Reload the shell configuration
source ~/.bashrc
```

:::warning Security
Never commit your API key to version control. Use environment variables or a secrets manager.
:::

### Step 4: Test the API

Create a test file `test_whisper_api.py`:

```python
#!/usr/bin/env python3
"""Test OpenAI Whisper API connection."""

from openai import OpenAI
import tempfile
import wave
import numpy as np

def create_test_audio():
    """Create a silent audio file for testing."""
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        with wave.open(f.name, "w") as wav:
            wav.setnchannels(1)
            wav.setsampwidth(2)
            wav.setframerate(16000)
            # 1 second of silence
            wav.writeframes(np.zeros(16000, dtype=np.int16).tobytes())
        return f.name

def test_whisper_api():
    """Test the Whisper API connection."""
    client = OpenAI()

    # Create test audio
    audio_path = create_test_audio()

    try:
        with open(audio_path, "rb") as audio_file:
            transcript = client.audio.transcriptions.create(
                model="whisper-1",
                file=audio_file
            )
        print("API Connection: SUCCESS")
        print(f"Transcript: '{transcript.text}'")
        return True
    except Exception as e:
        print(f"API Connection: FAILED")
        print(f"Error: {e}")
        return False

if __name__ == "__main__":
    test_whisper_api()
```

Run the test:

```bash
python3 test_whisper_api.py
```

Expected output:
```
API Connection: SUCCESS
Transcript: ''
```

---

## Option 2: Local Whisper (Python Package)

For offline use or when you want full control.

### Step 1: Install Dependencies

```bash
# System dependencies for audio processing
sudo apt update
sudo apt install ffmpeg portaudio19-dev

# Create virtual environment
python3 -m venv ~/vla_env
source ~/vla_env/bin/activate

# Install Whisper and dependencies
pip install openai-whisper torch torchaudio
```

### Step 2: Verify GPU Support (Optional but Recommended)

```bash
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
```

If CUDA is available, Whisper will automatically use your GPU for faster inference.

### Step 3: Download a Model

Models are downloaded automatically on first use, but you can pre-download:

```python
import whisper

# Download the 'base' model (~140MB)
model = whisper.load_model("base")
print("Model loaded successfully!")
```

### Step 4: Test Local Whisper

Create a test file `test_whisper_local.py`:

```python
#!/usr/bin/env python3
"""Test local Whisper installation."""

import whisper
import tempfile
import wave
import numpy as np
import time

def create_test_audio():
    """Create a test audio file."""
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        with wave.open(f.name, "w") as wav:
            wav.setnchannels(1)
            wav.setsampwidth(2)
            wav.setframerate(16000)
            # 1 second of silence
            wav.writeframes(np.zeros(16000, dtype=np.int16).tobytes())
        return f.name

def test_local_whisper():
    """Test local Whisper model."""
    print("Loading Whisper model...")
    start = time.time()
    model = whisper.load_model("base")
    print(f"Model loaded in {time.time() - start:.1f}s")

    # Create test audio
    audio_path = create_test_audio()

    # Transcribe
    print("Transcribing test audio...")
    start = time.time()
    result = model.transcribe(audio_path)
    print(f"Transcription completed in {time.time() - start:.1f}s")

    print(f"Transcript: '{result['text']}'")
    print("Local Whisper: SUCCESS")

if __name__ == "__main__":
    test_local_whisper()
```

Run the test:

```bash
python3 test_whisper_local.py
```

Expected output:
```
Loading Whisper model...
Model loaded in 2.3s
Transcribing test audio...
Transcription completed in 0.5s
Transcript: ''
Local Whisper: SUCCESS
```

---

## Option 3: whisper.cpp (High Performance)

For edge deployment on Jetson or CPU-only systems.

### Step 1: Clone and Build

```bash
# Clone the repository
git clone https://github.com/ggerganov/whisper.cpp.git
cd whisper.cpp

# Build with optimizations
make clean
make -j$(nproc)
```

### Step 2: Download Models

```bash
# Download the base model in ggml format
bash ./models/download-ggml-model.sh base.en
```

### Step 3: Test whisper.cpp

Record a test audio file:

```bash
# Record 3 seconds of audio
arecord -f S16_LE -r 16000 -d 3 test.wav

# Transcribe with whisper.cpp
./main -m models/ggml-base.en.bin -f test.wav
```

### Python Bindings for whisper.cpp

For ROS 2 integration, use the Python bindings:

```bash
pip install pywhispercpp
```

```python
from pywhispercpp.model import Model

model = Model("base.en")
result = model.transcribe("test.wav")
print(result)
```

---

## Python Environment Setup for ROS 2

To use Whisper with ROS 2, create a unified environment:

```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Create Python package for voice commands
cd src
ros2 pkg create --build-type ament_python voice_commands \
    --dependencies rclpy std_msgs

# Install Python dependencies in the ROS 2 environment
pip install openai sounddevice webrtcvad numpy
```

### Environment Verification Script

Create `verify_setup.py`:

```python
#!/usr/bin/env python3
"""Verify VLA environment setup."""

import sys

checks = []

# Python version
checks.append(("Python 3.10+", sys.version_info >= (3, 10)))

# OpenAI package
try:
    import openai
    checks.append(("openai package", True))
except ImportError:
    checks.append(("openai package", False))

# Sounddevice
try:
    import sounddevice as sd
    devices = sd.query_devices()
    has_input = any(d['max_input_channels'] > 0 for d in devices)
    checks.append(("sounddevice + microphone", has_input))
except ImportError:
    checks.append(("sounddevice package", False))
except Exception as e:
    checks.append(("sounddevice package", False))

# WebRTC VAD
try:
    import webrtcvad
    checks.append(("webrtcvad package", True))
except ImportError:
    checks.append(("webrtcvad package", False))

# NumPy
try:
    import numpy
    checks.append(("numpy package", True))
except ImportError:
    checks.append(("numpy package", False))

# API key
import os
checks.append(("OPENAI_API_KEY set", bool(os.getenv("OPENAI_API_KEY"))))

# Local Whisper (optional)
try:
    import whisper
    checks.append(("Local Whisper (optional)", True))
except ImportError:
    checks.append(("Local Whisper (optional)", "SKIP"))

# Results
print("\n=== VLA Environment Verification ===\n")
all_pass = True
for name, passed in checks:
    if passed == "SKIP":
        status = "SKIP"
    elif passed:
        status = "PASS"
    else:
        status = "FAIL"
        all_pass = False
    print(f"  [{status}] {name}")

print()
if all_pass:
    print("All required checks passed! Ready for Chapter 8.")
else:
    print("Some checks failed. Review installation steps above.")
```

Run verification:

```bash
python3 verify_setup.py
```

---

## Troubleshooting

### "No module named 'openai'"

```bash
pip install openai
```

### "OPENAI_API_KEY not set"

```bash
export OPENAI_API_KEY="your-key-here"
# Add to ~/.bashrc for persistence
```

### "No input devices available"

Check microphone connection:
```bash
arecord -l  # List capture devices
```

Grant audio permissions:
```bash
sudo usermod -a -G audio $USER
# Log out and back in
```

### "CUDA out of memory" (Local Whisper)

Use a smaller model:
```python
model = whisper.load_model("tiny")  # or "base"
```

### "ffmpeg not found"

```bash
sudo apt install ffmpeg
```

---

## Summary

You now have Whisper set up for voice recognition:

| Option | Best For | Setup Complexity |
|--------|----------|------------------|
| OpenAI API | Quick start, cloud | Easy |
| Local Whisper | Privacy, offline | Medium |
| whisper.cpp | Edge, performance | Advanced |

**Next**: We'll build the voice command pipeline, capturing audio and streaming it to Whisper for real-time transcription.
