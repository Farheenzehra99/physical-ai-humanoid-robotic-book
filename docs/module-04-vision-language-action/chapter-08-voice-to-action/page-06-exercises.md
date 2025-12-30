---
sidebar_position: 6
title: "Exercises and Challenges"
description: "Hands-on exercises to reinforce voice command concepts and extend functionality"
---

# Exercises and Challenges

## Overview

These exercises build progressively on the concepts covered in Chapter 8. Complete them in order to reinforce your understanding and create a production-ready voice command system.

---

## Exercise 1: Basic Voice Capture

**Objective**: Capture and visualize audio from your microphone.

### Task

Create a Python script that:
1. Captures 5 seconds of audio
2. Displays the audio waveform
3. Calculates and prints audio statistics (RMS, peak amplitude)

### Starter Code

```python
#!/usr/bin/env python3
"""Exercise 1: Basic voice capture and visualization."""

import sounddevice as sd
import numpy as np
import matplotlib.pyplot as plt

SAMPLE_RATE = 16000
DURATION = 5.0

def capture_audio(duration: float) -> np.ndarray:
    """Capture audio for specified duration."""
    # TODO: Implement audio capture
    pass

def calculate_stats(audio: np.ndarray) -> dict:
    """Calculate audio statistics."""
    # TODO: Calculate RMS, peak amplitude, etc.
    pass

def plot_waveform(audio: np.ndarray, sample_rate: int):
    """Plot audio waveform."""
    # TODO: Create time axis and plot
    pass

if __name__ == "__main__":
    print("Recording...")
    audio = capture_audio(DURATION)
    print("Done!")

    stats = calculate_stats(audio)
    print(f"Statistics: {stats}")

    plot_waveform(audio, SAMPLE_RATE)
    plt.show()
```

### Expected Output

- Console shows RMS level (typically 500-5000 for speech)
- Matplotlib window displays waveform with time on x-axis
- Visible speech patterns in the waveform

### Solution Hints

<details>
<summary>Click to reveal hints</summary>

- Use `sd.rec()` for simple recording
- RMS: `np.sqrt(np.mean(audio.astype(float)**2))`
- Time axis: `np.arange(len(audio)) / sample_rate`

</details>

---

## Exercise 2: Custom Command Vocabulary

**Objective**: Implement command recognition with a predefined vocabulary.

### Task

Create a command parser that:
1. Accepts a vocabulary of valid commands
2. Matches transcribed text to the closest command
3. Handles variations and synonyms
4. Returns confidence scores

### Vocabulary

```python
ROBOT_COMMANDS = {
    "move_forward": ["move forward", "go forward", "advance", "walk forward"],
    "move_backward": ["move backward", "go back", "reverse", "step back"],
    "turn_left": ["turn left", "rotate left", "go left"],
    "turn_right": ["turn right", "rotate right", "go right"],
    "stop": ["stop", "halt", "freeze", "emergency stop"],
    "pick_up": ["pick up", "grab", "grasp", "take"],
    "put_down": ["put down", "release", "drop", "place"],
}
```

### Starter Code

```python
#!/usr/bin/env python3
"""Exercise 2: Command vocabulary matching."""

from dataclasses import dataclass
from typing import Optional, List
import difflib

ROBOT_COMMANDS = {
    "move_forward": ["move forward", "go forward", "advance", "walk forward"],
    "move_backward": ["move backward", "go back", "reverse", "step back"],
    "turn_left": ["turn left", "rotate left", "go left"],
    "turn_right": ["turn right", "rotate right", "go right"],
    "stop": ["stop", "halt", "freeze", "emergency stop"],
    "pick_up": ["pick up", "grab", "grasp", "take"],
    "put_down": ["put down", "release", "drop", "place"],
}


@dataclass
class CommandMatch:
    """Matched command result."""
    command: str
    matched_phrase: str
    confidence: float
    original_text: str


class CommandMatcher:
    """Match transcribed text to robot commands."""

    def __init__(self, vocabulary: dict):
        """
        Initialize with command vocabulary.

        Args:
            vocabulary: Dict mapping command names to phrase lists
        """
        self.vocabulary = vocabulary
        # TODO: Build reverse lookup for efficiency

    def match(self, text: str, threshold: float = 0.6) -> Optional[CommandMatch]:
        """
        Match text to a command.

        Args:
            text: Transcribed text
            threshold: Minimum confidence for match

        Returns:
            CommandMatch if found, None otherwise
        """
        # TODO: Implement fuzzy matching
        pass

    def extract_parameters(self, text: str, command: str) -> dict:
        """
        Extract parameters from command text.

        Example: "move forward 2 meters" -> {"distance": 2, "unit": "meters"}
        """
        # TODO: Implement parameter extraction
        pass


# Test the matcher
if __name__ == "__main__":
    matcher = CommandMatcher(ROBOT_COMMANDS)

    test_inputs = [
        "move forward",
        "go forwards please",
        "can you turn left",
        "stop now",
        "pick that up",
        "move forward 2 meters",
        "random gibberish",
    ]

    for text in test_inputs:
        result = matcher.match(text)
        if result:
            print(f"'{text}' -> {result.command} (conf={result.confidence:.2f})")
        else:
            print(f"'{text}' -> NO MATCH")
```

### Expected Output

```
'move forward' -> move_forward (conf=1.00)
'go forwards please' -> move_forward (conf=0.75)
'can you turn left' -> turn_left (conf=0.80)
'stop now' -> stop (conf=0.85)
'pick that up' -> pick_up (conf=0.70)
'move forward 2 meters' -> move_forward (conf=0.90)
'random gibberish' -> NO MATCH
```

### Solution Hints

<details>
<summary>Click to reveal hints</summary>

- Use `difflib.SequenceMatcher` for fuzzy matching
- Normalize text: lowercase, strip punctuation
- Try matching against each phrase in vocabulary
- Keep best match above threshold
- For parameters, use regex patterns like `(\d+)\s*(meters?|feet|cm)`

</details>

---

## Exercise 3: Noise Handling

**Objective**: Improve voice recognition in noisy environments.

### Task

Implement noise reduction strategies:
1. Background noise estimation
2. Simple noise gate
3. Adaptive VAD threshold
4. Signal-to-noise ratio calculation

### Starter Code

```python
#!/usr/bin/env python3
"""Exercise 3: Noise handling for voice recognition."""

import numpy as np
import sounddevice as sd
from collections import deque

SAMPLE_RATE = 16000
CHUNK_SIZE = 480  # 30ms


class NoiseHandler:
    """Handle background noise for improved voice detection."""

    def __init__(self, noise_floor_percentile: float = 10.0):
        """
        Initialize noise handler.

        Args:
            noise_floor_percentile: Percentile for noise floor estimation
        """
        self.noise_floor_percentile = noise_floor_percentile
        self.energy_history = deque(maxlen=100)  # ~3 seconds of history
        self.noise_floor = 0.0

    def update(self, audio_chunk: np.ndarray):
        """Update noise estimation with new audio chunk."""
        # TODO: Calculate chunk energy and update history
        pass

    def estimate_noise_floor(self) -> float:
        """Estimate current noise floor from history."""
        # TODO: Use percentile-based estimation
        pass

    def apply_noise_gate(
        self,
        audio: np.ndarray,
        threshold_db: float = -40.0
    ) -> np.ndarray:
        """
        Apply noise gate to audio.

        Args:
            audio: Input audio samples
            threshold_db: Gate threshold in dB

        Returns:
            Gated audio
        """
        # TODO: Implement noise gate
        pass

    def calculate_snr(self, audio: np.ndarray) -> float:
        """
        Calculate signal-to-noise ratio.

        Returns:
            SNR in dB
        """
        # TODO: Calculate SNR using noise floor
        pass

    def is_speech(self, audio_chunk: np.ndarray, snr_threshold: float = 10.0) -> bool:
        """
        Determine if chunk contains speech based on SNR.

        Args:
            audio_chunk: Audio samples
            snr_threshold: Minimum SNR for speech (dB)

        Returns:
            True if likely speech
        """
        # TODO: Implement SNR-based speech detection
        pass


def test_noise_handler():
    """Test noise handling with live audio."""
    handler = NoiseHandler()

    print("Calibrating noise floor (stay quiet for 3 seconds)...")

    # Calibration phase
    for _ in range(100):
        audio = sd.rec(CHUNK_SIZE, samplerate=SAMPLE_RATE, channels=1, dtype=np.int16)
        sd.wait()
        handler.update(audio.flatten())

    print(f"Noise floor: {handler.noise_floor:.0f}")
    print("\nNow speak! Press Ctrl+C to stop.\n")

    # Detection phase
    try:
        while True:
            audio = sd.rec(CHUNK_SIZE, samplerate=SAMPLE_RATE, channels=1, dtype=np.int16)
            sd.wait()
            chunk = audio.flatten()

            handler.update(chunk)
            snr = handler.calculate_snr(chunk)
            is_speech = handler.is_speech(chunk)

            status = "SPEECH" if is_speech else "noise"
            print(f"SNR: {snr:6.1f} dB | {status}")

    except KeyboardInterrupt:
        print("\nDone!")


if __name__ == "__main__":
    test_noise_handler()
```

### Expected Output

```
Calibrating noise floor (stay quiet for 3 seconds)...
Noise floor: 245

Now speak! Press Ctrl+C to stop.

SNR:   -2.3 dB | noise
SNR:   -1.8 dB | noise
SNR:   15.4 dB | SPEECH
SNR:   18.2 dB | SPEECH
SNR:   12.1 dB | SPEECH
SNR:    3.2 dB | noise
```

### Solution Hints

<details>
<summary>Click to reveal hints</summary>

- Energy: `np.sqrt(np.mean(chunk.astype(float)**2))`
- Noise floor: `np.percentile(list(self.energy_history), self.noise_floor_percentile)`
- SNR: `20 * np.log10(signal_energy / noise_floor)` (handle division by zero)
- Noise gate: Zero samples below threshold with smooth fade

</details>

---

## Challenge: Multi-Language Support

**Objective**: Extend the voice command system to support multiple languages.

### Requirements

1. **Language Detection**: Automatically detect the spoken language
2. **Multi-language Commands**: Map commands in different languages to actions
3. **Feedback**: Respond in the detected language
4. **Seamless Switching**: Handle language changes mid-session

### Command Mappings

```python
MULTILINGUAL_COMMANDS = {
    "move_forward": {
        "en": ["move forward", "go forward"],
        "es": ["avanzar", "ir adelante", "caminar"],
        "ur": ["آگے چلو", "آگے بڑھو"],
        "de": ["vorwärts", "nach vorne"],
    },
    "stop": {
        "en": ["stop", "halt"],
        "es": ["para", "detente", "alto"],
        "ur": ["رکو", "بس"],
        "de": ["stopp", "halt", "anhalten"],
    },
}

FEEDBACK_TEMPLATES = {
    "en": "Executing: {command}",
    "es": "Ejecutando: {command}",
    "ur": "عمل کر رہا ہوں: {command}",
    "de": "Ausführen: {command}",
}
```

### Starter Code

```python
#!/usr/bin/env python3
"""Challenge: Multi-language voice command support."""

from dataclasses import dataclass
from typing import Optional, Tuple
from openai import OpenAI

MULTILINGUAL_COMMANDS = {
    "move_forward": {
        "en": ["move forward", "go forward"],
        "es": ["avanzar", "ir adelante"],
        "ur": ["آگے چلو", "آگے بڑھو"],
    },
    "stop": {
        "en": ["stop", "halt"],
        "es": ["para", "detente"],
        "ur": ["رکو", "بس"],
    },
}


@dataclass
class MultiLangCommand:
    """Multi-language command result."""
    command: str
    language: str
    original_text: str
    confidence: float
    feedback: str


class MultiLanguageVoiceCommands:
    """Voice commands with multi-language support."""

    def __init__(self):
        self.client = OpenAI()
        self.commands = MULTILINGUAL_COMMANDS
        self.last_language = "en"

    def transcribe_with_language(self, audio_path: str) -> Tuple[str, str]:
        """
        Transcribe audio and detect language.

        Returns:
            (transcription, language_code)
        """
        # TODO: Use Whisper with language detection
        pass

    def match_command(
        self,
        text: str,
        language: str
    ) -> Optional[Tuple[str, float]]:
        """
        Match text to command in detected language.

        Returns:
            (command_name, confidence) or None
        """
        # TODO: Match against language-specific phrases
        pass

    def generate_feedback(self, command: str, language: str) -> str:
        """Generate feedback message in detected language."""
        # TODO: Use template or generate with LLM
        pass

    def process(self, audio_path: str) -> Optional[MultiLangCommand]:
        """
        Process audio file and return command.

        Args:
            audio_path: Path to audio file

        Returns:
            MultiLangCommand if recognized, None otherwise
        """
        # TODO: Complete pipeline
        pass


def test_multilingual():
    """Test multi-language support."""
    processor = MultiLanguageVoiceCommands()

    # Test with sample audio files
    test_files = [
        ("test_en.wav", "en"),
        ("test_es.wav", "es"),
        ("test_ur.wav", "ur"),
    ]

    for audio_file, expected_lang in test_files:
        print(f"\nProcessing: {audio_file}")
        result = processor.process(audio_file)

        if result:
            print(f"  Language: {result.language}")
            print(f"  Command: {result.command}")
            print(f"  Feedback: {result.feedback}")
        else:
            print("  No command recognized")


if __name__ == "__main__":
    test_multilingual()
```

### Evaluation Criteria

Your solution should:

- [ ] Correctly detect language from Whisper output
- [ ] Match commands in at least 3 languages
- [ ] Generate appropriate feedback in detected language
- [ ] Handle language switching gracefully
- [ ] Work with real audio input

### Bonus Points

- Implement confidence-based language fallback
- Add support for code-switching (mixed languages)
- Create a ROS 2 node with language parameter

---

## Summary

### What You've Learned

In this chapter, you've built a complete voice-to-ROS 2 pipeline:

| Component | Technology | Purpose |
|-----------|------------|---------|
| Audio Capture | sounddevice | Real-time microphone input |
| Voice Detection | WebRTC VAD | Filter silence, detect speech |
| Transcription | OpenAI Whisper | Convert speech to text |
| ROS 2 Integration | rclpy, custom msgs | Publish commands to robot |

### Key Takeaways

1. **Whisper** provides robust, multilingual speech recognition
2. **VAD** is essential for real-time voice processing
3. **Custom messages** enable rich command metadata
4. **Error handling** ensures reliable operation

### Next Steps

With voice commands working, you're ready for **Chapter 9: Cognitive Planning**, where you'll:

- Use LLMs to interpret natural language commands
- Generate action sequences for complex tasks
- Build the bridge between voice input and robot actions

---

## Additional Resources

### Documentation

- [OpenAI Whisper API](https://platform.openai.com/docs/guides/speech-to-text)
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [sounddevice Documentation](https://python-sounddevice.readthedocs.io/)
- [WebRTC VAD](https://github.com/wiseman/py-webrtcvad)

### Community Projects

- [whisper.cpp](https://github.com/ggerganov/whisper.cpp) - Efficient C++ implementation
- [faster-whisper](https://github.com/guillaumekln/faster-whisper) - CTranslate2 optimized
- [whisper-streaming](https://github.com/ufal/whisper_streaming) - Real-time streaming

### Related Research

- "Robust Speech Recognition in Robotics" - IEEE ICRA
- "Voice-Activated Robot Control: A Survey" - Journal of Field Robotics
- "Multi-Modal Human-Robot Interaction" - HRI Conference
