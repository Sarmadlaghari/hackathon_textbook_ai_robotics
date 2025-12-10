---
title: Voice-to-Action with OpenAI Whisper
sidebar_label: Week 9 - Voice-to-Action with OpenAI Whisper
sidebar_position: 9
---

# Week 9: Voice-to-Action with OpenAI Whisper

## Introduction

Welcome to Week 9 of the Vision-Language-Action (VLA) module! This week we'll explore how to implement voice-to-action systems using OpenAI Whisper for speech recognition. We'll learn how to convert spoken commands into actionable robot behaviors, creating intuitive human-robot interaction interfaces. This technology enables robots to understand natural language commands and execute corresponding actions in real-world environments.

## Learning Objectives

By the end of this week, you will be able to:
- Understand the fundamentals of speech recognition and voice-to-action systems
- Install and configure OpenAI Whisper for real-time speech recognition
- Process audio input and convert speech to text commands
- Map recognized commands to robot actions
- Integrate voice commands with ROS 2 control systems

## Prerequisites

Before starting this week's content, ensure you have:
- Understanding of ROS 2 fundamentals (Weeks 1-3)
- Basic knowledge of audio processing concepts
- Experience with Python programming
- Familiarity with natural language processing concepts

## 1. Introduction to Voice-to-Action Systems

### 1.1 What are Voice-to-Action Systems?

Voice-to-action systems enable robots to:
- Recognize spoken commands using speech recognition
- Interpret natural language instructions
- Convert voice commands into executable robot actions
- Provide intuitive human-robot interaction

### 1.2 Applications in Robotics

- Assistive robotics for elderly care
- Industrial automation with voice commands
- Educational robotics
- Service robotics in homes and offices
- Search and rescue operations

### 1.3 System Architecture

A typical voice-to-action system includes:
- **Audio Input**: Microphones for capturing speech
- **Speech Recognition**: Converting audio to text
- **Natural Language Processing**: Understanding command intent
- **Action Mapping**: Converting commands to robot actions
- **Execution**: Robot control and feedback

## 2. OpenAI Whisper for Speech Recognition

### 2.1 What is OpenAI Whisper?

OpenAI Whisper is a state-of-the-art speech recognition model that:
- Provides high accuracy across multiple languages
- Handles various accents and speaking styles
- Works well in noisy environments
- Supports real-time and batch processing
- Is available as an open-source model

### 2.2 Whisper Model Variants

- **tiny**: Fastest, least accurate (76MB)
- **base**: Good balance of speed and accuracy (145MB)
- **small**: Better accuracy, moderate speed (484MB)
- **medium**: High accuracy, slower (1.5GB)
- **large**: Highest accuracy, slowest (3.0GB)

### 2.3 Installation and Setup

```bash
pip install openai-whisper
# For GPU acceleration
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

### 2.4 Basic Whisper Usage

```python
import whisper

# Load model (downloads if not present)
model = whisper.load_model("small")

# Transcribe audio file
result = model.transcribe("command.wav")
print(result["text"])
```

## 3. Real-Time Voice Recognition

### 3.1 Audio Input Processing

For real-time voice recognition, we need to:
- Capture audio from microphone
- Process audio in chunks
- Handle streaming input efficiently
- Filter out background noise

### 3.2 Audio Stream Processing

```python
import pyaudio
import numpy as np
import queue
import threading
import whisper
import torch

class VoiceToAction:
    def __init__(self, model_size="small"):
        # Initialize Whisper model
        self.model = whisper.load_model(model_size)
        self.audio_queue = queue.Queue()

        # Audio parameters
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        self.chunk = 1024

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

    def start_listening(self):
        # Open audio stream
        stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        # Start audio recording thread
        threading.Thread(target=self.record_audio, args=(stream,), daemon=True).start()

    def record_audio(self, stream):
        while True:
            data = stream.read(self.chunk)
            self.audio_queue.put(data)
```

### 3.3 Voice Activity Detection

Implement voice activity detection to:
- Detect when speech starts and ends
- Reduce unnecessary processing
- Improve real-time performance
- Handle background noise

## 4. Natural Language Understanding

### 4.1 Command Parsing

Convert recognized text into structured commands:
- Extract action verbs (move, pick, place, etc.)
- Identify objects and locations
- Parse numerical parameters
- Handle complex multi-step commands

### 4.2 Intent Recognition

```python
import re

class CommandParser:
    def __init__(self):
        # Define command patterns
        self.move_patterns = [
            r'move to (.+)',
            r'go to (.+)',
            r'go to the (.+)',
            r'navigate to (.+)'
        ]

        self.pick_patterns = [
            r'pick up the (.+)',
            r'pick the (.+)',
            r'grab the (.+)',
            r'take the (.+)'
        ]

        self.place_patterns = [
            r'place it on the (.+)',
            r'put it on the (.+)',
            r'place the (.+) on the (.+)',
            r'put the (.+) on the (.+)'
        ]

    def parse_command(self, text):
        text = text.lower().strip()

        # Check move patterns
        for pattern in self.move_patterns:
            match = re.search(pattern, text)
            if match:
                return {'action': 'move', 'target': match.group(1)}

        # Check pick patterns
        for pattern in self.pick_patterns:
            match = re.search(pattern, text)
            if match:
                return {'action': 'pick', 'object': match.group(1)}

        # Check place patterns
        for pattern in self.place_patterns:
            match = re.search(pattern, text)
            if match:
                if len(match.groups()) == 1:
                    return {'action': 'place', 'target': match.group(1)}
                else:
                    return {'action': 'place', 'object': match.group(1), 'target': match.group(2)}

        return {'action': 'unknown', 'raw': text}
```

### 4.3 Context Awareness

Implement context awareness for:
- Understanding relative positions
- Handling ambiguous commands
- Maintaining conversation state
- Learning user preferences

## 5. Integration with ROS 2

### 5.1 ROS 2 Node Structure

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import AudioData

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # Publishers for robot commands
        self.move_pub = self.create_publisher(Pose, 'move_command', 10)
        self.action_pub = self.create_publisher(String, 'action_command', 10)

        # Subscriber for audio input
        self.audio_sub = self.create_subscription(
            AudioData, 'audio_input', self.audio_callback, 10)

        # Timer for processing audio queue
        self.timer = self.create_timer(0.1, self.process_audio)

        # Initialize Whisper model
        self.whisper_model = whisper.load_model("small")

    def audio_callback(self, msg):
        # Process audio data and convert to text
        audio_array = np.frombuffer(msg.data, dtype=np.int16)
        result = self.whisper_model.transcribe(audio_array)
        command_text = result["text"]

        # Parse and execute command
        self.execute_command(command_text)

    def execute_command(self, command_text):
        parser = CommandParser()
        parsed_command = parser.parse_command(command_text)

        if parsed_command['action'] == 'move':
            self.send_move_command(parsed_command['target'])
        elif parsed_command['action'] == 'pick':
            self.send_pick_command(parsed_command['object'])
        elif parsed_command['action'] == 'place':
            self.send_place_command(parsed_command['target'])
```

### 5.2 Action Mapping

Map recognized commands to ROS 2 actions:
- Navigation commands → Navigation2 stack
- Manipulation commands → MoveIt! or custom controllers
- System commands → Service calls
- Query commands → Parameter requests

## 6. Voice Command Vocabulary

### 6.1 Basic Navigation Commands

- "Go to the kitchen"
- "Move to the table"
- "Navigate to the charging station"
- "Return to base"

### 6.2 Manipulation Commands

- "Pick up the red cup"
- "Place the book on the shelf"
- "Open the door"
- "Close the drawer"

### 6.3 System Commands

- "Stop" or "Halt"
- "Pause"
- "Resume"
- "Shutdown"
- "Status"

## 7. Performance Optimization

### 7.1 Model Optimization

- Use appropriate model size for your hardware
- Implement model quantization
- Use GPU acceleration when available
- Cache frequently used models

### 7.2 Real-time Processing

- Optimize audio buffer sizes
- Use efficient threading
- Implement command queuing
- Handle processing delays gracefully

### 7.3 Accuracy Improvements

- Train custom language models
- Implement command confirmation
- Use context-aware recognition
- Add error correction mechanisms

## 8. Advanced Features

### 8.1 Multi-language Support

Whisper supports multiple languages:
- English, German, French, Spanish, Italian
- Portuguese, Polish, Chinese, Japanese, Korean
- And many more languages

### 8.2 Custom Training

Fine-tune Whisper for:
- Domain-specific vocabulary
- Specific accents or dialects
- Noisy environments
- Specialized command sets

### 8.3 Voice Authentication

Implement voice biometrics for:
- User identification
- Security verification
- Personalized responses
- Access control

## 9. Practical Implementation

### 9.1 Complete Voice-to-Action System

```python
import rclpy
import whisper
import pyaudio
import numpy as np
import threading
import queue
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose

class CompleteVoiceToActionNode(Node):
    def __init__(self):
        super().__init__('complete_voice_to_action')

        # Initialize Whisper model
        self.model = whisper.load_model("small")

        # Audio processing setup
        self.audio_queue = queue.Queue()
        self.setup_audio()

        # ROS 2 publishers
        self.command_pub = self.create_publisher(String, 'robot_commands', 10)

        # Start audio processing thread
        self.processing_thread = threading.Thread(
            target=self.process_audio_stream, daemon=True)
        self.processing_thread.start()

    def setup_audio(self):
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=1024
        )

    def process_audio_stream(self):
        while rclpy.ok():
            # Read audio chunk
            data = self.stream.read(1024)
            audio_array = np.frombuffer(data, dtype=np.int16)

            # Process with Whisper
            result = self.model.transcribe(audio_array)
            text = result["text"]

            if text.strip():  # If we have recognized text
                self.process_command(text)

    def process_command(self, text):
        # Publish command to robot
        msg = String()
        msg.data = text
        self.command_pub.publish(msg)

        self.get_logger().info(f'Recognized: {text}')
```

## 10. Testing and Validation

### 10.1 Unit Testing

Test individual components:
- Audio input processing
- Speech recognition accuracy
- Command parsing
- ROS 2 integration

### 10.2 Integration Testing

Test the complete system:
- End-to-end voice command processing
- Robot response accuracy
- System robustness
- Error handling

## Exercises

1. **Basic Setup**: Install Whisper and test speech recognition with audio files
2. **Real-time Processing**: Implement real-time audio capture and recognition
3. **Command Mapping**: Create a command parser for navigation tasks
4. **ROS Integration**: Integrate voice commands with a simple ROS 2 navigation system

## Summary

This week we explored voice-to-action systems using OpenAI Whisper for speech recognition. We learned how to process audio input, recognize speech commands, parse natural language, and integrate with ROS 2 systems. Voice-to-action technology provides an intuitive interface for human-robot interaction, enabling robots to understand and respond to natural language commands.

## References

- [OpenAI Whisper Documentation](https://github.com/openai/whisper)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [PyAudio Documentation](https://pyaudio.readthedocs.io/)
- [Speech Recognition in Robotics](https://arxiv.org/abs/2103.13233)