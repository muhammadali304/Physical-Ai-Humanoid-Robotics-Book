# Voice Command Processing System

This document provides a comprehensive guide for implementing a voice command processing system for robotics applications.

## Overview

The voice command processing system enables natural human-robot interaction through spoken commands. This system handles speech recognition, natural language processing, and command execution for robotics applications.

## System Architecture

### 1. High-Level Architecture
```
Voice Input → Speech Recognition → Natural Language Processing → Command Execution → Robot Actions
```

### 2. Component Breakdown
- **Audio Input Module**: Captures and preprocesses audio from microphones
- **Speech Recognition**: Converts speech to text using local or cloud-based ASR
- **Intent Recognition**: Parses text to identify user intent and parameters
- **Command Mapping**: Maps intents to specific robot actions
- **Execution Module**: Executes commands safely on the robot

## Implementation Components

### 1. Audio Input and Preprocessing

```python
# audio_input.py
import pyaudio
import numpy as np
import webrtcvad
import collections
import queue
from typing import Generator, Tuple
import threading
import time

class AudioInput:
    """
    Audio input module with voice activity detection
    """
    def __init__(self, sample_rate: int = 16000, chunk_size: int = 320):
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.audio = pyaudio.PyAudio()

        # Voice activity detection
        self.vad = webrtcvad.Vad(2)  # Aggressiveness level 2
        self.ring_buffer = collections.deque(maxlen=300)  # 300 frames
        self.triggered = False
        self.vad_frames = []

        # Audio stream
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=sample_rate,
            input=True,
            frames_per_buffer=chunk_size
        )

        # Audio queue for processing
        self.audio_queue = queue.Queue()
        self.is_listening = False

    def start_listening(self):
        """Start audio capture in a separate thread"""
        self.is_listening = True
        self.capture_thread = threading.Thread(target=self._capture_audio)
        self.capture_thread.daemon = True
        self.capture_thread.start()

    def stop_listening(self):
        """Stop audio capture"""
        self.is_listening = False
        if hasattr(self, 'capture_thread'):
            self.capture_thread.join()

    def _capture_audio(self):
        """Capture audio in a loop"""
        while self.is_listening:
            audio_data = self.stream.read(self.chunk_size, exception_on_overflow=False)
            self.audio_queue.put(audio_data)

    def get_audio_stream(self) -> Generator[bytes, None, None]:
        """Generator that yields audio chunks when voice activity is detected"""
        while self.is_listening:
            try:
                audio_chunk = self.audio_queue.get(timeout=0.1)
                yield audio_chunk
            except queue.Empty:
                continue

    def detect_voice_activity(self, audio_chunk: bytes) -> bool:
        """Detect if voice activity is present in audio chunk"""
        # Convert to appropriate format for VAD
        # VAD requires 16kHz, 16-bit, mono audio in 10, 20, or 30ms chunks
        chunk_duration = len(audio_chunk) * 1000 // (self.sample_rate * 2)  # Duration in ms

        if chunk_duration not in [10, 20, 30]:
            # Adjust chunk size or skip
            return False

        return self.vad.is_speech(audio_chunk, self.sample_rate)

    def close(self):
        """Close audio resources"""
        self.is_listening = False
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()
```

### 2. Speech Recognition Module

```python
# speech_recognition.py
import speech_recognition as sr
import asyncio
import logging
from typing import Optional
import tempfile
import os
import wave

class SpeechRecognizer:
    """
    Speech recognition module with multiple backend support
    """
    def __init__(self, backend: str = "google"):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.backend = backend

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=1.0)

        # Set energy threshold
        self.recognizer.energy_threshold = 4000

        # Supported backends
        self.supported_backends = {
            "google": self._recognize_google,
            "wit": self._recognize_wit,
            "houndify": self._recognize_houndify,
            "ibm": self._recognize_ibm
        }

        self.logger = logging.getLogger(__name__)

    def recognize_speech(self, audio_data: sr.AudioData) -> Optional[str]:
        """
        Recognize speech from audio data using configured backend
        """
        if self.backend not in self.supported_backends:
            raise ValueError(f"Unsupported backend: {self.backend}")

        try:
            result = self.supported_backends[self.backend](audio_data)
            self.logger.info(f"Recognized speech: {result}")
            return result
        except Exception as e:
            self.logger.error(f"Speech recognition error: {e}")
            return None

    def _recognize_google(self, audio_data: sr.AudioData) -> str:
        """Google Web Speech API recognition"""
        return self.recognizer.recognize_google(audio_data)

    def _recognize_wit(self, audio_data: sr.AudioData) -> str:
        """Wit.ai recognition (requires API key)"""
        # This would require wit.ai API key configuration
        # return self.recognizer.recognize_wit(audio_data, key="YOUR_WIT_AI_KEY")
        raise NotImplementedError("Wit.ai integration requires API key setup")

    def _recognize_houndify(self, audio_data: sr.AudioData) -> str:
        """Houndify recognition (requires client ID and key)"""
        # This would require Houndify credentials
        # return self.recognizer.recognize_houndify(audio_data, client_id="YOUR_CLIENT_ID", client_key="YOUR_CLIENT_KEY")
        raise NotImplementedError("Houndify integration requires credentials")

    def _recognize_ibm(self, audio_data: sr.AudioData) -> str:
        """IBM Watson recognition (requires credentials)"""
        # This would require IBM Watson credentials
        # return self.recognizer.recognize_ibm(audio_data, username="YOUR_USERNAME", password="YOUR_PASSWORD")
        raise NotImplementedError("IBM Watson integration requires credentials")

    def listen_for_command(self, timeout: int = 5, phrase_time_limit: int = 10) -> Optional[str]:
        """
        Listen for a voice command using the microphone
        """
        try:
            with self.microphone as source:
                self.logger.info("Listening for voice command...")
                audio = self.recognizer.listen(source, timeout=timeout, phrase_time_limit=phrase_time_limit)

            return self.recognize_speech(audio)
        except sr.WaitTimeoutError:
            self.logger.warning("Timeout waiting for speech")
            return None
        except sr.UnknownValueError:
            self.logger.warning("Could not understand audio")
            return None
        except sr.RequestError as e:
            self.logger.error(f"Speech recognition service error: {e}")
            return None

# Example of using Whisper for local speech recognition
class LocalSpeechRecognizer:
    """
    Local speech recognition using OpenAI Whisper (requires installation)
    """
    def __init__(self, model_size: str = "base"):
        try:
            import whisper
            self.model = whisper.load_model(model_size)
        except ImportError:
            raise ImportError("Please install whisper: pip install openai-whisper")

    def recognize_from_audio_data(self, audio_data: sr.AudioData) -> Optional[str]:
        """
        Recognize speech using local Whisper model
        """
        import io
        import numpy as np

        # Convert audio data to WAV format
        wav_buffer = io.BytesIO()
        with wave.open(wav_buffer, 'wb') as wav_file:
            wav_file.setnchannels(1)  # Mono
            wav_file.setsampwidth(2)  # 16-bit
            wav_file.setframerate(16000)  # 16kHz
            wav_file.writeframes(audio_data.frame_data)

        # Convert to numpy array for Whisper
        wav_buffer.seek(0)
        audio_array = np.frombuffer(wav_buffer.read(), dtype=np.int16).astype(np.float32) / 32768.0

        # Transcribe
        result = self.model.transcribe(audio_array)
        return result["text"]
```

### 3. Natural Language Processing and Intent Recognition

```python
# nlp_processor.py
import re
import spacy
import logging
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass

@dataclass
class CommandIntent:
    """Structured representation of a recognized command"""
    intent: str
    parameters: Dict[str, str]
    confidence: float
    raw_text: str

class NLPProcessor:
    """
    Natural language processing module for intent recognition
    """
    def __init__(self):
        # Load spaCy model (install with: python -m spacy download en_core_web_sm)
        try:
            self.nlp = spacy.load("en_core_web_sm")
        except OSError:
            raise OSError("Please install spaCy English model: python -m spacy download en_core_web_sm")

        self.logger = logging.getLogger(__name__)

        # Define intent patterns
        self.intent_patterns = {
            "move_to": [
                r"move to (.+)",
                r"go to (.+)",
                r"navigate to (.+)",
                r"move towards (.+)",
                r"head to (.+)"
            ],
            "pick_up": [
                r"pick up (.+)",
                r"grasp (.+)",
                r"take (.+)",
                r"grab (.+)"
            ],
            "place": [
                r"place (.+) (?:at|on|in) (.+)",
                r"put (.+) (?:at|on|in) (.+)",
                r"drop (.+) (?:at|on|in) (.+)"
            ],
            "stop": [
                r"stop",
                r"halt",
                r"freeze",
                r"pause"
            ],
            "follow": [
                r"follow (.+)",
                r"follow me",
                r"come with me"
            ],
            "greet": [
                r"hello",
                r"hi",
                r"hey",
                r"greetings"
            ]
        }

        # Location keywords
        self.location_keywords = {
            "kitchen", "living room", "bedroom", "bathroom", "office",
            "dining room", "hallway", "garage", "garden", "entrance"
        }

    def process_command(self, text: str) -> Optional[CommandIntent]:
        """
        Process text command and extract intent and parameters
        """
        text = text.lower().strip()
        doc = self.nlp(text)

        # Try pattern matching first
        intent, params, confidence = self._match_patterns(text)

        if intent:
            # Enhance with NLP processing
            enhanced_params = self._enhance_parameters(params, doc)
            return CommandIntent(
                intent=intent,
                parameters=enhanced_params,
                confidence=confidence,
                raw_text=text
            )

        # If no pattern matched, try semantic analysis
        semantic_intent = self._analyze_semantic_intent(doc)
        if semantic_intent:
            return semantic_intent

        self.logger.warning(f"Could not identify intent for: {text}")
        return None

    def _match_patterns(self, text: str) -> Tuple[Optional[str], Dict, float]:
        """
        Match text against predefined patterns to identify intent
        """
        for intent, patterns in self.intent_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text)
                if match:
                    groups = match.groups()

                    if intent == "move_to":
                        return intent, {"location": groups[0]}, 0.9
                    elif intent == "pick_up":
                        return intent, {"object": groups[0]}, 0.9
                    elif intent == "place":
                        return intent, {"object": groups[0], "location": groups[1]}, 0.9
                    elif intent == "follow":
                        return intent, {"target": groups[0] if len(groups) > 0 else "user"}, 0.8
                    elif intent in ["stop", "greet"]:
                        return intent, {}, 0.95

        return None, {}, 0.0

    def _enhance_parameters(self, params: Dict, doc) -> Dict:
        """
        Enhance extracted parameters using NLP analysis
        """
        enhanced = params.copy()

        # Extract named entities
        for ent in doc.ents:
            if ent.label_ in ["PERSON", "GPE", "LOC", "FAC"]:
                enhanced[f"named_entity_{ent.label_.lower()}"] = ent.text

        # Extract adjectives and adverbs for context
        for token in doc:
            if token.pos_ == "ADJ":
                if "adjectives" not in enhanced:
                    enhanced["adjectives"] = []
                enhanced["adjectives"].append(token.text)
            elif token.pos_ == "ADV":
                if "adverbs" not in enhanced:
                    enhanced["adverbs"] = []
                enhanced["adverbs"].append(token.text)

        return enhanced

    def _analyze_semantic_intent(self, doc) -> Optional[CommandIntent]:
        """
        Analyze semantic intent when pattern matching fails
        """
        # Look for action verbs
        action_verbs = [token.lemma_ for token in doc if token.pos_ == "VERB"]

        # Look for objects
        objects = [token.text for token in doc if token.pos_ in ["NOUN", "PROPN"]]

        # Look for locations
        locations = [ent.text for ent in doc.ents if ent.label_ in ["GPE", "LOC", "FAC"]]

        # Determine intent based on semantic analysis
        if any(verb in ["go", "move", "navigate", "walk", "run", "head"] for verb in action_verbs):
            location = locations[0] if locations else None
            return CommandIntent(
                intent="move_to",
                parameters={"location": location or "unknown"},
                confidence=0.7,
                raw_text=doc.text
            )

        if any(verb in ["pick", "take", "grasp", "grab", "hold"] for verb in action_verbs):
            obj = objects[0] if objects else None
            return CommandIntent(
                intent="pick_up",
                parameters={"object": obj or "unknown"},
                confidence=0.7,
                raw_text=doc.text
            )

        return None
```

### 4. Voice Command Processing System

```python
# voice_command_system.py
import asyncio
import logging
from typing import Dict, Optional, Callable
import threading
import time
from dataclasses import dataclass

@dataclass
class VoiceCommandResult:
    """Result of voice command processing"""
    success: bool
    message: str
    command: Optional[CommandIntent] = None
    execution_result: Optional[Dict] = None

class VoiceCommandProcessor:
    """
    Main voice command processing system
    """
    def __init__(self, speech_recognizer, nlp_processor):
        self.speech_recognizer = speech_recognizer
        self.nlp_processor = nlp_processor
        self.logger = logging.getLogger(__name__)

        # Command execution callbacks
        self.command_handlers = {}

        # System state
        self.is_active = False
        self.command_queue = asyncio.Queue()
        self.response_callbacks = []

        # Wake word detection (simple implementation)
        self.wake_words = ["robot", "hey robot", "hello robot", "attention"]
        self.wake_word_threshold = 0.8

    def register_command_handler(self, intent: str, handler: Callable):
        """
        Register a handler for a specific command intent
        """
        self.command_handlers[intent] = handler

    async def process_voice_command(self, timeout: int = 10) -> VoiceCommandResult:
        """
        Process a single voice command from start to finish
        """
        try:
            # Listen for command
            self.logger.info("Listening for voice command...")
            text = await asyncio.get_event_loop().run_in_executor(
                None,
                self.speech_recognizer.listen_for_command,
                timeout
            )

            if not text:
                return VoiceCommandResult(
                    success=False,
                    message="No speech detected or recognition failed"
                )

            self.logger.info(f"Recognized: {text}")

            # Process with NLP
            command_intent = self.nlp_processor.process_command(text)
            if not command_intent:
                return VoiceCommandResult(
                    success=False,
                    message=f"Could not understand command: {text}"
                )

            # Execute command if handler exists
            if command_intent.intent in self.command_handlers:
                handler = self.command_handlers[command_intent.intent]
                execution_result = await asyncio.get_event_loop().run_in_executor(
                    None,
                    handler,
                    command_intent
                )

                return VoiceCommandResult(
                    success=True,
                    message=f"Command '{command_intent.intent}' executed successfully",
                    command=command_intent,
                    execution_result=execution_result
                )
            else:
                return VoiceCommandResult(
                    success=False,
                    message=f"No handler for command intent: {command_intent.intent}",
                    command=command_intent
                )

        except Exception as e:
            self.logger.error(f"Error processing voice command: {e}")
            return VoiceCommandResult(
                success=False,
                message=f"Error processing voice command: {str(e)}"
            )

    def add_response_callback(self, callback: Callable):
        """
        Add a callback for voice command responses
        """
        self.response_callbacks.append(callback)

    def speak_response(self, text: str):
        """
        Generate speech response (placeholder - implement with TTS)
        """
        import pyttsx3
        try:
            engine = pyttsx3.init()
            # Configure speech properties
            rate = engine.getProperty('rate')
            engine.setProperty('rate', rate - 50)  # Slower speech

            volume = engine.getProperty('volume')
            engine.setProperty('volume', volume + 0.25)

            engine.say(text)
            engine.runAndWait()
        except Exception as e:
            self.logger.error(f"Text-to-speech error: {e}")

    async def continuous_listening(self):
        """
        Continuously listen for voice commands
        """
        self.is_active = True
        self.logger.info("Starting continuous voice command listening...")

        while self.is_active:
            try:
                result = await self.process_voice_command(timeout=5)

                if result.success:
                    self.logger.info(f"Command executed: {result.message}")

                    # Execute response callbacks
                    for callback in self.response_callbacks:
                        callback(result)

                    # Provide audio feedback
                    self.speak_response("Command executed successfully")
                else:
                    self.logger.warning(f"Command failed: {result.message}")
                    self.speak_response("Sorry, I couldn't understand that command")

                # Small delay to prevent excessive processing
                await asyncio.sleep(0.5)

            except KeyboardInterrupt:
                self.logger.info("Voice command processing interrupted")
                break
            except Exception as e:
                self.logger.error(f"Error in continuous listening: {e}")
                await asyncio.sleep(1)  # Wait before retrying

    def stop_listening(self):
        """
        Stop continuous listening
        """
        self.is_active = False
        self.logger.info("Stopped voice command listening")
```

### 5. ROS 2 Integration

```python
# ros_voice_integration.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import AudioData
import asyncio
import threading

class ROSVoiceInterface(Node):
    """
    ROS 2 interface for voice command processing
    """
    def __init__(self, voice_processor):
        super().__init__('voice_command_interface')
        self.voice_processor = voice_processor

        # Publishers
        self.command_publisher = self.create_publisher(String, 'robot_commands', 10)
        self.speech_publisher = self.create_publisher(String, 'speech_output', 10)

        # Subscribers
        self.voice_command_sub = self.create_subscription(
            String, 'voice_commands', self.voice_command_callback, 10
        )

        # Timer for continuous processing
        self.processing_timer = self.create_timer(0.1, self.process_voice_queue)

        # Voice command queue
        self.voice_command_queue = asyncio.Queue()

        # Start async processing in separate thread
        self.voice_thread = threading.Thread(target=self._run_voice_processing)
        self.voice_thread.daemon = True
        self.voice_thread.start()

    def voice_command_callback(self, msg):
        """
        Handle voice command messages
        """
        asyncio.run_coroutine_threadsafe(
            self.voice_command_queue.put(msg.data),
            self.voice_loop
        )

    async def process_voice_queue(self):
        """
        Process voice commands from the queue
        """
        try:
            # Non-blocking check for voice commands
            command = self.voice_command_queue.get_nowait()
            result = await self.voice_processor.process_voice_command_from_text(command)

            if result.success:
                # Publish robot command
                cmd_msg = String()
                cmd_msg.data = str(result.command)
                self.command_publisher.publish(cmd_msg)

                # Publish success feedback
                feedback_msg = String()
                feedback_msg.data = "Command executed successfully"
                self.speech_publisher.publish(feedback_msg)
        except asyncio.QueueEmpty:
            pass  # No commands to process
        except Exception as e:
            self.get_logger().error(f"Error processing voice command: {e}")

    def _run_voice_processing(self):
        """
        Run the voice processing loop in a separate thread
        """
        self.voice_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.voice_loop)
        self.voice_loop.run_forever()

    def publish_robot_command(self, command: str):
        """
        Publish a command to the robot
        """
        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)

    def publish_speech_output(self, text: str):
        """
        Publish speech output for TTS
        """
        msg = String()
        msg.data = text
        self.speech_publisher.publish(msg)
```

### 6. Example Usage and Integration

```python
# example_voice_robot.py
import asyncio
import logging
from voice_command_system import VoiceCommandProcessor, VoiceCommandResult
from speech_recognition import SpeechRecognizer
from nlp_processor import NLPProcessor

# Example robot command handlers
class RobotCommandHandlers:
    def __init__(self, robot_interface):
        self.robot_interface = robot_interface

    def handle_move_to(self, command_intent):
        """Handle move_to commands"""
        location = command_intent.parameters.get("location", "unknown")
        self.robot_interface.move_to_location(location)
        return {"status": "success", "location": location}

    def handle_pick_up(self, command_intent):
        """Handle pick_up commands"""
        obj = command_intent.parameters.get("object", "unknown")
        self.robot_interface.pick_up_object(obj)
        return {"status": "success", "object": obj}

    def handle_place(self, command_intent):
        """Handle place commands"""
        obj = command_intent.parameters.get("object", "unknown")
        location = command_intent.parameters.get("location", "unknown")
        self.robot_interface.place_object(obj, location)
        return {"status": "success", "object": obj, "location": location}

    def handle_stop(self, command_intent):
        """Handle stop commands"""
        self.robot_interface.stop_movement()
        return {"status": "stopped"}

def main():
    # Setup logging
    logging.basicConfig(level=logging.INFO)

    # Initialize components
    speech_recognizer = SpeechRecognizer(backend="google")  # or "local" for Whisper
    nlp_processor = NLPProcessor()

    # Create voice command processor
    voice_processor = VoiceCommandProcessor(speech_recognizer, nlp_processor)

    # Example robot interface (replace with actual robot interface)
    class MockRobotInterface:
        def move_to_location(self, location):
            print(f"Moving to {location}")

        def pick_up_object(self, obj):
            print(f"Picking up {obj}")

        def place_object(self, obj, location):
            print(f"Placing {obj} at {location}")

        def stop_movement(self):
            print("Stopping movement")

    # Create command handlers
    robot_interface = MockRobotInterface()
    command_handlers = RobotCommandHandlers(robot_interface)

    # Register command handlers
    voice_processor.register_command_handler("move_to", command_handlers.handle_move_to)
    voice_processor.register_command_handler("pick_up", command_handlers.handle_pick_up)
    voice_processor.register_command_handler("place", command_handlers.handle_place)
    voice_processor.register_command_handler("stop", command_handlers.handle_stop)

    # Add response callback
    def command_response_callback(result: VoiceCommandResult):
        print(f"Command result: {result.message}")
        if result.execution_result:
            print(f"Execution result: {result.execution_result}")

    voice_processor.add_response_callback(command_response_callback)

    # Run continuous listening
    print("Starting voice command system...")
    print("Say 'robot' followed by a command, or speak a direct command")
    print("Examples: 'move to kitchen', 'pick up the cup', 'stop'")

    try:
        # Run the continuous listening loop
        asyncio.run(voice_processor.continuous_listening())
    except KeyboardInterrupt:
        print("\nStopping voice command system...")
        voice_processor.stop_listening()

if __name__ == "__main__":
    main()
```

## Installation Requirements

### Python Dependencies
```bash
pip install SpeechRecognition pyaudio webrtcvad spacy pyttsx3 numpy
python -m spacy download en_core_web_sm
```

### For Local Whisper Support (optional)
```bash
pip install openai-whisper
```

### For ROS 2 Integration
```bash
pip install rclpy
```

## Configuration and Tuning

### 1. Audio Configuration
- Adjust `energy_threshold` based on ambient noise levels
- Modify `phrase_time_limit` for longer/shorter commands
- Tune VAD aggressiveness level (0-3) for voice detection sensitivity

### 2. Recognition Accuracy
- Use appropriate speech recognition backend based on requirements:
  - Google: Good accuracy, requires internet, free tier
  - Local Whisper: Privacy, offline capability, resource intensive
  - Wit.ai/IBM: Customizable, requires API keys

### 3. Intent Recognition
- Extend intent patterns for domain-specific commands
- Train spaCy models on robotics-specific language
- Implement confidence thresholds for command execution

## Security and Privacy Considerations

### 1. Data Privacy
- Use local speech recognition for privacy-sensitive applications
- Encrypt audio data in transit if using cloud services
- Implement data retention policies for audio recordings

### 2. Command Validation
- Validate all voice commands before execution
- Implement safety checks and bounds verification
- Use authentication for sensitive commands

### 3. Access Control
- Limit voice command vocabulary to safe operations
- Implement user identification and authorization
- Log all voice commands for audit purposes

## Performance Optimization

### 1. Resource Management
- Use appropriate model sizes for target hardware
- Implement audio buffering and streaming
- Optimize recognition timeout values

### 2. Accuracy Improvements
- Train domain-specific NLP models
- Implement acoustic adaptation for environment
- Use wake word detection to reduce processing load

## Troubleshooting

### Common Issues:
1. **Audio Input Problems**: Check microphone permissions and audio drivers
2. **Recognition Accuracy**: Adjust energy threshold and ambient noise settings
3. **Network Issues**: Implement fallback mechanisms for offline capability
4. **Processing Delays**: Optimize model sizes and implement async processing

### Verification Steps:
1. Test audio input and voice activity detection
2. Verify speech recognition accuracy in your environment
3. Confirm intent recognition works with expected commands
4. Test command execution safety and validation