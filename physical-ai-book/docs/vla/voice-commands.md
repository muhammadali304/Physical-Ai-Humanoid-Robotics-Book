---
sidebar_position: 1
---

# Voice Commands - Natural Language Interaction with Robots

## Learning Objectives

By the end of this chapter, you will be able to:
- Implement speech recognition systems for robot command interpretation
- Design natural language processing pipelines for voice commands
- Integrate voice command systems with robot control architectures
- Create custom voice command vocabularies and grammars
- Handle speech-to-text conversion and command parsing
- Implement voice feedback and confirmation systems

## Prerequisites

Before starting this chapter, you should:
- Have ROS 2 Humble Hawksbill installed and configured
- Understand ROS 2 nodes, topics, and message types
- Completed the Isaac perception and navigation chapters
- Basic knowledge of Python and speech processing concepts
- Understanding of natural language processing fundamentals

## Conceptual Overview

**Voice Command Systems** enable robots to understand and respond to spoken instructions from humans. This creates a more natural and intuitive interaction model compared to traditional button-based or app-based interfaces.

### Key Components of Voice Command Systems

1. **Speech Recognition**: Converting audio to text
2. **Natural Language Understanding (NLU)**: Interpreting meaning from text
3. **Command Mapping**: Connecting understood commands to robot actions
4. **Voice Feedback**: Providing audio confirmation to users
5. **Noise Filtering**: Processing audio in noisy environments

### Voice Command Architecture

The typical voice command architecture follows this flow:
```
Audio Input → Noise Reduction → Speech Recognition → NLU → Command Mapping → Robot Action
```

### Benefits of Voice Commands

- **Natural Interaction**: More intuitive than manual controls
- **Accessibility**: Helps users with mobility limitations
- **Hands-Free Operation**: Allows multitasking
- **Remote Control**: Works at distances where physical controls don't
- **Multilingual Support**: Can support multiple languages

## Hands-On Implementation

### Installing Speech Recognition Dependencies

```bash
# Install system dependencies
sudo apt update
sudo apt install python3-pyaudio python3-speechrecognition python3-pip portaudio19-dev

# Install Python packages
pip3 install SpeechRecognition pyttsx3 vosk transformers torch numpy
pip3 install sounddevice webrtcvad pyaudio
```

### Basic Speech Recognition Node

```python
#!/usr/bin/env python3

"""
Basic speech recognition node for voice commands.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import speech_recognition as sr
import threading
import queue
import time


class VoiceCommandNode(Node):
    """
    Node to recognize voice commands and convert them to robot actions.
    """

    def __init__(self):
        super().__init__('voice_command_node')

        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Calibrate microphone for ambient noise
        with self.microphone as source:
            self.get_logger().info('Calibrating for ambient noise...')
            self.recognizer.adjust_for_ambient_noise(source)
            self.get_logger().info('Calibration complete')

        # Publishers
        self.command_pub = self.create_publisher(String, 'voice_commands', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Command mappings
        self.command_mappings = {
            'move forward': self.move_forward,
            'go forward': self.move_forward,
            'move backward': self.move_backward,
            'go backward': self.move_backward,
            'turn left': self.turn_left,
            'turn right': self.turn_right,
            'stop': self.stop_robot,
            'halt': self.stop_robot,
            'go left': self.move_left,
            'go right': self.move_right,
        }

        # Start listening thread
        self.listen_thread = threading.Thread(target=self.listen_continuously)
        self.listen_thread.daemon = True
        self.listen_thread.start()

        # Queue for audio processing
        self.audio_queue = queue.Queue()

        self.get_logger().info('Voice command node initialized')

    def listen_continuously(self):
        """Continuously listen for voice commands."""
        with self.microphone as source:
            self.get_logger().info('Listening for voice commands...')

        while rclpy.ok():
            try:
                # Listen for audio with timeout
                with self.microphone as source:
                    audio = self.recognizer.listen(source, timeout=1.0, phrase_time_limit=5.0)

                # Add audio to queue for processing
                self.audio_queue.put(audio)

            except sr.WaitTimeoutError:
                # Continue listening if no audio detected
                continue
            except Exception as e:
                self.get_logger().error(f'Error in audio capture: {e}')
                time.sleep(0.1)

    def process_audio(self):
        """Process audio from the queue."""
        while not self.audio_queue.empty():
            try:
                audio = self.audio_queue.get_nowait()

                # Recognize speech using Google's service
                try:
                    text = self.recognizer.recognize_google(audio)
                    self.get_logger().info(f'Recognized: {text}')

                    # Process the recognized text
                    self.process_command(text.lower().strip())

                except sr.UnknownValueError:
                    self.get_logger().info('Could not understand audio')
                except sr.RequestError as e:
                    self.get_logger().error(f'Error with speech recognition service: {e}')

            except queue.Empty:
                break

    def process_command(self, text):
        """Process recognized text and execute appropriate command."""
        # Publish the recognized command
        cmd_msg = String()
        cmd_msg.data = text
        self.command_pub.publish(cmd_msg)

        # Check if command matches any mapping
        for command_phrase, command_func in self.command_mappings.items():
            if command_phrase in text:
                self.get_logger().info(f'Executing command: {command_phrase}')
                command_func()
                return

        # If no command matched
        self.get_logger().info(f'Unrecognized command: {text}')

    def move_forward(self):
        """Move robot forward."""
        twist = Twist()
        twist.linear.x = 0.5  # m/s
        self.cmd_vel_pub.publish(twist)

    def move_backward(self):
        """Move robot backward."""
        twist = Twist()
        twist.linear.x = -0.5  # m/s
        self.cmd_vel_pub.publish(twist)

    def turn_left(self):
        """Turn robot left."""
        twist = Twist()
        twist.angular.z = 0.5  # rad/s
        self.cmd_vel_pub.publish(twist)

    def turn_right(self):
        """Turn robot right."""
        twist = Twist()
        twist.angular.z = -0.5  # rad/s
        self.cmd_vel_pub.publish(twist)

    def move_left(self):
        """Strafe robot left."""
        twist = Twist()
        twist.linear.y = 0.5  # m/s
        self.cmd_vel_pub.publish(twist)

    def move_right(self):
        """Strafe robot right."""
        twist = Twist()
        twist.linear.y = -0.5  # m/s
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        """Stop robot movement."""
        twist = Twist()
        # All velocities remain 0
        self.cmd_vel_pub.publish(twist)

    def spin_once(self):
        """Process any queued audio."""
        self.process_audio()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()

    try:
        while rclpy.ok():
            node.spin_once()
            time.sleep(0.01)  # Small delay to prevent busy waiting
    except KeyboardInterrupt:
        node.get_logger().info('Voice command node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Advanced Voice Command with Natural Language Processing

```python
#!/usr/bin/env python3

"""
Advanced voice command node with natural language understanding.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import speech_recognition as sr
import threading
import queue
import time
import re
from enum import Enum


class CommandType(Enum):
    MOVE = "move"
    TURN = "turn"
    STOP = "stop"
    NAVIGATE = "navigate"
    QUERY = "query"


class Command:
    """Represents a parsed voice command."""
    def __init__(self, cmd_type, direction=None, distance=None, speed=None, destination=None):
        self.type = cmd_type
        self.direction = direction
        self.distance = distance
        self.speed = speed
        self.destination = destination


class AdvancedVoiceCommandNode(Node):
    """
    Advanced voice command node with natural language understanding.
    """

    def __init__(self):
        super().__init__('advanced_voice_command_node')

        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Calibrate microphone
        with self.microphone as source:
            self.get_logger().info('Calibrating for ambient noise...')
            self.recognizer.adjust_for_ambient_noise(source)
            self.get_logger().info('Calibration complete')

        # Publishers
        self.command_pub = self.create_publisher(String, 'parsed_commands', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Initialize variables
        self.laser_data = None
        self.command_history = []

        # Start listening thread
        self.listen_thread = threading.Thread(target=self.listen_continuously)
        self.listen_thread.daemon = True
        self.listen_thread.start()

        # Queue for audio processing
        self.audio_queue = queue.Queue()

        self.get_logger().info('Advanced voice command node initialized')

    def scan_callback(self, msg):
        """Store laser scan data for obstacle detection."""
        self.laser_data = msg

    def listen_continuously(self):
        """Continuously listen for voice commands."""
        with self.microphone as source:
            self.get_logger().info('Listening for voice commands...')

        while rclpy.ok():
            try:
                with self.microphone as source:
                    audio = self.recognizer.listen(source, timeout=1.0, phrase_time_limit=5.0)
                self.audio_queue.put(audio)
            except sr.WaitTimeoutError:
                continue
            except Exception as e:
                self.get_logger().error(f'Error in audio capture: {e}')
                time.sleep(0.1)

    def process_audio(self):
        """Process audio from the queue."""
        while not self.audio_queue.empty():
            try:
                audio = self.audio_queue.get_nowait()

                try:
                    text = self.recognizer.recognize_google(audio)
                    self.get_logger().info(f'Recognized: {text}')

                    # Parse and execute command
                    self.parse_and_execute_command(text.lower().strip())

                except sr.UnknownValueError:
                    self.get_logger().info('Could not understand audio')
                except sr.RequestError as e:
                    self.get_logger().error(f'Error with speech recognition service: {e}')

            except queue.Empty:
                break

    def parse_and_execute_command(self, text):
        """Parse text and execute appropriate command."""
        # Publish original recognized text
        original_msg = String()
        original_msg.data = text
        self.command_pub.publish(original_msg)

        # Parse the command
        command = self.parse_command(text)

        if command:
            self.get_logger().info(f'Parsed command: {command.type.value}')

            # Execute based on command type
            if command.type == CommandType.MOVE:
                self.execute_move_command(command)
            elif command.type == CommandType.TURN:
                self.execute_turn_command(command)
            elif command.type == CommandType.STOP:
                self.execute_stop_command(command)
            elif command.type == CommandType.NAVIGATE:
                self.execute_navigate_command(command)
            elif command.type == CommandType.QUERY:
                self.execute_query_command(command)

            # Store in history
            self.command_history.append(command)
        else:
            self.get_logger().info(f'Could not parse command: {text}')

    def parse_command(self, text):
        """Parse text to extract command components."""
        # Define patterns for different command types

        # Move commands: "move forward 2 meters", "go left slowly"
        move_patterns = [
            r'(?:move|go)\s+(forward|backward|ahead|back|left|right|up|down)',
            r'(?:move|go)\s+(?:the\s+)?robot\s+(forward|backward|ahead|back|left|right)'
        ]

        # Turn commands: "turn left", "rotate right"
        turn_patterns = [
            r'(?:turn|rotate)\s+(left|right)',
            r'(?:spin|pivot)\s+(left|right)'
        ]

        # Stop commands: "stop", "halt", "freeze"
        stop_patterns = [
            r'(?:stop|halt|freeze|pause|stand)',
            r'(?:come\s+to\s+a\s+stop|stop\s+moving)'
        ]

        # Navigate commands: "go to kitchen", "navigate to charging station"
        navigate_patterns = [
            r'(?:go\s+to|navigate\s+to|move\s+to)\s+(.+?)(?:\s+please|\.|$)',
            r'(?:take\s+me\s+to|bring\s+me\s+to)\s+(.+?)(?:\s+please|\.|$)'
        ]

        # Query commands: "where are you", "what is your status"
        query_patterns = [
            r'(?:where\s+are\s+you|where\s+is\s+the\s+robot|location|position)',
            r'(?:status|what\s+are\s+you\s+doing|what\s+is\s+your\s+status)',
            r'(?:battery|power|charge|energy)'
        ]

        # Extract distance and speed modifiers
        distance_match = re.search(r'(\d+(?:\.\d+)?)\s*(?:meter|m|foot|ft)', text)
        speed_match = re.search(r'(slowly|slow|fast|quickly|quick)', text)

        # Try to match patterns
        for pattern in move_patterns:
            match = re.search(pattern, text)
            if match:
                direction = match.group(1)
                distance = float(distance_match.group(1)) if distance_match else None
                speed = speed_match.group(1) if speed_match else None
                return Command(CommandType.MOVE, direction=direction, distance=distance, speed=speed)

        for pattern in turn_patterns:
            match = re.search(pattern, text)
            if match:
                direction = match.group(1)
                speed = speed_match.group(1) if speed_match else None
                return Command(CommandType.TURN, direction=direction, speed=speed)

        for pattern in stop_patterns:
            if re.search(pattern, text):
                return Command(CommandType.STOP)

        for pattern in navigate_patterns:
            match = re.search(pattern, text)
            if match:
                destination = match.group(1).strip()
                return Command(CommandType.NAVIGATE, destination=destination)

        for pattern in query_patterns:
            if re.search(pattern, text):
                return Command(CommandType.QUERY)

        return None  # No command matched

    def execute_move_command(self, command):
        """Execute a move command."""
        twist = Twist()

        if command.direction in ['forward', 'ahead']:
            twist.linear.x = 0.5 if command.speed != 'slow' else 0.2
        elif command.direction in ['backward', 'back']:
            twist.linear.x = -0.5 if command.speed != 'slow' else -0.2
        elif command.direction == 'left':
            twist.linear.y = 0.3 if command.speed != 'slow' else 0.15
        elif command.direction == 'right':
            twist.linear.y = -0.3 if command.speed != 'slow' else -0.15
        elif command.direction == 'up':
            twist.linear.z = 0.2
        elif command.direction == 'down':
            twist.linear.z = -0.2

        self.cmd_vel_pub.publish(twist)

    def execute_turn_command(self, command):
        """Execute a turn command."""
        twist = Twist()

        if command.direction == 'left':
            twist.angular.z = 0.5 if command.speed != 'slow' else 0.2
        elif command.direction == 'right':
            twist.angular.z = -0.5 if command.speed != 'slow' else -0.2

        self.cmd_vel_pub.publish(twist)

    def execute_stop_command(self, command):
        """Execute a stop command."""
        twist = Twist()
        # All velocities remain 0
        self.cmd_vel_pub.publish(twist)

    def execute_navigate_command(self, command):
        """Execute a navigate command."""
        self.get_logger().info(f'Navigating to: {command.destination}')
        # In a real implementation, this would interface with navigation stack
        # For now, just log the command
        pass

    def execute_query_command(self, command):
        """Execute a query command."""
        # In a real implementation, this would respond with robot status
        self.get_logger().info(f'Query command received: {command.destination}')
        pass

    def spin_once(self):
        """Process any queued audio."""
        self.process_audio()


def main(args=None):
    rclpy.init(args=args)
    node = AdvancedVoiceCommandNode()

    try:
        while rclpy.ok():
            node.spin_once()
            time.sleep(0.01)
    except KeyboardInterrupt:
        node.get_logger().info('Advanced voice command node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Offline Speech Recognition with Vosk

For privacy-sensitive applications, we can use offline speech recognition:

```python
#!/usr/bin/env python3

"""
Offline voice command node using Vosk for speech recognition.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from vosk import Model, KaldiRecognizer
import pyaudio
import json
import threading
import queue


class OfflineVoiceCommandNode(Node):
    """
    Voice command node using offline Vosk speech recognition.
    """

    def __init__(self):
        super().__init__('offline_voice_command_node')

        # Initialize Vosk model (download from https://alphacephei.com/vosk/models)
        try:
            self.model = Model(lang="en-us")  # Change language as needed
        except Exception as e:
            self.get_logger().error(f'Failed to load Vosk model: {e}')
            self.get_logger().error('Download model from https://alphacephei.com/vosk/models')
            raise

        # Initialize audio
        self.sample_rate = 16000
        self.rec = KaldiRecognizer(self.model, self.sample_rate)

        # Initialize PyAudio
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=8000
        )

        # Publishers
        self.command_pub = self.create_publisher(String, 'offline_commands', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Command mappings
        self.command_mappings = {
            'move forward': self.move_forward,
            'go forward': self.move_forward,
            'move backward': self.move_backward,
            'go backward': self.move_backward,
            'turn left': self.turn_left,
            'turn right': self.turn_right,
            'stop': self.stop_robot,
            'halt': self.stop_robot,
        }

        # Audio processing thread
        self.audio_thread = threading.Thread(target=self.process_audio_stream)
        self.audio_thread.daemon = True
        self.audio_thread.start()

        self.get_logger().info('Offline voice command node initialized')

    def process_audio_stream(self):
        """Process audio stream continuously."""
        while rclpy.ok():
            try:
                data = self.stream.read(4000, exception_on_overflow=False)

                if len(data) == 0:
                    continue

                if self.rec.AcceptWaveform(data):
                    result = self.rec.Result()
                    result_dict = json.loads(result)

                    if 'text' in result_dict and result_dict['text']:
                        text = result_dict['text'].lower().strip()
                        if text:  # Only process non-empty text
                            self.get_logger().info(f'Recognized (offline): {text}')
                            self.process_command(text)

            except Exception as e:
                self.get_logger().error(f'Error in audio processing: {e}')

    def process_command(self, text):
        """Process recognized text and execute appropriate command."""
        # Publish the recognized command
        cmd_msg = String()
        cmd_msg.data = text
        self.command_pub.publish(cmd_msg)

        # Check if command matches any mapping
        for command_phrase, command_func in self.command_mappings.items():
            if command_phrase in text:
                self.get_logger().info(f'Executing command: {command_phrase}')
                command_func()
                return

        # If no command matched
        self.get_logger().info(f'Unrecognized command: {text}')

    def move_forward(self):
        """Move robot forward."""
        twist = Twist()
        twist.linear.x = 0.5
        self.cmd_vel_pub.publish(twist)

    def move_backward(self):
        """Move robot backward."""
        twist = Twist()
        twist.linear.x = -0.5
        self.cmd_vel_pub.publish(twist)

    def turn_left(self):
        """Turn robot left."""
        twist = Twist()
        twist.angular.z = 0.5
        self.cmd_vel_pub.publish(twist)

    def turn_right(self):
        """Turn robot right."""
        twist = Twist()
        twist.angular.z = -0.5
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        """Stop robot movement."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def destroy_node(self):
        """Clean up resources."""
        if hasattr(self, 'stream'):
            self.stream.stop_stream()
            self.stream.close()
        if hasattr(self, 'p'):
            self.p.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OfflineVoiceCommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Offline voice command node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Text-to-Speech Feedback Node

```python
#!/usr/bin/env python3

"""
Text-to-speech feedback node for voice command responses.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3
import threading


class VoiceFeedbackNode(Node):
    """
    Node to provide voice feedback for robot actions.
    """

    def __init__(self):
        super().__init__('voice_feedback_node')

        # Initialize text-to-speech engine
        self.tts_engine = pyttsx3.init()

        # Configure voice properties
        voices = self.tts_engine.getProperty('voices')
        if voices:
            self.tts_engine.setProperty('voice', voices[0].id)  # Use first available voice
        self.tts_engine.setProperty('rate', 150)  # Speed of speech
        self.tts_engine.setProperty('volume', 0.9)  # Volume level

        # Create subscriber for feedback requests
        self.feedback_sub = self.create_subscription(
            String,
            'voice_feedback',
            self.feedback_callback,
            10
        )

        # Thread lock for TTS
        self.tts_lock = threading.Lock()

        self.get_logger().info('Voice feedback node initialized')

    def feedback_callback(self, msg):
        """Handle feedback requests."""
        text = msg.data
        self.get_logger().info(f'Playing voice feedback: {text}')

        # Speak the text in a separate thread to avoid blocking
        speak_thread = threading.Thread(target=self.speak_text, args=(text,))
        speak_thread.daemon = True
        speak_thread.start()

    def speak_text(self, text):
        """Speak text using TTS engine."""
        with self.tts_lock:
            try:
                self.tts_engine.say(text)
                self.tts_engine.runAndWait()
            except Exception as e:
                self.get_logger().error(f'Error in text-to-speech: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VoiceFeedbackNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Voice feedback node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Creating a Voice Command Package

**Create the package:**

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python voice_commands
```

**Update package.xml:**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>voice_commands</name>
  <version>0.0.1</version>
  <description>Voice command processing for robotics</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Create setup.py:**

```python
from setuptools import find_packages, setup

package_name = 'voice_commands'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Voice command processing for robotics',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_command_node = voice_commands.voice_command_node:main',
            'advanced_voice_node = voice_commands.advanced_voice_node:main',
            'offline_voice_node = voice_commands.offline_voice_node:main',
            'voice_feedback_node = voice_commands.voice_feedback_node:main',
        ],
    },
)
```

### Launch File for Voice Commands

**Create launch/voice_commands_launch.py:**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    # Voice command node
    voice_command_node = Node(
        package='voice_commands',
        executable='voice_command_node',
        name='voice_command_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # Voice feedback node
    voice_feedback_node = Node(
        package='voice_commands',
        executable='voice_feedback_node',
        name='voice_feedback_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time,
        voice_command_node,
        voice_feedback_node
    ])
```

## Testing & Verification

### Running Voice Command System

1. **Install dependencies:**
```bash
pip3 install SpeechRecognition pyttsx3 vosk transformers torch numpy
sudo apt install python3-pyaudio portaudio19-dev
```

2. **Build the package:**
```bash
cd ~/ros2_ws
colcon build --packages-select voice_commands
source install/setup.bash
```

3. **Run the voice command system:**
```bash
# Terminal 1: Run the voice command node
ros2 run voice_commands voice_command_node

# Terminal 2: Test with manual commands
ros2 topic pub /voice_feedback std_msgs/msg/String "data: 'Hello, I am ready to receive commands'"
```

4. **Test with different voice commands:**
```bash
# Speak commands like:
# "Move forward"
# "Turn left"
# "Stop"
# "Go backward"
```

### Useful Voice Command Commands

- **Monitor voice commands:**
```bash
ros2 topic echo /voice_commands
```

- **Send feedback:**
```bash
ros2 topic pub /voice_feedback std_msgs/msg/String "data: 'Command received'"
```

- **Check audio devices:**
```bash
# Python script to list audio devices
python3 -c "import pyaudio; p = pyaudio.PyAudio(); print('Devices:', p.get_device_count()); [print(i, p.get_device_info_by_index(i)['name']) for i in range(p.get_device_count())]"
```

### Performance Testing

```bash
# Test recognition accuracy in different environments
# Test response time to commands
# Test with different speakers and accents
```

## Common Issues

### Issue: Microphone not detected or no audio input
**Solution**:
- Check microphone permissions and connections
- Verify audio device using `arecord -l`
- Test microphone with `arecord -D hw:0,0 -f cd test.wav`
- Ensure proper ALSA/PulseAudio configuration

### Issue: High CPU usage with speech recognition
**Solution**:
- Use offline models like Vosk for continuous recognition
- Implement wake word detection to activate recognition only when needed
- Reduce sampling rate if quality permits
- Use threading to prevent blocking

### Issue: Poor recognition accuracy
**Solution**:
- Calibrate microphone for ambient noise
- Use directional microphones in noisy environments
- Train custom acoustic models for specific environments
- Implement confidence thresholds to filter uncertain recognitions

### Issue: Delay in voice feedback
**Solution**:
- Use streaming TTS engines
- Implement interruptible speech for urgent feedback
- Optimize audio processing pipeline
- Use lightweight synthesis models

## Key Takeaways

- Voice commands provide natural, intuitive robot interaction
- Offline recognition offers privacy and reliability
- Natural language processing enables complex command understanding
- Audio feedback improves user experience and confirms actions
- Wake word detection reduces power consumption
- Noise filtering is crucial for real-world deployment
- Integration with navigation and control systems enables complex tasks

## Next Steps

In the next chapter, you'll learn about Large Language Model (LLM) integration for advanced planning and reasoning in robotics applications.