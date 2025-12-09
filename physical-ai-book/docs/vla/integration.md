---
sidebar_position: 3
---

# VLA Integration - Vision-Language-Action Systems

## Learning Objectives

By the end of this chapter, you will be able to:
- Integrate vision, language, and action systems into a cohesive robot architecture
- Implement multimodal perception for enhanced robot awareness
- Create feedback loops between vision, language, and action components
- Design coordination mechanisms for VLA system components
- Implement error handling and recovery in VLA systems
- Validate and test integrated VLA system performance

## Prerequisites

Before starting this chapter, you should:
- Have ROS 2 Humble Hawksbill installed and configured
- Completed the Isaac perception and navigation chapters
- Completed the voice commands and LLM planning chapters
- Understanding of sensor fusion and multimodal systems
- Experience with ROS 2 action servers and clients
- Knowledge of system integration patterns

## Conceptual Overview

**Vision-Language-Action (VLA)** systems represent the integration of three key modalities in robotics:
- **Vision**: Sensory perception and environmental understanding
- **Language**: Natural communication and high-level reasoning
- **Action**: Physical interaction and task execution

### VLA System Architecture

A complete VLA system follows this architecture:

```
[VISION] ←→ [LANGUAGE] ←→ [ACTION]
    ↑           ↑           ↑
Sensors    Natural      Actuators
          Language
         Processing
```

With feedback loops for:
- Perception → Action → Correction
- Language → Plan → Execution → Update
- Multi-modal fusion for enhanced decision making

### Key Integration Challenges

1. **Temporal Coordination**: Aligning processing across different time scales
2. **Spatial Coordination**: Maintaining consistent coordinate frames
3. **Semantic Coordination**: Connecting visual concepts with linguistic concepts
4. **Behavioral Coordination**: Sequencing actions based on perception and language

### Benefits of VLA Integration

- **Enhanced Situational Awareness**: Combined perception and reasoning
- **Natural Human-Robot Interaction**: Intuitive communication and control
- **Adaptive Behavior**: Respond to changing environments and goals
- **Robust Performance**: Multiple modalities provide redundancy
- **Cognitive Capabilities**: Higher-level reasoning and planning

## Hands-On Implementation

### Creating an Integrated VLA Node

```python
#!/usr/bin/env python3

"""
Integrated Vision-Language-Action node for multimodal robotics.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from vision_msgs.msg import Detection2DArray
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from typing import Dict, List, Optional, Any
import json
import threading
import time
from dataclasses import dataclass


@dataclass
class VLAState:
    """Current state of the VLA system."""
    vision_data: Optional[Any] = None
    language_input: Optional[str] = None
    action_plan: Optional[List[Dict]] = None
    current_action: Optional[Dict] = None
    execution_status: str = "idle"
    timestamp: float = 0.0


class VLAIntegrationNode(Node):
    """
    Node to integrate Vision, Language, and Action systems.
    """

    def __init__(self):
        super().__init__('vla_integration_node')

        # Initialize VLA state
        self.state = VLAState()
        self.state_lock = threading.Lock()

        # Create subscribers for all modalities
        # Vision inputs
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Language inputs
        self.voice_command_sub = self.create_subscription(
            String,
            '/voice_commands',
            self.voice_command_callback,
            10
        )

        self.text_command_sub = self.create_subscription(
            String,
            '/text_commands',
            self.text_command_callback,
            10
        )

        # Action inputs
        self.action_feedback_sub = self.create_subscription(
            String,
            '/action_feedback',
            self.action_feedback_callback,
            10
        )

        # Create publishers
        self.command_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.navigation_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.vla_status_pub = self.create_publisher(String, '/vla_status', 10)
        self.system_response_pub = self.create_publisher(String, '/system_response', 10)

        # Timer for coordination loop
        self.coordination_timer = self.create_timer(0.1, self.coordination_loop)

        # Initialize perception and planning components
        self.perception_processor = PerceptionProcessor(self)
        self.language_processor = LanguageProcessor(self)
        self.action_executor = ActionExecutor(self)

        self.get_logger().info('VLA integration node initialized')

    def image_callback(self, msg):
        """Handle image input."""
        with self.state_lock:
            self.state.vision_data = {
                'image': msg,
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            }
            self.state.timestamp = time.time()

    def detection_callback(self, msg):
        """Handle object detection input."""
        with self.state_lock:
            self.state.vision_data = {
                'detections': msg,
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            }

    def scan_callback(self, msg):
        """Handle laser scan input."""
        with self.state_lock:
            self.state.vision_data = {
                'scan': msg,
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            }

    def odom_callback(self, msg):
        """Handle odometry input."""
        with self.state_lock:
            self.state.vision_data = {
                'odometry': msg,
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            }

    def voice_command_callback(self, msg):
        """Handle voice command input."""
        with self.state_lock:
            self.state.language_input = msg.data
            self.state.timestamp = time.time()

    def text_command_callback(self, msg):
        """Handle text command input."""
        with self.state_lock:
            self.state.language_input = msg.data
            self.state.timestamp = time.time()

    def action_feedback_callback(self, msg):
        """Handle action execution feedback."""
        with self.state_lock:
            try:
                feedback = json.loads(msg.data)
                self.state.execution_status = feedback.get('status', 'unknown')
                self.state.current_action = feedback.get('current_action', None)
            except json.JSONDecodeError:
                self.state.execution_status = 'error'
                self.get_logger().error('Error parsing action feedback')

    def coordination_loop(self):
        """Main coordination loop for VLA integration."""
        with self.state_lock:
            current_state = self.state

        # Process based on available inputs
        if current_state.language_input:
            # Process language input to generate plan
            plan = self.language_processor.process_command(current_state.language_input)

            if plan:
                # Update state with new plan
                with self.state_lock:
                    self.state.action_plan = plan
                    self.state.language_input = None  # Clear processed input

                # Execute the plan
                self.execute_plan(plan)

        # Process vision data for situational awareness
        if current_state.vision_data:
            # Update perception based on vision data
            self.perception_processor.update_perception(current_state.vision_data)

        # Publish system status
        self.publish_vla_status()

    def execute_plan(self, plan: List[Dict]):
        """Execute a plan with multimodal feedback."""
        for step in plan:
            # Check for interruptions during execution
            with self.state_lock:
                if self.state.language_input:  # New command arrived
                    self.get_logger().info('Plan interrupted by new command')
                    return

            # Execute the action step
            success = self.action_executor.execute_action(step)

            if not success:
                self.get_logger().error(f'Action failed: {step}')
                break

            # Wait for action completion
            time.sleep(0.5)

    def publish_vla_status(self):
        """Publish current VLA system status."""
        status_msg = String()
        with self.state_lock:
            status_data = {
                'execution_status': self.state.execution_status,
                'vision_available': self.state.vision_data is not None,
                'language_input_pending': self.state.language_input is not None,
                'plan_steps_remaining': len(self.state.action_plan) if self.state.action_plan else 0,
                'timestamp': time.time()
            }
            status_msg.data = json.dumps(status_data)

        self.vla_status_pub.publish(status_msg)


class PerceptionProcessor:
    """Handles vision processing and multimodal perception."""

    def __init__(self, parent_node):
        self.parent_node = parent_node
        self.perception_model = None  # Would be a real perception model in practice

    def update_perception(self, vision_data: Dict):
        """Update perception based on vision data."""
        # Process different types of vision data
        if 'image' in vision_data:
            self.process_image(vision_data['image'])
        elif 'detections' in vision_data:
            self.process_detections(vision_data['detections'])
        elif 'scan' in vision_data:
            self.process_scan(vision_data['scan'])
        elif 'odometry' in vision_data:
            self.process_odometry(vision_data['odometry'])

    def process_image(self, image_msg):
        """Process camera image."""
        self.parent_node.get_logger().debug('Processing camera image')

    def process_detections(self, detection_array):
        """Process object detections."""
        for detection in detection_array.detections:
            self.parent_node.get_logger().debug(f'Detected: {detection.results}')

    def process_scan(self, scan_msg):
        """Process laser scan for obstacle detection."""
        min_distance = min(scan_msg.ranges) if scan_msg.ranges else float('inf')
        if min_distance < 0.5:
            self.parent_node.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m')

    def process_odometry(self, odom_msg):
        """Process odometry for localization."""
        pos = odom_msg.pose.pose.position
        self.parent_node.get_logger().debug(f'Position: ({pos.x:.2f}, {pos.y:.2f})')


class LanguageProcessor:
    """Handles language processing and command interpretation."""

    def __init__(self, parent_node):
        self.parent_node = parent_node
        self.command_interpreter = None  # Would be a real NLP model in practice

    def process_command(self, command: str) -> Optional[List[Dict]]:
        """Process natural language command and return action plan."""
        self.parent_node.get_logger().info(f'Processing command: {command}')

        # Simple rule-based command interpreter for demonstration
        # In practice, this would use LLMs or other NLP models
        return self.interpret_command(command)

    def interpret_command(self, command: str) -> Optional[List[Dict]]:
        """Interpret command and generate action plan."""
        command_lower = command.lower()

        # Example interpretations
        if 'kitchen' in command_lower:
            return [
                {
                    'action': 'navigate_to',
                    'parameters': {'location': 'kitchen'},
                    'description': 'Navigate to kitchen'
                }
            ]
        elif 'bedroom' in command_lower:
            return [
                {
                    'action': 'navigate_to',
                    'parameters': {'location': 'bedroom'},
                    'description': 'Navigate to bedroom'
                }
            ]
        elif 'find' in command_lower or 'look for' in command_lower:
            return [
                {
                    'action': 'localize',
                    'parameters': {},
                    'description': 'Localize to current position'
                },
                {
                    'action': 'perceive',
                    'parameters': {},
                    'description': 'Perceive surroundings'
                }
            ]
        else:
            # Default action for unrecognized commands
            return [
                {
                    'action': 'report_status',
                    'parameters': {},
                    'description': f'Received command: {command}'
                }
            ]


class ActionExecutor:
    """Handles action execution and coordination."""

    def __init__(self, parent_node):
        self.parent_node = parent_node
        self.command_publisher = parent_node.command_pub
        self.navigation_publisher = parent_node.navigation_goal_pub

    def execute_action(self, action: Dict) -> bool:
        """Execute a single action."""
        action_type = action.get('action', '')
        parameters = action.get('parameters', {})

        self.parent_node.get_logger().info(f'Executing action: {action_type}')

        try:
            if action_type == 'navigate_to':
                return self.execute_navigate_to(parameters)
            elif action_type == 'move_forward':
                return self.execute_move_forward(parameters)
            elif action_type == 'turn':
                return self.execute_turn(parameters)
            elif action_type == 'report_status':
                return self.execute_report_status(parameters)
            elif action_type == 'perceive':
                return self.execute_perceive(parameters)
            else:
                self.parent_node.get_logger().warn(f'Unknown action type: {action_type}')
                return False

        except Exception as e:
            self.parent_node.get_logger().error(f'Error executing action {action_type}: {e}')
            return False

    def execute_navigate_to(self, params: Dict) -> bool:
        """Execute navigation action."""
        location = params.get('location', 'unknown')

        # In a real system, this would send navigation goals
        # For this example, we'll simulate the action
        self.parent_node.get_logger().info(f'Navigating to {location}')
        time.sleep(2)  # Simulate navigation time

        return True

    def execute_move_forward(self, params: Dict) -> bool:
        """Execute forward movement."""
        distance = float(params.get('distance', 1.0))
        speed = float(params.get('speed', 0.5))

        # Create and publish velocity command
        twist = Twist()
        twist.linear.x = speed
        self.command_publisher.publish(twist)

        # Simulate movement
        time.sleep(distance / speed)

        # Stop the robot
        stop_twist = Twist()
        self.command_publisher.publish(stop_twist)

        return True

    def execute_turn(self, params: Dict) -> bool:
        """Execute turning action."""
        angle = float(params.get('angle', 90.0))
        direction = params.get('direction', 'left')

        # Create and publish turning command
        twist = Twist()
        twist.angular.z = 0.5 if direction == 'left' else -0.5
        self.command_publisher.publish(twist)

        # Simulate turn
        time.sleep(abs(angle) / 90.0 * 2)  # Rough timing

        # Stop turning
        stop_twist = Twist()
        self.command_publisher.publish(stop_twist)

        return True

    def execute_report_status(self, params: Dict) -> bool:
        """Execute status reporting."""
        self.parent_node.get_logger().info('Reporting system status')
        return True

    def execute_perceive(self, params: Dict) -> bool:
        """Execute perception action."""
        self.parent_node.get_logger().info('Performing environmental perception')
        time.sleep(1)  # Simulate perception time
        return True


def main(args=None):
    rclpy.init(args=args)
    node = VLAIntegrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('VLA integration node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Creating a Multimodal Fusion Node

```python
#!/usr/bin/env python3

"""
Multimodal fusion node for combining vision, language, and action data.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
from typing import Dict, List, Optional, Any
import json
import numpy as np
from collections import deque


class MultimodalFusionNode(Node):
    """
    Node to fuse information from vision, language, and action modalities.
    """

    def __init__(self):
        super().__init__('multimodal_fusion_node')

        # Create subscribers for all modalities
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.language_sub = self.create_subscription(
            String,
            '/parsed_commands',
            self.language_callback,
            10
        )

        self.action_sub = self.create_subscription(
            String,
            '/execution_status',
            self.action_callback,
            10
        )

        # Create publishers
        self.fused_state_pub = self.create_publisher(String, '/fused_state', 10)
        self.decision_pub = self.create_publisher(String, '/fused_decision', 10)

        # Initialize fusion components
        self.vision_buffer = deque(maxlen=10)
        self.language_buffer = deque(maxlen=5)
        self.action_buffer = deque(maxlen=10)

        # Initialize fusion model (simplified for this example)
        self.fusion_model = MultimodalFusionModel()

        # Timer for fusion processing
        self.fusion_timer = self.create_timer(0.2, self.fusion_loop)

        self.get_logger().info('Multimodal fusion node initialized')

    def image_callback(self, msg):
        """Handle image input."""
        # Convert image to features (simplified)
        features = self.extract_image_features(msg)
        self.vision_buffer.append({
            'type': 'image',
            'features': features,
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        })

    def detection_callback(self, msg):
        """Handle detection input."""
        detections = []
        for detection in msg.detections:
            for result in detection.results:
                detections.append({
                    'class': result.hypothesis.class_id,
                    'confidence': result.hypothesis.score,
                    'bbox': [detection.bbox.center.x, detection.bbox.center.y,
                             detection.bbox.size_x, detection.bbox.size_y]
                })

        self.vision_buffer.append({
            'type': 'detection',
            'data': detections,
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        })

    def scan_callback(self, msg):
        """Handle laser scan input."""
        # Process laser scan for obstacle information
        obstacle_distances = [r for r in msg.ranges if 0.1 < r < 10.0]  # Filter valid ranges
        min_distance = min(obstacle_distances) if obstacle_distances else float('inf')

        self.vision_buffer.append({
            'type': 'scan',
            'min_distance': min_distance,
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        })

    def language_callback(self, msg):
        """Handle language input."""
        self.language_buffer.append({
            'text': msg.data,
            'timestamp': self.get_clock().now().nanoseconds * 1e-9
        })

    def action_callback(self, msg):
        """Handle action input."""
        try:
            action_data = json.loads(msg.data)
            self.action_buffer.append({
                'data': action_data,
                'timestamp': self.get_clock().now().nanoseconds * 1e-9
            })
        except json.JSONDecodeError:
            self.get_logger().error(f'Error parsing action data: {msg.data}')

    def extract_image_features(self, image_msg):
        """Extract simple features from image (simplified for example)."""
        # In practice, this would use a CNN or other vision model
        # For this example, we'll return dummy features
        return {
            'mean_intensity': 128,  # Placeholder
            'edges': 100,           # Placeholder
            'motion': False         # Placeholder
        }

    def fusion_loop(self):
        """Main fusion loop."""
        # Get current data from all modalities
        vision_data = list(self.vision_buffer)
        language_data = list(self.language_buffer)
        action_data = list(self.action_buffer)

        # Fuse the modalities
        fused_state = self.fusion_model.fuse_modalities(
            vision_data, language_data, action_data
        )

        # Make decision based on fused state
        decision = self.make_decision(fused_state)

        # Publish results
        if fused_state:
            fused_msg = String()
            fused_msg.data = json.dumps(fused_state)
            self.fused_state_pub.publish(fused_msg)

        if decision:
            decision_msg = String()
            decision_msg.data = json.dumps(decision)
            self.decision_pub.publish(decision_msg)

    def make_decision(self, fused_state: Dict) -> Optional[Dict]:
        """Make decision based on fused state."""
        # Example decision logic
        if not fused_state:
            return None

        # Check for obstacles
        if 'min_distance' in fused_state and fused_state['min_distance'] < 0.5:
            return {
                'action': 'avoid_obstacle',
                'reason': 'Obstacle detected in path',
                'confidence': 0.9
            }

        # Check for language commands
        if 'language_intent' in fused_state:
            return {
                'action': 'execute_command',
                'command': fused_state['language_intent'],
                'confidence': 0.8
            }

        # Default: continue current behavior
        return {
            'action': 'continue',
            'confidence': 0.7
        }


class MultimodalFusionModel:
    """Simplified multimodal fusion model."""

    def __init__(self):
        # In practice, this would be a neural network or other fusion model
        pass

    def fuse_modalities(self, vision_data: List, language_data: List, action_data: List) -> Dict:
        """Fuse information from all modalities."""
        fused_state = {}

        # Process vision data
        if vision_data:
            latest_vision = vision_data[-1]  # Most recent
            if latest_vision['type'] == 'scan':
                fused_state['min_distance'] = latest_vision['min_distance']
            elif latest_vision['type'] == 'detection':
                fused_state['detections'] = latest_vision['data']

        # Process language data
        if language_data:
            latest_language = language_data[-1]  # Most recent
            fused_state['language_input'] = latest_language['text']
            # Simple intent extraction
            text_lower = latest_language['text'].lower()
            if 'go to' in text_lower or 'navigate' in text_lower:
                fused_state['language_intent'] = 'navigation'
            elif 'find' in text_lower or 'look' in text_lower:
                fused_state['language_intent'] = 'perception'
            else:
                fused_state['language_intent'] = 'other'

        # Process action data
        if action_data:
            latest_action = action_data[-1]  # Most recent
            fused_state['action_status'] = latest_action['data']

        # Add timestamp
        fused_state['timestamp'] = time.time()

        return fused_state


def main(args=None):
    rclpy.init(args=args)
    node = MultimodalFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Multimodal fusion node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Creating a VLA Coordinator Node

```python
#!/usr/bin/env python3

"""
VLA coordinator node for managing the overall system flow.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from typing import Dict, List, Optional, Any
import json
import time
from enum import Enum


class VLAState(Enum):
    IDLE = "idle"
    PERCEIVING = "perceiving"
    PLANNING = "planning"
    EXECUTING = "executing"
    WAITING = "waiting"
    ERROR = "error"


class VLACoordinatorNode(Node):
    """
    Node to coordinate the overall VLA system workflow.
    """

    def __init__(self):
        super().__init__('vla_coordinator_node')

        # Initialize system state
        self.current_state = VLAState.IDLE
        self.pending_commands = []
        self.current_plan = []
        self.current_action = None
        self.system_context = {}

        # Create subscribers
        self.voice_command_sub = self.create_subscription(
            String,
            '/voice_commands',
            self.voice_command_callback,
            10
        )

        self.vla_status_sub = self.create_subscription(
            String,
            '/vla_status',
            self.vla_status_callback,
            10
        )

        self.fused_state_sub = self.create_subscription(
            String,
            '/fused_state',
            self.fused_state_callback,
            10
        )

        self.decision_sub = self.create_subscription(
            String,
            '/fused_decision',
            self.decision_callback,
            10
        )

        # Create publishers
        self.command_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.system_status_pub = self.create_publisher(String, '/system_status', 10)
        self.coordinator_status_pub = self.create_publisher(String, '/coordinator_status', 10)

        # Timer for state management
        self.state_timer = self.create_timer(0.1, self.state_management_loop)

        self.get_logger().info('VLA coordinator node initialized')

    def voice_command_callback(self, msg):
        """Handle voice commands."""
        command = msg.data
        self.get_logger().info(f'Received voice command: {command}')

        # Add to pending commands
        self.pending_commands.append({
            'command': command,
            'timestamp': time.time(),
            'priority': 1  # Default priority
        })

        # Update state if idle
        if self.current_state == VLAState.IDLE:
            self.transition_to(VLAState.PLANNING)

    def vla_status_callback(self, msg):
        """Handle VLA status updates."""
        try:
            status = json.loads(msg.data)
            self.system_context.update(status)
        except json.JSONDecodeError:
            self.get_logger().error('Error parsing VLA status')

    def fused_state_callback(self, msg):
        """Handle fused state updates."""
        try:
            fused_state = json.loads(msg.data)
            self.system_context.update(fused_state)
        except json.JSONDecodeError:
            self.get_logger().error('Error parsing fused state')

    def decision_callback(self, msg):
        """Handle decision updates."""
        try:
            decision = json.loads(msg.data)
            self.handle_decision(decision)
        except json.JSONDecodeError:
            self.get_logger().error('Error parsing decision')

    def state_management_loop(self):
        """Main state management loop."""
        if self.current_state == VLAState.IDLE:
            # Check for pending commands
            if self.pending_commands:
                self.transition_to(VLAState.PLANNING)

        elif self.current_state == VLAState.PLANNING:
            # Process pending commands and create plans
            self.process_commands()

        elif self.current_state == VLAState.EXECUTING:
            # Monitor execution progress
            self.monitor_execution()

        elif self.current_state == VLAState.WAITING:
            # Check if waiting condition is resolved
            self.check_waiting_conditions()

        # Publish coordinator status
        self.publish_coordinator_status()

    def process_commands(self):
        """Process pending commands and create plans."""
        if not self.pending_commands:
            self.transition_to(VLAState.IDLE)
            return

        # Sort commands by priority
        sorted_commands = sorted(self.pending_commands, key=lambda x: x['priority'], reverse=True)
        command = sorted_commands[0]

        # Generate plan for the command
        # In practice, this would call the LLM planning node
        self.current_plan = self.generate_plan(command['command'])

        if self.current_plan:
            self.pending_commands.remove(command)
            self.transition_to(VLAState.EXECUTING)
        else:
            self.get_logger().error(f'Failed to generate plan for command: {command["command"]}')
            self.transition_to(VLAState.ERROR)

    def generate_plan(self, command: str) -> List[Dict]:
        """Generate plan for command (simplified for example)."""
        # This would normally call the LLM planning node
        # For this example, we'll use a simple rule-based approach
        command_lower = command.lower()

        if 'kitchen' in command_lower:
            return [
                {'action': 'navigate_to', 'parameters': {'location': 'kitchen'}},
                {'action': 'report_arrival', 'parameters': {'location': 'kitchen'}}
            ]
        elif 'bedroom' in command_lower:
            return [
                {'action': 'navigate_to', 'parameters': {'location': 'bedroom'}},
                {'action': 'report_arrival', 'parameters': {'location': 'bedroom'}}
            ]
        elif 'stop' in command_lower or 'halt' in command_lower:
            return [
                {'action': 'stop_robot', 'parameters': {}}
            ]
        else:
            return [
                {'action': 'acknowledge', 'parameters': {'command': command}}
            ]

    def monitor_execution(self):
        """Monitor plan execution."""
        if not self.current_plan:
            self.transition_to(VLAState.IDLE)
            return

        # Execute next action in plan
        if self.current_action is None and self.current_plan:
            self.current_action = self.current_plan.pop(0)
            self.execute_current_action()

    def execute_current_action(self):
        """Execute the current action."""
        if not self.current_action:
            return

        action_type = self.current_action['action']
        parameters = self.current_action['parameters']

        self.get_logger().info(f'Executing action: {action_type}')

        # Execute based on action type
        if action_type == 'navigate_to':
            self.execute_navigation(parameters)
        elif action_type == 'stop_robot':
            self.execute_stop()
        elif action_type == 'acknowledge':
            self.execute_acknowledgment(parameters)

        # Mark action as complete and move to next
        self.current_action = None

        # If plan is complete, return to idle
        if not self.current_plan:
            self.transition_to(VLAState.IDLE)

    def execute_navigation(self, params: Dict):
        """Execute navigation action."""
        location = params.get('location', 'unknown')
        self.get_logger().info(f'Navigating to {location}')
        # In practice, this would send navigation goals

    def execute_stop(self):
        """Execute stop action."""
        twist = Twist()
        self.command_pub.publish(twist)
        self.get_logger().info('Robot stopped')

    def execute_acknowledgment(self, params: Dict):
        """Execute acknowledgment action."""
        command = params.get('command', 'unknown')
        self.get_logger().info(f'Acknowledged command: {command}')

    def handle_decision(self, decision: Dict):
        """Handle fused decision."""
        action = decision.get('action', 'unknown')
        confidence = decision.get('confidence', 0.0)

        if confidence > 0.7:  # High confidence decision
            if action == 'avoid_obstacle':
                self.avoid_obstacle()
            elif action == 'execute_command':
                command = decision.get('command', 'unknown')
                self.execute_external_command(command)

    def avoid_obstacle(self):
        """Handle obstacle avoidance decision."""
        self.get_logger().info('Avoiding obstacle')
        # Implement obstacle avoidance logic
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5  # Turn to avoid
        self.command_pub.publish(twist)

    def execute_external_command(self, command: str):
        """Execute externally generated command."""
        self.get_logger().info(f'Executing external command: {command}')
        # Add to pending commands
        self.pending_commands.append({
            'command': command,
            'timestamp': time.time(),
            'priority': 2  # Higher priority for system decisions
        })

    def transition_to(self, new_state: VLAState):
        """Transition to new state."""
        old_state = self.current_state
        self.current_state = new_state
        self.get_logger().info(f'State transition: {old_state.value} -> {new_state.value}')

    def check_waiting_conditions(self):
        """Check if waiting conditions are resolved."""
        # Implement logic to check if we can exit waiting state
        pass

    def publish_coordinator_status(self):
        """Publish coordinator status."""
        status_msg = String()
        status_data = {
            'current_state': self.current_state.value,
            'pending_commands': len(self.pending_commands),
            'current_plan_length': len(self.current_plan),
            'current_action': self.current_action['action'] if self.current_action else None,
            'timestamp': time.time()
        }
        status_msg.data = json.dumps(status_data)
        self.coordinator_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VLACoordinatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('VLA coordinator node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Creating a Complete VLA Launch File

**Create launch/vla_system_launch.py:**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    enable_vision = DeclareLaunchArgument(
        'enable_vision',
        default_value='true',
        description='Enable vision processing nodes'
    )

    enable_language = DeclareLaunchArgument(
        'enable_language',
        default_value='true',
        description='Enable language processing nodes'
    )

    # VLA integration node
    vla_integration_node = Node(
        package='vla_integration',
        executable='vla_integration_node',
        name='vla_integration_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        condition=IfCondition(LaunchConfiguration('enable_language')),
        output='screen'
    )

    # Multimodal fusion node
    multimodal_fusion_node = Node(
        package='vla_integration',
        executable='multimodal_fusion_node',
        name='multimodal_fusion_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        condition=IfCondition(LaunchConfiguration('enable_vision')),
        output='screen'
    )

    # VLA coordinator node
    vla_coordinator_node = Node(
        package='vla_integration',
        executable='vla_coordinator_node',
        name='vla_coordinator_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # Launch description
    ld = LaunchDescription([
        use_sim_time,
        enable_vision,
        enable_language,
        vla_integration_node,
        multimodal_fusion_node,
        vla_coordinator_node
    ])

    return ld
```

### Creating a System Monitor Node

```python
#!/usr/bin/env python3

"""
System monitor node for VLA system health and performance.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from typing import Dict, List
import time
import psutil
import threading


class SystemMonitorNode(Node):
    """
    Node to monitor VLA system health and performance.
    """

    def __init__(self):
        super().__init__('system_monitor_node')

        # Create subscribers for system status
        self.vla_status_sub = self.create_subscription(
            String,
            '/vla_status',
            self.vla_status_callback,
            10
        )

        self.coordinator_status_sub = self.create_subscription(
            String,
            '/coordinator_status',
            self.coordinator_status_callback,
            10
        )

        # Create diagnostic publisher
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # Initialize monitoring variables
        self.vla_status = {}
        self.coordinator_status = {}
        self.monitoring_data = {}

        # Timer for periodic diagnostics
        self.diag_timer = self.create_timer(1.0, self.publish_diagnostics)

        self.get_logger().info('System monitor node initialized')

    def vla_status_callback(self, msg):
        """Handle VLA status updates."""
        try:
            self.vla_status = eval(msg.data)  # In practice, use json.loads
        except:
            self.get_logger().error('Error parsing VLA status')

    def coordinator_status_callback(self, msg):
        """Handle coordinator status updates."""
        try:
            self.coordinator_status = eval(msg.data)  # In practice, use json.loads
        except:
            self.get_logger().error('Error parsing coordinator status')

    def publish_diagnostics(self):
        """Publish system diagnostics."""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # System resources
        cpu_percent = psutil.cpu_percent(interval=1)
        memory_percent = psutil.virtual_memory().percent
        disk_percent = psutil.disk_usage('/').percent

        # VLA subsystem status
        vla_diag = DiagnosticStatus()
        vla_diag.name = 'VLA System Status'
        vla_diag.hardware_id = 'vla_integration'

        # Determine overall status
        if self.coordinator_status.get('current_state') == 'error':
            vla_diag.level = DiagnosticStatus.ERROR
            vla_diag.message = 'System in error state'
        elif self.vla_status.get('execution_status') == 'failed':
            vla_diag.level = DiagnosticStatus.WARN
            vla_diag.message = 'Execution issues detected'
        else:
            vla_diag.level = DiagnosticStatus.OK
            vla_diag.message = 'System operational'

        # Add key-value pairs for detailed info
        vla_diag.values.extend([
            KeyValue(key='CPU Usage (%)', value=str(cpu_percent)),
            KeyValue(key='Memory Usage (%)', value=str(memory_percent)),
            KeyValue(key='Disk Usage (%)', value=str(disk_percent)),
            KeyValue(key='Current State', value=self.coordinator_status.get('current_state', 'unknown')),
            KeyValue(key='Pending Commands', value=str(self.coordinator_status.get('pending_commands', 0))),
            KeyValue(key='Vision Available', value=str(self.vla_status.get('vision_available', False))),
            KeyValue(key='Language Input Pending', value=str(self.vla_status.get('language_input_pending', False)))
        ])

        diag_array.status.append(vla_diag)

        # Publish diagnostics
        self.diag_pub.publish(diag_array)


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('System monitor node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Testing & Verification

### Running the Complete VLA System

1. **Build the integration packages:**
```bash
cd ~/ros2_ws
colcon build --packages-select vla_integration
source install/setup.bash
```

2. **Run the complete VLA system:**
```bash
# Terminal 1: Launch the complete system
ros2 launch vla_integration vla_system_launch.py

# Terminal 2: Send test commands
ros2 topic pub /voice_commands std_msgs/msg/String "data: 'Go to the kitchen'"

# Terminal 3: Monitor system status
ros2 topic echo /system_status

# Terminal 4: Monitor diagnostics
ros2 topic echo /diagnostics
```

3. **Test multimodal integration:**
```bash
# Combine voice and vision inputs
ros2 topic pub /voice_commands std_msgs/msg/String "data: 'Find the red ball'"
# Then check if perception system responds appropriately
```

### Useful VLA Integration Commands

- **Monitor all VLA topics:**
```bash
# Use multiple terminals to monitor
ros2 topic echo /vla_status
ros2 topic echo /fused_state
ros2 topic echo /fused_decision
ros2 topic echo /coordinator_status
```

- **Check system diagnostics:**
```bash
ros2 topic echo /diagnostics
```

- **Visualize in RViz:**
```bash
# For navigation and perception visualization
rviz2
```

### Performance Testing

```bash
# Test response times for different input combinations
# Test coordination between modalities
# Test system robustness under various conditions
# Measure resource utilization
```

## Common Issues

### Issue: Timing mismatches between modalities
**Solution**:
- Implement proper timestamp synchronization
- Use message filters for time-based synchronization
- Implement buffering mechanisms for temporal alignment
- Use appropriate QoS profiles for different data types

### Issue: Coordination conflicts between modalities
**Solution**:
- Implement priority-based arbitration
- Use state machines for clear coordination protocols
- Establish clear ownership of action execution
- Implement conflict resolution strategies

### Issue: Computational overload with multimodal processing
**Solution**:
- Use efficient fusion algorithms
- Implement selective processing based on relevance
- Use multi-threading for parallel processing
- Optimize individual modality processing pipelines

### Issue: Semantic gap between modalities
**Solution**:
- Develop shared semantic representations
- Implement cross-modal grounding mechanisms
- Use attention mechanisms to focus on relevant information
- Create domain-specific ontologies for concept alignment

## Key Takeaways

- VLA integration requires careful coordination of multiple processing modalities
- Temporal and spatial alignment is crucial for effective fusion
- System architecture should support modular development and testing
- Safety and error handling are paramount in integrated systems
- Performance monitoring ensures system reliability
- Clear interfaces between modules enable maintainable code
- Feedback loops improve system adaptability and robustness

## Next Steps

In the next chapter, you'll learn about the complete capstone project that brings together all the concepts learned in this book to create an autonomous humanoid robot system.