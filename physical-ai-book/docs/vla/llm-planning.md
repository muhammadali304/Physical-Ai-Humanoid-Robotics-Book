---
sidebar_position: 2
---

# LLM Planning - Cognitive Planning with Large Language Models

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand how Large Language Models (LLMs) can be integrated with robotic planning
- Implement cognitive planning systems using LLMs for high-level reasoning
- Create natural language interfaces for robot task specification
- Design prompting strategies for effective robot planning
- Integrate LLM-based planning with low-level robot control
- Evaluate and validate LLM-generated plans for safety and feasibility

## Prerequisites

Before starting this chapter, you should:
- Have ROS 2 Humble Hawksbill installed and configured
- Understand ROS 2 nodes, services, and action interfaces
- Completed the voice commands chapter for natural language understanding
- Basic knowledge of Python and machine learning concepts
- Understanding of robotic planning and navigation systems
- Familiarity with API calls and external service integration

## Conceptual Overview

**Large Language Models (LLMs)** bring cognitive reasoning capabilities to robotics by enabling natural language understanding, task decomposition, and high-level planning. Unlike traditional symbolic planners, LLMs can interpret natural language instructions, reason about complex multi-step tasks, and adapt to novel situations.

### Key Capabilities of LLMs in Robotics

1. **Natural Language Understanding**: Interpret human instructions in natural language
2. **Task Decomposition**: Break complex tasks into executable steps
3. **World Modeling**: Understand spatial relationships and environmental constraints
4. **Commonsense Reasoning**: Apply general knowledge to novel situations
5. **Adaptive Planning**: Modify plans based on changing conditions
6. **Human-Robot Interaction**: Engage in natural dialogue about tasks and goals

### LLM Integration Architecture

The typical LLM integration architecture includes:

```
Natural Language → LLM → Task Plan → ROS Actions → Robot Execution
```

With feedback loops for:
- Execution monitoring and error handling
- Plan refinement based on real-world observations
- Natural language responses to human operators

### Advantages of LLM-Based Planning

- **Natural Interaction**: Humans can specify tasks in everyday language
- **Flexibility**: Handle novel tasks without pre-programming
- **Reasoning**: Apply general knowledge to specific situations
- **Adaptability**: Adjust plans based on context and constraints
- **Scalability**: Leverage pre-trained models without extensive training

## Hands-On Implementation

### Installing LLM Dependencies

```bash
# Install LLM libraries
pip3 install openai anthropic transformers torch accelerate
pip3 install python-dotenv openai gymnasium numpy

# For local models (optional)
pip3 install llama-cpp-python
```

### Creating an LLM Planning Node

```python
#!/usr/bin/env python3

"""
LLM-based planning node for cognitive robotics.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from action_msgs.msg import GoalStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy
import openai
import os
import json
import asyncio
from typing import Dict, List, Optional
from dataclasses import dataclass


@dataclass
class RobotCapability:
    """Defines a robot's capabilities."""
    name: str
    description: str
    parameters: Dict[str, str]


@dataclass
class PlanningStep:
    """Represents a single step in a plan."""
    action: str
    parameters: Dict[str, str]
    description: str


class LLMPlanningNode(Node):
    """
    Node to use LLM for cognitive planning and task decomposition.
    """

    def __init__(self):
        super().__init__('llm_planning_node')

        # Initialize OpenAI API (use your API key)
        # For local models, use different approach
        self.api_key = os.getenv('OPENAI_API_KEY')
        if self.api_key:
            openai.api_key = self.api_key
            self.use_remote_llm = True
        else:
            self.get_logger().warn('OPENAI_API_KEY not found, using mock responses')
            self.use_remote_llm = False

        # Define robot capabilities
        self.capabilities = [
            RobotCapability(
                name="move_to",
                description="Move the robot to a specific location",
                parameters={"x": "float", "y": "float", "theta": "float", "frame_id": "string"}
            ),
            RobotCapability(
                name="pick_object",
                description="Pick up an object at the current location",
                parameters={"object_id": "string", "height": "float"}
            ),
            RobotCapability(
                name="place_object",
                description="Place an object at the current location",
                parameters={"object_id": "string", "height": "float"}
            ),
            RobotCapability(
                name="navigate_to",
                description="Navigate to a named location in the environment",
                parameters={"location_name": "string"}
            ),
            RobotCapability(
                name="detect_object",
                description="Detect and identify objects in the environment",
                parameters={"object_type": "string", "range": "float"}
            ),
            RobotCapability(
                name="follow_path",
                description="Follow a predefined path",
                parameters={"path_name": "string"}
            ),
            RobotCapability(
                name="report_status",
                description="Report the current status of the robot",
                parameters={}
            )
        ]

        # Create subscribers
        self.task_sub = self.create_subscription(
            String,
            'natural_language_task',
            self.task_callback,
            10
        )

        # Create publishers
        self.plan_pub = self.create_publisher(String, 'generated_plan', 10)
        self.response_pub = self.create_publisher(String, 'llm_response', 10)

        # Initialize variables
        self.current_plan = []
        self.task_history = []

        self.get_logger().info('LLM planning node initialized')

    def task_callback(self, msg):
        """Handle incoming natural language tasks."""
        task_description = msg.data
        self.get_logger().info(f'Received task: {task_description}')

        # Generate plan using LLM
        plan = self.generate_plan(task_description)

        if plan:
            # Publish the generated plan
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)

            # Publish response
            response_msg = String()
            response_msg.data = f'Generated plan with {len(plan)} steps for task: {task_description}'
            self.response_pub.publish(response_msg)

            self.get_logger().info(f'Published plan with {len(plan)} steps')
        else:
            self.get_logger().error('Failed to generate plan')

    def generate_plan(self, task_description: str) -> Optional[List[Dict]]:
        """Generate a plan for the given task using LLM."""
        try:
            # Create a prompt for the LLM
            prompt = self.create_planning_prompt(task_description)

            if self.use_remote_llm:
                # Call OpenAI API
                response = openai.ChatCompletion.create(
                    model="gpt-3.5-turbo",
                    messages=[
                        {"role": "system", "content": self.get_system_prompt()},
                        {"role": "user", "content": prompt}
                    ],
                    temperature=0.3,
                    max_tokens=1000
                )

                plan_text = response.choices[0].message.content.strip()
            else:
                # Mock response for testing without API key
                plan_text = self.mock_plan_generation(task_description)

            # Parse the response
            plan = self.parse_plan_response(plan_text)
            return plan

        except Exception as e:
            self.get_logger().error(f'Error generating plan: {e}')
            return None

    def create_planning_prompt(self, task_description: str) -> str:
        """Create a prompt for the LLM to generate a plan."""
        capabilities_str = "\n".join([
            f"- {cap.name}: {cap.description} (params: {', '.join(cap.parameters.keys())})"
            for cap in self.capabilities
        ])

        prompt = f"""
You are a helpful assistant that generates robot action plans from natural language descriptions.
The robot has the following capabilities:
{capabilities_str}

The environment has known locations like: kitchen, bedroom, living room, office, charging_station.

Given the task: "{task_description}"

Generate a step-by-step plan using the available robot capabilities. Return only a JSON array of steps with the following format:
[
  {{
    "action": "...",
    "parameters": {{"param1": "value1", "param2": "value2"}},
    "description": "..."
  }}
]

Be specific about locations and objects. If you need to navigate to a named location, use "navigate_to". If you need to go to specific coordinates, use "move_to".
"""

        return prompt

    def get_system_prompt(self) -> str:
        """Get the system prompt for the LLM."""
        return """
You are a helpful assistant that generates robot action plans from natural language descriptions.
Always respond with a valid JSON array of steps.
Each step should have "action", "parameters", and "description" fields.
Only use the robot capabilities that are available.
Be specific about locations and objects.
If information is missing, make reasonable assumptions based on common sense.
"""

    def parse_plan_response(self, response: str) -> Optional[List[Dict]]:
        """Parse the LLM response into a structured plan."""
        try:
            # Try to extract JSON from the response
            start_idx = response.find('[')
            end_idx = response.rfind(']') + 1

            if start_idx != -1 and end_idx != 0:
                json_str = response[start_idx:end_idx]
                plan = json.loads(json_str)

                # Validate the plan structure
                for step in plan:
                    if 'action' not in step or 'parameters' not in step or 'description' not in step:
                        raise ValueError("Invalid plan format")

                return plan
            else:
                self.get_logger().error(f'Could not find JSON in response: {response}')
                return None

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parsing JSON response: {e}')
            self.get_logger().info(f'Response: {response}')
            return None
        except Exception as e:
            self.get_logger().error(f'Error parsing plan response: {e}')
            return None

    def mock_plan_generation(self, task_description: str) -> str:
        """Generate a mock plan for testing without API key."""
        # This is a simplified mock - in reality, you'd want more sophisticated logic
        if "kitchen" in task_description.lower():
            return '''[
  {
    "action": "navigate_to",
    "parameters": {"location_name": "kitchen"},
    "description": "Navigate to the kitchen area"
  },
  {
    "action": "report_status",
    "parameters": {},
    "description": "Report arrival at kitchen"
  }
]'''
        elif "bedroom" in task_description.lower():
            return '''[
  {
    "action": "navigate_to",
    "parameters": {"location_name": "bedroom"},
    "description": "Navigate to the bedroom area"
  },
  {
    "action": "report_status",
    "parameters": {},
    "description": "Report arrival at bedroom"
  }
]'''
        else:
            return '''[
  {
    "action": "report_status",
    "parameters": {},
    "description": "Received task and ready to execute"
  }
]'''

    def validate_plan(self, plan: List[Dict]) -> bool:
        """Validate that the plan contains only valid actions."""
        valid_actions = [cap.name for cap in self.capabilities]

        for step in plan:
            if step['action'] not in valid_actions:
                self.get_logger().error(f'Invalid action in plan: {step["action"]}')
                return False

        return True


def main(args=None):
    rclpy.init(args=args)
    node = LLMPlanningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('LLM planning node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Creating a Plan Execution Node

```python
#!/usr/bin/env python3

"""
Node to execute plans generated by the LLM planning node.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
import json
import time
from typing import Dict, Any


class PlanExecutionNode(Node):
    """
    Node to execute plans generated by the LLM planning node.
    """

    def __init__(self):
        super().__init__('plan_execution_node')

        # Create subscribers
        self.plan_sub = self.create_subscription(
            String,
            'generated_plan',
            self.plan_callback,
            10
        )

        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.execution_status_pub = self.create_publisher(String, 'execution_status', 10)

        # Subscribe to odometry for position feedback
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # Initialize variables
        self.current_odom = None
        self.current_plan = []
        self.current_step_index = 0
        self.is_executing = False

        self.get_logger().info('Plan execution node initialized')

    def odom_callback(self, msg):
        """Update current robot position."""
        self.current_odom = msg

    def plan_callback(self, msg):
        """Handle incoming plan."""
        try:
            plan_data = json.loads(msg.data)
            self.get_logger().info(f'Received plan with {len(plan_data)} steps')

            # Store the plan and start execution
            self.current_plan = plan_data
            self.current_step_index = 0
            self.is_executing = True

            # Start executing the plan
            self.execute_next_step()

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error decoding plan: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing plan: {e}')

    def execute_next_step(self):
        """Execute the next step in the plan."""
        if not self.is_executing or self.current_step_index >= len(self.current_plan):
            self.is_executing = False
            self.publish_status('Plan completed')
            return

        step = self.current_plan[self.current_step_index]
        self.get_logger().info(f'Executing step {self.current_step_index + 1}: {step["action"]}')

        # Execute based on action type
        success = self.execute_action(step)

        if success:
            self.publish_status(f'Step completed: {step["description"]}')
            self.current_step_index += 1

            # Schedule next step after a delay
            self.create_timer(1.0, self.execute_next_step)
        else:
            self.publish_status(f'Step failed: {step["description"]}')
            self.is_executing = False

    def execute_action(self, step: Dict[str, Any]) -> bool:
        """Execute a single action from the plan."""
        action = step['action']
        parameters = step['parameters']

        try:
            if action == 'move_to':
                return self.execute_move_to(parameters)
            elif action == 'navigate_to':
                return self.execute_navigate_to(parameters)
            elif action == 'pick_object':
                return self.execute_pick_object(parameters)
            elif action == 'place_object':
                return self.execute_place_object(parameters)
            elif action == 'detect_object':
                return self.execute_detect_object(parameters)
            elif action == 'follow_path':
                return self.execute_follow_path(parameters)
            elif action == 'report_status':
                return self.execute_report_status(parameters)
            else:
                self.get_logger().error(f'Unknown action: {action}')
                return False

        except Exception as e:
            self.get_logger().error(f'Error executing action {action}: {e}')
            return False

    def execute_move_to(self, params: Dict[str, Any]) -> bool:
        """Execute move_to action."""
        x = float(params.get('x', 0.0))
        y = float(params.get('y', 0.0))
        theta = float(params.get('theta', 0.0))

        # Create and publish velocity command
        twist = Twist()
        twist.linear.x = 0.5  # Simple movement for demonstration
        twist.angular.z = 0.2

        # In a real implementation, this would interface with navigation stack
        self.cmd_vel_pub.publish(twist)

        self.get_logger().info(f'Moving to ({x}, {y}, {theta})')
        time.sleep(2)  # Simulate movement time

        # Stop the robot
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)

        return True

    def execute_navigate_to(self, params: Dict[str, Any]) -> bool:
        """Execute navigate_to action."""
        location_name = params.get('location_name', '')

        self.get_logger().info(f'Navigating to {location_name}')

        # In a real implementation, this would send goals to navigation system
        # For now, just simulate the action
        time.sleep(3)  # Simulate navigation time

        return True

    def execute_pick_object(self, params: Dict[str, Any]) -> bool:
        """Execute pick_object action."""
        object_id = params.get('object_id', '')
        height = float(params.get('height', 0.5))

        self.get_logger().info(f'Picking up object {object_id} at height {height}')

        # Simulate picking action
        time.sleep(2)

        return True

    def execute_place_object(self, params: Dict[str, Any]) -> bool:
        """Execute place_object action."""
        object_id = params.get('object_id', '')
        height = float(params.get('height', 0.5))

        self.get_logger().info(f'Placing object {object_id} at height {height}')

        # Simulate placing action
        time.sleep(2)

        return True

    def execute_detect_object(self, params: Dict[str, Any]) -> bool:
        """Execute detect_object action."""
        object_type = params.get('object_type', '')
        range_val = float(params.get('range', 1.0))

        self.get_logger().info(f'Detecting {object_type} within {range_val}m')

        # Simulate detection
        time.sleep(1)

        return True

    def execute_follow_path(self, params: Dict[str, Any]) -> bool:
        """Execute follow_path action."""
        path_name = params.get('path_name', '')

        self.get_logger().info(f'Following path {path_name}')

        # Simulate path following
        time.sleep(3)

        return True

    def execute_report_status(self, params: Dict[str, Any]) -> bool:
        """Execute report_status action."""
        self.get_logger().info('Reporting status')
        return True

    def publish_status(self, status: str):
        """Publish execution status."""
        status_msg = String()
        status_msg.data = status
        self.execution_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlanExecutionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Plan execution node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Creating a Task Manager Node

```python
#!/usr/bin/env python3

"""
Task manager node to coordinate LLM planning and execution.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
import asyncio
from typing import Dict, List, Optional


class TaskManagerNode(Node):
    """
    Node to manage the overall task execution flow.
    """

    def __init__(self):
        super().__init__('task_manager_node')

        # Create subscribers
        self.voice_command_sub = self.create_subscription(
            String,
            'voice_commands',  # From voice command node
            self.voice_command_callback,
            10
        )

        self.execution_status_sub = self.create_subscription(
            String,
            'execution_status',
            self.execution_status_callback,
            10
        )

        # Create publishers
        self.natural_task_pub = self.create_publisher(String, 'natural_language_task', 10)
        self.system_status_pub = self.create_publisher(String, 'system_status', 10)

        # Initialize variables
        self.current_task = None
        self.task_queue = []

        self.get_logger().info('Task manager node initialized')

    def voice_command_callback(self, msg):
        """Handle voice commands and convert to natural language tasks."""
        command = msg.data
        self.get_logger().info(f'Received voice command: {command}')

        # Convert voice command to natural language task
        # In a real system, you might do more sophisticated NLP here
        natural_task = self.convert_to_natural_task(command)

        if natural_task:
            # Publish the natural language task for LLM planning
            task_msg = String()
            task_msg.data = natural_task
            self.natural_task_pub.publish(task_msg)

            self.get_logger().info(f'Published natural task: {natural_task}')
        else:
            self.get_logger().warn(f'Could not convert command to natural task: {command}')

    def convert_to_natural_task(self, command: str) -> Optional[str]:
        """Convert voice command to natural language task."""
        # This is a simplified conversion - in practice, you'd have more sophisticated NLP
        command_lower = command.lower()

        # Define common command patterns and their natural language equivalents
        if 'move to' in command_lower or 'go to' in command_lower:
            # Extract location if specified
            if 'kitchen' in command_lower:
                return 'Navigate to the kitchen and wait there'
            elif 'bedroom' in command_lower:
                return 'Go to the bedroom and report your arrival'
            elif 'living room' in command_lower:
                return 'Move to the living room'
            else:
                # Try to extract a general location
                import re
                location_match = re.search(r'(?:to|the)\s+(\w+)', command_lower)
                if location_match:
                    location = location_match.group(1)
                    return f'Go to the {location} area'

        elif 'pick' in command_lower or 'grasp' in command_lower:
            # Extract object if specified
            import re
            object_match = re.search(r'(?:pick|grasp|take)\s+(?:up\s+)?(\w+)', command_lower)
            if object_match:
                obj = object_match.group(1)
                return f'Pick up the {obj} from the current location'

        elif 'place' in command_lower or 'put' in command_lower:
            # Extract object if specified
            import re
            object_match = re.search(r'(?:place|put)\s+(?:down\s+)?(\w+)', command_lower)
            if object_match:
                obj = object_match.group(1)
                return f'Put down the {obj} at the current location'

        elif 'stop' in command_lower or 'halt' in command_lower:
            return 'Stop all current activities and remain in place'

        elif 'status' in command_lower or 'where' in command_lower:
            return 'Report your current status and location'

        # If no specific pattern matched, use the command as-is with some context
        if len(command) > 3:  # At least 3 characters
            return f'{command.capitalize()}. Perform this task using your available capabilities.'

        return None

    def execution_status_callback(self, msg):
        """Handle execution status updates."""
        status = msg.data
        self.get_logger().info(f'Execution status: {status}')

        # Publish system status
        system_status = String()
        system_status.data = f'LLM Planning System: {status}'
        self.system_status_pub.publish(system_status)

        # Check if we have more tasks to process
        if self.task_queue and status.startswith('Plan completed'):
            next_task = self.task_queue.pop(0)
            task_msg = String()
            task_msg.data = next_task
            self.natural_task_pub.publish(task_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Task manager node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Creating a Local LLM Integration (Alternative to API-based)

```python
#!/usr/bin/env python3

"""
Local LLM integration using transformers library.
This provides an alternative to API-based LLMs for privacy or offline use.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from transformers import AutoTokenizer, AutoModelForCausalLM
import torch
import json
import os
from typing import Dict, List, Optional


class LocalLLMNode(Node):
    """
    Node to use local LLM for planning (privacy-preserving alternative).
    """

    def __init__(self):
        super().__init__('local_llm_node')

        # Initialize local model (using a smaller, faster model for demonstration)
        # For production, you might use models like Phi-3, TinyLlama, or others
        try:
            model_name = "microsoft/DialoGPT-small"  # Example - replace with appropriate model

            # For this example, we'll use a mock approach since actual model loading
            # might be resource-intensive for the documentation example
            self.use_local_model = False
            self.get_logger().info('Local LLM node initialized (mock mode)')

        except Exception as e:
            self.get_logger().warn(f'Could not load local model: {e}')
            self.use_local_model = False

        # Create subscribers
        self.task_sub = self.create_subscription(
            String,
            'natural_language_task',
            self.task_callback,
            10
        )

        # Create publishers
        self.plan_pub = self.create_publisher(String, 'generated_plan', 10)
        self.response_pub = self.create_publisher(String, 'llm_response', 10)

        self.get_logger().info('Local LLM planning node initialized')

    def task_callback(self, msg):
        """Handle incoming natural language tasks."""
        task_description = msg.data
        self.get_logger().info(f'Received task (local): {task_description}')

        # Generate plan using local processing or mock
        plan = self.generate_plan_local(task_description)

        if plan:
            # Publish the generated plan
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)

            # Publish response
            response_msg = String()
            response_msg.data = f'Generated plan with {len(plan)} steps for task: {task_description}'
            self.response_pub.publish(response_msg)

            self.get_logger().info(f'Published plan with {len(plan)} steps')
        else:
            self.get_logger().error('Failed to generate plan locally')

    def generate_plan_local(self, task_description: str) -> Optional[List[Dict]]:
        """Generate a plan locally (mock implementation)."""
        # In a real implementation, you would use a local LLM
        # For this example, we'll use rule-based generation with some ML concepts

        # Define a simple rule-based planner for demonstration
        task_lower = task_description.lower()

        plan = []

        # Rule-based planning logic
        if 'kitchen' in task_lower:
            plan.extend([
                {
                    "action": "navigate_to",
                    "parameters": {"location_name": "kitchen"},
                    "description": "Navigate to the kitchen area"
                },
                {
                    "action": "report_status",
                    "parameters": {},
                    "description": "Report arrival at kitchen"
                }
            ])
        elif 'bedroom' in task_lower:
            plan.extend([
                {
                    "action": "navigate_to",
                    "parameters": {"location_name": "bedroom"},
                    "description": "Navigate to the bedroom area"
                },
                {
                    "action": "report_status",
                    "parameters": {},
                    "description": "Report arrival at bedroom"
                }
            ])
        elif 'pick' in task_lower or 'grasp' in task_lower:
            # Extract object name if possible
            import re
            obj_match = re.search(r'(?:pick|grasp|take)\s+(?:up\s+)?(\w+)', task_lower)
            obj_name = obj_match.group(1) if obj_match else "object"

            plan.extend([
                {
                    "action": "detect_object",
                    "parameters": {"object_type": obj_name, "range": 1.0},
                    "description": f"Detect the {obj_name} in the environment"
                },
                {
                    "action": "pick_object",
                    "parameters": {"object_id": obj_name, "height": 0.5},
                    "description": f"Pick up the {obj_name}"
                }
            ])
        else:
            # Default action for unrecognized tasks
            plan.append({
                "action": "report_status",
                "parameters": {},
                "description": f"Received task: {task_description}"
            })

        return plan


def main(args=None):
    rclpy.init(args=args)
    node = LocalLLMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Local LLM node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Creating a Safety and Validation Node

```python
#!/usr/bin/env python3

"""
Safety and validation node for LLM-generated plans.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import json
from typing import Dict, List, Optional


class SafetyValidatorNode(Node):
    """
    Node to validate LLM-generated plans for safety and feasibility.
    """

    def __init__(self):
        super().__init__('safety_validator_node')

        # Create subscribers
        self.plan_sub = self.create_subscription(
            String,
            'generated_plan',
            self.plan_callback,
            10
        )

        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Create publishers
        self.validated_plan_pub = self.create_publisher(String, 'validated_plan', 10)
        self.safety_alert_pub = self.create_publisher(String, 'safety_alerts', 10)

        # Initialize variables
        self.laser_data = None
        self.map_data = None
        self.map_metadata = None

        self.get_logger().info('Safety validator node initialized')

    def plan_callback(self, msg):
        """Validate incoming plan."""
        try:
            plan = json.loads(msg.data)
            self.get_logger().info(f'Validating plan with {len(plan)} steps')

            # Validate the plan
            is_safe, validated_plan, alerts = self.validate_plan(plan)

            if is_safe:
                # Publish validated plan
                validated_msg = String()
                validated_msg.data = json.dumps(validated_plan)
                self.validated_plan_pub.publish(validated_msg)

                self.get_logger().info('Plan validated and published')
            else:
                # Publish safety alerts
                for alert in alerts:
                    alert_msg = String()
                    alert_msg.data = alert
                    self.safety_alert_pub.publish(alert_msg)

                self.get_logger().warn(f'Plan rejected with {len(alerts)} safety issues')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error decoding plan: {e}')
        except Exception as e:
            self.get_logger().error(f'Error validating plan: {e}')

    def validate_plan(self, plan: List[Dict]) -> tuple:
        """Validate plan for safety and feasibility."""
        alerts = []
        validated_plan = []

        for i, step in enumerate(plan):
            action = step['action']
            params = step['parameters']

            # Validate based on action type
            if action == 'move_to':
                is_valid, alert = self.validate_move_to(params)
                if not is_valid:
                    alerts.append(f'Step {i}: {alert}')
                    continue  # Skip invalid step
            elif action == 'navigate_to':
                is_valid, alert = self.validate_navigate_to(params)
                if not is_valid:
                    alerts.append(f'Step {i}: {alert}')
                    continue  # Skip invalid step

            # Add valid step to validated plan
            validated_plan.append(step)

        is_safe = len(alerts) == 0
        return is_safe, validated_plan, alerts

    def validate_move_to(self, params: Dict) -> tuple:
        """Validate move_to action for safety."""
        try:
            x = float(params.get('x', 0.0))
            y = float(params.get('y', 0.0))

            # Check if coordinates are within reasonable bounds
            if abs(x) > 100 or abs(y) > 100:
                return False, f'Coordinates ({x}, {y}) exceed safe operational bounds'

            # Check if destination is in known map (if map is available)
            if self.map_data and self.map_metadata:
                if not self.is_location_traversable(x, y):
                    return False, f'Destination ({x}, {y}) is not traversable'

            # Check for immediate obstacles (using laser data if available)
            if self.laser_data:
                if self.has_immediate_obstacles(x, y):
                    return False, f'Immediate obstacles detected near destination ({x}, {y})'

            return True, ""

        except (ValueError, TypeError) as e:
            return False, f'Invalid coordinates in parameters: {e}'

    def validate_navigate_to(self, params: Dict) -> tuple:
        """Validate navigate_to action for safety."""
        location_name = params.get('location_name', '').lower()

        # Check if location name is valid
        valid_locations = ['kitchen', 'bedroom', 'living room', 'office', 'charging_station']
        if location_name not in valid_locations:
            # For this example, we'll accept any location name
            # In a real system, you'd validate against known locations
            pass

        # Additional validation can be added based on map data
        if self.map_data and self.map_metadata:
            if not self.is_known_location(location_name):
                return False, f'Location "{location_name}" is not in known map'

        return True, ""

    def is_location_traversable(self, x: float, y: float) -> bool:
        """Check if a location is traversable based on map data."""
        if not self.map_data or not self.map_metadata:
            return True  # Assume traversable if no map data

        try:
            # Convert world coordinates to map coordinates
            origin_x = self.map_metadata.origin.position.x
            origin_y = self.map_metadata.origin.position.y
            resolution = self.map_metadata.resolution

            map_x = int((x - origin_x) / resolution)
            map_y = int((y - origin_y) / resolution)

            # Check bounds
            if (map_x < 0 or map_x >= self.map_metadata.width or
                map_y < 0 or map_y >= self.map_metadata.height):
                return False

            # Check occupancy value (0 = free, 100 = occupied)
            idx = map_y * self.map_metadata.width + map_x
            if idx < len(self.map_data.data):
                occupancy = self.map_data.data[idx]
                return occupancy < 50  # Consider <50 as traversable

        except Exception as e:
            self.get_logger().warn(f'Error checking traversability: {e}')

        return True

    def has_immediate_obstacles(self, x: float, y: float) -> bool:
        """Check if there are immediate obstacles near destination."""
        if not self.laser_data:
            return False

        # This is a simplified check - in practice, you'd do more sophisticated analysis
        # Check if any laser readings indicate close obstacles
        min_distance = min(self.laser_data.ranges) if self.laser_data.ranges else float('inf')
        return min_distance < 0.5  # Consider <0.5m as immediate obstacle

    def is_known_location(self, location_name: str) -> bool:
        """Check if location is known in the map."""
        # In a real implementation, this would check against semantic map
        # For this example, we'll return True to avoid blocking execution
        return True

    def laser_callback(self, msg):
        """Update laser data."""
        self.laser_data = msg

    def map_callback(self, msg):
        """Update map data."""
        self.map_data = msg.data
        self.map_metadata = msg.info


def main(args=None):
    rclpy.init(args=args)
    node = SafetyValidatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Safety validator node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Creating a Complete LLM Planning Package

**Create the package:**

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python llm_planning
```

**Update package.xml:**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>llm_planning</name>
  <version>0.0.1</version>
  <description>LLM-based planning for robotics</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Create setup.py:**

```python
from setuptools import find_packages, setup

package_name = 'llm_planning'

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
    description='LLM-based planning for robotics',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_planning_node = llm_planning.llm_planning_node:main',
            'plan_execution_node = llm_planning.plan_execution_node:main',
            'task_manager_node = llm_planning.task_manager_node:main',
            'local_llm_node = llm_planning.local_llm_node:main',
            'safety_validator_node = llm_planning.safety_validator_node:main',
        ],
    },
)
```

### Creating a Launch File

**Create launch/llm_planning_launch.py:**

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

    # LLM planning node
    llm_planning_node = Node(
        package='llm_planning',
        executable='llm_planning_node',
        name='llm_planning_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # Plan execution node
    plan_execution_node = Node(
        package='llm_planning',
        executable='plan_execution_node',
        name='plan_execution_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # Task manager node
    task_manager_node = Node(
        package='llm_planning',
        executable='task_manager_node',
        name='task_manager_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # Safety validator node
    safety_validator_node = Node(
        package='llm_planning',
        executable='safety_validator_node',
        name='safety_validator_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time,
        llm_planning_node,
        plan_execution_node,
        task_manager_node,
        safety_validator_node
    ])
```

## Testing & Verification

### Running LLM Planning System

1. **Set up your OpenAI API key (optional):**
```bash
# Create a .env file or export the key
export OPENAI_API_KEY="your-api-key-here"
```

2. **Install dependencies:**
```bash
pip3 install openai anthropic transformers torch
```

3. **Build the package:**
```bash
cd ~/ros2_ws
colcon build --packages-select llm_planning
source install/setup.bash
```

4. **Run the LLM planning system:**
```bash
# Terminal 1: Launch the system
ros2 launch llm_planning llm_planning_launch.py

# Terminal 2: Send a test task
ros2 topic pub /natural_language_task std_msgs/msg/String "data: 'Go to the kitchen and wait there'"
```

5. **Monitor the planning process:**
```bash
# Check generated plans
ros2 topic echo /generated_plan

# Check execution status
ros2 topic echo /execution_status

# Check safety alerts
ros2 topic echo /safety_alerts
```

### Useful LLM Planning Commands

- **Test with voice commands:**
```bash
# If you have the voice command system running
ros2 topic pub /voice_commands std_msgs/msg/String "data: 'move to kitchen'"
```

- **Monitor plan validation:**
```bash
ros2 topic echo /validated_plan
```

- **Check system status:**
```bash
ros2 topic echo /system_status
```

### Performance Testing

```bash
# Test response times for different types of commands
# Test with various levels of task complexity
# Test safety validation effectiveness
# Test integration with navigation system
```

## Common Issues

### Issue: API rate limits or costs with remote LLMs
**Solution**:
- Implement local models for frequently-used tasks
- Use caching for repeated requests
- Implement request queuing to respect rate limits
- Consider hybrid approach with local models for simple tasks

### Issue: Hallucinations or incorrect plans from LLM
**Solution**:
- Implement safety validation layers
- Use structured output formats (JSON)
- Add domain-specific constraints
- Implement plan verification before execution

### Issue: Latency in plan generation
**Solution**:
- Use smaller, faster models for real-time applications
- Implement plan caching for common tasks
- Use local inference where possible
- Implement asynchronous processing

### Issue: Safety and validation challenges
**Solution**:
- Implement multiple validation layers
- Use formal methods for critical safety checks
- Implement human-in-the-loop for high-risk actions
- Maintain conservative safety margins

## Key Takeaways

- LLMs enable natural language task specification for robots
- Safety validation is crucial for LLM-generated plans
- Local models provide privacy and reliability benefits
- Integration with existing ROS systems requires careful architecture
- Hybrid approaches combine LLM flexibility with traditional planning
- Human-robot collaboration is enhanced through cognitive planning
- Proper error handling and fallback mechanisms are essential

## Next Steps

In the next chapter, you'll learn about integrating voice commands with LLM planning to create a complete Vision-Language-Action system.