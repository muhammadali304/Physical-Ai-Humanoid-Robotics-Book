# Capstone Project Integration Guide

This document provides a comprehensive guide for integrating all the components learned throughout the Physical AI and Humanoid Robotics curriculum into a cohesive capstone project.

## Overview

The capstone project integrates all major components covered in the curriculum:
- ROS 2 development environment
- Gazebo simulation
- Isaac ROS perception and navigation
- Vision-Language-Action (VLA) systems
- Reinforcement learning
- LLM integration
- Voice command processing
- Edge device deployment

## Capstone Project Architecture

### 1. System Architecture Diagram
```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   User Input    │    │   LLM Interface  │    │   ROS 2 Core    │
│                 │───▶│                  │───▶│                 │
│ • Voice Commands│    │ • Natural        │    │ • Navigation    │
│ • Text Commands │    │   Language       │    │ • Perception    │
│ • GUI Control   │    │   Processing     │    │ • Control       │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                              │
                              ▼
                    ┌──────────────────┐
                    │  Decision Engine │
                    │                  │
                    │ • Task Planning  │
                    │ • Behavior       │
                    │   Selection      │
                    └──────────────────┘
                              │
              ┌───────────────┼───────────────┐
              ▼               ▼               ▼
    ┌─────────────────┐┌─────────────────┐┌─────────────────┐
    │  Navigation     ││  Perception     ││  Manipulation  │
    │  System         ││  System         ││  System        │
    │                 ││                 ││                 │
    │ • Path Planning ││ • Object        ││ • Grasp         │
    │ • Localization  ││   Detection     ││   Planning      │
    │ • Path Execution││ • SLAM          ││ • Motion        │
    └─────────────────┘└─────────────────┘│   Control       │
                                        └─────────────────┘
```

### 2. Component Integration Points

#### A. ROS 2 Communication Layer
- **Action Servers**: Navigation goals, manipulation tasks
- **Topics**: Sensor data, robot state, command execution
- **Services**: System configuration, calibration, diagnostics
- **Parameters**: Runtime configuration, system settings

#### B. Simulation to Real World Interface
- **Gazebo Simulation**: Development and testing environment
- **Isaac Sim**: Advanced physics and RL training
- **Hardware Abstraction Layer**: Unified interface for sim/real

## Implementation Guide

### 1. Project Structure Setup

```bash
# Capstone project structure
capstone_project/
├── src/
│   ├── robot_control/           # Low-level robot control
│   ├── perception/              # Perception system
│   ├── navigation/              # Navigation system
│   ├── manipulation/            # Manipulation system
│   ├── llm_interface/           # LLM integration
│   ├── voice_control/           # Voice command processing
│   └── decision_engine/         # Main decision system
├── config/
│   ├── robot_config.yaml        # Robot-specific configuration
│   ├── navigation_params.yaml   # Navigation parameters
│   ├── perception_params.yaml   # Perception parameters
│   └── capstone_params.yaml     # Capstone project parameters
├── launch/
│   ├── capstone_sim.launch.py   # Simulation launch
│   ├── capstone_real.launch.py  # Real robot launch
│   └── capstone_test.launch.py  # Testing launch
├── worlds/                      # Simulation worlds
├── models/                      # Robot and object models
├── scripts/                     # Utility scripts
└── test/                        # Test cases
```

### 2. Core Integration Components

#### A. Main Capstone Node
```python
# src/decision_engine/capstone_manager.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, LaserScan
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist

import json
import asyncio
from typing import Dict, Any, Optional


class CapstoneManager(Node):
    """
    Main capstone project manager that integrates all components
    """
    def __init__(self):
        super().__init__('capstone_manager')

        # Initialize component managers
        self.llm_interface = LLMInterface(self)
        self.voice_processor = VoiceCommandProcessor(self)
        self.navigation_manager = NavigationManager(self)
        self.perception_manager = PerceptionManager(self)
        self.manipulation_manager = ManipulationManager(self)

        # Publishers
        self.status_pub = self.create_publisher(String, 'capstone_status', 10)
        self.command_pub = self.create_publisher(String, 'capstone_commands', 10)

        # Subscribers
        self.voice_sub = self.create_subscription(
            String, 'voice_commands', self.voice_command_callback, 10
        )
        self.text_sub = self.create_subscription(
            String, 'text_commands', self.text_command_callback, 10
        )

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # State management
        self.current_task = None
        self.robot_state = {
            'location': None,
            'battery_level': 100,
            'gripper_status': 'open',
            'current_action': 'idle'
        }

        # Task queue
        self.task_queue = []

        # Start periodic state update
        self.state_timer = self.create_timer(1.0, self.update_robot_state)

    def voice_command_callback(self, msg):
        """
        Handle voice commands through LLM processing
        """
        self.get_logger().info(f"Processing voice command: {msg.data}")

        # Process with LLM to extract intent
        intent = self.llm_interface.process_command(msg.data, self.robot_state)

        if intent:
            self.execute_task(intent)

    def text_command_callback(self, msg):
        """
        Handle text commands
        """
        self.get_logger().info(f"Processing text command: {msg.data}")

        # Parse and execute
        command = json.loads(msg.data)
        self.execute_task(command)

    def execute_task(self, task: Dict[str, Any]):
        """
        Execute a high-level task using appropriate subsystems
        """
        task_type = task.get('type', 'unknown')

        if task_type == 'navigation':
            self.navigation_manager.navigate_to(task['destination'])
        elif task_type == 'manipulation':
            self.manipulation_manager.manipulate_object(task['object'], task.get('action', 'pick'))
        elif task_type == 'perception':
            self.perception_manager.analyze_environment()
        elif task_type == 'combined':
            self.execute_combined_task(task)
        else:
            self.get_logger().error(f"Unknown task type: {task_type}")

    def execute_combined_task(self, task: Dict[str, Any]):
        """
        Execute complex tasks that involve multiple subsystems
        """
        steps = task.get('steps', [])

        for step in steps:
            step_type = step.get('type')
            if step_type == 'navigate':
                result = self.navigation_manager.navigate_to(step['location'])
                if not result.success:
                    self.get_logger().error(f"Navigation step failed: {step}")
                    return False
            elif step_type == 'perceive':
                perception_result = self.perception_manager.analyze_area(step['location'])
                step['perception_data'] = perception_result
            elif step_type == 'manipulate':
                result = self.manipulation_manager.manipulate_object(
                    step['object'], step['action'], step.get('location')
                )
                if not result.success:
                    self.get_logger().error(f"Manipulation step failed: {step}")
                    return False

        return True

    def update_robot_state(self):
        """
        Periodically update robot state from various sources
        """
        # Update location from localization system
        # Update battery from power system
        # Update gripper status from manipulation system
        # etc.

        state_msg = String()
        state_msg.data = json.dumps(self.robot_state)
        self.status_pub.publish(state_msg)

    def shutdown(self):
        """
        Clean shutdown of all components
        """
        self.navigation_manager.shutdown()
        self.perception_manager.shutdown()
        self.manipulation_manager.shutdown()
        self.llm_interface.shutdown()


class LLMInterface:
    """
    Interface to LLM for natural language processing
    """
    def __init__(self, node):
        self.node = node
        self.llm_client = OpenAIIntegrator(api_key="your-api-key")  # Or your preferred LLM

    def process_command(self, command: str, robot_state: Dict) -> Optional[Dict]:
        """
        Process natural language command and return structured task
        """
        try:
            structured_command = self.llm_client.generate_robot_command(command, robot_state)
            return structured_command
        except Exception as e:
            self.node.get_logger().error(f"LLM processing error: {e}")
            return None

    def shutdown(self):
        """
        Shutdown LLM interface
        """
        pass


class NavigationManager:
    """
    Navigation system manager
    """
    def __init__(self, node):
        self.node = node
        self.nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    def navigate_to(self, destination: str) -> Dict:
        """
        Navigate to specified destination
        """
        # Convert destination string to PoseStamped
        pose = self._get_pose_for_destination(destination)

        if pose is None:
            return {'success': False, 'error': f'Unknown destination: {destination}'}

        # Send navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)

        # Wait for result (with timeout)
        # Implementation would wait for result here
        return {'success': True, 'destination': destination}

    def _get_pose_for_destination(self, destination: str) -> Optional[PoseStamped]:
        """
        Get pose for named destination
        """
        # This would typically look up destinations in a map
        # For now, return a default pose
        if destination.lower() == 'kitchen':
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = 2.0
            pose.pose.position.y = 1.0
            pose.pose.position.z = 0.0
            return pose
        return None

    def shutdown(self):
        """
        Shutdown navigation manager
        """
        pass


class PerceptionManager:
    """
    Perception system manager
    """
    def __init__(self, node):
        self.node = node
        self.object_detector = None  # Initialize your object detection model

    def analyze_environment(self):
        """
        Analyze current environment
        """
        # Process sensor data to detect objects, people, etc.
        pass

    def analyze_area(self, location: str):
        """
        Analyze specific area
        """
        # Navigate to area and perform detailed analysis
        pass

    def shutdown(self):
        """
        Shutdown perception manager
        """
        pass


class ManipulationManager:
    """
    Manipulation system manager
    """
    def __init__(self, node):
        self.node = node

    def manipulate_object(self, obj: str, action: str, location: str = None):
        """
        Manipulate specified object
        """
        # Implement object manipulation logic
        pass

    def shutdown(self):
        """
        Shutdown manipulation manager
        """
        pass


def main(args=None):
    rclpy.init(args=args)

    capstone_manager = CapstoneManager()

    # Use multi-threaded executor to handle multiple callbacks
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(capstone_manager)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        capstone_manager.shutdown()
        capstone_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### B. Capstone Launch File
```python
# launch/capstone_sim.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='capstone_world.sdf',
        description='SDF world file for simulation'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='humanoid_robot',
        description='Name of the robot model'
    )

    # Get launch configurations
    world_name = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')

    # Launch Gazebo with capstone world
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('capstone_project'),
                'worlds',
                world_name
            ])
        }.items()
    )

    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    # Spawn robot in the world
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robot_name,
            '-x', '0.0', '-y', '0.0', '-z', '0.0'
        ],
        output='screen'
    )

    # Launch robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': PathJoinSubstitution([
                FindPackageShare('capstone_project'),
                'models',
                'humanoid_robot.urdf'
            ])
        }]
    )

    # Launch Nav2 stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': True}.items()
    )

    # Launch capstone manager
    capstone_manager = Node(
        package='capstone_project',
        executable='capstone_manager.py',
        name='capstone_manager',
        parameters=[{
            'use_sim_time': True,
            'config_file': PathJoinSubstitution([
                FindPackageShare('capstone_project'),
                'config',
                'capstone_params.yaml'
            ])
        }],
        output='screen'
    )

    # Launch voice command processor
    voice_processor = Node(
        package='capstone_project',
        executable='voice_processor.py',
        name='voice_processor',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Launch RViz for visualization
    rviz_config = PathJoinSubstitution([
        FindPackageShare('capstone_project'),
        'rviz',
        'capstone_view.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        world_arg,
        robot_name_arg,
        gzserver_launch,
        gzclient_launch,
        spawn_robot,
        robot_state_publisher,
        nav2_launch,
        capstone_manager,
        voice_processor,
        rviz_node
    ])
```

### 3. Capstone Project Scenarios

#### A. Scenario 1: Object Retrieval Task
```python
# capstone_scenarios/object_retrieval.py
class ObjectRetrievalScenario:
    """
    Scenario: Robot retrieves object based on voice command
    """
    def __init__(self, capstone_manager):
        self.capstone_manager = capstone_manager

    def execute(self):
        """
        Execute object retrieval scenario
        """
        # 1. Wait for voice command
        self.capstone_manager.get_logger().info("Waiting for object retrieval command...")

        # 2. Process command to identify object and destination
        # This would be triggered by voice input
        task = {
            'type': 'combined',
            'steps': [
                {
                    'type': 'perceive',
                    'location': 'living_room',
                    'description': 'Look for the red cup in the living room'
                },
                {
                    'type': 'navigate',
                    'location': 'living_room',
                    'description': 'Go to living room to find the cup'
                },
                {
                    'type': 'manipulate',
                    'action': 'pick',
                    'object': 'red cup',
                    'description': 'Pick up the red cup'
                },
                {
                    'type': 'navigate',
                    'location': 'kitchen',
                    'description': 'Go to kitchen'
                },
                {
                    'type': 'manipulate',
                    'action': 'place',
                    'object': 'red cup',
                    'location': 'kitchen_counter',
                    'description': 'Place cup on kitchen counter'
                }
            ]
        }

        # 3. Execute the task
        result = self.capstone_manager.execute_combined_task(task)

        # 4. Report results
        if result:
            self.capstone_manager.get_logger().info("Object retrieval task completed successfully!")
        else:
            self.capstone_manager.get_logger().error("Object retrieval task failed!")

        return result
```

#### B. Scenario 2: Room Navigation and Mapping
```python
# capstone_scenarios/room_mapping.py
class RoomMappingScenario:
    """
    Scenario: Robot maps a room and reports findings
    """
    def __init__(self, capstone_manager):
        self.capstone_manager = capstone_manager

    def execute(self):
        """
        Execute room mapping scenario
        """
        self.capstone_manager.get_logger().info("Starting room mapping scenario...")

        # 1. Navigate to different waypoints in the room
        waypoints = [
            {'name': 'entrance', 'x': 0.0, 'y': 0.0},
            {'name': 'center', 'x': 2.0, 'y': 1.0},
            {'name': 'corner1', 'x': 3.0, 'y': 2.0},
            {'name': 'corner2', 'x': 1.0, 'y': 3.0}
        ]

        # 2. Build map while navigating
        for waypoint in waypoints:
            self.capstone_manager.get_logger().info(f"Navigating to {waypoint['name']}")

            # Navigate to waypoint
            nav_task = {
                'type': 'navigation',
                'destination': waypoint['name']
            }
            self.capstone_manager.execute_task(nav_task)

            # 3. Perform perception at each waypoint
            perception_task = {
                'type': 'perception',
                'location': waypoint['name']
            }
            self.capstone_manager.execute_task(perception_task)

            # Small delay between waypoints
            import time
            time.sleep(2)

        # 4. Generate final map report
        self.capstone_manager.get_logger().info("Room mapping completed!")

        return True
```

### 4. Testing and Validation Framework

#### A. Unit Tests for Components
```python
# test/test_capstone_components.py
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from capstone_manager import CapstoneManager


class TestCapstoneComponents(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = CapstoneManager()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def tearDown(self):
        self.node.destroy_node()

    def test_navigation_integration(self):
        """Test navigation component integration"""
        # Test that navigation manager can be initialized
        self.assertIsNotNone(self.node.navigation_manager)

        # Test navigation to known location
        result = self.node.navigation_manager.navigate_to('kitchen')
        self.assertIsNotNone(result)

    def test_perception_integration(self):
        """Test perception component integration"""
        self.assertIsNotNone(self.node.perception_manager)

    def test_manipulation_integration(self):
        """Test manipulation component integration"""
        self.assertIsNotNone(self.node.manipulation_manager)

    def test_llm_integration(self):
        """Test LLM interface integration"""
        self.assertIsNotNone(self.node.llm_interface)


if __name__ == '__main__':
    unittest.main()
```

#### B. Integration Tests
```python
# test/test_capstone_integration.py
import unittest
import rclpy
from std_msgs.msg import String
import time


class TestCapstoneIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('capstone_integration_tester')

        # Create publisher for voice commands
        self.voice_pub = self.node.create_publisher(String, 'voice_commands', 10)

        # Create subscriber for status
        self.status_sub = self.node.create_subscription(
            String, 'capstone_status', self.status_callback, 10
        )

        self.status_received = False
        self.status_data = None

    def tearDown(self):
        self.node.destroy_node()

    def status_callback(self, msg):
        self.status_received = True
        self.status_data = msg.data

    def test_voice_command_processing(self):
        """Test end-to-end voice command processing"""
        # Send a test command
        test_cmd = String()
        test_cmd.data = "Go to the kitchen"
        self.voice_pub.publish(test_cmd)

        # Wait for response
        timeout = time.time() + 10.0  # 10 second timeout
        while not self.status_received and time.time() < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(self.status_received)
        self.assertIsNotNone(self.status_data)
```

### 5. Performance Monitoring and Optimization

#### A. Performance Dashboard
```python
# scripts/performance_monitor.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import psutil
import time
import matplotlib.pyplot as plt
from collections import deque
import threading


class CapstonePerformanceMonitor(Node):
    """
    Monitor and visualize capstone project performance
    """
    def __init__(self):
        super().__init__('capstone_performance_monitor')

        # Performance data storage
        self.cpu_history = deque(maxlen=100)
        self.memory_history = deque(maxlen=100)
        self.time_history = deque(maxlen=100)

        # Publishers for performance metrics
        self.cpu_pub = self.create_publisher(Float32, 'performance/cpu_percent', 10)
        self.memory_pub = self.create_publisher(Float32, 'performance/memory_percent', 10)

        # Timer for periodic monitoring
        self.monitor_timer = self.create_timer(1.0, self.monitor_performance)

        # Visualization thread
        self.viz_thread = threading.Thread(target=self.run_visualization)
        self.viz_thread.daemon = True
        self.viz_thread.start()

        self.start_time = time.time()

    def monitor_performance(self):
        """
        Monitor system performance metrics
        """
        cpu_percent = psutil.cpu_percent()
        memory_percent = psutil.virtual_memory().percent

        current_time = time.time() - self.start_time

        # Store in history
        self.cpu_history.append(cpu_percent)
        self.memory_history.append(memory_percent)
        self.time_history.append(current_time)

        # Publish metrics
        cpu_msg = Float32()
        cpu_msg.data = float(cpu_percent)
        self.cpu_pub.publish(cpu_msg)

        memory_msg = Float32()
        memory_msg.data = float(memory_percent)
        self.memory_pub.publish(memory_msg)

        self.get_logger().info(f"CPU: {cpu_percent:.1f}%, Memory: {memory_percent:.1f}%")

    def run_visualization(self):
        """
        Run real-time performance visualization
        """
        plt.ion()  # Interactive mode
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

        while rclpy.ok():
            if len(self.time_history) > 1:
                ax1.clear()
                ax1.plot(list(self.time_history), list(self.cpu_history), 'b-', label='CPU %')
                ax1.set_ylabel('CPU Usage (%)')
                ax1.set_title('Capstone Project Performance')
                ax1.legend()
                ax1.grid(True)

                ax2.clear()
                ax2.plot(list(self.time_history), list(self.memory_history), 'r-', label='Memory %')
                ax2.set_ylabel('Memory Usage (%)')
                ax2.set_xlabel('Time (seconds)')
                ax2.legend()
                ax2.grid(True)

                plt.tight_layout()
                plt.pause(0.01)

            time.sleep(0.1)

    def get_performance_summary(self):
        """
        Get performance summary statistics
        """
        if not self.cpu_history:
            return {}

        return {
            'avg_cpu': sum(self.cpu_history) / len(self.cpu_history),
            'max_cpu': max(self.cpu_history),
            'avg_memory': sum(self.memory_history) / len(self.memory_history),
            'max_memory': max(self.memory_history),
            'duration': time.time() - self.start_time
        }


def main(args=None):
    rclpy.init(args=args)
    monitor = CapstonePerformanceMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        summary = monitor.get_performance_summary()
        print(f"\nPerformance Summary:")
        print(f"  Duration: {summary.get('duration', 0):.2f}s")
        print(f"  Avg CPU: {summary.get('avg_cpu', 0):.2f}%")
        print(f"  Max CPU: {summary.get('max_cpu', 0):.2f}%")
        print(f"  Avg Memory: {summary.get('avg_memory', 0):.2f}%")
        print(f"  Max Memory: {summary.get('max_memory', 0):.2f}%")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 6. Deployment Considerations

#### A. Simulation to Real Deployment Checklist
```yaml
# config/deployment_checklist.yaml
deployment_checklist:
  simulation_phase:
    - "Verify all components work in Gazebo simulation"
    - "Test navigation in various simulated environments"
    - "Validate perception system with simulated sensors"
    - "Test voice command processing with simulated input"
    - "Run performance benchmarks in simulation"

  hardware_integration_phase:
    - "Verify hardware interfaces match simulation"
    - "Calibrate sensors for real-world conditions"
    - "Test safety systems and emergency stops"
    - "Validate communication protocols"
    - "Check power consumption profiles"

  real_world_testing_phase:
    - "Start with simple navigation tasks"
    - "Gradually increase task complexity"
    - "Monitor system performance and stability"
    - "Validate safety mechanisms"
    - "Collect real-world performance metrics"

  optimization_phase:
    - "Apply edge optimization techniques"
    - "Fine-tune for real-time performance"
    - "Optimize power consumption"
    - "Implement robust error handling"
    - "Validate system reliability"
```

#### B. Configuration Management
```yaml
# config/capstone_params.yaml
capstone_project:
  ros__parameters:
    # System parameters
    use_sim_time: true
    system_timeout: 30.0
    max_navigation_attempts: 3

    # LLM integration
    llm_api_key: "your-api-key-here"
    llm_model: "gpt-3.5-turbo"
    llm_temperature: 0.3

    # Voice processing
    voice_recognition_timeout: 10.0
    voice_energy_threshold: 4000
    wake_word_detection: true

    # Navigation parameters
    navigation:
      goal_tolerance: 0.5
      max_velocity: 0.5
      min_velocity: 0.1
      obstacle_threshold: 0.5

    # Perception parameters
    perception:
      detection_confidence_threshold: 0.7
      max_detection_range: 5.0
      object_classes: ["cup", "bottle", "box", "chair", "table"]

    # Manipulation parameters
    manipulation:
      grasp_success_threshold: 0.8
      manipulation_timeout: 15.0
      safety_margin: 0.1
```

## Evaluation Criteria

### 1. Technical Requirements
- **Integration**: All components work together seamlessly
- **Performance**: System meets real-time requirements
- **Reliability**: System operates without critical failures
- **Safety**: All safety mechanisms function correctly

### 2. Functional Requirements
- **Navigation**: Robot can navigate to specified locations
- **Perception**: Robot can identify and locate objects
- **Manipulation**: Robot can manipulate objects safely
- **Interaction**: Natural language interface works effectively

### 3. Assessment Rubric
- **Component Integration (25%)**: How well components work together
- **System Performance (25%)**: Real-time performance and efficiency
- **Functionality (25%)**: Completeness of implemented features
- **Robustness (15%)**: Error handling and system reliability
- **Documentation (10%)**: Quality of code and system documentation

## Troubleshooting and Maintenance

### Common Issues:
1. **Timing Issues**: Use appropriate QoS settings for real-time performance
2. **Memory Leaks**: Implement proper cleanup in all components
3. **Communication Failures**: Verify network configuration and topic connections
4. **Performance Bottlenecks**: Profile and optimize critical paths

### Maintenance Tasks:
- Regular performance monitoring
- System health checks
- Log analysis and error tracking
- Component updates and patches

## Resources and Further Development

### Extension Opportunities:
1. **Multi-Robot Coordination**: Extend to multiple robots
2. **Learning from Demonstration**: Add Imitation Learning
3. **Advanced Manipulation**: Complex bimanual tasks
4. **Human-Robot Collaboration**: Safe human-robot interaction

### Documentation References:
- [ROS 2 Documentation](https://docs.ros.org/)
- [Navigation2 Documentation](https://navigation.ros.org/)
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
- [OpenAI API Documentation](https://platform.openai.com/docs/api-reference)

This capstone project demonstrates the integration of all concepts learned in the Physical AI and Humanoid Robotics curriculum, providing a comprehensive example of a real-world robotics application.