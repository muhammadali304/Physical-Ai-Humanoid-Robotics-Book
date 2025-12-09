# Goal Sending Interface for Navigation Testing

## Overview

This guide provides comprehensive instructions for implementing a goal sending interface for navigation testing. The interface allows users to send navigation goals to the robot, supports batch testing, and provides feedback on goal execution status. This is essential for validating navigation performance and testing different scenarios.

## Understanding Goal Sending Interface

### Components of Goal Sending Interface

1. **Goal Publisher**: Sends navigation goals to the navigation system
2. **Goal Manager**: Manages multiple goals, sequences, and priorities
3. **Feedback Monitor**: Tracks goal execution status and results
4. **Test Sequencer**: Runs predefined test sequences automatically
5. **Visualization Interface**: Shows goals and progress in RViz2

### Key Features

- Single goal sending
- Batch goal processing
- Goal sequence execution
- Real-time feedback monitoring
- Test result recording
- Integration with navigation stack

## Basic Goal Sending Implementation

### 1. Simple Goal Publisher Node

Create a basic goal publisher node `simple_goal_publisher.py`:

```python
#!/usr/bin/env python3
"""
Simple Goal Publisher for Navigation Testing
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
import time
import math


class SimpleGoalPublisher(Node):
    def __init__(self):
        super().__init__('simple_goal_publisher')

        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishers
        self.goal_status_pub = self.create_publisher(String, '/goal_status', 10)

        # Subscriptions
        self.goal_command_sub = self.create_subscription(
            String, '/goal_command', self.goal_command_callback, 10)

        # Internal state
        self.current_goal_id = 0
        self.goal_queue = []
        self.is_executing = False

        self.get_logger().info('Simple Goal Publisher initialized')

    def goal_command_callback(self, msg):
        """Handle goal commands"""
        command = msg.data.strip()

        if command.startswith('send_goal:'):
            # Parse goal coordinates: send_goal:x,y,yaw
            try:
                parts = command.split(':')[1].split(',')
                x = float(parts[0])
                y = float(parts[1])
                yaw = float(parts[2]) if len(parts) > 2 else 0.0

                self.send_single_goal(x, y, yaw)
            except (ValueError, IndexError):
                self.get_logger().error(f'Invalid goal command format: {command}')
        elif command == 'cancel_goal':
            self.cancel_current_goal()
        elif command.startswith('queue_goal:'):
            # Queue a goal for later execution
            try:
                parts = command.split(':')[1].split(',')
                x = float(parts[0])
                y = float(parts[1])
                yaw = float(parts[2]) if len(parts) > 2 else 0.0

                self.queue_goal(x, y, yaw)
            except (ValueError, IndexError):
                self.get_logger().error(f'Invalid queue command format: {command}')

    def send_single_goal(self, x, y, yaw):
        """Send a single navigation goal"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.is_executing = True
        self.current_goal_id += 1

        self.get_logger().info(f'Sending goal {self.current_goal_id}: ({x}, {y}, {yaw})')

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected')
            self.is_executing = False
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status = future.result().status

        if result is not None and status == 3:  # STATUS_SUCCEEDED
            self.get_logger().info('Goal reached successfully')
            status_msg = String()
            status_msg.data = 'GOAL_SUCCEEDED'
            self.goal_status_pub.publish(status_msg)
        else:
            self.get_logger().error(f'Goal failed with status: {status}')
            status_msg = String()
            status_msg.data = 'GOAL_FAILED'
            self.goal_status_pub.publish(status_msg)

        self.is_executing = False

        # Process next queued goal if available
        if self.goal_queue:
            next_goal = self.goal_queue.pop(0)
            self.send_single_goal(next_goal[0], next_goal[1], next_goal[2])

    def queue_goal(self, x, y, yaw):
        """Add a goal to the queue"""
        self.goal_queue.append((x, y, yaw))
        self.get_logger().info(f'Goal queued: ({x}, {y}, {yaw}). Queue size: {len(self.goal_queue)}')

        # If not currently executing, start processing
        if not self.is_executing and len(self.goal_queue) == 1:
            next_goal = self.goal_queue.pop(0)
            self.send_single_goal(next_goal[0], next_goal[1], next_goal[2])

    def cancel_current_goal(self):
        """Cancel the current goal"""
        self.get_logger().info('Canceling current goal')
        # In a real implementation, you would cancel the goal here
        self.is_executing = False


def main(args=None):
    rclpy.init(args=args)
    goal_publisher = SimpleGoalPublisher()

    try:
        rclpy.spin(goal_publisher)
    except KeyboardInterrupt:
        goal_publisher.get_logger().info('Goal publisher stopped by user')
    finally:
        goal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Advanced Goal Manager Node

Create a more sophisticated goal manager `advanced_goal_manager.py`:

```python
#!/usr/bin/env python3
"""
Advanced Goal Manager for Navigation Testing
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from rclpy.action import ActionClient
from std_msgs.msg import String, Int32, Float32
from builtin_interfaces.msg import Time
import time
import math
import json
from enum import Enum
from dataclasses import dataclass
from typing import List, Dict, Tuple


class GoalStatus(Enum):
    PENDING = 0
    ACTIVE = 1
    SUCCEEDED = 2
    FAILED = 3
    CANCELLED = 4


@dataclass
class NavigationGoal:
    id: int
    x: float
    y: float
    yaw: float
    priority: int = 1
    timeout: float = 30.0
    created_time: float = 0.0
    status: GoalStatus = GoalStatus.PENDING
    execution_time: float = 0.0
    result: str = ""


class AdvancedGoalManager(Node):
    def __init__(self):
        super().__init__('advanced_goal_manager')

        # Parameters
        self.declare_parameter('max_goals', 50)
        self.declare_parameter('default_timeout', 60.0)
        self.declare_parameter('goal_tolerance', 0.25)
        self.declare_parameter('min_goal_spacing', 0.1)

        self.max_goals = self.get_parameter('max_goals').value
        self.default_timeout = self.get_parameter('default_timeout').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.min_goal_spacing = self.get_parameter('min_goal_spacing').value

        # Navigation action clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        # Publishers
        self.goal_status_pub = self.create_publisher(String, '/goal_status', 10)
        self.goal_count_pub = self.create_publisher(Int32, '/goal_count', 10)
        self.goal_result_pub = self.create_publisher(String, '/goal_result', 10)
        self.active_goal_pub = self.create_publisher(PoseStamped, '/active_goal', 10)

        # Subscriptions
        self.goal_command_sub = self.create_subscription(
            String, '/goal_command', self.goal_command_callback, 10)
        self.goals_batch_sub = self.create_subscription(
            String, '/goals_batch', self.goals_batch_callback, 10)
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)

        # Internal state
        self.goals: List[NavigationGoal] = []
        self.active_goal_id = None
        self.goal_id_counter = 0
        self.current_robot_pose = None
        self.active_goal_timer = None
        self.timeout_timers = {}

        # Timer for periodic status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('Advanced Goal Manager initialized')

    def goal_command_callback(self, msg):
        """Handle goal commands"""
        command = msg.data.strip()

        if command.startswith('send_goal:'):
            self.handle_send_goal_command(command)
        elif command.startswith('send_goals:'):
            self.handle_send_goals_command(command)
        elif command.startswith('cancel_goal:'):
            goal_id = int(command.split(':')[1])
            self.cancel_goal(goal_id)
        elif command == 'cancel_all':
            self.cancel_all_goals()
        elif command == 'get_status':
            self.publish_all_goals_status()
        elif command.startswith('set_priority:'):
            self.handle_priority_command(command)

    def goals_batch_callback(self, msg):
        """Handle batch goals command"""
        try:
            batch_data = json.loads(msg.data)
            self.process_goals_batch(batch_data)
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in batch command: {msg.data}')

    def initial_pose_callback(self, msg):
        """Update current robot pose"""
        self.current_robot_pose = msg.pose.pose

    def handle_send_goal_command(self, command):
        """Handle single goal command"""
        try:
            # Format: send_goal:x,y,yaw,priority,timeout
            parts = command.split(':')[1].split(',')
            x = float(parts[0])
            y = float(parts[1])
            yaw = float(parts[2]) if len(parts) > 2 else 0.0
            priority = int(parts[3]) if len(parts) > 3 else 1
            timeout = float(parts[4]) if len(parts) > 4 else self.default_timeout

            goal_id = self.add_goal(x, y, yaw, priority, timeout)
            self.get_logger().info(f'Added goal {goal_id}: ({x}, {y}, {yaw}) with priority {priority}')

            # Start processing if no active goal
            if self.active_goal_id is None:
                self.process_next_goal()

        except (ValueError, IndexError):
            self.get_logger().error(f'Invalid goal command format: {command}')

    def handle_send_goals_command(self, command):
        """Handle multiple goals command"""
        try:
            # Format: send_goals:[x1,y1,yaw1,priority1,timeout1];[x2,y2,yaw2,priority2,timeout2];...
            goals_str = command.split(':')[1]
            goals_data = [g.strip('[]').split(',') for g in goals_str.split(';') if g.strip()]

            for goal_data in goals_data:
                x = float(goal_data[0])
                y = float(goal_data[1])
                yaw = float(goal_data[2]) if len(goal_data) > 2 else 0.0
                priority = int(goal_data[3]) if len(goal_data) > 3 else 1
                timeout = float(goal_data[4]) if len(goal_data) > 4 else self.default_timeout

                goal_id = self.add_goal(x, y, yaw, priority, timeout)
                self.get_logger().info(f'Added batch goal {goal_id}: ({x}, {y}, {yaw})')

            # Start processing if no active goal
            if self.active_goal_id is None:
                self.process_next_goal()

        except (ValueError, IndexError):
            self.get_logger().error(f'Invalid goals command format: {command}')

    def handle_priority_command(self, command):
        """Handle priority setting command"""
        try:
            # Format: set_priority:goal_id,priority
            parts = command.split(':')[1].split(',')
            goal_id = int(parts[0])
            new_priority = int(parts[1])

            for goal in self.goals:
                if goal.id == goal_id:
                    goal.priority = new_priority
                    self.get_logger().info(f'Updated priority for goal {goal_id} to {new_priority}')
                    break
            else:
                self.get_logger().error(f'Goal {goal_id} not found')

        except (ValueError, IndexError):
            self.get_logger().error(f'Invalid priority command format: {command}')

    def add_goal(self, x, y, yaw, priority=1, timeout=None) -> int:
        """Add a goal to the queue"""
        if len(self.goals) >= self.max_goals:
            self.get_logger().warn('Goal queue is full, removing lowest priority goal')
            # Remove lowest priority goal
            lowest_priority_goal = min(self.goals, key=lambda g: g.priority)
            self.goals.remove(lowest_priority_goal)

        self.goal_id_counter += 1
        goal = NavigationGoal(
            id=self.goal_id_counter,
            x=x,
            y=y,
            yaw=yaw,
            priority=priority,
            timeout=timeout or self.default_timeout,
            created_time=time.time()
        )

        self.goals.append(goal)
        self.sort_goals_by_priority()

        # Publish updated goal count
        count_msg = Int32()
        count_msg.data = len(self.goals)
        self.goal_count_pub.publish(count_msg)

        return self.goal_id_counter

    def sort_goals_by_priority(self):
        """Sort goals by priority (higher priority first) and creation time"""
        self.goals.sort(key=lambda g: (-g.priority, g.created_time))

    def process_next_goal(self):
        """Process the next goal in the queue"""
        if not self.goals:
            self.get_logger().info('No more goals to process')
            return

        # Find next pending goal
        next_goal = None
        for goal in self.goals:
            if goal.status == GoalStatus.PENDING:
                next_goal = goal
                break

        if next_goal:
            self.execute_goal(next_goal)

    def execute_goal(self, goal: NavigationGoal):
        """Execute a navigation goal"""
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available')
            goal.status = GoalStatus.FAILED
            self.publish_goal_result(goal.id, "Navigation server unavailable")
            self.process_next_goal()
            return

        goal.status = GoalStatus.ACTIVE
        self.active_goal_id = goal.id

        # Create navigation goal message
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.header.frame_id = 'map'
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav_goal.pose.pose.position.x = goal.x
        nav_goal.pose.pose.position.y = goal.y
        nav_goal.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        nav_goal.pose.pose.orientation.z = math.sin(goal.yaw / 2.0)
        nav_goal.pose.pose.orientation.w = math.cos(goal.yaw / 2.0)

        self.get_logger().info(f'Executing goal {goal.id}: ({goal.x}, {goal.y}, {goal.yaw})')

        # Publish active goal for visualization
        active_goal_msg = PoseStamped()
        active_goal_msg.header.frame_id = 'map'
        active_goal_msg.header.stamp = self.get_clock().now().to_msg()
        active_goal_msg.pose.position.x = goal.x
        active_goal_msg.pose.position.y = goal.y
        active_goal_msg.pose.position.z = 0.0
        active_goal_msg.pose.orientation.z = math.sin(goal.yaw / 2.0)
        active_goal_msg.pose.orientation.w = math.cos(goal.yaw / 2.0)
        self.active_goal_pub.publish(active_goal_msg)

        # Start timeout timer
        self.timeout_timers[goal.id] = self.create_timer(
            goal.timeout,
            lambda: self.handle_goal_timeout(goal.id)
        )

        # Send goal
        future = self.nav_to_pose_client.send_goal_async(nav_goal)
        future.add_done_callback(lambda f: self.goal_response_callback(f, goal.id))

    def goal_response_callback(self, future, goal_id):
        """Handle goal response"""
        goal = self.find_goal_by_id(goal_id)
        if not goal:
            return

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Goal {goal_id} was rejected')
            goal.status = GoalStatus.FAILED
            goal.result = 'Goal rejected by navigation server'
            self.publish_goal_result(goal_id, goal.result)
            self.process_next_goal()
            return

        self.get_logger().info(f'Goal {goal_id} accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.result_callback(f, goal_id))

    def result_callback(self, future, goal_id):
        """Handle navigation result"""
        goal = self.find_goal_by_id(goal_id)
        if not goal:
            return

        # Cancel timeout timer
        if goal_id in self.timeout_timers:
            self.timeout_timers[goal_id].cancel()
            del self.timeout_timers[goal_id]

        result = future.result().result
        status = future.result().status

        if result is not None and status == 3:  # STATUS_SUCCEEDED
            goal.status = GoalStatus.SUCCEEDED
            goal.result = 'Goal reached successfully'
            goal.execution_time = time.time() - goal.created_time
            self.get_logger().info(f'Goal {goal_id} succeeded in {goal.execution_time:.2f}s')
        else:
            goal.status = GoalStatus.FAILED
            goal.result = f'Goal failed with status: {status}'
            self.get_logger().error(f'Goal {goal_id} failed: {goal.result}')

        # Publish result
        self.publish_goal_result(goal_id, goal.result)

        # Clear active goal if it's the current one
        if self.active_goal_id == goal_id:
            self.active_goal_id = None

        # Process next goal
        self.process_next_goal()

    def handle_goal_timeout(self, goal_id):
        """Handle goal timeout"""
        goal = self.find_goal_by_id(goal_id)
        if not goal:
            return

        self.get_logger().warn(f'Goal {goal_id} timed out')
        goal.status = GoalStatus.FAILED
        goal.result = f'Goal timed out after {goal.timeout}s'

        # Publish result
        self.publish_goal_result(goal_id, goal.result)

        # Clear active goal if it's the current one
        if self.active_goal_id == goal_id:
            self.active_goal_id = None
            self.process_next_goal()

    def cancel_goal(self, goal_id):
        """Cancel a specific goal"""
        goal = self.find_goal_by_id(goal_id)
        if not goal:
            self.get_logger().error(f'Goal {goal_id} not found')
            return

        if goal.status == GoalStatus.PENDING:
            # Remove from queue
            self.goals.remove(goal)
            self.get_logger().info(f'Cancelled pending goal {goal_id}')
        elif goal.status == GoalStatus.ACTIVE and self.active_goal_id == goal_id:
            # TODO: In a real implementation, cancel the active goal
            goal.status = GoalStatus.CANCELLED
            goal.result = 'Goal cancelled by user'
            self.active_goal_id = None
            self.get_logger().info(f'Cancelled active goal {goal_id}')
            self.process_next_goal()
        else:
            self.get_logger().warn(f'Cannot cancel goal {goal_id} with status {goal.status}')

        # Publish result
        self.publish_goal_result(goal_id, goal.result)

    def cancel_all_goals(self):
        """Cancel all pending goals"""
        pending_goals = [g for g in self.goals if g.status == GoalStatus.PENDING]
        for goal in pending_goals:
            goal.status = GoalStatus.CANCELLED
            goal.result = 'Goal cancelled by user'
            self.publish_goal_result(goal.id, goal.result)

        self.goals = [g for g in self.goals if g.status != GoalStatus.PENDING]
        self.get_logger().info(f'Cancelled {len(pending_goals)} pending goals')

    def process_goals_batch(self, batch_data):
        """Process a batch of goals from JSON data"""
        try:
            for goal_data in batch_data.get('goals', []):
                x = goal_data.get('x', 0.0)
                y = goal_data.get('y', 0.0)
                yaw = goal_data.get('yaw', 0.0)
                priority = goal_data.get('priority', 1)
                timeout = goal_data.get('timeout', self.default_timeout)

                goal_id = self.add_goal(x, y, yaw, priority, timeout)
                self.get_logger().info(f'Added batch goal {goal_id}: ({x}, {y}, {yaw})')

            # Start processing if no active goal
            if self.active_goal_id is None:
                self.process_next_goal()

        except Exception as e:
            self.get_logger().error(f'Error processing batch goals: {e}')

    def find_goal_by_id(self, goal_id) -> NavigationGoal:
        """Find a goal by its ID"""
        for goal in self.goals:
            if goal.id == goal_id:
                return goal
        return None

    def publish_goal_result(self, goal_id, result):
        """Publish goal result"""
        result_msg = String()
        result_msg.data = f'GOAL_{goal_id}:{result}'
        self.goal_result_pub.publish(result_msg)

    def publish_status(self):
        """Publish current status"""
        status_msg = String()
        status_msg.data = f'ACTIVE_GOALS:{len([g for g in self.goals if g.status == GoalStatus.ACTIVE])},PENDING_GOALS:{len([g for g in self.goals if g.status == GoalStatus.PENDING])},COMPLETED_GOALS:{len([g for g in self.goals if g.status in [GoalStatus.SUCCEEDED, GoalStatus.FAILED, GoalStatus.CANCELLED]])}'
        self.goal_status_pub.publish(status_msg)

    def publish_all_goals_status(self):
        """Publish status of all goals"""
        status_data = []
        for goal in self.goals:
            status_data.append({
                'id': goal.id,
                'status': goal.status.name,
                'x': goal.x,
                'y': goal.y,
                'yaw': goal.yaw,
                'priority': goal.priority,
                'result': goal.result
            })

        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.goal_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    goal_manager = AdvancedGoalManager()

    try:
        rclpy.spin(goal_manager)
    except KeyboardInterrupt:
        goal_manager.get_logger().info('Goal manager stopped by user')
    finally:
        goal_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## RViz2 Goal Interface

### 1. RViz2 Goal Tool Configuration

Create an RViz2 configuration file `goal_interface.rviz`:

```yaml
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /Grid1
        - /TF1
        - /Path1
        - /Pose1
        - /MarkerArray1
      Splitter Ratio: 0.5
    Tree Height: 787
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Goal1
      - /2D Pose1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
  - Class: nav2_rviz_plugins/Navigation 2
    Name: Navigation 2
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 100
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: false
      Tree:
        {}
      Update Interval: 0
      Value: true
    - Alpha: 0.699999988079071
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Map
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map_updates
      Use Timestamp: false
      Value: true
    - Alpha: 1
      Buffer Length: 1
      Class: rviz_default_plugins/Path
      Color: 25; 255; 0
      Enabled: true
      Head Diameter: 0.30000001192092896
      Head Length: 0.20000000298023224
      Length: 0.30000001192092896
      Line Style: Lines
      Line Width: 0.029999999329447746
      Name: Path
      Offset:
        X: 0
        Y: 0
        Z: 0
      Pose Color: 255; 85; 255
      Pose Style: None
      Radius: 0.029999999329447746
      Shaft Diameter: 0.10000000149011612
      Shaft Length: 0.10000000149011612
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /plan
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/Pose
      Color: 255; 25; 0
      Enabled: true
      Head Length: 0.30000001192092896
      Head Radius: 0.10000000149011612
      Name: Current Goal
      Shaft Length: 1
      Shaft Radius: 0.05000000074505806
      Shape: Arrow
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /active_goal
      Value: true
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: Goals Queue
      Namespaces:
        {}
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goals_markers
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
    - Class: rviz_default_plugins/SetInitialPose
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
    - Class: nav2_rviz_plugins/Navigation 2
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.5
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.5
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 1025
  Hide Left Dock: false
  Hide Right Dock: false
  Navigation 2:
    collapsed: false
  QMainWindow State: 000000ff00000000fd000000040000000000000156000003a7fc0200000009fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000003a7000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261fb00000010004e0061007600690067006100740069006f006e0020003201000003a9000000c90000000000000000fb0000000a0049006d00610067006501000003a7000001e20000000000000000000000010000010f000003a7fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d000003a7000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004420000000000000000fb0000000800540069006d006501000000000000045000000000000000000000050a000003a700000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Width: 1853
  X: 67
  Y: 27
```

### 2. Goal Visualization Node

Create a node to visualize goals in RViz2:

```python
#!/usr/bin/env python3
"""
Goal Visualization Node for RViz2
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math


class GoalVisualizer(Node):
    def __init__(self):
        super().__init__('goal_visualizer')

        # Publishers
        self.goals_markers_pub = self.create_publisher(MarkerArray, '/goals_markers', 10)
        self.active_goal_pub = self.create_publisher(Marker, '/active_goal_marker', 10)

        # Subscriptions
        self.goal_status_sub = self.create_subscription(
            String, '/goal_status', self.goal_status_callback, 10)
        self.active_goal_sub = self.create_subscription(
            PoseStamped, '/active_goal', self.active_goal_callback, 10)

        # Internal state
        self.goals = []
        self.active_goal = None

        # Timer for visualization updates
        self.viz_timer = self.create_timer(0.1, self.update_visualization)

        self.get_logger().info('Goal Visualizer initialized')

    def goal_status_callback(self, msg):
        """Process goal status updates"""
        # In a real implementation, this would parse the status message
        # and update the internal goals list
        pass

    def active_goal_callback(self, msg):
        """Update active goal visualization"""
        self.active_goal = msg

    def update_visualization(self):
        """Update goal visualization markers"""
        # Create markers for queued goals
        markers = MarkerArray()

        # Add queued goals
        for i, goal in enumerate(self.goals):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "queued_goals"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            marker.pose = goal.pose
            marker.scale.x = 0.5  # Length of arrow
            marker.scale.y = 0.1  # Width of arrow
            marker.scale.z = 0.1  # Height of arrow

            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0  # Blue for queued goals
            marker.color.a = 0.8

            markers.markers.append(marker)

        # Publish markers
        self.goals_markers_pub.publish(markers)

        # Publish active goal marker if available
        if self.active_goal:
            active_marker = Marker()
            active_marker.header.frame_id = "map"
            active_marker.header.stamp = self.get_clock().now().to_msg()
            active_marker.ns = "active_goal"
            active_marker.id = 0
            active_marker.type = Marker.ARROW
            active_marker.action = Marker.ADD

            active_marker.pose = self.active_goal.pose
            active_marker.scale.x = 0.8  # Longer arrow for active goal
            active_marker.scale.y = 0.2
            active_marker.scale.z = 0.2

            active_marker.color.r = 1.0  # Red for active goal
            active_marker.color.g = 0.0
            active_marker.color.b = 0.0
            active_marker.color.a = 1.0

            self.active_goal_pub.publish(active_marker)


def main(args=None):
    rclpy.init(args=args)
    visualizer = GoalVisualizer()

    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        visualizer.get_logger().info('Goal visualizer stopped by user')
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Test Scenario Implementation

### 1. Navigation Test Scenarios

Create a test scenarios configuration `navigation_test_scenarios.py`:

```python
#!/usr/bin/env python3
"""
Navigation Test Scenarios for Goal Interface
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
import time


class NavigationTestScenarios(Node):
    def __init__(self):
        super().__init__('navigation_test_scenarios')

        # Publishers
        self.goals_batch_pub = self.create_publisher(String, '/goals_batch', 10)
        self.goal_command_pub = self.create_publisher(String, '/goal_command', 10)

        # Test scenarios
        self.test_scenarios = {
            'corridor_test': [
                {'x': 1.0, 'y': 0.0, 'yaw': 0.0, 'priority': 1, 'timeout': 30.0},
                {'x': 3.0, 'y': 0.0, 'yaw': 0.0, 'priority': 1, 'timeout': 30.0},
                {'x': 5.0, 'y': 0.0, 'yaw': 0.0, 'priority': 1, 'timeout': 30.0},
                {'x': 7.0, 'y': 0.0, 'yaw': 0.0, 'priority': 1, 'timeout': 30.0}
            ],
            'maze_test': [
                {'x': -2.0, 'y': -8.0, 'yaw': 0.0, 'priority': 1, 'timeout': 60.0},
                {'x': -1.0, 'y': -6.5, 'yaw': 0.0, 'priority': 1, 'timeout': 60.0},
                {'x': 0.0, 'y': -5.0, 'yaw': 0.0, 'priority': 1, 'timeout': 60.0}
            ],
            'obstacle_avoidance_test': [
                {'x': -6.0, 'y': 0.0, 'yaw': 0.0, 'priority': 1, 'timeout': 45.0},
                {'x': -6.0, 'y': 2.0, 'yaw': 0.0, 'priority': 1, 'timeout': 45.0},
                {'x': -6.0, 'y': 4.0, 'yaw': 0.0, 'priority': 1, 'timeout': 45.0}
            ],
            'dynamic_obstacle_test': [
                {'x': 4.0, 'y': 4.0, 'yaw': 0.0, 'priority': 1, 'timeout': 90.0},
                {'x': 5.0, 'y': 5.0, 'yaw': 0.0, 'priority': 1, 'timeout': 90.0},
                {'x': 6.0, 'y': 6.0, 'yaw': 0.0, 'priority': 1, 'timeout': 90.0}
            ]
        }

        self.get_logger().info('Navigation Test Scenarios initialized')

    def run_scenario(self, scenario_name):
        """Run a specific test scenario"""
        if scenario_name not in self.test_scenarios:
            self.get_logger().error(f'Unknown scenario: {scenario_name}')
            return

        self.get_logger().info(f'Running scenario: {scenario_name}')

        # Publish the scenario goals as a batch
        scenario_data = {
            'scenario_name': scenario_name,
            'goals': self.test_scenarios[scenario_name],
            'timestamp': time.time()
        }

        goals_msg = String()
        goals_msg.data = json.dumps(scenario_data)
        self.goals_batch_pub.publish(goals_msg)

    def run_all_scenarios(self):
        """Run all test scenarios sequentially"""
        for scenario_name in self.test_scenarios.keys():
            self.get_logger().info(f'Starting scenario: {scenario_name}')
            self.run_scenario(scenario_name)

            # Wait between scenarios
            time.sleep(2.0)


def main(args=None):
    rclpy.init(args=args)
    test_scenarios = NavigationTestScenarios()

    # Example: Run a specific scenario
    test_scenarios.run_scenario('corridor_test')

    # Or run all scenarios
    # test_scenarios.run_all_scenarios()

    test_scenarios.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Command Line Interface

### 1. Goal Command Line Tool

Create a command line tool for sending goals:

```python
#!/usr/bin/env python3
"""
Command Line Interface for Navigation Goals
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import argparse


class GoalCLI(Node):
    def __init__(self):
        super().__init__('goal_cli')

        # Publisher
        self.goal_command_pub = self.create_publisher(String, '/goal_command', 10)

        self.get_logger().info('Goal CLI initialized')

    def send_goal(self, x, y, yaw=0.0, priority=1, timeout=60.0):
        """Send a single goal"""
        command = f'send_goal:{x},{y},{yaw},{priority},{timeout}'
        cmd_msg = String()
        cmd_msg.data = command
        self.goal_command_pub.publish(cmd_msg)
        self.get_logger().info(f'Sent goal: {command}')

    def send_goals_batch(self, goals_list):
        """Send a batch of goals"""
        goals_str = ';'.join([f'[{x},{y},{yaw},{priority},{timeout}]'
                             for x, y, yaw, priority, timeout in goals_list])
        command = f'send_goals:{goals_str}'
        cmd_msg = String()
        cmd_msg.data = command
        self.goal_command_pub.publish(cmd_msg)
        self.get_logger().info(f'Sent batch goals: {command}')

    def cancel_goal(self, goal_id):
        """Cancel a specific goal"""
        command = f'cancel_goal:{goal_id}'
        cmd_msg = String()
        cmd_msg.data = command
        self.goal_command_pub.publish(cmd_msg)
        self.get_logger().info(f'Sent cancel command for goal: {goal_id}')

    def cancel_all_goals(self):
        """Cancel all goals"""
        command = 'cancel_all'
        cmd_msg = String()
        cmd_msg.data = command
        self.goal_command_pub.publish(cmd_msg)
        self.get_logger().info('Sent cancel all command')


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Navigation Goal CLI')
    parser.add_argument('command', choices=['send', 'batch', 'cancel', 'cancel_all', 'status'])
    parser.add_argument('--x', type=float, help='X coordinate')
    parser.add_argument('--y', type=float, help='Y coordinate')
    parser.add_argument('--yaw', type=float, default=0.0, help='Yaw angle')
    parser.add_argument('--priority', type=int, default=1, help='Goal priority')
    parser.add_argument('--timeout', type=float, default=60.0, help='Goal timeout')
    parser.add_argument('--goal_id', type=int, help='Goal ID for cancellation')

    args = parser.parse_args()

    cli = GoalCLI()

    if args.command == 'send':
        if args.x is None or args.y is None:
            print("Error: X and Y coordinates are required for 'send' command")
            sys.exit(1)
        cli.send_goal(args.x, args.y, args.yaw, args.priority, args.timeout)
    elif args.command == 'batch':
        # Example batch - in real usage, this would be parameterized
        goals = [
            (1.0, 1.0, 0.0, 1, 30.0),
            (2.0, 2.0, 0.0, 1, 30.0),
            (3.0, 3.0, 0.0, 1, 30.0)
        ]
        cli.send_goals_batch(goals)
    elif args.command == 'cancel':
        if args.goal_id is None:
            print("Error: Goal ID is required for 'cancel' command")
            sys.exit(1)
        cli.cancel_goal(args.goal_id)
    elif args.command == 'cancel_all':
        cli.cancel_all_goals()
    elif args.command == 'status':
        cmd_msg = String()
        cmd_msg.data = 'get_status'
        cli.goal_command_pub.publish(cmd_msg)
        cli.get_logger().info('Requested status')

    # Keep node alive briefly to send message
    rclpy.spin_once(cli, timeout_sec=0.1)
    cli.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Integration Launch Files

### 1. Complete Goal Interface Launch

Create a launch file `goal_interface_launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    run_rviz = LaunchConfiguration('run_rviz', default='true')
    autostart = LaunchConfiguration('autostart', default='true')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true')

    declare_run_rviz = DeclareLaunchArgument(
        'run_rviz',
        default_value='true',
        description='Whether to start RViz')

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    # Advanced goal manager
    goal_manager = Node(
        package='your_robot_navigation',
        executable='advanced_goal_manager',
        name='goal_manager',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_goals': 50,
            'default_timeout': 60.0,
            'goal_tolerance': 0.25
        }],
        output='screen'
    )

    # Goal visualizer
    goal_visualizer = Node(
        package='your_robot_navigation',
        executable='goal_visualizer',
        name='goal_visualizer',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Navigation2 stack
    navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart
        }.items()
    )

    # RViz2 with goal interface
    rviz = Node(
        condition=IfCondition(run_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('your_robot_navigation'),
            'rviz', 'goal_interface.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_run_rviz)
    ld.add_action(declare_autostart)

    # Add nodes
    ld.add_action(goal_manager)
    ld.add_action(goal_visualizer)
    ld.add_action(navigation2)
    ld.add_action(rviz)

    return ld
```

## Testing and Validation

### 1. Goal Interface Testing Script

Create a comprehensive testing script:

```python
#!/usr/bin/env python3
"""
Goal Interface Testing Script
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
import time
import json


class GoalInterfaceTester(Node):
    def __init__(self):
        super().__init__('goal_interface_tester')

        # Publishers
        self.goal_command_pub = self.create_publisher(String, '/goal_command', 10)
        self.goals_batch_pub = self.create_publisher(String, '/goals_batch', 10)

        # Subscriptions
        self.goal_status_sub = self.create_subscription(
            String, '/goal_status', self.goal_status_callback, 10)
        self.goal_result_sub = self.create_subscription(
            String, '/goal_result', self.goal_result_callback, 10)

        # Internal state
        self.test_results = []
        self.current_test = None

        self.get_logger().info('Goal Interface Tester initialized')

    def goal_status_callback(self, msg):
        """Handle goal status updates"""
        self.get_logger().info(f'Goal status: {msg.data}')

    def goal_result_callback(self, msg):
        """Handle goal result updates"""
        self.get_logger().info(f'Goal result: {msg.data}')

        # Record result
        self.test_results.append(msg.data)

    def run_comprehensive_tests(self):
        """Run comprehensive goal interface tests"""
        self.get_logger().info('Starting comprehensive goal interface tests...')

        # Test 1: Single goal
        self.test_single_goal()

        # Wait for completion
        time.sleep(5.0)

        # Test 2: Multiple goals
        self.test_multiple_goals()

        # Wait for completion
        time.sleep(10.0)

        # Test 3: Batch goals
        self.test_batch_goals()

        # Wait for completion
        time.sleep(10.0)

        # Test 4: Priority testing
        self.test_priority_goals()

        self.get_logger().info('All tests completed')
        self.print_test_summary()

    def test_single_goal(self):
        """Test single goal functionality"""
        self.get_logger().info('Testing single goal...')
        cmd_msg = String()
        cmd_msg.data = 'send_goal:1.0,1.0,0.0,1,30.0'
        self.goal_command_pub.publish(cmd_msg)

    def test_multiple_goals(self):
        """Test multiple sequential goals"""
        self.get_logger().info('Testing multiple goals...')
        # Send several goals in sequence
        for i in range(3):
            cmd_msg = String()
            cmd_msg.data = f'send_goal:{i+2}.0,{i+2}.0,0.0,1,30.0'
            self.goal_command_pub.publish(cmd_msg)
            time.sleep(0.5)  # Small delay between goals

    def test_batch_goals(self):
        """Test batch goal functionality"""
        self.get_logger().info('Testing batch goals...')
        batch_data = {
            'goals': [
                {'x': -1.0, 'y': -1.0, 'yaw': 0.0, 'priority': 1, 'timeout': 30.0},
                {'x': -2.0, 'y': -2.0, 'yaw': 0.0, 'priority': 1, 'timeout': 30.0},
                {'x': -3.0, 'y': -3.0, 'yaw': 0.0, 'priority': 1, 'timeout': 30.0}
            ]
        }

        batch_msg = String()
        batch_msg.data = json.dumps(batch_data)
        self.goals_batch_pub.publish(batch_msg)

    def test_priority_goals(self):
        """Test goal priority functionality"""
        self.get_logger().info('Testing goal priorities...')

        # Send low priority goal first
        cmd_msg = String()
        cmd_msg.data = 'send_goal:5.0,5.0,0.0,1,30.0'  # Low priority
        self.goal_command_pub.publish(cmd_msg)

        time.sleep(1.0)

        # Send high priority goal
        cmd_msg = String()
        cmd_msg.data = 'send_goal:6.0,6.0,0.0,5,30.0'  # High priority
        self.goal_command_pub.publish(cmd_msg)

    def print_test_summary(self):
        """Print test results summary"""
        self.get_logger().info('=== Test Results Summary ===')
        for result in self.test_results:
            self.get_logger().info(f'  {result}')
        self.get_logger().info(f'Total results recorded: {len(self.test_results)}')


def main(args=None):
    rclpy.init(args=args)
    tester = GoalInterfaceTester()

    # Run tests
    tester.run_comprehensive_tests()

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Best Practices and Tips

### 1. Goal Interface Best Practices

1. **Error Handling**: Always verify that the navigation server is available before sending goals
2. **Timeout Management**: Implement proper timeouts to prevent goals from hanging indefinitely
3. **Priority Management**: Use priority levels to handle important goals first
4. **Resource Management**: Limit the number of concurrent goals to prevent system overload
5. **Status Monitoring**: Continuously monitor goal status and provide feedback
6. **Graceful Degradation**: Handle failures gracefully and continue with other goals

### 2. Performance Considerations

- Use appropriate queue sizes to balance responsiveness and resource usage
- Implement proper cleanup of completed goals to prevent memory leaks
- Optimize goal validation to reduce processing overhead
- Consider using multi-threading for handling multiple concurrent goals

## Conclusion

This comprehensive goal sending interface provides all the necessary components for effective navigation testing:

- A simple publisher for basic goal sending
- An advanced manager for complex scenarios with priorities and batching
- RViz2 integration for visualization
- Command-line tools for easy testing
- Test scenarios for validation
- Proper error handling and status monitoring

The interface is designed to be modular and extensible, allowing for customization based on specific testing requirements. Regular testing with this interface will help ensure reliable navigation performance and identify potential issues before deployment.