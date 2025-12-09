# Path Execution Controller for Robot Movement

## Overview

This guide provides comprehensive instructions for implementing path execution controllers that translate planned paths into actual robot movement. The path execution controller is a critical component of the navigation stack that follows global plans while avoiding local obstacles and maintaining safe motion.

## Understanding Path Execution

### What is Path Execution?
Path execution is the process of converting a planned path (sequence of waypoints) into velocity commands that drive the robot along the path while avoiding obstacles and maintaining stability.

### Key Components
1. **Local Planner**: Generates velocity commands based on global path and local obstacles
2. **Trajectory Controller**: Smooths and executes motion commands
3. **Safety Controller**: Ensures safe operation and obstacle avoidance
4. **Feedback Controller**: Adjusts based on actual robot state

## Navigation2 Controller Configuration

### 1. Basic Controller Server Configuration

Create a controller configuration file `controller_config.yaml`:

```yaml
controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker parameters
    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    # Controller parameters
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
      restore_defaults: False
      publish_cost_grid_pc: False
      use_dwb: True
      max_vel_obstacle: 1.32
```

### 2. Advanced Controller Configuration

For more sophisticated path execution:

```yaml
controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 50.0  # Higher frequency for better control
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.5
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker", "precise_goal_checker"]
    controller_plugins: ["FollowPath", "BackupController"]

    # Enhanced progress checker
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.25  # Smaller for more sensitive checking
      movement_time_allowance: 5.0    # Shorter timeout

    # Multiple goal checkers for different scenarios
    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    precise_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.10  # Tighter tolerance for precision
      yaw_goal_tolerance: 0.10
      stateful: True

    # Main path following controller
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: False
      min_vel_x: 0.05          # Minimum forward speed to prevent stalling
      min_vel_y: 0.0
      max_vel_x: 0.8           # Higher speed for efficiency
      max_vel_y: 0.0
      max_vel_theta: 2.0       # Higher angular speed for better turning
      min_speed_xy: 0.05
      max_speed_xy: 0.8
      min_speed_theta: 0.1
      acc_lim_x: 3.0           # Higher acceleration for better responsiveness
      acc_lim_y: 0.0
      acc_lim_theta: 4.0       # Higher angular acceleration
      decel_lim_x: -3.0
      decel_lim_y: 0.0
      decel_lim_theta: -4.0
      vx_samples: 40           # More samples for better optimization
      vy_samples: 5
      vtheta_samples: 40
      sim_time: 2.0            # Longer simulation horizon
      linear_granularity: 0.025  # Finer granularity
      angular_granularity: 0.01
      transform_tolerance: 0.1
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
      restore_defaults: False
      publish_cost_grid_pc: True  # Enable for debugging
      use_dwb: True
      max_vel_obstacle: 2.0

    # Backup controller for emergency situations
    BackupController:
      plugin: "nav2_controller::BackUpController"
      min_vel_x: -0.2
      max_vel_x: -0.1
      acc_lim_x: 1.0
      decel_lim_x: -1.0
      sim_time: 1.0
      vx_samples: 10
      tolerance: 0.1
      threshold_to_rotate: 0.2
```

## Custom Path Execution Controller Implementation

### 1. Basic Pure Pursuit Controller

Create a custom pure pursuit path follower `pure_pursuit_controller.py`:

```python
#!/usr/bin/env python3
"""
Custom Pure Pursuit Path Execution Controller
"""
import math
from typing import List, Tuple
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # Parameters
        self.declare_parameter('lookahead_distance', 0.6)
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('min_linear_velocity', 0.1)
        self.declare_parameter('control_frequency', 20.0)

        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.min_linear_vel = self.get_parameter('min_linear_velocity').value
        self.control_freq = self.get_parameter('control_frequency').value

        # Current path and state
        self.current_path = []
        self.current_path_index = 0
        self.robot_pose = None
        self.path_received = False

        # TF buffer for pose transformation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers and subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.path_sub = self.create_subscription(
            Path, 'global_plan', self.path_callback, qos_profile)
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, qos_profile)

        # Timer for control loop
        self.control_timer = self.create_timer(1.0/self.control_freq, self.control_loop)

        # Safety variables
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')

        self.get_logger().info('Pure Pursuit Controller initialized')

    def path_callback(self, msg):
        """Callback for receiving path to follow"""
        self.current_path = msg.poses
        self.current_path_index = 0
        self.path_received = True
        self.get_logger().info(f'Received path with {len(self.current_path)} waypoints')

    def laser_callback(self, msg):
        """Callback for laser scan to detect obstacles"""
        # Simple obstacle detection - check minimum range in front
        front_ranges = msg.ranges[len(msg.ranges)//2-30:len(msg.ranges)//2+30]
        valid_ranges = [r for r in front_ranges if not (math.isnan(r) or math.isinf(r))]

        if valid_ranges:
            self.obstacle_distance = min(valid_ranges)
            self.obstacle_detected = self.obstacle_distance < 0.8  # 0.8m threshold
        else:
            self.obstacle_distance = float('inf')
            self.obstacle_detected = False

    def get_robot_pose(self):
        """Get current robot pose using TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())

            pose = PoseStamped()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.orientation = transform.transform.rotation

            return pose.pose
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform: {ex}')
            return None

    def find_lookahead_point(self):
        """Find the lookahead point on the path"""
        if not self.current_path or self.current_path_index >= len(self.current_path):
            return None

        robot_pose = self.get_robot_pose()
        if not robot_pose:
            return None

        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y

        # Find the closest point on the path
        closest_idx = self.current_path_index
        min_dist = float('inf')

        for i in range(self.current_path_index, len(self.current_path)):
            path_x = self.current_path[i].pose.position.x
            path_y = self.current_path[i].pose.position.y
            dist = math.sqrt((robot_x - path_x)**2 + (robot_y - path_y)**2)

            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Find the lookahead point beyond the closest point
        for i in range(closest_idx, len(self.current_path)):
            path_x = self.current_path[i].pose.position.x
            path_y = self.current_path[i].pose.position.y
            dist = math.sqrt((robot_x - path_x)**2 + (robot_y - path_y)**2)

            if dist >= self.lookahead_distance:
                self.current_path_index = i
                return (path_x, path_y)

        # If no point is far enough, return the last point
        if self.current_path:
            last_point = self.current_path[-1]
            return (last_point.pose.position.x, last_point.pose.position.y)

        return None

    def calculate_control(self, lookahead_point):
        """Calculate linear and angular velocities using pure pursuit"""
        robot_pose = self.get_robot_pose()
        if not robot_pose or not lookahead_point:
            return 0.0, 0.0

        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y
        robot_yaw = self.quaternion_to_yaw(robot_pose.orientation)

        lookahead_x, lookahead_y = lookahead_point

        # Calculate distance to lookahead point
        dist_to_lookahead = math.sqrt(
            (lookahead_x - robot_x)**2 + (lookahead_y - robot_y)**2)

        # Calculate angle to lookahead point
        angle_to_lookahead = math.atan2(lookahead_y - robot_y, lookahead_x - robot_x)
        angle_diff = self.normalize_angle(angle_to_lookahead - robot_yaw)

        # Pure pursuit formula
        curvature = 2 * math.sin(angle_diff) / dist_to_lookahead

        # Calculate velocities
        linear_vel = min(self.max_linear_vel, self.max_linear_vel * math.cos(angle_diff))
        angular_vel = linear_vel * curvature

        # Limit angular velocity
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))

        return linear_vel, angular_vel

    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def control_loop(self):
        """Main control loop"""
        if not self.path_received or not self.current_path:
            return

        # Check if path is completed
        if self.current_path_index >= len(self.current_path) - 1:
            self.stop_robot()
            self.get_logger().info('Path execution completed')
            return

        # Check for obstacles
        if self.obstacle_detected:
            self.get_logger().warn(f'Obstacle detected at {self.obstacle_distance:.2f}m, stopping')
            self.stop_robot()
            return

        # Find lookahead point
        lookahead_point = self.find_lookahead_point()
        if not lookahead_point:
            self.stop_robot()
            return

        # Calculate control commands
        linear_vel, angular_vel = self.calculate_control(lookahead_point)

        # Apply safety limits
        linear_vel = max(self.min_linear_vel, min(self.max_linear_vel, linear_vel))

        # Create and publish velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel

        self.cmd_vel_pub.publish(cmd_vel)

        # Log control information
        self.get_logger().debug(
            f'Linear: {linear_vel:.3f}, Angular: {angular_vel:.3f}, '
            f'Lookahead: ({lookahead_point[0]:.2f}, {lookahead_point[1]:.2f})')

    def stop_robot(self):
        """Stop the robot"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    def is_path_complete(self):
        """Check if path execution is complete"""
        if not self.current_path or self.current_path_index >= len(self.current_path):
            return True

        robot_pose = self.get_robot_pose()
        if not robot_pose:
            return False

        last_waypoint = self.current_path[-1]
        dist_to_goal = math.sqrt(
            (robot_pose.position.x - last_waypoint.pose.position.x)**2 +
            (robot_pose.position.y - last_waypoint.pose.position.y)**2)

        return dist_to_goal < 0.25  # 25cm tolerance


def main(args=None):
    rclpy.init(args=args)
    controller = PurePursuitController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Pure pursuit controller stopped by user')
    finally:
        controller.stop_robot()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Advanced Path Execution with Obstacle Avoidance

Create a more sophisticated path execution controller `advanced_path_controller.py`:

```python
#!/usr/bin/env python3
"""
Advanced Path Execution Controller with Obstacle Avoidance
"""
import math
import numpy as np
from typing import List, Tuple
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32
import time


class AdvancedPathController(Node):
    def __init__(self):
        super().__init__('advanced_path_controller')

        # Parameters
        self.declare_parameter('lookahead_distance', 0.8)
        self.declare_parameter('max_linear_velocity', 0.6)
        self.declare_parameter('max_angular_velocity', 1.5)
        self.declare_parameter('min_linear_velocity', 0.05)
        self.declare_parameter('control_frequency', 30.0)
        self.declare_parameter('obstacle_threshold', 0.8)
        self.declare_parameter('inflation_radius', 0.3)
        self.declare_parameter('path_smoothing', True)

        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.min_linear_vel = self.get_parameter('min_linear_velocity').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.path_smoothing = self.get_parameter('path_smoothing').value

        # Current state
        self.current_path = []
        self.smoothed_path = []
        self.current_path_index = 0
        self.path_received = False

        # Robot state
        self.robot_pose = None
        self.robot_velocity = 0.0
        self.robot_angular_velocity = 0.0

        # TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers and subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.path_sub = self.create_subscription(
            Path, 'global_plan', self.path_callback, qos_profile)
        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, qos_profile)
        self.odom_sub = self.create_subscription(
            Path, 'odom', self.odom_callback, qos_profile)  # Simplified - in real implementation use proper odometry

        # Timer for control loop
        self.control_timer = self.create_timer(1.0/self.control_freq, self.control_loop)

        # Safety and obstacle avoidance
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')
        self.local_path = []  # Local path with obstacle avoidance
        self.recovery_mode = False
        self.recovery_start_time = 0

        # Performance monitoring
        self.last_cmd_time = time.time()
        self.velocity_publisher = self.create_publisher(Float32, 'current_velocity', 1)

        self.get_logger().info('Advanced Path Controller initialized')

    def path_callback(self, msg):
        """Callback for receiving path to follow"""
        self.current_path = msg.poses
        self.current_path_index = 0

        # Smooth the path if enabled
        if self.path_smoothing:
            self.smoothed_path = self.smooth_path(self.current_path)
        else:
            self.smoothed_path = self.current_path

        self.path_received = True
        self.recovery_mode = False
        self.get_logger().info(f'Received path with {len(self.current_path)} waypoints')

    def laser_callback(self, msg):
        """Callback for laser scan to detect obstacles and create local path"""
        # Process laser scan for obstacle detection
        self.process_laser_scan(msg)

        # Create local path considering obstacles
        if self.path_received and self.current_path:
            self.local_path = self.create_local_path_with_obstacles(msg)

    def process_laser_scan(self, scan_msg):
        """Process laser scan to detect obstacles"""
        # Check for obstacles in front of robot
        front_ranges = scan_msg.ranges[len(scan_msg.ranges)//2-45:len(scan_msg.ranges)//2+45]
        valid_ranges = [r for r in front_ranges if not (math.isnan(r) or math.isinf(r) or r > 10.0)]

        if valid_ranges:
            self.obstacle_distance = min(valid_ranges)
            self.obstacle_detected = self.obstacle_distance < self.obstacle_threshold
        else:
            self.obstacle_distance = float('inf')
            self.obstacle_detected = False

    def create_local_path_with_obstacles(self, scan_msg):
        """Create a local path that avoids detected obstacles"""
        if not self.current_path or self.current_path_index >= len(self.current_path):
            return []

        robot_pose = self.get_robot_pose()
        if not robot_pose:
            return []

        # Get the next few waypoints from global path
        local_waypoints = []
        start_idx = min(self.current_path_index, len(self.current_path) - 1)

        for i in range(start_idx, min(start_idx + 10, len(self.current_path))):
            local_waypoints.append(self.current_path[i])

        # If obstacles detected, modify local path to avoid them
        if self.obstacle_detected:
            local_waypoints = self.avoid_obstacles_in_local_path(local_waypoints, scan_msg, robot_pose)

        return local_waypoints

    def avoid_obstacles_in_local_path(self, local_waypoints, scan_msg, robot_pose):
        """Modify local path to avoid obstacles"""
        if not local_waypoints:
            return []

        # Simple obstacle avoidance: find alternative points that avoid obstacles
        adjusted_waypoints = []

        for waypoint in local_waypoints:
            waypoint_x = waypoint.pose.position.x
            waypoint_y = waypoint.pose.position.y

            # Calculate range and bearing to waypoint
            dx = waypoint_x - robot_pose.position.x
            dy = waypoint_y - robot_pose.position.y
            distance_to_waypoint = math.sqrt(dx*dx + dy*dy)

            if distance_to_waypoint > 0:
                # Check if path to waypoint is clear
                angle_to_waypoint = math.atan2(dy, dx)

                # Check laser ranges in the direction of the waypoint
                angle_idx = int((angle_to_waypoint + math.pi) * len(scan_msg.ranges) / (2 * math.pi))
                angle_idx = max(0, min(len(scan_msg.ranges) - 1, angle_idx))

                if scan_msg.ranges[angle_idx] < self.obstacle_threshold:
                    # Obstacle detected in path, try to find alternative
                    adjusted_waypoint = self.find_alternative_waypoint(
                        waypoint, robot_pose, scan_msg)
                    adjusted_waypoints.append(adjusted_waypoint)
                else:
                    adjusted_waypoints.append(waypoint)
            else:
                adjusted_waypoints.append(waypoint)

        return adjusted_waypoints

    def find_alternative_waypoint(self, original_waypoint, robot_pose, scan_msg):
        """Find an alternative waypoint that avoids obstacles"""
        # Create a new waypoint slightly offset from the original
        original_x = original_waypoint.pose.position.x
        original_y = original_waypoint.pose.position.y

        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y

        # Calculate direction from robot to original waypoint
        dx = original_x - robot_x
        dy = original_y - robot_y
        distance = math.sqrt(dx*dx + dy*dy)

        if distance > 0:
            # Create offset perpendicular to the path
            perp_dx = -dy / distance  # Perpendicular direction
            perp_dy = dx / distance

            # Try offset to the left first
            offset_x = original_x + perp_dx * 0.3  # 30cm offset
            offset_y = original_y + perp_dy * 0.3

            # Check if this offset is clear of obstacles
            if self.is_path_clear(robot_x, robot_y, offset_x, offset_y, scan_msg):
                new_waypoint = PoseStamped()
                new_waypoint.pose.position.x = offset_x
                new_waypoint.pose.position.y = offset_y
                new_waypoint.pose.orientation = original_waypoint.pose.orientation
                return new_waypoint

        # If offset doesn't work, return original (controller will handle)
        return original_waypoint

    def is_path_clear(self, start_x, start_y, end_x, end_y, scan_msg):
        """Check if path between two points is clear of obstacles"""
        # Simplified check - in practice, use ray tracing through laser data
        distance = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)

        # Sample points along the path
        steps = max(1, int(distance / 0.1))  # Check every 10cm
        for i in range(steps + 1):
            t = i / steps
            check_x = start_x + t * (end_x - start_x)
            check_y = start_y + t * (end_y - start_y)

            # Convert to laser frame and check if clear
            # This is a simplified check - full implementation would require
            # transforming to laser frame and checking scan ranges
            pass

        return True  # Simplified - return true for now

    def get_robot_pose(self):
        """Get current robot pose using TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())

            pose = PoseStamped()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.orientation = transform.transform.rotation

            return pose.pose
        except TransformException as ex:
            self.get_logger().debug(f'Transform lookup failed: {ex}')
            return None

    def smooth_path(self, path):
        """Apply path smoothing using a simple averaging filter"""
        if len(path) < 3:
            return path

        smoothed_path = []
        path_array = [(p.pose.position.x, p.pose.position.y) for p in path]

        for i in range(len(path_array)):
            if i == 0 or i == len(path_array) - 1:
                # Keep start and end points
                smoothed_path.append(path[i])
            else:
                # Average with neighbors
                prev_x, prev_y = path_array[i-1]
                curr_x, curr_y = path_array[i]
                next_x, next_y = path_array[i+1]

                smooth_x = (prev_x + curr_x + next_x) / 3
                smooth_y = (prev_y + curr_y + next_y) / 3

                new_pose = PoseStamped()
                new_pose.pose.position.x = smooth_x
                new_pose.pose.position.y = smooth_y
                new_pose.pose.orientation = path[i].pose.orientation
                smoothed_path.append(new_pose)

        return smoothed_path

    def find_lookahead_point(self):
        """Find the lookahead point on the local path"""
        if not self.local_path and not self.smoothed_path:
            return None

        path_to_use = self.local_path if self.local_path else self.smoothed_path
        current_idx = 0

        robot_pose = self.get_robot_pose()
        if not robot_pose:
            return None

        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y

        # Find the closest point on the path
        closest_idx = 0
        min_dist = float('inf')

        for i in range(len(path_to_use)):
            path_x = path_to_use[i].pose.position.x
            path_y = path_to_use[i].pose.position.y
            dist = math.sqrt((robot_x - path_x)**2 + (robot_y - path_y)**2)

            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Find the lookahead point beyond the closest point
        for i in range(closest_idx, len(path_to_use)):
            path_x = path_to_use[i].pose.position.x
            path_y = path_to_use[i].pose.position.y
            dist = math.sqrt((robot_x - path_x)**2 + (robot_y - path_y)**2)

            if dist >= self.lookahead_distance:
                self.current_path_index = i
                return (path_x, path_y)

        # If no point is far enough, return the last point
        if path_to_use:
            last_point = path_to_use[-1]
            return (last_point.pose.position.x, last_point.pose.position.y)

        return None

    def calculate_control(self, lookahead_point):
        """Calculate linear and angular velocities"""
        robot_pose = self.get_robot_pose()
        if not robot_pose or not lookahead_point:
            return 0.0, 0.0

        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y
        robot_yaw = self.quaternion_to_yaw(robot_pose.orientation)

        lookahead_x, lookahead_y = lookahead_point

        # Calculate distance to lookahead point
        dist_to_lookahead = math.sqrt(
            (lookahead_x - robot_x)**2 + (lookahead_y - robot_y)**2)

        # Calculate angle to lookahead point
        angle_to_lookahead = math.atan2(lookahead_y - robot_y, lookahead_x - robot_x)
        angle_diff = self.normalize_angle(angle_to_lookahead - robot_yaw)

        # Pure pursuit with velocity scaling
        curvature = 2 * math.sin(angle_diff) / dist_to_lookahead if dist_to_lookahead > 0.01 else 0.0

        # Calculate velocities with obstacle awareness
        base_linear_vel = min(self.max_linear_vel, self.max_linear_vel * math.cos(angle_diff))

        # Reduce speed when close to obstacles
        speed_factor = min(1.0, self.obstacle_distance / self.obstacle_threshold)
        linear_vel = base_linear_vel * speed_factor if self.obstacle_detected else base_linear_vel

        # Ensure minimum speed to prevent stalling
        linear_vel = max(self.min_linear_vel, linear_vel) if linear_vel > 0 else linear_vel

        angular_vel = linear_vel * curvature

        # Limit angular velocity
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))

        return linear_vel, angular_vel

    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def control_loop(self):
        """Main control loop"""
        if not self.path_received:
            self.stop_robot()
            return

        # Check if path is completed
        if (self.current_path_index >= len(self.current_path) - 1 and
            len(self.current_path) > 0):
            self.stop_robot()
            self.get_logger().info('Path execution completed')
            return

        # Handle obstacle detection and recovery
        if self.obstacle_detected:
            if not self.recovery_mode:
                self.recovery_mode = True
                self.recovery_start_time = time.time()
                self.get_logger().warn(f'Obstacle detected at {self.obstacle_distance:.2f}m, initiating recovery')

            # In recovery mode, try to navigate around obstacle
            cmd_vel = self.handle_obstacle_recovery()
            self.cmd_vel_pub.publish(cmd_vel)
            return

        # Normal path following
        if self.recovery_mode:
            # Clear recovery mode after some time without obstacles
            if time.time() - self.recovery_start_time > 2.0:
                self.recovery_mode = False
                self.get_logger().info('Obstacle cleared, resuming path following')

        # Find lookahead point
        lookahead_point = self.find_lookahead_point()
        if not lookahead_point:
            self.stop_robot()
            return

        # Calculate control commands
        linear_vel, angular_vel = self.calculate_control(lookahead_point)

        # Create and publish velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel

        self.cmd_vel_pub.publish(cmd_vel)

        # Publish current velocity for monitoring
        vel_msg = Float32()
        vel_msg.data = linear_vel
        self.velocity_publisher.publish(vel_msg)

        # Log control information
        if time.time() - self.last_cmd_time > 1.0:  # Log every second
            self.get_logger().info(
                f'Linear: {linear_vel:.3f}, Angular: {angular_vel:.3f}, '
                f'Obstacles: {"Yes" if self.obstacle_detected else "No"}')
            self.last_cmd_time = time.time()

    def handle_obstacle_recovery(self):
        """Handle obstacle recovery behavior"""
        # Simple recovery: slow down and try to go around
        cmd_vel = Twist()

        if self.obstacle_distance < 0.3:  # Very close, backup
            cmd_vel.linear.x = -0.1
            cmd_vel.angular.z = 0.0
        elif self.obstacle_distance < 0.6:  # Medium distance, turn
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5  # Turn in place
        else:  # Far enough, slow down and continue
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = 0.2  # Gentle turn

        return cmd_vel

    def stop_robot(self):
        """Stop the robot"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    def is_path_complete(self):
        """Check if path execution is complete"""
        if not self.current_path or self.current_path_index >= len(self.current_path):
            return True

        robot_pose = self.get_robot_pose()
        if not robot_pose:
            return False

        last_waypoint = self.current_path[-1]
        dist_to_goal = math.sqrt(
            (robot_pose.position.x - last_waypoint.pose.position.x)**2 +
            (robot_pose.position.y - last_waypoint.pose.position.y)**2)

        return dist_to_goal < 0.25  # 25cm tolerance


def main(args=None):
    rclpy.init(args=args)
    controller = AdvancedPathController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Advanced path controller stopped by user')
    finally:
        controller.stop_robot()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Path Execution Integration with Isaac ROS

### 1. Perception-Enhanced Path Execution

```yaml
# Configuration for perception-enhanced path execution
controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 30.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.5
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath", "IsaacPerceptionController"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.25
      movement_time_allowance: 5.0

    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: False
      min_vel_x: 0.05
      min_vel_y: 0.0
      max_vel_x: 0.6
      max_vel_y: 0.0
      max_vel_theta: 1.5
      min_speed_xy: 0.05
      max_speed_xy: 0.6
      min_speed_theta: 0.1
      acc_lim_x: 2.0
      acc_lim_y: 0.0
      acc_lim_theta: 3.0
      decel_lim_x: -2.0
      decel_lim_y: 0.0
      decel_lim_theta: -3.0
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      sim_time: 1.5
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.1
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
      restore_defaults: False
      publish_cost_grid_pc: False
      use_dwb: True
      max_vel_obstacle: 1.5

    IsaacPerceptionController:
      plugin: "nav2_isaac_controller/IsaacPerceptionController"
      enabled: True
      perception_topic: "/segmentation/segmentation_map"
      dynamic_object_topic: "/dynamic_objects"
      obstacle_cost_multiplier: 2.0
      free_space_multiplier: 0.5
      perception_timeout: 1.0
      safety_buffer: 0.3
```

### 2. Visual-Inertial Path Execution

```yaml
# Configuration for visual-inertial enhanced path execution
controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 50.0  # Higher frequency for visual-inertial
    min_x_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "visual_inertial_progress_checker"
    goal_checker_plugins: ["visual_inertial_goal_checker"]
    controller_plugins: ["VisualInertialController"]

    visual_inertial_progress_checker:
      plugin: "nav2_controller::VisualInertialProgressChecker"
      required_movement_radius: 0.2
      movement_time_allowance: 3.0
      visual_confidence_threshold: 0.7

    visual_inertial_goal_checker:
      plugin: "nav2_controller::VisualInertialGoalChecker"
      xy_goal_tolerance: 0.15  # Tighter tolerance with visual feedback
      yaw_goal_tolerance: 0.15
      visual_alignment_tolerance: 0.2
      stateful: True

    VisualInertialController:
      plugin: "nav2_visual_inertial_controller/VisualInertialController"
      max_vel_x: 0.7
      max_vel_theta: 2.0
      visual_weight: 0.6
      inertial_weight: 0.4
      visual_inertial_fusion: true
      pose_update_frequency: 100.0  # High frequency for visual-inertial
      tracking_confidence_threshold: 0.6
```

## Performance Optimization

### 1. Controller Performance Tuning

```yaml
# Optimized controller configuration for performance
controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0  # Balance between performance and control quality
    min_x_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.5
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: False  # Disable for performance
      min_vel_x: 0.05
      min_vel_y: 0.0
      max_vel_x: 0.5
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.05
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 1.5  # Lower acceleration for smoother motion
      acc_lim_y: 0.0
      acc_lim_theta: 2.0
      decel_lim_x: -1.5
      decel_lim_y: 0.0
      decel_lim_theta: -2.0
      vx_samples: 15  # Reduce samples for performance
      vy_samples: 3
      vtheta_samples: 15
      sim_time: 1.0  # Shorter simulation time
      linear_granularity: 0.1  # Coarser granularity
      angular_granularity: 0.05
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
      restore_defaults: False
      publish_cost_grid_pc: False  # Disable for performance
      use_dwb: True
      max_vel_obstacle: 1.0
```

### 2. Memory and Computation Optimization

```yaml
# Memory-optimized controller configuration
controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 10.0  # Lower frequency to reduce CPU usage
    min_x_velocity_threshold: 0.01
    min_theta_velocity_threshold: 0.01
    failure_tolerance: 1.0  # Higher tolerance to reduce re-planning
    progress_checker_plugin: "simple_progress_checker"
    goal_checker_plugins: ["simple_goal_checker"]
    controller_plugins: ["SimpleController"]

    simple_progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 1.0  # Larger radius to reduce checks
      movement_time_allowance: 20.0

    simple_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.5  # Larger tolerance for performance
      yaw_goal_tolerance: 0.5
      stateful: False  # Disable stateful behavior

    SimpleController:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: False
      min_vel_x: 0.1
      max_vel_x: 0.3
      max_vel_theta: 0.5
      vx_samples: 10  # Minimal samples
      vtheta_samples: 10
      sim_time: 0.5  # Minimal simulation time
      linear_granularity: 0.2
      angular_granularity: 0.1
      transform_tolerance: 0.5
      xy_goal_tolerance: 0.5
      stateful: False
```

## Path Execution Quality Metrics

### 1. Path Following Performance Metrics

```python
#!/usr/bin/env python3
"""
Path Execution Quality Metrics
"""
import math
from typing import List
from geometry_msgs.msg import PoseStamped, Twist


class PathExecutionMetrics:
    def __init__(self):
        self.path_deviation_history = []
        self.velocity_profile = []
        self.execution_time = 0
        self.start_time = None

    def calculate_deviation(self, actual_pose: PoseStamped, path_poses: List[PoseStamped], current_idx: int):
        """Calculate deviation from planned path"""
        if not path_poses or current_idx >= len(path_poses):
            return float('inf')

        # Find closest point on path
        min_dist = float('inf')
        for i in range(current_idx, len(path_poses)):
            path_pose = path_poses[i]
            dist = math.sqrt(
                (actual_pose.pose.position.x - path_pose.pose.position.x)**2 +
                (actual_pose.pose.position.y - path_pose.pose.position.y)**2
            )
            min_dist = min(min_dist, dist)

        self.path_deviation_history.append(min_dist)
        return min_dist

    def calculate_smoothness(self, velocity_profile: List[Twist]):
        """Calculate path following smoothness"""
        if len(velocity_profile) < 2:
            return 0

        acceleration_changes = []
        for i in range(1, len(velocity_profile)):
            prev_vel = velocity_profile[i-1]
            curr_vel = velocity_profile[i]

            linear_acc_change = abs(curr_vel.linear.x - prev_vel.linear.x)
            angular_acc_change = abs(curr_vel.angular.z - prev_vel.angular.z)
            total_change = linear_acc_change + angular_acc_change
            acceleration_changes.append(total_change)

        if acceleration_changes:
            return sum(acceleration_changes) / len(acceleration_changes)
        return 0

    def calculate_efficiency(self, actual_distance: float, optimal_distance: float):
        """Calculate path efficiency"""
        if optimal_distance > 0:
            return optimal_distance / actual_distance if actual_distance > 0 else 0
        return 1.0  # Perfect efficiency if no optimal distance

    def get_comprehensive_metrics(self, path_poses: List[PoseStamped], start_pose: PoseStamped, end_pose: PoseStamped):
        """Get comprehensive path execution metrics"""
        # Calculate total path length
        total_path_length = 0
        for i in range(1, len(path_poses)):
            p1 = path_poses[i-1].pose.position
            p2 = path_poses[i].pose.position
            dist = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
            total_path_length += dist

        # Calculate actual travel distance
        actual_distance = math.sqrt(
            (end_pose.pose.position.x - start_pose.pose.position.x)**2 +
            (end_pose.pose.position.y - start_pose.pose.position.y)**2
        )

        # Calculate metrics
        avg_deviation = sum(self.path_deviation_history) / len(self.path_deviation_history) if self.path_deviation_history else float('inf')
        max_deviation = max(self.path_deviation_history) if self.path_deviation_history else float('inf')
        path_efficiency = self.calculate_efficiency(actual_distance, total_path_length)
        smoothness = self.calculate_smoothness(self.velocity_profile)

        return {
            'total_path_length': total_path_length,
            'actual_distance': actual_distance,
            'average_deviation': avg_deviation,
            'max_deviation': max_deviation,
            'path_efficiency': path_efficiency,
            'smoothness': smoothness,
            'execution_time': self.execution_time,
            'deviation_std': self.calculate_std(self.path_deviation_history)
        }

    def calculate_std(self, values):
        """Calculate standard deviation"""
        if len(values) < 2:
            return 0
        mean = sum(values) / len(values)
        variance = sum((x - mean) ** 2 for x in values) / len(values)
        return math.sqrt(variance)
```

## Troubleshooting Common Issues

### 1. Path Following Problems

#### Issue: Robot oscillates around path
**Symptoms**: Robot sways back and forth across the path
**Solutions**:
1. Increase lookahead distance:
   ```yaml
   lookahead_distance: 1.0  # Increase from 0.6
   ```
2. Reduce angular velocity limits:
   ```yaml
   max_vel_theta: 0.8  # Reduce from 1.0
   ```
3. Increase minimum linear velocity to prevent stalling

#### Issue: Robot stops frequently during path execution
**Symptoms**: Robot stops and starts repeatedly
**Solutions**:
1. Increase goal tolerances:
   ```yaml
   xy_goal_tolerance: 0.5  # Increase from 0.25
   ```
2. Check for costmap updates interfering with navigation
3. Verify sensor data quality and frequency

#### Issue: Robot cannot follow sharp turns
**Symptoms**: Robot cuts corners or fails to turn properly
**Solutions**:
1. Reduce linear velocity during turns:
   ```yaml
   max_vel_x: 0.3  # Reduce for sharp turns
   ```
2. Increase angular velocity limits:
   ```yaml
   max_vel_theta: 2.0  # Increase for sharper turns
   ```
3. Implement velocity scaling based on curvature

### 2. Performance Issues

#### Issue: High CPU usage during path execution
**Symptoms**: System becomes unresponsive during navigation
**Solutions**:
1. Reduce controller frequency:
   ```yaml
   controller_frequency: 10.0  # Reduce from 20.0
   ```
2. Simplify path following algorithm
3. Reduce number of velocity samples:
   ```yaml
   vx_samples: 10  # Reduce from 20
   vtheta_samples: 10  # Reduce from 20
   ```

## Best Practices

### 1. Controller Selection Guidelines
- **DWB (Dynamic Window Approach)**: Best for dynamic obstacle avoidance
- **MPC (Model Predictive Control)**: Best for precise control and constraints
- **Pure Pursuit**: Best for smooth, predictable paths
- **Trajectory Rollout**: Best for complex dynamics

### 2. Parameter Tuning Best Practices
- Start with conservative parameters
- Tune for your specific robot dynamics
- Test in simulation before real-world deployment
- Monitor performance metrics during operation
- Adjust parameters based on environment complexity

### 3. Safety Considerations
- Implement emergency stop mechanisms
- Set appropriate velocity limits
- Monitor obstacle detection continuously
- Implement recovery behaviors for failure cases

## Integration with Navigation System

### 1. Complete Navigation Configuration

```yaml
# Complete navigation stack configuration
planner_server:
  ros__parameters:
    expected_planner_frequency: 1.0
    use_sim_time: false
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: false

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    # ... (controller config from above)

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]
    # ... (behavior config)

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: false
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      # ... (costmap config)

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: false
      robot_radius: 0.22
      # ... (costmap config)
```

## Resources

- [Navigation2 Controller Documentation](https://navigation.ros.org/configuration/packages/configuring-controllers.html)
- [DWB Local Planner Guide](https://github.com/ros-planning/navigation2/blob/main/dwb_core/dwb_plugins/README.md)
- [Path Execution Tutorials](https://navigation.ros.org/tutorials/docs/path_execution_tutorial.html)

## Conclusion

This guide provides comprehensive coverage of path execution controllers for ROS 2 Navigation2. The path execution controller is a critical component that bridges the gap between global path planning and actual robot motion. Proper configuration and tuning are essential for achieving smooth, safe, and efficient navigation. The choice of controller algorithm and parameters depends on your specific robot dynamics, environment, and performance requirements.