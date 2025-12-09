---
sidebar_position: 3
---

# Isaac Navigation - Autonomous Navigation and Path Planning

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the navigation stack in ROS 2 and Isaac ecosystem
- Implement SLAM (Simultaneous Localization and Mapping) systems
- Create navigation pipelines for autonomous robot movement
- Integrate perception data with navigation for obstacle avoidance
- Configure and tune navigation parameters for optimal performance
- Implement recovery behaviors and safety mechanisms

## Prerequisites

Before starting this chapter, you should:
- Have ROS 2 Humble Hawksbill installed and configured
- Understand ROS 2 nodes, topics, and the navigation stack
- Completed the perception chapter for sensor integration
- Basic knowledge of SLAM concepts and path planning algorithms
- Experience with URDF modeling and simulation

## Conceptual Overview

**Navigation** in robotics refers to the ability of a robot to move autonomously from one location to another while avoiding obstacles and localizing itself within its environment. The Isaac Navigation system builds upon the ROS 2 Navigation2 stack with additional optimizations and features.

### Navigation Stack Components

The navigation stack typically consists of:

1. **Localization**: Determining the robot's position in the environment
2. **Mapping**: Creating a representation of the environment
3. **Path Planning**: Finding a route from start to goal
4. **Path Execution**: Following the planned path while avoiding obstacles
5. **Recovery**: Handling situations where the robot gets stuck

### Key Navigation Concepts

- **SLAM (Simultaneous Localization and Mapping)**: Building a map while localizing
- **AMCL (Adaptive Monte Carlo Localization)**: Probabilistic localization in known maps
- **Costmaps**: 2D/3D representations of the environment with obstacle information
- **Path Planners**: Global and local planners for route computation
- **Controllers**: Low-level systems to execute planned paths

### Isaac Navigation Features

- **GPU Acceleration**: Optimized algorithms for NVIDIA hardware
- **Deep Learning Integration**: ML-based perception and planning
- **Multi-Sensor Fusion**: Integration of various sensor types
- **Real-time Performance**: Optimized for real-time applications
- **Simulation Integration**: Seamless transition from sim to real hardware

## Hands-On Implementation

### Installing Navigation Packages

```bash
# Install Navigation2 packages
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox ros-humble-navigation-msgs
sudo apt install ros-humble-robot-localization ros-humble-interactive-markers

# For Isaac-specific navigation
cd ~/isaac_ros_ws/src
git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_occupancy_grid_localizer.git

# Build the workspace
cd ~/isaac_ros_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

### Basic Navigation Setup

#### Creating a Navigation Configuration Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python navigation_config
```

#### Costmap Configuration

**Create navigation_config/config/costmap_common_params.yaml:**

```yaml
# Common costmap parameters
map_topic: /map
track_unknown_space: true
use_dijkstra: true
use_grid_path: false

always_send_full_costmap: true

unknown_cost_value: 255
lethal_cost_threshold: 254

# Robot footprint
robot_radius: 0.3  # For circular robots
# footprint: [[x1, y1], [x2, y2], ...]  # For polygonal robots

# Obstacle parameters
obstacle_layer:
  enabled: true
  obstacle_range: 3.0
  raytrace_range: 4.0
  observation_sources: laser_scan
  laser_scan:
    topic: /scan
    sensor_frame: laser_link
    observation_persistence: 0.0
    max_obstacle_height: 2.0
    min_obstacle_height: 0.0
    inf_is_valid: true
    clearing: true
    marking: true

# Inflation layer
inflation_layer:
  enabled: true
  cost_scaling_factor: 3.0
  inflation_radius: 0.55
```

**Create navigation_config/config/local_costmap_params.yaml:**

```yaml
local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 10.0
  publish_frequency: 10.0
  static_map: false
  rolling_window: true
  width: 3
  height: 3
  resolution: 0.05
  origin_x: 0.0
  origin_y: 0.0
  plugins:
    - {name: obstacle_layer, type: "nav2_costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}
```

**Create navigation_config/config/global_costmap_params.yaml:**

```yaml
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  update_frequency: 1.0
  publish_frequency: 1.0
  static_map: true
  rolling_window: false
  plugins:
    - {name: static_layer, type: "nav2_costmap_2d::StaticLayer"}
    - {name: obstacle_layer, type: "nav2_costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}
```

**Create navigation_config/config/nav2_params.yaml:**

```yaml
amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

amcl_map_client:
  ros__parameters:
    use_sim_time: False

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: False

bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_nav_through_poses_bt_xml: nav2_bt_navigator/nav_through_poses_w_replanning_and_recovery.xml
    default_nav_to_pose_bt_xml: nav2_bt_navigator/nav_to_pose_w_replanning_and_recovery.xml
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_smooth_path_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_assisted_teleop_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_drive_on_heading_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node
    - nav2_spin_cancel_bt_node
    - nav2_back_up_cancel_bt_node
    - nav2_assisted_teleop_cancel_bt_node
    - nav2_drive_on_heading_cancel_bt_node

bt_navigator_rclcpp_node:
  ros__parameters:
    use_sim_time: False

controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"] # "precise_goal_checker"
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
      plugin: "nav2_rotation_shim::RotationShimController"
      rotational_scaler: 1.5
      velocity_deadband: 0.05
      velocity_scale_to_min: 0.25
      velocity_scaling_type: 0 # 0: Disabled, 1: Time, 2: Distance
      min_approach_velocity: 0.05
      max_allowed_time_to_collision: 1.0
      carrot_planner:
        plugin: "nav2_navfn_planner::NavfnPlanner"
        tolerance: 0.5
        use_astar: false
        allow_unknown: true

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: False
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["obstacle_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: False
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True

map_server:
  ros__parameters:
    use_sim_time: False
    yaml_filename: "turtlebot3_world.yaml"

map_saver:
  ros__parameters:
    use_sim_time: False
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: False
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

smoother_server:
  ros__parameters:
    use_sim_time: False
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 0.01
      max_its: 1000
      do_refinement: True

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
      spin_dist: 1.57
    backup:
      plugin: "nav2_behaviors::BackUp"
      backup_dist: 0.15
      backup_speed: 0.025
    wait:
      plugin: "nav2_behaviors::Wait"
      wait_duration: 1.0

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      wait_time: 1
```

### Creating Navigation Launch Files

**Create navigation_config/launch/navigation_launch.py:**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('navigation_config'),
            'config',
            'nav2_params.yaml'
        ]),
        description='Full path to params file for navigation nodes'
    )

    # Navigation server
    navigation_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('/cmd_vel', '/cmd_vel_nav')]
    )

    # Planner server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('~/global_costmap/costmap', 'global_costmap/costmap'),
                   ('~/global_costmap/costmap_updates', 'global_costmap/costmap_updates'),
                   ('~/local_costmap/costmap', 'local_costmap/costmap'),
                   ('~/local_costmap/costmap_updates', 'local_costmap/costmap_updates')]
    )

    # Recoveries server
    recoveries_server = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # BT navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('~/local_costmap/costmap', 'local_costmap/costmap'),
                   ('~/local_costmap/costmap_updates', 'local_costmap/costmap_updates'),
                   ('~/global_costmap/costmap', 'global_costmap/costmap'),
                   ('~/global_costmap/costmap_updates', 'global_costmap/costmap_updates')]
    )

    # Lifecycle manager for navigation
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                   {'autostart': True},
                   {'node_names': ['controller_server',
                                  'planner_server',
                                  'recoveries_server',
                                  'bt_navigator']}]
    )

    return LaunchDescription([
        use_sim_time,
        params_file,
        navigation_server,
        planner_server,
        recoveries_server,
        bt_navigator,
        lifecycle_manager
    ])
```

### Creating a Navigation Controller Node

```python
#!/usr/bin/env python3

"""
Navigation controller node that sends goals to the navigation system.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import random


class NavigationController(Node):
    """
    Node to control navigation by sending goals to the navigation system.
    """

    def __init__(self):
        super().__init__('navigation_controller')

        # Create action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Create publisher for status
        self.status_pub = self.create_publisher(String, 'navigation_status', 10)

        # Wait for navigation server
        self.get_logger().info('Waiting for navigation server...')
        self.nav_client.wait_for_server()

        # Timer to send goals periodically
        self.timer = self.create_timer(10.0, self.send_goal)

        # Goal counter
        self.goal_count = 0

        self.get_logger().info('Navigation controller initialized')

    def send_goal(self):
        """Send a random goal to the navigation system."""
        # Wait for server
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Navigation server not available')
            return

        # Create a goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Set a random goal position (in a real scenario, these would be meaningful coordinates)
        goal_msg.pose.pose.position.x = random.uniform(-5.0, 5.0)
        goal_msg.pose.pose.position.y = random.uniform(-5.0, 5.0)
        goal_msg.pose.pose.position.z = 0.0

        # Set orientation (pointing along positive x-axis)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        # Send the goal
        self.get_logger().info(f'Sending navigation goal #{self.goal_count}: '
                              f'({goal_msg.pose.pose.position.x:.2f}, {goal_msg.pose.pose.position.y:.2f})')

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(self.goal_response_callback)
        self.goal_count += 1

    def goal_response_callback(self, future):
        """Handle the response from the navigation server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle the result from the navigation server."""
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f'Navigation completed with status: {status}')

        status_msg = String()
        if status == 3:  # SUCCEEDED
            status_msg.data = 'Navigation succeeded'
        elif status == 4:  # CANCELED
            status_msg.data = 'Navigation canceled'
        else:
            status_msg.data = f'Navigation failed with status: {status}'

        self.status_pub.publish(status_msg)

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the navigation server."""
        feedback = feedback_msg.feedback
        # In a real implementation, you might process feedback here
        self.get_logger().debug('Received feedback')


def main(args=None):
    rclpy.init(args=args)
    node = NavigationController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Navigation controller stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Creating a SLAM Integration Node

```python
#!/usr/bin/env python3

"""
SLAM integration node that combines mapping with navigation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster
import numpy as np


class SLAMIntegrationNode(Node):
    """
    Node to integrate SLAM with navigation.
    """

    def __init__(self):
        super().__init__('slam_integration_node')

        # Create subscribers
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

        # Create publishers
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            10
        )

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # Initialize variables
        self.current_pose = None
        self.map_resolution = 0.05  # meters per cell
        self.map_width = 400  # cells
        self.map_height = 400  # cells
        self.map_origin_x = -10.0  # meters
        self.map_origin_y = -10.0  # meters

        # Create empty map
        self.map_data = np.full(self.map_width * self.map_height, -1, dtype=np.int8)  # -1 = unknown

        self.get_logger().info('SLAM integration node initialized')

    def scan_callback(self, msg):
        """Process laser scan data for mapping."""
        if self.current_pose is None:
            return

        # Convert laser scan to map coordinates and update map
        # This is a simplified approach - real SLAM would be more complex
        robot_x = self.current_pose.pose.pose.position.x
        robot_y = self.current_pose.pose.pose.position.y
        robot_yaw = self.get_yaw_from_quaternion(self.current_pose.pose.pose.orientation)

        # Process each laser reading
        for i, range_val in enumerate(msg.ranges):
            if not (msg.range_min < range_val < msg.range_max):
                continue  # Skip invalid ranges

            # Calculate angle of this reading
            angle = msg.angle_min + i * msg.angle_increment + robot_yaw

            # Calculate end point of this laser beam
            end_x = robot_x + range_val * np.cos(angle)
            end_y = robot_y + range_val * np.sin(angle)

            # Convert to map coordinates
            map_x = int((end_x - self.map_origin_x) / self.map_resolution)
            map_y = int((end_y - self.map_origin_y) / self.map_resolution)

            # Check bounds
            if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                # Mark as occupied (value 100)
                idx = map_y * self.map_width + map_x
                self.map_data[idx] = 100

    def odom_callback(self, msg):
        """Process odometry data."""
        self.current_pose = msg
        # Could also update robot pose in map frame here

    def get_yaw_from_quaternion(self, quat):
        """Extract yaw from quaternion."""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def publish_map(self):
        """Publish the occupancy grid map."""
        if self.map_data is None:
            return

        # Create occupancy grid message
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'

        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.x = 0.0
        map_msg.info.origin.orientation.y = 0.0
        map_msg.info.origin.orientation.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Convert numpy array to list for message
        map_msg.data = self.map_data.tolist()

        # Publish map
        self.map_pub.publish(map_msg)

        self.get_logger().info(f'Published map with {len(self.map_data)} cells')


def main(args=None):
    rclpy.init(args=args)
    node = SLAMIntegrationNode()

    # Timer to publish map periodically
    node.create_timer(2.0, node.publish_map)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('SLAM integration node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Isaac-Specific Navigation Features

#### Visual SLAM Integration

Isaac provides advanced visual SLAM capabilities:

```python
#!/usr/bin/env python3

"""
Isaac Visual SLAM integration node.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np


class IsaacVisualSLAMNode(Node):
    """
    Node to demonstrate Isaac Visual SLAM concepts.
    """

    def __init__(self):
        super().__init__('isaac_visual_slam_node')

        # Create subscribers for stereo camera or RGB-D
        self.left_image_sub = self.create_subscription(
            Image,
            '/camera/left/image_rect_color',
            self.left_image_callback,
            10
        )

        self.right_image_sub = self.create_subscription(
            Image,
            '/camera/right/image_rect_color',
            self.right_image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/left/camera_info',
            self.camera_info_callback,
            10
        )

        # Create publisher for visual odometry
        self.odom_pub = self.create_publisher(
            Odometry,
            '/visual_odom',
            10
        )

        # Initialize variables
        self.bridge = CvBridge()
        self.left_image = None
        self.right_image = None
        self.camera_info = None
        self.previous_pose = None

        self.get_logger().info('Isaac Visual SLAM node initialized')

    def left_image_callback(self, msg):
        """Process left camera image."""
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_stereo_pair()
        except Exception as e:
            self.get_logger().error(f'Error processing left image: {e}')

    def right_image_callback(self, msg):
        """Process right camera image."""
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_stereo_pair()
        except Exception as e:
            self.get_logger().error(f'Error processing right image: {e}')

    def camera_info_callback(self, msg):
        """Process camera information."""
        self.camera_info = msg

    def process_stereo_pair(self):
        """Process stereo images for visual SLAM."""
        if self.left_image is None or self.right_image is None or self.camera_info is None:
            return

        # In a real implementation, you would use Isaac's visual SLAM pipeline
        # This is a simplified example showing the concept

        # Compute stereo disparity (simplified)
        # In practice, Isaac uses optimized GPU-accelerated algorithms
        gray_left = cv2.cvtColor(self.left_image, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(self.right_image, cv2.COLOR_BGR2GRAY)

        # Use OpenCV's stereo matcher as an example
        stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=16*10,  # Must be divisible by 16
            blockSize=5,
            P1=8*3*5**2,
            P2=32*3*5**2,
            disp12MaxDiff=1,
            uniquenessRatio=15,
            speckleWindowSize=0,
            speckleRange=2,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )

        disparity = stereo.compute(gray_left, gray_right).astype(np.float32) / 16.0

        # Convert disparity to 3D points and estimate motion
        # This would integrate with the navigation system in a real implementation

        # Publish odometry estimate
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set pose (simplified - would come from actual SLAM algorithm)
        if self.previous_pose is None:
            # Initial pose
            odom_msg.pose.pose.position.x = 0.0
            odom_msg.pose.pose.position.y = 0.0
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.w = 1.0
            self.previous_pose = odom_msg.pose.pose
        else:
            # Update based on estimated motion
            odom_msg.pose.pose.position.x = self.previous_pose.position.x + 0.01
            odom_msg.pose.pose.position.y = self.previous_pose.position.y + 0.01
            odom_msg.pose.pose.orientation.w = 1.0
            self.previous_pose = odom_msg.pose.pose

        # Set velocity (simplified)
        odom_msg.twist.twist.linear.x = 0.1
        odom_msg.twist.twist.angular.z = 0.05

        self.odom_pub.publish(odom_msg)

        self.get_logger().info('Processed stereo pair for visual SLAM')


def main(args=None):
    rclpy.init(args=args)
    node = IsaacVisualSLAMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Visual SLAM node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Testing & Verification

### Running Navigation System

1. **Start your robot with sensors:**
```bash
# This could be a real robot or simulation
# Make sure you have /scan, /odom, /tf topics available
```

2. **Launch navigation:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch navigation_config navigation_launch.py
```

3. **Send navigation goals:**
```bash
# Using navigation controller node
ros2 run navigation_config navigation_controller

# Or manually with RViz2
ros2 run rviz2 rviz2
# Then use the "Navigation 2" plugin to send goals
```

4. **Monitor navigation performance:**
```bash
# Check navigation status
ros2 topic echo /navigation_status

# Monitor costmaps
ros2 run rviz2 rviz2  # Add costmap displays

# Check robot position
ros2 topic echo /amcl_pose
```

### Useful Navigation Commands

- **Check navigation topics:**
```bash
ros2 topic list | grep -E "(nav|costmap|localization)"
```

- **Send a goal programmatically:**
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

- **Reset navigation:**
```bash
ros2 service call /global_costmap/clear_entirely_global_costmap std_srvs/srv/Empty
```

- **Monitor navigation performance:**
```bash
# Use navigation tools
ros2 run nav2_util navigation_metrics
```

### Navigation Tuning Parameters

Navigation performance can be tuned by adjusting parameters in the config files:

- **Costmap resolution**: Higher resolution = more precise but slower
- **Inflation radius**: Larger radius = safer but more conservative paths
- **Controller frequency**: Higher frequency = more responsive but more CPU usage
- **Goal tolerances**: Smaller values = more precise goal achievement

## Common Issues

### Issue: Robot oscillates or cannot reach goal
**Solution**:
- Check costmap inflation parameters
- Verify robot footprint is correctly configured
- Adjust controller parameters (XY goal tolerance, yaw goal tolerance)
- Check that the goal is in a navigable area

### Issue: Navigation fails with "No valid path found"
**Solution**:
- Verify map quality and resolution
- Check that obstacles are properly detected
- Ensure global planner can find a path
- Verify robot can physically navigate to the goal

### Issue: Robot gets stuck in local minima
**Solution**:
- Improve local planner parameters
- Add recovery behaviors
- Use more sophisticated path planning algorithms
- Check for proper obstacle inflation

### Issue: Localization drifts during navigation
**Solution**:
- Improve sensor quality and calibration
- Use more robust localization methods
- Add landmark-based relocalization
- Verify odometry quality

## Key Takeaways

- Navigation combines localization, mapping, path planning, and control
- Costmaps are crucial for obstacle avoidance and path planning
- Parameter tuning is essential for optimal navigation performance
- Isaac provides GPU-accelerated navigation algorithms
- Integration with perception systems enables robust autonomous navigation
- Recovery behaviors are important for handling edge cases
- Simulation testing is crucial before real-world deployment

## Next Steps

In the next chapter, you'll learn about Vision-Language-Action (VLA) systems, which integrate computer vision, natural language processing, and robotic action for advanced human-robot interaction.