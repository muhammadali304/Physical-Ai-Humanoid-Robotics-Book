# Perception Pipeline Integration with Navigation System

## Overview

This guide provides comprehensive instructions for integrating Isaac ROS perception pipelines with Navigation2. The integration enables robots to use visual and sensor data for enhanced navigation, including dynamic obstacle detection, semantic mapping, and visual-inertial odometry for improved localization.

## Understanding Perception-Navigation Integration

### Key Integration Points

1. **Sensor Data Integration**: Feeding perception outputs to costmaps
2. **Localization Enhancement**: Using visual data to improve pose estimation
3. **Dynamic Obstacle Detection**: Real-time detection and avoidance of moving objects
4. **Semantic Navigation**: Using object recognition for intelligent navigation
5. **Visual-Inertial Odometry**: Enhanced motion estimation using visual data

### Benefits of Integration

- Improved obstacle detection and classification
- Enhanced navigation in dynamic environments
- Better localization accuracy
- Semantic-aware path planning
- Robust operation in challenging conditions

## Architecture Overview

### 1. High-Level Integration Architecture

```
Isaac ROS Perception Pipeline
├── Visual SLAM → Pose/Odometry → AMCL/Localization
├── Stereo Processing → Depth → Costmap Obstacle Layer
├── Semantic Segmentation → Object Classification → Semantic Costmap
├── Object Detection → Dynamic Objects → Dynamic Obstacle Layer
└── IMU Integration → Visual-Inertial Odometry → Controller

Navigation2 Stack
├── Global Planner (uses semantic map)
├── Local Planner (uses perception-enhanced costmaps)
├── Controller (uses enhanced localization)
└── Behavior Trees (uses semantic information)
```

## Integration Implementation

### 1. Perception-Enhanced Costmap Configuration

Create a configuration file `perception_costmap_config.yaml`:

```yaml
# Perception-enhanced costmap configuration
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 0.5
      publish_frequency: 0.5
      transform_tolerance: 1.0
      use_sim_time: false

      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.22
      resolution: 0.05

      plugins: [
        "static_layer",
        "obstacle_layer",
        "isaac_perception_layer",
        "semantic_layer",
        "inflation_layer"
      ]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
        transform_tolerance: 1.0

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
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
          obstacle_max_range: 4.0
          obstacle_min_range: 0.0

      # Isaac ROS perception layer
      isaac_perception_layer:
        plugin: "nav2_isaac_perception_layer/IsaacPerceptionLayer"
        enabled: True
        observation_sources: stereo_depth segmentation
        stereo_depth:
          topic: /stereo/depth/disparity
          sensor_frame: stereo_camera_link
          data_type: "Disparity"
          clearing: True
          marking: True
          obstacle_range: 3.0
          raytrace_range: 4.0
        segmentation:
          topic: /segmentation/segmentation_map
          sensor_frame: camera_link
          data_type: "Image"
          clearing: False
          marking: True
          obstacle_value: 254
          free_space_value: 0

      # Semantic layer for object classification
      semantic_layer:
        plugin: "nav2_semantic_costmap_layer/SemanticLayer"
        enabled: True
        observation_sources: semantic_input
        semantic_input:
          topic: /semantic_segmentation/output
          sensor_frame: camera_frame
          data_type: "Image"
          clearing: False
          marking: True
          class_mappings:
            - {class_id: 0, class_name: "free_space", cost: 0, is_obstacle: false}
            - {class_id: 1, class_name: "wall", cost: 254, is_obstacle: true}
            - {class_id: 2, class_name: "person", cost: 200, is_obstacle: true}
            - {class_id: 3, class_name: "furniture", cost: 254, is_obstacle: true}
            - {class_id: 4, class_name: "plant", cost: 100, is_obstacle: true}
            - {class_id: 5, class_name: "clutter", cost: 150, is_obstacle: true}

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 5.0
        inflation_radius: 0.8
        inflate_unknown: False

  global_costmap_client:
    ros__parameters:
      use_sim_time: false
  global_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: false

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 5.0
      transform_tolerance: 0.2
      use_sim_time: false

      global_frame: odom
      robot_base_frame: base_link
      robot_radius: 0.22
      resolution: 0.025

      rolling_window: true
      width: 6
      height: 6
      origin_x: -3.0
      origin_y: -3.0

      plugins: [
        "voxel_layer",
        "isaac_perception_layer",
        "dynamic_object_layer",
        "inflation_layer"
      ]

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
          obstacle_max_range: 4.0
          obstacle_min_range: 0.0

      # Isaac ROS perception layer for local costmap
      isaac_perception_layer:
        plugin: "nav2_isaac_perception_layer/IsaacPerceptionLayer"
        enabled: True
        observation_sources: stereo_depth segmentation
        stereo_depth:
          topic: /stereo/depth/disparity
          sensor_frame: stereo_camera_link
          data_type: "Disparity"
          clearing: True
          marking: True
        segmentation:
          topic: /segmentation/segmentation_map
          sensor_frame: camera_link
          data_type: "Image"
          clearing: False
          marking: True

      # Dynamic object layer for moving obstacles
      dynamic_object_layer:
        plugin: "nav2_dynamic_obstacles_layer/DynamicObstaclesLayer"
        enabled: True
        observation_sources: dynamic_objects
        dynamic_objects:
          topic: /dynamic_objects
          sensor_frame: base_link
          data_type: "PointCloud2"
          clearing: True
          marking: True
          obstacle_value: 254
          free_space_value: 0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 8.0
        inflation_radius: 0.6
        inflate_unknown: False

  local_costmap_client:
    ros__parameters:
      use_sim_time: false
  local_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: false
```

### 2. Perception-Enhanced Localization Configuration

Create a localization configuration file `perception_localization.yaml`:

```yaml
# Perception-enhanced localization configuration
amcl:
  ros__parameters:
    use_sim_time: false
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

# Robot localization for visual-inertial fusion
robot_localization:
  ros__parameters:
    # Visual SLAM as primary pose source
    pose0: /visual_slam/odometry
    pose0_config: [true, true, false,    # x, y, no z
                   false, false, true,   # no roll, pitch, yes yaw
                   false, false, false]  # no linear/angular velocity
    pose0_differential: false
    pose0_relative: false

    # Wheel odometry for local motion
    twist0: /wheel/odometry
    twist0_config: [false, false, false,  # no position
                    false, false, false,  # no orientation
                    true, true, true]     # linear velocity x,y,z
    twist0_differential: false
    twist0_relative: false

    # IMU for orientation
    imu0: /imu/data
    imu0_config: [false, false, false,    # no position
                  true, true, true,       # orientation x,y,z
                  false, false, false]    # no angular velocity/linear acceleration
    imu0_differential: false
    imu0_relative: false
    imu0_remove_gravitational_acceleration: true

    # Fusion parameters
    frequency: 50.0
    sensor_timeout: 0.1
    two_d_mode: true
    transform_time_offset: 0.0
    transform_timeout: 0.0
    print_diagnostics: false

    # Process noise
    process_noise_covariance: [0.05, 0.0,    0.0,    0.0,    0.0,    0.0,
                              0.0,    0.05, 0.0,    0.0,    0.0,    0.0,
                              0.0,    0.0,    0.06, 0.0,    0.0,    0.0,
                              0.0,    0.0,    0.0,    0.03, 0.0,    0.0,
                              0.0,    0.0,    0.0,    0.0,    0.03, 0.0,
                              0.0,    0.0,    0.0,    0.0,    0.0,    0.06]

    # Initial estimate covariance
    initial_estimate_covariance: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.05, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.05, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.05]
```

### 3. Perception-Enhanced Controller Configuration

Create a controller configuration with perception feedback:

```yaml
controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 30.0  # Higher for perception integration
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "perception_progress_checker"
    goal_checker_plugins: ["perception_goal_checker"]
    controller_plugins: ["PerceptionAwareController", "BackupController"]

    # Perception-aware progress checker
    perception_progress_checker:
      plugin: "nav2_controller::PerceptionProgressChecker"
      required_movement_radius: 0.3
      movement_time_allowance: 5.0
      visual_confidence_threshold: 0.6

    # Perception-aware goal checker
    perception_goal_checker:
      plugin: "nav2_controller::PerceptionGoalChecker"
      xy_goal_tolerance: 0.2
      yaw_goal_tolerance: 0.2
      stateful: True
      visual_alignment_tolerance: 0.3

    # Main perception-aware controller
    PerceptionAwareController:
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
      vx_samples: 25
      vy_samples: 5
      vtheta_samples: 25
      sim_time: 1.8
      linear_granularity: 0.04
      angular_granularity: 0.02
      transform_tolerance: 0.1
      xy_goal_tolerance: 0.2
      yaw_goal_tolerance: 0.2
      stateful: True
      restore_defaults: False
      publish_cost_grid_pc: False
      use_dwb: True
      max_vel_obstacle: 1.8
      # Perception-specific parameters
      perception_weight: 0.7
      visual_inertial_fusion: true
      dynamic_object_aware: true

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

## Custom Integration Nodes

### 1. Perception-Navigation Bridge Node

Create a bridge node that connects perception outputs to navigation inputs:

```python
#!/usr/bin/env python3
"""
Perception-Navigation Integration Bridge
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
import numpy as np
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class PerceptionNavigationBridge(Node):
    def __init__(self):
        super().__init__('perception_navigation_bridge')

        # Parameters
        self.declare_parameter('perception_timeout', 1.0)
        self.declare_parameter('dynamic_object_threshold', 0.5)
        self.declare_parameter('semantic_confidence_threshold', 0.7)

        self.perception_timeout = self.get_parameter('perception_timeout').value
        self.dynamic_threshold = self.get_parameter('dynamic_object_threshold').value
        self.confidence_threshold = self.get_parameter('semantic_confidence_threshold').value

        # TF buffer for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Perception input topics
        self.semantic_sub = self.create_subscription(
            Image, '/segmentation/segmentation_map', self.semantic_callback, 10)
        self.dynamic_obj_sub = self.create_subscription(
            MarkerArray, '/dynamic_objects', self.dynamic_objects_callback, 10)
        self.stereo_depth_sub = self.create_subscription(
            Image, '/stereo/depth/disparity', self.stereo_depth_callback, 10)

        # Navigation output topics
        self.dynamic_costmap_pub = self.create_publisher(
            PointCloud2, '/local_costmap/dynamic_layer/clearing_endpoints', 10)
        self.semantic_costmap_pub = self.create_publisher(
            Image, '/semantic_costmap_input', 10)
        self.perception_status_pub = self.create_publisher(
            Header, '/perception_navigation_status', 10)

        # Internal state
        self.last_perception_time = self.get_clock().now()
        self.dynamic_objects = []
        self.semantic_map = None
        self.stereo_depth = None

        # Timer for periodic processing
        self.process_timer = self.create_timer(0.1, self.process_perception_data)

        self.get_logger().info('Perception-Navigation Bridge initialized')

    def semantic_callback(self, msg):
        """Process semantic segmentation data"""
        self.semantic_map = msg
        self.last_perception_time = self.get_clock().now()
        self.get_logger().debug('Received semantic segmentation data')

    def dynamic_objects_callback(self, msg):
        """Process dynamic object detections"""
        self.dynamic_objects = msg.markers
        self.last_perception_time = self.get_clock().now()
        self.get_logger().debug(f'Received {len(msg.markers)} dynamic objects')

    def stereo_depth_callback(self, msg):
        """Process stereo depth data"""
        self.stereo_depth = msg
        self.last_perception_time = self.get_clock().now()
        self.get_logger().debug('Received stereo depth data')

    def process_perception_data(self):
        """Process perception data and publish to navigation system"""
        current_time = self.get_clock().now()

        # Check if perception data is still valid
        if (current_time - self.last_perception_time).nanoseconds / 1e9 > self.perception_timeout:
            self.get_logger().warn('Perception data timeout, navigation may be degraded')
            return

        # Process dynamic objects for costmap
        if self.dynamic_objects:
            self.publish_dynamic_objects_to_costmap()

        # Process semantic data for semantic costmap
        if self.semantic_map:
            self.publish_semantic_to_costmap()

        # Process stereo depth for obstacle layer
        if self.stereo_depth:
            self.publish_depth_to_costmap()

        # Publish status
        status_msg = Header()
        status_msg.stamp = current_time.to_msg()
        status_msg.frame_id = "perception_active"
        self.perception_status_pub.publish(status_msg)

    def publish_dynamic_objects_to_costmap(self):
        """Convert dynamic objects to costmap-compatible format"""
        # Create point cloud from dynamic objects
        points = []
        for obj in self.dynamic_objects:
            if obj.type == 1:  # Sphere type
                # Check if object is moving (dynamic)
                if self.is_moving_object(obj):
                    points.append([obj.pose.position.x, obj.pose.position.y, obj.pose.position.z])

        if points:
            # Convert to PointCloud2 message
            pc2_msg = self.create_pointcloud2(points, "odom")
            self.dynamic_costmap_pub.publish(pc2_msg)

    def is_moving_object(self, marker):
        """Determine if a detected object is moving"""
        # In a real implementation, this would check object velocity
        # For now, we'll assume all detected objects are potentially dynamic
        return True

    def publish_semantic_to_costmap(self):
        """Publish semantic data to semantic costmap layer"""
        # In a real implementation, this would convert semantic segmentation
        # to a format compatible with semantic costmap layer
        # For now, we just republish the semantic map
        self.semantic_costmap_pub.publish(self.semantic_map)

    def publish_depth_to_costmap(self):
        """Convert depth data to costmap-compatible format"""
        # Process depth image and convert to obstacle points
        # This would typically involve:
        # 1. Converting disparity to depth
        # 2. Transforming to costmap frame
        # 3. Creating obstacle points
        pass

    def create_pointcloud2(self, points, frame_id):
        """Create a PointCloud2 message from a list of points"""
        from sensor_msgs.msg import PointCloud2, PointField
        import struct

        # Create PointCloud2 message
        pc2_msg = PointCloud2()
        pc2_msg.header = Header()
        pc2_msg.header.stamp = self.get_clock().now().to_msg()
        pc2_msg.header.frame_id = frame_id
        pc2_msg.height = 1
        pc2_msg.width = len(points)
        pc2_msg.is_dense = False
        pc2_msg.is_bigendian = False

        # Define point fields
        pc2_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        pc2_msg.point_step = 12  # 3 floats * 4 bytes
        pc2_msg.row_step = pc2_msg.point_step * pc2_msg.width

        # Pack the data
        data = []
        for point in points:
            data.append(struct.pack('fff', point[0], point[1], point[2]))

        pc2_msg.data = b''.join(data)
        return pc2_msg


def main(args=None):
    rclpy.init(args=args)
    bridge = PerceptionNavigationBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        bridge.get_logger().info('Perception-Navigation Bridge stopped by user')
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Semantic Navigation Node

Create a node that uses semantic information for intelligent navigation:

```python
#!/usr/bin/env python3
"""
Semantic Navigation Node
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String, Int32
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np


class SemanticNavigationNode(Node):
    def __init__(self):
        super().__init__('semantic_navigation_node')

        # Parameters
        self.declare_parameter('semantic_class_weights', [0.1, 10.0, 8.0, 10.0, 5.0, 6.0])
        self.declare_parameter('navigation_priority_threshold', 0.8)

        self.class_weights = self.get_parameter('semantic_class_weights').value
        self.priority_threshold = self.get_parameter('navigation_priority_threshold').value

        # Semantic class mapping
        self.semantic_classes = {
            0: "free_space",
            1: "wall",
            2: "person",
            3: "furniture",
            4: "plant",
            5: "clutter"
        }

        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscriptions
        self.semantic_sub = self.create_subscription(
            Image, '/segmentation/segmentation_map', self.semantic_callback, 10)
        self.dynamic_objects_sub = self.create_subscription(
            MarkerArray, '/dynamic_objects', self.dynamic_objects_callback, 10)

        # Publishers
        self.semantic_path_pub = self.create_publisher(
            Path, '/semantic_plan', 10)
        self.navigation_intent_pub = self.create_publisher(
            String, '/navigation_intent', 10)

        # Internal state
        self.current_semantic_map = None
        self.dynamic_objects = []
        self.navigation_goals = []

        self.get_logger().info('Semantic Navigation Node initialized')

    def semantic_callback(self, msg):
        """Process semantic segmentation data"""
        self.current_semantic_map = msg
        self.get_logger().debug('Processed semantic segmentation data')

    def dynamic_objects_callback(self, msg):
        """Process dynamic object detections"""
        self.dynamic_objects = msg.markers
        self.get_logger().debug(f'Processed {len(msg.markers)} dynamic objects')

    def navigate_with_semantic_awareness(self, goal_pose, target_object_class=None):
        """Navigate with semantic awareness"""
        if target_object_class:
            # If navigating to a specific object, use semantic guidance
            goal_pose = self.adjust_goal_for_target_object(goal_pose, target_object_class)

        # Send navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)

        # Publish navigation intent
        intent_msg = String()
        intent_msg.data = f"navigating_to_{target_object_class if target_object_class else 'position'}"
        self.navigation_intent_pub.publish(intent_msg)

        return future

    def adjust_goal_for_target_object(self, goal_pose, target_class):
        """Adjust navigation goal based on target object class"""
        if not self.current_semantic_map:
            return goal_pose

        # In a real implementation, this would:
        # 1. Analyze semantic map to find instances of target_class
        # 2. Adjust goal to approach the target object appropriately
        # 3. Consider object orientation and approach angle

        # For now, just return the original goal
        return goal_pose

    def get_semantic_path_cost(self, path):
        """Calculate cost of a path based on semantic information"""
        if not self.current_semantic_map:
            return float('inf')

        total_cost = 0
        for pose in path.poses:
            # Get semantic class at this position
            semantic_class = self.get_semantic_class_at_position(
                pose.pose.position.x, pose.pose.position.y)

            # Apply class-specific cost
            if semantic_class < len(self.class_weights):
                cost = self.class_weights[semantic_class]
                total_cost += cost

        return total_cost

    def get_semantic_class_at_position(self, x, y):
        """Get semantic class at a specific world position"""
        # This would involve transforming world coordinates to image coordinates
        # and sampling the semantic segmentation map
        # For now, return a default value
        return 0  # free space

    def create_semantic_aware_path(self, start, goal, preferred_classes=None):
        """Create a path that considers semantic information"""
        # This would implement a semantic-aware path planning algorithm
        # that prefers certain semantic classes over others
        path = Path()
        path.header.frame_id = "map"

        # For now, create a simple straight-line path
        # In reality, this would use semantic information to guide path planning
        current = start
        steps = 10
        for i in range(steps + 1):
            t = i / steps
            x = start.x + t * (goal.x - start.x)
            y = start.y + t * (goal.y - start.y)
            z = start.z + t * (goal.z - start.z)

            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = z
            path.poses.append(pose_stamped)

        return path


def main(args=None):
    rclpy.init(args=args)
    semantic_nav = SemanticNavigationNode()

    try:
        rclpy.spin(semantic_nav)
    except KeyboardInterrupt:
        semantic_nav.get_logger().info('Semantic Navigation Node stopped by user')
    finally:
        semantic_nav.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Launch Files for Integration

### 1. Complete Integration Launch File

Create a launch file `perception_navigation_integration_launch.py`:

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
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file')
    run_rviz = LaunchConfiguration('run_rviz', default='true')
    autostart = LaunchConfiguration('autostart', default='true')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('your_robot_navigation'),
            'config', 'perception_navigation.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_run_rviz = DeclareLaunchArgument(
        'run_rviz',
        default_value='true',
        description='Whether to start RViz')

    # Isaac ROS Visual SLAM node
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='visual_slam_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_occupancy_map': True,
            'enable_point_cloud_output': True
        }],
        remappings=[
            ('/stereo_camera/left/image', '/camera/left/image_rect_color'),
            ('/stereo_camera/left/camera_info', '/camera/left/camera_info'),
            ('/stereo_camera/right/image', '/camera/right/image_rect_color'),
            ('/stereo_camera/right/camera_info', '/camera/right/camera_info'),
        ],
        output='screen'
    )

    # Isaac ROS stereo image processing
    stereo_image_proc_node = Node(
        package='isaac_ros_stereo_image_proc',
        executable='stereo_image_proc',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Isaac ROS segmentation
    segmentation_node = Node(
        package='isaac_ros_dnn_segmentation',
        executable='dnn_segmentation_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model_name': 'unet',
            'input_topic': '/camera/image_raw',
            'output_topic': '/segmentation/segmentation_map'
        }],
        output='screen'
    )

    # Perception-Navigation bridge
    perception_bridge_node = Node(
        package='your_robot_navigation',
        executable='perception_navigation_bridge',
        parameters=[{
            'use_sim_time': use_sim_time,
            'perception_timeout': 1.0,
            'dynamic_object_threshold': 0.5
        }],
        output='screen'
    )

    # Semantic navigation node
    semantic_navigation_node = Node(
        package='your_robot_navigation',
        executable='semantic_navigation_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'semantic_class_weights': [0.1, 10.0, 8.0, 10.0, 5.0, 6.0]
        }],
        output='screen'
    )

    # Navigation2 stack
    navigation2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart
        }.items()
    )

    # RViz2
    rviz_node = Node(
        condition=IfCondition(run_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('nav2_bringup'),
            'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file)
    ld.add_action(declare_autostart)
    ld.add_action(declare_run_rviz)

    # Add Isaac ROS nodes
    ld.add_action(stereo_image_proc_node)
    ld.add_action(visual_slam_node)
    ld.add_action(segmentation_node)

    # Add perception-navigation bridge
    ld.add_action(perception_bridge_node)
    ld.add_action(semantic_navigation_node)

    # Add Navigation2 stack
    ld.add_action(navigation2_launch)

    # Add RViz2
    ld.add_action(rviz_node)

    return ld
```

## Performance Optimization for Integration

### 1. Optimized Integration Configuration

```yaml
# Optimized perception-navigation integration
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 0.5  # Lower for global map
      publish_frequency: 0.5
      transform_tolerance: 1.0
      use_sim_time: false

      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.22
      resolution: 0.05

      plugins: ["static_layer", "isaac_perception_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True

      # Optimized Isaac perception layer
      isaac_perception_layer:
        plugin: "nav2_isaac_perception_layer/IsaacPerceptionLayer"
        enabled: True
        observation_sources: segmentation
        segmentation:
          topic: /segmentation/segmentation_map
          sensor_frame: camera_link
          data_type: "Image"
          clearing: False
          marking: True
          obstacle_value: 254
          free_space_value: 0
          observation_persistence: 0.5  # Keep observations for 0.5s

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.5

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 15.0  # Higher for local planning
      publish_frequency: 10.0
      transform_tolerance: 0.2
      use_sim_time: false

      global_frame: odom
      robot_base_frame: base_link
      robot_radius: 0.22
      resolution: 0.025

      rolling_window: true
      width: 4
      height: 4
      origin_x: -2.0
      origin_y: -2.0

      plugins: ["voxel_layer", "isaac_perception_layer", "inflation_layer"]

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: False  # Disable for performance
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 8  # Reduce for performance
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan

      # Optimized perception layer for local costmap
      isaac_perception_layer:
        plugin: "nav2_isaac_perception_layer/IsaacPerceptionLayer"
        enabled: True
        observation_sources: stereo_depth
        stereo_depth:
          topic: /stereo/depth/disparity
          sensor_frame: stereo_camera_link
          data_type: "Disparity"
          clearing: True
          marking: True
          obstacle_range: 2.0  # Shorter range for local planning
          raytrace_range: 3.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 5.0
        inflation_radius: 0.4
```

### 2. Memory and Computation Optimization

```yaml
# Memory-optimized integration configuration
controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0  # Balance performance and control quality
    min_x_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.5
    progress_checker_plugin: "simple_progress_checker"
    goal_checker_plugins: ["simple_goal_checker"]
    controller_plugins: ["SimplePerceptionController"]

    simple_progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    simple_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.3
      yaw_goal_tolerance: 0.3
      stateful: False  # Disable stateful behavior for performance

    SimplePerceptionController:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: False
      min_vel_x: 0.1
      max_vel_x: 0.4
      max_vel_theta: 1.0
      vx_samples: 15  # Reduce samples for performance
      vtheta_samples: 15
      sim_time: 1.0  # Shorter simulation time
      linear_granularity: 0.1
      angular_granularity: 0.05
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.3
      stateful: False
      publish_cost_grid_pc: False  # Disable for performance
```

## Quality Assurance and Testing

### 1. Integration Testing Framework

```python
#!/usr/bin/env python3
"""
Perception-Navigation Integration Testing
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time


class IntegrationTester(Node):
    def __init__(self):
        super().__init__('integration_tester')

        # Parameters
        self.declare_parameter('test_duration', 30.0)
        self.declare_parameter('success_threshold', 0.9)

        self.test_duration = self.get_parameter('test_duration').value
        self.success_threshold = self.get_parameter('success_threshold').value

        # Publishers and subscribers
        self.test_status_pub = self.create_publisher(Bool, '/integration_test_status', 10)
        self.test_result_pub = self.create_publisher(Float32, '/integration_test_result', 10)

        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Test parameters
        self.test_start_time = None
        self.test_active = False
        self.success_count = 0
        self.total_attempts = 0

        # Timer for periodic testing
        self.test_timer = self.create_timer(5.0, self.run_test_cycle)

        self.get_logger().info('Integration Tester initialized')

    def run_test_cycle(self):
        """Run a navigation test cycle"""
        if not self.test_active:
            self.start_test()
            return

        # Check if test duration has elapsed
        if (self.get_clock().now().nanoseconds / 1e9 - self.test_start_time) > self.test_duration:
            self.end_test()
            return

        # Run navigation test
        self.run_navigation_test()

    def start_test(self):
        """Start integration test"""
        self.test_active = True
        self.test_start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f'Starting integration test for {self.test_duration} seconds')

    def run_navigation_test(self):
        """Run a single navigation test"""
        # Define test goal (this would be more sophisticated in practice)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = 2.0
        goal_pose.pose.position.y = 2.0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        # Send navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        if self.nav_client.wait_for_server(timeout_sec=1.0):
            self.total_attempts += 1

            # Send goal and wait for result
            future = self.nav_client.send_goal_async(goal_msg)
            # In a real test, we'd wait for the result and check success
            # For this example, we'll simulate success/failure
            success = True  # Simulated result
            if success:
                self.success_count += 1

    def end_test(self):
        """End integration test and report results"""
        success_rate = self.success_count / self.total_attempts if self.total_attempts > 0 else 0
        self.get_logger().info(f'Test completed - Success rate: {success_rate:.2f} ({self.success_count}/{self.total_attempts})')

        # Publish results
        result_msg = Float32()
        result_msg.data = success_rate
        self.test_result_pub.publish(result_msg)

        status_msg = Bool()
        status_msg.data = success_rate >= self.success_threshold
        self.test_status_pub.publish(status_msg)

        self.test_active = False


def main(args=None):
    rclpy.init(args=args)
    tester = IntegrationTester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info('Integration tester stopped by user')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Troubleshooting Integration Issues

### 1. Common Integration Problems and Solutions

#### Issue: Perception data not affecting navigation
**Symptoms**: Robot doesn't respond to dynamic objects or semantic information
**Solutions**:
1. Check topic connections:
   ```bash
   ros2 topic echo /segmentation/segmentation_map
   ros2 topic list | grep costmap
   ```
2. Verify costmap configuration includes perception layers
3. Check TF tree for proper frame relationships
4. Confirm perception nodes are publishing data

#### Issue: High CPU usage with integration
**Symptoms**: System becomes unresponsive when perception is active
**Solutions**:
1. Reduce perception processing frequency
2. Lower costmap resolution
3. Simplify perception algorithms
4. Use hardware acceleration for perception

#### Issue: Navigation conflicts with perception
**Symptoms**: Robot stops frequently due to false obstacle detections
**Solutions**:
1. Adjust perception confidence thresholds
2. Increase obstacle filtering
3. Tune costmap inflation parameters
4. Implement perception validation

### 2. Performance Monitoring

```bash
# Monitor integration performance
ros2 run topic_tools relay /perception_processing_time
ros2 run topic_tools relay /navigation_compute_time
ros2 run topic_tools relay /perception_navigation_status

# Check TF tree
ros2 run tf2_tools view_frames

# Monitor node performance
ros2 run top top
```

## Best Practices for Integration

### 1. Design Guidelines
- Use appropriate data fusion techniques
- Implement proper error handling and fallbacks
- Design for graceful degradation when perception fails
- Maintain real-time performance requirements

### 2. Testing Strategies
- Test in simulation before real-world deployment
- Validate individual components before integration
- Test with various environmental conditions
- Verify safety and reliability requirements

### 3. Performance Optimization
- Use appropriate data structures for real-time processing
- Implement efficient algorithms for perception tasks
- Optimize communication between nodes
- Consider hardware acceleration where possible

## Resources

- [Isaac ROS Navigation Integration Guide](https://nvidia-isaac-ros.github.io/concepts/navigation_integration/index.html)
- [Navigation2 Perception Tutorials](https://navigation.ros.org/tutorials/docs/navigation2_with_vslam.html)
- [Costmap Layer Development](https://navigation.ros.org/plugins/costmap_plugins/index.html)

## Conclusion

This guide provides comprehensive instructions for integrating Isaac ROS perception with Navigation2. The integration enables robots to leverage visual and sensor data for enhanced navigation capabilities, including better obstacle detection, semantic awareness, and improved localization. Proper integration requires careful attention to data flow, timing, and performance considerations. The key to successful integration is starting with basic configurations and gradually adding complexity while maintaining system stability and performance.