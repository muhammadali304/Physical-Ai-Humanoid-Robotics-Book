# Costmap Configuration for Obstacle Avoidance

## Overview

This guide provides instructions for configuring costmaps for effective obstacle avoidance in robot navigation. Costmaps are critical components of the Navigation2 stack that represent the environment with different cost values for navigation planning and obstacle avoidance.

## Understanding Costmaps

### What are Costmaps?
Costmaps are 2D or 3D representations of the robot's environment where each cell contains a cost value indicating how difficult or dangerous it is for the robot to navigate through that area. Higher cost values represent areas to avoid, while lower values indicate free space.

### Types of Costmaps
1. **Local Costmap**: Short-term planning around the robot (typically 3-10 meters)
2. **Global Costmap**: Long-term planning for entire navigation route
3. **Static Layer**: Represents permanent obstacles from the map
4. **Obstacle Layer**: Represents dynamic obstacles from sensors
5. **Inflation Layer**: Expands obstacles to account for robot size and safety margin

## Costmap Configuration

### 1. Basic Costmap Configuration

Create a comprehensive costmap configuration file `costmap_config.yaml`:

```yaml
# Global Costmap Configuration
global_costmap:
  global_costmap:
    ros__parameters:
      # Global parameters
      update_frequency: 1.0
      publish_frequency: 1.0
      transform_tolerance: 0.5
      use_sim_time: false

      # Map parameters
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.22  # Radius of the robot in meters
      resolution: 0.05    # Meters per cell

      # Plugin configuration
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      # Static layer (from map)
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
        transform_tolerance: 0.5

      # Obstacle layer (from sensors)
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
          inf_is_valid: False

      # Inflation layer (safety buffer)
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0  # Exponential decay factor
        inflation_radius: 0.55    # Maximum inflation distance in meters
        inflate_unknown: False
        inflate_around_unknown: False

      # Always send full costmap for debugging
      always_send_full_costmap: True

  # Client parameters for global costmap
  global_costmap_client:
    ros__parameters:
      use_sim_time: false

  # RCLCPP node parameters
  global_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: false

# Local Costmap Configuration
local_costmap:
  local_costmap:
    ros__parameters:
      # Local parameters
      update_frequency: 5.0
      publish_frequency: 2.0
      transform_tolerance: 0.5
      use_sim_time: false

      # Map parameters
      global_frame: odom
      robot_base_frame: base_link
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: false

      # Rolling window for local costmap
      rolling_window: true
      width: 6
      height: 6
      origin_x: 0.0
      origin_y: 0.0

      # Plugin configuration
      plugins: ["voxel_layer", "inflation_layer"]

      # Voxel layer for 3D obstacle detection
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
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0

      # Inflation layer for local costmap
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
        inflate_unknown: False

      # Always send full costmap
      always_send_full_costmap: True

  # Client parameters for local costmap
  local_costmap_client:
    ros__parameters:
      use_sim_time: false

  # RCLCPP node parameters
  local_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: false
```

### 2. Advanced Costmap Configuration for Complex Environments

For more complex obstacle avoidance scenarios:

```yaml
# Advanced costmap configuration
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 0.5  # Slower updates for global map
      publish_frequency: 0.5
      transform_tolerance: 1.0
      use_sim_time: false

      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.22
      resolution: 0.025  # Higher resolution for better accuracy

      plugins: ["static_layer", "obstacle_layer", "inflation_layer", "range_sensor_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
        transform_tolerance: 1.0
        track_unknown_space: True
        use_maximum: True  # Use maximum of static and other layers

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
          raytrace_max_range: 5.0  # Extended range for better planning
          raytrace_min_range: 0.1
          obstacle_max_range: 4.0  # Extended range for obstacle detection
          obstacle_min_range: 0.1
          inf_is_valid: False
          clearing_endpoints: False

      # Range sensor layer for additional sensor types
      range_sensor_layer:
        plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
        enabled: True
        voxel_decay: 10.0
        decay_model: 0  # 0=linear, 1=exponential, 2=quadratic
        voxel_size: 0.05
        track_unknown_space: True
        observation_sources: range_sensor
        range_sensor:
          topic: /range/sensor
          sensor_frame: range_sensor_frame
          data_type: "Range"
          clearing: True
          marking: True

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: True
        cost_scaling_factor: 5.0  # More aggressive inflation
        inflation_radius: 0.8     # Larger safety margin
        inflate_unknown: False
        inflate_around_unknown: False

# Advanced local costmap
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0  # Faster updates for local planning
      publish_frequency: 5.0
      transform_tolerance: 0.2
      use_sim_time: false

      global_frame: odom
      robot_base_frame: base_link
      robot_radius: 0.22
      resolution: 0.025  # Higher resolution for local planning

      rolling_window: true
      width: 8
      height: 8
      origin_x: -4.0
      origin_y: -4.0

      plugins: ["voxel_layer", "inflation_layer", "dynamic_layer"]

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
          raytrace_min_range: 0.1
          obstacle_max_range: 4.0
          obstacle_min_range: 0.1

      # Dynamic obstacle layer for moving objects
      dynamic_layer:
        plugin: "nav2_dynamic_obstacles_layer/DynamicObstaclesLayer"
        enabled: True
        observation_sources: dynamic_obs
        dynamic_obs:
          topic: /dynamic_obstacles
          sensor_frame: base_link
          data_type: "PointCloud2"
          clearing: True
          marking: True

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: True
        cost_scaling_factor: 8.0  # More aggressive for local planning
        inflation_radius: 0.6
        inflate_unknown: False
```

## Costmap Layers Configuration

### 1. Static Layer Configuration

The static layer represents permanent obstacles from the known map:

```yaml
static_layer:
  plugin: "nav2_costmap_2d::StaticLayer"
  map_subscribe_transient_local: True
  transform_tolerance: 0.5
  track_unknown_space: False
  use_maximum: False
  first_map_only: False
  subscribe_to_updates: False
  map_topic: "map"
  lethal_cost_threshold: 100  # Values >= this are considered obstacles
  unknown_cost_value: -1      # Value for unknown space (-1 = 255 in nav2)
  trinary_costmap: True       # Three values: free, unknown, lethal
```

### 2. Obstacle Layer Configuration

The obstacle layer processes sensor data to detect dynamic obstacles:

```yaml
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  enabled: True
  observation_sources: scan
  scan:
    topic: /scan
    sensor_frame: laser_frame
    max_obstacle_height: 2.0
    clearing: True
    marking: True
    data_type: "LaserScan"
    raytrace_max_range: 3.0
    raytrace_min_range: 0.0
    obstacle_max_range: 2.5
    obstacle_min_range: 0.0
    obstacle_max_height: 2.0
    obstacle_min_height: 0.0
    inf_is_valid: False
    clearing_endpoints: False
    model_type: "beam"
    clearing_threshold: 0.196
    marking_threshold: 0.25
```

### 3. Inflation Layer Configuration

The inflation layer creates safety margins around obstacles:

```yaml
inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  enabled: True
  cost_scaling_factor: 10.0    # Exponential decay factor (higher = faster decay)
  inflation_radius: 1.0        # Maximum inflation distance
  inflate_unknown: False       # Whether to inflate unknown space
  inflate_around_unknown: False # Whether to inflate around unknown space
  inflate_free_space: False    # Whether to inflate free space
```

## Costmap Integration with Isaac ROS

### 1. Perception-Enhanced Costmaps

Integrate Isaac ROS perception data into costmaps:

```yaml
# Costmap configuration with Isaac ROS perception
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "isaac_perception_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True

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

      # Isaac ROS perception layer
      isaac_perception_layer:
        plugin: "nav2_isaac_perception_layer/IsaacPerceptionLayer"
        enabled: True
        observation_sources: semantic_segmentation
        semantic_segmentation:
          topic: /segmentation/segmentation_map
          sensor_frame: camera_link
          data_type: "Image"
          clearing: False
          marking: True
          obstacle_value: 254  # Value representing obstacles in segmentation
          free_space_value: 0  # Value representing free space

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 5.0
        inflation_radius: 0.8

# Local costmap with perception
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["voxel_layer", "isaac_perception_layer", "inflation_layer"]

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

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 8.0
        inflation_radius: 0.6
```

### 2. Semantic Costmap Configuration

Use semantic segmentation for intelligent obstacle classification:

```yaml
semantic_costmap:
  semantic_costmap:
    ros__parameters:
      plugins: ["semantic_layer", "inflation_layer"]

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
          # Semantic class mappings
          class_mappings:
            - {class_id: 0, class_name: "free_space", cost: 0, is_obstacle: false}
            - {class_id: 1, class_name: "wall", cost: 254, is_obstacle: true}
            - {class_id: 2, class_name: "person", cost: 200, is_obstacle: true}
            - {class_id: 3, class_name: "furniture", cost: 254, is_obstacle: true}
            - {class_id: 4, class_name: "plant", cost: 100, is_obstacle: true}
            - {class_id: 5, class_name: "clutter", cost: 150, is_obstacle: true}

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.5
```

## Costmap Performance Tuning

### 1. Performance Optimization Parameters

```yaml
# Optimized costmap configuration for performance
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 0.5    # Lower frequency for global map
      publish_frequency: 0.5
      resolution: 0.1          # Lower resolution for performance
      width: 100              # Smaller initial size
      height: 100
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      # Optimized obstacle layer
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
          raytrace_max_range: 10.0  # Reduce range if not needed
          obstacle_max_range: 8.0
          observation_persistence: 0.0  # Process immediately

      # Optimized inflation
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.5  # Smaller radius for performance

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0   # Higher frequency for local planning
      publish_frequency: 5.0
      resolution: 0.05         # Higher resolution for local planning
      rolling_window: true
      width: 4                 # Smaller window for performance
      height: 4
      plugins: ["voxel_layer", "inflation_layer"]

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: False  # Disable if not needed
        z_voxels: 8              # Reduce if not needed
        max_obstacle_height: 2.0
        observation_sources: scan
        scan:
          topic: /scan
          observation_persistence: 0.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 5.0
        inflation_radius: 0.4
```

### 2. Memory Management

```yaml
# Memory-optimized costmap configuration
global_costmap:
  global_costmap:
    ros__parameters:
      # Reduce memory footprint
      width: 50
      height: 50
      resolution: 0.1
      track_unknown_space: False
      always_send_full_costmap: False  # Only send updates

      # Plugins with memory optimization
      plugins: ["static_layer", "obstacle_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True

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
          observation_persistence: 0.1  # Short persistence
          expected_update_rate: 10.0
```

## Costmap Troubleshooting

### 1. Common Issues and Solutions

#### Issue: Costmap Not Updating
**Symptoms**: Costmap shows static obstacles only, no dynamic obstacle detection
**Solutions**:
1. Check sensor topic availability:
   ```bash
   ros2 topic echo /scan
   ```
2. Verify TF tree:
   ```bash
   ros2 run tf2_tools view_frames
   ```
3. Check costmap parameters:
   ```bash
   ros2 param list | grep costmap
   ```

#### Issue: Poor Obstacle Detection
**Symptoms**: Robot collides with obstacles or doesn't detect them properly
**Solutions**:
1. Adjust sensor range parameters:
   ```yaml
   scan:
     raytrace_max_range: 5.0
     obstacle_max_range: 4.0
   ```
2. Verify sensor calibration and mounting
3. Check for sensor noise or interference

#### Issue: High CPU Usage
**Symptoms**: System becomes unresponsive during navigation
**Solutions**:
1. Reduce costmap resolution:
   ```yaml
   resolution: 0.1  # Increase from 0.05
   ```
2. Lower update frequencies:
   ```yaml
   update_frequency: 2.0  # Reduce from 5.0
   ```
3. Reduce costmap size:
   ```yaml
   width: 4  # Reduce from 6
   height: 4
   ```

#### Issue: Robot Gets Stuck Near Obstacles
**Symptoms**: Robot fails to navigate around obstacles
**Solutions**:
1. Increase inflation radius:
   ```yaml
   inflation_radius: 0.8  # Increase from 0.55
   ```
2. Adjust cost scaling factor:
   ```yaml
   cost_scaling_factor: 8.0  # Increase from 3.0
   ```
3. Verify robot radius:
   ```yaml
   robot_radius: 0.3  # Adjust based on actual robot size
   ```

### 2. Debugging Tools

#### Costmap Visualization
```bash
# Visualize costmaps in RViz
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz

# Add costmap displays:
# - Global Costmap
# - Local Costmap
# - Voxel Grid
# - Particle Cloud
```

#### Costmap Analysis
```bash
# Monitor costmap updates
ros2 topic hz /global_costmap/costmap
ros2 topic hz /local_costmap/costmap

# Check costmap statistics
ros2 run nav2_util costmap_analysis
```

## Best Practices

### 1. Parameter Selection Guidelines

- **Resolution**: 0.025-0.1m (finer for precise navigation, coarser for performance)
- **Update Frequency**: 1-10Hz for global, 5-20Hz for local
- **Inflation Radius**: 1.2-2x robot radius for safety
- **Robot Radius**: Measure actual robot dimensions + safety margin

### 2. Layer Configuration Best Practices

- Use static layer for known obstacles
- Use obstacle/voxel layer for sensor data
- Always include inflation layer for safety
- Consider semantic layers for intelligent navigation
- Test layer combinations thoroughly

### 3. Performance Optimization

- Start with conservative parameters
- Gradually optimize based on performance
- Monitor CPU and memory usage
- Balance accuracy with performance requirements

## Integration with Navigation System

### 1. Controller Integration
```yaml
controller_server:
  ros__parameters:
    # Use costmaps for local planning
    local_costmap_topic: local_costmap/costmap_raw
    global_costmap_topic: global_costmap/costmap_raw
    local_footprint_topic: local_costmap/published_footprint
    global_footprint_topic: global_costmap/published_footprint
```

### 2. Planner Integration
```yaml
planner_server:
  ros__parameters:
    # Use global costmap for path planning
    global_costmap_topic: global_costmap/costmap_raw
    global_footprint_topic: global_costmap/published_footprint
```

## Quality Assurance

### 1. Costmap Validation
- Verify obstacle detection accuracy
- Check inflation behavior around obstacles
- Test navigation in various obstacle configurations
- Monitor costmap update rates

### 2. Safety Validation
- Ensure adequate safety margins
- Test edge cases (narrow passages, dynamic obstacles)
- Validate behavior with different robot speeds
- Check robustness to sensor noise

## Resources

- [Navigation2 Costmap Documentation](https://navigation.ros.org/configuration/costmap_configuration/index.html)
- [Costmap Parameters Guide](https://github.com/ros-planning/navigation2/blob/main/nav2_costmap_2d/params/costmap.yaml)
- [Costmap Plugins Documentation](https://navigation.ros.org/plugins/costmap_plugins/index.html)

## Conclusion

This guide provides comprehensive configuration for costmaps in robot navigation obstacle avoidance. Proper costmap configuration is essential for safe and effective navigation. The key is finding the right balance between safety, performance, and accuracy for your specific application. Start with basic configurations and gradually optimize parameters based on testing results and performance requirements. The integration with Isaac ROS perception can provide even more intelligent and adaptive obstacle avoidance capabilities.