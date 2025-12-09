# VSLAM Pipeline Configuration for Isaac ROS

## Overview

This guide provides instructions for configuring a Visual Simultaneous Localization and Mapping (VSLAM) pipeline using Isaac ROS. VSLAM enables robots to build maps of their environment while simultaneously localizing themselves within those maps using visual sensors like cameras.

## Prerequisites

- Isaac ROS perception packages installed (completed in T063)
- ROS 2 Humble Hawksbill
- Camera sensors (monocular, stereo, or RGB-D)
- Properly calibrated camera(s)
- Sufficient computational resources (GPU recommended)

## Understanding Isaac ROS Visual SLAM

### Components
- **Stereo Image Processing**: Converts stereo images to depth maps
- **Visual SLAM**: Performs mapping and localization
- **Pose Graph Optimization**: Refines trajectory estimates
- **Loop Closure Detection**: Corrects drift over time

### Available Packages
- `isaac_ros_visual_slam`: Main VSLAM package
- `isaac_ros_stereo_image_proc`: Stereo processing
- `isaac_ros_image_proc`: Image processing utilities

## VSLAM Pipeline Configuration

### 1. Basic Visual SLAM Node Configuration

Create a configuration file `vsland_pipeline.yaml`:

```yaml
# Visual SLAM pipeline configuration
visual_slam_node:
  ros__parameters:
    # Input topics
    camera_pose_with_covariance_output_topic: "/visual_slam/camera_pose_with_covariance"
    corrected_pose_with_covariance_output_topic: "/visual_slam/corrected_pose_with_covariance"
    feedback_frame_id: "base_link"
    enable_slam_visualization: true
    enable_landmarks_view: false
    enable_observations_view: false
    map_frame: "map"
    odom_frame: "odom"
    base_frame: "base_link"
    input_bound: 10
    min_num_images_to_start: 3
    enable_localization_n_mapping: true
    enable_localization: false
    enable_occupancy_map: false
    enable_point_cloud_output: true
    enable_imu: false
    enable_rectification: true
    rectified_frame_id: "base_link"
    use_odometry_input: false
    use_sim_time: false

    # Feature tracking parameters
    tracker:
      max_num_points: 1000
      min_distance: 20.0
      quality_level: 0.01
      pyramid_level: 3
      window_size: [21, 21]
      termination_criteria: [30, 0.01]
      use_gpu: true  # Enable GPU acceleration

    # Keyframe selection
    keyframe:
      translation_threshold: 0.5
      rotation_threshold: 0.5
      minimum_translation: 0.1
      minimum_rotation: 0.1
      maximum_observations_lost: 10
      minimum_observations_for_tracking: 10

    # Mapping parameters
    mapping:
      max_num_landmarks: 10000
      minimum_observations_for_landmark: 2
      maximum_observations_lost: 5
      minimum_observations_for_tracking: 3
      reprojection_threshold: 2.0
      use_gpu: true

    # Loop closure detection
    loop_closure:
      enable_loop_detection: true
      minimum_loop_closure_interval: 10.0
      minimum_translation_for_loop: 1.0
      minimum_keyframes_for_loop: 10
      similarity_threshold: 0.7
      geometric_verification_threshold: 10.0
      inlier_threshold: 0.9
      use_gpu: true

# Stereo image processing node
stereo_image_proc:
  ros__parameters:
    # Input topics
    left/image_raw: "/camera/left/image_raw"
    left/camera_info: "/camera/left/camera_info"
    right/image_raw: "/camera/right/image_raw"
    right/camera_info: "/camera/right/camera_info"

    # Output topics
    left/image_rect: "/stereo/left/image_rect_color"
    right/image_rect: "/stereo/right/image_rect_color"
    disparity: "/stereo/disparity"

    # Processing parameters
    queue_size: 5
    alpha: -1.0  # Auto-calculated rectification
    use_camera_info: true
    alpha_adjuster: "none"
    stereo_algorithm: 0  # Block matching
    disparity_range: [0, 128]
    correlation_window_size: 15
    prefilter_cap: 31
    prefilter_size: 9
    speckle_size: 100
    speckle_range: 32
    disp12_max_diff: 1
    min_disparity: 0
    uniqueness_ratio: 15
    texture_threshold: 10
    uniqueness_ratio: 15
    P1: 200
    P2: 400
    full_dp: false
```

### 2. Launch File Configuration

Create a launch file `vsland_pipeline_launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    vsland_config_file = LaunchConfiguration('vsland_config_file')
    enable_visualization = LaunchConfiguration('enable_visualization', default='true')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')

    declare_vsland_config_file = DeclareLaunchArgument(
        'vsland_config_file',
        default_value=os.path.join(
            get_package_share_directory('your_robot_navigation'),
            'config', 'vsland_pipeline.yaml'),
        description='Full path to the VSLAM configuration file')

    declare_enable_visualization = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable VSLAM visualization')

    # Visual SLAM node
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='visual_slam_node',
        parameters=[vsland_config_file,
                   {'use_sim_time': use_sim_time}],
        remappings=[('stereo_camera/left/image', '/camera/left/image_rect_color'),
                   ('stereo_camera/left/camera_info', '/camera/left/camera_info'),
                   ('stereo_camera/right/image', '/camera/right/image_rect_color'),
                   ('stereo_camera/right/camera_info', '/camera/right/camera_info'),
                   ('visual_slam/imu', '/imu/data')],
        output='screen'
    )

    # Stereo image processing node (if using stereo)
    stereo_image_proc_node = Node(
        package='isaac_ros_stereo_image_proc',
        executable='stereo_image_proc',
        parameters=[vsland_config_file,
                   {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Point cloud mapper node
    point_cloud_node = Node(
        package='isaac_ros_visual_slam',
        executable='pointcloud_mapper_node',
        parameters=[vsland_config_file,
                   {'use_sim_time': use_sim_time}],
        remappings=[('visual_slam/landmarks', '/visual_slam/landmarks'),
                   ('visual_slam/optimized_landmarks', '/visual_slam/optimized_landmarks')],
        output='screen'
    )

    # RViz2 node for visualization (optional)
    rviz_node = Node(
        condition=IfCondition(enable_visualization),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('isaac_ros_visual_slam'),
                                     'rviz', 'visual_slam.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_vsland_config_file)
    ld.add_action(declare_enable_visualization)

    # Add nodes
    ld.add_action(stereo_image_proc_node)
    ld.add_action(visual_slam_node)
    ld.add_action(point_cloud_node)
    ld.add_action(rviz_node)

    return ld
```

### 3. Monocular Camera Configuration

For monocular VSLAM, modify the configuration:

```yaml
# Monocular VSLAM configuration
visual_slam_node:
  ros__parameters:
    # Use monocular input instead of stereo
    image_raw_topic: "/camera/image_raw"
    camera_info_topic: "/camera/camera_info"

    # Disable stereo-specific features
    enable_imu: false
    use_stereo: false

    # Monocular-specific parameters
    tracker:
      max_num_points: 800  # Slightly fewer features for monocular
      min_distance: 15.0
      quality_level: 0.015  # Higher quality features for monocular
      pyramidal_level: 4    # More pyramid levels for better tracking

    # Scale estimation (monocular is scale ambiguous)
    scale_estimator:
      enable_scale_estimation: true
      initial_scale: 1.0
      scale_window_size: 10

    # Depth estimation for monocular
    depth_estimator:
      enable_depth_estimation: true
      max_depth: 10.0
      min_depth: 0.3
```

### 4. RGB-D Camera Configuration

For RGB-D VSLAM:

```yaml
# RGB-D VSLAM configuration
visual_slam_node:
  ros__parameters:
    # RGB-D specific topics
    rgb_image_topic: "/camera/color/image_raw"
    depth_image_topic: "/camera/depth/image_rect_raw"
    camera_info_topic: "/camera/color/camera_info"

    # RGB-D specific parameters
    use_depth_input: true
    depth_scale_factor: 1000.0  # Convert depth to meters
    max_depth: 5.0
    min_depth: 0.1

    # Enhanced tracking with depth
    tracker:
      max_num_points: 1200  # More features with depth guidance
      use_depth_for_tracking: true
      depth_threshold: 0.1  # Depth consistency threshold

    # Mapping improvements with depth
    mapping:
      enable_3d_landmarks: true
      depth_uncertainty_threshold: 0.05
```

## Performance Optimization

### 1. GPU Acceleration Configuration

Enable GPU acceleration in the configuration:

```yaml
visual_slam_node:
  ros__parameters:
    # GPU acceleration settings
    use_gpu: true
    gpu_id: 0
    cuda_device_id: 0

    # Feature extraction GPU settings
    feature_extraction:
      use_gpu: true
      gpu_memory_pool_size: 100  # MB
      gpu_max_memory_usage: 500  # MB

    # Tracking GPU settings
    tracking:
      use_gpu: true
      gpu_thread_count: 4
```

### 2. Memory Management

Optimize memory usage:

```yaml
visual_slam_node:
  ros__parameters:
    # Memory optimization
    max_map_size: 50000  # Maximum landmarks in map
    landmark_forgetting_policy: "oldest"  # Remove oldest landmarks
    keyframe_forgetting_policy: "aggressive"  # Remove redundant keyframes

    # Processing optimization
    processing_queue_size: 3  # Limit processing backlog
    max_processing_threads: 8  # CPU threads for processing
```

## Integration with Navigation Stack

### 1. TF Tree Configuration

Ensure proper TF relationships:

```yaml
# TF configuration for VSLAM integration
visual_slam_node:
  ros__parameters:
    # TF frames
    map_frame: "map"
    odom_frame: "odom"
    base_frame: "base_link"
    camera_frame: "camera_link"

    # TF publishing
    publish_tf: true
    tf_publish_rate: 50.0  # Hz
```

### 2. Odometry Integration

Combine VSLAM with other odometry sources:

```yaml
# Robot localization node configuration
robot_localization:
  ros__parameters:
    # VSLAM as odometry source
    odom0: /visual_slam/odometry
    odom0_config: [true, true, true,  # x, y, z
                   true, true, true,  # roll, pitch, yaw
                   false, false, false]  # No linear velocity
    odom0_differential: false
    odom0_relative: false

    # Other odometry sources
    odom1: /wheel/odometry  # Wheel encoders
    odom1_config: [true, true, false,  # x, y, no z
                   false, false, true,  # Only yaw from IMU
                   false, false, false]

    # Fusion parameters
    frequency: 50.0
    sensor_timeout: 0.1
    two_d_mode: true
    transform_time_offset: 0.0
```

## Calibration Requirements

### 1. Camera Calibration

Ensure cameras are properly calibrated:

```bash
# Check camera calibration
ros2 run camera_calibration_parsers read_calibration /path/to/calibration.yaml

# Verify calibration parameters
ros2 topic echo /camera/camera_info
```

### 2. Extrinsic Calibration

Calibrate camera-to-robot transforms:

```yaml
# Extrinsic calibration in URDF
<link name="camera_link">
  <visual>
    <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
    <!-- Camera offset from base -->
  </visual>
  <collision>
    <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
  </collision>
</link>
```

## Troubleshooting Common Issues

### Issue: Poor Tracking Performance
**Symptoms**: Frequent tracking failures, drifting pose estimates
**Solutions**:
1. Increase feature count:
   ```yaml
   tracker:
     max_num_points: 1500
     min_distance: 10.0
   ```
2. Improve lighting conditions
3. Check camera calibration
4. Reduce motion speed during initialization

### Issue: Map Drift
**Symptoms**: Map deforms over time, loop closure fails
**Solutions**:
1. Enable loop closure:
   ```yaml
   loop_closure:
     enable_loop_detection: true
     minimum_translation_for_loop: 0.5
   ```
2. Increase keyframe frequency
3. Improve feature tracking parameters

### Issue: High CPU/GPU Usage
**Symptoms**: System becomes unresponsive, frame drops
**Solutions**:
1. Reduce feature count:
   ```yaml
   tracker:
     max_num_points: 500
   ```
2. Lower processing frequency
3. Use simpler stereo algorithms

### Issue: Scale Ambiguity (Monocular)
**Symptoms**: Map scale changes during operation
**Solutions**:
1. Use known object sizes for scale reference
2. Integrate with IMU for scale estimation
3. Use stereo or RGB-D instead of monocular

## Quality Assurance

### 1. Performance Metrics

Monitor VSLAM performance:

```bash
# Track frame rates
ros2 topic hz /camera/image_raw
ros2 topic hz /visual_slam/odometry

# Monitor processing times
ros2 run topic_tools relay /visual_slam/timing_stats
```

### 2. Accuracy Validation

Validate mapping accuracy:

```bash
# Compare with ground truth (if available)
ros2 run tf2_tools view_frames
ros2 topic echo /visual_slam/odometry --field pose.pose.position
```

## Advanced Configuration

### 1. Multi-Camera Setup

For multiple cameras:

```yaml
visual_slam_node:
  ros__parameters:
    # Multi-camera parameters
    enable_multi_camera: true
    camera_ids: [0, 1, 2]

    # Camera 0 (front)
    camera_0:
      topic: "/camera0/image_raw"
      info_topic: "/camera0/camera_info"
      transform: [0.2, 0, 0.3, 0, 0, 0]  # x, y, z, roll, pitch, yaw

    # Camera 1 (left)
    camera_1:
      topic: "/camera1/image_raw"
      info_topic: "/camera1/camera_info"
      transform: [0.1, 0.1, 0.3, 0, 0, 1.57]  # 90 degree offset
```

### 2. Dynamic Object Filtering

Filter dynamic objects:

```yaml
visual_slam_node:
  ros__parameters:
    # Dynamic object handling
    enable_dynamic_object_filtering: true
    dynamic_object_threshold: 5.0  # Pixels of motion threshold
    temporal_consistency_window: 10  # Frames for consistency check
```

## Best Practices

### 1. Environment Considerations
- Ensure sufficient lighting
- Avoid repetitive patterns
- Provide distinctive visual features
- Minimize motion blur

### 2. Hardware Optimization
- Use cameras with global shutters for fast motion
- Ensure sufficient GPU memory
- Use fast storage for map saving/loading
- Maintain stable power supply

### 3. Parameter Tuning
- Start with default parameters
- Adjust one parameter at a time
- Test in controlled environments first
- Document working configurations

## Testing and Validation

### 1. Simple Test Environment
1. Start with a simple, well-textured environment
2. Move robot slowly in straight lines
3. Verify pose estimates are reasonable
4. Check map quality and consistency

### 2. Loop Closure Test
1. Drive in a loop pattern
2. Verify loop closure detection
3. Check map consistency after closure
4. Measure drift correction

## Resources

- [Isaac ROS Visual SLAM Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html)
- [ROS 2 Navigation with VSLAM Tutorial](https://navigation.ros.org/tutorials/docs/navigation2_with_vslam.html)
- [Camera Calibration Guide](https://wiki.ros.org/camera_calibration)

## Conclusion

This guide provides a comprehensive configuration for Isaac ROS Visual SLAM pipeline. Proper configuration is essential for achieving accurate mapping and localization. Start with basic configurations and gradually optimize parameters based on your specific application requirements and hardware capabilities. The combination of visual SLAM with navigation systems enables powerful autonomous capabilities for robots operating in unknown environments.