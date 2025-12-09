# Nav2 Navigation Stack Setup Guide for ROS 2 Humble

## Overview

This guide provides instructions for installing and configuring the Navigation2 (Nav2) stack for ROS 2 Humble. Nav2 is the official navigation stack for ROS 2, providing path planning, execution, and obstacle avoidance capabilities for mobile robots.

## Prerequisites

- ROS 2 Humble Hawksbill installed
- Ubuntu 22.04 LTS
- Basic understanding of ROS 2 concepts (topics, services, parameters)
- A robot with differential drive or similar mobility base

## Installation

### Method 1: Binary Installation (Recommended)

#### 1. Update Package Lists

```bash
sudo apt update
```

#### 2. Install Nav2 Packages

```bash
# Install the complete Nav2 stack
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Install additional useful packages
sudo apt install ros-humble-nav2-common
sudo apt install ros-humble-nav2-controllers
sudo apt install ros-humble-nav2-behaviors
sudo apt install ros-humble-nav2-util
sudo apt install ros-humble-nav2-amcl
sudo apt install ros-humble-nav2-map-server
sudo apt install ros-humble-nav2-planner
sudo apt install ros-humble-nav2-simulator
sudo apt install ros-humble-nav2-rviz-plugins
sudo apt install ros-humble-nav2-lifecycle-manager
```

#### 3. Install Additional Dependencies

```bash
# Install SLAM Toolbox for mapping (if needed)
sudo apt install ros-humble-slam-toolbox

# Install robot localization packages
sudo apt install ros-humble-robot-localization

# Install navigation-related tools
sudo apt install ros-humble-dwb-core ros-humble-dwb-msgs
sudo apt install ros-humble-nav-2d-msgs ros-humble-nav-2d-utils
```

### Method 2: Source Installation (For Development)

#### 1. Create Navigation Workspace

```bash
mkdir -p ~/nav2_ws/src
cd ~/nav2_ws
source /opt/ros/humble/setup.bash
```

#### 2. Clone Nav2 Repository

```bash
cd ~/nav2_ws/src

# Clone the navigation2 repository
git clone -b humble https://github.com/ros-planning/navigation2.git

# Clone navigation2_tutorials (optional, for examples)
git clone -b humble https://github.com/ros-planning/navigation2_tutorials.git
```

#### 3. Install Dependencies

```bash
cd ~/nav2_ws

# Install dependencies using rosdep
rosdep install --from-paths src --ignore-src -r -y
```

#### 4. Build the Workspace

```bash
cd ~/nav2_ws

# Build with colcon
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## Basic Configuration

### 1. Create Navigation Configuration Package

Create a new package for your navigation configurations:

```bash
cd ~/your_robot_ws/src
ros2 pkg create --dependencies rclcpp rclpy std_msgs geometry_msgs nav2_msgs -- your_robot_navigation
```

### 2. Create Configuration Files

Create the main navigation configuration file at `your_robot_navigation/config/navigation.yaml`:

```yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
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
    use_sim_time: True

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: True

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    navigate_through_poses: False
    navigate_to_pose: True
    behavior_tree_xml_filename: "navigate_w_replanning_and_recovery.xml"
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
    - nav2_globally_updated_goal_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
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
    use_sim_time: True

controller_server:
  ros__parameters:
    use_sim_time: True
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

controller_server_rclcpp_node:
  ros__parameters:
    use_sim_time: True

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
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
      always_send_full_costmap: True
  local_costmap_client:
    ros__parameters:
      use_sim_time: True
  local_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.22
      resolution: 0.05
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
  global_costmap_client:
    ros__parameters:
      use_sim_time: True
  global_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

map_server:
  ros__parameters:
    use_sim_time: True
    yaml_filename: "turtlebot3_world.yaml"

map_saver:
  ros__parameters:
    use_sim_time: True
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65
    map_subscribe_transient_local: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

planner_server_rclcpp_node:
  ros__parameters:
    use_sim_time: True

smoother_server:
  ros__parameters:
    use_sim_time: True
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      w_smooth: 0.9
      w_data: 0.1

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait", "assisted_teleop", "drive_on_heading"]
    spin:
      plugin: "nav2_behaviors::Spin"
      server_name: spin
      is_server_node: True
    backup:
      plugin: "nav2_behaviors::BackUp"
      server_name: backup
      is_server_node: True
    wait:
      plugin: "nav2_behaviors::Wait"
      server_name: wait
      is_server_node: True
    assisted_teleop:
      plugin: "nav2_behaviors::AssistedTeleop"
      server_name: assisted_teleop
      is_server_node: True
    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"
      server_name: drive_on_heading
      is_server_node: True
    global_frame: odom
    robot_base_frame: base_link
    transform_tolerance: 0.1
    use_sim_time: True
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2

waypoint_follower:
  ros__parameters:
    use_sim_time: True
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: True
      waypoint_pause_duration: 200
```

### 3. Create Launch File

Create a launch file at `your_robot_navigation/launch/navigation_launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_nav2_container = 'nav2_container'
    nav2_container = LaunchConfiguration('nav2_container')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),
        DeclareLaunchArgument(
            'nav2_container',
            default_value=default_nav2_container,
            description='The name of the nav2 container'),

        # Launch the main Nav2 node
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),
    ])
```

## Testing Nav2 Installation

### 1. Verify Installation

```bash
# Check if Nav2 packages are available
ros2 pkg list | grep nav2

# Check available Nav2 executables
ros2 run --list | grep nav2
```

### 2. Run Basic Navigation Test

```bash
# Launch the navigation system with a simple test
ros2 launch nav2_bringup tb3_simulation_launch.py

# Or with your custom configuration
ros2 launch your_robot_navigation navigation_launch.py params_file:=/path/to/your/navigation.yaml
```

### 3. Send Navigation Goals

In another terminal:

```bash
# Send a simple navigation goal
ros2 run nav2_msgs action_client /navigate_to_pose

# Or use RViz2 to send goals through the GUI
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

## Nav2 Components Overview

### 1. Planner Server
- **Purpose**: Global path planning
- **Default**: NavFn (A* variant)
- **Configuration**: `planner_server` in YAML

### 2. Controller Server
- **Purpose**: Local path following and obstacle avoidance
- **Default**: DWB (Dynamic Window Approach)
- **Configuration**: `controller_server` in YAML

### 3. Behavior Server
- **Purpose**: Recovery behaviors (backup, spin, wait)
- **Configuration**: `behavior_server` in YAML

### 4. BT Navigator
- **Purpose**: Behavior tree-based navigation orchestration
- **Configuration**: `bt_navigator` in YAML

### 5. Costmap Servers
- **Purpose**: Local and global obstacle mapping
- **Configuration**: `local_costmap` and `global_costmap` in YAML

## Integration with Isaac ROS

### 1. Perception-to-Navigation Pipeline

To integrate Isaac ROS perception with Nav2 navigation:

```yaml
# Add Isaac ROS perception nodes to your launch file
# Example: Using Isaac ROS Visual SLAM with Nav2
slam_toolbox:
  ros__parameters:
    # SLAM configuration
    use_sim_time: True
    mode: "localization"  # or "mapping"
    # ... other SLAM parameters
```

### 2. Coordinate Frame Considerations

Ensure consistent frame IDs across perception and navigation:
- `map` frame from SLAM
- `odom` frame from odometry
- `base_link` frame from robot URDF

## Performance Tuning

### 1. CPU and Memory Optimization

```yaml
# In your navigation.yaml
controller_server:
  ros__parameters:
    controller_frequency: 10.0  # Lower if CPU constrained
    # Adjust other parameters based on hardware capabilities
```

### 2. Real-time Performance

```bash
# Monitor navigation performance
ros2 topic hz /cmd_vel
ros2 topic hz /navigate_to_pose/_action/status
```

## Troubleshooting Common Issues

### Issue: Navigation nodes fail to start
**Symptoms**: Lifecycle nodes fail to activate
**Solutions**:
1. Check parameter file syntax:
   ```bash
   python3 -c "import yaml; print(yaml.safe_load(open('navigation.yaml')))"
   ```
2. Verify all required parameters are present
3. Check for typos in plugin names

### Issue: Robot doesn't move to goals
**Symptoms**: Navigation goals accepted but robot doesn't move
**Solutions**:
1. Check TF tree: `ros2 run tf2_tools view_frames`
2. Verify odometry topics: `ros2 topic echo /odom`
3. Check costmaps: `ros2 run rviz2 rviz2`

### Issue: Poor path planning
**Symptoms**: Robot takes inefficient paths or gets stuck
**Solutions**:
1. Adjust costmap inflation radius
2. Tune planner parameters (tolerance, resolution)
3. Verify sensor data quality

### Issue: Localization fails
**Symptoms**: Robot doesn't know its position
**Solutions**:
1. Check AMCL configuration
2. Verify initial pose is set correctly
3. Ensure sensor data is available and accurate

## Best Practices

### 1. Parameter Management
- Use separate parameter files for different scenarios
- Document parameter purposes and valid ranges
- Use launch arguments for runtime configuration

### 2. Testing and Validation
- Test navigation in simulation before real hardware
- Use simple environments first
- Gradually increase complexity

### 3. Safety Considerations
- Set appropriate velocity limits
- Configure proper obstacle detection
- Implement emergency stop mechanisms

## Next Steps

After setting up Nav2:

1. **Test with simulation**: Use Gazebo to validate navigation
2. **Tune parameters**: Adjust for your specific robot and environment
3. **Integrate perception**: Connect with Isaac ROS perception
4. **Add custom behaviors**: Extend with domain-specific capabilities

## Resources

- [Nav2 Documentation](https://navigation.ros.org/)
- [Nav2 Tutorials](https://navigation.ros.org/tutorials/index.html)
- [ROS 2 Navigation GitHub](https://github.com/ros-planning/navigation2)
- [Nav2 Configuration Guide](https://navigation.ros.org/configuration/index.html)

## Conclusion

This guide provides the foundation for installing and configuring the Nav2 navigation stack. Proper setup is crucial for reliable robot navigation. Take time to understand each component and adjust parameters based on your specific robot and application requirements. The combination of Nav2 with Isaac ROS perception will provide a powerful navigation system for your robotics applications.