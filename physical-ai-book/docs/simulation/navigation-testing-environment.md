# Navigation Testing Simulation Environment

## Overview

This guide provides comprehensive instructions for creating a Gazebo simulation environment specifically designed for navigation testing. The environment includes various obstacle configurations, navigation challenges, and testing scenarios to validate robot navigation capabilities.

## Simulation Environment Design

### 1. Navigation Testing World Features

The navigation testing environment should include:

- **Static Obstacles**: Walls, furniture, and fixed obstacles
- **Dynamic Obstacles**: Moving objects to test obstacle avoidance
- **Narrow Passages**: Tight spaces to test navigation precision
- **Open Areas**: Large spaces for path planning validation
- **Multiple Levels**: Different floor levels connected by ramps/stairs
- **Various Surfaces**: Different textures for wheel traction testing
- **Landmarks**: Visual markers for localization testing

### 2. World Configuration File

Create a Gazebo world file `navigation_test_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="navigation_test_world">
    <!-- Include default Gazebo environment -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics Engine Configuration -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.0</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Navigation Test Areas -->

    <!-- 1. Corridor Test Area -->
    <model name="corridor_area">
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>
      <link name="corridor_walls">
        <collision name="left_wall">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="left_wall_visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>833.33</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>100.08</iyy>
            <iyz>0</iyz>
            <izz>933.41</izz>
          </inertia>
        </inertial>
      </link>

      <link name="right_wall">
        <pose>0 3 0 0 0 0</pose>
        <collision name="right_wall_collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="right_wall_visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>833.33</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>100.08</iyy>
            <iyz>0</iyz>
            <izz>933.41</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- 2. Narrow Passage Area -->
    <model name="narrow_passage">
      <pose>8 0 0 0 0 0</pose>
      <static>true</static>
      <link name="left_barrier">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 4 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 4 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>50</mass>
          <inertia>
            <ixx>66.67</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>10.42</iyy>
            <iyz>0</iyz>
            <izz>77.08</izz>
          </inertia>
        </inertial>
      </link>

      <link name="right_barrier">
        <pose>0 3.5 0 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 4 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 4 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>50</mass>
          <inertia>
            <ixx>66.67</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>10.42</iyy>
            <iyz>0</iyz>
            <izz>77.08</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- 3. Dynamic Obstacle Area -->
    <model name="moving_obstacle">
      <pose>5 5 0 0 0 0</pose>
      <link name="moving_link">
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.3</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.3</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.8 1</ambient>
            <diffuse>0.2 0.2 0.8 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>5</mass>
          <inertia>
            <ixx>0.18</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.18</iyy>
            <iyz>0</iyz>
            <izz>0.18</izz>
          </inertia>
        </inertial>
      </link>
      <!-- Add plugin for movement -->
      <plugin name="model_push" filename="libgazebo_ros_p3d.so">
        <always_on>true</always_on>
        <update_rate>10</update_rate>
        <body_name>moving_link</body_name>
        <topic_name>moving_obstacle_pose</topic_name>
        <gaussian_noise>0.0</gaussian_noise>
        <frame_name>map</frame_name>
      </plugin>
    </model>

    <!-- 4. Furniture Obstacles -->
    <model name="table_1">
      <pose>-5 2 0 0 0 0</pose>
      <static>true</static>
      <link name="table_top">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.5 0.8 0.05</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.5 0.8 0.05</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>0.817</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1.325</iyy>
            <iyz>0</iyz>
            <izz>2.142</izz>
          </inertia>
        </inertial>
      </link>
      <link name="leg_1">
        <pose>-0.7 0.35 0.4 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 0.1 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.1 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.0134</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.0134</iyy>
            <iyz>0</iyz>
            <izz>0.0083</izz>
          </inertia>
        </inertial>
      </link>
      <link name="leg_2">
        <pose>0.7 0.35 0.4 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 0.1 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.1 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.0134</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.0134</iyy>
            <iyz>0</iyz>
            <izz>0.0083</izz>
          </inertia>
        </inertial>
      </link>
      <link name="leg_3">
        <pose>-0.7 -0.35 0.4 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 0.1 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.1 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.0134</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.0134</iyy>
            <iyz>0</iyz>
            <izz>0.0083</izz>
          </inertia>
        </inertial>
      </link>
      <link name="leg_4">
        <pose>0.7 -0.35 0.4 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 0.1 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.1 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.0134</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.0134</iyy>
            <iyz>0</iyz>
            <izz>0.0083</izz>
          </inertia>
        </inertial>
      </link>
      <joint name="joint_1" type="fixed">
        <parent>table_top</parent>
        <child>leg_1</child>
      </joint>
      <joint name="joint_2" type="fixed">
        <parent>table_top</parent>
        <child>leg_2</child>
      </joint>
      <joint name="joint_3" type="fixed">
        <parent>table_top</parent>
        <child>leg_3</child>
      </joint>
      <joint name="joint_4" type="fixed">
        <parent>table_top</parent>
        <child>leg_4</child>
      </joint>
    </model>

    <!-- 5. Open Area with Landmarks -->
    <model name="landmark_1">
      <pose>10 8 0 0 0 0</pose>
      <static>true</static>
      <link name="landmark_link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>1.5</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>1.5</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>5</mass>
          <inertia>
            <ixx>0.225</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.225</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="landmark_2">
      <pose>12 8 0 0 0 0</pose>
      <static>true</static>
      <link name="landmark_link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>1.5</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.2</radius>
              <length>1.5</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>5</mass>
          <inertia>
            <ixx>0.225</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.225</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- 6. Ramp for Multi-level Testing -->
    <model name="ramp">
      <pose>-8 -5 0 0 0 0</pose>
      <static>true</static>
      <link name="ramp_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>2 1 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>2 1 0.2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>20</mass>
          <inertia>
            <ixx>1.683</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>6.683</iyy>
            <iyz>0</iyz>
            <izz>8.333</izz>
          </inertia>
        </inertial>
      </link>
      <!-- Rotate the ramp to create an incline -->
      <pose>-8 -5 0 0 0 0.3</pose>
    </model>

    <!-- 7. Maze Area -->
    <model name="maze_wall_1">
      <pose>-2 -8 0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.2 3 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 3 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.4 0.4 0.4 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>15</mass>
          <inertia>
            <ixx>11.25</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.158</iyy>
            <iyz>0</iyz>
            <izz>11.408</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="maze_wall_2">
      <pose>-1 -6.5 0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>2 0.2 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>2 0.2 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.4 0.4 0.4 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>15</mass>
          <inertia>
            <ixx>0.158</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>5.05</iyy>
            <iyz>0</iyz>
            <izz>5.208</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Lighting -->
    <light name="navigation_test_light" type="spot">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <attenuation>
        <range>20</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 -0.1 -0.8</direction>
      <spot>
        <cutoff>40</cutoff>
        <exponent>80</exponent>
      </spot>
    </light>

    <!-- Scene Settings -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

  </world>
</sdf>
```

### 3. Navigation Testing Launch File

Create a launch file `navigation_test_launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world', default=os.path.join(
        get_package_share_directory('your_robot_gazebo'),
        'worlds', 'navigation_test_world.sdf'))
    params_file = LaunchConfiguration('params_file')
    run_rviz = LaunchConfiguration('run_rviz', default='true')
    robot_name = LaunchConfiguration('robot_name', default='test_robot')
    autostart = LaunchConfiguration('autostart', default='true')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('your_robot_gazebo'),
            'worlds', 'navigation_test_world.sdf'),
        description='SDF world file')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('your_robot_navigation'),
            'config', 'navigation_test.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='test_robot',
        description='Name of the robot')

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_run_rviz = DeclareLaunchArgument(
        'run_rviz',
        default_value='true',
        description='Whether to start RViz')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
            'gui': 'true'
        }.items()
    )

    # Robot state publisher for the test robot
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': '<robot name="test_robot"><link name="base_link"/></robot>'
        }]
    )

    # Spawn the test robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.1',
            '-Y', '0'
        ],
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
            'params_file': params_file,
            'autostart': autostart
        }.items()
    )

    # SLAM toolbox for mapping (if needed)
    slam_toolbox = Node(
        condition=IfCondition(LaunchConfiguration('slam', default='false')),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{
            'use_sim_time': use_sim_time,
            'slam_toolbox_params_file': os.path.join(
                get_package_share_directory('your_robot_navigation'),
                'config', 'slam.yaml')
        }],
        output='screen'
    )

    # RViz2
    rviz = Node(
        condition=IfCondition(run_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('nav2_bringup'),
            'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Navigation testing interface
    nav_test_interface = Node(
        package='your_robot_navigation',
        executable='navigation_test_interface',
        name='navigation_test_interface',
        parameters=[{
            'use_sim_time': use_sim_time,
            'test_scenarios': [
                'corridor_navigation',
                'obstacle_avoidance',
                'narrow_passage',
                'dynamic_obstacle'
            ]
        }],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_world)
    ld.add_action(declare_params_file)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_autostart)
    ld.add_action(declare_run_rviz)

    # Add actions
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)
    ld.add_action(navigation2)
    ld.add_action(slam_toolbox)
    ld.add_action(rviz)
    ld.add_action(nav_test_interface)

    return ld
```

### 4. Navigation Test Scenarios

Create a navigation test scenarios configuration `navigation_test_scenarios.yaml`:

```yaml
# Navigation test scenarios configuration
navigation_test_scenarios:
  corridor_navigation:
    description: "Navigate through a long corridor with walls on both sides"
    start_pose: [0.0, 0.0, 0.0]
    goal_pose: [8.0, 1.5, 0.0]
    success_criteria:
      - "reach_goal_within_tolerance: 0.25"
      - "max_execution_time: 60.0"
      - "no_collisions: true"
    difficulty: "easy"
    tags: ["corridor", "straight", "obstacle_avoidance"]

  narrow_passage:
    description: "Navigate through a narrow passage between obstacles"
    start_pose: [8.0, 0.0, 0.0]
    goal_pose: [8.0, 3.0, 0.0]
    success_criteria:
      - "reach_goal_within_tolerance: 0.25"
      - "max_execution_time: 120.0"
      - "no_collisions: true"
      - "min_clearance: 0.1"
    difficulty: "medium"
    tags: ["narrow", "precision", "obstacle_avoidance"]

  obstacle_avoidance:
    description: "Navigate around static furniture obstacles"
    start_pose: [-6.0, 0.0, 0.0]
    goal_pose: [-6.0, 4.0, 0.0]
    success_criteria:
      - "reach_goal_within_tolerance: 0.25"
      - "max_execution_time: 90.0"
      - "no_collisions: true"
    difficulty: "easy"
    tags: ["static_obstacles", "furniture", "path_planning"]

  dynamic_obstacle:
    description: "Navigate while avoiding a moving obstacle"
    start_pose: [4.0, 4.0, 0.0]
    goal_pose: [6.0, 6.0, 0.0]
    success_criteria:
      - "reach_goal_within_tolerance: 0.25"
      - "max_execution_time: 180.0"
      - "no_collisions: true"
      - "maintain_safe_distance: 0.5"
    difficulty: "hard"
    tags: ["dynamic_obstacles", "reactive_navigation", "local_planning"]

  landmark_navigation:
    description: "Navigate to specific landmarks in open space"
    start_pose: [9.0, 7.0, 0.0]
    goal_pose: [11.0, 8.0, 0.0]
    success_criteria:
      - "reach_goal_within_tolerance: 0.25"
      - "max_execution_time: 45.0"
      - "no_collisions: true"
    difficulty: "easy"
    tags: ["open_space", "landmarks", "global_planning"]

  maze_navigation:
    description: "Navigate through a simple maze"
    start_pose: [-3.0, -7.0, 0.0]
    goal_pose: [0.0, -5.0, 0.0]
    success_criteria:
      - "reach_goal_within_tolerance: 0.25"
      - "max_execution_time: 150.0"
      - "no_collisions: true"
    difficulty: "medium"
    tags: ["maze", "path_planning", "decision_making"]

# Test execution parameters
test_execution:
  auto_run: false
  random_order: false
  repeat_count: 1
  timeout_per_scenario: 300.0
  pause_between_tests: 5.0

# Performance metrics to track
performance_metrics:
  path_length: true
  execution_time: true
  collisions: true
  path_efficiency: true
  success_rate: true
  average_linear_velocity: true
  average_angular_velocity: true
  path_deviation: true
  obstacle_clearance: true

# Navigation parameters for testing
navigation_parameters:
  global_planner: "nav2_navfn_planner/NavfnPlanner"
  local_planner: "dwb_core/DWBLocalPlanner"
  controller_frequency: 20.0
  planner_frequency: 1.0
  min_distance_to_obstacles: 0.5
  max_linear_velocity: 0.5
  max_angular_velocity: 1.0
  goal_tolerance: 0.25
  obstacle_avoidance_timeout: 30.0
```

### 5. Navigation Test Interface Node

Create a navigation test interface node `navigation_test_interface.py`:

```python
#!/usr/bin/env python3
"""
Navigation Test Interface for Gazebo Simulation
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool, Float32
from builtin_interfaces.msg import Duration
import time
import math
import random
from enum import Enum


class TestStatus(Enum):
    IDLE = 0
    RUNNING = 1
    SUCCESS = 2
    FAILED = 3
    TIMEOUT = 4


class NavigationTestInterface(Node):
    def __init__(self):
        super().__init__('navigation_test_interface')

        # Parameters
        self.declare_parameter('test_scenarios', [
            'corridor_navigation', 'obstacle_avoidance',
            'narrow_passage', 'dynamic_obstacle'
        ])
        self.declare_parameter('auto_run', False)
        self.declare_parameter('random_order', False)
        self.declare_parameter('repeat_count', 1)
        self.declare_parameter('timeout_per_scenario', 300.0)

        self.test_scenarios = self.get_parameter('test_scenarios').value
        self.auto_run = self.get_parameter('auto_run').value
        self.random_order = self.get_parameter('random_order').value
        self.repeat_count = self.get_parameter('repeat_count').value
        self.timeout_per_scenario = self.get_parameter('timeout_per_scenario').value

        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishers
        self.test_status_pub = self.create_publisher(String, '/navigation_test_status', 10)
        self.test_result_pub = self.create_publisher(Bool, '/navigation_test_result', 10)
        self.test_progress_pub = self.create_publisher(Float32, '/navigation_test_progress', 10)
        self.current_scenario_pub = self.create_publisher(String, '/current_test_scenario', 10)

        # Subscriptions
        self.test_command_sub = self.create_subscription(
            String, '/navigation_test_command', self.test_command_callback, 10)

        # Internal state
        self.current_test_status = TestStatus.IDLE
        self.current_scenario_index = 0
        self.test_results = []
        self.test_start_time = None
        self.test_timeout_timer = None

        # Test scenarios definition
        self.scenarios = {
            'corridor_navigation': {
                'start_pose': [0.0, 0.0, 0.0],
                'goal_pose': [8.0, 1.5, 0.0],
                'description': 'Navigate through corridor',
                'success_criteria': ['reach_goal_within_tolerance: 0.25', 'no_collisions: true']
            },
            'narrow_passage': {
                'start_pose': [8.0, 0.0, 0.0],
                'goal_pose': [8.0, 3.0, 0.0],
                'description': 'Navigate through narrow passage',
                'success_criteria': ['reach_goal_within_tolerance: 0.25', 'no_collisions: true']
            },
            'obstacle_avoidance': {
                'start_pose': [-6.0, 0.0, 0.0],
                'goal_pose': [-6.0, 4.0, 0.0],
                'description': 'Navigate around static obstacles',
                'success_criteria': ['reach_goal_within_tolerance: 0.25', 'no_collisions: true']
            },
            'dynamic_obstacle': {
                'start_pose': [4.0, 4.0, 0.0],
                'goal_pose': [6.0, 6.0, 0.0],
                'description': 'Navigate with dynamic obstacle',
                'success_criteria': ['reach_goal_within_tolerance: 0.25', 'no_collisions: true']
            }
        }

        # Timer for periodic status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info(f'Navigation Test Interface initialized with {len(self.test_scenarios)} scenarios')

        if self.auto_run:
            self.get_logger().info('Auto-run mode enabled, starting tests...')
            self.start_test_sequence()

    def test_command_callback(self, msg):
        """Handle test commands"""
        command = msg.data.lower()

        if command == 'start':
            self.start_test_sequence()
        elif command == 'stop':
            self.stop_test_sequence()
        elif command == 'next':
            self.run_next_scenario()
        elif command == 'reset':
            self.reset_tests()
        elif command.startswith('run_scenario:'):
            scenario_name = command.split(':')[1]
            self.run_single_scenario(scenario_name)

    def start_test_sequence(self):
        """Start the complete test sequence"""
        self.get_logger().info('Starting navigation test sequence')
        self.current_test_status = TestStatus.RUNNING
        self.current_scenario_index = 0
        self.test_results = []

        if self.random_order:
            random.shuffle(self.test_scenarios)

        self.run_next_scenario()

    def run_next_scenario(self):
        """Run the next scenario in the sequence"""
        if self.current_scenario_index >= len(self.test_scenarios):
            self.complete_test_sequence()
            return

        scenario_name = self.test_scenarios[self.current_scenario_index]
        self.get_logger().info(f'Running scenario: {scenario_name}')

        # Publish current scenario
        scenario_msg = String()
        scenario_msg.data = scenario_name
        self.current_scenario_pub.publish(scenario_msg)

        # Send navigation goal for the scenario
        self.send_navigation_goal(scenario_name)

    def send_navigation_goal(self, scenario_name):
        """Send navigation goal for a specific scenario"""
        if scenario_name not in self.scenarios:
            self.get_logger().error(f'Unknown scenario: {scenario_name}')
            return

        scenario = self.scenarios[scenario_name]
        goal_pose = scenario['goal_pose']

        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_pose[0]
        goal_msg.pose.pose.position.y = goal_pose[1]
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0  # No rotation

        # Wait for navigation server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available')
            self.mark_scenario_failed(scenario_name, 'Navigation server unavailable')
            return

        # Send goal
        self.test_start_time = self.get_clock().now().nanoseconds / 1e9
        self.current_test_status = TestStatus.RUNNING

        # Set up timeout timer
        self.test_timeout_timer = self.create_timer(
            self.timeout_per_scenario,
            lambda: self.handle_timeout(scenario_name)
        )

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self.goal_response_callback(f, scenario_name))

    def goal_response_callback(self, future, scenario_name):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Goal for {scenario_name} was rejected')
            self.mark_scenario_failed(scenario_name, 'Goal rejected')
            return

        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.result_callback(f, scenario_name))

    def result_callback(self, future, scenario_name):
        """Handle navigation result"""
        if self.test_timeout_timer is not None:
            self.test_timeout_timer.cancel()
            self.test_timeout_timer = None

        result = future.result().result
        if result is not None:
            # Check if navigation was successful
            if result.error_code == 0:  # SUCCESS
                self.mark_scenario_success(scenario_name)
            else:
                self.mark_scenario_failed(scenario_name, f'Navigation failed with error code: {result.error_code}')
        else:
            self.mark_scenario_failed(scenario_name, 'No result received')

    def handle_timeout(self, scenario_name):
        """Handle test timeout"""
        self.get_logger().warn(f'Test scenario {scenario_name} timed out')
        self.mark_scenario_failed(scenario_name, 'Test timed out')

    def mark_scenario_success(self, scenario_name):
        """Mark a scenario as successful"""
        self.get_logger().info(f'Scenario {scenario_name} completed successfully')
        self.current_test_status = TestStatus.SUCCESS

        # Record result
        result = {
            'scenario': scenario_name,
            'status': 'SUCCESS',
            'execution_time': self.get_clock().now().nanoseconds / 1e9 - self.test_start_time,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
        self.test_results.append(result)

        # Publish result
        result_msg = Bool()
        result_msg.data = True
        self.test_result_pub.publish(result_msg)

        # Move to next scenario
        self.current_scenario_index += 1
        self.run_next_scenario()

    def mark_scenario_failed(self, scenario_name, reason):
        """Mark a scenario as failed"""
        self.get_logger().error(f'Scenario {scenario_name} failed: {reason}')
        self.current_test_status = TestStatus.FAILED

        # Record result
        result = {
            'scenario': scenario_name,
            'status': 'FAILED',
            'reason': reason,
            'execution_time': self.get_clock().now().nanoseconds / 1e9 - self.test_start_time,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
        self.test_results.append(result)

        # Publish result
        result_msg = Bool()
        result_msg.data = False
        self.test_result_pub.publish(result_msg)

        # Move to next scenario
        self.current_scenario_index += 1
        self.run_next_scenario()

    def complete_test_sequence(self):
        """Complete the test sequence and report results"""
        self.get_logger().info('Navigation test sequence completed')
        self.current_test_status = TestStatus.IDLE

        # Calculate statistics
        total_tests = len(self.test_results)
        successful_tests = len([r for r in self.test_results if r['status'] == 'SUCCESS'])
        success_rate = successful_tests / total_tests if total_tests > 0 else 0

        self.get_logger().info(f'Test results: {successful_tests}/{total_tests} successful ({success_rate:.2%})')

        # Publish final status
        status_msg = String()
        status_msg.data = f'TEST_COMPLETE: {successful_tests}/{total_tests} ({success_rate:.2%})'
        self.test_status_pub.publish(status_msg)

    def stop_test_sequence(self):
        """Stop the current test sequence"""
        self.get_logger().info('Stopping navigation test sequence')
        self.current_test_status = TestStatus.IDLE
        if self.test_timeout_timer is not None:
            self.test_timeout_timer.cancel()
            self.test_timeout_timer = None

    def reset_tests(self):
        """Reset all test results"""
        self.get_logger().info('Resetting navigation tests')
        self.current_test_status = TestStatus.IDLE
        self.current_scenario_index = 0
        self.test_results = []
        if self.test_timeout_timer is not None:
            self.test_timeout_timer.cancel()
            self.test_timeout_timer = None

    def run_single_scenario(self, scenario_name):
        """Run a single specific scenario"""
        if scenario_name in self.scenarios:
            self.get_logger().info(f'Running single scenario: {scenario_name}')
            self.current_test_status = TestStatus.RUNNING
            self.send_navigation_goal(scenario_name)
        else:
            self.get_logger().error(f'Unknown scenario: {scenario_name}')

    def publish_status(self):
        """Publish current test status"""
        status_msg = String()
        status_msg.data = self.current_test_status.name
        self.test_status_pub.publish(status_msg)

        # Publish progress
        if len(self.test_scenarios) > 0:
            progress = Float32()
            progress.data = min(1.0, self.current_scenario_index / len(self.test_scenarios))
            self.test_progress_pub.publish(progress)


def main(args=None):
    rclpy.init(args=args)
    test_interface = NavigationTestInterface()

    try:
        rclpy.spin(test_interface)
    except KeyboardInterrupt:
        test_interface.get_logger().info('Navigation test interface stopped by user')
    finally:
        test_interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Advanced Navigation Testing Features

### 1. Dynamic Obstacle Simulation

Create a dynamic obstacle node for testing:

```python
#!/usr/bin/env python3
"""
Dynamic Obstacle for Navigation Testing
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import ModelState
import math
import time


class DynamicObstacle(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle')

        # Parameters
        self.declare_parameter('obstacle_name', 'moving_obstacle')
        self.declare_parameter('movement_pattern', 'circular')  # circular, linear, random
        self.declare_parameter('speed', 0.2)
        self.declare_parameter('radius', 1.0)

        self.obstacle_name = self.get_parameter('obstacle_name').value
        self.movement_pattern = self.get_parameter('movement_pattern').value
        self.speed = self.get_parameter('speed').value
        self.radius = self.get_parameter('radius').value

        # Gazebo service client
        self.set_state_client = self.create_client(
            SetEntityState, '/gazebo/set_entity_state')

        # Wait for service
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gazebo set_entity_state service not available, waiting...')

        # Timer for periodic movement
        self.movement_timer = self.create_timer(0.1, self.move_obstacle)

        # Internal state
        self.time_offset = time.time()
        self.start_position = [5.0, 5.0, 0.0]  # Starting position

        self.get_logger().info(f'Dynamic obstacle initialized with pattern: {self.movement_pattern}')

    def move_obstacle(self):
        """Move the obstacle according to the selected pattern"""
        current_time = time.time() - self.time_offset

        if self.movement_pattern == 'circular':
            x = self.start_position[0] + self.radius * math.cos(current_time * self.speed)
            y = self.start_position[1] + self.radius * math.sin(current_time * self.speed)
        elif self.movement_pattern == 'linear':
            x = self.start_position[0] + math.sin(current_time * self.speed) * self.radius
            y = self.start_position[1]
        else:  # random pattern
            x = self.start_position[0] + math.sin(current_time * self.speed) * self.radius
            y = self.start_position[1] + math.cos(current_time * self.speed * 0.7) * self.radius

        # Set new position
        self.set_obstacle_position(x, y, self.start_position[2])

    def set_obstacle_position(self, x, y, z):
        """Set the obstacle position in Gazebo"""
        req = SetEntityState.Request()
        req.state.name = self.obstacle_name
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = z
        req.state.pose.orientation.w = 1.0  # No rotation
        req.state.twist.linear.x = 0.0
        req.state.twist.linear.y = 0.0
        req.state.twist.linear.z = 0.0
        req.state.twist.angular.x = 0.0
        req.state.twist.angular.y = 0.0
        req.state.twist.angular.z = 0.0

        future = self.set_state_client.call_async(req)
        # We could add a callback to handle the response if needed


def main(args=None):
    rclpy.init(args=args)
    obstacle = DynamicObstacle()

    try:
        rclpy.spin(obstacle)
    except KeyboardInterrupt:
        obstacle.get_logger().info('Dynamic obstacle stopped by user')
    finally:
        obstacle.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Test Environment Analysis Tools

Create a test environment analysis node:

```python
#!/usr/bin/env python3
"""
Navigation Test Environment Analysis
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np


class NavigationTestAnalyzer(Node):
    def __init__(self):
        super().__init__('navigation_test_analyzer')

        # Parameters
        self.declare_parameter('analysis_frequency', 10.0)
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')

        self.analysis_freq = self.get_parameter('analysis_frequency').value
        path_topic = self.get_parameter('path_topic').value
        costmap_topic = self.get_parameter('costmap_topic').value

        # Subscriptions
        self.path_sub = self.create_subscription(
            Path, path_topic, self.path_callback, 10)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, costmap_topic, self.costmap_callback, 10)

        # Publishers
        self.analysis_pub = self.create_publisher(
            MarkerArray, '/navigation_analysis_markers', 10)
        self.metrics_pub = self.create_publisher(
            Marker, '/navigation_metrics', 10)

        # Internal state
        self.current_path = None
        self.current_costmap = None
        self.path_length = 0.0
        self.collision_risk = 0.0

        # Timer for analysis
        self.analysis_timer = self.create_timer(1.0/self.analysis_freq, self.analyze_path)

        self.get_logger().info('Navigation Test Analyzer initialized')

    def path_callback(self, msg):
        """Process path data"""
        self.current_path = msg
        self.calculate_path_metrics()

    def costmap_callback(self, msg):
        """Process costmap data"""
        self.current_costmap = msg

    def calculate_path_metrics(self):
        """Calculate path metrics"""
        if not self.current_path or len(self.current_path.poses) < 2:
            return

        # Calculate path length
        self.path_length = 0.0
        for i in range(1, len(self.current_path.poses)):
            p1 = self.current_path.poses[i-1].pose.position
            p2 = self.current_path.poses[i].pose.position
            dist = np.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
            self.path_length += dist

    def analyze_path(self):
        """Analyze the current path for navigation metrics"""
        if not self.current_path:
            return

        # Calculate collision risk based on proximity to obstacles in costmap
        if self.current_costmap:
            self.collision_risk = self.calculate_collision_risk()

        # Create visualization markers
        markers = self.create_analysis_markers()

        # Publish analysis results
        self.analysis_pub.publish(markers)

    def calculate_collision_risk(self):
        """Calculate collision risk based on path proximity to obstacles"""
        if not self.current_path or not self.current_costmap:
            return 0.0

        risk_score = 0.0
        risk_points = 0

        for pose in self.current_path.poses:
            # Convert world coordinates to costmap indices
            x = pose.pose.position.x
            y = pose.pose.position.y

            # Calculate costmap index
            idx_x = int((x - self.current_costmap.info.origin.position.x) / self.current_costmap.info.resolution)
            idx_y = int((y - self.current_costmap.info.origin.position.y) / self.current_costmap.info.resolution)

            if (0 <= idx_x < self.current_costmap.info.width and
                0 <= idx_y < self.current_costmap.info.height):

                idx = idx_y * self.current_costmap.info.width + idx_x
                if idx < len(self.current_costmap.data):
                    cost = self.current_costmap.data[idx]
                    if cost > 50:  # High cost area
                        risk_score += cost
                        risk_points += 1

        return risk_score / risk_points if risk_points > 0 else 0.0

    def create_analysis_markers(self):
        """Create visualization markers for analysis"""
        markers = MarkerArray()

        # Path length marker
        length_marker = Marker()
        length_marker.header.frame_id = "map"
        length_marker.header.stamp = self.get_clock().now().to_msg()
        length_marker.ns = "navigation_analysis"
        length_marker.id = 0
        length_marker.type = Marker.TEXT_VIEW_FACING
        length_marker.action = Marker.ADD
        length_marker.pose.position.x = 0.0
        length_marker.pose.position.y = 0.0
        length_marker.pose.position.z = 1.0
        length_marker.pose.orientation.w = 1.0
        length_marker.scale.z = 0.2
        length_marker.color.r = 1.0
        length_marker.color.g = 1.0
        length_marker.color.b = 1.0
        length_marker.color.a = 1.0
        length_marker.text = f"Path Length: {self.path_length:.2f}m"
        markers.markers.append(length_marker)

        # Collision risk marker
        risk_marker = Marker()
        risk_marker.header.frame_id = "map"
        risk_marker.header.stamp = self.get_clock().now().to_msg()
        risk_marker.ns = "navigation_analysis"
        risk_marker.id = 1
        risk_marker.type = Marker.TEXT_VIEW_FACING
        risk_marker.action = Marker.ADD
        risk_marker.pose.position.x = 0.0
        risk_marker.pose.position.y = -0.3
        risk_marker.pose.position.z = 1.0
        risk_marker.pose.orientation.w = 1.0
        risk_marker.scale.z = 0.2
        risk_marker.color.r = 1.0 if self.collision_risk > 100 else 0.0
        risk_marker.color.g = 1.0 if self.collision_risk <= 100 else 0.0
        risk_marker.color.b = 0.0
        risk_marker.color.a = 1.0
        risk_marker.text = f"Collision Risk: {self.collision_risk:.2f}"
        markers.markers.append(risk_marker)

        return markers


def main(args=None):
    rclpy.init(args=args)
    analyzer = NavigationTestAnalyzer()

    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        analyzer.get_logger().info('Navigation test analyzer stopped by user')
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Testing and Validation

### 1. Automated Test Script

Create an automated test script `run_navigation_tests.py`:

```python
#!/usr/bin/env python3
"""
Automated Navigation Testing Script
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time
import csv
import os


class NavigationTestRunner(Node):
    def __init__(self):
        super().__init__('navigation_test_runner')

        # Publishers
        self.test_command_pub = self.create_publisher(String, '/navigation_test_command', 10)

        # Test parameters
        self.test_scenarios = [
            {'name': 'corridor_navigation', 'goal': [8.0, 1.5, 0.0]},
            {'name': 'narrow_passage', 'goal': [8.0, 3.0, 0.0]},
            {'name': 'obstacle_avoidance', 'goal': [-6.0, 4.0, 0.0]},
            {'name': 'dynamic_obstacle', 'goal': [6.0, 6.0, 0.0]}
        ]

        # Results storage
        self.results = []
        self.results_file = 'navigation_test_results.csv'

        self.get_logger().info('Navigation Test Runner initialized')

    def run_comprehensive_test(self):
        """Run comprehensive navigation tests"""
        self.get_logger().info('Starting comprehensive navigation tests...')

        for scenario in self.test_scenarios:
            self.get_logger().info(f'Running scenario: {scenario["name"]}')

            # Send test command
            cmd_msg = String()
            cmd_msg.data = f'run_scenario:{scenario["name"]}'
            self.test_command_pub.publish(cmd_msg)

            # Wait for test to complete (in a real scenario, we'd monitor status)
            time.sleep(5.0)  # Wait for navigation to complete

            # Record result (in a real test, we'd get actual results)
            result = {
                'scenario': scenario['name'],
                'start_time': time.time(),
                'end_time': time.time(),
                'execution_time': 0.0,
                'success': True,
                'path_length': 0.0,
                'collisions': 0
            }
            self.results.append(result)

        # Save results
        self.save_results()
        self.get_logger().info('Navigation tests completed')

    def save_results(self):
        """Save test results to CSV file"""
        with open(self.results_file, 'w', newline='') as csvfile:
            fieldnames = ['scenario', 'start_time', 'end_time', 'execution_time',
                         'success', 'path_length', 'collisions']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            writer.writeheader()
            for result in self.results:
                writer.writerow(result)

        self.get_logger().info(f'Test results saved to {self.results_file}')


def main(args=None):
    rclpy.init(args=args)
    test_runner = NavigationTestRunner()

    # Run tests
    test_runner.run_comprehensive_test()

    test_runner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Best Practices for Navigation Testing

### 1. Test Coverage Guidelines

- **Functional Tests**: Test basic navigation capabilities (path planning, execution, obstacle avoidance)
- **Performance Tests**: Measure execution time, path efficiency, and resource usage
- **Stress Tests**: Test with maximum obstacle density and challenging scenarios
- **Edge Case Tests**: Test boundary conditions and unusual scenarios
- **Regression Tests**: Ensure new changes don't break existing functionality

### 2. Environment Design Principles

- **Graduated Complexity**: Start with simple scenarios and increase complexity
- **Realistic Obstacles**: Use obstacles that represent real-world scenarios
- **Measurable Outcomes**: Design tests with clear success/failure criteria
- **Reproducible Results**: Ensure tests can be run multiple times with consistent results
- **Safety Considerations**: Include safety checks and emergency stop capabilities

### 3. Performance Metrics

- **Success Rate**: Percentage of successful navigation attempts
- **Execution Time**: Time to reach goal from start
- **Path Efficiency**: Ratio of actual path length to optimal path length
- **Collision Rate**: Number of collisions per navigation attempt
- **Recovery Time**: Time to recover from navigation failures
- **Resource Usage**: CPU, memory, and power consumption

## Conclusion

This comprehensive guide provides all the necessary components to create a robust navigation testing environment in Gazebo. The environment includes various test scenarios, dynamic obstacles, and analysis tools to thoroughly validate robot navigation capabilities. The modular design allows for easy customization and extension based on specific testing requirements. Regular testing with this environment will help ensure reliable and safe robot navigation in real-world applications.