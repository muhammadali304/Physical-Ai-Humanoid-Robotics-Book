---
sidebar_position: 3
---

# URDF Modeling - Robot Description Format

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the Unified Robot Description Format (URDF) structure and syntax
- Create complex robot models with multiple links and joints
- Add visual, collision, and inertial properties to robot models
- Include sensors and actuators in URDF descriptions
- Validate and debug URDF models
- Use Xacro macros to simplify complex robot descriptions

## Prerequisites

Before starting this chapter, you should:
- Understand ROS 2 nodes, topics, and packages
- Have basic knowledge of 3D geometry and coordinate systems
- Completed the simulation chapters (Gazebo and Unity)
- Basic familiarity with XML syntax

## Conceptual Overview

**URDF (Unified Robot Description Format)** is an XML-based format used to describe robots in ROS. It defines the physical and visual properties of a robot including:

- **Links**: Rigid parts of the robot (e.g., chassis, wheels, arms)
- **Joints**: Connections between links (e.g., revolute, prismatic, fixed)
- **Visual properties**: How the robot appears in simulation
- **Collision properties**: How the robot interacts physically
- **Inertial properties**: Mass, center of mass, and inertia for physics simulation
- **Sensors and actuators**: For simulation and control

### URDF Structure

A URDF file typically contains:
- Robot name and basic properties
- Link definitions (visual, collision, inertial)
- Joint definitions (parent-child relationships)
- Material definitions
- Transmission elements (for control)

### Why URDF is Important

URDF is crucial for:
- **Simulation**: Accurate physics and visualization in Gazebo/Unity
- **Robot State Publishing**: TF transforms for coordinate frames
- **Motion Planning**: MoveIt! uses URDF for kinematic models
- **Control**: Joint limits and dynamics for controllers
- **Visualization**: RViz displays robot models in proper poses

## Hands-On Implementation

### Basic URDF Structure

Let's start with a simple URDF example that includes the essential elements:

**simple_robot.urdf:**

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link - the main body of the robot -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- A simple wheel -->
  <link name="wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Joint connecting wheel to base -->
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel"/>
    <origin xyz="0 0.2 -0.05" rpy="1.5707 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

### Understanding Link Elements

#### Visual Element
The `<visual>` element defines how the link appears in visualization:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1 1 1"/>
    <!-- Other options: <sphere radius="1"/>, <cylinder radius="1" length="1"/> -->
  </geometry>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
</visual>
```

#### Collision Element
The `<collision>` element defines the physical boundaries for collision detection:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1 1 1"/>
  </geometry>
</collision>
```

#### Inertial Element
The `<inertial>` element defines physical properties for simulation:

```xml
<inertial>
  <mass value="1.0"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
</inertial>
```

### Joint Types and Properties

URDF supports several joint types:

1. **Fixed**: No movement (0 DOF)
2. **Revolute**: Rotational joint with limits (1 DOF)
3. **Continuous**: Rotational joint without limits (1 DOF)
4. **Prismatic**: Linear sliding joint (1 DOF)
5. **Planar**: Movement in a plane (2 DOF)
6. **Floating**: Free movement in 3D space (6 DOF)

#### Example Joint Definitions

```xml
<!-- Fixed joint (no movement) -->
<joint name="fixed_joint" type="fixed">
  <parent link="base_link"/>
  <child link="sensor_mount"/>
  <origin xyz="0.1 0 0.1" rpy="0 0 0"/>
</joint>

<!-- Revolute joint (rotational with limits) -->
<joint name="arm_joint" type="revolute">
  <parent link="base_link"/>
  <child link="arm_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
</joint>

<!-- Continuous joint (rotational without limits) -->
<joint name="wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="wheel"/>
  <origin xyz="0 0.2 -0.05" rpy="1.5707 0 0"/>
  <axis xyz="0 0 1"/>
</joint>

<!-- Prismatic joint (linear sliding) -->
<joint name="slider_joint" type="prismatic">
  <parent link="base_link"/>
  <child link="slider"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="0.5" effort="100.0" velocity="0.5"/>
</joint>
```

### Creating a More Complex Robot Model

Let's create a differential drive robot with sensors:

**diff_robot.urdf:**

```xml
<?xml version="1.0"?>
<robot name="diff_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Robot base -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.3"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Left wheel joint -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 -0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Right wheel joint -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.15 -0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Camera link -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.03"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Camera joint -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Hokuyo Laser Scanner -->
  <link name="laser_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Laser joint -->
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_link"/>
    <origin xyz="0.15 0 0.15" rpy="0 0 0"/>
  </joint>
</robot>
```

### Adding Sensors to URDF

To include sensors in your URDF, you typically add Gazebo plugins:

**diff_robot_with_sensors.urdf (with Gazebo plugins):**

```xml
<?xml version="1.0"?>
<robot name="diff_robot_with_sensors" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Include the previous robot definition -->
  <!-- ... (previous robot definition here) ... -->

  <!-- Gazebo plugin for differential drive -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>diff_robot</namespace>
        <remapping>cmd_vel:=cmd_vel</remapping>
        <remapping>odom:=odom</remapping>
      </ros>
      <update_rate>30</update_rate>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.1</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>

  <!-- Gazebo plugin for camera -->
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <update_rate>30</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>600</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>diff_robot</namespace>
          <remapping>image_raw:=camera/image_raw</remapping>
          <remapping>camera_info:=camera/camera_info</remapping>
        </ros>
        <camera_name>camera</camera_name>
        <frame_name>camera_link</frame_name>
        <hack_baseline>0.07</hack_baseline>
        <distortion_k1>0.0</distortion_k1>
        <distortion_k2>0.0</distortion_k2>
        <distortion_k3>0.0</distortion_k3>
        <distortion_t1>0.0</distortion_t1>
        <distortion_t2>0.0</distortion_t2>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Gazebo plugin for laser scanner -->
  <gazebo reference="laser_link">
    <sensor name="laser" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>40</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-1.570796</min_angle>
            <max_angle>1.570796</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.10</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>diff_robot</namespace>
          <remapping>scan:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>laser_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

### Using Xacro for Complex Models

Xacro is a macro language for URDF that allows for:
- Parameterization
- Reusable components
- Cleaner code

**Create a wheel macro (wheel.xacro):**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="wheel" params="prefix parent *origin radius length mass">
    <link name="${prefix}_wheel">
      <visual>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${mass}"/>
        <origin xyz="0 0 0"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <xacro:insert_block name="origin"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>
</robot>
```

**Use the macro in your main URDF:**

```xml
<?xml version="1.0"?>
<robot name="diff_robot_xacro" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Include the wheel macro -->
  <xacro:include filename="wheel.xacro"/>

  <!-- Constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Robot base -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.3"/>
    </inertial>
  </link>

  <!-- Use the wheel macro for left wheel -->
  <xacro:wheel prefix="left" parent="base_link" radius="0.05" length="0.04" mass="0.5">
    <origin xyz="0 0.15 -0.05" rpy="0 0 0"/>
  </xacro:wheel>

  <!-- Use the wheel macro for right wheel -->
  <xacro:wheel prefix="right" parent="base_link" radius="0.05" length="0.04" mass="0.5">
    <origin xyz="0 -0.15 -0.05" rpy="0 0 0"/>
  </xacro:wheel>
</robot>
```

### Creating a Complete Robot Package

Let's create a proper ROS 2 package for our robot model:

1. **Create the package:**
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake robot_description
```

2. **Create the URDF directory structure:**
```bash
mkdir -p ~/ros2_ws/src/robot_description/urdf
mkdir -p ~/ros2_ws/src/robot_description/meshes
mkdir -p ~/ros2_ws/src/robot_description/config
mkdir -p ~/ros2_ws/src/robot_description/launch
```

3. **Add your URDF files to the urdf directory**

4. **Update package.xml:**
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_description</name>
  <version>0.0.1</version>
  <description>URDF description for a differential drive robot</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>xacro</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>rviz2</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

5. **Create a launch file to visualize the robot:**

**robot_description/launch/robot_display.launch.py:**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true.'
        )
    )

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get URDF via xacro
    robot_description_content = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'urdf',
        'diff_robot.urdf.xacro'  # assuming your file is named this way
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description_content}
        ]
    )

    # Joint state publisher for visualization
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz2 for visualization
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'config',
        'robot_display.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    nodes = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes)
```

## Testing & Verification

### Validating URDF Files

1. **Check URDF syntax:**
```bash
# Check if URDF is well-formed
check_urdf $(ros2 pkg prefix robot_description)/share/robot_description/urdf/diff_robot.urdf
```

2. **Visualize the robot:**
```bash
# Source your workspace
cd ~/ros2_ws
source install/setup.bash

# Launch the display
ros2 launch robot_description robot_display.launch.py
```

3. **Use the URDF spawner tool:**
```bash
# Install if not already installed
sudo apt install ros-humble-urdf-tutorial

# Parse and check the URDF
ros2 run urdf_parser_py display_urdf /path/to/your/robot.urdf
```

### Useful URDF Commands

- **List robot joints:**
```bash
ros2 param list | grep joint
```

- **Check TF tree:**
```bash
ros2 run tf2_tools view_frames
```

- **Echo joint states:**
```bash
ros2 topic echo /joint_states
```

### Common Validation Steps

1. **Check for proper XML syntax**
2. **Verify all joint parent/child relationships**
3. **Ensure all referenced files exist (meshes, etc.)**
4. **Validate mass and inertia values**
5. **Check for proper coordinate frame definitions**

## Common Issues

### Issue: URDF fails to load or parse
**Solution**:
- Check for proper XML syntax (all tags closed properly)
- Verify all referenced files exist
- Ensure proper file permissions
- Use `check_urdf` to identify specific issues

### Issue: Robot appears distorted or with wrong proportions
**Solution**:
- Verify units are consistent (usually meters for length)
- Check that geometry dimensions are correct
- Ensure origins and rotations are properly defined

### Issue: Physics simulation is unstable
**Solution**:
- Verify inertial properties are physically realistic
- Check mass values are appropriate
- Ensure center of mass is correctly positioned
- Use proper moment of inertia values

### Issue: Joint limits not working in simulation
**Solution**:
- Verify joint type matches intended movement
- Check that limits are properly defined in URDF
- Ensure Gazebo plugins are configured correctly

## Key Takeaways

- URDF is the standard format for robot descriptions in ROS
- Proper URDF structure includes links, joints, and physical properties
- Xacro simplifies complex robot models with macros and parameters
- Gazebo plugins integrate sensors and actuators with simulation
- Validation tools help identify and fix URDF issues
- Well-structured URDF is essential for simulation, visualization, and control

## Next Steps

In the next chapter, you'll learn about the NVIDIA Isaac platform, which provides advanced tools for robotics perception, navigation, and AI integration.