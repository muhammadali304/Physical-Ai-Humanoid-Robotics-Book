---
sidebar_position: 1
---

# Gazebo Simulation - Basics

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the Gazebo simulation environment and its components
- Install and configure Gazebo for ROS 2 integration
- Create basic simulation worlds with objects and environments
- Spawn and control robots in simulation
- Integrate Gazebo with ROS 2 nodes for sensor simulation
- Test robotics algorithms in a safe virtual environment

## Prerequisites

Before starting this chapter, you should:
- Have ROS 2 Humble Hawksbill installed and configured
- Understand ROS 2 nodes, topics, and packages
- Have basic knowledge of Linux command line operations
- Completed the ROS 2 fundamentals chapters

## Conceptual Overview

**Gazebo** is a 3D simulation environment that provides realistic physics simulation, high-quality graphics, and convenient programmatic interfaces. It's widely used in robotics research and development for testing algorithms before deploying to real hardware.

### Key Components of Gazebo

- **Physics Engine**: Provides realistic simulation of rigid body dynamics
- **Rendering Engine**: Creates visual representations of the environment
- **Sensor Simulation**: Emulates real-world sensors like cameras, LIDAR, IMU
- **Plugins**: Extend functionality through custom code
- **World Editor**: Create and modify simulation environments

### Integration with ROS 2

Gazebo integrates seamlessly with ROS 2 through:
- **Gazebo ROS packages**: Bridge between Gazebo and ROS 2
- **Sensor plugins**: Publish sensor data to ROS 2 topics
- **Actuator plugins**: Subscribe to ROS 2 topics to control simulated robots
- **Launch files**: Start Gazebo and ROS 2 nodes together

### Benefits of Simulation

- **Safety**: Test algorithms without risk to hardware
- **Cost-effective**: No physical hardware required
- **Repeatability**: Consistent testing conditions
- **Speed**: Faster than real-time execution possible
- **Flexibility**: Easy to modify environments and scenarios

## Hands-On Implementation

### Installing Gazebo

For ROS 2 Humble, we'll use Gazebo Garden (formerly Ignition):

```bash
# Add the Gazebo repository
sudo apt update && sudo apt install wget
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-gazebo-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update package lists
sudo apt update

# Install Gazebo Garden
sudo apt install gazebo-garden
```

### Installing Gazebo ROS Packages

```bash
# Install ROS 2 Gazebo packages
sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-plugins ros-humble-gazebo-dev
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros-control
```

### Basic Gazebo Commands

1. **Start Gazebo GUI:**
```bash
gz sim
```

2. **Start Gazebo with a world:**
```bash
gz sim -r empty.sdf
```

3. **List running Gazebo instances:**
```bash
gz service -s /gazebo/worlds
```

### Creating a Simple World

Let's create a basic world file with some objects:

**Create a world file (my_world.sdf):**

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_world">
    <!-- Include a default lighting configuration -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Add a simple box object -->
    <model name="box">
      <pose>2 2 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
            <specular>1 0 0 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Add a cylinder object -->
    <model name="cylinder">
      <pose>-2 2 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>2</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>2</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
            <specular>0 1 0 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Add a sphere object -->
    <model name="sphere">
      <pose>0 -2 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.5</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.5</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
            <specular>0 0 1 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Running Gazebo with Your World

1. **Save the world file** as `my_world.sdf` in your home directory

2. **Start Gazebo with your world:**
```bash
gz sim -r ~/my_world.sdf
```

### Creating a Simple Robot Model (URDF)

Let's create a simple differential drive robot:

**Create a URDF file (simple_robot.urdf):**

```xml
<?xml version="1.0"?>
<robot name="simple_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base link -->
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

  <!-- Left wheel -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.2 -0.05" rpy="1.5707 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.2 -0.05" rpy="1.5707 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Add a simple camera sensor -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.05" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>
</robot>
```

### Converting URDF to SDF for Gazebo

To use URDF in Gazebo, we often need to convert it to SDF format or use ROS 2 integration:

```bash
# Install urdf2gazebo if available, or use xacro to preprocess
sudo apt install ros-humble-xacro
```

### Creating a Gazebo Plugin for ROS 2 Integration

Let's create a simple launch file that starts Gazebo with our robot:

**Create a launch directory in your package:**
```bash
mkdir -p ~/ros2_ws/src/robot_controller/launch
```

**Create a launch file (robot_controller/launch/gazebo_simulation.launch.py):**

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Start Gazebo with a world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': '-r empty.sdf'  # Use empty world initially
        }.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'simple_robot',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[PathJoinSubstitution([
            FindPackageShare('robot_controller'),
            'urdf',
            'simple_robot.urdf'
        ])]
    )

    # Joint state publisher (for visualization)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity
    ])
```

### Creating a ROS 2 Node for Robot Control

Let's create a simple controller node that can drive the robot:

**Create a controller node (robot_controller/robot_controller/gazebo_controller.py):**

```python
#!/usr/bin/env python3

"""
Simple controller for Gazebo simulation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class GazeboController(Node):
    """
    A simple controller for the simulated robot.
    """

    def __init__(self):
        super().__init__('gazebo_controller')

        # Create publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        # Create subscriber for laser scan data
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # Create subscriber for odometry
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Robot state
        self.scan_data = None
        self.odom_data = None
        self.obstacle_detected = False

        self.get_logger().info('Gazebo controller initialized')

    def scan_callback(self, msg):
        """Callback for laser scan data."""
        self.scan_data = msg
        # Simple obstacle detection: check if anything is closer than 1 meter
        if min(msg.ranges) < 1.0:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def odom_callback(self, msg):
        """Callback for odometry data."""
        self.odom_data = msg

    def control_loop(self):
        """Main control loop."""
        msg = Twist()

        if self.obstacle_detected:
            # Stop and turn if obstacle detected
            msg.linear.x = 0.0
            msg.angular.z = 0.5  # Turn right
        else:
            # Move forward if no obstacle
            msg.linear.x = 0.5
            msg.angular.z = 0.0

        self.cmd_vel_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    controller = GazeboController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Controller stopped by user')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Creating a URDF Directory and Adding Your Robot

```bash
mkdir -p ~/ros2_ws/src/robot_controller/urdf
```

Save your URDF file as `~/ros2_ws/src/robot_controller/urdf/simple_robot.urdf`

### Updating package.xml

Add dependencies to your `package.xml`:

```xml
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>nav_msgs</depend>
<depend>ros_gz_sim</depend>
<depend>ros_gz_interfaces</depend>
<depend>robot_state_publisher</depend>
<depend>joint_state_publisher</depend>
```

## Testing & Verification

### Running the Simulation

1. **Build your package:**
```bash
cd ~/ros2_ws
colcon build --packages-select robot_controller
source install/setup.bash
```

2. **Start Gazebo with your robot:**
```bash
# Terminal 1: Start Gazebo
ros2 launch robot_controller gazebo_simulation.launch.py
```

3. **In another terminal, run the controller:**
```bash
ros2 run robot_controller gazebo_controller
```

4. **Or send manual commands:**
```bash
# Terminal 3: Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
```

### Useful Gazebo Commands

- **List Gazebo services:**
```bash
gz service -s
```

- **Get world information:**
```bash
gz service -s /gazebo/worlds --req-type gz.msgs.Empty --rep-type gz.msgs.StringMsg_V
```

- **Get model information:**
```bash
gz topic -e -t /world/empty/model/status
```

- **Spawn a model:**
```bash
gz service -s /world/empty/create --req-type gz.msgs.EntityFactory --req 'sdf_filename: "model://cylinder"'
```

### ROS 2 Integration Commands

- **List topics published by Gazebo:**
```bash
ros2 topic list | grep gz
```

- **Echo sensor data:**
```bash
ros2 topic echo /scan sensor_msgs/msg/LaserScan
```

- **Check TF tree:**
```bash
ros2 run tf2_tools view_frames
```

## Common Issues

### Issue: Gazebo doesn't start or crashes
**Solution**:
- Check if your graphics drivers are properly installed
- Try running with software rendering: `MESA_GL_VERSION_OVERRIDE=3.3 gz sim`
- Ensure sufficient system resources (RAM, GPU)

### Issue: Robot doesn't appear in Gazebo
**Solution**:
- Verify the URDF/SDF file is correctly formatted
- Check that the robot model is being published to `/robot_description`
- Ensure the spawn command has correct parameters

### Issue: No sensor data from Gazebo
**Solution**:
- Check that sensor plugins are properly configured in URDF/SDF
- Verify Gazebo ROS bridge is running
- Check topic names match between simulation and code

### Issue: Robot control not working
**Solution**:
- Verify topic names match between controller and Gazebo
- Check that the robot has the correct joint names
- Ensure the control plugin is properly configured

## Key Takeaways

- Gazebo provides a realistic 3D simulation environment for robotics
- Integration with ROS 2 allows seamless communication between simulation and code
- Creating custom worlds and robots enhances testing capabilities
- Sensor simulation allows for realistic testing of perception algorithms
- Simulation is crucial for safe and cost-effective robotics development
- Proper URDF/SDF models are essential for accurate simulation

## Next Steps

In the next chapter, you'll learn about Unity integration for robotics simulation, which provides an alternative simulation environment with advanced graphics capabilities.