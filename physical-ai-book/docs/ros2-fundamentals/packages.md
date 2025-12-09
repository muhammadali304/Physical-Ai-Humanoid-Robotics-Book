---
sidebar_position: 3
---

# ROS 2 Packages - Organization and Management

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the structure and purpose of ROS 2 packages
- Create, build, and manage ROS 2 packages
- Organize code, dependencies, and resources within packages
- Use package management tools effectively
- Follow best practices for package organization

## Prerequisites

Before starting this chapter, you should:
- Understand ROS 2 nodes, topics, services, and actions
- Have ROS 2 Humble Hawksbill installed and configured
- Be familiar with basic Linux command line operations
- Completed the ROS 2 fundamentals chapters

## Conceptual Overview

A **ROS 2 package** is the fundamental unit for organizing and distributing ROS 2 software. It contains:
- Source code (nodes, libraries, etc.)
- Configuration files
- Launch files
- Documentation
- Dependencies
- Build instructions

Packages enable:
- Code reusability and modularity
- Dependency management
- Distribution and sharing
- Proper organization of robotics software

### Package Types

ROS 2 packages can be of different types based on their build system:
- **ament_python**: For Python-based packages
- **ament_cmake**: For CMake-based packages (typically C++)
- **ament_cargo**: For Rust-based packages

## Hands-On Implementation

### Creating a Package

#### Using ros2 pkg create

The easiest way to create a new package is using the `ros2 pkg create` command:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_package
```

This creates a basic Python package structure with:
- `package.xml`: Package metadata and dependencies
- `setup.py`: Python setup configuration
- `setup.cfg`: Additional Python configuration
- `my_robot_package/__init__.py`: Python package initialization
- `my_robot_package/my_robot_package/`: Main package directory

#### Package.xml Structure

The `package.xml` file contains important metadata about your package:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO: License declaration</license>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

#### Setup.py Configuration

The `setup.py` file configures how your package is built and installed:

```python
from setuptools import find_packages, setup

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_robot_package.my_node:main',
        ],
    },
)
```

### Package Structure

A well-organized ROS 2 package typically follows this structure:

```
my_robot_package/
├── CMakeLists.txt          # Build instructions for CMake packages
├── package.xml             # Package metadata
├── setup.py                # Python package configuration
├── setup.cfg               # Python configuration
├── resource/               # Resource files
│   └── my_robot_package    # Resource index
├── my_robot_package/       # Main Python package directory
│   ├── __init__.py         # Python package initialization
│   ├── my_node.py          # Python nodes
│   ├── my_library.py       # Python libraries
│   └── msg/                # Custom message definitions
│       ├── MyMessage.msg
│       └── AnotherMessage.msg
├── launch/                 # Launch files
│   ├── my_launch_file.launch.py
│   └── another_launch_file.launch.py
├── config/                 # Configuration files
│   ├── params.yaml
│   └── controllers.yaml
├── test/                   # Test files
│   ├── test_my_node.py
│   └── test_my_library.py
├── scripts/                # Standalone scripts (if not nodes)
└── include/                # Header files (for C++ packages)
    └── my_robot_package/
        └── my_header.hpp
```

### Creating a More Complex Package Example

Let's create a package that includes multiple components:

1. **Create the package:**
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python robot_controller
```

2. **Update package.xml with dependencies:**
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_controller</name>
  <version>0.1.0</version>
  <description>Robot controller package with various components</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>

  <exec_depend>ros2launch</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

3. **Create a library module (robot_controller/robot_controller/motor_control.py):**
```python
"""
Motor control utilities for the robot controller package.
"""

import math
from typing import List


class MotorController:
    """
    A class to handle basic motor control operations.
    """

    def __init__(self, motor_id: int, max_speed: float = 1.0):
        self.motor_id = motor_id
        self.max_speed = max_speed
        self.current_speed = 0.0
        self.is_active = False

    def set_speed(self, speed: float) -> bool:
        """
        Set the motor speed with safety checks.

        Args:
            speed: Desired speed (-max_speed to +max_speed)

        Returns:
            True if speed was set successfully, False otherwise
        """
        if abs(speed) > self.max_speed:
            print(f"Warning: Speed {speed} exceeds maximum {self.max_speed}")
            speed = max(min(speed, self.max_speed), -self.max_speed)

        self.current_speed = speed
        if not self.is_active:
            self.is_active = True

        return True

    def stop(self) -> None:
        """Stop the motor."""
        self.current_speed = 0.0
        self.is_active = False

    def get_status(self) -> dict:
        """Get the current motor status."""
        return {
            'motor_id': self.motor_id,
            'current_speed': self.current_speed,
            'is_active': self.is_active,
            'max_speed': self.max_speed
        }


def calculate_wheel_speeds(velocity_x: float, velocity_y: float,
                          angular_z: float, wheel_positions: List[float]) -> List[float]:
    """
    Calculate individual wheel speeds for differential drive.

    Args:
        velocity_x: Linear velocity in x direction
        velocity_y: Linear velocity in y direction
        angular_z: Angular velocity around z axis
        wheel_positions: List of wheel positions [x, y] relative to robot center

    Returns:
        List of wheel speeds
    """
    wheel_speeds = []
    for pos_x, pos_y in wheel_positions:
        # Calculate the required wheel speed based on desired motion
        linear_contribution = velocity_x  # Simplified for example
        angular_contribution = angular_z * math.sqrt(pos_x**2 + pos_y**2)
        wheel_speed = linear_contribution + angular_contribution
        wheel_speeds.append(wheel_speed)

    return wheel_speeds
```

4. **Create a main node (robot_controller/robot_controller/controller_node.py):**
```python
#!/usr/bin/env python3

"""
Robot controller node that subscribes to velocity commands and controls motors.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from .motor_control import MotorController, calculate_wheel_speeds


class RobotControllerNode(Node):
    """
    A ROS 2 node that controls robot motors based on velocity commands.
    """

    def __init__(self):
        super().__init__('robot_controller')

        # Initialize motor controllers
        self.left_motor = MotorController(0, max_speed=2.0)
        self.right_motor = MotorController(1, max_speed=2.0)

        # Create subscriber for velocity commands
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Create publisher for status updates
        self.status_publisher = self.create_publisher(
            String,
            'motor_status',
            10)

        # Log initialization
        self.get_logger().info('Robot controller node initialized')

    def velocity_callback(self, msg: Twist) -> None:
        """
        Callback function for velocity commands.

        Args:
            msg: Twist message containing linear and angular velocities
        """
        self.get_logger().info(f'Received velocity: linear=({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), '
                              f'angular=({msg.angular.x}, {msg.angular.y}, {msg.angular.z})')

        # Calculate wheel speeds (simplified for this example)
        # In a real robot, you would use kinematic equations
        left_speed = msg.linear.x - msg.angular.z * 0.5  # 0.5 is wheelbase/2 approximation
        right_speed = msg.linear.x + msg.angular.z * 0.5

        # Apply speeds to motors
        self.left_motor.set_speed(left_speed)
        self.right_motor.set_speed(right_speed)

        # Publish status
        status_msg = String()
        status_msg.data = f'Left: {self.left_motor.current_speed}, Right: {self.right_motor.current_speed}'
        self.status_publisher.publish(status_msg)

        self.get_logger().info(f'Set motor speeds - Left: {left_speed}, Right: {right_speed}')


def main(args=None):
    rclpy.init(args=args)

    controller_node = RobotControllerNode()

    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        controller_node.get_logger().info('Node interrupted by user')
    finally:
        # Clean up when shutting down
        controller_node.left_motor.stop()
        controller_node.right_motor.stop()
        controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

5. **Update setup.py to include the new node:**
```python
from setuptools import find_packages, setup

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Robot controller package with various components',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = robot_controller.controller_node:main',
        ],
    },
)
```

6. **Build the package:**
```bash
cd ~/ros2_ws
colcon build --packages-select robot_controller
source install/setup.bash
```

### Launch Files

Create a launch file to start your nodes easily:

**Create launch directory:**
```bash
mkdir -p ~/ros2_ws/src/robot_controller/launch
```

**Create a launch file (robot_controller/launch/controller_launch.py):**
```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_controller',
            executable='controller_node',
            name='robot_controller',
            output='screen',
            parameters=[
                # Add parameters here if needed
            ]
        )
    ])
```

### Configuration Files

Create a configuration directory and YAML file:

**Create config directory:**
```bash
mkdir -p ~/ros2_ws/src/robot_controller/config
```

**Create a config file (robot_controller/config/robot_params.yaml):**
```yaml
robot_controller:
  ros__parameters:
    max_linear_speed: 1.0
    max_angular_speed: 1.5
    wheel_separation: 0.5
    wheel_radius: 0.1
    update_rate: 50.0
```

## Testing & Verification

### Building Packages

1. **Build a specific package:**
```bash
cd ~/ros2_ws
colcon build --packages-select robot_controller
```

2. **Build all packages:**
```bash
cd ~/ros2_ws
colcon build
```

3. **Build with specific options:**
```bash
# Build with symlinks (faster rebuilds)
colcon build --symlink-install

# Build only Python packages
colcon build --packages-select robot_controller --cmake-clean-cache
```

### Running the Package

1. **Source the workspace:**
```bash
cd ~/ros2_ws
source install/setup.bash
```

2. **Run the node directly:**
```bash
ros2 run robot_controller controller_node
```

3. **Run using launch file:**
```bash
ros2 launch robot_controller controller_launch.py
```

4. **Test with velocity commands:**
```bash
# In another terminal
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
```

### Package Management Commands

- **List all packages:**
```bash
ros2 pkg list
```

- **Get information about a package:**
```bash
ros2 pkg info robot_controller
```

- **Find a package:**
```bash
ros2 pkg prefix robot_controller
```

- **Show package dependencies:**
```bash
ros2 pkg deps robot_controller
```

- **Find executables in a package:**
```bash
ros2 pkg executables robot_controller
```

## Common Issues

### Issue: Package not found after building
**Solution**:
- Make sure to source the workspace: `source install/setup.bash`
- Check that the package was built successfully
- Verify the package name is correct

### Issue: Import errors in Python packages
**Solution**:
- Ensure the package was built with `colcon build`
- Check that the `__init__.py` files are present
- Verify that the package name in setup.py matches the directory name

### Issue: Dependencies not found
**Solution**:
- Add dependencies to `package.xml`
- Run `rosdep install --from-paths src --ignore-src -r -y` to install dependencies
- Rebuild the package after adding dependencies

### Issue: Console scripts not available
**Solution**:
- Make sure entry points are properly defined in `setup.py`
- Verify the function exists and has the correct signature
- Rebuild the package after updating setup.py

## Key Takeaways

- Packages are the fundamental unit for organizing ROS 2 software
- Proper package structure improves maintainability and reusability
- Dependencies must be declared in `package.xml` and `setup.py`
- Launch files simplify running multiple nodes together
- Configuration files separate parameters from code
- Use `colcon build` to compile packages
- Always source the workspace after building

## Next Steps

In the next chapter, you'll learn about simulation with Gazebo, which will allow you to test your ROS 2 packages in a virtual environment before deploying to real hardware.