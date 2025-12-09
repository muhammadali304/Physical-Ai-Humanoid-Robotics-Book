---
sidebar_position: 2
---

# Unity Integration for Robotics Simulation

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand Unity's role in robotics simulation and its advantages
- Set up Unity with robotics simulation packages
- Create basic robotic environments in Unity
- Integrate Unity with ROS 2 for bidirectional communication
- Implement sensor simulation in Unity
- Deploy Unity-based simulations for robotics development

## Prerequisites

Before starting this chapter, you should:
- Have ROS 2 Humble Hawksbill installed and configured
- Understand ROS 2 nodes, topics, and packages
- Have basic knowledge of Linux command line operations
- Completed the Gazebo simulation chapter
- Basic familiarity with Unity concepts (optional but helpful)

## Conceptual Overview

**Unity** is a powerful 3D development platform that has gained significant traction in robotics simulation due to its advanced graphics capabilities, physics engine, and flexible architecture. Unity provides:

- **High-quality rendering**: Photo-realistic environments and lighting
- **Advanced physics**: Realistic physics simulation with PhysX engine
- **Flexible development**: Extensive scripting and asset creation tools
- **Cross-platform deployment**: Deploy to various platforms and devices
- **Asset Store**: Access to pre-made models and environments

### Unity Robotics Simulation Pipeline

Unity's robotics integration primarily uses the **Unity Robotics Hub**, which includes:
- **Unity Robot Framework**: Tools for creating and controlling robots
- **Unity Perception Package**: Tools for generating synthetic training data
- **ROS# Communication**: Bridge between Unity and ROS 2
- **Unity ML-Agents**: Framework for training AI using reinforcement learning

### When to Use Unity vs Gazebo

**Choose Unity when:**
- High-fidelity graphics are required
- Creating photorealistic environments for perception tasks
- Developing AR/VR applications for robotics
- Need advanced rendering effects (shaders, lighting, etc.)
- Creating user interfaces for robot teleoperation

**Choose Gazebo when:**
- Physics accuracy is the primary concern
- Integration with existing ROS 2 packages is needed
- Performance in complex multi-robot scenarios is critical
- Working with standard robot models and sensors

## Hands-On Implementation

### Installing Unity

1. **Download Unity Hub** from https://unity.com/download
2. **Install Unity Hub** following the standard installation process
3. **Use Unity Hub to install Unity 2021.3 LTS** (recommended for robotics projects)

### Installing Unity Robotics Packages

Unity provides several packages for robotics integration:

1. **Unity Robotics Hub**: Main package for robotics simulation
2. **Unity Perception**: For generating synthetic training data
3. **ROS#**: For ROS 2 communication
4. **ML-Agents**: For reinforcement learning

#### Installing ROS# for Unity

1. **Open Unity Hub** and create a new 3D project
2. **In the Unity Editor, open the Package Manager** (Window > Package Manager)
3. **Add the ROS# package**:
   - Click the `+` button in the top-left corner
   - Select "Add package from git URL..."
   - Enter: `https://github.com/siemens/ros-sharp.git`
   - Click "Add"

#### Installing Unity Robotics Package

1. **In the Package Manager**:
   - Click the `+` button
   - Select "Add package from git URL..."
   - Enter: `com.unity.robotics.urdf-importer`
   - Click "Add"

### Setting up ROS 2 Bridge

#### Installing ROS 2 Unity Bridge

Unity provides the `ros_unity_bridge` package for communication between ROS 2 and Unity:

```bash
# Clone the ROS 2 Unity Bridge repository
cd ~/ros2_ws/src
git clone -b humble https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build
```

### Creating a Basic Unity Robot Scene

#### Setting up the Unity Scene

1. **Create a new 3D project** in Unity Hub
2. **Import the Unity Robotics Package**:
   - Go to Assets > Import Package > Custom Package
   - Import the URDF Importer package

3. **Create a basic robot model**:
   - Create an empty GameObject (GameObject > Create Empty)
   - Name it "Robot"
   - Add basic primitive shapes (cubes, cylinders) for the robot body

4. **Set up physics**:
   - Add Rigidbody components to movable parts
   - Add Collider components for collision detection
   - Configure physics properties appropriately

#### Unity Script for Robot Control

**Create a C# script (RobotController.cs) in Unity:**

```csharp
using UnityEngine;
using System.Collections;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RobotController : MonoBehaviour
{
    [SerializeField] private float linearSpeed = 1.0f;
    [SerializeField] private float angularSpeed = 1.0f;

    private ROSConnection ros;
    private string cmdVelTopic = "cmd_vel";

    private float linearVelocity = 0.0f;
    private float angularVelocity = 0.0f;

    void Start()
    {
        // Get the ROS connection static instance
        ros = ROSConnection.instance;

        // Subscribe to the cmd_vel topic
        ros.Subscribe<TwistMsg>(cmdVelTopic, CmdVelCallback);
    }

    void CmdVelCallback(TwistMsg twist)
    {
        // Extract linear and angular velocities from ROS message
        linearVelocity = (float)twist.linear.x;
        angularVelocity = (float)twist.angular.z;
    }

    void Update()
    {
        // Apply movement based on received velocities
        transform.Translate(Vector3.forward * linearVelocity * Time.deltaTime * linearSpeed);
        transform.Rotate(Vector3.up, angularVelocity * Time.deltaTime * angularSpeed);
    }

    void OnDestroy()
    {
        // Unsubscribe when the object is destroyed
        if (ros != null)
        {
            ros.Unsubscribe(cmdVelTopic);
        }
    }
}
```

### Setting up Sensor Simulation in Unity

#### Creating a Simple Camera Sensor

**Create a C# script for camera sensor (CameraSensor.cs):**

```csharp
using UnityEngine;
using System.Collections;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine.Rendering;

public class CameraSensor : MonoBehaviour
{
    [SerializeField] private Camera camera;
    [SerializeField] private string imageTopic = "/camera/image_raw";
    [SerializeField] private int publishRate = 30; // Hz

    private ROSConnection ros;
    private RenderTexture renderTexture;
    private Texture2D texture2D;
    private float publishInterval;
    private float lastPublishTime;

    void Start()
    {
        ros = ROSConnection.instance;
        publishInterval = 1.0f / publishRate;
        lastPublishTime = 0;

        // Set up the camera if not assigned
        if (camera == null)
        {
            camera = GetComponent<Camera>();
        }

        // Create render texture for camera capture
        int width = camera.pixelWidth;
        int height = camera.pixelHeight;
        renderTexture = new RenderTexture(width, height, 24);
        camera.targetTexture = renderTexture;

        texture2D = new Texture2D(width, height, TextureFormat.RGB24, false);
    }

    void Update()
    {
        float currentTime = Time.time;
        if (currentTime - lastPublishTime >= publishInterval)
        {
            PublishImage();
            lastPublishTime = currentTime;
        }
    }

    void PublishImage()
    {
        // Capture the camera image
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, texture2D.width, texture2D.height), 0, 0);
        texture2D.Apply();

        // Convert to byte array
        byte[] imageData = texture2D.EncodeToPNG();

        // Create ROS Image message
        ImageMsg imageMsg = new ImageMsg
        {
            header = new std_msgs.HeaderMsg
            {
                stamp = new builtin_interfaces.TimeMsg
                {
                    sec = (int)System.DateTime.UtcNow.Subtract(
                        new System.DateTime(1970, 1, 1)).TotalSeconds,
                    nanosec = (uint)(System.DateTime.UtcNow.Millisecond * 1000000)
                },
                frame_id = "camera_frame"
            },
            height = (uint)texture2D.height,
            width = (uint)texture2D.width,
            encoding = "rgb8",
            is_bigendian = 0,
            step = (uint)(texture2D.width * 3), // 3 bytes per pixel for RGB
            data = imageData
        };

        // Publish the image
        ros.Publish(imageTopic, imageMsg);
    }

    void OnDestroy()
    {
        if (renderTexture != null)
        {
            renderTexture.Release();
        }
    }
}
```

### Creating a Unity Package for ROS Integration

#### Unity Scene Setup

1. **Create a new scene** called "RobotSimulation"
2. **Add a ground plane**:
   - Create a cube and scale it to represent the ground
   - Add a Collider component
   - Apply appropriate materials

3. **Add lighting**:
   - Add a Directional Light to simulate sun
   - Adjust lighting settings for realistic appearance

4. **Create the robot**:
   - Use GameObjects to create a simple robot body
   - Add the RobotController script to the main robot object
   - Add the CameraSensor script to a camera component

### Setting up ROS 2 Unity Bridge Communication

#### Starting the ROS 2 Bridge

1. **Source your ROS 2 workspace:**
```bash
cd ~/ros2_ws
source install/setup.bash
```

2. **Start the TCP endpoint:**
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000
```

3. **In Unity**, configure the ROS connection:
   - Go to GameObject > Unity Robotics > ROS Settings
   - Set the ROS IP to `127.0.0.1`
   - Set the port to `10000`

#### Testing Unity-ROS Communication

**Create a simple ROS 2 node to test communication (unity_tester.py):**

```python
#!/usr/bin/env python3

"""
Simple ROS 2 node to test Unity communication.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class UnityTester(Node):
    def __init__(self):
        super().__init__('unity_tester')

        # Publisher for robot commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10)

        # Subscriber for camera images
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # Timer for sending commands
        self.timer = self.create_timer(1.0, self.send_command)

        self.bridge = CvBridge()
        self.command_index = 0

        self.get_logger().info('Unity tester node initialized')

    def send_command(self):
        """Send a command to the Unity robot."""
        msg = Twist()

        # Alternate between different commands
        if self.command_index % 4 == 0:
            msg.linear.x = 0.5
            msg.angular.z = 0.0
        elif self.command_index % 4 == 1:
            msg.linear.x = 0.0
            msg.angular.z = 0.5
        elif self.command_index % 4 == 2:
            msg.linear.x = -0.5
            msg.angular.z = 0.0
        else:
            msg.linear.x = 0.0
            msg.angular.z = -0.5

        self.cmd_vel_publisher.publish(msg)
        self.command_index += 1

        self.get_logger().info(f'Sent command: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

    def image_callback(self, msg):
        """Process images from Unity."""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")

            # Display the image (optional)
            cv2.imshow("Unity Camera", cv_image)
            cv2.waitKey(1)

            self.get_logger().info(f'Received image: {msg.width}x{msg.height}')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)

    tester = UnityTester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info('Tester stopped by user')
    finally:
        cv2.destroyAllWindows()
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Creating a Unity Launch System

#### Unity Simulation Launch Script

**Create a launch file (unity_simulation.launch.py):**

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    ros_ip_arg = DeclareLaunchArgument(
        'ros_ip',
        default_value='127.0.0.1',
        description='IP address of the ROS system'
    )

    tcp_port_arg = DeclareLaunchArgument(
        'tcp_port',
        default_value='10000',
        description='Port for ROS TCP communication'
    )

    # ROS TCP Endpoint
    ros_tcp_endpoint = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_tcp_endpoint', 'default_server_endpoint',
             '--ros-args',
             '-p', 'ROS_IP:=127.0.0.1',
             '-p', 'ROS_TCP_PORT:=10000'],
        output='screen'
    )

    # Unity tester node
    unity_tester = Node(
        package='robot_controller',
        executable='unity_tester',  # You'll need to add this to setup.py
        name='unity_tester',
        output='screen'
    )

    return LaunchDescription([
        ros_ip_arg,
        tcp_port_arg,
        ros_tcp_endpoint,
        unity_tester
    ])
```

## Testing & Verification

### Running Unity Simulation

1. **Start ROS 2 Bridge:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000
```

2. **In Unity Editor:**
   - Make sure ROS settings are configured correctly
   - Press Play to start the simulation

3. **In another terminal:**
```bash
# Run the tester node
ros2 run robot_controller unity_tester
```

4. **Monitor the communication:**
```bash
# Check topics
ros2 topic list | grep -E "(cmd_vel|camera)"

# Echo camera data
ros2 topic echo /camera/image_raw --field data --field width --field height
```

### Useful Unity-ROS Commands

- **Check connection:**
```bash
telnet 127.0.0.1 10000
```

- **Monitor Unity topics:**
```bash
ros2 topic list
ros2 topic info /cmd_vel
```

- **Send manual commands:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
```

## Common Issues

### Issue: Unity cannot connect to ROS 2
**Solution**:
- Verify ROS TCP endpoint is running
- Check IP and port configuration in Unity
- Ensure firewall is not blocking the connection
- Try using localhost (127.0.0.1) instead of actual IP

### Issue: Sensor data not publishing
**Solution**:
- Verify the sensor script is properly attached to the correct GameObject
- Check that the topic name matches between Unity and ROS 2
- Ensure the camera/render texture is properly configured

### Issue: Performance problems in Unity
**Solution**:
- Reduce rendering quality for simulation
- Use simpler physics settings
- Optimize the scene with fewer draw calls
- Consider using Unity's Profiler to identify bottlenecks

### Issue: URDF import problems
**Solution**:
- Ensure the URDF Importer package is installed
- Check that the URDF file is properly formatted
- Verify all referenced mesh files exist

## Key Takeaways

- Unity provides high-quality graphics and rendering for robotics simulation
- ROS# enables communication between Unity and ROS 2
- Unity is particularly useful for perception tasks requiring photorealistic rendering
- Unity's physics engine provides realistic simulation capabilities
- Unity allows for creation of complex, visually rich environments
- Integration requires careful setup of the communication bridge

## Next Steps

In the next chapter, you'll learn about URDF modeling, which is essential for creating accurate robot representations in both Gazebo and Unity simulation environments.