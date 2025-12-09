---
sidebar_position: 2
---

# Isaac ROS Perception - Computer Vision and Sensor Processing

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the Isaac ROS perception ecosystem and its components
- Implement computer vision pipelines using Isaac ROS packages
- Process sensor data from cameras, LIDAR, and other sensors
- Create perception pipelines for object detection and tracking
- Integrate perception results with navigation and planning systems
- Optimize perception pipelines for real-time performance

## Prerequisites

Before starting this chapter, you should:
- Have ROS 2 Humble Hawksbill installed and configured
- Understand ROS 2 nodes, topics, and message types
- Completed the Isaac Sim introduction chapter
- Basic knowledge of computer vision concepts
- Familiarity with deep learning frameworks (optional but helpful)

## Conceptual Overview

**Isaac ROS Perception** is a collection of optimized packages that provide advanced computer vision and sensor processing capabilities for robotics applications. These packages are specifically designed to leverage NVIDIA's GPU acceleration and deep learning frameworks.

### Key Components of Isaac ROS Perception

1. **Image Pipeline**: Handles image acquisition, rectification, and preprocessing
2. **Detection Pipeline**: Performs object detection, classification, and segmentation
3. **Tracking Pipeline**: Tracks objects across frames and provides temporal consistency
4. **Sensor Processing**: Handles various sensor types (LIDAR, IMU, etc.)
5. **Deep Learning Integration**: Optimized for TensorRT and other NVIDIA AI frameworks

### Advantages of Isaac ROS Perception

- **Hardware Acceleration**: Optimized for NVIDIA GPUs and TensorRT
- **Real-time Performance**: Designed for real-time robotics applications
- **ROS 2 Native**: Seamless integration with ROS 2 ecosystem
- **Modular Design**: Flexible components that can be combined as needed
- **Industrial Grade**: Built for production robotics applications

### Perception Pipeline Architecture

A typical Isaac ROS perception pipeline includes:

```
Sensors → Preprocessing → Detection → Tracking → Post-processing → ROS 2 Topics
```

Each stage can be customized based on the specific application requirements.

## Hands-On Implementation

### Installing Isaac ROS Perception Packages

Isaac ROS perception packages are available as part of the Isaac ROS repository:

```bash
# Create a workspace for Isaac ROS packages
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src

# Clone Isaac ROS perception packages
git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git
git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_detectnet.git
git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git
git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_point_cloud_processor.git

# Install dependencies
cd ~/isaac_ros_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --packages-select \
  isaac_ros_common \
  isaac_ros_image_pipeline \
  isaac_ros_detectnet \
  isaac_ros_apriltag \
  isaac_ros_visual_slam \
  isaac_ros_point_cloud_processor
```

### Image Pipeline Components

The Isaac ROS image pipeline provides optimized image processing components:

#### Image Rectification

```python
#!/usr/bin/env python3

"""
Example of image rectification using Isaac ROS image pipeline.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageRectificationNode(Node):
    """
    Node to demonstrate image rectification using Isaac ROS concepts.
    """

    def __init__(self):
        super().__init__('image_rectification_node')

        # Create subscribers for raw image and camera info
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10)

        # Create publisher for rectified image
        self.rectified_pub = self.create_publisher(
            Image,
            '/camera/image_rect',
            10)

        # Initialize variables
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rectification_initialized = False

        self.get_logger().info('Image rectification node initialized')

    def camera_info_callback(self, msg):
        """Process camera info to extract calibration parameters."""
        if not self.rectification_initialized:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.rectification_initialized = True
            self.get_logger().info('Camera calibration parameters loaded')

    def image_callback(self, msg):
        """Process incoming image and publish rectified version."""
        if not self.rectification_initialized:
            return

        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Apply undistortion
            h, w = cv_image.shape[:2]
            new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
                self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h))

            rectified_image = cv2.undistort(
                cv_image, self.camera_matrix, self.dist_coeffs, None, new_camera_matrix)

            # Crop the image based on ROI
            x, y, w, h = roi
            rectified_image = rectified_image[y:y+h, x:x+w]

            # Convert back to ROS image
            rectified_msg = self.bridge.cv2_to_imgmsg(rectified_image, "bgr8")
            rectified_msg.header = msg.header  # Preserve timestamp and frame ID

            # Publish rectified image
            self.rectified_pub.publish(rectified_msg)

            self.get_logger().info('Published rectified image')

        except Exception as e:
            self.get_logger().error(f'Error in image processing: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageRectificationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Object Detection with DetectNet

Isaac ROS includes optimized object detection packages:

#### Creating a Detection Node

```python
#!/usr/bin/env python3

"""
Object detection node using Isaac ROS DetectNet concepts.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np


class ObjectDetectionNode(Node):
    """
    Node to perform object detection using Isaac ROS concepts.
    """

    def __init__(self):
        super().__init__('object_detection_node')

        # Create subscriber for camera images
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # Create publisher for detections
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10)

        # Initialize variables
        self.bridge = CvBridge()

        # For demonstration, we'll use a simple Haar cascade
        # In practice, you'd use a TensorRT-optimized model
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        self.get_logger().info('Object detection node initialized')

    def image_callback(self, msg):
        """Process image and detect objects."""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Convert to grayscale for detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Perform face detection (for demonstration)
            faces = self.face_cascade.detectMultiScale(
                gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

            # Create detections message
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header

            # Process each detection
            for (x, y, w, h) in faces:
                detection = Detection2D()

                # Set bounding box
                detection.bbox.center.x = x + w / 2
                detection.bbox.center.y = y + h / 2
                detection.bbox.size_x = w
                detection.bbox.size_y = h

                # Set confidence and class
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = "face"
                hypothesis.hypothesis.score = 0.9  # Example confidence

                detection.results.append(hypothesis)

                # Add to detections array
                detections_msg.detections.append(detection)

                # Draw bounding box on image for visualization
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 0, 0), 2)

            # Publish detections
            self.detections_pub.publish(detections_msg)

            self.get_logger().info(f'Published {len(faces)} detections')

        except Exception as e:
            self.get_logger().error(f'Error in detection: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### AprilTag Detection

AprilTag detection is commonly used for precise pose estimation:

#### AprilTag Detection Node

```python
#!/usr/bin/env python3

"""
AprilTag detection node using Isaac ROS concepts.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2
import numpy as np

# Note: In a real Isaac ROS setup, you'd use the optimized AprilTag package
# For this example, we'll simulate the functionality
try:
    import pupil_apriltags as apriltag
except ImportError:
    print("AprilTag library not found. Install with: pip install pupil-apriltags")


class AprilTagDetectionNode(Node):
    """
    Node to detect AprilTags and estimate poses.
    """

    def __init__(self):
        super().__init__('apriltag_detection_node')

        # Create subscriber for camera images
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # Create publisher for tag poses
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/apriltag_pose',
            10)

        # Create publisher for detections
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            '/apriltag_detections',
            10)

        # Initialize variables
        self.bridge = CvBridge()

        # Camera intrinsic parameters (these should come from camera_info topic in real usage)
        self.camera_matrix = np.array([
            [615.0, 0.0, 320.0],
            [0.0, 615.0, 240.0],
            [0.0, 0.0, 1.0]
        ])

        # Distortion coefficients
        self.dist_coeffs = np.zeros((5, 1))

        # AprilTag detector
        try:
            self.detector = apriltag.Detector(families='tag36h11')
        except:
            self.detector = None
            self.get_logger().warn('AprilTag detector not available')

        self.get_logger().info('AprilTag detection node initialized')

    def image_callback(self, msg):
        """Process image and detect AprilTags."""
        if self.detector is None:
            return

        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Detect AprilTags
            tags = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=[615.0, 615.0, 320.0, 240.0],  # fx, fy, cx, cy
                tag_size=0.16  # Size of tag in meters
            )

            # Create detections message
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header

            for tag in tags:
                # Create pose message
                pose_msg = PoseStamped()
                pose_msg.header = msg.header
                pose_msg.pose.position.x = float(tag.pose_t[0])
                pose_msg.pose.position.y = float(tag.pose_t[1])
                pose_msg.pose.position.z = float(tag.pose_t[2])

                # Set orientation from rotation matrix
                R = tag.pose_R
                # Convert rotation matrix to quaternion (simplified)
                # In practice, you'd use proper conversion
                pose_msg.pose.orientation.w = 1.0  # Placeholder

                # Publish pose
                self.pose_pub.publish(pose_msg)

                self.get_logger().info(f'Detected tag {tag.tag_id} at position: {pose_msg.pose.position}')

                # Draw tag on image for visualization
                for idx in range(len(tag.corners)):
                    pt1 = tuple(tag.corners[idx][0].astype(int))
                    pt2 = tuple(tag.corners[(idx + 1) % len(tag.corners)][0].astype(int))
                    cv2.line(cv_image, pt1, pt2, (0, 255, 0), 2)

            # Publish detections message
            self.detections_pub.publish(detections_msg)

        except Exception as e:
            self.get_logger().error(f'Error in AprilTag detection: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Point Cloud Processing

Isaac ROS provides tools for processing 3D point cloud data:

#### Point Cloud Processing Node

```python
#!/usr/bin/env python3

"""
Point cloud processing node using Isaac ROS concepts.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct
from sensor_msgs_py import point_cloud2

class PointCloudProcessingNode(Node):
    """
    Node to process point cloud data using Isaac ROS concepts.
    """

    def __init__(self):
        super().__init__('pointcloud_processing_node')

        # Create subscriber for point cloud
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/points',
            self.pointcloud_callback,
            10)

        # Create publisher for processed point cloud
        self.processed_pc_pub = self.create_publisher(
            PointCloud2,
            '/points_processed',
            10)

        self.get_logger().info('Point cloud processing node initialized')

    def pointcloud_callback(self, msg):
        """Process incoming point cloud."""
        try:
            # Convert PointCloud2 to list of points
            points = []
            for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append([point[0], point[1], point[2]])

            if not points:
                return

            points = np.array(points)

            # Example processing: remove ground plane using RANSAC
            processed_points = self.remove_ground_plane(points)

            # Create new PointCloud2 message
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = msg.header.frame_id

            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]

            # Convert processed points back to PointCloud2 format
            processed_msg = point_cloud2.create_cloud(header, fields, processed_points)

            # Publish processed point cloud
            self.processed_pc_pub.publish(processed_msg)

            self.get_logger().info(f'Processed point cloud: {len(points)} -> {len(processed_points)} points')

        except Exception as e:
            self.get_logger().error(f'Error in point cloud processing: {e}')

    def remove_ground_plane(self, points, distance_threshold=0.1):
        """
        Simple ground plane removal using height thresholding.
        In practice, you'd use RANSAC or other more sophisticated methods.
        """
        # For this example, remove points with z < 0.1 (assuming ground is at z=0)
        return points[points[:, 2] > distance_threshold]


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProcessingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Creating a Perception Pipeline Launch File

**Create a launch file for the perception pipeline:**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    camera_namespace = LaunchConfiguration('camera_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Image rectification node
    image_rectification_node = Node(
        package='isaac_ros_image_pipeline',
        executable='isaac_ros_image_rectification',
        name='image_rectification',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'camera_namespace': camera_namespace}
        ],
        remappings=[
            ('image_raw', 'image_raw'),
            ('image_rect', 'image_rect'),
            ('camera_info', 'camera_info')
        ]
    )

    # Object detection node
    detection_node = Node(
        package='isaac_ros_detectnet',
        executable='isaac_ros_detectnet',
        name='object_detection',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'camera_namespace': camera_namespace},
            {'model_name': 'detectnet'},
            {'input_topic': 'image_rect'},
            {'output_topic': 'detections'}
        ]
    )

    # AprilTag detection node
    apriltag_node = Node(
        package='isaac_ros_apriltag',
        executable='isaac_ros_apriltag',
        name='apriltag_detection',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'camera_namespace': camera_namespace},
            {'input_topic': 'image_rect'}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_namespace',
            default_value='camera',
            description='Namespace for camera topics'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        image_rectification_node,
        detection_node,
        apriltag_node
    ])
```

### Isaac ROS Perception Best Practices

#### Optimizing for Performance

1. **Use TensorRT**: Convert models to TensorRT format for GPU acceleration
2. **Batch Processing**: Process multiple frames together when possible
3. **Memory Management**: Use CUDA unified memory for efficient transfers
4. **Threading**: Use appropriate threading models for your pipeline

#### Example: TensorRT Optimization

```python
# This is a conceptual example - actual implementation would use Isaac ROS tools
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

def optimize_model_for_tensorrt(model_path):
    """
    Conceptual function to optimize a model for TensorRT.
    In practice, Isaac ROS provides tools for this.
    """
    # Create TensorRT builder
    builder = trt.Builder(trt.Logger(trt.Logger.WARNING))
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))

    # Parse ONNX model
    parser = trt.OnnxParser(network, trt.Logger())

    # Configure optimization settings
    config = builder.create_builder_config()
    config.max_workspace_size = 1 << 30  # 1GB

    # Build engine
    serialized_engine = builder.build_serialized_network(network, config)

    return serialized_engine
```

## Testing & Verification

### Running the Perception Pipeline

1. **Build Isaac ROS packages:**
```bash
cd ~/isaac_ros_ws
source install/setup.bash
colcon build
```

2. **Launch the perception pipeline:**
```bash
# Source both workspaces
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash

# Launch the pipeline
ros2 launch perception_pipeline perception_launch.py
```

3. **Provide test data:**
```bash
# Play a bag file with camera data
ros2 bag play --clock /path/to/camera_data.bag

# Or use a simulated camera from Isaac Sim
```

4. **Monitor outputs:**
```bash
# Check detection results
ros2 topic echo /detections

# Check processed images
ros2 topic echo /camera/image_rect

# Check AprilTag poses
ros2 topic echo /apriltag_pose
```

### Useful Perception Commands

- **Check available Isaac ROS packages:**
```bash
ros2 pkg list | grep isaac
```

- **View image topics:**
```bash
# Use rqt_image_view to visualize images
rqt_image_view
```

- **Monitor point clouds:**
```bash
# Use RViz2 to visualize point clouds
rviz2
```

- **Performance monitoring:**
```bash
# Monitor node performance
ros2 run top top
```

### Benchmarking Perception Performance

```bash
# Use ROS 2 tools to measure pipeline performance
ros2 run topic_tools relay /camera/image_raw /benchmark/image_raw

# Or use specialized tools like isaac_ros_benchmark
```

## Common Issues

### Issue: Perception nodes not running or crashing
**Solution**:
- Verify Isaac ROS packages are properly built
- Check GPU compatibility and drivers
- Ensure TensorRT is properly installed
- Verify CUDA compatibility

### Issue: Poor detection performance
**Solution**:
- Check camera calibration parameters
- Verify lighting conditions
- Ensure appropriate model is used for the task
- Check that input resolution matches model expectations

### Issue: High latency in perception pipeline
**Solution**:
- Use appropriate image resolution for your application
- Consider processing every Nth frame if real-time performance is critical
- Optimize model size for your hardware
- Use appropriate threading models

### Issue: Memory allocation errors
**Solution**:
- Reduce batch size or input resolution
- Use appropriate GPU memory settings
- Monitor memory usage during operation
- Consider using memory-efficient models

## Key Takeaways

- Isaac ROS perception packages provide optimized computer vision capabilities
- Hardware acceleration is key to achieving real-time performance
- Modular design allows for custom pipeline configurations
- Proper camera calibration is essential for accurate results
- Performance optimization requires careful consideration of model and hardware
- Integration with navigation and planning systems enables autonomous behavior

## Next Steps

In the next chapter, you'll learn about navigation systems in the Isaac ecosystem, which will allow you to use the perception results for autonomous navigation and path planning.