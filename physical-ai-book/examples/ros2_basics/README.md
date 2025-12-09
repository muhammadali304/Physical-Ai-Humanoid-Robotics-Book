# ROS 2 Basic Examples

This directory contains basic ROS 2 publisher/subscriber examples for the Physical AI & Humanoid Robotics educational book.

## Overview

These examples demonstrate fundamental ROS 2 concepts including:
- Node creation and management
- Publisher/subscriber communication pattern
- Message passing between nodes
- Basic ROS 2 API usage

## Examples

### 1. talker_node.py
A simple publisher node that sends "Hello World" messages to the `/chatter` topic.

### 2. listener_node.py
A simple subscriber node that receives messages from the `/chatter` topic and logs them.

### 3. talker_listener.py
A combined node that both publishes and subscribes to topics, demonstrating bidirectional communication.

## Prerequisites

- ROS 2 Humble Hawksbill installed
- Python 3.8+
- Basic understanding of ROS 2 concepts

## Usage

### Running the Publisher and Subscriber Separately

1. **Terminal 1** - Run the publisher:
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
python3 examples/ros2_basics/talker_node.py
```

2. **Terminal 2** - Run the subscriber:
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
python3 examples/ros2_basics/listener_node.py
```

### Running the Combined Example

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
python3 examples/ros2_basics/talker_listener.py
```

## Expected Output

### Publisher Output
```
[INFO] [1612345678.123456789] [talker_node]: Publishing: "Hello World: 0"
[INFO] [1612345679.123456789] [talker_node]: Publishing: "Hello World: 1"
[INFO] [1612345680.123456789] [talker_node]: Publishing: "Hello World: 2"
```

### Subscriber Output
```
[INFO] [1612345678.123456789] [listener_node]: I heard: "Hello World: 0"
[INFO] [1612345679.123456789] [listener_node]: I heard: "Hello World: 1"
[INFO] [1612345680.123456789] [listener_node]: I heard: "Hello World: 2"
```

## Key Concepts Demonstrated

1. **Node Creation**: Using `rclpy.node.Node` to create ROS 2 nodes
2. **Publishers**: Creating publishers with `create_publisher()`
3. **Subscribers**: Creating subscribers with `create_subscription()`
4. **Timers**: Using timers for periodic message publishing
5. **Message Types**: Using standard message types like `std_msgs.String`
6. **Logging**: Using node's logger for output
7. **ROS Client Library**: Initializing and shutting down `rclpy`

## Learning Outcomes

After running these examples, you should understand:
- How to create ROS 2 nodes in Python
- How to publish messages to topics
- How to subscribe to topics and process messages
- How to use timers for periodic publishing
- Basic ROS 2 node lifecycle management

## Troubleshooting

### Common Issues

1. **"Command 'python3' not found"**: Ensure Python 3 is installed
2. **"ModuleNotFoundError: No module named 'rclpy'"**: Ensure ROS 2 environment is sourced
3. **"Topic not found"**: Ensure publisher and subscriber nodes are running simultaneously

### Verification Commands

```bash
# Check if ROS 2 is properly sourced
ros2 --version

# List available topics
ros2 topic list

# Check topic type
ros2 topic type /chatter

# Echo messages on the chatter topic
ros2 topic echo /chatter std_msgs/msg/String
```

## Next Steps

- Explore more complex message types
- Learn about services and actions
- Understand launch files for managing multiple nodes
- Investigate ROS 2 parameters and lifecycle nodes

## References

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Python Client Library (rclpy)](https://docs.ros.org/en/humble/p/rclpy/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)