# Running and Testing ROS 2 Publisher/Subscriber Examples

## Overview

This guide provides detailed instructions for running and testing the ROS 2 publisher/subscriber examples created for the Physical AI & Humanoid Robotics educational book. The examples demonstrate fundamental ROS 2 communication patterns using both Python and C++.

## Prerequisites

Before running the examples, ensure you have:

- [X] ROS 2 Humble Hawksbill installed and configured
- [X] Python 3.8+ environment set up
- [X] C++ development tools installed
- [X] Basic ROS 2 workspace created (`~/ros2_ws`)
- [X] Environment properly sourced

## Directory Structure

Make sure your examples are in the correct location:

```
~/ros2_ws/src/
└── ros2_basics_examples/          # C++ examples package
    ├── CMakeLists.txt
    ├── package.xml
    ├── talker.cpp
    └── listener.cpp

~/ros2_ws/examples/ros2_basics/     # Python examples
    ├── talker_node.py
    ├── listener_node.py
    ├── talker_listener.py
    └── README.md
```

## Running Python Examples

### Method 1: Direct Python Execution

#### Running the Publisher (Terminal 1)

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Navigate to the examples directory
cd ~/ros2_ws

# Run the talker node
python3 examples/ros2_basics/talker_node.py
```

#### Running the Subscriber (Terminal 2)

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Navigate to the examples directory
cd ~/ros2_ws

# Run the listener node
python3 examples/ros2_basics/listener_node.py
```

### Method 2: Using ROS 2 Run Command

First, ensure your Python files are executable and in a proper ROS 2 package structure, then:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Run publisher
ros2 run your_package_name talker_node.py

# Run subscriber
ros2 run your_package_name listener_node.py
```

## Running C++ Examples

### Building the C++ Package

Before running C++ examples, you need to build them:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Navigate to your workspace
cd ~/ros2_ws

# Copy the C++ package to src directory if not already there
cp -r examples/ros2_basics src/ros2_basics_examples

# Build the package
colcon build --packages-select ros2_basics_examples

# Source the workspace
source install/setup.bash
```

### Running the Built Examples

#### Running the Publisher (Terminal 1)

```bash
# Source the workspace with built examples
source ~/ros2_ws/install/setup.bash

# Run the talker executable
ros2 run ros2_basics_examples talker
```

#### Running the Subscriber (Terminal 2)

```bash
# Source the workspace with built examples
source ~/ros2_ws/install/setup.bash

# Run the listener executable
ros2 run ros2_basics_examples listener
```

## Testing the Examples

### 1. Basic Communication Test

**Expected Behavior:**
- Publisher sends "Hello World: X" messages every 0.5 seconds
- Subscriber receives and logs the messages
- Both nodes should run without errors

**Verification Steps:**
1. Start the publisher in Terminal 1
2. Start the subscriber in Terminal 2
3. Observe messages being published and received
4. Stop both nodes with Ctrl+C

### 2. Topic Verification

Check that topics are being created and messages are flowing:

```bash
# List all topics
ros2 topic list

# You should see:
# /chatter
# /parameter_events
# /rosout

# Check the type of the chatter topic
ros2 topic type /chatter
# Should return: std_msgs/msg/String

# Echo messages on the chatter topic
ros2 topic echo /chatter
```

### 3. Node Verification

Verify that nodes are running correctly:

```bash
# List all nodes
ros2 node list

# You should see your nodes:
# /talker_node (or /listener_node depending on which is running)

# Get information about a specific node
ros2 node info /talker_node
```

### 4. Quality of Service (QoS) Verification

Check the QoS settings of your publisher/subscriber:

```bash
# Check publisher info
ros2 topic info /chatter -v

# This will show publisher and subscriber information
```

## Advanced Testing

### Testing with Multiple Nodes

Run multiple subscribers to test one-to-many communication:

```bash
# Terminal 1: Publisher
ros2 run ros2_basics_examples talker

# Terminal 2: Subscriber 1
ros2 run ros2_basics_examples listener

# Terminal 3: Subscriber 2
ros2 run ros2_basics_examples listener
```

All subscribers should receive the same messages from the single publisher.

### Testing Message Rates

Monitor message rates using ROS 2 tools:

```bash
# Check message rate
ros2 topic hz /chatter

# Check message delay
ros2 topic delay /chatter
```

## Troubleshooting Common Issues

### Issue 1: "Command not found" errors
**Symptoms:** `ros2: command not found` or `talker: command not found`
**Solutions:**
```bash
# Ensure ROS 2 environment is sourced
source /opt/ros/humble/setup.bash

# For C++ examples, ensure workspace is built and sourced
cd ~/ros2_ws
source install/setup.bash
```

### Issue 2: Nodes can't communicate
**Symptoms:** Publisher runs but subscriber doesn't receive messages
**Solutions:**
```bash
# Check if both nodes are on the same ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# Verify topic names match
ros2 topic list

# Check if firewall is blocking communication
# Temporarily disable firewall for testing
```

### Issue 3: Build errors for C++ examples
**Symptoms:** Compilation errors when building C++ package
**Solutions:**
```bash
# Clean the build directory
rm -rf build/ install/ log/

# Ensure dependencies are installed
sudo apt update
sudo apt install build-essential cmake

# Rebuild
colcon build --packages-select ros2_basics_examples
```

### Issue 4: Python import errors
**Symptoms:** `ModuleNotFoundError: No module named 'rclpy'`
**Solutions:**
```bash
# Ensure ROS 2 environment is sourced
source /opt/ros/humble/setup.bash

# Check Python path
python3 -c "import rclpy; print('rclpy imported successfully')"
```

## Performance Testing

### Monitoring Resource Usage

```bash
# Monitor CPU and memory usage of ROS 2 processes
htop

# Check network usage
iftop -i lo  # for local communication
```

### Message Throughput Testing

For performance evaluation, you can modify the examples to send larger messages or increase the frequency to test throughput.

## Expected Output

### Publisher Output
```
[INFO] [1612345678.123456789] [talker_node]: Publishing: 'Hello World: 0'
[INFO] [1612345679.123456789] [talker_node]: Publishing: 'Hello World: 1'
[INFO] [1612345680.123456789] [talker_node]: Publishing: 'Hello World: 2'
```

### Subscriber Output
```
[INFO] [1612345678.123456789] [listener_node]: I heard: 'Hello World: 0'
[INFO] [1612345679.123456789] [listener_node]: I heard: 'Hello World: 1'
[INFO] [1612345680.123456789] [listener_node]: I heard: 'Hello World: 2'
```

## Verification Checklist

Before proceeding, verify that:

- [ ] Publisher node starts without errors
- [ ] Subscriber node starts without errors
- [ ] Messages are successfully published and received
- [ ] Both Python and C++ examples work as expected
- [ ] Topics are correctly created and accessible
- [ ] Nodes appear in the ROS 2 node list
- [ ] Communication follows expected timing (0.5 second intervals)

## Next Steps

After successfully running and testing these basic examples:

1. Explore more complex message types
2. Learn about ROS 2 services and actions
3. Create your own custom message types
4. Move on to more advanced robotics examples
5. Integrate with simulation environments like Gazebo

## Additional Resources

- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS 2 Concepts](https://docs.ros.org/en/humble/Concepts.html)
- [ROS 2 CLI Tools](https://docs.ros.org/en/humble/Reference/CLI-Tools.html)
- [ROS 2 Quality of Service](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)

---

**Note:** Always remember to source your ROS 2 environment before running any ROS 2 commands. The examples demonstrate fundamental ROS 2 concepts that form the basis for more complex robotics applications.