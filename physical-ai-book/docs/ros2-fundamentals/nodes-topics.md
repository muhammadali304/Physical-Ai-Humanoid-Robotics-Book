---
sidebar_position: 1
---

# ROS 2 Nodes and Topics - Fundamentals

## Learning Objectives

By the end of this chapter, you will be able to:
- Explain the concepts of nodes and topics in ROS 2
- Create a simple ROS 2 node in Python and C++
- Implement publisher and subscriber nodes
- Understand the communication patterns between nodes
- Test and verify node communication

## Prerequisites

Before starting this chapter, you should:
- Have ROS 2 Humble Hawksbill installed and configured
- Understand basic Linux command line operations
- Have Python and C++ programming knowledge
- Completed the ROS 2 setup chapter

## Conceptual Overview

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. At its core, ROS 2 provides a communication infrastructure that allows different parts of your robot software to work together seamlessly.

### Nodes

A **node** is a process that performs computation. Nodes are the fundamental building blocks of ROS 2 programs. In a typical robot system, nodes might handle different tasks such as sensor processing, motion planning, or control.

### Topics

**Topics** enable message passing between nodes. Nodes can publish messages to topics or subscribe to topics to receive messages. This creates a publish-subscribe communication pattern where publishers and subscribers are decoupled.

### Communication Pattern

The publisher-subscriber pattern works as follows:
1. Publisher nodes send messages to a specific topic
2. Subscriber nodes receive messages from that topic
3. The ROS 2 middleware handles the delivery of messages
4. Multiple publishers can send to the same topic
5. Multiple subscribers can receive from the same topic

## Hands-On Implementation

### Python Implementation

Let's create a simple publisher node that publishes messages with a counter:

**Publisher Node (publisher_member_function.py):**

```python
#!/usr/bin/env python3
# publisher_member_function.py

"""
This example demonstrates how to create a simple publisher using a class member function to handle
publishing data being sent to a topic.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        # Calls the superclass constructor
        super().__init__('minimal_publisher')

        # Create a publisher that will publish String messages to the 'topic' topic
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Create a timer that will call the timer_callback method every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize a counter
        self.i = 0

    def timer_callback(self):
        # Create a String message
        msg = String()
        msg.data = 'Hello World: %d' % self.i

        # Publish the message
        self.publisher_.publish(msg)

        # Log the message being published
        self.get_logger().info('Publishing: "%s"' % msg.data)

        # Increment the counter
        self.i += 1


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create an instance of the MinimalPublisher class
    minimal_publisher = MinimalPublisher()

    # Spin the node so the callback function is called
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Subscriber Node (subscriber_member_function.py):**

```python
#!/usr/bin/env python3
# subscriber_member_function.py

"""
This example demonstrates how to create a simple subscriber using a class member function to handle
receiving data from a topic.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        # Calls the superclass constructor
        super().__init__('minimal_subscriber')

        # Create a subscription that will receive String messages from the 'topic' topic
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Log the received message
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create an instance of the MinimalSubscriber class
    minimal_subscriber = MinimalSubscriber()

    # Spin the node so the callback function is called
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### C++ Implementation

For completeness, here are the C++ versions of the same nodes:

**Publisher Node (minimal_publisher.cpp):**

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

**Subscriber Node (minimal_subscriber.cpp):**

```cpp
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

### Setting up the Package

To create a ROS 2 package for these examples:

1. **Create the package:**
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python py_pubsub
```

2. **Add the Python files to the package:**
```bash
# Copy the publisher to py_pubsub/py_pubsub/publisher_member_function.py
# Copy the subscriber to py_pubsub/py_pubsub/subscriber_member_function.py
```

3. **Make the Python files executable:**
```bash
chmod +x py_pubsub/py_pubsub/publisher_member_function.py
chmod +x py_pubsub/py_pubsub/subscriber_member_function.py
```

4. **Update setup.py to include entry points:**
```python
entry_points={
    'console_scripts': [
        'talker = py_pubsub.publisher_member_function:main',
        'listener = py_pubsub.subscriber_member_function:main',
    ],
},
```

5. **Build the package:**
```bash
cd ~/ros2_ws
colcon build --packages-select py_pubsub
```

6. **Source the workspace:**
```bash
source install/setup.bash
```

## Testing & Verification

### Running the Nodes

1. **Open two terminal windows**

2. **In the first terminal, source your workspace and run the publisher:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run py_pubsub talker
```

3. **In the second terminal, source your workspace and run the subscriber:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run py_pubsub listener
```

4. **You should see the publisher sending messages and the subscriber receiving them:**
```
[INFO] [1601234567.123456789] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [1601234567.623456789] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [1601234567.123456789] [minimal_subscriber]: I heard: "Hello World: 0"
[INFO] [1601234567.623456789] [minimal_subscriber]: I heard: "Hello World: 1"
```

### Useful Commands for Topic Inspection

- **List all topics:**
```bash
ros2 topic list
```

- **Get info about a specific topic:**
```bash
ros2 topic info /topic
```

- **Echo messages on a topic:**
```bash
ros2 topic echo /topic std_msgs/msg/String
```

- **Publish directly to a topic:**
```bash
ros2 topic pub /topic std_msgs/msg/String 'data: "Hello from command line"'
```

- **Check topic statistics:**
```bash
ros2 topic hz /topic
```

## Common Issues

### Issue: "Topic not found" or no communication between nodes
**Solution**:
- Verify both nodes are running
- Check that topic names match exactly
- Ensure both nodes are in the same ROS domain (ROS_DOMAIN_ID environment variable)

### Issue: Python import errors
**Solution**:
- Make sure the package was built with `colcon build`
- Source the workspace with `source install/setup.bash`
- Check that the Python files have the correct permissions

### Issue: Nodes cannot communicate between different machines
**Solution**:
- Check network connectivity
- Verify ROS_LOCALHOST_ONLY is not set to 1
- Ensure RMW_IMPLEMENTATION is consistent across machines
- Check firewall settings for DDS communication ports

## Key Takeaways

- Nodes are the basic execution units in ROS 2 that perform computation
- Topics enable decoupled communication between nodes using publisher-subscriber pattern
- Messages are the data packets sent between nodes via topics
- Multiple publishers and subscribers can use the same topic
- ROS 2 provides built-in tools for debugging and inspecting topic communication
- Both Python and C++ implementations follow the same patterns with language-specific syntax

## Next Steps

In the next chapter, you'll learn about ROS 2 services and actions, which provide request-response communication patterns that complement the publish-subscribe model you've learned here.