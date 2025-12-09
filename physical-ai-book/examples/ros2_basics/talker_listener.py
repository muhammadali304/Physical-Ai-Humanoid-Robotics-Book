#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Talker/Listener Node - Complete Publisher/Subscriber Example

This example combines both publisher and subscriber functionality in a single node.
It's part of the Physical AI & Humanoid Robotics educational book examples.

Learning Objectives:
    1. Understand combined publisher/subscriber node architecture
    2. Learn how to create multiple publishers and subscribers in one node
    3. Understand bidirectional communication patterns
    4. Learn advanced ROS 2 node design patterns

Prerequisites:
    - ROS 2 Humble Hawksbill installed
    - Basic Python knowledge
    - Understanding of ROS 2 concepts (nodes, topics, messages)

Usage:
    ros2 run examples talker_listener.py
    or
    python3 talker_listener.py

Author: Physical AI & Humanoid Robotics Educational Book
Date: 2025
Version: 1.0
License: MIT
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys


class TalkerListenerNode(Node):
    """
    A ROS 2 node that both publishes and subscribes to messages.
    """

    def __init__(self):
        """Initialize the talker/listener node."""
        super().__init__('talker_listener_node')

        # Create a publisher for the 'chatter' topic
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create a subscription to the 'chatter' topic
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Create a publisher for echo messages
        self.echo_publisher = self.create_publisher(String, 'echo', 10)

        # Create a timer to publish messages at regular intervals (1.0 seconds)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to keep track of messages sent
        self.i = 0

        # Log that the node has started
        self.get_logger().info('Talker/Listener node started')
        self.get_logger().info('Publishing to /chatter, echoing to /echo, and subscribing to /chatter')

    def timer_callback(self):
        """Callback function that publishes messages at regular intervals."""
        # Create a message
        msg = String()
        msg.data = f'Hello World: {self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the message being sent
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter
        self.i += 1

    def listener_callback(self, msg):
        """
        Callback function that processes incoming messages.

        Args:
            msg: The incoming message of type String
        """
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')

        # Echo the message to another topic
        echo_msg = String()
        echo_msg.data = f'Echo: {msg.data}'
        self.echo_publisher.publish(echo_msg)


def main(args=None):
    """
    Main function to run the talker/listener node.

    Args:
        args: Command line arguments (default: None)
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create the talker/listener node
    talker_listener_node = TalkerListenerNode()

    try:
        # Start processing data from the node
        # This will block until the node is shut down
        rclpy.spin(talker_listener_node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        talker_listener_node.get_logger().info('Interrupted by user, shutting down...')
    finally:
        # Destroy the node explicitly
        talker_listener_node.destroy_node()
        # Shutdown the ROS 2 client library
        rclpy.shutdown()


if __name__ == '__main__':
    main()