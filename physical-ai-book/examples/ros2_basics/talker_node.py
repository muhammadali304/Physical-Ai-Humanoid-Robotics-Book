#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Talker Node - Publisher Example

This is a simple ROS 2 publisher node that publishes messages to a topic.
It's part of the Physical AI & Humanoid Robotics educational book examples.

Learning Objectives:
    1. Understand how to create a ROS 2 publisher node
    2. Learn how to publish messages to a topic
    3. Understand the publisher/subscriber communication pattern

Prerequisites:
    - ROS 2 Humble Hawksbill installed
    - Basic Python knowledge
    - Understanding of ROS 2 concepts (nodes, topics, messages)

Usage:
    ros2 run examples talker_node.py
    or
    python3 talker_node.py

Author: Physical AI & Humanoid Robotics Educational Book
Date: 2025
Version: 1.0
License: MIT
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time


class TalkerNode(Node):
    """
    A ROS 2 publisher node that sends messages to a topic.
    """

    def __init__(self):
        """Initialize the talker node."""
        super().__init__('talker_node')

        # Create a publisher for the 'chatter' topic
        # The topic name is 'chatter', message type is String, and queue size is 10
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create a timer to publish messages at regular intervals (0.5 seconds)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to keep track of messages sent
        self.i = 0

        # Log that the node has started
        self.get_logger().info('Talker node started, publishing to /chatter topic')

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


def main(args=None):
    """
    Main function to run the talker node.

    Args:
        args: Command line arguments (default: None)
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create the talker node
    talker_node = TalkerNode()

    try:
        # Start processing data from the node
        # This will block until the node is shut down
        rclpy.spin(talker_node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        talker_node.get_logger().info('Interrupted by user, shutting down...')
    finally:
        # Destroy the node explicitly
        talker_node.destroy_node()
        # Shutdown the ROS 2 client library
        rclpy.shutdown()


if __name__ == '__main__':
    main()