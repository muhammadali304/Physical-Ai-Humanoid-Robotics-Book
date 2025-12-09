#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Listener Node - Subscriber Example

This is a simple ROS 2 subscriber node that subscribes to messages from a topic.
It's part of the Physical AI & Humanoid Robotics educational book examples.

Learning Objectives:
    1. Understand how to create a ROS 2 subscriber node
    2. Learn how to subscribe to messages from a topic
    3. Understand the publisher/subscriber communication pattern
    4. Learn how to process incoming messages

Prerequisites:
    - ROS 2 Humble Hawksbill installed
    - Basic Python knowledge
    - Understanding of ROS 2 concepts (nodes, topics, messages)
    - Talker node running to publish messages

Usage:
    ros2 run examples listener_node.py
    or
    python3 listener_node.py

Author: Physical AI & Humanoid Robotics Educational Book
Date: 2025
Version: 1.0
License: MIT
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys


class ListenerNode(Node):
    """
    A ROS 2 subscriber node that receives messages from a topic.
    """

    def __init__(self):
        """Initialize the listener node."""
        super().__init__('listener_node')

        # Create a subscription to the 'chatter' topic
        # The topic name is 'chatter', message type is String, and queue size is 10
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)

        # Set the subscription to not be lazy (keep it alive even if no publishers exist)
        self.subscription  # prevent unused variable warning

        # Log that the node has started
        self.get_logger().info('Listener node started, subscribing to /chatter topic')

    def listener_callback(self, msg):
        """
        Callback function that processes incoming messages.

        Args:
            msg: The incoming message of type String
        """
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """
    Main function to run the listener node.

    Args:
        args: Command line arguments (default: None)
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create the listener node
    listener_node = ListenerNode()

    try:
        # Start processing data from the node
        # This will block until the node is shut down
        rclpy.spin(listener_node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        listener_node.get_logger().info('Interrupted by user, shutting down...')
    finally:
        # Destroy the node explicitly
        listener_node.destroy_node()
        # Shutdown the ROS 2 client library
        rclpy.shutdown()


if __name__ == '__main__':
    main()