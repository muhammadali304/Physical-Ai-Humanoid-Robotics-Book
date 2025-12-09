#!/usr/bin/env python3
"""
Navigation Validator - Tests navigation success rate to waypoints

This script validates that the navigation system achieves at least 90% success rate
when navigating to specified waypoints in the simulation environment.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
import time
import random
import math
from typing import List, Tuple, Dict
import json
import os


class NavigationValidator(Node):
    """
    Navigation validation node that tests navigation success rate to waypoints
    """

    def __init__(self):
        super().__init__('navigation_validator')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.results = []
        self.test_count = 0
        self.success_count = 0

        # Define test waypoints in the simulation environment
        # These are coordinates that should be reachable in the test world
        self.test_waypoints = [
            # Center area (easy)
            (0.0, 0.0, 0.0),
            (1.0, 1.0, 0.0),
            (-1.0, -1.0, 0.0),
            (2.0, 0.0, 0.0),
            (0.0, 2.0, 0.0),

            # Slightly more challenging areas
            (3.0, 3.0, 0.0),
            (-2.0, 4.0, 0.0),
            (4.0, -2.0, 0.0),
            (-3.0, -3.0, 0.0),

            # Edge cases that might be challenging
            (5.0, 5.0, 0.0),
            (-5.0, -5.0, 0.0),
            (6.0, 0.0, 0.0),
        ]

        # Add some random waypoints for comprehensive testing
        for _ in range(10):
            x = random.uniform(-6.0, 6.0)
            y = random.uniform(-6.0, 6.0)
            self.test_waypoints.append((x, y, 0.0))

    def send_goal_async(self, x: float, y: float, theta: float = 0.0) -> bool:
        """Send a navigation goal to the specified coordinates"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Set position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Set orientation (simple orientation for now - facing forward)
        # Convert angle to quaternion
        import math
        cos_half_theta = math.cos(theta / 2.0)
        sin_half_theta = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = cos_half_theta
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = sin_half_theta

        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)

        return future

    def wait_for_result(self, goal_handle, timeout: float = 30.0) -> bool:
        """Wait for navigation result with timeout"""
        start_time = time.time()

        while not goal_handle.done():
            if time.time() - start_time > timeout:
                self.get_logger().info(f'Timeout waiting for navigation result after {timeout}s')
                return False
            time.sleep(0.1)

        result = goal_handle.result()
        if result is None:
            self.get_logger().info('Navigation failed - no result received')
            return False

        # Check if navigation was successful
        # The result structure depends on the NavigateToPose action definition
        # For Nav2, if the goal completes without error, it's generally successful
        status = goal_handle.status
        if status == 3:  # SUCCEEDED
            self.get_logger().info(f'Navigation to ({goal_handle.goal.pose.pose.position.x:.2f}, {goal_handle.goal.pose.pose.position.y:.2f}) successful')
            return True
        else:
            self.get_logger().info(f'Navigation to ({goal_handle.goal.pose.pose.position.x:.2f}, {goal_handle.goal.pose.pose.position.y:.2f}) failed with status: {status}')
            return False

    def run_validation_test(self, x: float, y: float, theta: float = 0.0) -> Dict:
        """Run a single navigation validation test"""
        self.get_logger().info(f'Starting navigation test to ({x:.2f}, {y:.2f})')

        # Send navigation goal
        future = self.send_goal_async(x, y, theta)

        # Wait for result with timeout
        success = self.wait_for_result(future, timeout=60.0)  # 60 second timeout

        # Record result
        result = {
            'waypoint': (x, y, theta),
            'success': success,
            'timestamp': time.time()
        }

        self.results.append(result)

        if success:
            self.success_count += 1

        self.test_count += 1
        success_rate = (self.success_count / self.test_count) * 100 if self.test_count > 0 else 0

        self.get_logger().info(f'Test {self.test_count}: {"SUCCESS" if success else "FAILURE"} - Current success rate: {success_rate:.1f}%')

        # Wait a bit between tests to allow system to reset
        time.sleep(2.0)

        return result

    def run_comprehensive_validation(self, max_tests: int = 20) -> Dict:
        """Run comprehensive validation across multiple waypoints"""
        self.get_logger().info(f'Starting comprehensive navigation validation with up to {max_tests} tests')

        # Use a subset of waypoints to keep the test reasonable
        waypoints_to_test = self.test_waypoints[:max_tests]

        for i, (x, y, theta) in enumerate(waypoints_to_test):
            self.get_logger().info(f'Running test {i+1}/{len(waypoints_to_test)} to ({x:.2f}, {y:.2f})')
            self.run_validation_test(x, y, theta)

        # Calculate final statistics
        success_rate = (self.success_count / self.test_count) * 100 if self.test_count > 0 else 0

        summary = {
            'total_tests': self.test_count,
            'successful_tests': self.success_count,
            'failed_tests': self.test_count - self.success_count,
            'success_rate': success_rate,
            'target_rate': 90.0,
            'met_target': success_rate >= 90.0,
            'results': self.results
        }

        self.get_logger().info(f'Validation complete: {success_rate:.1f}% success rate ({self.success_count}/{self.test_count})')
        self.get_logger().info(f'Target: 90% - Target met: {summary["met_target"]}')

        return summary

    def save_results(self, results: Dict, filename: str = None):
        """Save validation results to a file"""
        if filename is None:
            timestamp = int(time.time())
            filename = f'navigation_validation_results_{timestamp}.json'

        output_dir = os.path.join(os.path.dirname(__file__), 'results')
        os.makedirs(output_dir, exist_ok=True)

        filepath = os.path.join(output_dir, filename)

        with open(filepath, 'w') as f:
            json.dump(results, f, indent=2)

        self.get_logger().info(f'Results saved to {filepath}')


def main(args=None):
    rclpy.init(args=args)

    validator = NavigationValidator()

    # Run comprehensive validation
    results = validator.run_comprehensive_validation(max_tests=20)

    # Save results
    validator.save_results(results)

    # Print summary
    print("\n" + "="*60)
    print("NAVIGATION VALIDATION SUMMARY")
    print("="*60)
    print(f"Total tests: {results['total_tests']}")
    print(f"Successful:  {results['successful_tests']}")
    print(f"Failed:      {results['failed_tests']}")
    print(f"Success rate: {results['success_rate']:.1f}%")
    print(f"Target rate:  {results['target_rate']:.1f}%")
    print(f"Target met:   {'YES' if results['met_target'] else 'NO'}")
    print("="*60)

    # Determine if validation passed
    if results['met_target']:
        print("✅ VALIDATION PASSED: Navigation system meets 90% success rate requirement")
        exit_code = 0
    else:
        print("❌ VALIDATION FAILED: Navigation system does not meet 90% success rate requirement")
        exit_code = 1

    # Shutdown
    validator.destroy_node()
    rclpy.shutdown()

    return exit_code


if __name__ == '__main__':
    main()