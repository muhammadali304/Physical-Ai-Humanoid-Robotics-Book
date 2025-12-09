#!/usr/bin/env python3
"""
FPS Validation Script for Gazebo Simulation

This script validates that the Gazebo physics simulation maintains at least 30 FPS.
It monitors the simulation clock and calculates the actual frame rate.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import numpy as np
from collections import deque


class FPSValidator(Node):
    def __init__(self):
        super().__init__('fps_validator')

        # QoS profile for clock topic (best effort for performance monitoring)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to Gazebo's clock topic
        self.clock_subscription = self.create_subscription(
            Header,
            '/clock',
            self.clock_callback,
            qos_profile
        )

        # Data storage for FPS calculation
        self.time_stamps = deque(maxlen=100)  # Store last 100 timestamps
        self.fps_history = deque(maxlen=50)    # Store last 50 FPS values
        self.fps_readings = []  # Store all FPS readings for statistics

        # Performance thresholds
        self.target_fps = 30.0
        self.min_acceptable_fps = 25.0  # Allow some tolerance

        # Statistics
        self.total_samples = 0
        self.valid_samples = 0
        self.invalid_samples = 0

        # Timer for periodic reporting
        self.report_timer = self.create_timer(5.0, self.report_status)

        self.get_logger().info(f'FPS Validator started - Target: {self.target_fps} FPS')
        self.get_logger().info(f'Minimum acceptable: {self.min_acceptable_fps} FPS')

    def clock_callback(self, msg):
        """Callback for clock messages to calculate FPS"""
        current_time = msg.stamp.sec + msg.stamp.nanosec * 1e-9

        # Add timestamp to our collection
        self.time_stamps.append(current_time)

        # Calculate FPS if we have enough data points
        if len(self.time_stamps) >= 2:
            # Calculate time difference between first and last timestamp
            time_diff = self.time_stamps[-1] - self.time_stamps[0]
            num_frames = len(self.time_stamps) - 1

            if time_diff > 0:
                fps = num_frames / time_diff
                self.fps_history.append(fps)
                self.fps_readings.append(fps)
                self.total_samples += 1

                # Check if this sample meets requirements
                if fps >= self.min_acceptable_fps:
                    self.valid_samples += 1
                else:
                    self.invalid_samples += 1
                    self.get_logger().warn(f'Low FPS detected: {fps:.2f} FPS (below {self.min_acceptable_fps})')

    def report_status(self):
        """Periodically report FPS statistics"""
        if len(self.fps_history) > 0:
            current_fps = self.fps_history[-1]
            avg_fps = np.mean(self.fps_history) if self.fps_history else 0
            min_fps = min(self.fps_history) if self.fps_history else 0
            max_fps = max(self.fps_history) if self.fps_history else 0

            # Calculate success rate
            success_rate = (self.valid_samples / self.total_samples * 100) if self.total_samples > 0 else 0

            self.get_logger().info(f'--- FPS Validation Report ---')
            self.get_logger().info(f'Current FPS: {current_fps:.2f}')
            self.get_logger().info(f'Average FPS: {avg_fps:.2f}')
            self.get_logger().info(f'Min FPS: {min_fps:.2f}')
            self.get_logger().info(f'Max FPS: {max_fps:.2f}')
            self.get_logger().info(f'Total samples: {self.total_samples}')
            self.get_logger().info(f'Success rate: {success_rate:.1f}% (>= {self.min_acceptable_fps} FPS)')
            self.get_logger().info(f'---------------------------')

            # Check overall performance
            if avg_fps < self.target_fps:
                self.get_logger().warn(f'Average FPS ({avg_fps:.2f}) is below target ({self.target_fps})')
            else:
                self.get_logger().info(f'Average FPS ({avg_fps:.2f}) meets target ({self.target_fps})')

    def get_validation_results(self):
        """Return comprehensive validation results"""
        if len(self.fps_readings) == 0:
            return {
                'status': 'NO_DATA',
                'message': 'No FPS data collected yet',
                'current_fps': 0,
                'average_fps': 0,
                'min_fps': 0,
                'max_fps': 0,
                'success_rate': 0
            }

        current_fps = self.fps_history[-1] if self.fps_history else 0
        avg_fps = np.mean(self.fps_readings)
        min_fps = min(self.fps_readings)
        max_fps = max(self.fps_readings)
        success_rate = (self.valid_samples / self.total_samples * 100) if self.total_samples > 0 else 0

        # Determine overall status
        if avg_fps >= self.target_fps:
            status = 'PASS'
            message = f'Performance validation PASSED - Average FPS: {avg_fps:.2f}'
        elif avg_fps >= self.min_acceptable_fps:
            status = 'WARNING'
            message = f'Performance validation WARNING - Average FPS: {avg_fps:.2f} (below target {self.target_fps})'
        else:
            status = 'FAIL'
            message = f'Performance validation FAILED - Average FPS: {avg_fps:.2f} (below minimum {self.min_acceptable_fps})'

        return {
            'status': status,
            'message': message,
            'current_fps': current_fps,
            'average_fps': avg_fps,
            'min_fps': min_fps,
            'max_fps': max_fps,
            'success_rate': success_rate,
            'total_samples': self.total_samples,
            'valid_samples': self.valid_samples,
            'invalid_samples': self.invalid_samples
        }

    def print_final_report(self):
        """Print a final comprehensive report"""
        results = self.get_validation_results()

        print("\n" + "="*60)
        print("Gazebo Physics Simulation FPS Validation Results")
        print("="*60)
        print(f"Status: {results['status']}")
        print(f"Message: {results['message']}")
        print(f"Current FPS: {results['current_fps']:.2f}")
        print(f"Average FPS: {results['average_fps']:.2f}")
        print(f"Min FPS: {results['min_fps']:.2f}")
        print(f"Max FPS: {results['max_fps']:.2f}")
        print(f"Success Rate: {results['success_rate']:.1f}% (>= {self.min_acceptable_fps} FPS)")
        print(f"Total Samples: {results['total_samples']}")
        print(f"Valid Samples: {results['valid_samples']}")
        print(f"Invalid Samples: {results['invalid_samples']}")
        print("="*60)


def main(args=None):
    rclpy.init(args=args)

    validator = FPSValidator()

    # Run for a specified duration or until interrupted
    try:
        print("Starting FPS validation. Press Ctrl+C to stop and see results.")
        rclpy.spin(validator)
    except KeyboardInterrupt:
        validator.get_logger().info('FPS validation interrupted by user')
    finally:
        validator.print_final_report()
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()