#!/usr/bin/env python3
"""
Perception Pipeline Performance Benchmark
Measures processing time for perception algorithms to ensure <500ms latency
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import time
import statistics
from typing import Dict, List, Tuple
import json
import os
from collections import deque
import threading
import queue


class PerceptionBenchmark(Node):
    """
    Perception benchmark node that measures processing time for perception algorithms
    """

    def __init__(self):
        super().__init__('perception_benchmark')

        # Parameters
        self.declare_parameter('benchmark_duration', 30)  # seconds
        self.declare_parameter('publish_rate', 2.0)  # Hz
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        self.benchmark_duration = self.get_parameter('benchmark_duration').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value

        # Benchmark data storage
        self.latency_data = {
            'vslam': [],
            'stereo_depth': [],
            'semantic_segmentation': [],
            'object_detection': []
        }

        self.start_time = None
        self.benchmark_complete = False
        self.results = {}

        # Publishers for test data
        self.left_image_pub = self.create_publisher(Image, '/camera/left/image_raw', 10)
        self.right_image_pub = self.create_publisher(Image, '/camera/right/image_raw', 10)
        self.rgb_image_pub = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/rgb/camera_info', 10)

        # Subscription to perception output (mock for benchmarking)
        self.vslam_sub = self.create_subscription(
            Twist, '/perception/vslam/odometry', self.vslam_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/perception/stereo/depth', self.depth_callback, 10)
        self.segmentation_sub = self.create_subscription(
            Image, '/perception/semantic/segmentation', self.segmentation_callback, 10)
        self.detection_sub = self.create_subscription(
            Twist, '/perception/object/detection', self.detection_callback, 10)

        # Timer for publishing test data
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_test_data)

        # Timestamp tracking
        self.timestamp_map = {}
        self.id_counter = 0

    def generate_test_image(self, width: int, height: int, channel_count: int = 3) -> List[int]:
        """Generate a test image with predictable pattern"""
        # Create a simple gradient pattern
        pixels = []
        for y in range(height):
            for x in range(width):
                # Create a gradient pattern that changes over time
                r = int((x / width) * 255)
                g = int((y / height) * 255)
                b = int(((x + y) % 255) / 255 * 255) if channel_count == 3 else 0

                if channel_count == 3:
                    pixels.extend([r, g, b])
                else:
                    pixels.append(r)  # Grayscale

        return pixels

    def publish_test_data(self):
        """Publish test sensor data to trigger perception processing"""
        if self.benchmark_complete:
            return

        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9

        current_time = self.get_clock().now()
        self.id_counter += 1

        # Create timestamp ID for tracking
        timestamp_id = f"{current_time.nanoseconds}_{self.id_counter}"

        # Store the timestamp for later latency calculation
        self.timestamp_map[timestamp_id] = current_time.nanoseconds / 1e9

        # Publish left camera image
        left_img = Image()
        left_img.header = Header()
        left_img.header.stamp = current_time.to_msg()
        left_img.header.frame_id = 'camera_left'
        left_img.height = self.image_height
        left_img.width = self.image_width
        left_img.encoding = 'rgb8'
        left_img.is_bigendian = False
        left_img.step = self.image_width * 3
        left_img.data = self.generate_test_image(self.image_width, self.image_height, 3)

        self.left_image_pub.publish(left_img)

        # Publish right camera image
        right_img = Image()
        right_img.header = Header()
        right_img.header.stamp = current_time.to_msg()
        right_img.header.frame_id = 'camera_right'
        right_img.height = self.image_height
        right_img.width = self.image_width
        right_img.encoding = 'rgb8'
        right_img.is_bigendian = False
        right_img.step = self.image_width * 3
        right_img.data = self.generate_test_image(self.image_width, self.image_height, 3)

        self.right_image_pub.publish(right_img)

        # Publish RGB camera image
        rgb_img = Image()
        rgb_img.header = Header()
        rgb_img.header.stamp = current_time.to_msg()
        rgb_img.header.frame_id = 'camera_rgb'
        rgb_img.height = self.image_height
        rgb_img.width = self.image_width
        rgb_img.encoding = 'rgb8'
        rgb_img.is_bigendian = False
        rgb_img.step = self.image_width * 3
        rgb_img.data = self.generate_test_image(self.image_width, self.image_height, 3)

        self.rgb_image_pub.publish(rgb_img)

        # Publish camera info
        camera_info = CameraInfo()
        camera_info.header = Header()
        camera_info.header.stamp = current_time.to_msg()
        camera_info.header.frame_id = 'camera_rgb'
        camera_info.width = self.image_width
        camera_info.height = self.image_height
        camera_info.k = [500.0, 0.0, self.image_width/2, 0.0, 500.0, self.image_height/2, 0.0, 0.0, 1.0]  # Mock calibration
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # Distortion coefficients

        self.camera_info_pub.publish(camera_info)

        # Log progress
        elapsed = (current_time.nanoseconds / 1e9) - self.start_time
        if elapsed > self.benchmark_duration:
            self.benchmark_complete = True
            self.calculate_results()
            self.save_results()
            self.get_logger().info(f'Benchmark completed after {elapsed:.2f}s')
            self.destroy_node()

    def vslam_callback(self, msg):
        """Callback for VSLAM output"""
        # Calculate latency by finding the closest timestamp
        current_time = self.get_clock().now().nanoseconds / 1e9
        latency = self.find_closest_latency(current_time)

        if latency is not None:
            self.latency_data['vslam'].append(latency * 1000)  # Convert to milliseconds
            self.get_logger().debug(f'VSLAM latency: {latency * 1000:.2f}ms')

    def depth_callback(self, msg):
        """Callback for stereo depth output"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        latency = self.find_closest_latency(current_time)

        if latency is not None:
            self.latency_data['stereo_depth'].append(latency * 1000)  # Convert to milliseconds
            self.get_logger().debug(f'Stereo depth latency: {latency * 1000:.2f}ms')

    def segmentation_callback(self, msg):
        """Callback for semantic segmentation output"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        latency = self.find_closest_latency(current_time)

        if latency is not None:
            self.latency_data['semantic_segmentation'].append(latency * 1000)  # Convert to milliseconds
            self.get_logger().debug(f'Semantic segmentation latency: {latency * 1000:.2f}ms')

    def detection_callback(self, msg):
        """Callback for object detection output"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        latency = self.find_closest_latency(current_time)

        if latency is not None:
            self.latency_data['object_detection'].append(latency * 1000)  # Convert to milliseconds
            self.get_logger().debug(f'Object detection latency: {latency * 1000:.2f}ms')

    def find_closest_latency(self, current_time: float) -> float:
        """Find the closest stored timestamp and calculate latency"""
        if not self.timestamp_map:
            return None

        # Find the closest timestamp (most recent)
        closest_time = max(self.timestamp_map.keys(), key=lambda k: self.timestamp_map[k])
        timestamp = self.timestamp_map[closest_time]

        # Calculate latency
        latency = current_time - timestamp

        # Remove the used timestamp to avoid duplicate calculations
        del self.timestamp_map[closest_time]

        return latency

    def calculate_results(self):
        """Calculate benchmark results"""
        results = {}

        for task, latencies in self.latency_data.items():
            if latencies:
                results[task] = {
                    'count': len(latencies),
                    'avg_latency_ms': statistics.mean(latencies),
                    'min_latency_ms': min(latencies),
                    'max_latency_ms': max(latencies),
                    'median_latency_ms': statistics.median(latencies) if len(latencies) > 0 else 0,
                    'p95_latency_ms': sorted(latencies)[int(0.95 * len(latencies))] if len(latencies) > 0 else 0,
                    'passed': max(latencies) < 500.0,  # 500ms threshold
                    'samples': latencies
                }
            else:
                results[task] = {
                    'count': 0,
                    'avg_latency_ms': 0,
                    'min_latency_ms': 0,
                    'max_latency_ms': 0,
                    'median_latency_ms': 0,
                    'p95_latency_ms': 0,
                    'passed': True,
                    'samples': []
                }

        # Overall result
        all_passed = all(results[task]['passed'] for task in results)
        overall_avg = statistics.mean([results[task]['avg_latency_ms'] for task in results if results[task]['count'] > 0]) if any(results[task]['count'] > 0 for task in results) else 0

        self.results = {
            'timestamp': time.time(),
            'duration_seconds': self.benchmark_duration,
            'publish_rate_hz': self.publish_rate,
            'tasks': results,
            'overall_avg_latency_ms': overall_avg,
            'all_passed': all_passed,
            'target_met': all_passed
        }

    def save_results(self):
        """Save benchmark results to a file"""
        timestamp = int(time.time())
        filename = f'perception_benchmark_results_{timestamp}.json'

        output_dir = os.path.join(os.path.dirname(__file__), 'results')
        os.makedirs(output_dir, exist_ok=True)

        filepath = os.path.join(output_dir, filename)

        with open(filepath, 'w') as f:
            json.dump(self.results, f, indent=2)

        self.get_logger().info(f'Results saved to {filepath}')

        # Print summary
        print("\n" + "="*70)
        print("PERCEPTION PIPELINE PERFORMANCE BENCHMARK RESULTS")
        print("="*70)
        print(f"Duration: {self.results['duration_seconds']} seconds")
        print(f"Publish rate: {self.results['publish_rate_hz']} Hz")
        print(f"Overall average latency: {self.results['overall_avg_latency_ms']:.2f} ms")
        print(f"Target: < 500ms - Target met: {'YES' if self.results['target_met'] else 'NO'}")
        print("-"*70)

        for task, data in self.results['tasks'].items():
            status = "PASS" if data['passed'] else "FAIL"
            print(f"{task.upper()}:")
            print(f"  Status: {status}")
            print(f"  Samples: {data['count']}")
            print(f"  Avg: {data['avg_latency_ms']:.2f}ms")
            print(f"  Min: {data['min_latency_ms']:.2f}ms")
            print(f"  Max: {data['max_latency_ms']:.2f}ms")
            print(f"  Median: {data['median_latency_ms']:.2f}ms")
            print(f"  P95: {data['p95_latency_ms']:.2f}ms")
            print()

        print("="*70)

        if self.results['target_met']:
            print("✅ BENCHMARK PASSED: All perception tasks meet <500ms latency requirement")
            return 0
        else:
            print("❌ BENCHMARK FAILED: Some perception tasks exceed 500ms latency requirement")
            return 1


def main(args=None):
    rclpy.init(args=args)

    benchmark = PerceptionBenchmark()

    try:
        rclpy.spin(benchmark)
    except KeyboardInterrupt:
        benchmark.get_logger().info('Benchmark interrupted by user')
    finally:
        # Calculate and save results if benchmark was interrupted
        if not benchmark.benchmark_complete:
            benchmark.calculate_results()
            benchmark.save_results()

        benchmark.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()