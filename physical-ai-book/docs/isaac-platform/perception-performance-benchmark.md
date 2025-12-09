# Perception Pipeline Performance Benchmark Guide

This document explains how to benchmark and validate that the perception pipeline processes data with latency under 500ms.

## Overview

The perception pipeline performance benchmarking system measures the time it takes for perception algorithms to process sensor data and generate meaningful outputs. This includes VSLAM, stereo processing, semantic segmentation, and other perception tasks.

## Performance Requirements

The perception pipeline must meet the following performance targets:

- **Maximum latency**: 500ms end-to-end processing time
- **Minimum throughput**: 2-10 Hz depending on the perception task
- **CPU utilization**: Under 80% on target hardware
- **Memory usage**: Stable without leaks during extended operation

## Benchmarking Components

### 1. Perception Benchmark Node
- **File**: `examples/perception_benchmark/perception_benchmark.py`
- **Purpose**: Measures processing time for perception pipeline components
- **Features**:
  - Timestamps input and output of perception nodes
  - Calculates end-to-end latency
  - Provides real-time performance statistics
  - Generates performance reports

### 2. Benchmark Test Scenarios

Different perception tasks have different performance characteristics:

- **VSLAM processing**: Visual-inertial odometry and mapping
- **Stereo depth estimation**: Depth map generation from stereo cameras
- **Semantic segmentation**: Pixel-level scene understanding
- **Object detection**: Detection and classification of objects
- **Feature extraction**: Key point and descriptor computation

### 3. Performance Metrics

Key metrics tracked during benchmarking:

- **Processing latency**: Time from input to processed output
- **Frame rate**: Processing rate in Hz
- **CPU utilization**: Percentage of CPU used by perception nodes
- **Memory usage**: RAM consumption during operation
- **GPU utilization**: GPU usage for accelerated algorithms

## Running Performance Benchmarks

### Prerequisites
- ROS 2 Humble with Isaac ROS perception packages
- Benchmark test data or simulation environment
- Performance monitoring tools (htop, nvidia-smi if applicable)

### Steps

1. **Set up the environment**:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

2. **Run the benchmark script**:
```bash
cd ~/ros2_ws/src/physical_ai_examples
python3 perception_benchmark/perception_benchmark.py
```

3. **Or use the launch file for integrated testing**:
```bash
cd ~/ros2_ws/src/physical_ai_examples
ros2 launch perception_benchmark perception_benchmark.launch.py
```

4. **Monitor performance in real-time**:
   - Use provided visualization tools
   - Check console output for real-time statistics
   - Monitor system resources with `htop` or similar tools

## Benchmark Report

The benchmark produces a comprehensive report including:

- Average, minimum, and maximum processing latencies
- Frame rates for different perception tasks
- CPU and memory usage statistics
- Whether the 500ms target was met
- Performance bottlenecks identification
- Recommendations for optimization

## Expected Results

For the benchmark to pass:
- Maximum latency must be < 500ms for all perception tasks
- Average latency should be < 250ms for optimal performance
- Frame rate should match or exceed sensor input rate
- No significant performance degradation over time

## Troubleshooting Performance Issues

If performance benchmarks fail to meet targets:

1. **Profile perception nodes**:
   - Use ROS 2 tools to identify bottlenecks
   - Check computational complexity of algorithms
   - Optimize critical code paths

2. **Hardware considerations**:
   - Verify adequate CPU/GPU resources
   - Check memory bandwidth limitations
   - Consider hardware acceleration options

3. **Algorithm optimization**:
   - Reduce image resolution if appropriate
   - Use approximate algorithms for faster processing
   - Implement multi-threading where possible

## Integration with CI/CD

The benchmark script can be integrated into continuous integration workflows:

```bash
# Example CI benchmark command
python3 perception_benchmark/perception_benchmark.py
if [ $? -eq 0 ]; then
  echo "Performance benchmark passed"
else
  echo "Performance benchmark failed - optimize perception pipeline"
  exit 1
fi
```