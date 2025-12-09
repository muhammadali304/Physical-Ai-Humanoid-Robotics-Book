#!/bin/bash
# Perception Pipeline Performance Benchmark Script
# Measures processing time to ensure <500ms latency

set -e  # Exit on any error

echo "==========================================="
echo "Perception Pipeline Performance Benchmark"
echo "==========================================="

# Set up environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Create results directory
mkdir -p ~/ros2_ws/src/physical_ai_examples/results

# Run the benchmark test
echo "Running perception performance benchmark..."
cd ~/ros2_ws/src/physical_ai_examples
python3 perception_benchmark/perception_benchmark.py

# Check the exit code from the benchmark
benchmark_result=$?

echo "==========================================="
echo "Perception Benchmark Complete"
echo "==========================================="

if [ $benchmark_result -eq 0 ]; then
    echo "✅ SUCCESS: Perception pipeline meets <500ms latency requirement"
    echo "Benchmark results saved in ~/ros2_ws/src/physical_ai_examples/results/"
else
    echo "❌ FAILURE: Perception pipeline exceeds 500ms latency requirement"
    echo "Review the logs above for details"
fi

exit $benchmark_result