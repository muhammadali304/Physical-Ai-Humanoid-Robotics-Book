#!/bin/bash
# Navigation Validation Script
# Runs comprehensive navigation validation tests to verify 90% success rate

set -e  # Exit on any error

echo "==========================================="
echo "Navigation Validation Test Suite"
echo "==========================================="

# Set up environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Create results directory
mkdir -p ~/ros2_ws/src/physical_ai_examples/results

# Run the validation test
echo "Running navigation validation tests..."
cd ~/ros2_ws/src/physical_ai_examples
python3 navigation_validation/navigation_validator.py

# Check the exit code from the validation
validation_result=$?

echo "==========================================="
echo "Validation Complete"
echo "==========================================="

if [ $validation_result -eq 0 ]; then
    echo "✅ SUCCESS: Navigation system meets 90% success rate requirement"
    echo "Validation results saved in ~/ros2_ws/src/physical_ai_examples/results/"
else
    echo "❌ FAILURE: Navigation system does not meet 90% success rate requirement"
    echo "Review the logs above for details"
fi

exit $validation_result