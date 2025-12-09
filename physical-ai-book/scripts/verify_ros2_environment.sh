#!/bin/bash

# ROS 2 Environment Verification Script
# This script validates the complete ROS 2 environment setup for the
# Physical AI & Humanoid Robotics educational book.

set -e  # Exit on any error

echo "🧪 ROS 2 Environment Verification Script"
echo "Physical AI & Humanoid Robotics Educational Book"
echo ""

# Function to check a condition and report result
check_condition() {
    local condition="$1"
    local description="$2"

    if eval "$condition"; then
        echo "✅ $description"
        return 0
    else
        echo "❌ $description"
        return 1
    fi
}

# Function to run a command and check its output
check_command() {
    local command="$1"
    local description="$2"
    local expected="$3"

    output=$(eval "$command" 2>/dev/null) || output=""

    if [[ "$output" == *"$expected"* ]]; then
        echo "✅ $description"
        return 0
    else
        echo "❌ $description"
        return 1
    fi
}

# Check if ROS 2 environment is sourced
echo "🔍 Checking ROS 2 environment..."
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS 2 environment not sourced"
    echo "   Please run: source /opt/ros/humble/setup.bash"
    echo "   Then run this script again."
    exit 1
else
    echo "✅ ROS 2 environment sourced ($ROS_DISTRO)"
fi

# Check ROS 2 installation
echo ""
echo "🔍 Checking ROS 2 installation..."
check_condition "command -v ros2" "ROS 2 command available"
check_command "ros2 --version" "ROS 2 version is Humble" "humble"

# Check basic ROS 2 functionality
echo ""
echo "🔍 Checking basic ROS 2 functionality..."
check_condition "ros2 topic list >/dev/null 2>&1" "Topic command works"
check_condition "ros2 node list >/dev/null 2>&1" "Node command works"
check_condition "ros2 service list >/dev/null 2>&1" "Service command works"
check_condition "ros2 param list >/dev/null 2>&1" "Param command works"

# Check essential packages
echo ""
echo "🔍 Checking essential ROS 2 packages..."
check_condition "ros2 pkg list | grep -q rclpy" "rclpy package available"
check_condition "ros2 pkg list | grep -q rclcpp" "rclcpp package available"
check_condition "ros2 pkg list | grep -q std_msgs" "std_msgs package available"
check_condition "ros2 pkg list | grep -q geometry_msgs" "geometry_msgs package available"
check_condition "ros2 pkg list | grep -q sensor_msgs" "sensor_msgs package available"

# Check workspace
echo ""
echo "🔍 Checking ROS 2 workspace..."
WORKSPACE_PATH="$HOME/ros2_ws"
if [ -d "$WORKSPACE_PATH" ]; then
    echo "✅ ROS 2 workspace exists at $WORKSPACE_PATH"

    if [ -d "$WORKSPACE_PATH/src" ]; then
        echo "✅ Workspace src directory exists"
    else
        echo "❌ Workspace src directory missing"
    fi

    if [ -d "$WORKSPACE_PATH/install" ]; then
        echo "✅ Workspace install directory exists"
    else
        echo "❌ Workspace install directory missing"
    fi
else
    echo "❌ ROS 2 workspace not found at $WORKSPACE_PATH"
fi

# Check example files
echo ""
echo "🔍 Checking example files..."
EXAMPLES_DIR="$HOME/ros2_ws/examples/ros2_basics"
if [ -d "$EXAMPLES_DIR" ]; then
    echo "✅ Examples directory exists"

    if [ -f "$EXAMPLES_DIR/talker_node.py" ]; then
        echo "✅ Python talker example exists"
    else
        echo "❌ Python talker example missing"
    fi

    if [ -f "$EXAMPLES_DIR/listener_node.py" ]; then
        echo "✅ Python listener example exists"
    else
        echo "❌ Python listener example missing"
    fi
else
    echo "❌ Examples directory not found"
fi

# Check Isaac ROS packages if they should be installed
echo ""
echo "🔍 Checking Isaac ROS packages..."
if ros2 pkg list | grep -q isaac_ros; then
    echo "✅ Isaac ROS packages found"
    # List some Isaac ROS packages
    echo "   Available Isaac ROS packages:"
    ros2 pkg list | grep isaac_ros | head -5 | while read pkg; do
        echo "   - $pkg"
    done
else
    echo "ℹ️  Isaac ROS packages not found (this may be expected)"
fi

# System information
echo ""
echo "📋 System Information:"
echo "   OS: $(lsb_release -d | cut -f2)"
echo "   Architecture: $(uname -m)"
echo "   Python: $(python3 --version 2>/dev/null || echo 'Not found')"
echo "   ROS Distribution: $ROS_DISTRO"
echo "   ROS Domain ID: ${ROS_DOMAIN_ID:-0}"

# Summary
echo ""
echo "📊 Verification Summary:"
total_checks=0
failed_checks=0

# Count checks by examining script lines that call check_condition or check_command
# This is a simplified version - in practice, you'd track results individually
echo "   This script ran multiple checks to validate your ROS 2 environment."

echo ""
echo "🎉 Verification complete!"
echo "If you see mostly checkmarks (✅), your ROS 2 environment is properly set up."
echo "If you see crosses (❌), please address those issues before proceeding."

# Optional: Run a simple test
echo ""
read -p "Run a simple talker/listener test? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Running quick test. This will start a publisher for 3 seconds..."
    timeout 3s ros2 run demo_nodes_cpp talker >/dev/null 2>&1 &
    talker_pid=$!

    # Give it a moment, then check if it's running
    sleep 1
    if kill -0 $talker_pid 2>/dev/null; then
        echo "✅ Simple test passed - ROS 2 nodes can run"
        kill $talker_pid 2>/dev/null
    else
        echo "❌ Simple test failed - issue with node execution"
    fi
fi