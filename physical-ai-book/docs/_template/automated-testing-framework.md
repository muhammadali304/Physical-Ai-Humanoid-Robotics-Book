# Automated Testing Framework for Code Examples

This framework provides tools and scripts to automatically test code examples from the Physical AI & Humanoid Robotics Educational Book in a controlled environment.

## Overview

The testing framework includes:
- Test runner scripts for different types of code examples
- VM configuration templates
- Test result reporting
- Integration with CI/CD pipelines

## Directory Structure

```
tests/
├── setup/
│   ├── vm-provision.sh          # VM provisioning script
│   ├── ros2-environment.sh      # ROS 2 setup for tests
│   └── dependencies.sh          # Install test dependencies
├── examples/
│   ├── test-runner.sh           # Generic test runner
│   ├── python-examples/         # Python example tests
│   ├── cpp-examples/            # C++ example tests
│   └── launch-files/            # ROS 2 launch file tests
├── reports/
│   └── results.json             # Test results output
└── config/
    ├── vm-config.json           # VM configuration
    └── test-config.json         # Test configuration
```

## VM Configuration

The framework assumes an Ubuntu 22.04 VM with ROS 2 Humble Hawksbill installed.

### Prerequisites
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Python 3.8+
- Git
- Docker (optional, for containerized testing)

## Test Runner Script

```bash
#!/bin/bash
# test-runner.sh - Run automated tests for code examples

set -e  # Exit on any error

# Configuration
EXAMPLES_DIR="../examples"
TEST_RESULTS_DIR="./reports"
LOG_FILE="$TEST_RESULTS_DIR/test-run-$(date +%Y%m%d-%H%M%S).log"

# Create results directory
mkdir -p "$TEST_RESULTS_DIR"

# Function to run a single test
run_test() {
    local test_file=$1
    local test_name=$(basename "$test_file")

    echo "🧪 Testing $test_name..." | tee -a "$LOG_FILE"

    # Add execution timeout to prevent hanging tests
    if timeout 60s bash -c "source /opt/ros/humble/setup.bash; cd '$EXAMPLES_DIR'; source activate.sh 2>/dev/null || true; $test_file" >> "$LOG_FILE" 2>&1; then
        echo "✅ $test_name PASSED" | tee -a "$LOG_FILE"
        return 0
    else
        echo "❌ $test_name FAILED" | tee -a "$LOG_FILE"
        return 1
    fi
}

# Main execution
echo "🚀 Starting automated tests for Physical AI & Humanoid Robotics examples..." | tee "$LOG_FILE"
echo "📅 $(date)" | tee -a "$LOG_FILE"
echo "📂 Testing examples in: $EXAMPLES_DIR" | tee -a "$LOG_FILE"
echo "" | tee -a "$LOG_FILE"

# Find and run all testable example files
FAILED_TESTS=0
TOTAL_TESTS=0

for example in $EXAMPLES_DIR/**/*.sh $EXAMPLES_DIR/**/*.py $EXAMPLES_DIR/**/*.cpp; do
    if [ -f "$example" ] && [ -x "$example" ]; then
        TOTAL_TESTS=$((TOTAL_TESTS + 1))
        if ! run_test "$example"; then
            FAILED_TESTS=$((FAILED_TESTS + 1))
        fi
        echo "" | tee -a "$LOG_FILE"
    fi
done

# Summary
echo "📊 Test Summary:" | tee -a "$LOG_FILE"
echo "   Total tests: $TOTAL_TESTS" | tee -a "$LOG_FILE"
echo "   Failed: $FAILED_TESTS" | tee -a "$LOG_FILE"
echo "   Passed: $((TOTAL_TESTS - FAILED_TESTS))" | tee -a "$LOG_FILE"

if [ $FAILED_TESTS -eq 0 ]; then
    echo "🎉 All tests passed!" | tee -a "$LOG_FILE"
    exit 0
else
    echo "💥 $FAILED_TESTS test(s) failed!" | tee -a "$LOG_FILE"
    exit 1
fi
```

## VM Provisioning Script

```bash
#!/bin/bash
# vm-provision.sh - Provision VM for testing code examples

set -e

echo "🔧 Provisioning VM for Physical AI & Humanoid Robotics testing..."

# Update system
sudo apt update && sudo apt upgrade -y

# Install basic dependencies
sudo apt install -y \
    curl \
    wget \
    git \
    python3 \
    python3-pip \
    build-essential \
    cmake \
    unzip \
    bash-completion \
    vim \
    htop

# Install ROS 2 Humble Hawksbill
echo "📦 Installing ROS 2 Humble Hawksbill..."
sudo apt install -y software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
sudo rosdep init
rosdep update

# Install Python packages for examples
pip3 install -U \
    setuptools \
    numpy \
    matplotlib \
    opencv-python \
    transforms3d

# Setup ROS environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install

echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

echo "✅ VM provisioning completed!"
```

## Test Configuration

```json
{
  "vm": {
    "os": "Ubuntu 22.04",
    "memory": "8GB",
    "cpus": 4,
    "disk": "50GB"
  },
  "testing": {
    "timeout_seconds": 60,
    "parallel_tests": 1,
    "report_format": "json",
    "include_code_coverage": false
  },
  "environments": {
    "ros2": {
      "distribution": "humble",
      "workspace_path": "~/ros2_ws"
    },
    "python": {
      "version": "3.10",
      "requirements_file": "requirements.txt"
    }
  }
}
```

## CI/CD Integration

Add to your `.github/workflows/test-examples.yml`:

```yaml
name: Test Code Examples

on:
  push:
    branches: [ main ]
  pull_request:
    branches: [ main ]

jobs:
  test-examples:
    runs-on: ubuntu-22.04

    steps:
    - uses: actions/checkout@v3

    - name: Set up ROS 2 Humble
      uses: ros-tooling/setup-ros@v0.7
      with:
        required-ros-distributions: humble

    - name: Install dependencies
      run: |
        cd tests/setup
        bash dependencies.sh

    - name: Run tests
      run: |
        cd tests/examples
        bash test-runner.sh

    - name: Upload test results
      if: always()
      uses: actions/upload-artifact@v3
      with:
        name: test-results
        path: tests/reports/
```

## Usage

1. Provision your VM using the provisioning script
2. Place code examples in the `examples/` directory
3. Run the test framework: `bash tests/examples/test-runner.sh`
4. Check results in the `tests/reports/` directory

## Reporting

Test results are saved in JSON format with details about:
- Test execution time
- Pass/fail status
- Error messages (if any)
- Resource usage
- Environment information