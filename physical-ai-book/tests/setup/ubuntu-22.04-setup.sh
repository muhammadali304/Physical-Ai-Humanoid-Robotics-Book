#!/bin/bash
# ubuntu-22.04-setup.sh - Setup script for testing framework on Ubuntu 22.04

set -e

echo "🔧 Setting up testing framework for Physical AI & Humanoid Robotics on Ubuntu 22.04..."

# Update system packages
sudo apt update && sudo apt upgrade -y

# Install testing dependencies
sudo apt install -y \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    unzip \
    bash-completion \
    jq \
    shellcheck \
    nodejs \
    npm

# Install ROS 2 dependencies (without ROS 2 itself, as it will be tested separately)
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# Create Python virtual environment for testing tools
python3 -m venv ~/testing-env
source ~/testing-env/bin/activate

# Install Python testing packages
pip install --upgrade pip
pip install \
    pytest \
    pytest-cov \
    pytest-xdist \
    flake8 \
    black \
    mypy \
    bandit \
    safety

# Install Node.js testing tools (if not already installed)
npm install -g \
    markdown-link-check \
    docusaurus

# Create test directories
mkdir -p ~/test-reports
mkdir -p ~/test-environment

# Setup test configuration
cat > ~/test-environment/test-config.json << EOF
{
  "environment": {
    "os": "Ubuntu 22.04",
    "platform": "linux",
    "shell": "bash"
  },
  "testing": {
    "python_version": "3.10",
    "timeout_seconds": 60,
    "parallel_tests": 2,
    "report_format": "junit",
    "coverage_threshold": 80
  },
  "directories": {
    "examples": "../examples",
    "reports": "~/test-reports",
    "temp": "~/test-environment/temp"
  }
}
EOF

# Create a test runner specific to Ubuntu 22.04
cat > ~/test-environment/run-tests.sh << 'EOF'
#!/bin/bash
# Ubuntu 22.04 specific test runner

set -e

# Source the virtual environment
source ~/testing-env/bin/activate

# Configuration
TEST_CONFIG="$HOME/test-environment/test-config.json"
REPORTS_DIR="$HOME/test-reports/$(date +%Y%m%d-%H%M%S)"
EXAMPLES_DIR="../examples"

# Create reports directory
mkdir -p "$REPORTS_DIR"

echo "🧪 Starting tests on Ubuntu 22.04..."
echo "📅 $(date)"
echo "📂 Examples directory: $EXAMPLES_DIR"
echo "📊 Reports directory: $REPORTS_DIR"

# Run different types of tests
echo "🔍 Running code quality checks..."
if [ -d "$EXAMPLES_DIR" ]; then
  find "$EXAMPLES_DIR" -name "*.py" -exec flake8 {} \; > "$REPORTS_DIR/flake8-report.txt" 2>&1 || true
  find "$EXAMPLES_DIR" -name "*.py" -exec black --check {} \; > "$REPORTS_DIR/black-report.txt" 2>&1 || true
  find "$EXAMPLES_DIR" -name "*.py" -exec bandit -r {} \; > "$REPORTS_DIR/bandit-report.txt" 2>&1 || true
fi

echo "🔍 Running security checks..."
safety check -r requirements.txt > "$REPORTS_DIR/safety-report.txt" 2>&1 || true

echo "✅ Ubuntu 22.04 testing framework setup complete!"
echo "📋 Reports saved to: $REPORTS_DIR"
EOF

chmod +x ~/test-environment/run-tests.sh

echo "✅ Testing framework setup complete on Ubuntu 22.04!"
echo "💡 To run tests, activate the environment: source ~/testing-env/bin/activate"
echo "💡 Then run: ~/test-environment/run-tests.sh"