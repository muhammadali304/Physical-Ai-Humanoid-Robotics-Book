#!/bin/bash
# test-runner.sh - Run automated tests for code examples

set -e  # Exit on any error

# Configuration
EXAMPLES_DIR="../examples"
TEST_RESULTS_DIR="../reports"
LOG_FILE="$TEST_RESULTS_DIR/test-run-$(date +%Y%m%d-%H%M%S).log"

# Create results directory
mkdir -p "$TEST_RESULTS_DIR"

# Function to run a single test
run_test() {
    local test_file=$1
    local test_name=$(basename "$test_file")

    echo "🧪 Testing $test_name..." | tee -a "$LOG_FILE"

    # Add execution timeout to prevent hanging tests
    if timeout 60s bash -c "source /opt/ros/humble/setup.bash 2>/dev/null || true; cd '$EXAMPLES_DIR'; source activate.sh 2>/dev/null || true; $test_file" >> "$LOG_FILE" 2>&1; then
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