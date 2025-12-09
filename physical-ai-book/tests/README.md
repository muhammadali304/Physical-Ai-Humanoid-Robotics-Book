# Testing Framework for Physical AI & Humanoid Robotics Educational Book

This directory contains the automated testing framework for code examples from the educational book.

## Directory Structure

- `setup/` - Scripts for provisioning test environments
- `examples/` - Test runners and example-specific tests
- `reports/` - Test results and reports
- `config/` - Configuration files for testing

## Getting Started

### For VM Testing

1. Provision a VM with Ubuntu 22.04
2. Run the VM provisioning script:
   ```bash
   bash tests/setup/vm-provision.sh
   ```
3. Run the test framework:
   ```bash
   bash tests/examples/test-runner.sh
   ```

### Test Results

Test results are saved in the `reports/` directory with timestamps. Each test run creates a log file with detailed results.

## Configuration

The test framework can be configured using `tests/config/test-config.json`. You can adjust:
- Timeout values
- Test parallelization
- Report formats
- Environment settings

## CI/CD Integration

The framework is designed to work with CI/CD pipelines. See the documentation in the main project for integration examples.