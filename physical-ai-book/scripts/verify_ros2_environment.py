#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 2 Environment Verification Script

This script validates the complete ROS 2 environment setup for the
Physical AI & Humanoid Robotics educational book. It checks for proper
installation of ROS 2, required packages, environment configuration,
and basic functionality.

Learning Objectives:
    1. Validate complete ROS 2 installation
    2. Check environment configuration
    3. Verify basic functionality
    4. Identify and report issues

Prerequisites:
    - ROS 2 Humble Hawksbill installed
    - Environment properly sourced
    - Basic Python knowledge

Usage:
    python3 verify_ros2_environment.py
    python3 verify_ros2_environment.py --verbose
    python3 verify_ros2_environment.py --fix  # Attempt to fix some issues

Author: Physical AI & Humanoid Robotics Educational Book
Date: 2025
Version: 1.0
License: MIT
"""

import os
import sys
import subprocess
import argparse
import shutil
import platform
from pathlib import Path
from typing import List, Dict, Tuple, Optional


class ROS2EnvironmentVerifier:
    """Class to verify ROS 2 environment setup."""

    def __init__(self, verbose: bool = False, fix_mode: bool = False):
        """
        Initialize the verifier.

        Args:
            verbose: Enable verbose output
            fix_mode: Enable fix mode for automatic corrections
        """
        self.verbose = verbose
        self.fix_mode = fix_mode
        self.results = {}
        self.issues_found = []

    def run_command(self, cmd: List[str], capture_output: bool = True) -> Tuple[int, str, str]:
        """
        Run a shell command and return the result.

        Args:
            cmd: Command to run as a list of strings
            capture_output: Whether to capture output

        Returns:
            Tuple of (return_code, stdout, stderr)
        """
        try:
            if capture_output:
                result = subprocess.run(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=30
                )
                return result.returncode, result.stdout, result.stderr
            else:
                result = subprocess.run(cmd, timeout=30)
                return result.returncode, "", ""
        except subprocess.TimeoutExpired:
            return -1, "", "Command timed out"
        except Exception as e:
            return -1, "", str(e)

    def check_system_requirements(self) -> Dict[str, bool]:
        """Check basic system requirements."""
        print("🔍 Checking system requirements...")

        results = {}

        # Check OS
        system = platform.system()
        release = platform.release()
        distro = ""

        if system == "Linux":
            try:
                with open("/etc/os-release", "r") as f:
                    for line in f:
                        if line.startswith("NAME="):
                            distro = line.split("=")[1].strip().strip('"')
                        elif line.startswith("VERSION_ID="):
                            version = line.split("=")[1].strip().strip('"')
                            break
            except:
                distro = "Unknown"

        results["os_correct"] = "Ubuntu 22.04" in distro or "jammy" in distro.lower()

        if self.verbose:
            print(f"   OS: {distro} (Ubuntu 22.04 expected)")

        # Check architecture
        arch = platform.machine()
        results["arch_correct"] = arch in ["x86_64", "amd64"]

        if self.verbose:
            print(f"   Architecture: {arch} (x86_64 expected)")

        # Check Python version
        python_version = sys.version_info
        results["python_version"] = python_version >= (3, 8)

        if self.verbose:
            print(f"   Python: {python_version.major}.{python_version.minor}.{python_version.micro} (3.8+ expected)")

        return results

    def check_ros2_installation(self) -> Dict[str, bool]:
        """Check ROS 2 installation."""
        print("🔍 Checking ROS 2 installation...")

        results = {}

        # Check if ROS 2 is installed
        results["ros2_installed"] = shutil.which("ros2") is not None

        if results["ros2_installed"]:
            # Check ROS 2 version
            ret, stdout, stderr = self.run_command(["ros2", "--version"])
            results["ros2_version_correct"] = ret == 0 and "humble" in stdout.lower()

            if self.verbose and results["ros2_version_correct"]:
                print(f"   ROS 2 version: {stdout.strip()}")
        else:
            results["ros2_version_correct"] = False
            if self.verbose:
                print("   ❌ ROS 2 not found in PATH")

        # Check if environment is sourced
        ros_distro = os.environ.get("ROS_DISTRO", "")
        results["env_sourced"] = "humble" in ros_distro.lower()

        if self.verbose:
            print(f"   ROS_DISTRO: {ros_distro}")

        return results

    def check_ros2_packages(self) -> Dict[str, bool]:
        """Check for essential ROS 2 packages."""
        print("🔍 Checking essential ROS 2 packages...")

        results = {}

        essential_packages = [
            "rclcpp",
            "rclpy",
            "std_msgs",
            "geometry_msgs",
            "sensor_msgs",
            "nav_msgs",
            "tf2_msgs",
            "builtin_interfaces"
        ]

        # Get list of available packages
        ret, stdout, stderr = self.run_command(["ros2", "pkg", "list"])
        available_packages = stdout.split() if ret == 0 else []

        for pkg in essential_packages:
            results[f"pkg_{pkg}_available"] = pkg in available_packages
            if self.verbose and not results[f"pkg_{pkg}_available"]:
                print(f"   ❌ Package {pkg} not found")

        # Check for Isaac ROS packages (if installed)
        isaac_packages = [
            "isaac_ros_apriltag",
            "isaac_ros_visual_odometry",
            "isaac_ros_compressed_image_transport"
        ]

        for pkg in isaac_packages:
            results[f"isaac_pkg_{pkg}_available"] = pkg in available_packages
            if self.verbose and pkg in available_packages:
                print(f"   ✅ Isaac ROS package {pkg} found")

        return results

    def check_workspace_setup(self) -> Dict[str, bool]:
        """Check ROS 2 workspace setup."""
        print("🔍 Checking ROS 2 workspace setup...")

        results = {}

        # Check if workspace directory exists
        ros_workspace = os.path.expanduser("~/ros2_ws")
        results["workspace_exists"] = os.path.isdir(ros_workspace)

        if results["workspace_exists"]:
            # Check for src directory
            src_dir = os.path.join(ros_workspace, "src")
            results["workspace_src_exists"] = os.path.isdir(src_dir)

            # Check for install directory
            install_dir = os.path.join(ros_workspace, "install")
            results["workspace_install_exists"] = os.path.isdir(install_dir)

            # Check if workspace is sourced
            workspace_setup = os.path.join(install_dir, "setup.bash")
            results["workspace_sourced"] = os.path.exists(workspace_setup)

        else:
            results["workspace_src_exists"] = False
            results["workspace_install_exists"] = False
            results["workspace_sourced"] = False

        if self.verbose:
            print(f"   Workspace exists: {results['workspace_exists']}")
            print(f"   Source directory: {results.get('workspace_src_exists', False)}")
            print(f"   Install directory: {results.get('workspace_install_exists', False)}")

        return results

    def check_basic_functionality(self) -> Dict[str, bool]:
        """Check basic ROS 2 functionality."""
        print("🔍 Checking basic ROS 2 functionality...")

        results = {}

        # Test ros2 topic command
        ret, stdout, stderr = self.run_command(["ros2", "topic", "list"])
        results["topic_command_works"] = ret == 0

        # Test ros2 node command
        ret, stdout, stderr = self.run_command(["ros2", "node", "list"])
        results["node_command_works"] = ret == 0

        # Test parameter command
        ret, stdout, stderr = self.run_command(["ros2", "param", "list"])
        results["param_command_works"] = ret == 0

        # Test service command
        ret, stdout, stderr = self.run_command(["ros2", "service", "list"])
        results["service_command_works"] = ret == 0

        if self.verbose:
            print(f"   Topic command: {results['topic_command_works']}")
            print(f"   Node command: {results['node_command_works']}")
            print(f"   Param command: {results['param_command_works']}")
            print(f"   Service command: {results['service_command_works']}")

        return results

    def check_python_environment(self) -> Dict[str, bool]:
        """Check Python environment for ROS 2."""
        print("🔍 Checking Python environment...")

        results = {}

        # Try to import essential ROS 2 Python packages
        try:
            import rclpy
            results["rclpy_importable"] = True
        except ImportError:
            results["rclpy_importable"] = False

        try:
            import std_msgs
            results["std_msgs_importable"] = True
        except ImportError:
            results["std_msgs_importable"] = False

        try:
            import geometry_msgs
            results["geometry_msgs_importable"] = True
        except ImportError:
            results["geometry_msgs_importable"] = False

        if self.verbose:
            print(f"   rclpy importable: {results['rclpy_importable']}")
            print(f"   std_msgs importable: {results['std_msgs_importable']}")
            print(f"   geometry_msgs importable: {results['geometry_msgs_importable']}")

        return results

    def check_examples(self) -> Dict[str, bool]:
        """Check if example files exist."""
        print("🔍 Checking example files...")

        results = {}

        # Check for Python examples
        python_example_path = os.path.expanduser("~/ros2_ws/examples/ros2_basics/talker_node.py")
        results["python_example_exists"] = os.path.exists(python_example_path)

        # Check for C++ examples
        cpp_example_path = os.path.expanduser("~/ros2_ws/src/ros2_basics_examples/talker.cpp")
        results["cpp_example_exists"] = os.path.exists(cpp_example_path)

        # Check for documentation
        setup_docs_path = os.path.expanduser("~/ros2_ws/docs/setup/running-examples.md")
        results["setup_docs_exist"] = os.path.exists(setup_docs_path)

        if self.verbose:
            print(f"   Python example: {results['python_example_exists']}")
            print(f"   C++ example: {results['cpp_example_exists']}")
            print(f"   Setup docs: {results['setup_docs_exist']}")

        return results

    def run_complete_verification(self) -> Dict[str, bool]:
        """Run complete verification of ROS 2 environment."""
        print("🚀 Starting ROS 2 Environment Verification\n")

        # Run all checks
        results = {}
        results.update(self.check_system_requirements())
        results.update(self.check_ros2_installation())
        results.update(self.check_ros2_packages())
        results.update(self.check_workspace_setup())
        results.update(self.check_basic_functionality())
        results.update(self.check_python_environment())
        results.update(self.check_examples())

        # Calculate summary
        total_checks = len(results)
        passed_checks = sum(1 for result in results.values() if result)
        failed_checks = total_checks - passed_checks

        print(f"\n📊 Verification Summary:")
        print(f"   Total checks: {total_checks}")
        print(f"   Passed: {passed_checks}")
        print(f"   Failed: {failed_checks}")
        print(f"   Success rate: {passed_checks/total_checks*100:.1f}%")

        # Determine overall status
        overall_success = failed_checks == 0
        print(f"\n{'✅' if overall_success else '❌'} Overall status: {'SUCCESS' if overall_success else 'ISSUES FOUND'}")

        # Print detailed results for failed checks
        if failed_checks > 0:
            print(f"\n❌ Failed checks:")
            for check, result in results.items():
                if not result:
                    print(f"   - {check}")

        return results

    def generate_report(self, results: Dict[str, bool]) -> str:
        """Generate a detailed report of the verification."""
        report = []
        report.append("# ROS 2 Environment Verification Report\n")
        report.append(f"Date: {__import__('datetime').datetime.now()}\n")

        report.append("## System Information")
        report.append(f"- Platform: {platform.platform()}")
        report.append(f"- Python Version: {sys.version}")
        report.append(f"- ROS Distribution: {os.environ.get('ROS_DISTRO', 'Not sourced')}\n")

        report.append("## Verification Results")
        for check, result in results.items():
            status = "✅ PASS" if result else "❌ FAIL"
            report.append(f"- {check}: {status}")

        report.append(f"\n## Summary")
        total_checks = len(results)
        passed_checks = sum(1 for result in results.values() if result)
        report.append(f"- Total checks: {total_checks}")
        report.append(f"- Passed: {passed_checks}")
        report.append(f"- Failed: {total_checks - passed_checks}")
        report.append(f"- Success rate: {passed_checks/total_checks*100:.1f}%")

        return "\n".join(report)

    def save_report(self, results: Dict[str, bool], filename: str = "ros2_verification_report.md"):
        """Save the verification report to a file."""
        report = self.generate_report(results)

        with open(filename, "w") as f:
            f.write(report)

        print(f"\n📋 Report saved to: {os.path.abspath(filename)}")


def main():
    """Main function to run the verification script."""
    parser = argparse.ArgumentParser(
        description="Verify ROS 2 environment setup for Physical AI & Humanoid Robotics"
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Enable verbose output"
    )
    parser.add_argument(
        "--fix",
        action="store_true",
        help="Attempt to fix some issues automatically"
    )
    parser.add_argument(
        "--report-file",
        "-r",
        default="ros2_verification_report.md",
        help="Report file name (default: ros2_verification_report.md)"
    )

    args = parser.parse_args()

    print("🧪 ROS 2 Environment Verification Tool")
    print("Physical AI & Humanoid Robotics Educational Book\n")

    # Check if ROS 2 environment is likely sourced
    if "ROS_DISTRO" not in os.environ:
        print("⚠️  ROS 2 environment may not be sourced!")
        print("   Please run: source /opt/ros/humble/setup.bash")
        print("   Then run this script again.\n")

        # Ask user if they want to continue anyway
        response = input("Continue anyway? (y/N): ").lower().strip()
        if response not in ['y', 'yes']:
            print("Exiting. Please source ROS 2 environment and run again.")
            return 1

    # Create verifier and run checks
    verifier = ROS2EnvironmentVerifier(verbose=args.verbose, fix_mode=args.fix)
    results = verifier.run_complete_verification()

    # Save report
    verifier.save_report(results, args.report_file)

    # Return appropriate exit code
    total_checks = len(results)
    passed_checks = sum(1 for result in results.values() if result)
    failed_checks = total_checks - passed_checks

    if failed_checks == 0:
        print("\n🎉 ROS 2 environment verification completed successfully!")
        return 0
    else:
        print(f"\n⚠️  ROS 2 environment verification completed with {failed_checks} issues.")
        print("Check the report for details and recommended fixes.")
        return 1


if __name__ == "__main__":
    sys.exit(main())