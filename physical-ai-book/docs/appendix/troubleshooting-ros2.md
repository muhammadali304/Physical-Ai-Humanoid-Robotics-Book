# Troubleshooting Guide for ROS 2 Installation and Setup

## Overview

This comprehensive troubleshooting guide addresses common issues encountered during ROS 2 Humble Hawksbill installation and setup on Ubuntu 22.04. Each section includes symptoms, causes, and step-by-step solutions for resolving problems.

## Installation Issues

### Issue 1: Repository Key Not Found

**Symptoms:**
- `W: GPG error: http://packages.ros.org/ros2/ubuntu jammy InRelease: The following signatures couldn't be verified because the public key is not available`
- `E: The repository 'http://packages.ros.org/ros2/ubuntu jammy InRelease' is not signed`

**Cause:** The ROS 2 GPG key is not properly added to the system.

**Solution:**
```bash
# Download and add the ROS 2 GPG key
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# Verify the key was added
ls -la /usr/share/keyrings/ros-archive-keyring.gpg

# Update package lists
sudo apt update
```

### Issue 2: Package Not Found During Installation

**Symptoms:**
- `E: Unable to locate package ros-humble-desktop`
- `E: Package 'ros-humble-desktop' has no installation candidate`

**Cause:** ROS 2 repository not properly added or package name incorrect.

**Solution:**
```bash
# Verify repository is added
cat /etc/apt/sources.list.d/ros2.list

# Add the repository if missing
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package lists
sudo apt update

# Verify available ROS 2 packages
apt-cache search ros-humble
```

### Issue 3: Dependencies Not Satisfied

**Symptoms:**
- `E: Unable to correct problems, you have held broken packages`
- `E: Some packages could not be installed`

**Cause:** Missing or conflicting dependencies.

**Solution:**
```bash
# Update system packages
sudo apt update && sudo apt upgrade

# Try installing with --fix-missing
sudo apt install -f

# Install base dependencies first
sudo apt install software-properties-common

# Then install ROS 2
sudo apt install ros-humble-desktop
```

## Environment Issues

### Issue 4: ROS 2 Command Not Found

**Symptoms:**
- `ros2: command not found`
- `rclpy: command not found`

**Cause:** ROS 2 environment not sourced or installation incomplete.

**Solution:**
```bash
# Verify ROS 2 is installed
ls /opt/ros/humble/

# Source the environment manually
source /opt/ros/humble/setup.bash

# Add to bashrc permanently
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --version
```

### Issue 5: Python Module Import Errors

**Symptoms:**
- `ModuleNotFoundError: No module named 'rclpy'`
- `ImportError: No module named 'rosgraph'`

**Cause:** Python environment not properly configured or modules not in path.

**Solution:**
```bash
# Check if modules are accessible
python3 -c "import rclpy; print('rclpy imported successfully')"

# Ensure environment is sourced
source /opt/ros/humble/setup.bash

# Check Python path
python3 -c "import sys; print('\n'.join(sys.path))"

# Install Python dependencies if missing
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool
```

### Issue 6: Workspace Not Sourcing Properly

**Symptoms:**
- Custom packages not found after building
- Workspace executables not accessible

**Cause:** Workspace not properly built or sourced after building.

**Solution:**
```bash
# Navigate to workspace
cd ~/ros2_ws

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash

# Add to bashrc permanently
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Network and Communication Issues

### Issue 7: Nodes Cannot Communicate

**Symptoms:**
- Publisher and subscriber nodes cannot see each other
- Topics not appearing in `ros2 topic list`

**Cause:** Network configuration issues or domain ID conflicts.

**Solution:**
```bash
# Check current domain ID
echo $ROS_DOMAIN_ID

# Set to default (0) if not set
export ROS_DOMAIN_ID=0

# Check if firewall is blocking communication
sudo ufw status

# For local testing, temporarily allow all communication
export ROS_LOCALHOST_ONLY=1

# Verify network interfaces
ip addr show
```

### Issue 8: DDS/RMW Implementation Issues

**Symptoms:**
- `Failed to create participant` errors
- Communication timeouts
- Discovery failures

**Cause:** RMW implementation conflicts or DDS configuration issues.

**Solution:**
```bash
# Check available RMW implementations
printenv | grep RMW

# Set to default implementation
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# Or try alternatives if issues persist
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Restart any running ROS 2 processes after changing
```

## Build and Compilation Issues

### Issue 9: C++ Build Failures

**Symptoms:**
- `colcon build` fails with compilation errors
- Missing CMake packages
- Linker errors

**Cause:** Missing build dependencies or incorrect package configuration.

**Solution:**
```bash
# Install build dependencies
sudo apt install build-essential cmake python3-colcon-common-extensions

# Clean previous build
rm -rf build/ install/ log/

# Run rosdep to install missing dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Rebuild
colcon build
```

### Issue 10: Package.xml or CMakeLists.txt Issues

**Symptoms:**
- `ament_package()` not found
- Package dependencies not resolved
- Build system errors

**Cause:** Incorrect package manifest or CMake configuration.

**Solution:**
```bash
# Verify package.xml format
cat src/your_package/package.xml

# Ensure proper format (format="3")
# Verify all dependencies are listed

# Check CMakeLists.txt
cat src/your_package/CMakeLists.txt

# Ensure find_package() calls match dependencies
```

## Performance and Resource Issues

### Issue 11: High Memory Usage

**Symptoms:**
- System becomes slow during ROS 2 operations
- Out of memory errors
- Nodes crashing due to resource limits

**Solution:**
```bash
# Check current memory usage
free -h

# Set memory limits for ROS 2 processes
export RMW_CONNEXT_INITIAL_PEERS="239.255.0.1"

# Monitor resource usage
htop
```

### Issue 12: Slow Topic Communication

**Symptoms:**
- High latency in message passing
- Messages arriving out of order
- Communication delays

**Solution:**
```bash
# Check network status
ros2 topic hz /your_topic

# Verify QoS settings
ros2 topic info /your_topic -v

# Optimize for performance
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
```

## Development Environment Issues

### Issue 13: IDE Integration Problems

**Symptoms:**
- IDE cannot find ROS 2 packages
- Code completion not working
- Import errors in development environment

**Solution:**
```bash
# Source ROS 2 environment in IDE terminal
source /opt/ros/humble/setup.bash

# For VS Code, ensure extensions are installed
code --list-extensions | grep ros

# Set Python interpreter to system Python with ROS 2 sourced
```

### Issue 14: Permission Issues

**Symptoms:**
- Cannot create or modify files in ROS workspace
- Permission denied errors during build

**Solution:**
```bash
# Check workspace ownership
ls -la ~/ros2_ws

# Fix permissions if needed
sudo chown -R $USER:$USER ~/ros2_ws

# Add user to dialout group for device access
sudo usermod -a -G dialout $USER
```

## System-Specific Issues

### Issue 15: Locale Configuration Issues

**Symptoms:**
- Unicode/encoding errors during installation
- Character encoding problems
- Installation scripts failing

**Solution:**
```bash
# Check current locale
locale

# Set proper locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Export in bashrc
echo 'export LANG=en_US.UTF-8' >> ~/.bashrc
source ~/.bashrc
```

### Issue 16: Graphics/OpenGL Issues

**Symptoms:**
- RViz2 or Gazebo failing to start
- Rendering errors
- GUI tools not displaying properly

**Solution:**
```bash
# Install graphics libraries
sudo apt install mesa-utils libgl1-mesa-glx libgl1-mesa-dri

# Check OpenGL support
glxinfo | grep "OpenGL version"

# For virtual machines, ensure 3D acceleration is enabled
```

## Verification and Testing

### Issue 17: Installation Verification Fails

**Symptoms:**
- Basic ROS 2 commands don't work
- Demo nodes fail to run
- Environment appears broken

**Solution:**
```bash
# Complete verification checklist:

# 1. Check ROS 2 installation
ros2 --version

# 2. Check available packages
ros2 pkg list | head -10

# 3. Test basic commands
ros2 topic list
ros2 node list

# 4. Run a simple demo
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_py listener

# 5. Check environment variables
env | grep -i ros
```

## Recovery Procedures

### Complete Reinstallation

If multiple issues persist, consider a complete reinstallation:

```bash
# Remove ROS 2 packages
sudo apt remove ros-humble-*
sudo apt autoremove

# Remove ROS 2 repository
sudo rm /etc/apt/sources.list.d/ros2.list

# Remove GPG key
sudo rm /usr/share/keyrings/ros-archive-keyring.gpg

# Clean up environment in bashrc
sed -i '/ros/d' ~/.bashrc

# Reinstall following the official guide
# (Re-run the installation steps from the beginning)
```

## Prevention Tips

1. **Always source the environment** before running ROS 2 commands
2. **Keep system updated** with `sudo apt update && sudo apt upgrade`
3. **Use proper workspace structure** with `~/ros2_ws/src`
4. **Check for conflicts** between ROS 1 and ROS 2 installations
5. **Monitor disk space** during builds and simulations
6. **Use version control** for your custom packages
7. **Document your environment** settings for troubleshooting

## Getting Additional Help

If issues persist after trying these solutions:

1. **Check the ROS 2 documentation:** https://docs.ros.org/en/humble/
2. **Search ROS Answers:** https://answers.ros.org/questions/
3. **Join the ROS community:** https://discourse.ros.org/
4. **Check system logs:** `journalctl -xe` for system-level issues

## Quick Reference Commands

```bash
# Environment check
source /opt/ros/humble/setup.bash && ros2 --version

# Clean build
rm -rf build/ install/ log/ && colcon build

# Network check
echo $ROS_DOMAIN_ID && ros2 topic list

# Process check
ps aux | grep ros

# Log check
tail -f ~/.ros/log/latest/*.log
```

---

**Note:** This troubleshooting guide covers the most common issues. For environment-specific problems or unique configurations, consult the official ROS 2 documentation or community forums.