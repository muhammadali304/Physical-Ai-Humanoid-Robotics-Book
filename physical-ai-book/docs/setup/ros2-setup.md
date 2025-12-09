---
sidebar_position: 2
---

# ROS 2 Humble Hawksbill Installation and Setup

## Overview

This guide will help you install ROS 2 Humble Hawksbill, the LTS (Long Term Support) version that will be used throughout this curriculum. ROS 2 Humble Hawksbill is supported until 2027, providing stability for your robotics development.

## Prerequisites

Before installing ROS 2, ensure you have:
- Ubuntu 22.04 LTS installed
- Administrative (sudo) access
- Internet connection
- Completed the Ubuntu installation and system preparation chapter

## Installation Steps

### Step 1: Set Up Your Sources

First, add the ROS 2 apt repository to your system:

```bash
# Add the ROS 2 GPG key
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Add the ROS 2 repository
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 2: Install ROS 2 Packages

Update your package list and install the ROS 2 desktop package:

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-ros-base -y
sudo apt install ros-dev-tools -y
```

The `ros-humble-desktop` package includes the full ROS 2 desktop environment with GUI tools, while `ros-humble-ros-base` provides only the core ROS 2 packages without GUI tools.

### Step 3: Environment Setup

Add the ROS 2 environment setup to your bash configuration:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

This ensures that ROS 2 commands are available in every new terminal session.

### Step 4: Verify Installation

Test that ROS 2 is properly installed:

```bash
# Check ROS 2 version
ros2 --version

# List available ROS 2 commands
ros2

# Check available topics (should show an empty list)
ros2 topic list
```

## Additional Setup for Development

### Install Python Development Tools

```bash
pip3 install -U argcomplete
```

### Install Additional ROS 2 Tools

```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y

# Initialize rosdep
sudo rosdep init
rosdep update
```

### Set Up a ROS 2 Workspace

Create a workspace for your ROS 2 packages:

```bash
# Create the workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Source the workspace
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Testing Your Installation

### Basic Test

Run a simple ROS 2 publisher/subscriber test:

1. Open a new terminal and run the talker:
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

2. In another terminal, run the listener:
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp listener
```

You should see messages being published by the talker and received by the listener.

### Check Available Packages

```bash
# List all installed ROS 2 packages
ros2 pkg list

# Check specific package information
ros2 pkg list | grep std_msgs
```

## Common Issues and Solutions

### Issue: Command 'ros2' not found
**Solution**:
- Verify you sourced the setup.bash file: `source /opt/ros/humble/setup.bash`
- Check your ~/.bashrc file has the source command
- Try opening a new terminal or sourcing your bashrc: `source ~/.bashrc`

### Issue: Permission denied when running ROS 2 commands
**Solution**:
- Make sure you're not using sudo with ROS 2 commands
- Verify your user has proper permissions
- Check that the ROS 2 installation directory has correct permissions

### Issue: Package installation fails
**Solution**:
- Update package lists: `sudo apt update`
- Check your internet connection
- Verify the ROS 2 repository is correctly added to your sources

### Issue: Workspace build fails
**Solution**:
- Ensure all dependencies are installed
- Check for proper directory structure
- Verify you're in the correct workspace directory

## Troubleshooting Commands

### Check ROS 2 Environment Variables
```bash
printenv | grep -i ros
```

### Check Installation Path
```bash
ls -la /opt/ros/humble/
```

### Verify Python Package Installation
```bash
python3 -c "import rclpy; print('rclpy imported successfully')"
```

## Performance Considerations

### Resource Usage
- ROS 2 core processes typically use minimal resources
- GUI tools (RViz2, rqt) require more memory and CPU
- Running multiple nodes simultaneously increases resource usage

### Network Configuration
By default, ROS 2 uses DDS (Data Distribution Service) for communication. For single-machine development:
- No special network configuration is needed
- All nodes on the same machine will automatically discover each other

## Next Steps

After successfully installing and testing ROS 2 Humble Hawksbill, you're ready to dive into ROS 2 fundamentals. The next chapters will cover nodes, topics, services, and packages in detail.

## Key Takeaways

- ROS 2 Humble Hawksbill is an LTS release with support until 2027
- The desktop package includes GUI tools, while ros-base is minimal
- Proper environment setup is crucial for ROS 2 to function correctly
- Workspaces are essential for organizing your ROS 2 packages
- Testing with basic publisher/subscriber examples validates the installation

Continue to the next chapter to learn about ROS 2 nodes, topics, and fundamental concepts.