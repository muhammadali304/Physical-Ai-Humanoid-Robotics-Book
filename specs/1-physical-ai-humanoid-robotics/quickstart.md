# Quick Start Guide: Physical AI & Humanoid Robotics

## Overview
This quick start guide will help you set up the basic environment to begin learning about Physical AI and Humanoid Robotics. The full curriculum spans 13 weeks with hands-on examples using ROS 2, NVIDIA Isaac, and Gazebo simulation.

## Prerequisites
- Ubuntu 22.04 LTS (recommended) or Ubuntu 20.04 LTS
- At least 8GB RAM (16GB recommended)
- 50GB free disk space
- Internet connection for package downloads
- Basic knowledge of Python and Linux command line

## Step 1: System Preparation
1. Update your system packages:
   ```bash
   sudo apt update && sudo apt upgrade -y
   ```

2. Install basic dependencies:
   ```bash
   sudo apt install python3-pip python3-colcon-common-extensions python3-rosdep python3-vcstool wget
   ```

3. Set up locale (required for ROS 2):
   ```bash
   locale  # check if LANG=en_US.UTF-8 is set
   sudo locale-gen en_US.UTF-8
   ```

## Step 2: Install ROS 2 Humble Hawksbill
1. Add the ROS 2 repository:
   ```bash
   sudo apt update && sudo apt install -y software-properties-common
   sudo add-apt-repository universe
   sudo apt update
   sudo apt install curl -y
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

2. Install ROS 2 packages:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop ros-humble-ros-base -y
   sudo apt install ros-dev-tools -y
   ```

3. Source the ROS 2 environment:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

## Step 3: Install Gazebo (Fortress)
1. Add the Gazebo repository:
   ```bash
   wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-gazebo-archive-keyring.gpg
   echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
   ```

2. Install Gazebo Fortress:
   ```bash
   sudo apt update
   sudo apt install gz-fortress -y
   ```

## Step 4: Set Up Workspace
1. Create a ROS 2 workspace:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   ```

2. Source ROS 2 and build the workspace:
   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   ```

3. Source the workspace:
   ```bash
   echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

## Step 5: Test Installation
1. Open a new terminal and run:
   ```bash
   ros2 topic list
   ```

2. Test Gazebo:
   ```bash
   gz sim
   ```

3. Run a basic ROS 2 demo:
   ```bash
   ros2 run demo_nodes_cpp talker
   ```
   In another terminal:
   ```bash
   ros2 run demo_nodes_cpp listener
   ```

## Next Steps
- Continue with Chapter 2: Ubuntu Installation and ROS 2 Setup
- Explore Chapter 3: ROS 2 Fundamentals (Nodes, Topics, Services)
- Try the first simulation example in Chapter 6: Gazebo Basics

## Troubleshooting
- If ROS 2 commands are not found, ensure you've sourced the setup.bash files
- If Gazebo doesn't start, check graphics drivers and X11 forwarding if using SSH
- For package installation issues, verify your Ubuntu version and internet connection

## Hardware Options
- **Simulation Only**: Follow this guide to work entirely in simulation
- **Budget Tier**: Consider TurtleBot 4 for hands-on experience (~$700)
- **Standard Tier**: Quadruped robot for advanced projects (~$3K)
- **Premium Tier**: Full humanoid robot for complete experience (~$16K+)

For the complete learning experience, continue with the full curriculum in the main documentation.