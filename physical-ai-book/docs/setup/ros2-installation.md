# ROS 2 Humble Hawksbill Installation Guide for Ubuntu 22.04

## Overview

This guide provides step-by-step instructions for installing ROS 2 Humble Hawksbill on Ubuntu 22.04 LTS with all necessary development tools. ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software, and Humble Hawksbill is the LTS (Long Term Support) version recommended for production and educational use.

## Prerequisites

- Ubuntu 22.04 LTS (Jammy Jellyfish)
- Internet connection
- Administrative (sudo) access
- At least 4GB of RAM recommended
- 20GB of free disk space

## System Preparation

### Update System Packages

First, ensure your system is up to date:

```bash
sudo apt update && sudo apt upgrade -y
```

### Set Locale

Make sure your locale is set to UTF-8:

```bash
locale  # Check current locale
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

## ROS 2 Repository Setup

### Add the ROS 2 GPG Key

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### Add the ROS 2 Repository

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## ROS 2 Installation

### Install ROS 2 Humble Desktop

The desktop package includes ROS, common tools, and GUI libraries:

```bash
sudo apt update
sudo apt install -y ros-humble-desktop
```

### Install Additional Development Tools

```bash
sudo apt install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-vcstool \
    wget \
    curl \
    gnupg \
    lsb-release \
    zip \
    unzip
```

### Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

## Environment Setup

### Source ROS 2 Environment

Add ROS 2 to your bash environment:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Verify Installation

Test that ROS 2 is installed correctly:

```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

You should see output similar to: `ros2 foxy YYYY.MM.DD` (where the version number reflects your installed version).

## Development Tools Installation

### Python Development Tools

```bash
pip3 install --user -U \
    argcomplete \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest \
    pytest-cov \
    pytest-repeat \
    pytest-rerunfailures
```

### C++ Development Tools

```bash
sudo apt install -y \
    clang-format \
    cppcheck \
    valgrind \
    gdb
```

### Additional ROS Tools

```bash
sudo apt install -y \
    ros-dev-tools \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-robot-plugins
```

## Create a ROS 2 Workspace

### Create Workspace Directory

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### Build the Workspace

```bash
colcon build --symlink-install
```

### Source the Workspace

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/ros2_ws/install/setup.bash
```

## Verification and Testing

### Test with Demo Nodes

```bash
# Terminal 1: Run a demo publisher
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker

# Terminal 2: Run a demo subscriber
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

You should see messages being published by the talker and received by the listener.

### Check Available Packages

```bash
ros2 pkg list
```

## Development Environment Setup

### Install IDE Support

For VS Code with ROS 2 support:

```bash
# Install VS Code extensions
code --install-extension ms-vscode.cpptools
code --install-extension twxs.cmake
code --install-extension ms-python.python
```

### Setup ROS 2 Environment Variables

Add to your `~/.bashrc`:

```bash
# ROS 2 Environment Variables
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=INFO
```

## Troubleshooting

### Common Issues

1. **Package not found errors**: Ensure your locale is set correctly and run:
   ```bash
   sudo apt update
   rosdep update
   ```

2. **Permission errors**: Make sure you're using the correct user account and have sudo access.

3. **ROS environment not found**: Verify the setup.bash file is sourced:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

### Useful Commands

```bash
# Check ROS 2 environment
printenv | grep -i ros

# List all ROS 2 topics
ros2 topic list

# List all ROS 2 services
ros2 service list

# Check ROS 2 version
ros2 --version
```

## Next Steps

After completing this installation:

1. Follow the ROS 2 tutorials to learn the basics
2. Set up your first ROS 2 package
3. Explore the example packages in `/opt/ros/humble/share/`
4. Begin working with the Physical AI & Humanoid Robotics examples

## Additional Resources

- [Official ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS Index](https://index.ros.org/)
- [ROS Answers](https://answers.ros.org/questions/)

## Maintenance

### Updating ROS 2

```bash
sudo apt update
sudo apt upgrade
```

### Checking for Updates

```bash
apt list --upgradable | grep ros
```

---

**Note**: This installation guide is specifically for Ubuntu 22.04 LTS and ROS 2 Humble Hawksbill. Using different versions may result in compatibility issues.