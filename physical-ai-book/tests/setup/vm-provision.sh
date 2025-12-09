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