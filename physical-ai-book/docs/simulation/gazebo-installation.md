# Gazebo Fortress Simulation Environment Installation Guide

## Overview

This guide provides comprehensive instructions for installing Gazebo Fortress, a physics-based simulation environment for robotics development. Gazebo Fortress is part of the Ignition Robotics suite and provides realistic physics simulation, sensor models, and rendering capabilities essential for robotics research and development.

## Prerequisites

- Ubuntu 22.04 LTS (Jammy Jellyfish)
- ROS 2 Humble Hawksbill (installed as per previous guide)
- At least 4GB of RAM recommended
- 10GB of free disk space
- Graphics card with OpenGL 2.1+ support
- Internet connection

## System Preparation

### Update System Packages

```bash
sudo apt update && sudo apt upgrade -y
```

### Install Graphics Drivers (if needed)

For NVIDIA graphics:
```bash
sudo apt install nvidia-driver-470
# Reboot after installation
sudo reboot
```

For Intel graphics (usually already installed):
```bash
sudo apt install intel-media-va-driver mesa-vulkan-drivers
```

## Gazebo Fortress Installation

### Add the Gazebo Repository

```bash
# Add the Gazebo repository
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

# Add the Gazebo signing key
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

### Update Package Lists

```bash
sudo apt update
```

### Install Gazebo Fortress

```bash
sudo apt install gazebo11 libgazebo11-dev
```

**Note**: Gazebo Fortress corresponds to Gazebo version 11. The Ignition Fortress distribution has been superseded by Gazebo Garden and newer versions, but Gazebo 11 remains stable and widely used.

## ROS 2 Gazebo Integration

### Install ROS 2 Gazebo Packages

```bash
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-dev \
    ros-humble-gazebo-msgs \
    ros-humble-gazebo-ros
```

### Install Additional Dependencies

```bash
sudo apt install -y \
    libignition-math6-dev \
    libsdformat13-dev \
    libignition-transport8-dev \
    libignition-fuel-tools8-dev \
    libignition-common4-dev \
    libignition-gazebo6-dev
```

## Environment Setup

### Add Gazebo to Environment

Add the following to your `~/.bashrc`:

```bash
# Gazebo Environment Variables
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/.gazebo/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/.gazebo/worlds
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/.gazebo/plugins
```

### Source the Environment

```bash
source ~/.bashrc
```

## Verification and Testing

### Test Gazebo Installation

```bash
gazebo --version
```

You should see the Gazebo version information.

### Launch Gazebo GUI

```bash
gazebo
```

This should open the Gazebo simulation environment with a default world.

### Test with ROS 2 Integration

In a new terminal:

```bash
# Source ROS 2 and Gazebo
source /opt/ros/humble/setup.bash
source ~/.bashrc

# Launch a simple example
ros2 launch gazebo_ros empty_world.launch.py
```

## Gazebo Model Database Setup

### Download Common Models

```bash
# Create models directory if it doesn't exist
mkdir -p ~/.gazebo/models

# Clone the fuel models repository (optional, for additional models)
# Note: This will download many models, so only do this if you have sufficient space
git clone https://github.com/osrf/gazebo_models.git ~/gazebo_models
cp -r ~/gazebo_models/* ~/.gazebo/models/
```

### Install Pre-built Models

```bash
sudo apt install -y \
    gazebo11-models \
    gazebo11-common
```

## Development Tools and Plugins

### Install Gazebo Development Tools

```bash
sudo apt install -y \
    libgazebo11-dev \
    gazebo11-dev \
    gazebo11-plugin-base
```

### Install Additional Sensors and Plugins

```bash
sudo apt install -y \
    ros-humble-velodyne-simulator \
    ros-humble-ackermann-msgs \
    ros-humble-geometry2 \
    ros-humble-tf2-tools \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher
```

## Configuration and Optimization

### Gazebo Configuration File

Create a custom Gazebo configuration file:

```bash
mkdir -p ~/.gazebo
cat > ~/.gazebo/config << EOF
<gazebo>
  <cache_location>~/.gazebo/cache</cache_location>
  <http_proxy></http_proxy>
  <https_proxy></https_proxy>
  <max_cache_size>1024</max_cache_size>
  <default_gui_configurations>
    <default_gui_configuration>
      <name>default</name>
      <window>
        <width>800</width>
        <height>600</height>
        <x>0</x>
        <y>0</y>
      </window>
    </default_gui_configuration>
  </default_gui_configurations>
</gazebo>
EOF
```

### Performance Optimization

Add to your `~/.bashrc` for better performance:

```bash
# Gazebo Performance Settings
export OGRE_RESOURCE_PATH=/usr/lib/x86_64-linux-gnu/OGRE-1.9.0
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
export GAZEBO_MASTER_URI=http://localhost:11345
export GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org
```

## Troubleshooting

### Common Issues

1. **Graphics Errors**: If you encounter OpenGL errors:
   ```bash
   # Check OpenGL version
   glxinfo | grep "OpenGL version"

   # Install mesa utilities
   sudo apt install mesa-utils
   ```

2. **Gazebo Won't Start**: Check for missing dependencies:
   ```bash
   ldd $(which gazebo) | grep "not found"
   ```

3. **ROS 2 Integration Issues**: Ensure both ROS 2 and Gazebo are properly sourced:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/.bashrc
   ```

### Useful Commands

```bash
# Check Gazebo processes
ps aux | grep gazebo

# Kill all Gazebo processes
pkill -f gazebo

# Check Gazebo environment
printenv | grep -i gazebo

# Run Gazebo with verbose output
gazebo --verbose
```

## Testing with Sample Robot

### Download a Test Robot

```bash
# Create a workspace for testing
mkdir -p ~/gazebo_test/src
cd ~/gazebo_test

# Create a simple launch file to test
mkdir -p launch
cat > launch/test_gazebo.launch.py << EOF
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
    ])
EOF
```

### Run the Test

```bash
source /opt/ros/humble/setup.bash
cd ~/gazebo_test
python3 launch/test_gazebo.launch.py
```

## Integration with ROS 2

### Verify ROS 2 Integration

```bash
# In one terminal
gazebo

# In another terminal
source /opt/ros/humble/setup.bash
ros2 topic list
```

You should see ROS 2 topics being published by Gazebo.

### Test with a Simple Robot Model

```bash
# Launch empty world with ROS 2 bridge
ros2 launch gazebo_ros empty_world.launch.py
```

## Next Steps

After completing this installation:

1. Explore Gazebo tutorials and examples
2. Create your first robot model for simulation
3. Integrate with ROS 2 nodes for control
4. Begin working with the Physical AI & Humanoid Robotics simulation examples

## Additional Resources

- [Gazebo Classic Documentation](http://classic.gazebosim.org/)
- [ROS 2 Gazebo Integration](https://github.com/ros-simulation/gazebo_ros_pkgs)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [Model Database](http://models.gazebosim.org/)

## Maintenance

### Updating Gazebo

```bash
sudo apt update
sudo apt upgrade
```

### Checking for Updates

```bash
apt list --upgradable | grep gazebo
```

---

**Note**: Gazebo Fortress (Gazebo 11) is the last version in the classic Gazebo series. For newer features, consider Gazebo Garden or Ignition Edifice, but Fortress remains stable and widely used in ROS 2 Humble environments.