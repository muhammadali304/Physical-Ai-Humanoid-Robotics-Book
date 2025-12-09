# Isaac ROS Perception Packages Setup Guide

## Overview

This guide provides instructions for installing and configuring Isaac ROS perception packages. Isaac ROS is NVIDIA's collection of hardware-accelerated perception and navigation packages designed to run on Jetson platforms and x86 systems with NVIDIA GPUs.

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- NVIDIA GPU with CUDA support (for x86) or Jetson platform
- CUDA 11.8 or later
- cuDNN 8.6 or later
- Sufficient disk space (5-10 GB recommended)

## System Requirements

### Hardware Requirements
- **GPU**: NVIDIA GPU with compute capability 6.0 or higher
  - For development: GTX 1060 or better
  - For production: Jetson AGX Orin, Jetson Orin NX, or equivalent
- **RAM**: 8GB minimum, 16GB recommended
- **Storage**: 50GB free space for complete installation

### Software Requirements
- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill (binary or source installation)
- **CUDA**: 11.8 or later
- **cuDNN**: 8.6 or later
- **NVIDIA Driver**: 520 or later

## Installation Methods

### Method 1: Binary Installation (Recommended for beginners)

#### 1. Add Isaac ROS Package Repository

```bash
# Add the Isaac ROS package repository
sudo apt update && sudo apt install wget
sudo wget -O /usr/share/keyrings/nvidia-isaacl-ros.gpg https://repo.download.nvidia.com/nvidia-isaacl-ros.gpg

# Add the repository to your sources list
echo "deb [signed-by=/usr/share/keyrings/nvidia-isaacl-ros.gpg] https://repo.download.nvidia.com/ all main" | sudo tee /etc/apt/sources.list.d/nvidia-isaacl-ros.list

# Update package list
sudo apt update
```

#### 2. Install Isaac ROS Perception Metapackage

```bash
# Install the complete Isaac ROS perception suite
sudo apt install ros-humble-isaac-ros-perception

# Or install specific packages individually:
sudo apt install ros-humble-isaac-ros-visual-slam
sudo apt install ros-humble-isaac-ros-apriltag
sudo apt install ros-humble-isaac-ros-occupancy-grid-localizer
sudo apt install ros-humble-isaac-ros-stereo-image-proc
sudo apt install ros-humble-isaac-ros-dnn-segmentation
sudo apt install ros-humble-isaac-ros-deep-learning-ligament
```

#### 3. Install Additional Dependencies

```bash
# Install additional dependencies
sudo apt install ros-humble-isaac-ros-common
sudo apt install ros-humble-isaac-ros-gxf
sudo apt install libgxf.so  # Ensure GXF libraries are available
```

### Method 2: Source Installation (Recommended for advanced users)

#### 1. Set up ROS 2 Workspace

```bash
# Create a new workspace for Isaac ROS
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash
```

#### 2. Clone Isaac ROS Perception Repositories

```bash
cd ~/isaac_ros_ws/src

# Clone the Isaac ROS perception repository
git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_perception.git

# Clone dependencies
git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_gxf.git
git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git
git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_stereo_image_proc.git
git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_segmentation.git
```

#### 3. Install Dependencies

```bash
cd ~/isaac_ros_ws

# Install dependencies using rosdep
rosdep install --from-paths src --ignore-src -r -y

# Install additional system dependencies
sudo apt update
sudo apt install libgxf.so libgxf_utils.so libgxf_extensions.so
```

#### 4. Build the Workspace

```bash
cd ~/isaac_ros_ws

# Build with colcon
colcon build --symlink-install --packages-select \
  isaac_ros_common \
  isaac_ros_gxf \
  isaac_ros_visual_slam \
  isaac_ros_apriltag \
  isaac_ros_stereo_image_proc \
  isaac_ros_dnn_segmentation

# Source the workspace
source install/setup.bash
```

## Verification Installation

### 1. Check Installed Packages

```bash
# List Isaac ROS packages
ros2 pkg list | grep isaac

# Check specific package information
ros2 pkg info isaac_ros_visual_slam
```

### 2. Verify GPU Access

```bash
# Check NVIDIA GPU status
nvidia-smi

# Check CUDA installation
nvcc --version

# Test GPU access from ROS
nvidia-ml-py3 # if installed
```

### 3. Run a Simple Test

```bash
# Source your ROS environment
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash  # if built from source

# Check if Isaac ROS nodes are available
ros2 run --list | grep isaac
```

## Configuration

### 1. Environment Setup

Add the following to your `~/.bashrc` file:

```bash
# Isaac ROS Environment Variables
export ISAAC_ROS_WS=~/isaac_ros_ws
export CUDA_DEVICE_ORDER=PCI_BUS_ID
export CUDA_VISIBLE_DEVICES=0

# Performance settings
export NVIDIA_TEGRA_MAINTENANCE_MODE=0
```

### 2. GPU Memory Configuration

For Jetson platforms, ensure proper memory allocation:

```bash
# Check current memory settings
cat /sys/kernel/debug/memblock/memory10/size

# For Jetson Orin, ensure sufficient memory for perception tasks
# This is typically handled by the system, but can be verified
```

### 3. Container Runtime (Optional)

If using Docker containers with Isaac ROS:

```bash
# Install NVIDIA Container Toolkit
sudo apt install nvidia-container-toolkit

# Configure Docker
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

## Common Isaac ROS Perception Packages

### 1. Visual SLAM
- **Package**: `isaac_ros_visual_slam`
- **Purpose**: Simultaneous Localization and Mapping using visual input
- **Nodes**: `visual_slam_node`
- **Topics**: `/visual_slam/odometry`, `/visual_slam/mapped_points`

### 2. AprilTag Detection
- **Package**: `isaac_ros_apriltag`
- **Purpose**: Detect AprilTag fiducial markers
- **Nodes**: `apriltag_node`
- **Topics**: `/apriltag/detections`, `/tf`

### 3. Stereo Image Processing
- **Package**: `isaac_ros_stereo_image_proc`
- **Purpose**: Generate depth maps from stereo images
- **Nodes**: `stereo_image_proc`
- **Topics**: `/ stereo/depth/disparity`, `/stereo/depth/points`

### 4. DNN Segmentation
- **Package**: `isaac_ros_dnn_segmentation`
- **Purpose**: Semantic segmentation using deep neural networks
- **Nodes**: `dnn_segmentation_node`
- **Topics**: `/segmentation/segmentation_map`

## Troubleshooting Common Issues

### Issue: CUDA/GPU Not Detected
**Symptoms**: Isaac ROS packages fail to initialize with GPU errors
**Solutions**:
1. Verify NVIDIA driver installation:
   ```bash
   nvidia-smi
   ```
2. Check CUDA installation:
   ```bash
   nvcc --version
   ```
3. Verify GPU access:
   ```bash
   sudo usermod -a -G video $USER
   # Log out and back in
   ```

### Issue: Missing Dependencies
**Symptoms**: Build fails with missing package errors
**Solutions**:
1. Update package lists:
   ```bash
   sudo apt update
   rosdep update
   ```
2. Install missing dependencies manually:
   ```bash
   sudo apt install ros-humble-<missing-package>
   ```

### Issue: Permission Errors
**Symptoms**: Permission denied when accessing GPU or running nodes
**Solutions**:
1. Add user to required groups:
   ```bash
   sudo usermod -a -G video,dialout $USER
   ```
2. Log out and log back in

### Issue: Memory Errors
**Symptoms**: Isaac ROS nodes crash with memory errors
**Solutions**:
1. Check available memory:
   ```bash
   free -h
   nvidia-smi
   ```
2. Reduce input resolution or processing parameters
3. Close unnecessary applications to free memory

## Performance Optimization

### 1. GPU Utilization
Monitor GPU usage during Isaac ROS operations:
```bash
watch -n 1 nvidia-smi
```

### 2. Memory Management
- Use appropriate input resolutions for your hardware
- Monitor GPU memory usage and adjust accordingly
- Consider using TensorRT optimization for neural networks

### 3. Pipeline Optimization
- Use appropriate queue sizes for topic subscriptions
- Consider using intra-process communication where possible
- Profile your pipeline to identify bottlenecks

## Next Steps

After successfully installing Isaac ROS perception packages:

1. **Test individual packages**: Run simple examples for each package
2. **Integrate with existing robot**: Connect Isaac ROS nodes to your robot's sensors
3. **Calibrate sensors**: Ensure proper camera and sensor calibration
4. **Build perception pipeline**: Combine multiple perception nodes into a complete pipeline

## Resources

- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
- [Isaac ROS GitHub Repository](https://github.com/NVIDIA-ISAAC-ROS)
- [NVIDIA Developer Zone](https://developer.nvidia.com/ros)
- [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)

## Conclusion

This guide provides the foundation for installing and configuring Isaac ROS perception packages. Proper installation and configuration are critical for leveraging NVIDIA's hardware-accelerated perception capabilities. Once installed, you can begin integrating these powerful perception tools into your robotics applications, taking advantage of GPU acceleration for real-time performance.