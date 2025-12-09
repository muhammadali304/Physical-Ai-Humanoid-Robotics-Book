---
sidebar_position: 1
---

# Ubuntu Installation and System Preparation

## Overview

This guide will help you set up Ubuntu 22.04 LTS for the Physical AI & Humanoid Robotics curriculum. Ubuntu 22.04 LTS is the primary target platform for all examples and exercises in this book.

## Prerequisites

- A computer with at least 8GB RAM (16GB recommended)
- 50GB free disk space
- Internet connection
- Administrative access to install software

## System Requirements

### Minimum Requirements
- CPU: Dual-core 2GHz or better
- RAM: 8GB
- Disk: 50GB free space
- Graphics: Standard display adapter

### Recommended Requirements
- CPU: Quad-core 3GHz or better
- RAM: 16GB or more
- Disk: 100GB SSD or more
- Graphics: Dedicated GPU recommended for simulation

## Installation Options

### Option 1: Native Installation (Recommended for serious development)
- Provides the best performance for simulation and robotics development
- Direct access to hardware features
- Best compatibility with ROS 2 and simulation tools

### Option 2: Virtual Machine (Good for evaluation)
- Safest option for trying out the system
- Easy to revert changes
- Slightly reduced performance for graphics-intensive tasks

### Option 3: Dual Boot (Best of both worlds)
- Native performance when needed
- Ability to run other operating systems
- Requires more advanced setup

## Ubuntu Installation Process

### For Native Installation:

1. **Download Ubuntu 22.04 LTS**
   - Visit https://releases.ubuntu.com/jammy/
   - Download the 64-bit PC (AMD64) desktop image
   - Verify the checksum for security (optional but recommended)

2. **Create Bootable USB Drive**
   - Use Rufus (Windows), Etcher (multi-platform), or Startup Disk Creator (Ubuntu)
   - Use a USB drive with at least 4GB capacity
   - Follow the tool-specific instructions to create the bootable drive

3. **Boot from USB**
   - Restart your computer
   - Enter BIOS/UEFI settings (usually F2, F12, Del, or Esc during startup)
   - Set USB drive as the first boot device
   - Save settings and exit

4. **Install Ubuntu**
   - Select "Install Ubuntu 22.04 LTS"
   - Choose your language
   - Select "Keyboard layout" (usually detected automatically)
   - Connect to WiFi if needed
   - For updates: Choose "Normal installation" or "Minimal installation"
   - For installation type:
     - If using entire disk: Select "Erase disk and install Ubuntu"
     - If dual-booting: Select "Install Ubuntu alongside Windows/macOS"
   - Select your timezone
   - Enter user information (username, password, etc.)

### For Virtual Machine Installation:

1. **Install Virtualization Software**
   - VirtualBox (free): https://www.virtualbox.org/
   - VMware Workstation Player (free for personal use): https://www.vmware.com/products/workstation-player.html
   - Or similar virtualization software

2. **Create Virtual Machine**
   - Allocate at least 4GB RAM (8GB+ recommended)
   - Allocate at least 40GB disk space (50GB+ recommended)
   - Enable hardware virtualization features if available
   - Enable 3D acceleration for better graphics performance

3. **Install Ubuntu**
   - Mount the Ubuntu ISO file as the virtual CD/DVD
   - Follow the same installation steps as above

## Post-Installation Configuration

### Update System Packages
After installation, update your system:

```bash
sudo apt update && sudo apt upgrade -y
```

### Install Essential Development Tools
```bash
sudo apt install build-essential cmake git curl wget vim htop python3-pip python3-dev python3-venv
```

### Configure Locale Settings
Ensure proper locale settings for ROS 2:

```bash
locale
# Check if LANG=en_US.UTF-8 is set
sudo locale-gen en_US.UTF-8
```

### Install Graphics Drivers (if needed)
For better simulation performance with 3D graphics:

```bash
# For NVIDIA GPUs
sudo apt install nvidia-driver-535  # Or latest available version

# For AMD GPUs
sudo apt install mesa-vulkan-drivers xserver-xorg-video-amdgpu

# For Intel integrated graphics
sudo apt install intel-media-va-driver mesa-vulkan-drivers
```

## Verification Steps

### Check Ubuntu Version
```bash
lsb_release -a
```
Expected output should show Ubuntu 22.04 LTS.

### Check System Resources
```bash
free -h  # Check available RAM
df -h    # Check available disk space
```

### Verify Internet Connection
```bash
ping -c 4 google.com
```

## Common Issues and Solutions

### Issue: Slow Performance in Virtual Machine
**Solution**:
- Enable hardware virtualization in BIOS
- Allocate more RAM and CPU cores to the VM
- Enable 3D acceleration
- Install guest additions/extensions

### Issue: No Sound or Graphics Acceleration
**Solution**:
- Install appropriate graphics drivers
- For virtual machines, install guest additions
- Check display settings in Ubuntu

### Issue: Boot Problems After Installation
**Solution**:
- Boot with nomodeset kernel parameter (for graphics issues)
- Check UEFI/Legacy boot settings in BIOS
- Verify installation media integrity

## Next Steps

After successfully installing and configuring Ubuntu 22.04 LTS, proceed to the next chapter where you'll install ROS 2 Humble Hawksbill, the primary robotics framework used throughout this curriculum.

## Key Takeaways

- Ubuntu 22.04 LTS provides the most stable and well-supported environment for robotics development
- Proper hardware configuration is essential for simulation performance
- System updates should be applied regularly for security and stability
- Graphics drivers are important for visualization tools used in robotics

Continue to the next chapter to install ROS 2 and set up your robotics development environment.