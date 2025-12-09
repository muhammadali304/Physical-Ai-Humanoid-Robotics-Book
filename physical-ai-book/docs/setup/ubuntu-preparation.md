# Ubuntu 22.04 Preparation Guide for ROS 2 Development

## Overview

This guide provides comprehensive instructions for preparing an Ubuntu 22.04 LTS system for ROS 2 development, specifically for the Physical AI & Humanoid Robotics educational curriculum. Following these steps will ensure your system meets all requirements for the robotics development environment.

## System Requirements

### Minimum Hardware Requirements
- **CPU**: Intel/AMD 64-bit processor with 4+ cores
- **RAM**: 8 GB (16 GB recommended)
- **Storage**: 50 GB free space (100 GB recommended for simulation work)
- **Graphics**: OpenGL 2.1+ compatible GPU (for visualization tools)
- **Network**: Internet connection for package installation

### Recommended Hardware
- **CPU**: Intel i7 or AMD Ryzen 7 with 8+ cores
- **RAM**: 16 GB or more
- **Storage**: SSD with 100+ GB free space
- **Graphics**: Dedicated GPU with CUDA support (for Isaac ROS packages)

### Operating System
- **Distribution**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Architecture**: 64-bit (amd64)
- **Installation Type**: Desktop or Server (Desktop recommended for GUI tools)

## Pre-Installation Checklist

Before beginning the installation, verify:

- [ ] System meets minimum hardware requirements
- [ ] Backup of important data (if installing fresh)
- [ ] Stable internet connection available
- [ ] Administrative (sudo) access to the system
- [ ] BIOS/UEFI settings allow for OS installation (if needed)

## System Preparation Steps

### 1. Update System Packages

After installing Ubuntu 22.04, update the system:

```bash
sudo apt update && sudo apt upgrade -y
```

### 2. Install Essential Build Tools

```bash
sudo apt install -y build-essential cmake git wget curl gnupg lsb-release
```

### 3. Set Up Locale for UTF-8

Ensure your locale is properly configured:

```bash
locale  # Check current locale
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 4. Configure Swap Space (Recommended)

For systems with less than 16GB RAM, configure swap space:

```bash
# Check current swap
free -h

# Create 4GB swap file (adjust size as needed)
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Make swap permanent
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### 5. Install Additional Development Tools

```bash
sudo apt install -y \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv \
    vim \
    htop \
    iotop \
    sysstat \
    usbutils \
    pciutils \
    bash-completion
```

## Performance Optimization

### 1. Configure Swappiness

For systems with 8GB+ RAM used for robotics development:

```bash
# Check current swappiness
cat /proc/sys/vm/swappiness

# Set swappiness to 10 (default is 60)
echo 'vm.swappiness=10' | sudo tee -a /etc/sysctl.conf
```

### 2. Increase File Descriptor Limits

For handling many ROS 2 nodes and processes:

```bash
echo "* soft nofile 65536" | sudo tee -a /etc/security/limits.conf
echo "* hard nofile 65536" | sudo tee -a /etc/security/limits.conf
```

### 3. Configure Network Settings

For ROS 2 multi-device communication:

```bash
# Add to ~/.bashrc for ROS 2 networking
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
```

## Graphics and Display Configuration

### 1. Install Graphics Drivers

For NVIDIA graphics (if applicable):

```bash
# Check if NVIDIA GPU is present
lspci | grep -i nvidia

# Install appropriate drivers
sudo ubuntu-drivers devices
sudo ubuntu-drivers autoinstall
```

### 2. Install Graphics Libraries

```bash
sudo apt install -y \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libosmesa6-dev \
    freeglut3-dev
```

## Development Environment Setup

### 1. Create Development Directory Structure

```bash
mkdir -p ~/ros2_ws/src
mkdir -p ~/simulations
mkdir -p ~/robotics_projects
```

### 2. Install Version Control Tools

```bash
sudo apt install -y git git-lfs
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

### 3. Set Up SSH Keys (for GitHub/git access)

```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
```

## Security Considerations

### 1. Firewall Configuration

For ROS 2 communication while maintaining security:

```bash
# Install and configure ufw
sudo apt install ufw
sudo ufw default deny incoming
sudo ufw default allow outgoing
sudo ufw allow ssh
sudo ufw enable
```

### 2. User Permissions

Add user to necessary groups:

```bash
sudo usermod -a -G dialout $USER  # For serial communication
sudo usermod -a -G plugdev $USER  # For device access
```

## Verification Steps

### 1. Check System Information

```bash
# Verify Ubuntu version
lsb_release -a

# Check available memory
free -h

# Check available disk space
df -h

# Verify graphics capabilities
glxinfo | grep "OpenGL version"
```

### 2. Test Internet Connectivity

```bash
ping -c 4 packages.ros.org
ping -c 4 github.com
```

## Troubleshooting Common Issues

### Issue: Insufficient disk space during installation
**Solution**: Clear package cache and temporary files
```bash
sudo apt autoremove
sudo apt autoclean
sudo rm -rf /tmp/*
```

### Issue: Locale errors during ROS 2 installation
**Solution**: Reconfigure locales
```bash
sudo dpkg-reconfigure locales
```

### Issue: Graphics driver problems
**Solution**: Check and reinstall graphics drivers
```bash
sudo ubuntu-drivers devices
sudo ubuntu-drivers autoinstall
sudo reboot
```

## Next Steps

After completing this preparation:

1. Proceed with ROS 2 Humble Hawksbill installation
2. Set up your ROS 2 workspace
3. Install Gazebo simulation environment
4. Begin with basic ROS 2 tutorials

## Maintenance Tips

### Regular System Maintenance
- Update packages regularly: `sudo apt update && sudo apt upgrade`
- Clean package cache: `sudo apt autoclean && sudo apt autoremove`
- Monitor disk space: `df -h`
- Check system logs: `journalctl -xe`

### Performance Monitoring
- Monitor system resources: `htop`
- Check disk usage: `du -sh ~/*`
- Monitor network: `iftop` (install with `sudo apt install iftop`)

---

**Note**: This preparation guide should be completed before proceeding with ROS 2 installation. The system should be rebooted after completing all steps to ensure all configurations take effect.