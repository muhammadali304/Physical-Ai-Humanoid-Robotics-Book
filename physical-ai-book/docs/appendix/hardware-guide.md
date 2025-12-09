---
sidebar_position: 1
---

# Hardware Guide - Physical AI & Humanoid Robotics

## Learning Objectives

By the end of this appendix, you will be able to:
- Understand the three-tier hardware approach (Simulation-only, Budget, Standard, Premium)
- Evaluate different hardware options for Physical AI and humanoid robotics
- Select appropriate hardware based on project requirements and budget
- Configure hardware for optimal performance with ROS 2 and Isaac platforms
- Troubleshoot common hardware compatibility issues
- Plan for hardware upgrades and expansions

## Prerequisites

Before reviewing this hardware guide, you should:
- Have completed the ROS 2 fundamentals chapters
- Understand simulation environments (Gazebo, Isaac Sim)
- Know the basic requirements for running ROS 2 and Isaac platforms
- Understand the difference between simulation and real-world deployment

## Conceptual Overview

The **three-tier hardware approach** provides flexibility for different project needs and budgets while maintaining the ability to develop and test robotics applications effectively.

### Hardware Tiers

1. **Simulation-only**: Perfect for learning without hardware investment
2. **Budget Tier** (~$700): TurtleBot 4 or similar for hands-on experience
3. **Standard Tier** (~$3K): Quadruped robots for advanced projects
4. **Premium Tier** (~$16K+): Full humanoid robots for complete experience

### Key Considerations

- **Computational Requirements**: Processing power for perception, planning, and control
- **Connectivity**: Reliable communication with sensors and actuators
- **Power Management**: Battery life and charging capabilities
- **Expandability**: Ability to add new sensors and components
- **Safety**: Emergency stops and collision avoidance capabilities
- **Development vs. Production**: Prototyping vs. deployment requirements

### Cost Transparency

This guide provides exact model numbers and current pricing (as of December 2025) to ensure realistic project planning and budgeting.

## Hardware Recommendations by Tier

### Tier 1: Simulation-only (Free-$100)

**Perfect for Learning and Development**

#### Minimum System Requirements
- **CPU**: Intel i5-10400 or AMD Ryzen 5 3600
- **RAM**: 16GB DDR4 (32GB recommended)
- **GPU**: GTX 1060 6GB or RTX 2060 (for Isaac Sim)
- **Storage**: 500GB SSD (1TB recommended)
- **OS**: Ubuntu 22.04 LTS

#### Recommended Desktop Configurations

**Budget Desktop (~$800)**
- CPU: AMD Ryzen 5 5600X ($180)
- Motherboard: B550 Chipset ($100)
- RAM: 16GB DDR4-3200 ($60)
- GPU: RTX 3060 12GB ($350)
- Storage: 1TB NVMe SSD ($60)
- PSU: 650W 80+ Bronze ($70)
- Case: Mid-tower ($50)
- **Total**: ~$870

**Performance Desktop (~$1,500)**
- CPU: AMD Ryzen 7 5800X ($280)
- Motherboard: B550 Chipset ($120)
- RAM: 32GB DDR4-3600 ($120)
- GPU: RTX 4070 ($600)
- Storage: 2TB NVMe SSD ($100)
- PSU: 750W 80+ Gold ($100)
- Case: Mid-tower ($80)
- **Total**: ~$1,400

#### Laptop Options

**Gaming Laptop for Portability (~$1,200-2,000)**
- Dell G15 Gaming Laptop
  - i7-12700H, 16GB RAM, RTX 3060, 512GB SSD: ~$1,200
- ASUS ROG Strix G15
  - i7-12700H, 32GB RAM, RTX 4060, 1TB SSD: ~$1,600
- Alienware m16 R1
  - i9-13900HX, 32GB RAM, RTX 4070, 1TB SSD: ~$2,200

### Tier 2: Budget Tier (~$700)

**TurtleBot 4 for Hands-on Experience**

#### TurtleBot 4 Kit (~$700)
- **Robot Platform**: TurtleBot 4 (includes iRobot Create 3 + Expansion Plate)
- **Compute**: Raspberry Pi 4 (4GB) or NVIDIA Jetson Nano
- **Sensors**:
  - RGB-D Camera (Intel RealSense D435i) - $150
  - IMU for orientation
- **Battery**: 18650 Li-ion pack
- **Accessories**: Charging dock, cables

**Total Cost**: ~$700-800

#### Alternative Budget Options

**DIY Differential Drive Robot (~$600-800)**
- **Chassis**: Dagu Rover 5 or similar tracked platform - $150
- **Controller**: Raspberry Pi 4 (4GB) - $100
- **Sensors**:
  - LIDAR: YDLIDAR X4 - $80
  - Camera: Pi Camera V2 or RealSense D435i - $75-150
  - IMU: MPU-6050 - $15
- **Motors & Drivers**: 4x DC motors + motor driver board - $100
- **Battery**: 11.1V LiPo + voltage regulators - $80
- **Frame**: Aluminum extrusion + mounting plates - $100

#### Budget Tier Considerations
- **Pros**: Affordable entry point, good for learning, expandable
- **Cons**: Limited computational power, basic sensors, not humanoid
- **Best For**: ROS 2 learning, basic navigation, perception experiments

### Tier 3: Standard Tier (~$3,000)

**Quadruped Robots for Advanced Projects**

#### Unitree Go2 Quadruped (~$2,600)
- **Platform**: Unitree Go2 (2-year-old model may be available used)
- **Specifications**:
  - 12 servo motors (3 per leg)
  - 7-15 minutes runtime
  - 10kg payload
  - IP54 rating
- **Compute**: Built-in NVIDIA Xavier NX
- **Sensors**: RGB camera, IMU, depth sensor
- **SDK**: ROS 2 support included

#### Alternative: DIY Quadruped (~$2,500-3,500)

**Custom Quadruped Build**
- **Frame**: Carbon fiber legs and body - $400
- **Servos**: 12x Dynamixel X-Series (XM430-W350-T) - $1,200
- **Compute**: NVIDIA Jetson AGX Orin - $600
- **Sensors**:
  - RGB Camera: FLIR Blackfly S - $300
  - IMU: VectorNav VN-100 - $500
- **Power**: 22.2V LiPo battery system - $200
- **PCB**: Custom servo controller board - $100
- **Assembly**: Hardware, wiring, etc. - $100

#### Standard Tier Considerations
- **Pros**: Dynamic movement, advanced locomotion, good computational power
- **Cons**: Complex control algorithms, higher cost, shorter battery life
- **Best For**: Advanced locomotion, gait planning, dynamic control

### Tier 4: Premium Tier (~$16,000+)

**Full Humanoid Robots for Complete Experience**

#### Popular Humanoid Platforms

**NAO Robot (~$14,000-16,000)**
- **Platform**: SoftBank Robotics NAO v6
- **Specifications**:
  - 25 degrees of freedom
  - 25 sensors (cameras, microphones, tactile, etc.)
  - 90 minutes battery life
  - WiFi, Ethernet connectivity
- **Compute**: Intel Atom quad-core
- **Software**: NAOqi OS with ROS bridge
- **Programming**: Python, C++, Choregraphe

**Alternative: Custom Humanoid Build (~$15,000-25,000)**
- **Frame**: Carbon fiber and aluminum construction - $2,000
- **Servos**: 20-25x high-torque servos (RH-P12-RN-A, etc.) - $8,000
- **Compute**: NVIDIA Jetson AGX Orin or equivalent - $700
- **Sensors**:
  - Stereo cameras: ZED 2i - $500
  - IMU: Advanced IMU system - $800
  - Force sensors: FSR arrays in feet - $300
  - Microphones: Array for localization - $200
- **Power**: Sophisticated battery management - $500
- **Safety**: Emergency stops, collision detection - $400
- **Assembly & Integration**: Labor and expertise - $2,000

#### Premium Tier Considerations
- **Pros**: Full humanoid experience, advanced interaction, complete platform
- **Cons**: Very high cost, complex maintenance, specialized knowledge required
- **Best For**: Advanced research, human-robot interaction studies, complete solution

## Hardware Configuration Guides

### Setting Up Compute Platforms

#### NVIDIA Jetson Series

**Jetson Nano (Budget AI Compute)**
```bash
# Flash SD card with Jetson Nano image
sudo apt update && sudo apt install -y jetson-stats
sudo jtop  # Monitor system status
```

**Jetson AGX Orin (High-Performance AI)**
```bash
# Install Isaac ROS packages
sudo apt update
sudo apt install ros-humble-isaac-ros-* ros-humble-nvidia-isaac-ros-*
```

#### Raspberry Pi Setup
```bash
# Install ROS 2 on Raspberry Pi
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-ros-base
```

### Sensor Integration

#### Intel RealSense D435i Setup
```bash
# Install RealSense drivers
sudo apt install ros-humble-realsense2-camera
# Launch camera
ros2 launch realsense2_camera rs_launch.py
```

#### LIDAR Integration
```bash
# For YDLIDAR X4
sudo apt install ros-humble-ydlidar
# Launch LIDAR
ros2 launch ydlidar lidar.launch.py
```

### Power Management

#### Battery Monitoring
```bash
# Create battery monitoring node
# Monitor voltage, current, and estimate remaining capacity
# Implement low-battery shutdown procedures
```

## Testing & Verification

### Hardware Compatibility Testing

1. **Compute Platform Verification:**
```bash
# Check system resources
nproc  # Number of CPU cores
free -h  # Available RAM
df -h  # Available storage
nvidia-smi  # GPU status (if applicable)
```

2. **Sensor Connectivity Testing:**
```bash
# Check connected USB devices
lsusb
# Check serial devices
ls /dev/ttyUSB* /dev/ttyACM*
# Test camera
v4l2-ctl --list-devices
```

3. **ROS 2 Hardware Interface Testing:**
```bash
# Check available topics
ros2 topic list
# Verify sensor data
ros2 topic echo /camera/color/image_raw
ros2 topic echo /scan
```

### Performance Benchmarks

#### Compute Performance
- **CPU**: Run `stress-ng --cpu 4` for sustained load test
- **GPU**: Run `gpu_burn 60` for 60-second stress test
- **Memory**: Run `stress-ng --vm 2 --vm-bytes 50%` to test RAM

#### Sensor Performance
- **Camera**: Verify frame rate and exposure settings
- **LIDAR**: Check scan frequency and range accuracy
- **IMU**: Verify orientation and acceleration readings

#### Robot Performance
- **Navigation**: Test path planning and obstacle avoidance
- **Battery Life**: Measure actual vs. rated battery duration
- **Communication**: Verify reliable data transmission rates

### Integration Testing

#### Complete System Test
```bash
# Launch robot bringup
ros2 launch robot_bringup robot.launch.py
# Test basic movements
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
# Monitor sensor data
rviz2  # Visualize all sensor feeds
```

## Common Issues

### Issue: Insufficient computational power for real-time processing
**Solution**:
- Use more efficient algorithms
- Reduce sensor data resolution/frequency
- Implement multi-threading for parallel processing
- Consider edge computing accelerators (Neural Compute Stick, etc.)

### Issue: Sensor compatibility problems
**Solution**:
- Verify ROS 2 driver compatibility
- Check power requirements
- Ensure proper cable connections
- Test sensors individually before system integration

### Issue: Battery life shorter than expected
**Solution**:
- Optimize code for power efficiency
- Monitor power consumption patterns
- Implement power management strategies
- Consider larger capacity batteries

### Issue: Communication interference between components
**Solution**:
- Use shielded cables
- Separate high-power and sensitive circuits
- Implement proper grounding
- Consider wireless communication for some sensors

### Issue: Overheating during extended operation
**Solution**:
- Add cooling fans to compute platforms
- Monitor thermal sensors
- Implement thermal throttling
- Reduce computational load during peak temperatures

## Key Takeaways

- Three-tier approach balances cost, capability, and learning objectives
- Simulation-only tier adequate for most learning objectives
- Budget tier provides hands-on experience with real sensors
- Standard tier enables advanced locomotion and dynamic behaviors
- Premium tier offers complete humanoid experience
- Proper power management critical for all tiers
- Sensor integration requires careful planning and testing
- Performance testing essential before deployment

## Next Steps

In the next appendix, you'll learn about troubleshooting techniques for both hardware and software issues in Physical AI and humanoid robotics projects.