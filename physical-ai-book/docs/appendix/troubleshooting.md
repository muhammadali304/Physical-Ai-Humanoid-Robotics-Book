---
sidebar_position: 2
---

# Troubleshooting Guide - Physical AI & Humanoid Robotics

## Learning Objectives

By the end of this appendix, you will be able to:
- Diagnose and resolve common ROS 2 and system issues
- Troubleshoot simulation environment problems
- Address Isaac platform and perception system issues
- Debug VLA (Vision-Language-Action) system problems
- Handle hardware-specific troubleshooting scenarios
- Apply systematic troubleshooting methodologies
- Create and maintain troubleshooting documentation

## Prerequisites

Before using this troubleshooting guide, you should:
- Have completed all previous chapters in this book
- Understand ROS 2 concepts, nodes, topics, and services
- Be familiar with the development environment and tools
- Have basic Linux command line troubleshooting skills
- Experience with debugging software and hardware systems

## Conceptual Overview

**Troubleshooting** is a systematic approach to identifying, diagnosing, and resolving problems in complex robotics systems. In Physical AI and humanoid robotics, troubleshooting spans multiple domains:

1. **Software Domain**: ROS 2 nodes, packages, and communication
2. **Simulation Domain**: Gazebo, Unity, and Isaac Sim issues
3. **Perception Domain**: Vision, sensor, and AI system problems
4. **Hardware Domain**: Robot components, sensors, and actuators
5. **Integration Domain**: Cross-system interactions and dependencies

### Troubleshooting Methodology

Effective troubleshooting follows a systematic approach:

1. **Problem Identification**: Clearly define the issue
2. **Information Gathering**: Collect relevant logs, error messages, and symptoms
3. **Hypothesis Formation**: Propose potential causes
4. **Testing**: Verify hypotheses through controlled experiments
5. **Resolution**: Implement and verify the fix
6. **Documentation**: Record the issue and solution for future reference

### Common Troubleshooting Categories

- **Environmental Issues**: System setup, dependencies, configurations
- **Communication Issues**: Network, topics, services, actions
- **Performance Issues**: Speed, responsiveness, resource usage
- **Integration Issues**: Component interactions and dependencies
- **Hardware Issues**: Sensors, actuators, connectivity, power

## Hands-On Implementation

### General ROS 2 Troubleshooting

#### Checking ROS 2 Environment

```bash
# Check ROS 2 installation
printenv | grep ROS
ros2 --version

# Check available packages
ros2 pkg list | grep -E "(navigation|perception|isaac|sim)"

# List active nodes
ros2 node list

# Check topics
ros2 topic list
ros2 topic info /topic_name

# Check services
ros2 service list
ros2 service info /service_name

# Check actions
ros2 action list
ros2 action info /action_name
```

#### ROS 2 Network and Communication Issues

```bash
# Check ROS domain ID
echo $ROS_DOMAIN_ID

# Verify network connectivity
ping localhost
hostname -I

# Check for multiple ROS installations
which ros2
printenv | grep -i ros

# Check for conflicting processes
ps aux | grep -E "(ros|gazebo|rviz)"
```

#### Common ROS 2 Error Resolution

**Issue: "command not found" for ROS 2 commands**
```bash
# Solution: Source the ROS 2 setup
source /opt/ros/humble/setup.bash
# Or add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

**Issue: Nodes can't communicate**
```bash
# Solution: Check if nodes are on the same ROS domain
export ROS_DOMAIN_ID=0
# Verify network settings
ros2 daemon stop
ros2 daemon start
```

### Simulation Environment Troubleshooting

#### Gazebo Troubleshooting

```bash
# Check Gazebo installation
gz --version
# Or for older versions
gazebo --version

# Check Gazebo resources
echo $GAZEBO_RESOURCE_PATH
echo $GAZEBO_MODEL_PATH

# Run Gazebo with verbose output
gz sim -v 4

# Check for GPU issues
nvidia-smi
glxinfo | grep -i "direct rendering"
glxgears  # Test OpenGL rendering
```

**Common Gazebo Issues:**

**Issue: Gazebo fails to start with graphics errors**
```bash
# Solution 1: Try software rendering
export MESA_GL_VERSION_OVERRIDE=3.3
gz sim

# Solution 2: Check graphics drivers
sudo apt install mesa-utils
sudo apt install nvidia-prime  # For NVIDIA systems
```

**Issue: Models not appearing in Gazebo**
```bash
# Check model paths
echo $GAZEBO_MODEL_PATH
ls -la /usr/share/gazebo-*/models/
# Or for custom models
ls -la ~/.gazebo/models/
```

#### Isaac Sim Troubleshooting

```bash
# Check Isaac Sim installation
# If using Docker
docker ps | grep isaac

# Check Omniverse logs
ls -la ~/.nvidia-omniverse/logs/

# Verify GPU compatibility
nvidia-smi
# Check CUDA version
nvcc --version
```

**Common Isaac Sim Issues:**

**Issue: Isaac Sim crashes or won't start**
```bash
# Check system requirements
nvidia-smi  # Verify GPU and driver
free -h     # Check available RAM
df -h       # Check available disk space

# Increase shared memory
mount -o remount,size=8G /dev/shm
# Or add to /etc/fstab:
# tmpfs /dev/shm tmpfs defaults,size=8G 0 0
```

### Isaac ROS Perception Troubleshooting

#### Image Pipeline Issues

```bash
# Check camera topics
ros2 topic list | grep -E "(image|camera)"

# Verify image data
ros2 topic echo /camera/image_raw --field data | head

# Check camera calibration
ros2 param list | grep camera
ros2 param get /camera_node_name camera_info_url
```

#### Detection and Perception Issues

```bash
# Check detection topics
ros2 topic list | grep -E "(detection|detect)"

# Monitor detection performance
ros2 topic hz /detections

# Check perception pipeline status
ros2 lifecycle list perception_node_name
```

**Common Perception Issues:**

**Issue: No detections or poor detection quality**
```bash
# Check sensor data quality
ros2 topic echo /camera/image_raw --field header | head -5
# Verify lighting conditions in simulation
# Check camera calibration parameters
ros2 param get /camera_node_name distortion_coefficients
```

### Navigation System Troubleshooting

#### Costmap and Localization Issues

```bash
# Check costmap topics
ros2 topic list | grep costmap

# Monitor localization
ros2 topic echo /amcl_pose
ros2 topic echo /particle_cloud

# Check map server
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: 'path/to/map.yaml'}"
```

#### Path Planning Issues

```bash
# Check planner status
ros2 action list | grep navigate
ros2 action info /navigate_to_pose

# Monitor global planner
ros2 topic echo /global_costmap/costmap
ros2 topic echo /plan

# Check local planner
ros2 topic echo /local_costmap/costmap
ros2 topic echo /local_plan
```

**Common Navigation Issues:**

**Issue: Robot gets stuck or cannot find path**
```bash
# Check costmap inflation
ros2 param get /global_costmap_node inflation_layer.inflation_radius
# Verify map quality and resolution
ros2 param get /map_server resolution
# Check for obstacles blocking path
ros2 topic echo /scan
```

### VLA (Vision-Language-Action) System Troubleshooting

#### Voice Command Issues

```bash
# Check audio devices
arecord -l
aplay -l

# Test microphone
arecord -D hw:0,0 -f cd test.wav
# Play back
aplay test.wav

# Check voice command topics
ros2 topic list | grep -E "(voice|speech|audio)"
```

#### LLM Integration Issues

```bash
# Check API connectivity (for cloud-based LLMs)
curl -I https://api.openai.com/v1/models

# Verify LLM response format
ros2 topic echo /llm_response

# Check plan generation
ros2 topic echo /generated_plan
```

**Common VLA Issues:**

**Issue: Voice commands not recognized**
```bash
# Check audio input quality
# Verify microphone permissions
# Test with different speech recognition engines
# Check ambient noise levels
```

### Hardware Troubleshooting

#### Compute Platform Issues

```bash
# Monitor system resources
htop
# Or for GPU usage
nvidia-smi
# Check temperatures
sensors

# Check for thermal throttling
dmesg | grep -i thermal
```

#### Sensor Issues

```bash
# Check sensor connections
lsusb
ls /dev/tty*

# Test sensor individually
# For camera:
v4l2-ctl --list-devices
# For IMU:
sudo i2cdetect -y -r 1

# Check sensor data streams
# Camera:
ros2 topic echo /camera/image_raw --field header | head -5
# IMU:
ros2 topic echo /imu
```

### Systematic Troubleshooting Approach

#### Creating a Troubleshooting Log

```bash
# Create troubleshooting log
mkdir -p ~/troubleshooting_logs
touch ~/troubleshooting_logs/$(date +%Y%m%d_%H%M%S)_issue.log

# Example log entry
cat >> ~/troubleshooting_logs/$(date +%Y%m%d_%H%M%S)_issue.log << EOF
Date: $(date)
Issue: [Brief description]
Symptoms: [What you observe]
Environment: [ROS version, Ubuntu version, hardware]
Steps taken: [What you've tried so far]
Error messages: [Copy exact error messages]
Resolution: [How you fixed it]
EOF
```

#### Diagnostic Commands Collection

```bash
#!/bin/bash
# diagnostic_collector.sh - Collect system diagnostics

echo "=== System Information ===" > diagnostics_$(date +%Y%m%d_%H%M%S).txt
uname -a >> diagnostics_$(date +%Y%m%d_%H%M%S).txt
echo -e "\n=== ROS 2 Environment ===" >> diagnostics_$(date +%Y%m%d_%H%M%S).txt
printenv | grep ROS >> diagnostics_$(date +%Y%m%d_%H%M%S).txt
echo -e "\n=== Available Packages ===" >> diagnostics_$(date +%Y%m%d_%H%M%S).txt
ros2 pkg list | head -20 >> diagnostics_$(date +%Y%m%d_%H%M%S).txt
echo -e "\n=== Active Nodes ===" >> diagnostics_$(date +%Y%m%d_%H%M%S).txt
ros2 node list >> diagnostics_$(date +%Y%m%d_%H%M%S).txt
echo -e "\n=== System Resources ===" >> diagnostics_$(date +%Y%m%d_%H%M%S).txt
free -h >> diagnostics_$(date +%Y%m%d_%H%M%S).txt
df -h >> diagnostics_$(date +%Y%m%d_%H%M%S).txt
echo -e "\n=== GPU Info ===" >> diagnostics_$(date +%Y%m%d_%H%M%S).txt
nvidia-smi >> diagnostics_$(date +%Y%m%d_%H%M%S).txt 2>&1 || echo "No NVIDIA GPU detected"
```

## Testing & Verification

### Creating Test Scripts

#### Basic System Health Check

```bash
#!/bin/bash
# system_health_check.sh

echo "=== ROS 2 System Health Check ==="
echo "Date: $(date)"
echo "ROS 2 Version: $(ros2 --version)"
echo "ROS Distribution: $ROS_DISTRO"
echo "ROS Domain ID: $ROS_DOMAIN_ID"
echo

echo "=== Active Nodes ==="
NODE_COUNT=$(ros2 node list | wc -l)
echo "Active nodes: $NODE_COUNT"
if [ $NODE_COUNT -lt 5 ]; then
    echo "WARNING: Low number of active nodes"
fi
echo

echo "=== Topic Status ==="
TOPIC_COUNT=$(ros2 topic list | wc -l)
echo "Active topics: $TOPIC_COUNT"
echo "Sample topics:"
ros2 topic list | head -10
echo

echo "=== System Resources ==="
echo "Memory usage:"
free -h
echo
echo "Disk usage:"
df -h $HOME
echo

echo "=== GPU Status ==="
nvidia-smi --query-gpu=name,memory.used,memory.total,temperature.gpu --format=csv,noheader,nounits 2>/dev/null || echo "No NVIDIA GPU detected"
echo

echo "=== Network Status ==="
hostname -I
echo "ROS communication test:"
ros2 topic list >/dev/null 2>&1 && echo "✓ ROS communication OK" || echo "✗ ROS communication FAILED"
```

#### Simulation Environment Test

```bash
#!/bin/bash
# simulation_test.sh

echo "=== Simulation Environment Test ==="
echo "Testing Gazebo installation..."
gz --version >/dev/null 2>&1 && echo "✓ Gazebo installed" || echo "✗ Gazebo not found"

echo "Testing Isaac Sim prerequisites..."
if command -v nvidia-smi &> /dev/null; then
    GPU_COUNT=$(nvidia-smi --list-gpus | wc -l)
    echo "✓ NVIDIA GPU detected: $GPU_COUNT GPU(s)"
else
    echo "✗ No NVIDIA GPU detected"
fi

echo "Testing Isaac Sim Docker (if applicable)..."
docker ps | grep isaac >/dev/null 2>&1 && echo "✓ Isaac Sim container running" || echo "✗ Isaac Sim container not running"
```

### Performance Monitoring

```bash
# Monitor system performance during operation
# CPU and memory usage
watch -n 1 'ps aux --sort=-%cpu | head -10'

# ROS 2 topic frequency
ros2 topic hz /camera/image_raw

# System resource usage
while true; do
    echo "$(date): CPU=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)%, MEM=$(free | grep Mem | awk '{printf("%.2f%%", $3/$2 * 100.0)}')"
    sleep 5
done
```

### Troubleshooting Verification Checklist

- [ ] **System Environment**: ROS 2 properly sourced and configured
- [ ] **Network Connectivity**: All nodes can communicate
- [ ] **Resource Availability**: Sufficient CPU, memory, and disk space
- [ ] **Hardware Status**: All sensors and actuators responding
- [ ] **Simulation**: Gazebo/Isaac Sim running without errors
- [ ] **Navigation**: Costmaps and planners functioning
- [ ] **Perception**: Sensors providing valid data
- [ ] **Communication**: All topics and services accessible
- [ ] **Performance**: System running within acceptable parameters
- [ ] **Safety**: Emergency stops and safety systems operational

## Common Issues

### Issue: ROS 2 Daemon Problems
**Symptoms**: Nodes can't communicate, "no route to host" errors
**Solution**:
```bash
# Restart ROS 2 daemon
ros2 daemon stop
ros2 daemon start
# Or restart networking
sudo systemctl restart networking
```

### Issue: Gazebo Rendering Problems
**Symptoms**: Black screen, slow rendering, crashes
**Solution**:
```bash
# Increase shared memory
sudo mount -o remount,size=8G /dev/shm
# Try software rendering
export MESA_GL_VERSION_OVERRIDE=3.3
gz sim
```

### Issue: Isaac Sim GPU Memory Problems
**Symptoms**: Out of memory errors, crashes during simulation
**Solution**:
```bash
# Check GPU memory usage
nvidia-smi
# Reduce simulation complexity
# Use lower resolution textures
# Close other GPU-intensive applications
```

### Issue: Navigation Costmap Inflation Problems
**Symptoms**: Robot avoids valid paths, gets stuck
**Solution**:
```bash
# Check inflation radius parameter
ros2 param get /global_costmap_node inflation_layer.inflation_radius
# Verify map resolution matches costmap resolution
ros2 param get /map_server resolution
```

### Issue: Voice Recognition Problems
**Symptoms**: Commands not recognized, high error rate
**Solution**:
```bash
# Check audio input levels
alsamixer
# Test microphone directly
arecord -D hw:0,0 -f cd test.wav
# Check for background noise
```

### Issue: LLM API Connectivity Problems
**Symptoms**: Timeout errors, rate limit exceeded
**Solution**:
```bash
# Check network connectivity
curl -I https://api.openai.com/v1/models
# Verify API key
echo $OPENAI_API_KEY
# Check rate limits in API provider dashboard
```

## Key Takeaways

- Systematic troubleshooting follows a consistent methodology
- Environmental issues should be checked first
- Log files are invaluable for diagnosing problems
- Isolate issues by testing components individually
- Performance monitoring helps prevent issues
- Documentation of solutions prevents repeated troubleshooting
- Hardware and software issues often interact
- Simulation testing should precede real hardware deployment

## Next Steps

This concludes the Physical AI & Humanoid Robotics educational book. You now have comprehensive knowledge spanning from basic ROS 2 concepts through advanced VLA systems and hardware integration. The skills learned throughout this book will enable you to develop, deploy, and maintain sophisticated robotic systems. Continue practicing with increasingly complex projects to deepen your expertise.