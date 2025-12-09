# Isaac Sim Setup Guide for Reinforcement Learning

This document provides instructions for setting up Isaac Sim for reinforcement learning applications in humanoid robotics.

## Overview

Isaac Sim is NVIDIA's robotics simulation environment that provides high-fidelity physics simulation and reinforcement learning capabilities. This guide covers the setup process for using Isaac Sim with reinforcement learning frameworks.

## Prerequisites

- NVIDIA GPU with RTX or GTX 1080+ (with CUDA support)
- Ubuntu 20.04 or 22.04 LTS
- NVIDIA GPU driver (version 470 or higher)
- CUDA 11.8 or higher
- Isaac Sim 2023.1.1 or higher
- Python 3.8 or higher

## Installation Steps

### 1. Install NVIDIA GPU Drivers

```bash
# Check current driver
nvidia-smi

# Install latest drivers
sudo apt update
sudo apt install nvidia-driver-535 nvidia-utils-535
sudo reboot
```

### 2. Install Isaac Sim

#### Option A: Using Omniverse Launcher (Recommended)
1. Download and install the Omniverse Launcher from NVIDIA Developer website
2. Sign in with your NVIDIA Developer account
3. Search for Isaac Sim and click Install
4. Launch Isaac Sim from the launcher

#### Option B: Direct Installation
```bash
# Download Isaac Sim
wget https://developer.nvidia.com/isaac-sim/latest/release-notes
# Follow the download link in the release notes for the latest version

# Extract and install
tar -xzf isaac-sim-*.tar.gz
cd isaac-sim
./install.sh
```

### 3. Verify Installation

Launch Isaac Sim and verify it runs correctly:
```bash
# If installed via Omniverse Launcher
# Launch from the Omniverse Launcher

# If installed directly
cd ~/isaac-sim
./python.sh
```

### 4. Install RL Framework Dependencies

```bash
# Activate Isaac Sim Python environment
cd ~/isaac-sim
source setup_conda_env.sh

# Install reinforcement learning frameworks
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install stable-baselines3[extra]
pip install gymnasium[box2d]
pip install ray[rllib]
pip install wandb  # For experiment tracking
```

## Isaac Sim RL Configuration

### 1. Enable RL Extension

1. Launch Isaac Sim
2. Go to Window → Extensions
3. Search for "RL Games" or "Reinforcement Learning"
4. Enable the extension

### 2. Configure Simulation Settings

For optimal RL training performance:

```python
# Example configuration for RL training
{
    "physics": {
        "solver_type": 1,  # TGS solver for better stability
        "use_gpu": True,   # Enable GPU physics
        "solver_position_iteration_count": 8,
        "solver_velocity_iteration_count": 2
    },
    "rendering": {
        "width": 640,
        "height": 480,
        "fps": 30
    },
    "rl": {
        "max_episode_length": 1000,
        "num_envs": 256,  # Number of parallel environments
        "enable_viewport_render": False  # Disable rendering for training
    }
}
```

### 3. Create RL Training Environment

#### Basic Robot Configuration
```python
# Example robot configuration for RL
{
    "robot": {
        "type": "Humanoid",
        "urdf_path": "/path/to/humanoid.urdf",
        "scale": 1.0,
        "default_dof_control": {
            "stiffness": 800.0,
            "damping": 50.0
        }
    },
    "scene": {
        "type": "basic",
        "gravity": -9.81
    },
    "tasks": {
        "locomotion": {
            "target_velocity": 1.0,
            "terrain_types": ["flat", "rough", "stairs"]
        }
    }
}
```

## RL Training Setup Examples

### 1. Basic Locomotion Training

Create a simple locomotion task for humanoid robots:

```python
# Example: Humanoid locomotion training script
import omni
from omni.isaac.kit import SimulationApp

# Launch Isaac Sim
config = {
    "headless": True,  # Set to False for visualization
    "enable_cameras": False,
    "num_threads": 4
}
simulation_app = SimulationApp(config)

# Import RL libraries
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage

# Setup world and robot
world = World(stage_units_in_meters=1.0)

# Add your humanoid robot
asset_path = get_assets_root_path() + "/Isaac/Robots/Humanoid/humanoid_instanceable.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Humanoid")

# Configure RL training parameters
rl_config = {
    "env": {
        "numEnvs": 256,
        "envSpacing": 5.0,
        "episodeLength": 1000,
        "enableDebugVis": False,
    },
    "sim": {
        "dt": 1.0 / 60.0,
        "use_gpu_pipeline": True,
        "gravity": [0.0, 0.0, -9.81],
    }
}

# Start simulation
world.reset()
while simulation_app.is_running():
    world.step(render=True)
    if world.current_time_step_index == 0:
        world.reset(soft=True)

simulation_app.close()
```

### 2. Isaac Sim RL Games Integration

For advanced RL training with Isaac Sim's built-in RL Games:

```bash
# Install Isaac Sim RL Games
cd ~/isaac-sim
python -m pip install -e ./isaacsim-rl
```

## Performance Optimization

### 1. GPU Acceleration
- Enable GPU physics simulation for faster training
- Use Tensor operations for parallel computation
- Optimize rendering settings for training vs. visualization

### 2. Parallel Environments
- Use multiple parallel environments for sample efficiency
- Balance number of environments with available GPU memory
- Monitor GPU utilization during training

### 3. Curriculum Learning
- Start with simpler tasks and gradually increase difficulty
- Use shaped rewards to guide learning
- Implement adaptive difficulty based on agent performance

## Troubleshooting

### Common Issues:

1. **GPU Memory Issues**:
   - Reduce number of parallel environments
   - Lower simulation resolution
   - Use mixed precision training

2. **Physics Instability**:
   - Adjust solver parameters
   - Reduce time step size
   - Increase solver iterations

3. **Rendering Performance**:
   - Disable viewport rendering during training
   - Reduce camera resolution
   - Use headless mode for training

### Verification Steps:

1. Launch Isaac Sim successfully
2. Load a simple robot model
3. Run physics simulation without errors
4. Verify RL extensions are enabled
5. Test basic movement in simulation

## Integration with ROS 2

Isaac Sim can be integrated with ROS 2 for hybrid simulation:

```bash
# Install Isaac ROS bridge
sudo apt update
sudo apt install ros-humble-isaac-ros-gems ros-humble-isaac-ros-interfaces
```

## Next Steps

After successful setup:
1. Create custom humanoid robot models
2. Define specific RL tasks for your application
3. Configure reward functions for training
4. Start with simple tasks before complex behaviors
5. Monitor training progress and adjust hyperparameters

## Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
- [Isaac ROS Integration Guide](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common)
- [NVIDIA Developer Program](https://developer.nvidia.com/)