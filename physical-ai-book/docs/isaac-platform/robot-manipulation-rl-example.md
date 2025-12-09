# Basic RL Training Example for Robot Manipulation

This document provides a practical example of reinforcement learning for robot manipulation tasks in Isaac Sim.

## Overview

This example demonstrates how to train a robotic arm to perform basic manipulation tasks using reinforcement learning. The example uses the Franka Emika Panda robot as a reference, but the principles apply to other manipulator robots including humanoid arms.

## Prerequisites

- Completed Isaac Sim RL setup (see isaac-sim-rl-setup-guide.md)
- Isaac Sim with RL Games extension enabled
- Python environment with RL libraries installed

## Environment Setup

### 1. Robot and Object Configuration

```python
# robot_manipulation_env.py
import numpy as np
import torch
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.objects import DynamicCuboid


class RobotManipulationEnv:
    def __init__(self, world: World):
        self.world = world
        self.robot = None
        self.object = None
        self.target = None

        # Environment parameters
        self.action_scale = 0.05  # Scale for joint position changes
        self.distance_threshold = 0.05  # Distance threshold for success
        self.max_episode_length = 500

        # Initialize the environment
        self.setup_scene()

    def setup_scene(self):
        """Setup the simulation scene with robot, object, and target"""
        # Get assets root path
        assets_root_path = get_assets_root_path()

        # Add Franka robot
        franka_asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_instanceable.usd"
        add_reference_to_stage(usd_path=franka_asset_path, prim_path="/World/Franka")

        # Create target object (cube to manipulate)
        self.object = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/Object",
                name="object",
                position=np.array([0.5, 0.0, 0.1]),
                size=0.05,
                color=np.array([1.0, 0.0, 0.0])
            )
        )

        # Create target position
        self.target = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/Target",
                name="target",
                position=np.array([0.6, 0.2, 0.1]),
                size=0.05,
                color=np.array([0.0, 1.0, 0.0]),
                mass=0  # Static target
            )
        )

        # Get robot view
        self.robot = self.world.scene.get_object("franka")

    def get_observation(self):
        """Get current observation from the environment"""
        # Get robot joint positions and velocities
        joint_positions = self.robot.get_joint_positions()
        joint_velocities = self.robot.get_joint_velocities()

        # Get object position
        object_position = self.object.get_world_poses()[0][0]

        # Get target position
        target_position = self.target.get_world_poses()[0][0]

        # Get end-effector position
        ee_position = self.get_end_effector_position()

        # Create observation vector
        observation = np.concatenate([
            joint_positions,           # Joint positions
            joint_velocities,          # Joint velocities
            object_position,           # Object position
            target_position,           # Target position
            ee_position,               # End-effector position
            [np.linalg.norm(object_position - ee_position)],  # Distance to object
            [np.linalg.norm(object_position - target_position)]  # Distance to target
        ])

        return observation

    def get_end_effector_position(self):
        """Get the position of the robot's end effector"""
        # Get the end effector link (typically the last link)
        # This may vary depending on the robot model
        ee_link_path = "/World/Franka/panda_hand"
        ee_prim = get_prim_at_path(ee_link_path)
        if ee_prim:
            # Get the world position of the end effector
            from pxr import Gf
            pos_attr = ee_prim.GetAttribute("xformOp:translate")
            if pos_attr:
                return np.array(pos_attr.Get())

        # Fallback: use robot's root position + offset
        robot_pos, _ = self.robot.get_world_poses()
        return robot_pos[0] + np.array([0.5, 0.0, 0.5])

    def apply_action(self, action):
        """Apply action to the robot"""
        # Rescale action
        scaled_action = action * self.action_scale

        # Get current joint positions
        current_positions = self.robot.get_joint_positions()

        # Calculate new joint positions
        new_positions = current_positions + scaled_action

        # Apply new joint positions
        self.robot.set_joint_positions(new_positions)

    def compute_reward(self, action):
        """Compute reward based on current state"""
        # Get positions
        object_position = self.object.get_world_poses()[0][0]
        target_position = self.target.get_world_poses()[0][0]
        ee_position = self.get_end_effector_position()

        # Distance from end-effector to object (for grasping)
        dist_ee_to_obj = np.linalg.norm(object_position - ee_position)

        # Distance from object to target (for manipulation)
        dist_obj_to_target = np.linalg.norm(object_position - target_position)

        # Reward components
        grasp_reward = -dist_ee_to_obj  # Encourage approaching object
        manipulation_reward = -dist_obj_to_target  # Encourage moving object to target
        action_penalty = -np.sum(np.abs(action)) * 0.01  # Penalty for large actions

        # Success bonus if object is close to target
        success_bonus = 100.0 if dist_obj_to_target < self.distance_threshold else 0.0

        # Total reward
        total_reward = grasp_reward * 0.1 + manipulation_reward + action_penalty + success_bonus

        return total_reward

    def is_done(self):
        """Check if episode is done"""
        current_step = self.world.current_time_step_index

        # Get positions
        object_position = self.object.get_world_poses()[0][0]
        target_position = self.target.get_world_poses()[0][0]
        dist_obj_to_target = np.linalg.norm(object_position - target_position)

        # Check if episode is done
        done = (
            current_step >= self.max_episode_length or
            dist_obj_to_target < self.distance_threshold
        )

        return done, dist_obj_to_target < self.distance_threshold

    def reset(self):
        """Reset the environment"""
        # Reset object position
        new_obj_pos = np.array([0.5, 0.0, 0.1]) + np.random.uniform(-0.1, 0.1, 3)
        self.object.set_world_poses(positions=np.array([new_obj_pos]))

        # Reset robot joints to initial positions
        initial_joint_positions = np.array([0.0, -1.0, 0.0, -2.2, 0.0, 1.2, 0.0])
        self.robot.set_joint_positions(initial_joint_positions)

        return self.get_observation()


# Training script using Stable Baselines3
def train_robot_manipulation():
    """Train a robot to manipulate objects using RL"""
    from stable_baselines3 import PPO
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.callbacks import EvalCallback
    import gymnasium as gym
    from gymnasium import spaces

    # Custom gym environment wrapper
    class IsaacGymEnv(gym.Env):
        def __init__(self, world):
            self.env = RobotManipulationEnv(world)

            # Define action and observation spaces
            num_joints = 7  # Franka has 7 joints
            self.action_space = spaces.Box(
                low=-1.0, high=1.0, shape=(num_joints,), dtype=np.float32
            )
            self.observation_space = spaces.Box(
                low=-np.inf, high=np.inf,
                shape=(num_joints * 2 + 3 + 3 + 3 + 2,),  # pos + vel + obj_pos + target_pos + ee_pos + distances
                dtype=np.float32
            )

        def reset(self, seed=None, options=None):
            super().reset(seed=seed)
            obs = self.env.reset()
            return obs.astype(np.float32), {}

        def step(self, action):
            self.env.apply_action(action)

            # Step the simulation
            self.env.world.step(render=True)

            obs = self.env.get_observation()
            reward = self.env.compute_reward(action)
            done, success = self.env.is_done()

            info = {"is_success": success}

            return obs.astype(np.float32), reward, done, False, info

    # Launch Isaac Sim
    config = {"headless": True}
    simulation_app = omni.simulation.SimulationApp(config)

    # Create world
    world = World(stage_units_in_meters=1.0)

    # Create environment
    isaac_env = IsaacGymEnv(world)

    # Create vectorized environment
    vec_env = make_vec_env(lambda: isaac_env, n_envs=1)

    # Create PPO model
    model = PPO(
        "MlpPolicy",
        vec_env,
        verbose=1,
        tensorboard_log="./tb_logs/",
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01
    )

    # Train the model
    model.learn(total_timesteps=100000)

    # Save the model
    model.save("robot_manipulation_model")

    # Close simulation
    simulation_app.close()

    print("Training completed and model saved!")


if __name__ == "__main__":
    train_robot_manipulation()
```

## Running the Example

### 1. Setup and Installation

First, ensure all dependencies are installed:

```bash
# Activate Isaac Sim environment
cd ~/isaac-sim
source setup_conda_env.sh

# Install required packages
pip install stable-baselines3[extra]
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install gymnasium[box2d]
pip install wandb
```

### 2. Execute Training

Save the example code as `robot_manipulation_rl.py` and run:

```bash
cd ~/isaac-sim
source setup_conda_env.sh
python robot_manipulation_rl.py
```

### 3. Monitor Training

Training progress can be monitored using TensorBoard:

```bash
tensorboard --logdir tb_logs/
```

## Advanced RL Techniques

### 1. Curriculum Learning

Implement progressive difficulty increase:

```python
class CurriculumLearning:
    def __init__(self):
        self.current_level = 0
        self.level_thresholds = [0.1, 0.05, 0.02]  # Distance thresholds for each level

    def update_curriculum(self, success_rate):
        if success_rate > 0.8 and self.current_level < len(self.level_thresholds) - 1:
            self.current_level += 1
            print(f"Curriculum advanced to level {self.current_level + 1}")

    def get_current_threshold(self):
        return self.level_thresholds[self.current_level]
```

### 2. Hindsight Experience Replay (HER)

For goal-conditioned tasks:

```python
# Example HER implementation concept
def hindsight_experience_replay(episode_transitions, reward_function):
    """
    Convert failed episodes to successful ones by changing the goal
    """
    her_transitions = []

    for transition in episode_transitions:
        # Original transition
        her_transitions.append(transition)

        # Future state as goal (for HER)
        future_state = transition['next_state']
        her_reward = reward_function(future_state, future_state)  # Success reward
        her_transition = {
            'state': transition['state'],
            'action': transition['action'],
            'reward': her_reward,
            'next_state': transition['next_state'],
            'goal': future_state
        }
        her_transitions.append(her_transition)

    return her_transitions
```

## Performance Considerations

### 1. Parallel Environments

For faster training, use multiple parallel environments:

```python
# Configuration for parallel environments
rl_config = {
    "env": {
        "numEnvs": 256,  # Number of parallel environments
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
```

### 2. GPU Acceleration

Enable GPU physics for better performance:

```python
# Enable GPU physics simulation
from omni.isaac.core import World

world = World(
    stage_units_in_meters=1.0,
    rendering_dt=1.0/60.0,
    sim_params={
        "use_gpu_pipeline": True,
        "dt": 1.0/60.0,
        "substeps": 1,
        "gravity": [0.0, 0.0, -9.81]
    }
)
```

## Troubleshooting

### Common Issues:

1. **Slow Training**:
   - Increase number of parallel environments
   - Optimize reward function computation
   - Use GPU acceleration for physics

2. **Unstable Training**:
   - Reduce learning rate
   - Adjust reward scaling
   - Implement reward clipping

3. **Robot Instability**:
   - Tune joint stiffness and damping
   - Reduce action magnitude
   - Adjust physics parameters

### Verification Steps:

1. Environment initializes without errors
2. Robot responds to actions correctly
3. Reward function provides meaningful feedback
4. Training shows improvement over time

## Next Steps

After implementing this basic manipulation example:

1. Extend to more complex manipulation tasks
2. Add multiple objects and obstacles
3. Implement more sophisticated reward shaping
4. Try different RL algorithms (SAC, TD3, etc.)
5. Transfer learned policies to real robots
``parameter>