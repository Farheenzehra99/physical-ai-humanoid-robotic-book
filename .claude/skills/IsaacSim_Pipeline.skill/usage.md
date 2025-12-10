# IsaacSim_Pipeline Usage Guide

## When to Invoke This Skill

Invoke IsaacSim_Pipeline skill when you need to:

### Primary Use Cases
1. **Large-Scale RL Training**: Train policies with 100s-1000s of parallel environments
2. **Photorealistic Simulation**: High-quality ray-traced rendering
3. **Synthetic Data Generation**: Create annotated datasets for ML
4. **GPU-Accelerated Physics**: Fast, parallelized physics simulation
5. **Domain Randomization**: Sim-to-real transfer preparation
6. **Multi-Agent Training**: Coordinate multiple robots
7. **Complex Manipulation**: Dexterous grasping and assembly
8. **Sensor Simulation**: High-fidelity camera, LiDAR, depth

### Specific Triggers
- User asks for "Isaac Sim" or "Isaac Gym"
- User wants "reinforcement learning" or "RL training"
- User needs "GPU acceleration" or "parallel simulation"
- User mentions "synthetic data" or "domain randomization"
- User asks for "photorealistic" simulation
- User needs "1000+ environments" or "massive parallelization"
- User wants "sim-to-real" transfer
- User mentions "NVIDIA" simulation tools

### Context Indicators
- RL training at scale required
- GPU resources available (RTX or better)
- Need for photorealistic rendering AND physics
- Synthetic dataset generation
- Research requiring domain randomization
- Performance-critical simulation workload

## When NOT to Invoke This Skill

Do not invoke when:
- User has no NVIDIA GPU or Isaac Sim license
- Simple single-environment testing (use Gazebo_Sim)
- Only visualization needed (use Unity_Vis)
- CPU-only setup (use Gazebo_Sim)
- Basic ROS2 workflow without ML (use ROS2_Core + Gazebo_Sim)
- Quick prototyping without performance needs
- No RL/ML component in project

## Skill Activation Examples

### Example 1: Direct Request
```
User: "Train a walking policy using Isaac Gym with 1024 parallel environments"
→ Activate IsaacSim_Pipeline skill
```

### Example 2: Synthetic Data Request
```
User: "Generate 10,000 labeled images of the robot in various poses"
→ Activate IsaacSim_Pipeline skill (Replicator)
```

### Example 3: Performance Request
```
User: "The training is too slow in Gazebo, I need GPU acceleration"
→ Activate IsaacSim_Pipeline skill (GPU physics)
```

### Example 4: Sim-to-Real Request
```
User: "Prepare the policy for real robot with domain randomization"
→ Activate IsaacSim_Pipeline skill (randomization)
```

## Workflow Integration

### Standalone Usage
IsaacSim_Pipeline can be used independently for RL training and data generation.

### Combined with Other Skills
- **URDF_Designer + IsaacSim_Pipeline**: Import robot model (converted to USD)
- **IsaacSim_Pipeline + VLA_Controller**: Train vision-language-action policies
- **IsaacSim_Pipeline + Edge_Deploy**: Optimize trained models for Jetson
- **IsaacSim_Pipeline + Hardware_Proxy**: Test trained policy on real robot
- **IsaacSim_Pipeline + ROS2_Core**: Bridge simulation to ROS2 ecosystem

## Sequential Usage Pattern
```
1. URDF_Designer → Create robot model
2. IsaacSim_Pipeline → Convert to USD and set up RL task
3. IsaacSim_Pipeline → Train policy with parallel environments
4. VLA_Controller → Integrate trained policy with vision
5. Edge_Deploy → Optimize for Jetson deployment
6. Hardware_Proxy → Deploy to real robot
```

## Priority Level
**HIGH** - Critical for modern GPU-accelerated RL and large-scale ML data generation.

## Expected Outputs
- USD robot asset files
- Python RL training scripts
- Trained policy checkpoints (.pth, .onnx)
- Synthetic datasets with annotations
- Domain randomization configurations
- Performance metrics and learning curves
- ROS2 bridge configurations
- Launch scripts for headless training

## Common RL Tasks

### Locomotion Tasks
- **Flat terrain walking**: Basic bipedal locomotion
- **Rough terrain**: Navigate uneven surfaces
- **Stair climbing**: Discrete height changes
- **Running**: High-speed locomotion
- **Jumping**: Dynamic maneuvers

### Manipulation Tasks
- **Object grasping**: Pick up various objects
- **Placement**: Precise object positioning
- **Assembly**: Part insertion and manipulation
- **Tool use**: Use tools to accomplish goals
- **Dexterous manipulation**: Multi-finger control

### Navigation Tasks
- **Goal reaching**: Navigate to target location
- **Obstacle avoidance**: Dynamic obstacles
- **Following**: Track moving target
- **Exploration**: Coverage and mapping

## Domain Randomization Strategies

### Visual Randomization
- Lighting: intensity, color, position
- Materials: colors, roughness, metallic
- Textures: floor, walls, objects
- Camera: position, orientation, FOV

### Physics Randomization
- Mass: ±20% variation
- Friction: 0.5-1.5 coefficient range
- Damping: joint friction variation
- Actuator: strength and response time

### Geometric Randomization
- Object sizes: scale variations
- Positions: spawn location noise
- Environment layout: obstacle placement
- Robot configuration: joint offsets

## Hardware Requirements

### Minimum (Development)
- GPU: RTX 3060 or better
- VRAM: 8 GB
- RAM: 16 GB
- Storage: 50 GB for Isaac Sim

### Recommended (Training)
- GPU: RTX 4090 or A100
- VRAM: 24 GB+
- RAM: 32 GB+
- Storage: 500 GB+ (for datasets)

### Production (Large-Scale)
- Multi-GPU: 4-8x A100 or H100
- VRAM: 80 GB per GPU
- RAM: 256 GB+
- Storage: NVMe SSD 2 TB+

## Performance Benchmarks

### RL Training Speed
- 512 envs: ~5,000-10,000 steps/sec
- 1024 envs: ~10,000-20,000 steps/sec
- 2048 envs: ~20,000-40,000 steps/sec
(On RTX 4090 / A100)

### Synthetic Data Generation
- 10-100 images/second depending on:
  - Resolution
  - Annotation types
  - Ray tracing quality
  - Scene complexity

## Validation Checklist
- [ ] Isaac Sim installed and licensed
- [ ] CUDA and drivers up to date
- [ ] Robot USD asset loads correctly
- [ ] Physics simulation is stable
- [ ] Parallel environments spawn successfully
- [ ] GPU memory usage is acceptable
- [ ] RL training loop runs without errors
- [ ] Rewards are computed correctly
- [ ] Policy checkpoints save successfully
- [ ] Domain randomization works as expected
- [ ] ROS2 bridge (if used) connects properly
- [ ] Synthetic data has correct annotations

## Isaac Sim vs Gazebo vs Unity Decision Matrix

Use **IsaacSim_Pipeline** when:
- GPU-accelerated physics needed
- Large-scale RL training (100+ envs)
- Photorealistic rendering required
- Synthetic data generation
- Domain randomization
- Performance critical (sim-to-real)

Use **Gazebo_Sim** when:
- Standard ROS2 workflow
- CPU-based simulation sufficient
- Open-source requirement
- Single environment testing
- Traditional robotics pipeline

Use **Unity_Vis** when:
- Visualization quality priority
- HRI scenarios
- VR/AR features
- Custom UI/UX
- No physics accuracy requirement

## RL Libraries Integration

### Supported Frameworks
- **Stable-Baselines3** (PPO, SAC, TD3)
- **RLlib** (Ray)
- **CleanRL**
- **Custom PyTorch implementations**

### Example Integration
```python
from stable_baselines3 import PPO
from omni.isaac.gym.vec_env import VecEnvBase

# Wrap Isaac Gym environment
env = IsaacGymEnv(task_name="Humanoid")

# Train with SB3
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=10_000_000)
model.save("humanoid_policy")
```
