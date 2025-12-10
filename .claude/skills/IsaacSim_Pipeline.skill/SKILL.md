# IsaacSim_Pipeline Skill

## Purpose
Provides NVIDIA Isaac Sim capabilities for photorealistic simulation, large-scale reinforcement learning, synthetic data generation, and GPU-accelerated physics. This skill enables advanced AI training workflows with domain randomization and parallel environment execution.

## Core Capabilities
- Photorealistic ray-traced rendering (RTX)
- GPU-accelerated physics (PhysX 5)
- Massive parallel environment simulation
- Synthetic data generation with annotations
- Domain randomization for sim-to-real transfer
- Isaac Gym integration for RL training
- ROS2 and Python API integration
- Procedural environment generation
- Real-time sensor simulation (cameras, LiDAR, depth)

## Pipeline

### 1. Environment Setup
```python
from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "width": 1280,
    "height": 720
})

from omni.isaac.core import World
world = World(stage_units_in_meters=1.0)
```

### 2. Robot Loading
```python
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage

# Load robot from URDF/USD
robot_path = "/path/to/robot.usd"
robot_prim_path = "/World/Robot"
add_reference_to_stage(robot_path, robot_prim_path)

robot = Robot(prim_path=robot_prim_path, name="humanoid")
world.scene.add(robot)
```

### 3. RL Training Setup
```python
from omni.isaac.gym.vec_env import VecEnvBase

class HumanoidTask(VecEnvBase):
    def __init__(self, cfg, sim_device, graphics_device, headless):
        self.cfg = cfg
        self.num_envs = cfg["env"]["numEnvs"]
        self.max_episode_length = cfg["env"]["episodeLength"]
        super().__init__(cfg, sim_device, graphics_device, headless)

    def set_up_scene(self, scene):
        # Create parallel environments
        for i in range(self.num_envs):
            robot_path = f"/World/Env_{i}/Robot"
            # Add robot, ground, and props
            pass

    def compute_reward(self):
        # Define reward function
        pass
```

### 4. Synthetic Data Generation
```python
from omni.replicator.core import BasicWriter
import omni.replicator.core as rep

# Set up camera and annotations
camera = rep.create.camera(position=(2, 2, 2))
render_product = rep.create.render_product(camera, (512, 512))

writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir="./output", rgb=True,
                 bounding_box_2d_tight=True, semantic_segmentation=True)
writer.attach([render_product])
```

## Key Functions

### `create_parallel_envs(num_envs, robot_config, task_config)`
Creates multiple parallel simulation environments for RL training.

### `setup_domain_randomization(params)`
Configures randomization of physics, appearance, and geometry.

### `generate_synthetic_dataset(config, num_samples)`
Generates annotated synthetic data for supervised learning.

### `configure_gpu_physics(device_id, params)`
Sets GPU physics solver parameters for performance and accuracy.

### `export_to_ros2(topics_config)`
Bridges Isaac Sim with ROS2 for robot control integration.

## Examples

### Example 1: Parallel Environment Training
```python
import torch
from omni.isaac.gym.vec_env import VecEnvBase

class WalkingTask(VecEnvBase):
    def __init__(self, cfg, *args, **kwargs):
        self.num_obs = 48  # joint states, IMU, etc.
        self.num_actions = 12  # joint commands
        super().__init__(cfg, *args, **kwargs)

    def get_observations(self):
        # Get joint positions, velocities, base orientation
        joint_pos = self.robot.get_joint_positions()
        joint_vel = self.robot.get_joint_velocities()
        base_quat = self.robot.get_world_pose()[1]

        obs = torch.cat([joint_pos, joint_vel, base_quat], dim=-1)
        return {"obs": obs}

    def pre_physics_step(self, actions):
        # Apply actions as joint position targets
        self.robot.set_joint_position_targets(actions)

    def post_physics_step(self):
        # Compute rewards
        base_pos = self.robot.get_world_pose()[0]
        forward_reward = base_pos[:, 0]  # reward for moving forward
        alive_reward = torch.ones_like(forward_reward)

        self.rew_buf = forward_reward + alive_reward - 0.01 * actions.square().sum(dim=-1)

        # Check termination
        self.reset_buf = torch.where(base_pos[:, 2] < 0.3,
                                     torch.ones_like(self.reset_buf),
                                     self.reset_buf)
```

### Example 2: Domain Randomization
```python
import omni.replicator.core as rep

with rep.new_layer():
    # Randomize lighting
    def randomize_lighting():
        lights = rep.get.light()
        with lights:
            rep.modify.attribute("intensity", rep.distribution.uniform(500, 3000))
            rep.modify.attribute("color", rep.distribution.uniform((0.8, 0.8, 0.8), (1, 1, 1)))
        return lights.node

    # Randomize materials
    def randomize_materials():
        materials = rep.get.prims(semantics=[("class", "floor")])
        with materials:
            rep.randomizer.color(colors=rep.distribution.uniform((0.1, 0.1, 0.1), (0.9, 0.9, 0.9)))
        return materials.node

    # Register and trigger
    rep.randomizer.register(randomize_lighting)
    rep.randomizer.register(randomize_materials)

    with rep.trigger.on_frame(num_frames=100):
        rep.randomizer.randomize_lighting()
        rep.randomizer.randomize_materials()
```

### Example 3: Synthetic Data Collection
```python
import omni.replicator.core as rep

# Create camera randomization
def setup_cameras():
    camera = rep.create.camera(
        position=rep.distribution.uniform((-2, -2, 0.5), (2, 2, 2)),
        look_at=(0, 0, 0.5)
    )
    return camera

# Setup writer for multiple annotation types
render_product = rep.create.render_product(camera, (640, 480))
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="./synthetic_data",
    rgb=True,
    depth=True,
    normals=True,
    bounding_box_2d_tight=True,
    bounding_box_3d=True,
    semantic_segmentation=True,
    instance_segmentation=True,
)
writer.attach([render_product])

# Run data generation
rep.orchestrator.run()
```

### Example 4: ROS2 Bridge Setup
```python
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

# Import after enabling extension
import omni.graph.core as og

# Create ROS2 publisher graph
keys = og.Controller.Keys
(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("JointStatePublisher", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "JointStatePublisher.inputs:execIn"),
        ],
        keys.SET_VALUES: [
            ("JointStatePublisher.inputs:targetPrim", robot_prim_path),
            ("JointStatePublisher.inputs:topicName", "/joint_states"),
        ]
    }
)
```

## Dependencies
- NVIDIA Isaac Sim 2023.1+
- CUDA 11.8+
- Python 3.10
- PyTorch (for RL training)
- omni.isaac.core
- omni.isaac.gym (for RL)
- omni.replicator (for synthetic data)
- RTX-capable GPU (recommended: RTX 3090/4090, A100, or higher)

## Best Practices
- Use GPU physics for large-scale parallel training
- Start with CPU physics for debugging single environment
- Implement domain randomization for sim-to-real transfer
- Use USD format for optimized asset loading
- Monitor GPU memory usage with many parallel envs
- Use headless mode for training, GUI for debugging
- Cache compiled physics graphs for faster startup
- Leverage Omniverse nucleus for asset management
- Profile performance with built-in profiler
- Save checkpoints frequently during RL training

## Performance Optimization

### For Maximum Throughput (RL)
- Use 512-2048 parallel environments
- Enable GPU physics pipeline
- Reduce physics substeps where possible
- Simplify collision geometries
- Disable rendering in headless mode
- Use lower sensor update rates

### For Visual Quality (Synthetic Data)
- Enable ray tracing (RTX)
- Use high-resolution rendering
- Add realistic materials (MDL)
- Configure proper lighting
- Enable post-processing effects
- Use denoising for faster convergence

## Isaac Gym vs Traditional RL

### Advantages
- 1000x faster than CPU simulation
- Massive parallelization (1000+ envs)
- End-to-end GPU pipeline
- Integrated with popular RL libraries
- Photorealistic rendering available

### Use Cases
- Locomotion (walking, running, jumping)
- Manipulation (grasping, assembly)
- Multi-agent coordination
- Dexterous control
- Sim-to-real transfer

## Integration Points
- Imports URDF from URDF_Designer (converts to USD)
- Provides trained policies to VLA_Controller
- Generates datasets for edge deployment via Edge_Deploy
- Bridges to ROS2_Core for hardware testing
- Exports to Hardware_Proxy for real robot validation
- Alternative to Gazebo_Sim for GPU-accelerated workloads
