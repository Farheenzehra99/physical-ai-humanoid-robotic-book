# Gazebo_Sim Skill

## Purpose
Provides comprehensive Gazebo physics simulation capabilities for robotic systems including world creation, physics configuration, sensor simulation, and collision detection. This skill enables high-fidelity testing of robots in virtual environments before hardware deployment.

## Core Capabilities
- World file creation and environment design
- Physics engine configuration (ODE, Bullet, Simbody, DART)
- Collision detection and contact dynamics
- Sensor simulation (cameras, LiDAR, IMU, force/torque)
- Plugin integration (model, world, sensor, visual)
- Lighting and visual effects
- Real-time factor optimization
- Gazebo-ROS2 bridge integration

## Pipeline

### 1. World Setup
```xml
<world name="humanoid_world">
  <physics type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
  </physics>
  <gravity>0 0 -9.81</gravity>
</world>
```

### 2. Model Integration
- Import URDF/SDF robot models
- Configure model plugins
- Set initial pose and state
- Define collision properties

### 3. Sensor Configuration
- Camera: RGB, depth, segmentation
- LiDAR: ray-based range sensing
- IMU: acceleration and angular velocity
- Contact sensors: collision detection
- Force/Torque sensors: interaction forces

### 4. Physics Tuning
- Timestep and solver settings
- Contact parameters (stiffness, damping)
- Friction coefficients
- Restitution (bounciness)
- Constraint forces

## Key Functions

### `create_world(name, physics_engine, gravity, time_step)`
Creates a Gazebo world with specified physics configuration.

### `spawn_model(model_path, name, pose, reference_frame)`
Spawns a robot or object model in the simulation.

### `add_sensor(sensor_type, parent_link, properties)`
Attaches a sensor to a robot link with specified properties.

### `configure_physics(solver, iterations, contact_params)`
Sets physics solver parameters and contact dynamics.

### `set_model_state(model_name, pose, twist)`
Sets position, orientation, and velocities of a model.

## Examples

### Example 1: Spawn Humanoid Robot
```python
from gazebo_msgs.srv import SpawnEntity
import rclpy

spawn_client = node.create_client(SpawnEntity, '/spawn_entity')
request = SpawnEntity.Request()
request.name = 'my_humanoid'
request.xml = open('humanoid.urdf', 'r').read()
request.initial_pose.position.z = 1.0
spawn_client.call_async(request)
```

### Example 2: Camera Sensor in SDF
```xml
<sensor name="camera" type="camera">
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.396</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so"/>
</sensor>
```

### Example 3: Physics Tuning for Stable Walking
```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

### Example 4: Ground Plane with Friction
```xml
<model name="ground_plane">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
        </plane>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>1.0</mu>
            <mu2>1.0</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
  </link>
</model>
```

## Dependencies
- Gazebo Classic (11.x) or Gazebo Sim (Ignition)
- gazebo_ros_pkgs
- ROS2 Humble/Iron
- SDF format specification
- Physics engines (ODE, Bullet, etc.)

## Best Practices
- Start with small timesteps for stability (0.001s)
- Use appropriate physics engine for use case
- Configure contact parameters for realistic interaction
- Optimize sensor update rates for performance
- Use static models for immovable objects
- Test physics with simple scenarios first
- Monitor real-time factor during simulation
- Use collision-only geometries for hidden objects
- Implement proper sensor noise models
- Save world states for reproducibility

## Common Configuration Patterns

### Stable Bipedal Contact
- High solver iterations (50+)
- Small timestep (0.001s or less)
- Low CFM (0.0), moderate ERP (0.2)
- Appropriate ground friction (0.8-1.2)

### Fast Simulation
- Larger timestep (0.005s)
- Fewer solver iterations (20)
- Reduced sensor rates
- Simplified collision geometries

### High-Fidelity Sensors
- Match real sensor specs
- Add realistic noise
- Set appropriate update rates
- Configure FOV and range accurately

## Troubleshooting

### Robot Falling Through Ground
- Check collision geometries exist
- Verify physics timestep is small enough
- Increase solver iterations
- Check for NaN in joint controllers

### Unstable Walking/Jittering
- Reduce timestep
- Increase solver iterations
- Adjust contact ERP/CFM
- Check joint damping values

### Slow Simulation
- Increase timestep (if stable)
- Reduce sensor rates
- Simplify collision meshes
- Use faster physics engine

## Integration Points
- Imports URDF from URDF_Designer
- Controlled via ROS2_Core topics/services
- Provides ground truth for VLA_Controller training
- Bridges to Unity_Vis for visualization
- Validates before Hardware_Proxy deployment
- Connects to IsaacSim_Pipeline for advanced RL
