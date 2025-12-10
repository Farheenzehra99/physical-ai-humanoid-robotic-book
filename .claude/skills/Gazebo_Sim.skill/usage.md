# Gazebo_Sim Usage Guide

## When to Invoke This Skill

Invoke Gazebo_Sim skill when you need to:

### Primary Use Cases
1. **Create Simulation Worlds**: Design environments for robot testing
2. **Physics Simulation**: Test robot dynamics and control
3. **Sensor Simulation**: Validate sensor configurations and processing
4. **Collision Testing**: Verify robot collision behavior
5. **Environment Interaction**: Test grasping, manipulation, navigation
6. **Physics Tuning**: Optimize simulation parameters for stability
7. **Plugin Integration**: Add custom Gazebo functionality

### Specific Triggers
- User asks to "simulate in Gazebo" or "test in simulation"
- User wants to "spawn a robot" or "create a world"
- User needs to "add sensors" or "configure camera"
- User mentions "physics parameters" or "collision detection"
- User asks about "ground contact" or "friction"
- User needs to "test walking" or "validate control"
- User wants to "add obstacles" or "create environment"
- User asks about "timestep", "solver", or "real-time factor"

### Context Indicators
- Robot needs physics-based testing
- Sensor validation required before hardware
- Multiple environmental conditions to test
- Control algorithms need validation
- Cost/safety concerns with real hardware testing
- Rapid iteration needed

## When NOT to Invoke This Skill

Do not invoke when:
- User only wants robot model creation (use URDF_Designer)
- Task is about ROS2 communication without simulation (use ROS2_Core)
- User wants GPU-accelerated RL training (use IsaacSim_Pipeline)
- Focus is on visualization only, not physics (use Unity_Vis)
- Task is about real hardware deployment (use Hardware_Proxy)
- User wants photorealistic rendering (use IsaacSim_Pipeline)

## Skill Activation Examples

### Example 1: Direct Request
```
User: "Simulate my humanoid robot walking in Gazebo"
→ Activate Gazebo_Sim skill
```

### Example 2: Physics Request
```
User: "The robot is falling through the ground, help me fix the physics"
→ Activate Gazebo_Sim skill (physics configuration)
```

### Example 3: Sensor Request
```
User: "Add a depth camera to the robot's head in simulation"
→ Activate Gazebo_Sim skill (sensor integration)
```

### Example 4: Environment Request
```
User: "Create a world with stairs and obstacles"
→ Activate Gazebo_Sim skill (world creation)
```

## Workflow Integration

### Standalone Usage
Gazebo_Sim can be used independently for physics simulation and testing.

### Combined with Other Skills
- **URDF_Designer + Gazebo_Sim**: Import robot model into simulation
- **Gazebo_Sim + ROS2_Core**: Control simulated robot via topics
- **Gazebo_Sim + VLA_Controller**: Test AI control in simulation
- **Gazebo_Sim + Hardware_Proxy**: Validate before hardware deployment
- **Gazebo_Sim + Unity_Vis**: Dual visualization (physics + aesthetics)

## Sequential Usage Pattern
```
1. URDF_Designer → Create robot model
2. Gazebo_Sim → Set up simulation world
3. ROS2_Core → Implement control
4. Gazebo_Sim → Test and tune physics
5. Hardware_Proxy → Deploy to real robot
```

## Priority Level
**HIGH** - Critical for validating robot control and dynamics before hardware deployment.

## Expected Outputs
- Gazebo world files (.world, .sdf)
- Launch files for simulation startup
- Plugin configurations
- Physics parameter recommendations
- Sensor configuration XML
- Debugging/tuning guidance

## Common Simulation Scenarios

### Indoor Navigation
- Flat floor with walls
- Furniture obstacles
- Doorways and corridors
- Appropriate lighting

### Outdoor Terrain
- Uneven ground with heightmap
- Slope variations
- Natural obstacles (rocks, vegetation)
- Skybox and sun

### Manipulation Tasks
- Table and objects
- Grasping targets
- Contact-rich interactions
- Force feedback

### Stair Climbing
- Standard stair dimensions
- Railings and landings
- Multiple stair configurations
- Varied step heights

## Performance Optimization Guide

### For Real-Time Performance
- Timestep: 0.005s
- Solver iterations: 20-30
- Simplified collision meshes
- Sensor rate: 10-30 Hz
- Disable shadows if not needed

### For Accuracy
- Timestep: 0.001s or smaller
- Solver iterations: 50+
- Detailed collision meshes
- Sensor rate: 30+ Hz
- High-quality contact dynamics

## Validation Checklist
- [ ] Robot spawns at correct pose
- [ ] All joints move within limits
- [ ] Collision detection works correctly
- [ ] Sensors publish expected data
- [ ] Physics is stable (no jittering/falling)
- [ ] Real-time factor is acceptable (>0.5)
- [ ] Ground contact has appropriate friction
- [ ] Robot responds correctly to controls
- [ ] No warning/error messages in console
- [ ] Simulation is reproducible

## Gazebo vs Isaac Sim Decision Matrix

Use **Gazebo_Sim** when:
- Standard ROS2 integration needed
- CPU-based simulation sufficient
- Open-source requirement
- Traditional robotics workflow
- Single robot testing

Use **IsaacSim_Pipeline** when:
- GPU acceleration needed
- Photorealistic rendering required
- Large-scale RL training
- Many parallel environments
- Domain randomization needed
