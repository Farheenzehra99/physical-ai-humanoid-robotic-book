# Unity_Vis Usage Guide

## When to Invoke This Skill

Invoke Unity_Vis skill when you need to:

### Primary Use Cases
1. **High-Quality Visualization**: Photorealistic rendering of robots
2. **Human-Robot Interaction**: Interactive HRI environments
3. **VR/AR Applications**: Immersive robot visualization and control
4. **Custom UI/UX**: User interfaces for robot control and monitoring
5. **Video Production**: Recording demonstrations and presentations
6. **Interactive Environments**: User-manipulable objects and scenes
7. **Synthetic Data Generation**: Creating labeled datasets
8. **Multi-View Rendering**: Simultaneous camera perspectives

### Specific Triggers
- User asks for "Unity visualization" or "3D rendering"
- User wants "VR" or "AR" robot interaction
- User needs "photorealistic" or "high-quality" visuals
- User mentions "human-robot interaction" or "HRI"
- User wants to "create a scene" or "design environment"
- User asks for "user interface" or "control panel"
- User needs to "record video" or "capture screenshots"
- User wants "interactive objects" or "manipulable environment"
- User mentions "synthetic data" or "dataset generation"

### Context Indicators
- Presentation or demonstration requirements
- User testing and evaluation scenarios
- Marketing and visualization needs
- Education and training applications
- Research requiring HRI environments
- VR/AR project requirements

## When NOT to Invoke This Skill

Do not invoke when:
- User only needs physics simulation (use Gazebo_Sim)
- Task is about robot control without visualization (use ROS2_Core)
- Focus is on GPU-accelerated RL training (use IsaacSim_Pipeline)
- User wants command-line tools only (use ROS2_Core)
- Task doesn't require 3D visualization
- Basic RViz visualization is sufficient

## Skill Activation Examples

### Example 1: Direct Request
```
User: "Create a Unity scene showing the robot in a living room"
→ Activate Unity_Vis skill
```

### Example 2: VR Request
```
User: "I want to control the robot in VR"
→ Activate Unity_Vis skill (VR setup)
```

### Example 3: Visualization Request
```
User: "Show me a photorealistic view of the humanoid walking"
→ Activate Unity_Vis skill (rendering setup)
```

### Example 4: HRI Request
```
User: "Create an environment where humans can interact with the robot"
→ Activate Unity_Vis skill (HRI environment)
```

## Workflow Integration

### Standalone Usage
Unity_Vis can be used independently for visualization and UI development.

### Combined with Other Skills
- **URDF_Designer + Unity_Vis**: Import robot model for visualization
- **ROS2_Core + Unity_Vis**: Real-time visualization of robot state
- **Unity_Vis + VLA_Controller**: Render scenes for vision input
- **Gazebo_Sim + Unity_Vis**: Parallel physics and aesthetic visualization
- **Unity_Vis + Hardware_Proxy**: Remote monitoring interface

## Sequential Usage Pattern
```
1. URDF_Designer → Create robot model
2. Unity_Vis → Import and set up visualization
3. ROS2_Core → Connect Unity to ROS2
4. Unity_Vis → Create interactive scene
5. VLA_Controller → Use rendered scenes for AI
```

## Priority Level
**MEDIUM** - Important for user-facing applications, demos, and HRI research.

## Expected Outputs
- Unity project folder structure
- C# scripts for ROS2 integration
- Scene files (.unity)
- Prefabs for reusable objects
- Materials and shaders
- UI canvas and controls
- Build settings and export
- Documentation for scene usage

## Common Visualization Scenarios

### Robot Demonstration
- Clean, professional environment
- Good lighting setup
- Smooth camera movement
- Clear robot visibility
- Background context

### User Control Interface
- Real-time joint control sliders
- Camera view switching
- Status indicators
- Emergency stop button
- Telemetry displays

### VR Robot Operation
- First-person or third-person view
- Hand controller mappings
- Teleportation navigation
- Object manipulation
- Safety boundaries

### Data Collection
- Multiple synchronized cameras
- Segmentation rendering
- Depth map generation
- Bounding box visualization
- Automated scene randomization

## Unity vs Gazebo vs Isaac Decision Matrix

Use **Unity_Vis** when:
- High visual quality needed
- HRI scenarios required
- VR/AR features desired
- Custom UI/UX development
- Marketing/demonstration focus
- Interactive user control

Use **Gazebo_Sim** when:
- Physics accuracy priority
- Standard ROS2 workflow
- Simulation testing needed
- No rendering quality requirement

Use **IsaacSim_Pipeline** when:
- Large-scale RL training
- GPU-accelerated physics
- Domain randomization
- Photorealistic + physics both needed

## Performance Targets

### Desktop Visualization
- Target: 60+ FPS
- Resolution: 1920×1080 or higher
- Quality: High/Ultra settings

### VR Mode
- Target: 90+ FPS (critical)
- Resolution: Per-eye rendering
- Quality: Medium/High settings

### Video Recording
- Target: 30-60 FPS stable
- Resolution: 4K possible
- Quality: Ultra settings acceptable

## Validation Checklist
- [ ] Robot model imports correctly
- [ ] Joint articulations move smoothly
- [ ] ROS2 connection established
- [ ] Joint states update in real-time
- [ ] Camera controls work properly
- [ ] Lighting is appropriate
- [ ] No rendering artifacts
- [ ] Performance meets target FPS
- [ ] UI elements are responsive
- [ ] Scene is aesthetically pleasing
- [ ] VR mode (if applicable) is comfortable
- [ ] No console errors or warnings
