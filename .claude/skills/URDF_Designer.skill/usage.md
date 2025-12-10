# URDF_Designer Usage Guide

## When to Invoke This Skill

Invoke URDF_Designer skill when you need to:

### Primary Use Cases
1. **Create Robot Models**: Design URDF files for new humanoid robots
2. **Modify Robot Structure**: Add/remove links or joints
3. **Define Joint Limits**: Set position, velocity, and effort constraints
4. **Configure Inertial Properties**: Calculate and set mass/inertia
5. **Add Sensors**: Integrate cameras, LiDAR, IMU into robot model
6. **Convert to Xacro**: Create parametric robot descriptions
7. **Validate Robot Models**: Check URDF syntax and structure

### Specific Triggers
- User asks to "create a URDF" or "design robot model"
- User wants to "add a joint" or "modify kinematic chain"
- User needs to "set joint limits" or "configure joint properties"
- User mentions "humanoid structure", "arm", "leg", "hand"
- User asks about "link geometry" or "collision shapes"
- User needs "inertia calculation" or "mass properties"
- User wants to "validate URDF" or "check robot description"
- User asks to "convert to Xacro" or "parametrize robot"

### Context Indicators
- Task involves robot mechanical structure
- Need to define kinematic relationships
- Working with robot visualization or simulation
- Preparing robot model for ROS2/Gazebo
- Setting up motion planning configuration

## When NOT to Invoke This Skill

Do not invoke when:
- User only wants to control existing robot (use ROS2_Core)
- Task is about simulation physics, not model definition (use Gazebo_Sim)
- Focus is on AI/ML training without model modification (use IsaacSim_Pipeline)
- Task is about robot code deployment (use Edge_Deploy)
- User only needs visualization without model changes (use Unity_Vis)

## Skill Activation Examples

### Example 1: Direct Request
```
User: "Create a URDF for a humanoid robot with 6-DOF arms"
→ Activate URDF_Designer skill
```

### Example 2: Modification Request
```
User: "Add a gripper to the robot's wrist joint"
→ Activate URDF_Designer skill (new links + joints)
```

### Example 3: Constraint Request
```
User: "Set the hip joint limits to prevent over-extension"
→ Activate URDF_Designer skill (joint limit configuration)
```

### Example 4: Validation Request
```
User: "Check if my URDF file is valid"
→ Activate URDF_Designer skill (validation function)
```

## Workflow Integration

### Standalone Usage
URDF_Designer can be used independently for robot model creation and validation.

### Combined with Other Skills
- **URDF_Designer + ROS2_Core**: Load and publish robot state
- **URDF_Designer + Gazebo_Sim**: Import model into physics simulation
- **URDF_Designer + Unity_Vis**: Visualize robot structure in Unity
- **URDF_Designer + Hardware_Proxy**: Map URDF joints to real actuators
- **URDF_Designer + VLA_Controller**: Provide kinematic structure for AI control

## Sequential Usage Pattern
```
1. URDF_Designer → Create robot model
2. Gazebo_Sim → Test in simulation
3. ROS2_Core → Control via topics
4. Hardware_Proxy → Deploy to real robot
```

## Priority Level
**HIGH** - Essential for defining robot structure before any simulation or control.

## Expected Outputs
- Complete URDF file (.urdf or .xacro)
- Validation report
- Inertia calculations
- Coordinate frame diagrams (if requested)
- RViz visualization configuration

## Common Humanoid Configurations

### Full Humanoid (30+ DOF)
- 2 legs × 6 DOF = 12 DOF
- 2 arms × 7 DOF = 14 DOF
- Torso: 3 DOF
- Head: 2 DOF
- Hands: 2 × 5 DOF = 10 DOF

### Simplified Humanoid (12-18 DOF)
- 2 legs × 5 DOF = 10 DOF
- 2 arms × 3 DOF = 6 DOF
- Head: 2 DOF

### Upper Body Only (10-15 DOF)
- 2 arms × 5-7 DOF
- Torso: 2-3 DOF
- Head: 2 DOF

## Validation Checklist
- [ ] All links have unique names
- [ ] All joints have unique names
- [ ] Parent-child relationships are valid
- [ ] No kinematic loops (unless intentional)
- [ ] Joint limits are physically reasonable
- [ ] Inertial properties are set for all links
- [ ] Origin transforms are correctly specified
- [ ] URDF passes `check_urdf` validation
- [ ] Model loads in RViz without errors
- [ ] Coordinate frames follow REP-103/REP-105 conventions
