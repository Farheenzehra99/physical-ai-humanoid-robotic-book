# URDF_Designer Skill

## Purpose
Provides expertise in creating, modifying, and validating URDF (Unified Robot Description Format) files for humanoid robots. This skill enables precise robot kinematic chain definition, joint limits, collision geometries, and visual representations for simulation and control.

## Core Capabilities
- URDF structure design and validation
- Kinematic chain configuration (serial and parallel)
- Joint definition (revolute, prismatic, fixed, continuous)
- Link properties (mass, inertia, visual, collision)
- Material and color specification
- Sensor integration (cameras, LiDAR, IMU)
- URDF to SRDF conversion for motion planning
- Xacro macros for parametric robot design

## Pipeline

### 1. Robot Structure Planning
- Define kinematic tree hierarchy
- Identify base link and end effectors
- Plan joint placement and types
- Calculate link dimensions

### 2. Link Definition
```xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.5 0.3 0.2"/>
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.5 0.3 0.2"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="10.0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0"
             iyy="0.2" iyz="0.0" izz="0.15"/>
  </inertial>
</link>
```

### 3. Joint Configuration
- Set joint limits (position, velocity, effort)
- Configure joint dynamics (damping, friction)
- Define parent-child relationships
- Specify origin transforms

### 4. Validation and Testing
- Check for closed kinematic loops
- Validate joint limits
- Verify coordinate frames
- Test in RVIZ/Gazebo

## Key Functions

### `create_link(name, visual_geometry, collision_geometry, mass, inertia)`
Creates a URDF link with complete properties.

### `create_joint(name, type, parent, child, origin, limits)`
Defines a joint connecting two links with specified constraints.

### `calculate_inertia(mass, geometry_type, dimensions)`
Computes inertial properties for common geometric shapes.

### `validate_urdf(file_path)`
Checks URDF syntax and structural validity.

### `urdf_to_xacro(urdf_file, output_file)`
Converts static URDF to parametric Xacro format.

## Examples

### Example 1: Humanoid Leg Joint
```xml
<joint name="hip_pitch" type="revolute">
  <parent link="pelvis"/>
  <child link="thigh_left"/>
  <origin xyz="0.0 0.1 -0.05" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57"
         effort="100" velocity="5.0"/>
  <dynamics damping="0.7" friction="0.0"/>
</joint>
```

### Example 2: Robot Hand with Fingers
```xml
<link name="finger_proximal">
  <visual>
    <geometry>
      <cylinder radius="0.01" length="0.04"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.01" length="0.04"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.02"/>
    <inertia ixx="0.00001" ixy="0" ixz="0"
             iyy="0.00001" iyz="0" izz="0.00001"/>
  </inertial>
</link>
```

### Example 3: Camera Sensor Integration
```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.02 0.05 0.02"/>
    </geometry>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.03" rpy="0 0 0"/>
</joint>
```

## Dependencies
- urdf_parser_py
- xacro
- RViz (visualization)
- check_urdf (validation tool)
- Python 3.8+

## Best Practices
- Use SI units (meters, radians, kg)
- Define collision geometry simpler than visual
- Calculate accurate inertial properties
- Use Xacro for complex/repeated structures
- Validate with `check_urdf` before deployment
- Test joint limits in simulation
- Add meaningful link/joint names
- Document coordinate frame conventions

## Common Humanoid Structures

### Bipedal Leg Chain (per leg)
1. Hip Yaw → Hip Roll → Hip Pitch
2. Knee Pitch
3. Ankle Pitch → Ankle Roll

### Arm Chain (per arm)
1. Shoulder Pitch → Shoulder Roll → Shoulder Yaw
2. Elbow Pitch
3. Wrist Yaw → Wrist Pitch → Wrist Roll

### Torso
1. Base → Waist Yaw → Waist Pitch
2. Neck Yaw → Neck Pitch

## Integration Points
- Loads into ROS2_Core for robot state publishing
- Imported by Gazebo_Sim for physics simulation
- Used by Unity_Vis for visualization
- Referenced by Hardware_Proxy for real robot mapping
- Provides kinematic structure to VLA_Controller
