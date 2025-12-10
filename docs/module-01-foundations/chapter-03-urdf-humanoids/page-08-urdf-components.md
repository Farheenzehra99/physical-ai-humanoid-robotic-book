# Module 1, Chapter 3, Page 8: URDF Components

**Book**: Physical AI & Humanoid Robotics — A Practical Guide
**Author**: Syeda Farheen Zehra
**Module 1**: Foundations of Physical AI
**Chapter 3**: Understanding URDF for Humanoids

---

## Introduction: The Building Blocks

You've learned that URDF is your robot's skeleton blueprint. Now let's examine the **building blocks** in detail—the four main components that make up every URDF file:

1. **Links** — The rigid body parts
2. **Joints** — The connections that enable movement
3. **Sensors** — The perception systems
4. **Transmissions** — The connection between controllers and joints

Think of building a URDF like constructing with LEGO blocks. Each component has a specific purpose, and when assembled correctly, they create a functional robot description.

---

## Component 1: Links (The Robot's Body Parts)

### What is a Link?

A **link** is a **rigid body part** of your robot—a piece that doesn't bend or deform. Each link has three aspects:

1. **Visual** — What it looks like (for visualization in RViz)
2. **Collision** — Simplified shape for collision detection (for physics simulation)
3. **Inertial** — Mass and inertia properties (for dynamics simulation)

### Link Structure in URDF

```xml
<link name="forearm">

  <!-- VISUAL: Appearance in RViz/Gazebo -->
  <visual>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.04" length="0.30"/>
    </geometry>
    <material name="gray">
      <color rgba="0.7 0.7 0.7 1.0"/>
    </material>
  </visual>

  <!-- COLLISION: Simplified shape for physics -->
  <collision>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.05" length="0.30"/>
      <!-- Slightly larger than visual for safety margin -->
    </geometry>
  </collision>

  <!-- INERTIAL: Mass and inertia for dynamics -->
  <inertial>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <mass value="1.5"/>  <!-- 1.5 kg -->
    <inertia
      ixx="0.012" ixy="0.0" ixz="0.0"
      iyy="0.012" iyz="0.0"
      izz="0.001"/>
  </inertial>

</link>
```

### Breaking Down the Link Components

#### Visual Geometry

**Purpose**: Defines how the link appears in visualization tools.

**Common Geometry Types**:

| **Geometry** | **Use Case** | **Example** |
|--------------|--------------|-------------|
| `<box>` | Rectangular parts | Torso, base_link |
| `<cylinder>` | Round parts | Arms, legs, fingers |
| `<sphere>` | Ball joints, heads | Shoulder joints, robot head |
| `<mesh>` | Complex shapes | Custom 3D models (STL, DAE, OBJ) |

**Example — Box Geometry**:
```xml
<geometry>
  <box size="0.3 0.2 0.5"/>  <!-- width, depth, height in meters -->
</geometry>
```

**Example — Mesh Geometry**:
```xml
<geometry>
  <mesh filename="package://my_robot/meshes/forearm.stl" scale="1 1 1"/>
</geometry>
```

**Material Colors**:
```xml
<material name="blue">
  <color rgba="0.0 0.0 1.0 1.0"/>  <!-- Red, Green, Blue, Alpha -->
</material>
```

#### Collision Geometry

**Purpose**: Simplified shape for fast collision detection in physics engines.

**Best Practices**:
- ✅ Use **primitive shapes** when possible (box, cylinder, sphere)
- ✅ Make collision geometry **slightly larger** than visual (safety margin)
- ❌ Don't use high-poly meshes for collision (too slow)

**Why Separate Visual and Collision?**
- Visual geometry can be detailed (complex meshes)
- Collision geometry must be fast (simple primitives)
- Physics engines check millions of collision tests per second

**Example**: A detailed hand mesh (visual) vs. a simple box (collision).

#### Inertial Properties

**Purpose**: Defines how the link behaves under forces (for realistic physics).

**Key Parameters**:

**Mass** (`<mass value="..."/>`):
- Weight of the link in kilograms
- Example: Forearm = 1.5 kg

**Inertia Tensor** (`<inertia .../>`):
- Resistance to rotational acceleration
- 3×3 symmetric matrix (6 unique values: ixx, iyy, izz, ixy, ixz, iyz)
- **Most users**: Use automatic calculation tools or rough estimates

**Example Inertia Values**:
```xml
<!-- Thin rod rotating around center -->
<inertia ixx="0.01" iyy="0.01" izz="0.0001"
         ixy="0" ixz="0" iyz="0"/>
```

**Tools to Calculate Inertia**:
- CAD software (SolidWorks, Fusion 360) exports inertia
- MeshLab can estimate from 3D models
- Online calculators for simple shapes

---

## Component 2: Joints (The Connections)

### What is a Joint?

A **joint** connects two links and defines **how they move relative to each other**. Every joint has:
- A **parent link** (the base)
- A **child link** (the moving part)
- A **joint type** (how it moves)
- **Limits** (range, speed, force)

### Joint Structure in URDF

```xml
<joint name="elbow" type="revolute">

  <!-- Parent-child relationship -->
  <parent link="upper_arm"/>
  <child link="forearm"/>

  <!-- Attachment point -->
  <origin xyz="0 0 -0.30" rpy="0 0 0"/>
  <!-- Child attached 0.30m below parent's origin -->

  <!-- Rotation axis -->
  <axis xyz="0 1 0"/>  <!-- Rotates around Y-axis -->

  <!-- Motion limits -->
  <limit
    lower="0.0"           <!-- Min angle: 0 radians (0°) -->
    upper="2.618"         <!-- Max angle: 2.618 rad (150°) -->
    effort="50.0"         <!-- Max torque: 50 Newton-meters -->
    velocity="2.0"/>      <!-- Max speed: 2.0 rad/s -->

  <!-- Damping and friction (optional) -->
  <dynamics damping="0.7" friction="0.5"/>

</joint>
```

### Joint Types

#### 1. Revolute (Most Common in Humanoids)

**Motion**: Rotation around a single axis with limits

**Examples**: Elbow, knee, shoulder pitch, wrist yaw

**Visualization**:
```
Parent Link (upper_arm)
       │
       │  Axis: Y
       ○────→  (revolute joint)
       │
       │
Child Link (forearm)
     (rotates 0° to 150°)
```

**Key Parameters**:
- `lower`, `upper`: Angle limits (radians)
- `effort`: Maximum torque the joint can apply
- `velocity`: Maximum rotational speed

#### 2. Continuous

**Motion**: Unlimited rotation around an axis (no limits)

**Examples**: Wheels, spinning sensors, turrets

```xml
<joint name="wheel_joint" type="continuous">
  <parent link="chassis"/>
  <child link="wheel"/>
  <axis xyz="0 1 0"/>
  <limit effort="10.0" velocity="20.0"/>
  <!-- No lower/upper limits -->
</joint>
```

**Use Case**: A robot with wheels that can spin indefinitely.

#### 3. Prismatic

**Motion**: Linear sliding along an axis

**Examples**: Elevator mechanisms, telescoping antennas, linear actuators

```xml
<joint name="elevator" type="prismatic">
  <parent link="base"/>
  <child link="platform"/>
  <axis xyz="0 0 1"/>  <!-- Slides along Z-axis (up/down) -->
  <limit
    lower="0.0"       <!-- Min extension: 0m -->
    upper="0.5"       <!-- Max extension: 0.5m -->
    effort="100.0"
    velocity="0.2"/>
</joint>
```

**Visualization**:
```
        ↑
        │ (platform slides up/down)
        │
    ┌───┴───┐
    │       │  Platform (child)
    └───────┘
        │
        │ prismatic joint (Z-axis)
        │
    ┌───┴───┐
    │       │  Base (parent)
    └───────┘
```

#### 4. Fixed

**Motion**: No movement (permanently attached)

**Examples**: Camera attached to head, sensor mounted on chest, gripper base to wrist

```xml
<joint name="camera_mount" type="fixed">
  <parent link="head"/>
  <child link="camera"/>
  <origin xyz="0.08 0 0.05" rpy="0 0 0"/>
  <!-- Camera fixed 8cm forward, 5cm up from head -->
</joint>
```

**Use Case**: Sensors that don't move independently.

#### 5. Planar (Rare)

**Motion**: Sliding in a 2D plane (X-Y)

**Examples**: Air hockey puck, sliding doors

#### 6. Floating (Rare)

**Motion**: Free movement in 3D space (6 DOF: 3 translation + 3 rotation)

**Examples**: Simulating a robot before it touches ground, underwater robots

### Joint Types Summary

| **Type** | **DOF** | **Motion** | **Limits?** | **Common Use** |
|----------|---------|------------|-------------|----------------|
| Revolute | 1 | Rotation (axis) | Yes | Elbows, knees |
| Continuous | 1 | Rotation (axis) | No | Wheels |
| Prismatic | 1 | Linear (axis) | Yes | Elevators |
| Fixed | 0 | None | N/A | Sensors |
| Planar | 2 | 2D translation | Yes | Sliding doors |
| Floating | 6 | Full 3D freedom | No | Base link |

---

## Component 3: Sensors (The Perception)

### What are Sensors in URDF?

Sensors are defined as **links** (for geometry) plus **Gazebo plugins** (for simulation behavior). They capture data from the simulated environment and publish to ROS 2 topics.

### Common Sensor Types

#### 1. Camera (RGB)

**Purpose**: Captures color images

**URDF Structure**:
```xml
<!-- Camera link (geometry) -->
<link name="camera">
  <visual>
    <geometry>
      <box size="0.02 0.06 0.02"/>  <!-- Small camera box -->
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
</link>

<!-- Attach camera to parent -->
<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera"/>
  <origin xyz="0.08 0 0.05" rpy="0 0 0"/>
</joint>

<!-- Gazebo plugin for simulation -->
<gazebo reference="camera">
  <sensor type="camera" name="camera_sensor">
    <update_rate>30.0</update_rate>  <!-- 30 fps -->
    <camera>
      <horizontal_fov>1.57</horizontal_fov>  <!-- 90° field of view -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>   <!-- Min distance: 10cm -->
        <far>100.0</far>   <!-- Max distance: 100m -->
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <argument>~/image_raw:=camera/image</argument>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

**Publishes To**: `/robot/camera/image` (sensor_msgs/Image)

#### 2. Depth Camera (RGB-D)

**Purpose**: Captures color + depth (distance) information

**Example**: Intel RealSense, Kinect, Stereolabs ZED

**Additional Parameters**:
```xml
<camera>
  <horizontal_fov>1.57</horizontal_fov>
  <image>
    <width>640</width>
    <height>480</height>
  </image>
  <!-- Depth-specific settings -->
  <depth_camera>
    <min_depth>0.1</min_depth>
    <max_depth>10.0</max_depth>
  </depth_camera>
</camera>
```

**Publishes To**:
- `/camera/image_raw` (RGB)
- `/camera/depth/image_raw` (depth map)
- `/camera/points` (3D point cloud)

#### 3. LIDAR (Laser Scanner)

**Purpose**: Measures distances using laser beams (2D or 3D)

```xml
<gazebo reference="lidar">
  <sensor type="ray" name="lidar_sensor">
    <pose>0 0 0 0 0 0</pose>
    <update_rate>10</update_rate>  <!-- 10 Hz -->
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>       <!-- 360 beams -->
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>  <!-- -180° -->
          <max_angle>3.14159</max_angle>   <!-- +180° -->
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>   <!-- Min distance: 10cm -->
        <max>30.0</max>  <!-- Max distance: 30m -->
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <argument>~/out:=scan</argument>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

**Publishes To**: `/scan` (sensor_msgs/LaserScan)

#### 4. IMU (Inertial Measurement Unit)

**Purpose**: Measures acceleration and angular velocity (for balance, orientation)

```xml
<gazebo reference="torso">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100.0</update_rate>  <!-- 100 Hz -->
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </x>
        <!-- Y and Z similar -->
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.1</stddev>
          </noise>
        </x>
        <!-- Y and Z similar -->
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <argument>~/out:=imu/data</argument>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

**Publishes To**: `/robot/imu/data` (sensor_msgs/Imu)

**Data Includes**:
- Linear acceleration (m/s²)
- Angular velocity (rad/s)
- Orientation (quaternion)

#### 5. Force/Torque Sensors

**Purpose**: Measures forces and torques (for contact detection, force control)

**Common Locations**: Feet (ground contact), wrists (manipulation forces)

```xml
<gazebo>
  <plugin name="ft_sensor" filename="libgazebo_ros_ft_sensor.so">
    <update_rate>100.0</update_rate>
    <joint_name>left_ankle</joint_name>
    <ros>
      <namespace>/robot</namespace>
      <argument>~/out:=left_foot/force</argument>
    </ros>
  </plugin>
</gazebo>
```

**Publishes To**: `/robot/left_foot/force` (geometry_msgs/WrenchStamped)

---

## Component 4: Transmissions (Controller Interface)

### What are Transmissions?

**Transmissions** define the relationship between **actuators** (motors) and **joints**. They tell ROS 2 controllers:
- Which joints can be controlled
- What type of interface to use (position, velocity, effort)
- Hardware interface mappings

**Analogy**: Think of transmissions as the **wiring diagram** connecting your robot's brain (controllers) to its muscles (motors).

### Why Transmissions Matter

Without transmissions:
- Controllers don't know which joints are actuated
- No connection between ROS 2 commands and simulated motors
- You can't control the robot

With transmissions:
- Controllers can command joint positions, velocities, or efforts
- Hardware and simulation use the same interface
- Smooth transition from simulation to real robot

### Transmission Structure

```xml
<transmission name="elbow_transmission">

  <!-- Joint this transmission controls -->
  <joint name="elbow">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>

  <!-- Actuator (motor) driving the joint -->
  <actuator name="elbow_motor">
    <mechanicalReduction>1</mechanicalReduction>  <!-- Gear ratio -->
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </actuator>

  <!-- Transmission type -->
  <type>transmission_interface/SimpleTransmission</type>

</transmission>
```

### Hardware Interface Types

| **Interface** | **Controls** | **Use Case** |
|---------------|--------------|--------------|
| PositionJointInterface | Joint angle (radians) | Precise positioning (arms, legs) |
| VelocityJointInterface | Joint speed (rad/s) | Continuous motion (wheels) |
| EffortJointInterface | Joint torque (Nm) | Force control (compliant grasping) |

### Transmission Types

**1. SimpleTransmission** (Most Common)
- One actuator drives one joint
- Direct 1:1 or geared connection
- Example: Elbow motor → elbow joint

**2. DifferentialTransmission**
- Two actuators drive two coupled joints
- Example: Differential drive (left/right wheels)

**3. FourBarLinkageTransmission**
- Complex mechanical linkages
- Rare in humanoids

### Complete Transmission Example

```xml
<!-- Shoulder pitch joint transmission -->
<transmission name="left_shoulder_pitch_transmission">
  <type>transmission_interface/SimpleTransmission</type>

  <joint name="left_shoulder_pitch">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>

  <actuator name="left_shoulder_pitch_motor">
    <mechanicalReduction>50</mechanicalReduction>  <!-- 50:1 gear reduction -->
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </actuator>
</transmission>
```

**What This Means**:
- Controller sends torque commands (EffortJointInterface)
- 50:1 gear reduction (motor spins 50 times → joint moves 1 time)
- Higher torque, lower speed (typical for shoulder joints)

---

## How Components Work Together

Let's see how all four components create a functional robot arm:

```
URDF Structure for 3-DOF Robot Arm:

┌─────────────────────────────────────────────────┐
│  LINK: base_link (torso)                        │
│    • Visual: Box (0.2 × 0.2 × 0.4m)             │
│    • Collision: Box (same)                      │
│    • Inertial: Mass 5kg                         │
└────────────────┬────────────────────────────────┘
                 │
        JOINT: shoulder_pitch (revolute)
                 │ Range: -90° to +90°
                 │ Axis: Y
                 ▼
┌─────────────────────────────────────────────────┐
│  LINK: upper_arm                                │
│    • Visual: Cylinder (r=0.04, l=0.30)          │
│    • Collision: Cylinder (r=0.05, l=0.30)       │
│    • Inertial: Mass 1.2kg                       │
└────────────────┬────────────────────────────────┘
                 │
        JOINT: elbow (revolute)
                 │ Range: 0° to 150°
                 │ Axis: Y
                 ▼
┌─────────────────────────────────────────────────┐
│  LINK: forearm                                  │
│    • Visual: Cylinder (r=0.04, l=0.25)          │
│    • Collision: Cylinder (r=0.05, l=0.25)       │
│    • Inertial: Mass 1.0kg                       │
└────────────────┬────────────────────────────────┘
                 │
        JOINT: wrist_pitch (revolute)
                 │ Range: -90° to +90°
                 │ Axis: Y
                 ▼
┌─────────────────────────────────────────────────┐
│  LINK: hand                                     │
│    • Visual: Box (0.08 × 0.05 × 0.12)           │
│    • Collision: Box (same)                      │
│    • Inertial: Mass 0.3kg                       │
└────────────────┬────────────────────────────────┘
                 │
        JOINT: camera_mount (fixed)
                 │
                 ▼
┌─────────────────────────────────────────────────┐
│  SENSOR: wrist_camera                           │
│    • Type: RGB Camera                           │
│    • Resolution: 640×480                        │
│    • FPS: 30                                    │
│    • Publishes: /camera/image                   │
└─────────────────────────────────────────────────┘

TRANSMISSIONS:
• shoulder_pitch_transmission → shoulder_pitch joint
• elbow_transmission → elbow joint
• wrist_pitch_transmission → wrist_pitch joint
```

**Data Flow**:
1. **Links** define physical structure
2. **Joints** connect links and enable motion
3. **Sensors** perceive environment and publish data
4. **Transmissions** connect controllers to actuated joints
5. **Controllers** command joint positions/velocities/efforts
6. **Simulation** updates physics and sensor data

---

## Key Concepts Summary

**Links** (Rigid Body Parts):
- Define visual, collision, and inertial properties
- Geometries: box, cylinder, sphere, mesh
- Each link has mass and inertia

**Joints** (Connections):
- Connect parent and child links
- Types: revolute (most common), continuous, prismatic, fixed
- Define axis, limits, effort, velocity
- Create kinematic chains

**Sensors** (Perception):
- Defined as links + Gazebo plugins
- Types: camera, depth camera, LIDAR, IMU, force/torque
- Publish data to ROS 2 topics
- Configurable: resolution, FPS, range, noise

**Transmissions** (Controller Interface):
- Connect controllers to joints
- Hardware interfaces: position, velocity, effort
- Define gear ratios (mechanical reduction)
- Enable ROS 2 control framework

**Together**: These four components create a complete, controllable, perceiving robot description that works in simulation and on real hardware.

---

## What's Next

You now understand the **four building blocks** of URDF. Next, you'll learn how to write these components by hand and assemble them into a complete humanoid robot description.

**Next Topics**:
- **Page 9**: Writing Your First Complete URDF (hands-on robot arm)
- **Chapter 4**: Visualizing URDF in RViz
- **Chapter 5**: Simulating URDF in Gazebo with physics

**The Journey**:
- ✅ Understood what URDF is (robot skeleton blueprint)
- ✅ **Learned URDF components** (links, joints, sensors, transmissions)
- 🔜 Write complete URDF files
- 🔜 See robots come alive in simulation

---

*"Components are the vocabulary. Structure is the grammar. Together, they let you speak the language of robot description."*
