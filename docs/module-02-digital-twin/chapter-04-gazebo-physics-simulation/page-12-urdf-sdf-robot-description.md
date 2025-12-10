# Module 2, Chapter 4, Page 12: URDF and SDF Robot Description

**Book**: Physical AI & Humanoid Robotics — A Practical Guide
**Author**: Syeda Farheen Zehra
**Module 2**: The Digital Twin (Gazebo & Unity)
**Chapter 4**: Physics Simulation in Gazebo

---

## Introduction: Two Languages, One Robot

You've learned to write URDF files to describe your robot. But when you open Gazebo, you might notice it uses something called **SDF (Simulation Description Format)** for worlds and models.

**The Question**: If URDF describes robots, and SDF also describes robots, why do we have two formats?

**The Answer**: They evolved for different purposes, but work together seamlessly:
- **URDF**: Designed for ROS robotics (focused on kinematic chains)
- **SDF**: Designed for Gazebo simulation (focused on complete scenes)

Think of them as two dialects of the same language—you can speak URDF, Gazebo translates to SDF internally, and everything works together.

---

## URDF: The ROS Robot Language

### What is URDF?

**URDF (Unified Robot Description Format)** is an **XML format** for describing:
- Robot structure (links and joints)
- Visual appearance (geometry, colors, meshes)
- Collision shapes (simplified geometry for physics)
- Inertial properties (mass, inertia)
- Sensors (via Gazebo plugins)
- Transmissions (for ROS control)

**Primary Purpose**: ROS 2 ecosystem integration

**Strengths**:
- ✅ ROS 2 native format
- ✅ Supported by all ROS 2 tools (RViz, MoveIt, robot_state_publisher)
- ✅ Simple, focused on single robots
- ✅ Large community and tutorials
- ✅ Xacro support (macros for parameterized robots)

**Limitations**:
- ❌ Assumes tree structure (one parent per link—no closed loops)
- ❌ Can't describe complete scenes (just single robots)
- ❌ Limited physics configuration options
- ❌ No nested models (can't compose robots from sub-robots)

**Example Use Cases**:
- Defining your humanoid robot's structure
- Creating custom robot arms, grippers
- Loading robots into RViz for visualization
- Using with MoveIt for motion planning

---

## SDF: The Gazebo Simulation Language

### What is SDF?

**SDF (Simulation Description Format)** is an **XML format** for describing:
- Complete simulation worlds (ground, sky, lighting)
- Multiple robots and objects in one scene
- Advanced physics settings (per-model, per-link)
- Nested models (robots made of sub-robots)
- Complex kinematic structures (closed loops, parallel mechanisms)
- Plugins (sensors, actuators, custom behaviors)

**Primary Purpose**: Gazebo simulation environments

**Strengths**:
- ✅ Describes entire scenes (world + multiple robots + objects)
- ✅ Advanced physics configuration (contact properties, friction models)
- ✅ Nested models (e.g., robot hand = palm + 5 finger sub-models)
- ✅ Supports closed kinematic loops (four-bar linkages, parallel robots)
- ✅ More flexible sensor definitions
- ✅ Can include multiple robots in one file

**Limitations**:
- ❌ Not natively supported by ROS 2 tools (needs conversion)
- ❌ More complex syntax than URDF
- ❌ Steeper learning curve
- ❌ Less community content (most tutorials use URDF)

**Example Use Cases**:
- Creating simulation worlds (warehouse, office, outdoor terrain)
- Defining complex multi-robot scenarios
- Advanced physics simulations (contact-rich tasks)
- Robots with parallel mechanisms or closed loops

---

## URDF vs SDF: Side-by-Side Comparison

| **Aspect** | **URDF** | **SDF** |
|------------|----------|---------|
| **Format** | XML | XML |
| **Designed For** | ROS robots | Gazebo simulation |
| **Scope** | Single robot | Complete world (many robots/objects) |
| **Structure** | Tree (no loops) | Graph (loops allowed) |
| **ROS 2 Tools** | Native support (RViz, MoveIt) | Requires conversion |
| **Physics Detail** | Basic | Advanced (contact models, friction) |
| **Nested Models** | No | Yes (compose from sub-models) |
| **Complexity** | Simple, beginner-friendly | More complex, flexible |
| **Xacro Support** | Yes (macros, parameters) | Limited |
| **Community** | Large (ROS ecosystem) | Smaller (Gazebo-specific) |

**Which Should You Use?**

**For robot definition**: Start with **URDF**
- Works with ROS 2 tools
- Simpler syntax
- More tutorials and examples
- Gazebo can load URDF directly (converts internally)

**For world building**: Use **SDF**
- Define environments (buildings, terrain, obstacles)
- Place multiple robots in one scene
- Advanced physics scenarios

**Best Practice**: **Use URDF for robots, SDF for worlds.** Gazebo handles the translation automatically.

---

## How Gazebo Uses URDF and SDF

### The Automatic Conversion Process

When you load a URDF robot into Gazebo:

```
1. You provide URDF file (your robot description)
   ↓
2. Gazebo reads URDF
   ↓
3. Gazebo converts URDF → SDF internally
   ↓
4. Gazebo spawns robot in simulation (using SDF representation)
   ↓
5. Physics engine uses SDF for simulation
```

**You never see the conversion**—it happens behind the scenes.

**Why Convert?**
- URDF is ROS-friendly (easy to create)
- SDF is Gazebo-friendly (more simulation features)
- Conversion gives you best of both worlds

### What Gets Converted?

**URDF Elements → SDF Elements**:

| **URDF** | **SDF Equivalent** | **Notes** |
|----------|-------------------|-----------|
| `<link>` | `<link>` | Almost identical |
| `<joint>` | `<joint>` | Same structure |
| `<visual>` | `<visual>` | Same geometry |
| `<collision>` | `<collision>` | Same shapes |
| `<inertial>` | `<inertial>` | Same mass/inertia |
| `<gazebo>` tags | Native SDF features | Plugin configs, physics |
| `<transmission>` | ROS 2 control plugin | Converted to SDF plugin |

**Result**: Your URDF robot works in Gazebo without modification.

---

## Example: Simple Humanoid Arm in URDF and SDF

Let's see the same robot arm in both formats.

### URDF Version (What You Write)

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.0033" iyy="0.0033" izz="0.0033"
               ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Upper Arm Link -->
  <link name="upper_arm">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.30"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.3 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.30"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.0083" iyy="0.0083" izz="0.00045"
               ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Shoulder Joint -->
  <joint name="shoulder_pan" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="50.0" velocity="1.0"/>
  </joint>

  <!-- Gazebo-specific: Add friction and damping -->
  <gazebo reference="shoulder_pan">
    <implicitSpringDamper>true</implicitSpringDamper>
  </gazebo>

</robot>
```

**Key Features**:
- Clean, simple syntax
- Familiar from previous chapters
- Works in RViz and Gazebo
- Xacro-compatible for parameterization

### SDF Version (What Gazebo Uses Internally)

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="simple_arm">

    <!-- Base Link -->
    <link name="base_link">
      <pose>0 0 0 0 0 0</pose>

      <visual name="base_visual">
        <geometry>
          <box>
            <size>0.1 0.1 0.1</size>
          </box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1.0</ambient>
          <diffuse>0.5 0.5 0.5 1.0</diffuse>
        </material>
      </visual>

      <collision name="base_collision">
        <geometry>
          <box>
            <size>0.1 0.1 0.1</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>  <!-- Friction coefficient -->
              <mu2>1.0</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>1000000.0</kp>  <!-- Contact stiffness -->
              <kd>100.0</kd>      <!-- Contact damping -->
            </ode>
          </contact>
        </surface>
      </collision>

      <inertial>
        <mass>2.0</mass>
        <inertia>
          <ixx>0.0033</ixx>
          <iyy>0.0033</iyy>
          <izz>0.0033</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>
    </link>

    <!-- Upper Arm Link -->
    <link name="upper_arm">
      <pose>0 0 0.1 0 0 0</pose>  <!-- Relative to world -->

      <visual name="upper_arm_visual">
        <pose>0 0 0.15 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.03</radius>
            <length>0.30</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.0 0.3 0.8 1.0</ambient>
          <diffuse>0.0 0.3 0.8 1.0</diffuse>
        </material>
      </visual>

      <collision name="upper_arm_collision">
        <pose>0 0 0.15 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.04</radius>
            <length>0.30</length>
          </cylinder>
        </geometry>
      </collision>

      <inertial>
        <pose>0 0 0.15 0 0 0</pose>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.0083</ixx>
          <iyy>0.0083</iyy>
          <izz>0.00045</izz>
        </inertia>
      </inertial>
    </link>

    <!-- Shoulder Joint -->
    <joint name="shoulder_pan" type="revolute">
      <parent>base_link</parent>
      <child>upper_arm</child>
      <pose>0 0 0 0 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
          <effort>50.0</effort>
          <velocity>1.0</velocity>
        </limit>
        <dynamics>
          <damping>0.7</damping>
          <friction>0.5</friction>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
        </dynamics>
      </axis>
      <physics>
        <ode>
          <implicit_spring_damper>true</implicit_spring_damper>
        </ode>
      </physics>
    </joint>

  </model>
</sdf>
```

**Key Differences from URDF**:
- More verbose (explicit pose, material properties)
- Advanced physics parameters (contact stiffness, damping, friction)
- Surface properties per collision shape
- More control over simulation behavior

**Important**: You rarely write SDF by hand for robots—you write URDF and let Gazebo convert.

---

## When to Use SDF: Building Complete Worlds

While URDF is best for robots, SDF shines for creating simulation environments.

### Example: Office Environment (SDF World)

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="office">

    <!-- Physics Settings -->
    <physics type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <gravity>0 0 -9.81</gravity>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1.0</diffuse>
      <direction>0.3 0.3 -1.0</direction>
    </light>

    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Office Desk (nested model) -->
    <include>
      <uri>model://desk</uri>
      <pose>2.0 0 0 0 0 0</pose>  <!-- 2m forward -->
    </include>

    <!-- Chair -->
    <include>
      <uri>model://chair</uri>
      <pose>1.5 0.5 0 0 0 -1.57</pose>  <!-- Facing desk -->
    </include>

    <!-- Coffee Cup (on desk) -->
    <include>
      <uri>model://coffee_cup</uri>
      <pose>2.0 0.3 0.8 0 0 0</pose>  <!-- On desk surface -->
    </include>

    <!-- Your Humanoid Robot -->
    <include>
      <uri>model://my_humanoid</uri>
      <pose>0 0 0 0 0 0</pose>  <!-- At origin -->
    </include>

  </world>
</sdf>
```

**What This Creates**:
- Complete office scene
- Multiple objects (desk, chair, cup)
- Your humanoid robot
- Physics and lighting configured
- All in one file

**This is SDF's Power**: Describe entire simulation scenarios, not just individual robots.

---

## Benefits of SDF for Complex Environments

### 1. Multi-Robot Scenarios

**URDF Limitation**: Can only describe one robot per file

**SDF Solution**: Include multiple robots in one world

```xml
<world name="warehouse">
  <!-- Robot 1: Forklift -->
  <include>
    <uri>model://forklift_robot</uri>
    <pose>0 0 0 0 0 0</pose>
  </include>

  <!-- Robot 2: Inspection Drone -->
  <include>
    <uri>model://quadcopter</uri>
    <pose>5 5 2 0 0 0</pose>
  </include>

  <!-- Robot 3: Humanoid -->
  <include>
    <uri>model://my_humanoid</uri>
    <pose>-3 2 0 0 0 1.57</pose>
  </include>
</world>
```

**Use Case**: Test robot coordination, multi-agent planning, swarm behavior

### 2. Nested Models (Composability)

**SDF Allows**: Building complex robots from simpler sub-models

**Example: Humanoid Hand**
```xml
<model name="humanoid_hand">
  <!-- Palm (base) -->
  <link name="palm">...</link>

  <!-- Thumb (nested model) -->
  <include>
    <uri>model://finger_3dof</uri>
    <name>thumb</name>
    <pose>0.05 0 0 0 0 0</pose>
  </include>

  <!-- Index Finger (nested model) -->
  <include>
    <uri>model://finger_3dof</uri>
    <name>index_finger</name>
    <pose>0.08 0.02 0 0 0 0</pose>
  </include>

  <!-- Middle, Ring, Pinky... (same pattern) -->
</model>
```

**Benefits**:
- Define finger once, reuse 5 times
- Easier maintenance (update finger → all 5 update)
- Modular design (swap finger types)

### 3. Advanced Contact Physics

**SDF Provides**: Fine-grained control over contact behavior

```xml
<collision name="foot_collision">
  <geometry>
    <box>
      <size>0.2 0.1 0.05</size>
    </box>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.5</mu>   <!-- High friction for walking -->
        <mu2>1.5</mu2>
      </ode>
    </friction>
    <contact>
      <ode>
        <kp>10000000</kp>  <!-- Very stiff (hard contact) -->
        <kd>1000</kd>      <!-- High damping (no bounce) -->
        <max_vel>0.01</max_vel>  <!-- Prevent penetration -->
        <min_depth>0.001</min_depth>
      </ode>
    </contact>
    <bounce>
      <restitution_coefficient>0.0</restitution_coefficient>
      <!-- No bounce when foot hits ground -->
    </bounce>
  </surface>
</collision>
```

**Use Case**: Realistic walking simulation (foot-ground contact critical for stability)

### 4. Environmental Effects

**SDF Supports**: Wind, magnetic fields, custom gravity regions

```xml
<world name="windy_environment">
  <!-- Global wind plugin -->
  <plugin name="wind_plugin" filename="libgazebo_ros_wind.so">
    <wind_force>5.0</wind_force>  <!-- 5 N/s wind force -->
    <wind_direction>1 0 0</wind_direction>  <!-- Blowing in +X -->
  </plugin>

  <!-- Region with different gravity (water pool) -->
  <model name="water_pool">
    <pose>10 0 -0.5 0 0 0</pose>
    <plugin name="buoyancy" filename="libgazebo_ros_buoyancy.so">
      <fluid_density>1000</fluid_density>  <!-- Water density -->
    </plugin>
  </model>
</world>
```

**Use Case**: Testing humanoid stability in wind, underwater robots

---

## Practical Workflow: URDF + SDF Together

**Step 1: Design Robot in URDF**
- Create robot structure (links, joints)
- Add visual meshes, collision shapes
- Define inertial properties
- Include Gazebo plugins for sensors

**Step 2: Create World in SDF**
- Define environment (ground, lighting, physics)
- Place obstacles, furniture, terrain
- Configure advanced physics if needed

**Step 3: Spawn Robot in World**
```bash
# Launch Gazebo with your world
ros2 launch gazebo_ros gazebo.launch.py world:=office.world

# Spawn robot from URDF
ros2 run gazebo_ros spawn_entity.py \
  -file my_humanoid.urdf \
  -entity my_humanoid \
  -x 0 -y 0 -z 0.5
```

**Result**: Your URDF robot appears in the SDF world, ready for testing.

---

## Key Concepts Summary

**URDF**:
- ROS 2 native format for robot description
- Simple, beginner-friendly syntax
- Supported by all ROS 2 tools (RViz, MoveIt)
- Best for: Single robot definitions
- Limitation: Tree structure only (no loops)

**SDF**:
- Gazebo native format for simulation scenes
- Advanced physics and contact configuration
- Supports nested models and closed loops
- Best for: Complete worlds, multi-robot scenarios
- Limitation: Not natively ROS 2 compatible

**URDF vs SDF**:
- Different purposes, complementary strengths
- URDF for robots, SDF for worlds
- Gazebo converts URDF → SDF automatically
- You rarely write SDF for robots by hand

**When to Use Each**:
- **Robot structure**: URDF (works everywhere)
- **Simulation world**: SDF (multiple objects, advanced physics)
- **Best practice**: Combine both (URDF robots in SDF worlds)

**Conversion**:
- Gazebo reads URDF, converts to SDF internally
- Conversion is automatic and transparent
- You get benefits of both formats

---

## What's Next

Now that you understand URDF and SDF, you're ready to spawn your robot in Gazebo and start testing.

**Next Topics**:
- **Page 13**: Installing Gazebo and ROS 2 Integration
- **Page 14**: Spawning Your URDF Robot in Gazebo
- **Page 15**: Configuring Physics for Humanoid Simulation
- **Page 16**: Testing Sensors in Gazebo

**The Journey**:
- ✅ Wrote complete URDF robot
- ✅ Understood Gazebo architecture
- ✅ **Learned URDF vs SDF (when to use each)**
- 🔜 Install Gazebo
- 🔜 Load robot and test in simulation

**You're Building Toward**: A seamless workflow where you design robots in URDF (ROS-compatible), simulate in Gazebo (using SDF internally), and deploy to real hardware—all using the same robot description.

---

*"URDF and SDF are not rivals—they're partners. Use URDF to describe your robot, SDF to build your world, and let Gazebo bridge them seamlessly."*
