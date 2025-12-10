# Module 2, Chapter 5, Page 14: Unity Scene Setup Basics

**Book**: Physical AI & Humanoid Robotics — A Practical Guide
**Author**: Syeda Farheen Zehra
**Module 2**: The Digital Twin (Gazebo & Unity)
**Chapter 5**: Unity for High-Fidelity Rendering

---

## Introduction: Building Your First Robot Scene

You've learned **why** Unity is valuable for robotics. Now let's build something—a simple scene where your robot lives, moves, and interacts with a photorealistic environment.

**What You'll Create**:
- A realistic indoor environment (office or lab)
- Your humanoid robot imported from URDF
- Proper lighting (natural + artificial)
- Physics-enabled objects to interact with
- Camera controls to view the scene

Think of this as **stage design** for your robot—setting up the environment, props, and lighting before the performance begins.

---

## Unity Interface Overview

Before we build, let's understand Unity's workspace:

```
┌──────────────────────────────────────────────────────────────┐
│  Unity Editor                                   [▶ Play]      │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌─────────────────────────┐  ┌─────────────────────────┐   │
│  │   Hierarchy             │  │   Scene View            │   │
│  │   (Object Tree)         │  │   (3D Viewport)         │   │
│  │                         │  │                         │   │
│  │  ▼ SampleScene          │  │   [3D visualization     │   │
│  │    • Main Camera        │  │    of your scene]       │   │
│  │    • Directional Light  │  │                         │   │
│  │    • Ground             │  │                         │   │
│  │    • Robot              │  │   Tools: Move, Rotate,  │   │
│  │    • Table              │  │          Scale          │   │
│  └─────────────────────────┘  └─────────────────────────┘   │
│                                                              │
│  ┌─────────────────────────┐  ┌─────────────────────────┐   │
│  │   Project               │  │   Inspector             │   │
│  │   (Assets)              │  │   (Properties)          │   │
│  │                         │  │                         │   │
│  │  📁 Assets              │  │  Selected: Robot        │   │
│  │    📁 Models            │  │                         │   │
│  │    📁 Materials         │  │  Transform:             │   │
│  │    📁 Scripts           │  │    Position (0, 0, 0)   │   │
│  │    📁 Scenes            │  │    Rotation (0, 0, 0)   │   │
│  └─────────────────────────┘  │    Scale (1, 1, 1)      │   │
│                                │                         │   │
│                                │  Add Component [+]      │   │
│                                └─────────────────────────┘   │
└──────────────────────────────────────────────────────────────┘
```

**Key Panels**:
- **Hierarchy**: Tree of all objects in your scene (like a family tree)
- **Scene View**: 3D viewport where you build and arrange objects
- **Project**: File browser for all your assets (models, textures, scripts)
- **Inspector**: Properties panel for selected object (position, materials, components)
- **Play Button**: Test your scene (run simulation)

---

## Step 1: Create a New Scene

### Starting Fresh

1. **Open Unity Hub** → Create New Project
   - Template: **3D (URP)** (Universal Render Pipeline—better graphics)
   - Name: `RoboticsSimulation`
   - Location: Choose where to save

2. **Unity opens** with default scene containing:
   - Main Camera (your viewpoint)
   - Directional Light (like the sun)

3. **Save Scene**: File → Save As → `Scenes/RobotLab.unity`

**What You Have Now**: Empty 3D world with basic lighting and a camera.

---

## Step 2: Add Ground and Environment

### Create the Floor

**Method 1: Simple Plane**

1. **Hierarchy** → Right-click → 3D Object → **Plane**
2. **Rename**: "Ground"
3. **Inspector** → Transform:
   - Position: `(0, 0, 0)`
   - Scale: `(5, 1, 5)` — 50m × 50m floor

**Method 2: Realistic Floor (Better)**

1. **Project** → Right-click → Create → **Material**
   - Name: `ConcreteFloor`
2. **Inspector** → Albedo (Color): Click color box
   - Import texture (e.g., concrete.jpg) OR use gray color `#CCCCCC`
3. **Drag material** onto Ground plane in Scene View

**Result**: Gray concrete floor (looks more realistic than default white)

### Add Walls (Optional for Indoor Scene)

1. **Create Cube**: Hierarchy → 3D Object → Cube
2. **Scale to wall**:
   - Name: `Wall_North`
   - Position: `(0, 2.5, 25)`
   - Scale: `(50, 5, 0.5)` — Long thin wall, 5m tall
3. **Duplicate** for other walls:
   - Wall_South: Position `(0, 2.5, -25)`
   - Wall_East: Position `(25, 2.5, 0)`, Scale `(0.5, 5, 50)`
   - Wall_West: Position `(-25, 2.5, 0)`, Scale `(0.5, 5, 50)`

**Result**: Enclosed room (50m × 50m × 5m tall)

---

## Step 3: Import Your Robot Model

### From URDF to Unity

**Prerequisites**:
- Install **URDF Importer** package (Unity Package Manager)
- Have your robot URDF file ready (from Module 1)

**Import Process**:

1. **Prepare URDF**:
   ```
   RoboticsSimulation/
   ├── Assets/
   │   └── Robots/
   │       └── my_humanoid/
   │           ├── urdf/
   │           │   └── robot.urdf
   │           └── meshes/
   │               ├── base.stl
   │               ├── upper_arm.stl
   │               └── forearm.stl
   ```

2. **Import in Unity**:
   - Assets → Right-click → Import New Asset
   - Select `robot.urdf`
   - Unity converts URDF → GameObject hierarchy

3. **Inspect Robot**:
   - Hierarchy shows: Robot → Links (base_link, upper_arm, forearm, etc.)
   - Each link has mesh renderer, collider

4. **Position Robot**:
   - Select root robot object
   - Inspector → Transform → Position: `(0, 0.5, 0)` (0.5m above ground)

**What You Have**: Your URDF robot standing in the scene!

### Add Materials to Robot

**Make robot look better than default gray**:

1. **Create Material**: Project → Create → Material → `RobotBody_Blue`
2. **Inspector** → Albedo: Blue color `#0066CC`
3. **Metallic**: `0.5` (semi-metallic look)
4. **Smoothness**: `0.7` (polished surface)
5. **Drag material** onto robot's body parts in Scene View

**Result**: Shiny blue robot instead of flat gray.

---

## Step 4: Configure Lighting

Lighting transforms your scene from "3D model viewer" to "photorealistic environment."

### Directional Light (Sunlight)

**Already exists** in default scene, but let's configure:

1. **Hierarchy** → Select `Directional Light`
2. **Inspector** → Light component:
   - **Intensity**: `1.0` (brightness)
   - **Color**: Slightly warm `#FFFFEE` (warm white)
   - **Rotation**: `(50, -30, 0)` (sun angle—morning light)
   - **Shadows**: Soft Shadows (realistic shadows)

**Effect**: Sunlight coming from upper-left, casting soft shadows.

### Add Indoor Lights (Point Lights)

**For realistic indoor environment**:

1. **Create Point Light**: Hierarchy → Light → Point Light
2. **Configure**:
   - Name: `CeilingLight_1`
   - Position: `(10, 4, 10)` (4m high, ceiling level)
   - Range: `15` (lights 15m radius)
   - Intensity: `2.0` (bright indoor light)
   - Color: Cool white `#F0F8FF`
3. **Duplicate** for multiple ceiling lights:
   - Ctrl+D (duplicate), move to new position
   - Create grid pattern (every 10m)

**Result**: Natural sunlight + artificial indoor lighting (like real office).

### Ambient Light (Global Fill)

**Prevents pitch-black shadows**:

1. **Window** → Rendering → Lighting
2. **Environment** tab:
   - **Skybox**: Default (blue sky)
   - **Ambient Color**: Light gray `#AAAAAA`
   - **Ambient Intensity**: `0.5`

**Effect**: Soft fill light (no pure black areas, looks natural).

---

## Step 5: Add Physics

Unity uses **NVIDIA PhysX** for physics simulation.

### Enable Physics on Ground

1. **Select Ground plane**
2. **Inspector** → Add Component → **Box Collider**
   - Automatically sized to plane
   - Check: `Is Trigger` = OFF (solid collision)

**Result**: Objects can land on ground (won't fall through).

### Configure Robot Physics

**URDF Importer auto-adds physics**, but verify:

1. **Select robot's base_link**
2. **Inspector** → Components:
   - **Rigidbody**: Controls physics (mass, gravity)
     - Mass: `30` (kg) for humanoid torso
     - Use Gravity: ✅ ON
     - Is Kinematic: ❌ OFF (affected by forces)
   - **Collider**: Defines collision shape
     - Usually Mesh Collider or Capsule Collider

3. **Joints** (if robot has movable parts):
   - Articulation Body (for complex kinematic chains)
   - Configure joint limits, damping

**Test Physics**:
- Press **Play** button
- Robot should stand on ground (not fall through)
- If falls over, adjust center of mass or add balance script

### Add Interactive Objects

**Create a box to interact with**:

1. **Hierarchy** → 3D Object → Cube
2. **Configure**:
   - Name: `Box_Red`
   - Position: `(2, 0.5, 0)` (on ground, near robot)
   - Scale: `(0.3, 0.3, 0.3)` (30cm cube)
3. **Add Physics**:
   - Inspector → Add Component → **Rigidbody**
     - Mass: `1.0` (kg)
     - Use Gravity: ✅
4. **Add Material**: Red material for visibility

**Press Play**: Box falls onto ground due to gravity. Robot can potentially push it!

---

## Step 6: Camera Setup and Navigation

### Main Camera Configuration

1. **Select Main Camera** in Hierarchy
2. **Inspector** → Transform:
   - Position: `(-5, 3, -5)` (behind and above robot)
   - Rotation: `(20, 45, 0)` (looking at robot from angle)
3. **Camera Component**:
   - Field of View: `60` (natural perspective)
   - Clipping Planes:
     - Near: `0.1` (objects closer than 10cm invisible)
     - Far: `1000` (objects farther than 1km invisible)
   - Clear Flags: Skybox (shows sky background)

**Result**: Angled view of robot in scene.

### Add Camera Control Script (Fly Camera)

**Enable WASD movement + mouse look**:

1. **Select Main Camera**
2. **Inspector** → Add Component → Search: `Fly Camera` (or create script)
3. **Create Script** (if needed):
   ```csharp
   using UnityEngine;

   public class FlyCamera : MonoBehaviour
   {
       public float moveSpeed = 5f;
       public float lookSpeed = 2f;

       void Update()
       {
           // WASD movement
           float h = Input.GetAxis("Horizontal"); // A/D
           float v = Input.GetAxis("Vertical");   // W/S
           transform.Translate(h * moveSpeed * Time.deltaTime,
                               0,
                               v * moveSpeed * Time.deltaTime);

           // Mouse look (right-click + drag)
           if (Input.GetMouseButton(1))
           {
               float mouseX = Input.GetAxis("Mouse X") * lookSpeed;
               float mouseY = Input.GetAxis("Mouse Y") * lookSpeed;
               transform.Rotate(-mouseY, mouseX, 0);
           }
       }
   }
   ```

4. **Attach Script**: Drag script onto Main Camera

**Press Play**:
- WASD = Move camera
- Right-click + drag mouse = Look around
- Navigate scene like first-person game

### Multiple Camera Views (Optional)

**Add top-down camera for overview**:

1. **Duplicate Main Camera**: Ctrl+D
2. **Rename**: `Camera_TopDown`
3. **Position**: `(0, 20, 0)` (20m above)
4. **Rotation**: `(90, 0, 0)` (looking straight down)
5. **Disable** initially (checkbox in Inspector)

**Switch cameras** with script or manually enable/disable.

---

## Step 7: Testing Your Scene

### The First Test Run

1. **Save Scene**: Ctrl+S
2. **Press Play** (▶ button at top)

**What Should Happen**:
- Robot stands on ground
- Physics active (box falls if elevated)
- Lights illuminate scene
- Camera controls work (WASD + mouse)

**Common Issues**:

| **Problem** | **Solution** |
|-------------|-------------|
| Robot falls through floor | Add Box Collider to ground |
| Robot tips over | Adjust center of mass or add balance |
| Scene too dark | Increase light intensities |
| Camera won't move | Check script attached and enabled |
| Physics too slow | Edit → Project Settings → Time → Fixed Timestep = 0.02 |

### Visual Quality Check

**Make scene look better**:

1. **Enable Shadows**: All lights → Shadows = Soft Shadows
2. **Anti-Aliasing**: Camera → Anti-Aliasing = 4x MSAA
3. **Post-Processing**:
   - Add Post-Process Volume
   - Enable Ambient Occlusion (shadows in corners)
   - Enable Bloom (bright lights glow slightly)

**Result**: Scene looks polished, not like early 2000s game.

---

## The Basic Scene Structure

Here's what your final Hierarchy should look like:

```
SampleScene
├── Main Camera (FlyCamera script)
├── Directional Light (sun, shadows enabled)
├── Environment
│   ├── Ground (Plane with ConcreteFloor material, Box Collider)
│   ├── Wall_North (Cube, scaled)
│   ├── Wall_South
│   ├── Wall_East
│   └── Wall_West
├── Lighting
│   ├── CeilingLight_1 (Point Light, 4m high)
│   ├── CeilingLight_2
│   └── CeilingLight_3
├── Robot (imported from URDF)
│   ├── base_link (Rigidbody, Collider)
│   ├── upper_arm
│   ├── forearm
│   └── hand
└── Interactive Objects
    ├── Box_Red (Cube, Rigidbody, 1kg)
    └── Table (optional, from Asset Store)
```

**Scene Characteristics**:
- **Size**: 50m × 50m room
- **Lighting**: Natural (directional) + artificial (point lights)
- **Physics**: Enabled on ground, robot, objects
- **Camera**: Fly navigation (WASD + mouse)
- **Materials**: Concrete floor, colored robot, red box

---

## Adding Assets from Unity Asset Store

**For richer environments**, use free assets:

### Find Assets

1. **Window** → Asset Store (or browser: assetstore.unity.com)
2. **Search**: "office furniture" or "laboratory equipment"
3. **Download** free packs:
   - "Office Props Pack" (desks, chairs, computers)
   - "Industrial Equipment" (shelves, crates)
   - "Modular Sci-Fi Lab" (futuristic environment)

### Import to Scene

1. **Package Manager** → My Assets → Download
2. **Import** into project
3. **Drag models** from Project → into Scene View
4. **Position** using Move tool (W key)

**Result**: Professional-looking environment without 3D modeling.

---

## Basic Navigation and Control

### Manual Robot Control (Proof of Concept)

**Simple keyboard control script**:

```csharp
using UnityEngine;

public class SimpleRobotControl : MonoBehaviour
{
    public ArticulationBody shoulder;
    public float rotationSpeed = 30f; // degrees/sec

    void Update()
    {
        // Arrow keys rotate shoulder joint
        if (Input.GetKey(KeyCode.UpArrow))
        {
            RotateJoint(shoulder, rotationSpeed * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.DownArrow))
        {
            RotateJoint(shoulder, -rotationSpeed * Time.deltaTime);
        }
    }

    void RotateJoint(ArticulationBody joint, float angle)
    {
        var drive = joint.xDrive;
        drive.target += angle;
        joint.xDrive = drive;
    }
}
```

**Attach to robot**:
1. Select robot root
2. Add Component → SimpleRobotControl
3. Drag shoulder joint into `shoulder` field in Inspector

**Press Play**: Arrow keys move robot's shoulder joint!

**Note**: This is manual control. Later chapters cover ROS 2 integration for autonomous control.

---

## Key Concepts Summary

**Unity Scene Setup**:
- **Hierarchy**: Tree of all scene objects
- **Scene View**: 3D workspace for building
- **Inspector**: Object properties and components
- **Project**: Asset library (models, materials, scripts)

**Building a Scene**:
1. **Ground**: Plane with collider (physics-enabled floor)
2. **Robot**: Import URDF, add materials
3. **Lighting**: Directional (sun) + Point (indoor) + Ambient
4. **Physics**: Rigidbody + Colliders on interactive objects
5. **Camera**: Position, configure, add fly controls

**Lighting for Realism**:
- Directional Light: Outdoor sunlight (shadows)
- Point Lights: Indoor ceiling lights (localized)
- Ambient Light: Soft fill (no pure black)

**Physics Configuration**:
- Ground: Box Collider (solid surface)
- Robot: Rigidbody + Articulation Body (joints)
- Objects: Rigidbody (affected by gravity, forces)

**Camera Navigation**:
- Fly Camera script: WASD movement + mouse look
- Multiple cameras: Switch views (top-down, isometric)

**Assets**:
- Unity Asset Store: Free environments, props
- Import via Package Manager
- Drag into scene, position with Move tool

---

## What's Next

You've built a basic Unity scene with a robot, environment, lighting, and physics. Next, you'll connect it to ROS 2 for real robotics integration.

**Next Topics**:
- **Page 15**: Unity Robotics Hub (ROS 2 Integration)
- **Page 16**: Connecting Unity to Gazebo (Dual Simulation)
- **Page 17**: Advanced Lighting and Materials
- **Page 18**: Generating Synthetic Training Data

**The Journey**:
- ✅ Understood why Unity matters (visualization, HRI)
- ✅ **Created basic Unity scene** (environment, robot, lighting, physics)
- 🔜 Connect to ROS 2 (publish/subscribe topics)
- 🔜 Synchronize with Gazebo (physics + graphics)
- 🔜 Generate synthetic datasets (CV training)

**You're Building Toward**: A complete Unity-ROS 2 pipeline where your robot's state (from Gazebo or real hardware) visualizes beautifully in Unity, and Unity generates realistic sensor data back to ROS 2.

---

*"A well-lit scene is the difference between a tech demo and a product showcase. Unity gives your robot the stage it deserves."*
