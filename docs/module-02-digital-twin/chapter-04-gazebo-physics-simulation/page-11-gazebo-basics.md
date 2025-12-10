# Module 2, Chapter 4, Page 11: Gazebo Basics

**Book**: Physical AI & Humanoid Robotics — A Practical Guide
**Author**: Syeda Farheen Zehra
**Module 2**: The Digital Twin (Gazebo & Unity)
**Chapter 4**: Physics Simulation in Gazebo

---

## Introduction: Your Virtual Robot Laboratory

Imagine having a physics laboratory where:
- Gravity is adjustable (Earth, Moon, Mars, or even zero-gravity)
- Time can run faster or slower than reality
- You can rewind and replay experiments
- Robots never break, no matter how many times they fall
- Sensors are perfectly calibrated (or realistically noisy)

**This is Gazebo**—a powerful, open-source robotics simulator that brings your URDF robots to life in a virtual 3D world with realistic physics.

Gazebo is to robotics what Minecraft is to architecture: a sandbox where you build, test, and iterate without real-world constraints.

---

## What is Gazebo?

**Gazebo** is a **3D robot simulator** that:
- Renders robots and environments in 3D graphics
- Simulates realistic physics (gravity, collisions, friction, forces)
- Emulates sensors (cameras, LIDAR, IMU, force/torque)
- Integrates seamlessly with ROS 2
- Runs on Linux, macOS, and Windows

**Current Version**: Gazebo (formerly known as "Gazebo Classic" or "Gazebo 11") is the stable version. **Ignition Gazebo** (now called **Gazebo Sim**) is the next-generation version with improved performance.

**In this book**, we'll use **Gazebo Classic** for compatibility with most ROS 2 tutorials, but concepts apply to both versions.

### What Gazebo Does

```
┌─────────────────────────────────────────────────────────┐
│                    GAZEBO SIMULATOR                     │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │   3D Render  │  │   Physics    │  │   Sensors    │  │
│  │   Engine     │  │   Engine     │  │  Simulation  │  │
│  │              │  │              │  │              │  │
│  │  • OpenGL    │  │  • Gravity   │  │  • Cameras   │  │
│  │  • Lighting  │  │  • Collisions│  │  • LIDAR     │  │
│  │  • Shadows   │  │  • Friction  │  │  • IMU       │  │
│  │  • Textures  │  │  • Forces    │  │  • Contact   │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
│                                                         │
│  ┌────────────────────────────────────────────────────┐ │
│  │              ROS 2 Integration                     │ │
│  │  • Topics (sensor data, commands)                  │ │
│  │  • Services (spawn/delete models)                  │ │
│  │  • TF (coordinate transforms)                      │ │
│  └────────────────────────────────────────────────────┘ │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

**Key Components**:
1. **3D Rendering**: Visualizes robots and environments
2. **Physics Simulation**: Computes motion, collisions, forces
3. **Sensor Simulation**: Generates realistic sensor data
4. **ROS 2 Integration**: Connects simulation to ROS 2 ecosystem

---

## Gazebo Environment Structure

Gazebo organizes simulations into three main concepts:

### 1. Worlds (The Environment)

A **world** defines the entire simulation environment:
- Ground plane (flat, terrain, custom surface)
- Lighting (sun, ambient light, shadows)
- Physics settings (gravity, time step, solver)
- Pre-placed objects (buildings, obstacles, furniture)

**World Files**: XML files (`.world`) defining the environment

**Example: Empty World**
```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="empty_world">

    <!-- Sun (lighting) -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Physics settings -->
    <physics type="ode">
      <gravity>0 0 -9.81</gravity>  <!-- Earth gravity -->
      <max_step_size>0.001</max_step_size>  <!-- 1ms time step -->
      <real_time_factor>1.0</real_time_factor>  <!-- Real-time -->
    </physics>

  </world>
</sdf>
```

**Common Worlds**:
- `empty.world`: Just ground and sky (blank canvas)
- `cafe.world`: Indoor café environment
- `office.world`: Office with furniture
- `warehouse.world`: Industrial warehouse
- `outdoor.world`: Hills, trees, roads

**Location**: `/usr/share/gazebo-11/worlds/` (on Linux)

### 2. Models (The Objects)

A **model** is any object in the simulation:
- Robots (humanoids, arms, mobile bases)
- Objects (chairs, tables, cups, tools)
- Obstacles (walls, boxes, barriers)
- Environmental features (trees, buildings)

**Model Structure**:
```
my_robot/
├── model.config       # Metadata (name, author, description)
├── model.sdf          # Structure (links, joints, plugins)
└── meshes/            # 3D geometry files (STL, DAE, OBJ)
    ├── base.stl
    ├── arm.dae
    └── gripper.obj
```

**Model Definition (SDF)**:
```xml
<sdf version="1.6">
  <model name="simple_box">

    <link name="box_link">
      <visual>
        <geometry>
          <box>
            <size>1 1 1</size>  <!-- 1m cube -->
          </box>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>  <!-- Red -->
        </material>
      </visual>

      <collision>
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
      </collision>

      <inertial>
        <mass>10.0</mass>  <!-- 10 kg -->
        <inertia>
          <ixx>1.67</ixx>
          <iyy>1.67</iyy>
          <izz>1.67</izz>
        </inertia>
      </inertial>
    </link>

  </model>
</sdf>
```

**Model Database**: Gazebo includes 100+ pre-built models (tables, chairs, cars, drones, etc.)

**Location**: `/usr/share/gazebo-11/models/` or downloaded on-demand

### 3. Plugins (The Behaviors)

**Plugins** add functionality to models:
- **Sensor plugins**: Camera, LIDAR, IMU (generate data)
- **Actuator plugins**: Motors, thrusters (apply forces)
- **Controller plugins**: PID controllers, trajectory followers
- **World plugins**: Custom physics, environmental effects

**Example: Camera Plugin**
```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="my_camera">
    <update_rate>30.0</update_rate>
    <camera>
      <horizontal_fov>1.57</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
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

**What This Does**:
- Attaches to `camera_link` in your robot
- Generates 640×480 images at 30 FPS
- Publishes to `/robot/camera/image` ROS topic

**Common Plugins**:
- `libgazebo_ros_camera.so`: Camera sensor
- `libgazebo_ros_ray_sensor.so`: LIDAR/ray sensor
- `libgazebo_ros_imu_sensor.so`: IMU sensor
- `libgazebo_ros_diff_drive.so`: Differential drive controller
- `libgazebo_ros_joint_state_publisher.so`: Publishes joint angles

---

## The Gazebo Architecture

### How Everything Connects

```
┌───────────────────────────────────────────────────────────┐
│                      GAZEBO PROCESS                       │
├───────────────────────────────────────────────────────────┤
│                                                           │
│  ┌─────────────────────────────────────────────────────┐  │
│  │  WORLD                                              │  │
│  │  • Ground plane                                     │  │
│  │  • Lighting (sun, shadows)                          │  │
│  │  • Physics settings (gravity, time step)            │  │
│  └─────────────────────────────────────────────────────┘  │
│                         │                                 │
│                         │ Contains                        │
│                         ▼                                 │
│  ┌─────────────────────────────────────────────────────┐  │
│  │  MODELS (Objects in the world)                      │  │
│  │                                                      │  │
│  │  ┌──────────────┐  ┌──────────────┐  ┌────────────┐ │  │
│  │  │  Robot       │  │  Table       │  │  Box       │ │  │
│  │  │              │  │              │  │            │ │  │
│  │  │  • Links     │  │  • Links     │  │  • Links   │ │  │
│  │  │  • Joints    │  │  • Joints    │  │  • Joints  │ │  │
│  │  │  • Plugins   │  │  • Plugins   │  │  • Plugins │ │  │
│  │  └──────────────┘  └──────────────┘  └────────────┘ │  │
│  └─────────────────────────────────────────────────────┘  │
│                         │                                 │
│                         │ Uses                            │
│                         ▼                                 │
│  ┌─────────────────────────────────────────────────────┐  │
│  │  PHYSICS ENGINE (Computes motion)                   │  │
│  │                                                      │  │
│  │  Options: ODE, Bullet, DART, Simbody                │  │
│  │                                                      │  │
│  │  Every time step (e.g., 1ms):                       │  │
│  │  1. Apply forces (gravity, motors, contact)         │  │
│  │  2. Detect collisions                               │  │
│  │  3. Solve constraints (joints, contacts)            │  │
│  │  4. Update positions and velocities                 │  │
│  └─────────────────────────────────────────────────────┘  │
│                         │                                 │
│                         │ Updates                         │
│                         ▼                                 │
│  ┌─────────────────────────────────────────────────────┐  │
│  │  SENSORS (Generate data)                            │  │
│  │                                                      │  │
│  │  • Camera: Render image from camera pose            │  │
│  │  • LIDAR: Ray-cast to measure distances             │  │
│  │  • IMU: Read accelerations and angular velocities   │  │
│  │  • Contact: Detect collisions and forces            │  │
│  └─────────────────────────────────────────────────────┘  │
│                         │                                 │
│                         │ Publish to                      │
│                         ▼                                 │
│  ┌─────────────────────────────────────────────────────┐  │
│  │  ROS 2 TOPICS                                       │  │
│  │                                                      │  │
│  │  • /camera/image (sensor_msgs/Image)                │  │
│  │  • /scan (sensor_msgs/LaserScan)                    │  │
│  │  • /imu (sensor_msgs/Imu)                           │  │
│  │  • /joint_states (sensor_msgs/JointState)           │  │
│  └─────────────────────────────────────────────────────┘  │
│                                                           │
└───────────────────────────────────────────────────────────┘
                          │
                          │ Subscribe/Publish
                          ▼
┌───────────────────────────────────────────────────────────┐
│                   YOUR ROS 2 NODES                        │
│                                                           │
│  • Motion planning                                        │
│  • Computer vision                                        │
│  • Control algorithms                                     │
│  • Behavior planning                                      │
└───────────────────────────────────────────────────────────┘
```

**Data Flow**:
1. **World** contains **Models** (robots, objects)
2. **Physics Engine** simulates motion (every 1ms)
3. **Sensors** generate data based on model states
4. **ROS 2 Topics** publish sensor data
5. **Your Nodes** read sensor data, publish commands
6. **Plugins** apply commands to model actuators (motors)
7. **Physics Engine** updates motion based on forces

---

## Physics Engines in Gazebo

Gazebo supports multiple physics engines. You choose based on your needs:

### 1. ODE (Open Dynamics Engine)

**Default physics engine in Gazebo Classic**

**Strengths**:
- ✅ Fast for simple scenarios
- ✅ Stable and well-tested
- ✅ Good for wheeled robots

**Weaknesses**:
- ❌ Less accurate for complex contacts
- ❌ Can be unstable with long kinematic chains (humanoids)

**Use Cases**: Mobile robots, simple manipulators, quick prototyping

### 2. Bullet

**High-performance physics engine (used in video games)**

**Strengths**:
- ✅ Very fast
- ✅ Good collision detection
- ✅ Handles complex shapes well

**Weaknesses**:
- ❌ Less accurate for precise contact forces
- ❌ Not ideal for force-controlled tasks

**Use Cases**: Multi-robot simulations, environments with many objects

### 3. DART (Dynamic Animation and Robotics Toolkit)

**Research-focused physics engine**

**Strengths**:
- ✅ Very accurate contact dynamics
- ✅ Excellent for humanoid robots
- ✅ Supports complex kinematic chains

**Weaknesses**:
- ❌ Slower than ODE/Bullet
- ❌ More computationally expensive

**Use Cases**: Humanoid walking, manipulation with contact forces, legged robots

### 4. Simbody

**Biomechanics-oriented physics engine**

**Strengths**:
- ✅ Extremely accurate
- ✅ Good for biological systems

**Weaknesses**:
- ❌ Slowest option
- ❌ Overkill for most robotics tasks

**Use Cases**: Biomechanical research, medical robotics

### Choosing a Physics Engine

| **Engine** | **Speed** | **Accuracy** | **Best For** |
|------------|-----------|--------------|--------------|
| ODE        | Fast      | Moderate     | Wheeled robots, quick tests |
| Bullet     | Very Fast | Moderate     | Multi-robot, many objects |
| DART       | Moderate  | High         | Humanoids, legged robots |
| Simbody    | Slow      | Very High    | Biomechanics research |

**For this book**: We'll use **DART** for humanoid simulations (accurate contact dynamics).

**Setting Physics Engine in World File**:
```xml
<physics type="dart">  <!-- Changed from "ode" to "dart" -->
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <gravity>0 0 -9.81</gravity>
</physics>
```

---

## How Sensors and Robots Interact

### The Sensor Simulation Loop

**Every simulation step (e.g., 1ms)**:

1. **Physics Update**:
   - Physics engine computes new robot pose
   - Joints update based on motor commands
   - Robot moves to new position

2. **Sensor Update**:
   - Sensors check their update rates
   - If time to update (e.g., camera at 30 FPS updates every 33ms):
     - **Camera**: Render scene from camera's pose
     - **LIDAR**: Cast rays in all directions, measure distances
     - **IMU**: Read robot's acceleration and angular velocity
     - **Contact**: Check for collisions, measure forces

3. **Data Publication**:
   - Sensor plugins format data as ROS 2 messages
   - Publish to topics (e.g., `/camera/image`, `/scan`)

4. **Command Reception**:
   - Robot controller plugins listen for commands
   - Receive joint position/velocity/torque commands
   - Apply forces/torques to joints

5. **Physics Response**:
   - Physics engine applies forces
   - Computes resulting motion
   - Cycle repeats

### Example: Camera Sees a Cup

```
1. Physics step updates robot arm position
   → Camera link now at position (0.5, 0.2, 0.8)

2. Camera sensor checks: "Time to update? (30 FPS = every 33ms)"
   → Yes, last update was 33ms ago

3. Gazebo renders scene from camera's viewpoint
   → Raytracing: Cup is visible at pixel (320, 240)

4. Camera plugin creates sensor_msgs/Image message
   → 640×480 pixels, RGB8 format, timestamp

5. Plugin publishes to /camera/image topic

6. Your Python vision node receives image
   → Runs object detection: "Cup detected at (320, 240)"

7. Vision node publishes grasp command to /arm/joint_commands

8. Arm controller plugin receives command
   → Applies torques to shoulder, elbow, wrist joints

9. Physics engine computes motion
   → Arm moves toward cup

10. Cycle repeats at 1000 Hz
```

**The entire loop runs in simulation**—no real hardware needed.

---

## Gazebo's Coordinate System

**Gazebo uses a right-handed coordinate system**:

```
       Z (up)
       │
       │
       │
       └─────── X (forward)
      ╱
     ╱
    Y (left)
```

**Key Conventions**:
- **X-axis**: Forward (red in visualization)
- **Y-axis**: Left (green in visualization)
- **Z-axis**: Up (blue in visualization)
- **Origin**: Where your robot spawns (0, 0, 0)
- **Gravity**: Pulls in negative Z direction (0, 0, -9.81 m/s²)

**Example: Robot Orientation**
- Facing forward: `orientation = (0, 0, 0)` (roll, pitch, yaw = 0)
- Turned 90° left: `yaw = 1.57` (π/2 radians)
- Tilted forward 30°: `pitch = 0.524` (30° = 0.524 rad)

---

## Gazebo Simulation Modes

### Real-Time vs Faster-Than-Real-Time

**Real-Time Factor** controls simulation speed:

```xml
<real_time_factor>1.0</real_time_factor>  <!-- 1× real-time -->
```

**Options**:
- `1.0`: Simulation matches reality (1 second sim = 1 second real)
- `2.0`: 2× faster (1 second sim = 0.5 seconds real)
- `0.5`: Half speed (1 second sim = 2 seconds real)
- `0.0`: As fast as possible (no real-time constraint)

**When to Use**:
- **Real-time (1.0)**: Testing with real sensors, human interaction
- **Faster (2.0+)**: Training ML models, collecting data quickly
- **Slower (0.5)**: Debugging complex interactions, analyzing in detail
- **Max speed (0.0)**: Batch testing, overnight runs

**Note**: Actual speed depends on CPU power. Complex scenes may run slower than requested.

---

## Key Concepts Summary

**Gazebo Structure**:
- **Worlds**: Define environment (ground, lighting, physics)
- **Models**: Objects in simulation (robots, furniture, obstacles)
- **Plugins**: Add behavior (sensors, actuators, controllers)

**Three Main Components**:
- **3D Rendering**: Visualizes robots and environments
- **Physics Simulation**: Computes motion, collisions, forces
- **Sensor Simulation**: Generates realistic sensor data

**Physics Engines**:
- **ODE**: Fast, moderate accuracy (wheeled robots)
- **Bullet**: Very fast, good collisions (multi-robot)
- **DART**: Moderate speed, high accuracy (humanoids)
- **Simbody**: Slow, very high accuracy (biomechanics)

**Sensor-Robot Interaction**:
1. Physics updates robot pose
2. Sensors generate data based on pose
3. Data published to ROS 2 topics
4. Your nodes read data, send commands
5. Plugins apply commands to robot
6. Physics updates motion

**Coordinate System**:
- X: Forward (red)
- Y: Left (green)
- Z: Up (blue)
- Gravity: -Z direction

**Simulation Speed**:
- Real-time factor: 1.0 = real-time, 0.0 = max speed
- Adjust based on use case (debugging, training, testing)

---

## What's Next

You now understand Gazebo's architecture and how it simulates robots. Next, you'll learn how to:

**Next Topics**:
- **Page 12**: Installing Gazebo and ROS 2 Integration
- **Page 13**: Loading Your URDF into Gazebo
- **Page 14**: Configuring Physics Properties
- **Page 15**: Adding and Testing Sensors

**The Journey**:
- ✅ Understood why simulation matters
- ✅ **Learned Gazebo architecture and components**
- 🔜 Install Gazebo and spawn your robot
- 🔜 Configure physics for humanoid simulation
- 🔜 Test sensors and controllers

**You're Building Toward**: A complete workflow where you load your URDF robot into Gazebo, configure realistic physics, test sensors, and run control algorithms—all before touching real hardware.

---

*"Gazebo transforms your robot description into a living, breathing virtual entity that obeys the laws of physics—your laboratory without walls."*
