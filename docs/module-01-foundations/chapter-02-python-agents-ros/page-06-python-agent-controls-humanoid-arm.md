# Module 1, Chapter 2, Page 6: Scenario — Python Agent Controls a Humanoid Arm

**Book**: Physical AI & Humanoid Robotics — A Practical Guide
**Author**: Syeda Farheen Zehra
**Module 1**: Foundations of Physical AI
**Chapter 2**: Bridging Python Agents to ROS Controllers

---

## The Mission: Reach and Grasp an Object

Imagine your humanoid robot standing in front of a table. A coffee cup sits 50 cm away. Your task: build a Python agent that controls the robot's right arm to reach out, position the hand correctly, and prepare to grasp the cup.

**The Challenge**: Coordinating 7 arm joints (shoulder pitch/roll/yaw, elbow, wrist pitch/roll/yaw) based on visual perception and force feedback—all in real-time.

**The Solution**: A Python agent that perceives the cup's location, computes the arm motion, and commands motors through ROS 2 topics.

---

## System Overview

Let's see all the components working together:

```
┌─────────────────────────────────────────────────────────────┐
│                    HARDWARE LAYER                           │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌────────────┐   ┌────────────┐   ┌────────────┐          │
│  │  Camera    │   │ Force      │   │ 7 Arm      │          │
│  │  (RGB-D)   │   │ Sensors    │   │ Motors     │          │
│  │            │   │ (Torque)   │   │            │          │
│  └─────┬──────┘   └─────┬──────┘   └─────▲──────┘          │
│        │                │                  │                │
└────────┼────────────────┼──────────────────┼────────────────┘
         │                │                  │
         │                │                  │
┌────────┼────────────────┼──────────────────┼────────────────┐
│        │  ROS 2 TOPICS  │                  │                │
│        │                │                  │                │
│   /camera/image    /force/torque      /arm/joint_commands  │
│        │                │                  │                │
└────────┼────────────────┼──────────────────┼────────────────┘
         │                │                  │
         ▼                ▼                  │
┌─────────────────────────────────────────────┴───────────────┐
│              PYTHON AGENT (INTELLIGENCE)                    │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  ArmControlAgent (Python Node)                       │   │
│  │                                                      │   │
│  │  Subscribes:                                         │   │
│  │  • /camera/image (sensor_msgs/Image)                 │   │
│  │  • /force/torque (sensor_msgs/JointState)            │   │
│  │                                                      │   │
│  │  Processes:                                          │   │
│  │  • Detects cup position (computer vision)            │   │
│  │  • Computes inverse kinematics (7-DOF arm)           │   │
│  │  • Plans trajectory (smooth motion)                  │   │
│  │  • Monitors safety (force limits)                    │   │
│  │                                                      │   │
│  │  Publishes:                                          │   │
│  │  • /arm/joint_commands (trajectory_msgs/JointTraj)   │   │
│  │                                                      │   │
│  └──────────────────────────────────────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────┐
│              C++ MOTOR CONTROLLER                           │
│                                                             │
│  • Subscribes to /arm/joint_commands                        │
│  • Executes trajectory at 1000 Hz                           │
│  • Controls 7 motors with PID loops                         │
│  • Publishes joint states to /arm/joint_states              │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## Step-by-Step: From Perception to Action

### Step 1: Sensor Input — Seeing the Cup

**Hardware**:
- RGB-D camera mounted on robot's head
- Captures color image + depth (distance) data
- Runs at 30 frames per second

**C++ Camera Driver Node**:
```
Camera Driver (C++)
    │
    ├─── Reads camera hardware
    ├─── Creates sensor_msgs/Image message
    │    • Header: timestamp, frame_id
    │    • Height: 480 pixels
    │    • Width: 640 pixels
    │    • Encoding: "rgb8"
    │    • Data: raw pixel array
    │
    └─── Publishes to /camera/image topic (30 Hz)
```

**What Gets Published**:
```python
# Conceptual message content
Image message:
  header:
    stamp: 1234567890  # Current time
    frame_id: "head_camera"
  height: 480
  width: 640
  encoding: "rgb8"
  data: [255, 120, 34, ...]  # Millions of pixel values
```

**The Flow**:
1. Camera captures scene (cup on table)
2. Driver converts raw data to ROS 2 Image message
3. Message published to `/camera/image` topic
4. **Any node can now see what the robot sees**

---

### Step 2: Python Agent Perceives — Finding the Cup

**Python Agent Receives Image**:

```python
# Inside ArmControlAgent class (conceptual)

def __init__(self):
    super().__init__('arm_control_agent')

    # Subscribe to camera
    self.image_subscription = self.create_subscription(
        Image,
        '/camera/image',
        self.image_callback,
        10
    )

    self.cup_position = None  # Will store detected cup location

def image_callback(self, msg):
    """Runs every time new image arrives (30 times/second)"""

    # 1. Convert ROS Image to OpenCV format
    cv_image = self.convert_ros_to_opencv(msg)

    # 2. Run object detection (e.g., YOLOv8, or color detection)
    detected_objects = self.detect_objects(cv_image)

    # 3. Find the cup
    for obj in detected_objects:
        if obj.class_name == "cup":
            # Get 3D position from depth data
            x = obj.center_x  # pixels
            y = obj.center_y  # pixels
            depth = self.get_depth_at(x, y)  # meters

            # Convert pixel + depth to 3D world coordinates
            self.cup_position = self.pixel_to_world(x, y, depth)

            self.get_logger().info(
                f'Cup detected at: {self.cup_position}'
            )

            # Now we know where the cup is!
            # Trigger arm motion planning
            self.plan_reach_motion()
```

**What Happened**:
- Image message arrived via topic
- Callback ran automatically
- Python agent used OpenCV to detect cup
- Extracted 3D position: `cup_position = (0.5, -0.2, 0.8)` meters
  - 0.5m forward
  - 0.2m to the left
  - 0.8m high (table height)

---

### Step 3: Python Agent Reasons — Planning the Motion

**The Problem**: How do I move my hand to position (0.5, -0.2, 0.8)?

**The Answer**: Inverse Kinematics (IK)
- You know **where** you want the hand (target position)
- You need to compute **what joint angles** achieve that position
- 7 joints means infinite solutions—choose the most natural one

**Python Agent Computes Trajectory**:

```python
def plan_reach_motion(self):
    """Plan smooth arm motion to cup position"""

    # 1. Current arm state
    current_joints = self.get_current_joint_angles()
    # Example: [0.1, 0.2, 0.0, 1.5, 0.0, 0.5, 0.0] radians

    # 2. Desired hand position
    target_position = self.cup_position  # (0.5, -0.2, 0.8)

    # 3. Compute inverse kinematics
    # (Use library like ikpy, MoveIt, or simple geometric IK)
    target_joints = self.compute_ik(target_position)
    # Result: [0.3, 0.5, 0.1, 1.2, 0.2, 0.8, 0.1] radians

    # 4. Generate smooth trajectory (interpolate from current to target)
    trajectory = self.interpolate_trajectory(
        start=current_joints,
        goal=target_joints,
        duration=2.0  # 2 seconds to reach
    )

    # 5. Send trajectory to motors
    self.execute_trajectory(trajectory)

    self.get_logger().info('Arm motion planned, executing...')
```

**Trajectory Generation**:
```
Time    Shoulder   Elbow   Wrist   ...
0.0s    [0.1,      1.5,    0.5]    (start position)
0.5s    [0.15,     1.4,    0.6]    (interpolated)
1.0s    [0.20,     1.3,    0.7]    (interpolated)
1.5s    [0.25,     1.25,   0.75]   (interpolated)
2.0s    [0.3,      1.2,    0.8]    (goal position)
```

---

### Step 4: ROS 2 Topic Communication — Commanding the Arm

**Python Agent Publishes Commands**:

```python
def execute_trajectory(self, trajectory):
    """Send trajectory to motor controller via ROS 2 topic"""

    # Create trajectory message
    msg = JointTrajectory()
    msg.joint_names = [
        'shoulder_pitch', 'shoulder_roll', 'shoulder_yaw',
        'elbow', 'wrist_pitch', 'wrist_roll', 'wrist_yaw'
    ]

    # Add each waypoint
    for waypoint in trajectory:
        point = JointTrajectoryPoint()
        point.positions = waypoint.joint_angles
        point.velocities = waypoint.velocities
        point.time_from_start = Duration(sec=waypoint.time)
        msg.points.append(point)

    # Publish to motor controller
    self.trajectory_publisher.publish(msg)

    self.get_logger().info('Trajectory sent to motors!')
```

**What Gets Published to `/arm/joint_commands`**:
```yaml
JointTrajectory:
  joint_names: [shoulder_pitch, shoulder_roll, ..., wrist_yaw]
  points:
    - positions: [0.1, 0.2, 0.0, 1.5, 0.0, 0.5, 0.0]
      velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      time_from_start: 0.0 sec
    - positions: [0.15, 0.25, 0.05, 1.4, 0.1, 0.6, 0.05]
      velocities: [0.1, 0.1, 0.1, -0.2, 0.2, 0.2, 0.1]
      time_from_start: 0.5 sec
    - positions: [0.3, 0.5, 0.1, 1.2, 0.2, 0.8, 0.1]
      velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      time_from_start: 2.0 sec
```

**The Message Travels**:
```
Python Agent
    │ Creates JointTrajectory message
    │ Calls trajectory_publisher.publish(msg)
    ▼
DDS Middleware
    │ Serializes message
    │ Sends over network/localhost
    ▼
C++ Motor Controller
    │ Receives message
    │ Deserializes into C++ structure
    │ Begins executing trajectory
```

---

### Step 5: Motor Actuation — The Arm Moves

**C++ Motor Controller Node**:

```
Motor Controller (C++)
    │
    ├─── Subscribes to /arm/joint_commands
    │
    ├─── Receives JointTrajectory message
    │
    ├─── Starts 1000 Hz control loop:
    │
    │    Every 1 millisecond:
    │    ┌─────────────────────────────────────┐
    │    │ 1. Read current joint angles        │
    │    │ 2. Compute where we should be now   │
    │    │    (interpolate trajectory)         │
    │    │ 3. Calculate error                  │
    │    │ 4. Run PID controller               │
    │    │ 5. Send motor commands              │
    │    │ 6. Publish joint states             │
    │    └─────────────────────────────────────┘
    │
    └─── Motors move smoothly from start to goal
```

**What the Motors Do**:
1. **t = 0.0s**: Arm at starting position (current pose)
2. **t = 0.5s**: Shoulder rotates 0.05 rad, elbow extends slightly
3. **t = 1.0s**: Arm halfway to target, smooth motion
4. **t = 1.5s**: Approaching cup, wrist adjusting orientation
5. **t = 2.0s**: Hand positioned above cup, ready to grasp

**Real-Time Execution**:
- Motor controller runs PID loops at **1000 Hz** (every 1 millisecond)
- Ensures smooth, precise motion
- Compensates for gravity, friction, external forces
- **Python agent doesn't handle real-time control—C++ does**

---

## Safety: Monitoring Force Sensors

While the arm moves, the Python agent also monitors force sensors to prevent collisions.

**Force Sensor Feedback Loop**:

```python
def __init__(self):
    # ... (other subscriptions)

    # Subscribe to force/torque sensors
    self.force_subscription = self.create_subscription(
        JointState,
        '/force/torque',
        self.force_callback,
        10
    )

    self.max_safe_torque = 5.0  # Newton-meters

def force_callback(self, msg):
    """Monitor forces during motion"""

    for i, torque in enumerate(msg.effort):
        if abs(torque) > self.max_safe_torque:
            # Excessive force detected!
            self.get_logger().warn(
                f'High torque on joint {i}: {torque} Nm'
            )

            # Send emergency stop
            self.stop_arm_immediately()

def stop_arm_immediately(self):
    """Emergency stop via ROS 2 topic"""

    stop_msg = JointTrajectory()
    stop_msg.joint_names = self.joint_names

    # Single point: current position, zero velocity
    point = JointTrajectoryPoint()
    point.positions = self.get_current_joint_angles()
    point.velocities = [0.0] * 7
    point.time_from_start = Duration(sec=0)
    stop_msg.points.append(point)

    self.trajectory_publisher.publish(stop_msg)
    self.get_logger().info('EMERGENCY STOP: Arm halted')
```

**Safety Flow**:
```
Force Sensors → Driver (C++) → /force/torque topic → Python Agent
                                                          │
                                   Detects excessive force │
                                                          │
                                    Publishes stop command │
                                                          ▼
                              /arm/joint_commands topic ← Python Agent
                                                          │
                                                          ▼
                                              Motor Controller (C++)
                                                          │
                                                          ▼
                                                   Motors stop
```

---

## Complete Data Flow Summary

Let's trace one complete cycle from seeing the cup to positioning the arm:

**1. Perception (Camera → Python)**
- Camera driver publishes image to `/camera/image` (30 Hz)
- Python agent receives image via subscription
- Callback `image_callback()` runs automatically

**2. Detection (Python Processing)**
- OpenCV detects cup in image
- Extracts 3D position from depth data
- Result: `cup_position = (0.5, -0.2, 0.8)` meters

**3. Planning (Python Computation)**
- Compute inverse kinematics for target position
- Generate smooth trajectory (7 joints, 2 seconds)
- Create `JointTrajectory` message with waypoints

**4. Communication (Python → C++ via ROS 2)**
- Python agent publishes trajectory to `/arm/joint_commands`
- DDS middleware delivers message to C++ motor controller
- **Language doesn't matter—ROS 2 handles communication**

**5. Execution (C++ Real-Time Control)**
- Motor controller receives trajectory
- 1000 Hz PID loop executes motion
- Motors move arm smoothly to cup position

**6. Safety Monitoring (Continuous)**
- Force sensors publish torque data to `/force/torque`
- Python agent monitors for excessive forces
- Can send emergency stop if needed

**Total Time**: ~2 seconds from detection to grasp-ready position

---

## Key Architectural Decisions

### Why Python for the Agent?
✅ **Computer vision**: OpenCV, PIL, object detection models
✅ **Inverse kinematics**: Libraries like `ikpy`, `pybullet`
✅ **Trajectory planning**: NumPy, SciPy for interpolation
✅ **Rapid iteration**: Test algorithms quickly

### Why C++ for Motor Control?
✅ **Real-time performance**: 1000 Hz control loop (1ms timing)
✅ **Hardware access**: Direct motor communication
✅ **Deterministic**: No garbage collection pauses
✅ **Safety-critical**: Reliable, tested, predictable

### Why ROS 2 Topics?
✅ **Decoupling**: Python agent doesn't know about motor hardware details
✅ **Modularity**: Swap motor controller without changing agent
✅ **Observability**: Other nodes can monitor `/arm/joint_commands`
✅ **Testing**: Publish test trajectories without real hardware

---

## What the Student Learns

**From This Scenario, You Now Understand:**

1. **Perception → Action Pipeline**
   - Sensor data flows through topics
   - Python agent processes high-level perception
   - Commands flow back through topics to actuators

2. **Mixed Language Architecture**
   - Python handles intelligence (vision, planning)
   - C++ handles real-time execution (motor control)
   - ROS 2 makes them work together seamlessly

3. **Message-Driven Design**
   - Agents react to incoming messages (callbacks)
   - Agents publish commands as messages
   - No direct function calls between nodes

4. **Real-Time Separation**
   - Python agent runs at perception speed (~30 Hz)
   - Motor controller runs at control speed (1000 Hz)
   - Each node operates at its natural frequency

5. **Safety Through Monitoring**
   - Multiple sensor streams (camera, force)
   - Agent can intervene (emergency stop)
   - Continuous feedback loops

---

## Visualizing the Complete System

```
                    THE HUMANOID ARM CONTROL SYSTEM

┌─────────────────────────────────────────────────────────────────┐
│                         SENSORS                                 │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐        │
│  │ Camera   │  │ Force    │  │ Joint    │  │ IMU      │        │
│  │ (RGB-D)  │  │ Sensors  │  │ Encoders │  │ (Balance)│        │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘        │
└───────┼─────────────┼─────────────┼─────────────┼───────────────┘
        │             │             │             │
        │ /camera/    │ /force/     │ /arm/       │ /imu/
        │ image       │ torque      │ joint_states│ data
        │             │             │             │
        ▼             ▼             ▼             ▼
┌─────────────────────────────────────────────────────────────────┐
│                  PYTHON AGENT (Intelligence)                    │
│                                                                 │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │  ArmControlAgent                                          │  │
│  │                                                           │  │
│  │  • Object Detection (cup position)                        │  │
│  │  • Inverse Kinematics (joint angles)                      │  │
│  │  • Trajectory Planning (smooth motion)                    │  │
│  │  • Force Monitoring (safety)                              │  │
│  │  • State Machine (reach → grasp → lift)                   │  │
│  └───────────────────────────────────────────────────────────┘  │
│                             │                                   │
│                             │ /arm/joint_commands               │
│                             │ (JointTrajectory)                 │
└─────────────────────────────┼─────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│              C++ MOTOR CONTROLLER (Real-Time)                   │
│                                                                 │
│  • Receives trajectory commands                                │
│  • Executes PID control loops (1000 Hz)                        │
│  • Commands 7 motors (position + velocity)                     │
│  • Publishes joint states                                      │
└─────────────────────────────┬───────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                       ACTUATORS                                 │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐        │
│  │ Shoulder │  │ Shoulder │  │ Shoulder │  │ Elbow    │        │
│  │ Pitch    │  │ Roll     │  │ Yaw      │  │          │        │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘        │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                      │
│  │ Wrist    │  │ Wrist    │  │ Wrist    │                      │
│  │ Pitch    │  │ Roll     │  │ Yaw      │                      │
│  └──────────┘  └──────────┘  └──────────┘                      │
│                                                                 │
│              ARM MOVES TO CUP POSITION                          │
└─────────────────────────────────────────────────────────────────┘
```

---

## Key Concepts Summary

**Python Agent's Role**:
- Subscribes to sensor topics (camera, force)
- Processes high-level perception (object detection)
- Computes motion plans (inverse kinematics, trajectories)
- Publishes commands to actuators (joint trajectories)
- Monitors safety (force limits, collisions)

**ROS 2's Role**:
- Delivers sensor data from hardware to Python (topics)
- Delivers commands from Python to motors (topics)
- Handles all serialization, networking, timing
- Decouples perception (Python) from control (C++)

**The Power of This Architecture**:
- **Modularity**: Swap camera without changing agent logic
- **Testability**: Send fake images, test agent in isolation
- **Language Freedom**: Python for intelligence, C++ for speed
- **Scalability**: Add more sensors, more agents, more arms
- **Real-World Ready**: This is how professional robots work

---

## What's Next

You've now seen a complete, realistic scenario of a Python agent controlling a humanoid arm through ROS 2. This pattern—**perceive, reason, act**—is the foundation of all robot intelligence.

**Next Topics:**
- Writing your first complete Python ROS 2 node (hands-on code)
- Testing agents in simulation (Gazebo, Isaac Sim)
- Advanced trajectory planning algorithms
- Multi-agent coordination (two arms working together)

**The Journey So Far:**
- ✅ Understood what ROS 2 solves
- ✅ Learned ROS 2 architecture (nodes, topics, services)
- ✅ Explored Python's role in robotics
- ✅ Studied rclpy node structure
- ✅ Traced complete sensor-to-motor data flow

**You're Ready**: In the next pages, you'll write actual code, run it in simulation, and see your Python agent bring a virtual humanoid arm to life.

---

*"From pixels to motion, from perception to action—this is how robots learn to interact with the physical world."*
