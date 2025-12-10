# Module 1, Chapter 2, Page 4: Python Agents Overview

**Book**: Physical AI & Humanoid Robotics — A Practical Guide
**Author**: Syeda Farheen Zehra
**Module 1**: Foundations of Physical AI
**Chapter 2**: Bridging Python Agents to ROS Controllers

---

## Why Python for Robotics?

Python has become the dominant language for AI and robotics development, and for good reason. When building intelligent humanoid robots, you need:

- **AI/ML libraries**: PyTorch, TensorFlow, scikit-learn
- **Computer vision**: OpenCV, PIL, mediapipe
- **Rapid prototyping**: Test ideas quickly without compilation
- **Readability**: Easy to understand and maintain
- **Ecosystem**: Thousands of packages for every task

**The Good News**: ROS 2 fully supports Python alongside C++. You can build entire robot systems using Python nodes, or mix Python (for AI logic) with C++ (for performance-critical control).

In this chapter, we'll explore how Python agents—intelligent programs that make decisions—integrate seamlessly with ROS 2's communication infrastructure.

---

## What is a Python Agent in ROS 2?

**Definition**: A **Python agent** is an intelligent ROS 2 node written in Python that perceives its environment (via sensors), makes decisions (using AI/logic), and takes actions (via actuators).

Think of agents as the **brain** of your robot:
- **Perception**: Reads sensor data from topics
- **Reasoning**: Processes information, runs AI models, plans actions
- **Action**: Publishes commands to motor controllers

### Agent vs Simple Node

| **Simple Node**                          | **Intelligent Agent**                    |
|------------------------------------------|------------------------------------------|
| Reads sensor, forwards raw data          | Reads sensor, interprets meaning         |
| Hardcoded responses (if X, do Y)         | Learned or adaptive behavior             |
| No memory or state                       | Maintains internal state, learns         |
| Rule-based logic                         | AI/ML models, planning algorithms        |
| Example: Camera driver                   | Example: Object recognition agent        |

**Key Distinction**: Agents **understand and decide**, not just **sense and forward**.

---

## Python's Role in the ROS 2 Ecosystem

ROS 2 uses a **client library** architecture. Instead of having a single monolithic ROS system, you use language-specific libraries:

- **rclpy**: ROS Client Library for Python (what we'll use)
- **rclcpp**: ROS Client Library for C++
- **rclnodejs**: ROS Client Library for Node.js

All libraries communicate through the same DDS middleware layer, so Python nodes and C++ nodes talk seamlessly.

### Architecture: Python Node in ROS 2

```
┌──────────────────────────────────────────────────┐
│          YOUR PYTHON AGENT                       │
│                                                  │
│  import rclpy                                    │
│  from rclpy.node import Node                     │
│  from sensor_msgs.msg import Image               │
│                                                  │
│  class MyAgent(Node):                            │
│      def __init__(self):                         │
│          # Subscribe to topics                   │
│          # Publish to topics                     │
│          # Run AI models                         │
│          # Make decisions                        │
└──────────────────────────────────────────────────┘
                    │
                    │ Uses rclpy library
                    ▼
┌──────────────────────────────────────────────────┐
│            RCLPY (Python ROS 2 Client)           │
│  - Handles topic pub/sub                         │
│  - Manages service calls                         │
│  - Provides timers, callbacks                    │
└──────────────────────────────────────────────────┘
                    │
                    ▼
┌──────────────────────────────────────────────────┐
│          DDS MIDDLEWARE (e.g., Fast-DDS)         │
│  - Real-time message passing                     │
│  - Network communication                         │
│  - Quality of Service enforcement                │
└──────────────────────────────────────────────────┘
                    │
                    ▼
        [Other ROS 2 Nodes: Python, C++, etc.]
```

**What This Means:**
- You write Python code using familiar syntax
- `rclpy` translates your commands to ROS 2 operations
- DDS middleware handles the heavy lifting (networking, timing)
- Your Python node communicates with **any** ROS 2 node (Python, C++, Rust, etc.)

---

## How Python Nodes Interact with Topics and Services

### Publishing to a Topic (Python Agent → ROS 2)

**Scenario**: Your Python agent detects an object and wants to command the robot to stop.

**Python Code Concept** (no full code yet, just the idea):
```python
# Inside your Python agent class
self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

# When you detect obstacle:
stop_command = Twist()
stop_command.linear.x = 0.0  # Stop moving forward
self.publisher.publish(stop_command)
```

**What Happens:**
1. Python agent creates a publisher for topic `/cmd_vel`
2. When obstacle detected, agent creates a `Twist` message (velocity command)
3. Agent publishes message to `/cmd_vel` topic
4. Motor controller node (could be C++) receives message
5. Robot stops

**Key Point**: The motor controller doesn't know your agent is written in Python. It just sees a message on `/cmd_vel`.

### Subscribing to a Topic (ROS 2 → Python Agent)

**Scenario**: Your Python agent needs camera images to detect objects.

**Python Code Concept**:
```python
# Inside your Python agent class
self.subscription = self.create_subscription(
    Image,
    '/camera/image',
    self.image_callback,
    10
)

def image_callback(self, msg):
    # This function runs every time a new image arrives
    # msg contains the image data
    # Run your AI model on msg
    # Detect objects
    # Make decisions
```

**What Happens:**
1. Python agent subscribes to `/camera/image` topic
2. Camera node (could be C++) publishes images to `/camera/image`
3. Every time new image published, `image_callback()` runs automatically
4. Agent processes image, detects objects, decides action

**Key Point**: Your callback function runs **automatically** when messages arrive. You don't poll or check manually.

### Calling a Service (Python Agent Requests → ROS 2 Server Responds)

**Scenario**: Your Python agent needs to compute a grasp pose before picking up an object.

**Python Code Concept**:
```python
# Inside your Python agent class
self.grasp_client = self.create_client(ComputeGrasp, '/compute_grasp')

# When you need grasp plan:
request = ComputeGrasp.Request()
request.object_id = "cup_01"

future = self.grasp_client.call_async(request)
# Wait for response...
response = future.result()  # Response contains grasp pose
```

**What Happens:**
1. Python agent creates a service client for `/compute_grasp`
2. Agent sends request: "Compute grasp for cup_01"
3. Grasp planner server (could be C++) computes answer
4. Server sends response: "Use this grip pose and approach angle"
5. Python agent receives response and continues

---

## Complete Example: Python Agent Reads Sensor → Commands Motor

Let's walk through a complete scenario: **Emergency Stop Agent**.

### The Mission
Your humanoid robot is walking. A Python agent monitors force sensors on the robot's feet. If excessive force detected (stumble, collision), the agent immediately stops all motors to prevent damage.

### The System

```
┌──────────────────┐
│  Force Sensors   │ (Hardware on robot feet)
└────────┬─────────┘
         │ Reads forces
         ▼
┌──────────────────┐
│ Force Sensor     │ (C++ node for real-time hardware access)
│    Driver        │
└────────┬─────────┘
         │ Publishes to /force/reading
         │ (sensor_msgs/JointState)
         │ 1000 Hz (real-time)
         ▼
    [TOPIC: /force/reading]
         │
         │ Subscribes
         ▼
┌──────────────────┐
│ Emergency Stop   │ (PYTHON AGENT - our intelligent node)
│     Agent        │
│  (Python)        │
│                  │
│ - Monitors force │
│ - Detects danger │
│ - Decides action │
└────────┬─────────┘
         │ Publishes to /emergency_stop
         │ (std_msgs/Bool)
         │ Only when needed
         ▼
    [TOPIC: /emergency_stop]
         │
         │ Subscribes
         ▼
┌──────────────────┐
│  Motor           │ (C++ node for low-latency motor control)
│  Controller      │
└────────┬─────────┘
         │ Sends stop command
         ▼
┌──────────────────┐
│   18 Motors      │ (Physical hardware)
└──────────────────┘
    Motors stop immediately
```

### Data Flow: Step-by-Step

**Step 1: Sensing (C++ → Python)**
- Force sensor driver (C++) reads hardware at 1000 Hz
- Creates `sensor_msgs/JointState` message with force values
- Publishes to `/force/reading` topic

**Step 2: Perception (Python Agent Receives)**
- Emergency stop agent (Python) is subscribed to `/force/reading`
- Callback function `force_callback()` runs automatically
- Receives message: `{left_foot: 850N, right_foot: 920N}`

**Step 3: Reasoning (Python Agent Decides)**
- Python agent analyzes forces
- Detects: "Right foot force > 900N threshold = stumble detected!"
- Makes decision: "Emergency stop required"

**Step 4: Action (Python → C++)**
- Python agent creates `std_msgs/Bool` message
- Sets value to `True` (emergency stop activated)
- Publishes to `/emergency_stop` topic

**Step 5: Execution (Motor Controller Responds)**
- Motor controller (C++) subscribes to `/emergency_stop`
- Receives `True` signal
- Immediately sends zero-velocity commands to all 18 motors
- Robot stops within 50 milliseconds

**Result**: Python agent detected danger and stopped robot using ROS 2 topics. No direct connection needed between Python and C++ code.

---

## Why Mix Python and C++?

You might wonder: "Why not write everything in Python?"

**The Reality of Robotics**: Different tasks need different tools.

### Use Python For:
✅ **AI/ML models**: Object detection, path planning, decision-making
✅ **High-level logic**: Task planning, behavior trees, state machines
✅ **Vision processing**: OpenCV, image segmentation, scene understanding
✅ **Rapid prototyping**: Test ideas quickly
✅ **Data processing**: Sensor fusion, filtering, analysis

**Example Nodes**: Object detector, grasp planner, navigation planner, speech recognizer

### Use C++ For:
✅ **Real-time control**: Motor controllers, balance loops (need \&lt;1ms timing)
✅ **Hardware drivers**: Sensors, actuators (low-level access)
✅ **Performance-critical**: High-frequency loops (1000+ Hz)
✅ **Resource-constrained**: Embedded boards with limited memory
✅ **Legacy integration**: Existing C++ libraries

**Example Nodes**: Motor driver, IMU driver, PID controller, low-level trajectory execution

### The Best of Both Worlds

**Typical Humanoid Robot Architecture**:

```
┌────────────────────────────────────────────────────┐
│              PYTHON LAYER (Intelligence)           │
│                                                    │
│  - Object Detection Agent (PyTorch)                │
│  - Navigation Planner (Python)                     │
│  - Grasp Planning Agent (Python)                   │
│  - Behavior Coordinator (Python state machine)     │
└────────────────┬───────────────────────────────────┘
                 │
                 │ ROS 2 Topics (Twist, Path, Pose, etc.)
                 │
┌────────────────▼───────────────────────────────────┐
│              C++ LAYER (Control)                   │
│                                                    │
│  - Motor Controller (C++ real-time)                │
│  - Balance Controller (C++ 1000Hz loop)            │
│  - Sensor Drivers (C++ hardware access)            │
│  - Emergency Stop Monitor (C++ safety-critical)    │
└────────────────────────────────────────────────────┘
```

**Python makes decisions** (what to do, where to go, what to grasp)
**C++ executes precisely** (how to move motors, how to balance)

They communicate through ROS 2 topics—**language doesn't matter**.

---

## Python Agent Capabilities in ROS 2

What can your Python agents do?

### 1. Subscribe to Multiple Topics Simultaneously

```
Python Agent
    │
    ├─── Subscribes to /camera/image (Camera)
    │
    ├─── Subscribes to /lidar/scan (LIDAR)
    │
    ├─── Subscribes to /imu/data (IMU)
    │
    └─── Fuses all sensor data → Makes navigation decision
```

**Use Case**: Multi-sensor fusion for robust navigation

### 2. Publish to Multiple Topics

```
Navigation Agent (Python)
    │
    ├─── Publishes to /cmd_vel (Motor commands)
    │
    ├─── Publishes to /path/plan (Planned trajectory)
    │
    └─── Publishes to /status (Agent status updates)
```

**Use Case**: Coordinating multiple subsystems

### 3. Call Services for Complex Computations

```
Manipulation Agent (Python)
    │
    └─── Calls service /inverse_kinematics
         Request: "Move hand to (x, y, z)"
         Response: "Use these 7 joint angles"
```

**Use Case**: Delegating expensive computations to specialized servers

### 4. Run Timers for Periodic Tasks

```
Monitoring Agent (Python)
    │
    └─── Timer: Every 1 second
         - Check battery level
         - Check system health
         - Publish diagnostics
```

**Use Case**: Periodic health checks, logging, status updates

### 5. Maintain Internal State

```
Behavior Agent (Python)
    │
    ├─── State: "exploring"
    │    - Wander randomly
    │    - Look for objects
    │
    ├─── Transition: Object detected
    │
    └─── State: "approaching"
         - Navigate to object
         - Prepare grasp
```

**Use Case**: State machines, behavior trees, task switching

---

## Python Agent Development Workflow

Here's how you'll develop Python agents in this book:

**1. Design Phase**
- What does the agent perceive? (Which topics to subscribe to?)
- What does the agent decide? (What logic/AI runs?)
- What does the agent do? (Which topics/services to publish/call?)

**2. Implementation Phase**
- Create Python class inheriting from `rclpy.node.Node`
- Set up subscriptions (callbacks for sensor data)
- Set up publishers (for commands/actions)
- Implement decision logic (AI models, algorithms)
- Add timers if needed

**3. Testing Phase**
- Run agent in simulation (Gazebo, Isaac Sim)
- Publish test data to input topics
- Verify agent's outputs (commands, decisions)
- Iterate and improve

**4. Deployment Phase**
- Launch agent alongside other nodes
- Monitor performance (latency, accuracy)
- Tune parameters (thresholds, rates)

**Tools You'll Use:**
- **rclpy**: Python ROS 2 library
- **Standard message types**: `sensor_msgs`, `geometry_msgs`, `std_msgs`
- **AI libraries**: PyTorch, OpenCV, NumPy (work seamlessly with ROS 2)
- **Simulation**: Gazebo, Isaac Sim (test before real hardware)

---

## Key Concepts Summary

**Python in ROS 2:**
- Python is a **first-class citizen** in ROS 2 via `rclpy` library
- Python nodes communicate with C++ nodes seamlessly through DDS middleware
- Use Python for **intelligence** (AI, planning, vision), C++ for **real-time control**

**Python Agents:**
- **Perceive** via topic subscriptions (sensor data)
- **Reason** using AI models, logic, planning algorithms
- **Act** via topic publications and service calls (commands, actions)
- Maintain **internal state** for complex behaviors

**Interaction Patterns:**
- **Subscribe**: Receive sensor data automatically via callbacks
- **Publish**: Send commands/results to other nodes
- **Service calls**: Request computations, wait for responses
- **Timers**: Run periodic tasks (monitoring, logging)

**Best Practices:**
- **Python for brains**: Decision-making, AI, high-level planning
- **C++ for muscles**: Motor control, real-time loops, hardware drivers
- **Mix freely**: Language choice is invisible in ROS 2 communication

---

## What's Next

Now that you understand Python's role in ROS 2, we'll move from concept to code:

**Next Page**: You'll write your first Python ROS 2 node—a simple publisher that sends messages, and a subscriber that receives them. You'll see `rclpy` in action and understand the code structure all Python agents follow.

**Upcoming Topics:**
- Python node structure (class-based design)
- Creating publishers and subscribers in code
- Message types and how to use them
- Callback functions and event-driven programming
- Building your first intelligent agent

**The Journey**: By the end of this chapter, you'll have built a complete Python agent that reads sensor data, makes decisions, and controls actuators—all through ROS 2.

---

*"Python brings intelligence. ROS 2 brings communication. Together, they bring robots to life."*
