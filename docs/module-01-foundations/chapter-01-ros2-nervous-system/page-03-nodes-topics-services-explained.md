# Module 1, Chapter 1, Page 3: Nodes, Topics, and Services Explained

**Book**: Physical AI & Humanoid Robotics — A Practical Guide
**Author**: Syeda Farheen Zehra
**Module 1**: Foundations of Physical AI
**Chapter 1**: The Robotic Nervous System (ROS 2)

---

## Understanding ROS 2 Through Simple Examples

You've learned *why* ROS 2 exists and its overall architecture. Now let's break down the three fundamental concepts—**nodes**, **topics**, and **services**—with concrete examples you can visualize.

By the end of this page, you'll understand exactly how a sensor node publishes data, how a motor node subscribes to commands, and when to use services for special requests.

---

## What is a Node?

**Definition**: A **node** is a single executable program that performs one specific task in your robot system.

Think of nodes as individual workers in a factory:
- One worker (node) operates the camera
- Another worker (node) processes images
- A third worker (node) controls the motors

Each worker focuses on their job and doesn't need to know how the others do theirs—they just exchange information.

### Node Characteristics

✅ **Independent**: Each node runs as its own process (program)
✅ **Single-purpose**: One node, one job (camera driver, obstacle detector, motor controller)
✅ **Communicates via topics/services**: Nodes don't call each other's functions directly
✅ **Can run anywhere**: On the robot's computer, your laptop, or a remote server
✅ **Restartable**: If a node crashes, restart it without rebooting the whole robot

### Example Nodes in a Humanoid Robot

| Node Name           | Job                                      |
|---------------------|------------------------------------------|
| `camera_driver`     | Captures images from camera              |
| `object_detector`   | Identifies objects in images             |
| `lidar_driver`      | Reads LIDAR distance measurements        |
| `obstacle_avoider`  | Plans paths around obstacles             |
| `imu_driver`        | Reads balance/orientation data           |
| `balance_controller`| Keeps robot upright using IMU data       |
| `motor_controller`  | Sends commands to motors                 |
| `speech_recognizer` | Converts voice to text commands          |

**Key Point**: Each node is **replaceable**. Swap the `camera_driver` node for a different camera model without touching any other node.

---

## What is a Topic?

**Definition**: A **topic** is a named channel that carries messages between nodes. Nodes **publish** (send) messages to topics, and other nodes **subscribe** (receive) messages from topics.

Think of topics like radio frequencies:
- A radio station broadcasts on 101.5 FM (topic name)
- Anyone with a radio can tune to 101.5 FM and listen
- The station doesn't know who's listening
- Multiple radios can listen simultaneously

### Topic Characteristics

✅ **Named channel**: Topics have names like `/camera/image` or `/cmd_vel` (command velocity)
✅ **One-way communication**: Publisher sends, subscribers receive (no acknowledgment)
✅ **Many-to-many**: Multiple publishers, multiple subscribers on the same topic
✅ **Asynchronous**: Publisher doesn't wait—it sends data and continues
✅ **Typed**: Each topic carries a specific message type (e.g., images, velocity commands)

### How Topics Work: The Publisher-Subscriber Model

```
┌─────────────────┐                    ┌─────────────────┐
│ CAMERA NODE     │                    │ OBJECT DETECTOR │
│ (Publisher)     │                    │ (Subscriber)    │
│                 │                    │                 │
│ Captures image  │                    │ Waiting for     │
│ 30 times/sec    │                    │ images...       │
└────────┬────────┘                    └────────▲────────┘
         │                                      │
         │ Publishes to topic:                  │
         │ "/camera/image"                      │
         │                                      │
         └──────────────────────────────────────┘
                   (Image messages flow)
```

**What Happens:**
1. Camera node captures an image
2. Camera node publishes the image to topic `/camera/image`
3. Object detector node is subscribed to `/camera/image`
4. Object detector receives the image automatically
5. Object detector processes the image
6. Repeat 30 times per second

**No Direct Connection**: The camera node doesn't know the object detector exists. It just publishes to `/camera/image`. Any node can subscribe.

---

## Publisher and Subscriber Roles

### Publisher Role

A **publisher** is a node that sends data to a topic.

**What Publishers Do:**
- Generate data (sensor readings, processed results, commands)
- Package data into messages
- Send messages to a topic
- Continue sending at their own rate (1 Hz, 30 Hz, 1000 Hz, etc.)

**Example**: Camera node publishes images
```
Camera Node (Publisher)
    ↓
Capture image from camera hardware
    ↓
Create "sensor_msgs/Image" message
    ↓
Publish to topic "/camera/image"
    ↓
Repeat 30 times per second
```

### Subscriber Role

A **subscriber** is a node that receives data from a topic.

**What Subscribers Do:**
- Listen to a topic
- Receive messages when published
- Process each message
- Trigger actions based on data

**Example**: Object detector subscribes to images
```
Object Detector (Subscriber)
    ↓
Subscribe to topic "/camera/image"
    ↓
Wait for message to arrive
    ↓
Receive "sensor_msgs/Image" message
    ↓
Process image (detect objects)
    ↓
Publish results to another topic
```

### One Node Can Be Both

A node can be **both** a publisher and subscriber simultaneously.

**Example**: Object Detector Node
- **Subscribes** to `/camera/image` (receives images)
- **Publishes** to `/detected_objects` (sends detection results)

```
        [Camera Node] ─────► /camera/image ─────► [Object Detector]
                                                           │
                                                           │ Publishes
                                                           ▼
                                                   /detected_objects
                                                           │
                                                           │ Subscribes
                                                           ▼
                                                    [Planning Node]
```

---

## Practical Example: LIDAR Sensor and Obstacle Avoidance

Let's see a complete real-world example with a LIDAR sensor and obstacle avoidance.

### The Scenario

Your humanoid robot is walking forward. It needs to:
1. Scan the environment with LIDAR (measure distances to obstacles)
2. Detect if obstacles are too close
3. Slow down or stop to avoid collision

### The Nodes

**1. LIDAR Driver Node** (Publisher)
- Reads distance measurements from LIDAR hardware
- Publishes data to topic `/scan`
- Update rate: 10 times per second (10 Hz)

**2. Obstacle Detector Node** (Subscriber + Publisher)
- Subscribes to `/scan` topic
- Analyzes distances to find obstacles
- Publishes detected obstacles to `/obstacles`

**3. Motion Controller Node** (Subscriber + Publisher)
- Subscribes to `/obstacles` topic
- Decides if robot should slow down or stop
- Publishes velocity commands to `/cmd_vel`

**4. Motor Driver Node** (Subscriber)
- Subscribes to `/cmd_vel` topic
- Sends speed commands to motors
- Executes the motion

### The Data Flow

```
┌──────────────────┐
│  LIDAR Hardware  │
└────────┬─────────┘
         │ Scans environment
         ▼
┌──────────────────┐     Topic: /scan          ┌──────────────────┐
│ LIDAR Driver     │────(LaserScan messages)──►│ Obstacle Detector│
│ Node (Publisher) │                           │ Node (Sub + Pub) │
└──────────────────┘                           └────────┬─────────┘
                                                        │
                                        Topic: /obstacles
                                       (Obstacle messages)
                                                        │
                                                        ▼
                                               ┌──────────────────┐
                                               │ Motion Controller│
                                               │ Node (Sub + Pub) │
                                               └────────┬─────────┘
                                                        │
                                             Topic: /cmd_vel
                                          (Velocity commands)
                                                        │
                                                        ▼
                                               ┌──────────────────┐
                                               │  Motor Driver    │
                                               │ Node (Subscriber)│
                                               └────────┬─────────┘
                                                        │
                                                        ▼
                                                   [Motors]
                                              (Robot slows/stops)
```

### Step-by-Step Flow

**Step 1**: LIDAR driver reads sensor
- Measures distances in 360 degrees
- Creates `sensor_msgs/LaserScan` message with distance array
- Publishes to `/scan` topic (10 times/second)

**Step 2**: Obstacle detector receives scan
- Subscribes to `/scan`
- Analyzes distances: "Object 0.5 meters ahead!"
- Creates `obstacle_msgs/Obstacle` message
- Publishes to `/obstacles` topic

**Step 3**: Motion controller receives obstacle
- Subscribes to `/obstacles`
- Decides: "Too close! Slow down to 0.2 m/s"
- Creates `geometry_msgs/Twist` message (velocity command)
- Publishes to `/cmd_vel` topic

**Step 4**: Motor driver receives velocity command
- Subscribes to `/cmd_vel`
- Sends reduced speed command to motors
- Robot slows down safely

**Result**: Robot avoids collision, all through topic-based communication.

---

## What is a Service?

**Definition**: A **service** is a request-response communication pattern. A **client** node sends a request, and a **server** node sends back a response.

Unlike topics (continuous streaming), services are for **one-time queries or commands** that need confirmation.

### Service Characteristics

✅ **Synchronous**: Client waits for server's response before continuing
✅ **Request-Response**: Two-way communication (unlike topic's one-way)
✅ **On-demand**: Called when needed (unlike topic's continuous stream)
✅ **Named**: Services have names like `/get_battery_level` or `/plan_grasp`

### When to Use Services vs Topics

| **Scenario**                           | **Use Topic** | **Use Service** |
|----------------------------------------|---------------|-----------------|
| Streaming sensor data (camera, IMU)   | ✅            | ❌              |
| Motor velocity commands                | ✅            | ❌              |
| Query: "What's the battery level?"     | ❌            | ✅              |
| Command: "Plan a path to kitchen"      | ❌            | ✅              |
| Request: "Compute grasp pose"          | ❌            | ✅              |
| Continuous position updates            | ✅            | ❌              |

**Rule of Thumb:**
- **High-frequency, continuous data** → Topic
- **One-time queries or computations** → Service

### How Services Work

```
┌──────────────────┐                    ┌──────────────────┐
│  PLANNING NODE   │                    │  PATH PLANNER    │
│    (Client)      │                    │    (Server)      │
│                  │                    │                  │
│ Needs path to    │                    │ Waiting for      │
│ kitchen          │                    │ requests...      │
└────────┬─────────┘                    └────────▲─────────┘
         │                                       │
         │ 1. Service Request:                   │
         │    "Plan path to (x=5, y=3)"          │
         └───────────────────────────────────────┘

         ┌───────────────────────────────────────┐
         │ 2. Server computes path               │
         │    (takes 100 milliseconds)           │
         └───────────────────────────────────────┘

         ┌───────────────────────────────────────┐
         │ 3. Service Response:                  │
         │    "Here's the path: [waypoints]"     │
         └───────────────────────────────────────┘

         Client receives path and continues
```

### Service Example: Battery Level Check

**Scenario**: Your robot's planning node needs to know if there's enough battery before starting a long task.

**Service Name**: `/get_battery_level`
**Client**: Planning Node
**Server**: Battery Monitor Node

**Flow:**
1. Planning node calls service: `/get_battery_level`
2. Battery monitor reads sensor and responds: "78%"
3. Planning node decides: "Enough battery, start task"

**Why Service (not Topic)?**
- **Infrequent**: Battery checked once before tasks, not continuously
- **Needs answer**: Planning node must wait for response before deciding
- **Request-driven**: Only called when planning node asks, not continuous broadcast

---

## Topics vs Services: Visual Comparison

### Topic (Continuous Stream)

```
PUBLISHER                         SUBSCRIBERS
  (Camera)                    (Multiple nodes listening)
     │                                 ▲
     │  Publishes images               │
     │  30 times/second                │
     │  continuously                   │
     └─────────────────────────────────┘

     No waiting, no confirmation
     Just streams data
```

### Service (One-Time Request)

```
CLIENT                           SERVER
(Planning)                   (Path Planner)
     │                                 │
     │  1. Request: "Plan path"        │
     └─────────────────────────────────►

     Waits...

     ◄─────────────────────────────────┘
          2. Response: "Here's path"
     │
     Continues with path
```

---

## Key Concepts Summary

### Nodes
- **What**: Independent programs performing specific tasks
- **Why**: Modularity, fault isolation, distributed processing
- **Example**: Camera driver, object detector, motor controller

### Topics
- **What**: Named channels for streaming messages
- **Pattern**: Publisher-Subscriber (one-way, asynchronous)
- **When**: Continuous data (sensors, commands, status updates)
- **Example**: `/camera/image`, `/cmd_vel`, `/scan`

### Publishers
- **Role**: Send data to topics
- **Behavior**: Publish continuously at their own rate
- **Don't know**: Who's listening (if anyone)

### Subscribers
- **Role**: Receive data from topics
- **Behavior**: Process each message as it arrives
- **Don't know**: Who's publishing

### Services
- **What**: Request-response communication
- **Pattern**: Client-Server (two-way, synchronous)
- **When**: One-time queries, computations requiring confirmation
- **Example**: `/get_battery_level`, `/plan_path`, `/compute_grasp`

---

## Putting It All Together

Here's a complete humanoid robot system using all concepts:

```
┌──────────────┐   /camera/image   ┌──────────────┐
│ Camera Node  │────(Topic: Pub)───►│  Detector    │
└──────────────┘                    └──────┬───────┘
                                           │
                                           │ /objects (Topic: Pub)
                                           ▼
                                    ┌──────────────┐
                                    │  Planning    │◄────Service: "Plan grasp"
                                    └──────┬───────┘         │
                                           │                 │
                                           │ /cmd_vel        │
                                           │ (Topic: Pub)    │
                                           ▼          ┌──────┴───────┐
                                    ┌──────────────┐ │ Grasp Planner│
                                    │ Motor Driver │ │  (Service    │
                                    └──────────────┘ │   Server)    │
                                                     └──────────────┘
```

**What's happening:**
1. **Camera publishes** images (topic)
2. **Detector subscribes**, processes, **publishes** objects (topic)
3. **Planning subscribes** to objects
4. **Planning calls service** to compute grasp (service request-response)
5. **Planning publishes** motor commands (topic)
6. **Motor driver subscribes** and executes (topic)

**Mix of topics and services** for optimal communication.

---

**Next**: Now that you understand nodes, topics, and services conceptually, **Page 4: Installing ROS 2 and Creating Your First Node** will guide you through hands-on setup and writing your first ROS 2 program.

---

*"Nodes are the workers, topics are the broadcasts, services are the conversations."*
