# Module 1, Chapter 2, Page 5: Introduction to rclpy

**Book**: Physical AI & Humanoid Robotics — A Practical Guide
**Author**: Syeda Farheen Zehra
**Module 1**: Foundations of Physical AI
**Chapter 2**: Bridging Python Agents to ROS Controllers

---

## What is rclpy?

**rclpy** (ROS Client Library for Python) is the Python library that lets you build ROS 2 nodes. It's the bridge between your Python code and the ROS 2 ecosystem.

Think of rclpy as your **translator**:
- You write Python code using familiar syntax
- rclpy translates your commands into ROS 2 operations
- Your nodes communicate with any other ROS 2 node (Python, C++, etc.)

**Without rclpy**: You'd have to manually handle network sockets, message serialization, threading, and all the complexity of distributed systems.

**With rclpy**: You write simple Python classes and functions. rclpy handles everything else.

### What rclpy Provides

✅ **Node creation**: Turn your Python script into a ROS 2 node
✅ **Publishers**: Send messages to topics
✅ **Subscribers**: Receive messages from topics (with automatic callbacks)
✅ **Service clients/servers**: Request-response communication
✅ **Timers**: Run functions periodically
✅ **Parameters**: Configure nodes at runtime
✅ **Logging**: Structured logging integrated with ROS 2 tools

**Key Point**: rclpy is your **only** interface to ROS 2 from Python. Every ROS 2 Python node uses rclpy.

---

## The rclpy Node Structure

Every Python ROS 2 node follows a standard pattern. Here's the conceptual structure:

### 1. Import rclpy and Required Types

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Or whatever message types you need
```

**What this does**:
- `rclpy`: The main ROS 2 Python library
- `Node`: The base class all nodes inherit from
- Message types: Import the specific messages you'll publish/subscribe to

### 2. Create a Node Class (Inheriting from Node)

```python
class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')  # Node name
        # Set up publishers, subscribers, timers here
```

**What this does**:
- Your node is a Python class that inherits from `Node`
- `__init__()` method sets up everything when node starts
- `super().__init__('node_name')` registers node with ROS 2

**Why inherit from Node?**
- You get all ROS 2 functionality (publish, subscribe, etc.)
- ROS 2 manages your node's lifecycle
- Standard pattern makes code readable

### 3. Set Up Communication (Publishers/Subscribers/Timers)

Inside `__init__()`, you configure how your node communicates:

```python
# Create a publisher
self.publisher = self.create_publisher(String, '/my_topic', 10)

# Create a subscriber
self.subscription = self.create_subscription(
    String,
    '/other_topic',
    self.callback_function,
    10
)

# Create a timer (runs callback every 0.5 seconds)
self.timer = self.create_timer(0.5, self.timer_callback)
```

**What this does**:
- `create_publisher()`: Sets up ability to send messages to a topic
- `create_subscription()`: Listens to a topic, calls your function when messages arrive
- `create_timer()`: Calls a function periodically (useful for sensors, status updates)

### 4. Define Callback Functions

Callbacks are functions that run automatically when events happen:

```python
def callback_function(self, msg):
    # This runs every time a message arrives on subscribed topic
    self.get_logger().info(f'Received: {msg.data}')

def timer_callback(self):
    # This runs every time the timer fires
    msg = String()
    msg.data = 'Hello ROS 2'
    self.publisher.publish(msg)
```

**What this does**:
- Subscriber callback: Processes incoming messages automatically
- Timer callback: Publishes messages periodically
- `self.get_logger()`: ROS 2's logging system (better than print statements)

### 5. Main Function (Starting the Node)

```python
def main(args=None):
    rclpy.init(args=args)           # Initialize ROS 2
    node = MyRobotNode()            # Create your node
    rclpy.spin(node)                # Keep node running
    node.destroy_node()             # Clean up when done
    rclpy.shutdown()                # Shutdown ROS 2
```

**What this does**:
- `rclpy.init()`: Starts up ROS 2 communication system
- `rclpy.spin()`: Keeps node running and processes callbacks
- `destroy_node()` and `shutdown()`: Clean exit when node stops

---

## Creating a Publisher in Python

A **publisher** sends messages to a topic. Here's the conceptual flow:

### Step 1: Create the Publisher (in `__init__`)

```python
self.publisher = self.create_publisher(
    String,           # Message type
    '/chatter',       # Topic name
    10                # Queue size
)
```

**Parameters Explained**:
- **Message type** (`String`): What kind of data you're sending
- **Topic name** (`'/chatter'`): The channel you're broadcasting on
- **Queue size** (`10`): How many messages to buffer if subscribers are slow

### Step 2: Publish Messages (when you have data to send)

```python
msg = String()              # Create a message object
msg.data = 'Hello World'    # Fill in the data
self.publisher.publish(msg) # Send it to the topic
```

**What happens**:
1. You create a message of the correct type
2. You fill in the message fields with your data
3. You call `publish()` and rclpy sends it to all subscribers

### Example Publisher Pattern

**Scenario**: A sensor node that publishes temperature readings every second.

```python
class TemperatureSensor(Node):
    def __init__(self):
        super().__init__('temperature_sensor')

        # Create publisher for temperature topic
        self.publisher = self.create_publisher(Float32, '/temperature', 10)

        # Create timer to publish every 1 second
        self.timer = self.create_timer(1.0, self.publish_temperature)

    def publish_temperature(self):
        # Read temperature from sensor (simplified)
        temperature = self.read_sensor()  # Returns float

        # Create and publish message
        msg = Float32()
        msg.data = temperature
        self.publisher.publish(msg)

        self.get_logger().info(f'Published: {temperature}°C')
```

**Flow**:
1. Timer fires every 1 second
2. `publish_temperature()` callback runs
3. Reads sensor value
4. Creates `Float32` message
5. Publishes to `/temperature` topic
6. Logs the action

---

## Creating a Subscriber in Python

A **subscriber** receives messages from a topic. Here's the conceptual flow:

### Step 1: Create the Subscriber (in `__init__`)

```python
self.subscription = self.create_subscription(
    String,                  # Message type
    '/chatter',              # Topic name
    self.listener_callback,  # Callback function
    10                       # Queue size
)
```

**Parameters Explained**:
- **Message type** (`String`): What kind of messages you're expecting
- **Topic name** (`'/chatter'`): The channel you're listening to
- **Callback function** (`self.listener_callback`): Your function that processes messages
- **Queue size** (`10`): How many messages to buffer if processing is slow

### Step 2: Define Callback Function

```python
def listener_callback(self, msg):
    # This runs automatically when a message arrives
    received_data = msg.data
    self.get_logger().info(f'I heard: {received_data}')

    # Process the data
    # Make decisions
    # Publish responses if needed
```

**What happens**:
1. A message arrives on `/chatter` topic
2. rclpy automatically calls your `listener_callback()` function
3. You receive the message as the `msg` parameter
4. You process the data however you need

**Important**: Your callback should be **fast**. Don't do heavy computations here—if processing takes time, use threads or separate nodes.

### Example Subscriber Pattern

**Scenario**: A motor controller that subscribes to velocity commands.

```python
class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Subscribe to velocity commands
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10
        )

    def velocity_callback(self, msg):
        # Extract velocity components
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        self.get_logger().info(
            f'Moving: {linear_speed} m/s, turning: {angular_speed} rad/s'
        )

        # Send commands to motors (simplified)
        self.set_motor_speeds(linear_speed, angular_speed)
```

**Flow**:
1. Planning node publishes `Twist` message to `/cmd_vel`
2. `velocity_callback()` runs automatically
3. Extracts linear and angular velocities from message
4. Sends commands to physical motors
5. Logs the action

---

## Publisher and Subscriber Together

Many nodes both **publish** and **subscribe**—they're part of a processing pipeline.

### Example: Object Detector Node

**What it does**:
- **Subscribes** to camera images
- Detects objects using AI model
- **Publishes** detected objects

```python
class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        # Subscribe to camera images
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )

        # Publish detected objects
        self.publisher = self.create_publisher(
            DetectedObjects,
            '/detected_objects',
            10
        )

    def image_callback(self, msg):
        # 1. Receive image from camera
        image_data = msg.data

        # 2. Run object detection AI model
        detected_objects = self.run_detector(image_data)

        # 3. Create message with results
        result_msg = DetectedObjects()
        result_msg.objects = detected_objects

        # 4. Publish results
        self.publisher.publish(result_msg)

        self.get_logger().info(
            f'Detected {len(detected_objects)} objects'
        )
```

**Data Flow**:
```
[Camera Node]
    │ Publishes Image
    ▼
/camera/image (topic)
    │
    ▼
[Object Detector Node] ◄── You are here
    │ Processes, detects
    │ Publishes results
    ▼
/detected_objects (topic)
    │
    ▼
[Planning Node]
    Receives detections, makes decisions
```

**This pattern is everywhere in robotics**:
- Input from one or more topics
- Process/transform/analyze
- Output to one or more topics

---

## The Spin Loop: Keeping Your Node Alive

When you run a node, you need to keep it alive to process callbacks. That's what `rclpy.spin()` does.

### How Spin Works

```python
def main():
    rclpy.init()
    node = MyNode()

    rclpy.spin(node)  # This line keeps node running forever

    # Code below only runs after Ctrl+C
    node.destroy_node()
    rclpy.shutdown()
```

**What `spin()` does**:
- Enters an infinite loop
- Checks for incoming messages
- Calls your subscriber callbacks when messages arrive
- Calls your timer callbacks when timers fire
- Handles service requests
- Processes all ROS 2 events

**Without `spin()`**: Your node would start, set up publishers/subscribers, then immediately exit. Callbacks would never run.

**Stopping the spin**: Press `Ctrl+C` in the terminal. rclpy handles the shutdown gracefully.

---

## Message Types: What Can You Send?

rclpy supports hundreds of standard message types, organized into packages:

### Common Message Packages

**std_msgs**: Basic types
- `String`: Text messages
- `Int32`, `Float64`: Numbers
- `Bool`: True/False

**sensor_msgs**: Sensor data
- `Image`: Camera images
- `LaserScan`: LIDAR data
- `Imu`: Inertial measurement (acceleration, rotation)
- `JointState`: Robot joint positions/velocities

**geometry_msgs**: Positions and movements
- `Point`: 3D coordinates (x, y, z)
- `Pose`: Position + orientation
- `Twist`: Linear and angular velocities (for moving robots)
- `Transform`: Coordinate frame transformations

**nav_msgs**: Navigation
- `Odometry`: Robot position estimate
- `Path`: Planned trajectory

### Using Message Types

When you want to publish or subscribe, you import the message type:

```python
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# Now you can use them
msg = Twist()
msg.linear.x = 0.5  # Move forward at 0.5 m/s
```

**Every message type has**:
- **Fields**: The data it contains (e.g., `Twist` has `linear` and `angular`)
- **Documentation**: Tells you what each field means
- **Standard format**: Everyone uses the same structure

---

## Quality of Service (QoS): Message Delivery Guarantees

When you create publishers/subscribers, you can specify **Quality of Service** (QoS) settings—how reliably messages should be delivered.

### The Queue Size Parameter

```python
self.publisher = self.create_publisher(String, '/topic', 10)
                                                        # ^^
                                                      # Queue size
```

**Queue size** (`10`): How many messages to buffer if:
- Subscribers are slow to process
- Network is temporarily congested
- System is under heavy load

**Choosing queue size**:
- **Small (1-10)**: Real-time data where old messages are irrelevant (sensor readings, velocity commands)
- **Large (100+)**: Important data that shouldn't be lost (logging, diagnostics)

### Advanced QoS Profiles (Conceptual)

ROS 2 also supports advanced QoS policies:
- **Reliability**: Guaranteed delivery vs best-effort
- **Durability**: New subscribers get recent messages vs only new messages
- **Deadline**: Messages must arrive within time limit

**For now**: Just use queue size `10` as default. You'll learn advanced QoS later when optimizing performance.

---

## Best Practices for rclpy Nodes

### 1. One Node, One Job
Keep nodes focused on a single task. Don't create a "do everything" node.

✅ **Good**: `camera_driver`, `object_detector`, `motion_planner` (separate nodes)
❌ **Bad**: `robot_brain` (one giant node doing everything)

### 2. Fast Callbacks
Subscriber callbacks should execute quickly (\&lt;10ms ideally).

✅ **Good**: Extract data, set flags, return quickly
❌ **Bad**: Train neural network inside callback (blocks other callbacks)

### 3. Use Logging, Not Print
```python
self.get_logger().info('Status message')     # Good - ROS 2 integrated
print('Status message')                      # Bad - not captured by ROS tools
```

### 4. Descriptive Node Names
```python
super().__init__('left_wheel_controller')    # Good - clear what it does
super().__init__('node_1')                   # Bad - meaningless
```

### 5. Clean Shutdown
Always let `rclpy.shutdown()` run for proper cleanup:
```python
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
```

---

## Key Concepts Summary

**rclpy**:
- Python library for creating ROS 2 nodes
- Handles all ROS 2 communication complexity
- Required for every Python ROS 2 node

**Node Creation**:
- Inherit from `Node` class
- Call `super().__init__('node_name')` to register
- Set up publishers/subscribers/timers in `__init__()`

**Publishers**:
- Created with `create_publisher(MessageType, '/topic', queue_size)`
- Send messages with `publisher.publish(msg)`
- Fire-and-forget (don't wait for acknowledgment)

**Subscribers**:
- Created with `create_subscription(MessageType, '/topic', callback, queue_size)`
- Callback runs automatically when messages arrive
- Should be fast (don't block)

**Spin**:
- `rclpy.spin(node)` keeps node alive and processes callbacks
- Required for subscribers and timers to work
- Exit with `Ctrl+C`

**Message Types**:
- Standard formats from packages (`std_msgs`, `sensor_msgs`, `geometry_msgs`)
- Import what you need, use provided fields
- Ensures interoperability across all ROS 2 nodes

---

**Next**: Now that you understand rclpy conceptually, the next pages will guide you through writing your first complete Python ROS 2 nodes—a simple publisher, a simple subscriber, and then combining them into intelligent agents.

---

*"rclpy is your gateway to ROS 2. Master it, and you can build any robotic intelligence in Python."*
