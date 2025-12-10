# Module 1, Chapter 2, Page 4b: Python Agents Integration with ROS 2

**Book**: Physical AI & Humanoid Robotics — A Practical Guide
**Author**: Syeda Farheen Zehra
**Module 1**: Foundations of Physical AI
**Chapter 2**: Bridging Python Agents to ROS Controllers

---

## Introduction

This page focuses specifically on how Python agents integrate with ROS 2 nodes, providing the practical implementation details you need to build intelligent robotic systems. We'll explore the communication mechanisms, implementation patterns, and best practices for connecting your Python-based AI logic to ROS 2's distributed system.

---

## Step-by-Step Integration Process

### Step 1: Agent Initialization and ROS 2 Connection

**Python Agent Setup**:
1. Import ROS 2 client library (`rclpy`)
2. Initialize the ROS 2 context
3. Create a node that inherits from `rclpy.node.Node`
4. Register publishers, subscribers, and services
5. Enter the ROS 2 spin loop

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class PythonAgent(Node):
    def __init__(self):
        super().__init__('python_agent')

        # Create publisher for sending commands
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create subscriber for receiving sensor data
        self.image_subscriber = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10
        )

        # Initialize agent's internal state
        self.agent_state = "IDLE"
        self.get_logger().info("Python Agent initialized and connected to ROS 2")

    def image_callback(self, msg):
        # Process incoming image data
        self.process_image(msg)
```

### Step 2: Data Processing and Decision Making

**AI/Logic Layer**:
1. Receive data through callback functions
2. Process data using AI models or algorithms
3. Update internal state based on analysis
4. Decide on appropriate actions
5. Publish commands to ROS 2 topics

```python
def process_image(self, image_msg):
    # Convert ROS image to OpenCV format
    cv_image = self.ros_to_cv2(image_msg)

    # Run AI model (object detection, etc.)
    objects = self.detect_objects(cv_image)

    # Make decision based on detected objects
    if self.should_stop(objects):
        self.send_stop_command()
    elif self.should_approach(objects):
        self.send_approach_command(objects)
```

### Step 3: Command Execution

**Command Dispatch**:
1. Format decision into ROS message
2. Publish message to appropriate topic
3. Optionally wait for feedback
4. Update agent state based on outcomes

```python
def send_approach_command(self, objects):
    cmd = Twist()
    cmd.linear.x = 0.5  # Move forward at 0.5 m/s
    cmd.angular.z = 0.0  # No rotation

    self.cmd_publisher.publish(cmd)
    self.agent_state = "APPROACHING"
```

---

## Mini Pseudo-Code Examples

### Complete Agent Structure

```
INITIALIZE Python Agent Node
├── Initialize ROS 2 context
├── Create node: "intelligent_agent"
├── Setup Publishers:
│   ├── /cmd_vel (Twist) - Motor commands
│   └── /agent_status (String) - Status updates
├── Setup Subscribers:
│   ├── /camera/image (Image) - Vision input
│   ├── /scan (LaserScan) - LIDAR input
│   └── /odom (Odometry) - Position feedback
└── Setup Services:
    ├── /request_plan (GetPlan) - Path planning
    └── /execute_action (ExecuteAction) - Action execution

ENTER Main Loop (rclpy.spin)
├── Process incoming messages via callbacks
├── Update internal state based on inputs
├── Make decisions using AI logic
├── Publish commands to actuator nodes
└── Monitor system health and status
```

### Agent Communication Pattern

```
WHILE Agent Running:
    IF new_sensor_data_arrived():
        sensor_data = get_message_data()
        processed_data = run_ai_processing(sensor_data)
        decision = make_decision(processed_data)

        IF decision.requires_command():
            command_msg = format_command(decision)
            publish_to_ros_topic(command_msg)

    IF timer_event():
        status_msg = create_status_report()
        publish_status(status_msg)

    IF service_request():
        response = process_service_request()
        return_response(response)
```

---

## Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                    PYTHON AGENT (Intelligence Layer)            │
│                                                                 │
│  ┌─────────────────┐                                            │
│  │   AI Models     │                                            │
│  │  (PyTorch,      │                                            │
│  │   OpenCV, etc.) │                                            │
│  └─────────┬───────┘                                            │
│            │                                                    │
│            │ Processed Decisions                                │
│            ▼                                                    │
│  ┌─────────────────┐                                            │
│  │ Decision Engine │─────────────┐                              │
│  └─────────┬───────┘             │                              │
│            │                     │                              │
│            │                     ▼                              │
│            │        ┌─────────────────────────┐                 │
│            │        │ Command Formatter       │                 │
│            │        │ - Twist for movement    │                 │
│            │        │ - JointState for arms   │                 │
│            │        │ - Bool for emergency    │                 │
│            │        └─────────┬───────────────┘                 │
│            │                  │                                 │
└────────────┼──────────────────┼─────────────────────────────────┘
             │                  │
             │                  │
             │                  ▼
             │    ┌─────────────────────────────────────┐
             │    │    ROS 2 COMMUNICATION LAYER      │
             │    │                                     │
             │    │  ┌─────────────────────────────┐    │
             │    │  │ rclpy (Python ROS Client) │    │
             │    │  │ - Handles pub/sub         │    │
             │    │  │ - Manages service calls   │    │
             │    │  │ - Provides timers         │    │
             │    │  └─────────────┬─────────────┘    │
             │    │                │                  │
             │    │                ▼                  │
             │    │    ┌─────────────────────────┐    │
             │    │    │ DDS Middleware          │    │
             │    │    │ (Fast-DDS, Cyclone DDS) │    │
             │    │    └─────────────┬───────────┘    │
             │    └──────────────────┼────────────────┘
             │                       │
             │                       ▼
             │         ┌─────────────────────────────┐
             │         │  OTHER ROS 2 NODES          │
             │         │                             │
             │         │  ┌──────────────────────┐   │
             │         │  │ Motor Controller     │   │
             │         │  │ (C++ - Real-time)    │   │
             │         │  └─────────┬────────────┘   │
             │         │            │                │
             │         │            ▼                │
             │         │    ┌───────────────────┐    │
             │         │    │ Physical Actuators│    │
             │         │    │ (Motors, Servos)  │    │
             │         │    └───────────────────┘    │
             │         └─────────────────────────────┘
             │
    ┌────────┴────────┐
    │ SENSOR DATA     │
    │ INPUT PATH      │
    │                 │
    │ ┌─────────────┐ │
    │ │ Camera      │ │
    │ │ (Hardware)  │ │
    │ └─────────┬───┘ │
    │           │     │
    │           ▼     │
    │    ┌──────────┐ │
    │    │ Camera   │ │
    │    │ Driver   │ │
    │    │ (C++)    │ │
    │    └─────┬────┘ │
    │          │      │
    │          ▼      │
    │    [TOPIC:      │
    │     /camera/    │
    │     image]      │
    │          │      │
    │          ▼      │
    │    ┌──────────┐ │
    │    │ Python   │ │
    │    │ Agent    │ │
    │    │ Callback │ │
    │    └──────────┘ │
    └─────────────────┘
```

---

## Practical Implementation Example: Object Following Agent

### Python Agent Code

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectFollowingAgent(Node):
    def __init__(self):
        super().__init__('object_following_agent')

        # Initialize ROS 2 components
        self.bridge = CvBridge()

        # Publishers and subscribers
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_subscriber = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10
        )

        # Agent parameters
        self.follow_color_lower = np.array([20, 100, 100])  # Yellow
        self.follow_color_upper = np.array([30, 255, 255])
        self.linear_speed = 0.3
        self.angular_speed = 0.5

        self.get_logger().info("Object Following Agent initialized")

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Process image to find object
            object_center = self.find_object(cv_image)

            if object_center is not None:
                # Calculate error from center
                image_center_x = cv_image.shape[1] // 2
                error_x = object_center[0] - image_center_x

                # Generate movement command
                cmd = self.calculate_movement_command(error_x)
                self.cmd_publisher.publish(cmd)
            else:
                # Stop if no object found
                stop_cmd = Twist()
                self.cmd_publisher.publish(stop_cmd)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def find_object(self, image):
        # Convert to HSV for color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create mask for target color
        mask = cv2.inRange(hsv, self.follow_color_lower, self.follow_color_upper)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find largest contour
            largest_contour = max(contours, key=cv2.contourArea)

            # Calculate center of contour
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return (cx, cy)

        return None

    def calculate_movement_command(self, error_x):
        cmd = Twist()

        # Move forward at constant speed
        cmd.linear.x = self.linear_speed

        # Adjust angular velocity based on error
        cmd.angular.z = -error_x * 0.002  # Proportional control

        # Limit angular velocity
        cmd.angular.z = max(-self.angular_speed,
                           min(self.angular_speed, cmd.angular.z))

        return cmd

def main(args=None):
    rclpy.init(args=args)
    agent = ObjectFollowingAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Best Practices for Python Agent Integration

### 1. Communication Best Practices

✅ **Do:**
- Use appropriate message types (Twist for velocity, JointState for joints, etc.)
- Set proper Quality of Service (QoS) settings for your use case
- Use consistent topic naming conventions
- Implement proper error handling in callbacks
- Keep callbacks short and fast to avoid blocking

❌ **Don't:**
- Block in callback functions (no time.sleep() or long computations)
- Use global variables for ROS communication
- Ignore message timestamps for time-sensitive applications
- Publish to topics without proper message validation

### 2. Performance Best Practices

✅ **Do:**
- Use efficient data structures for AI processing
- Implement message filtering if needed (throttle high-frequency topics)
- Use threading for CPU-intensive operations that can't run in callbacks
- Monitor agent performance and memory usage
- Use appropriate timer rates for different tasks

❌ **Don't:**
- Process every single message if high frequency (use message filters)
- Run heavy AI computations in callbacks
- Ignore garbage collection in long-running agents
- Use synchronous service calls in time-critical paths

### 3. Safety and Reliability Best Practices

✅ **Do:**
- Implement timeout mechanisms for service calls
- Add safety checks before publishing commands
- Use watchdog timers to detect agent failures
- Implement graceful degradation when sensors fail
- Log important events and decisions for debugging

❌ **Don't:**
- Publish commands without safety validation
- Ignore error conditions in communication
- Skip error handling in critical paths
- Send commands to actuators without bounds checking

### 4. Architecture Best Practices

✅ **Do:**
- Separate ROS communication from AI logic
- Use state machines for complex behaviors
- Implement proper node lifecycle management
- Use parameter servers for configurable values
- Follow single responsibility principle for agents

❌ **Don't:**
- Mix ROS communication and business logic
- Create monolithic agents that do everything
- Ignore ROS 2 lifecycle node patterns for complex systems
- Hardcode values instead of using parameters

---

## Advanced Integration Patterns

### 1. Multi-Sensor Fusion Agent

```python
class MultiSensorFusionAgent(Node):
    def __init__(self):
        super().__init__('multi_sensor_fusion')

        # Multiple sensor inputs
        self.camera_sub = self.create_subscription(Image, '/camera/image',
                                                  self.camera_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan',
                                                 self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data',
                                               self.imu_callback, 10)

        # Fused output
        self.fused_publisher = self.create_publisher(FusedSensorData,
                                                    '/fused_sensors', 10)
```

### 2. Hierarchical Agent Architecture

```
┌─────────────────────────────────────────────────────────┐
│                COORDINATION LAYER                       │
│  (High-level task planning, mission management)         │
└─────────────────┬───────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────┐
│                 BEHAVIOR LAYER                          │
│  (State machines, behavior trees, action selection)     │
└─────────────────┬───────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────┐
│                CONTROL LAYER                            │
│  (Motor control, trajectory execution, safety)         │
└─────────────────────────────────────────────────────────┘
```

---

## Summary

Python agents integrate seamlessly with ROS 2 through the `rclpy` client library, enabling you to:

1. **Subscribe** to sensor topics for environmental perception
2. **Process** data using AI/ML libraries (PyTorch, OpenCV, etc.)
3. **Decide** on appropriate actions based on analysis
4. **Publish** commands to actuator nodes
5. **Call services** for specialized computations

The key to successful integration is understanding the communication patterns, following best practices for performance and safety, and structuring your agents to work effectively within the ROS 2 ecosystem.

---

**Next**: **Page 5: Introduction to rclpy** will dive deep into the Python ROS 2 client library and show you how to implement the communication patterns discussed here.