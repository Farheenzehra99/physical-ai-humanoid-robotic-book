# ROS2_Core Skill

## Purpose
Provides core ROS2 functionality for robotics development including node creation, topic publishing/subscribing, service calls, and Python-based robot control. This skill enables real-time communication and control of robotic systems using the ROS2 framework.

## Core Capabilities
- ROS2 node initialization and lifecycle management
- Topic publishing and subscription (sensor data, commands, state)
- Service client/server implementation
- Action server/client for long-running tasks
- Parameter management and dynamic reconfiguration
- Transform (TF2) broadcasting and listening
- Message type handling (std_msgs, sensor_msgs, geometry_msgs, custom)

## Pipeline

### 1. Node Setup
```python
import rclpy
from rclpy.node import Node

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.get_logger().info('Node initialized')
```

### 2. Topic Communication
- **Publishers**: Send commands, state updates
- **Subscribers**: Receive sensor data, feedback
- **QoS Profiles**: Configure reliability and durability

### 3. Service Integration
- Synchronous request/response patterns
- Timeout handling and error recovery
- Custom service definitions

### 4. Transform Management
- Coordinate frame broadcasting
- Frame transformation lookups
- TF tree visualization

## Key Functions

### `create_publisher(topic, msg_type, qos)`
Creates a publisher for sending messages on a topic.

### `create_subscription(topic, msg_type, callback, qos)`
Creates a subscriber to receive messages from a topic.

### `create_service(service_name, srv_type, callback)`
Creates a service server for request/response communication.

### `create_client(service_name, srv_type)`
Creates a service client to call remote services.

### `create_timer(period, callback)`
Creates a periodic timer for regular execution.

## Examples

### Example 1: Velocity Publisher
```python
from geometry_msgs.msg import Twist

vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
msg = Twist()
msg.linear.x = 0.5
msg.angular.z = 0.0
vel_pub.publish(msg)
```

### Example 2: Joint State Subscriber
```python
from sensor_msgs.msg import JointState

def joint_callback(msg):
    self.get_logger().info(f'Received {len(msg.position)} joints')

sub = self.create_subscription(JointState, '/joint_states', joint_callback, 10)
```

### Example 3: Service Call
```python
from std_srvs.srv import SetBool

client = self.create_client(SetBool, '/enable_motors')
request = SetBool.Request()
request.data = True
future = client.call_async(request)
```

## Dependencies
- ROS2 Humble/Iron/Rolling
- Python 3.8+
- rclpy
- Standard ROS2 message packages

## Best Practices
- Use appropriate QoS profiles for your use case
- Implement proper shutdown handling
- Log meaningful information at appropriate levels
- Handle exceptions in callbacks
- Use composition for modularity
- Test with `ros2 topic echo` and `ros2 service call`

## Integration Points
- Works with URDF_Designer for robot model loading
- Interfaces with Gazebo_Sim for simulation control
- Connects to Hardware_Proxy for real robot communication
- Provides data to VLA_Controller for AI-driven control
