# Module 1, Chapter 1, Page 3b: Nodes, Topics, and Services - Practical Implementation

**Book**: Physical AI & Humanoid Robotics — A Practical Guide
**Author**: Syeda Farheen Zehra
**Module 1**: Foundations of Physical AI
**Chapter 1**: The Robotic Nervous System (ROS 2)

---

## Introduction

Now that you understand the concepts of nodes, topics, and services, let's implement them in code using ROS 2's Python client library (rclpy). This practical lesson will show you exactly how to create publishers, subscribers, and services in Python.

---

## Creating a Simple Publisher Node

Let's create a node that publishes messages to a topic:

### Publisher Node Code (publisher_member_function.py)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Breakdown:
- **Node creation**: `MinimalPublisher` inherits from `Node`
- **Publisher setup**: Creates a publisher for String messages on topic 'topic'
- **Timer callback**: Automatically publishes messages every 0.5 seconds
- **Message creation**: Creates String message with incrementing counter
- **Publishing**: Uses `publish()` method to send message to topic

---

## Creating a Simple Subscriber Node

Now let's create a node that subscribes to the topic:

### Subscriber Node Code (subscriber_member_function.py)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Breakdown:
- **Subscription creation**: `create_subscription()` for String messages on topic 'topic'
- **Callback function**: `listener_callback` automatically called when message arrives
- **Message processing**: Prints the received message data
- **Spin**: Keeps the node running and listening for messages

---

## Publisher-Subscriber Communication Flow

```
┌─────────────────────┐                    ┌─────────────────────┐
│   Publisher Node    │                    │   Subscriber Node   │
│                     │                    │                     │
│ 1. Create Publisher │                    │ 2. Create Subscriber│
│    for 'topic'      │───────────────────►│    for 'topic'      │
│                     │   String Messages  │                     │
│ 3. Timer Callback   │                    │ 4. Callback Function│
│    creates and      │                    │    processes msg    │
│    publishes msg    │                    │                     │
└─────────────────────┘                    └─────────────────────┘
```

---

## Creating a Service Server Node

Now let's create a service that responds to requests:

### Service Server Code (service_member_function.py)

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a}, b: {request.b}\n')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()

    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Breakdown:
- **Service creation**: `create_service()` for AddTwoInts service named 'add_two_ints'
- **Callback function**: `add_two_ints_callback` processes request and returns response
- **Request/response handling**: Accesses request parameters and sets response value

---

## Creating a Service Client Node

Now let's create a client that calls the service:

### Service Client Code (client_member_function.py)

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(f'Result of add_two_ints: {response.sum}')
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Code Breakdown:
- **Client creation**: `create_client()` for AddTwoInts service
- **Service waiting**: Waits until service is available
- **Request sending**: Creates request and sends asynchronously
- **Response handling**: Waits for and processes the response

---

## Service Communication Flow

```
┌─────────────────────┐                    ┌─────────────────────┐
│   Client Node       │                    │   Server Node       │
│                     │                    │                     │
│ 1. Create Client    │                    │ 3. Create Service   │
│    for 'add_two_ints│                    │    for 'add_two_ints│
│                     │                    │                     │
│ 2. Send Request     │───────────────────►│ 4. Receive Request  │
│    (a=1, b=2)       │   Request/Response │    and process      │
│                     │                    │                     │
│ 6. Receive Response │◄───────────────────│ 5. Send Response    │
│    (sum=3)          │                    │    (sum=3)          │
└─────────────────────┘                    └─────────────────────┘
```

---

## Complete Example: Temperature Monitor System

Let's create a more realistic example with multiple nodes working together:

### Temperature Publisher (temperature_publisher.py)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TemperaturePublisher(Node):

    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher_ = self.create_publisher(Float32, 'temperature', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float32()
        # Simulate temperature reading (20-30 degrees)
        msg.data = 20.0 + random.uniform(0, 10)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing temperature: {msg.data:.2f}°C')

def main(args=None):
    rclpy.init(args=args)
    temp_publisher = TemperaturePublisher()

    try:
        rclpy.spin(temp_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        temp_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Temperature Monitor (temperature_monitor.py)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from example_interfaces.srv import SetBool

class TemperatureMonitor(Node):

    def __init__(self):
        super().__init__('temperature_monitor')
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.temperature_callback,
            10)

        # Service to check if temperature is safe
        self.srv = self.create_service(
            SetBool,
            'check_temperature_safe',
            self.check_temperature_callback
        )
        self.current_temp = 0.0

    def temperature_callback(self, msg):
        self.current_temp = msg.data
        if self.current_temp > 28.0:
            self.get_logger().warn(f'Temperature too high: {self.current_temp:.2f}°C')
        else:
            self.get_logger().info(f'Current temperature: {self.current_temp:.2f}°C')

    def check_temperature_callback(self, request, response):
        response.success = self.current_temp <= 28.0
        response.message = f'Temperature {self.current_temp:.2f}°C is {"safe" if response.success else "unsafe"}'
        return response

def main(args=None):
    rclpy.init(args=args)
    temp_monitor = TemperatureMonitor()

    try:
        rclpy.spin(temp_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        temp_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## How to Run the Examples

### 1. Create a ROS 2 Package
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_examples
cd my_robot_examples/my_robot_examples
```

### 2. Copy the Python files into the package

### 3. Update setup.py to include the executables

### 4. Run the nodes
```bash
# Terminal 1: Start the temperature publisher
ros2 run my_robot_examples temperature_publisher

# Terminal 2: Start the temperature monitor
ros2 run my_robot_examples temperature_monitor

# Terminal 3: Call the service
ros2 service call /check_temperature_safe example_interfaces/srv/SetBool "{data: true}"
```

---

## Key Implementation Takeaways

### For Publishers:
- Use `create_publisher()` to create a publisher
- Create message objects and set their data
- Call `publish()` to send messages
- Use timers for regular publishing

### For Subscribers:
- Use `create_subscription()` to subscribe to topics
- Implement callback functions to process messages
- The callback is automatically called when messages arrive

### For Services:
- **Server**: Use `create_service()` and implement callback that returns response
- **Client**: Use `create_client()` and `call_async()` for requests
- Services are synchronous - client waits for response

### Best Practices:
- ✅ Use descriptive topic and service names
- ✅ Handle exceptions and shutdown gracefully
- ✅ Use appropriate message types for your data
- ✅ Keep callbacks short and fast
- ✅ Use QoS (Quality of Service) settings for performance tuning

---

## Summary

This practical lesson showed you how to implement the theoretical concepts:

1. **Nodes** are Python classes inheriting from `rclpy.node.Node`
2. **Topics** use publishers (`create_publisher`) and subscribers (`create_subscription`)
3. **Services** use servers (`create_service`) and clients (`create_client`)
4. **Communication** happens through ROS 2's middleware automatically

The code examples demonstrate real-world patterns you'll use in your own robotic applications.

---

**Next**: **Page 4: Setting Up Your ROS 2 Development Environment** will guide you through installing ROS 2 and creating your first complete robotic application.