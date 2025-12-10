# Hardware_Proxy Skill

## Purpose
Provides hardware abstraction and integration for real robotic platforms including Unitree robots (Go2, H1), mini humanoids, and custom hardware. This skill bridges simulation/AI control with actual robot actuators, sensors, and safety systems.

## Core Capabilities
- Hardware abstraction layer (HAL)
- Motor controller integration (CAN, Ethernet, Serial)
- Sensor data acquisition and processing
- Safety system implementation
- Hardware diagnostics and monitoring
- Emergency stop and fault handling
- Calibration and zeroing procedures
- Real-time communication protocols
- Sim-to-real transfer validation
- Multi-robot coordination

## Pipeline

### 1. Hardware Interface Setup
```python
from hardware_proxy import RobotInterface

class HumanoidRobot:
    def __init__(self, robot_type='unitree_h1'):
        self.interface = RobotInterface(robot_type)
        self.interface.connect()
        self.interface.enable_motors()
        self.safety = SafetyMonitor(self.interface)

    def connect(self):
        """Establish connection to robot hardware"""
        if self.interface.connect():
            print("Connected to robot")
            self.calibrate()
        else:
            raise ConnectionError("Failed to connect to robot")
```

### 2. Motor Control
```python
def set_joint_positions(self, positions, max_velocity=1.0):
    """Send joint position commands with safety checks"""

    # Validate inputs
    if not self.safety.check_joint_limits(positions):
        print("Warning: Position out of limits, clamping")
        positions = self.safety.clamp_to_limits(positions)

    # Apply velocity limits
    current_pos = self.get_joint_positions()
    delta = positions - current_pos
    if np.linalg.norm(delta) > max_velocity * self.dt:
        delta = delta / np.linalg.norm(delta) * max_velocity * self.dt
        positions = current_pos + delta

    # Send commands
    self.interface.send_joint_commands(positions)
```

### 3. Sensor Reading
```python
def get_observations(self):
    """Collect all sensor data"""
    return {
        'joint_positions': self.interface.get_joint_positions(),
        'joint_velocities': self.interface.get_joint_velocities(),
        'joint_torques': self.interface.get_joint_torques(),
        'imu_orientation': self.interface.get_imu_orientation(),
        'imu_angular_velocity': self.interface.get_imu_angular_velocity(),
        'imu_linear_acceleration': self.interface.get_imu_acceleration(),
        'contact_forces': self.interface.get_contact_sensors(),
        'battery_voltage': self.interface.get_battery_status()
    }
```

### 4. Safety Monitoring
```python
class SafetyMonitor:
    def __init__(self, robot_interface):
        self.interface = robot_interface
        self.emergency_stop = False
        self.fault_log = []

    def check_safety(self):
        """Continuous safety checks"""
        checks = {
            'joint_limits': self.check_joint_limits(),
            'temperature': self.check_motor_temperature(),
            'battery': self.check_battery_level(),
            'communication': self.check_communication_timeout(),
            'imu_anomaly': self.check_imu_anomaly()
        }

        for check_name, passed in checks.items():
            if not passed:
                self.handle_fault(check_name)

        return all(checks.values())

    def handle_fault(self, fault_type):
        """Handle safety faults"""
        self.fault_log.append({
            'timestamp': time.time(),
            'type': fault_type
        })

        if fault_type in ['joint_limits', 'temperature', 'imu_anomaly']:
            self.emergency_stop_robot()
```

## Key Functions

### `connect_robot(robot_type, config)`
Establishes connection to robot hardware interface.

### `send_joint_commands(positions, velocities, torques)`
Sends control commands to robot actuators.

### `get_sensor_data(sensor_types)`
Retrieves data from robot sensors.

### `calibrate_robot(calibration_type)`
Performs hardware calibration procedures.

### `emergency_stop()`
Immediately stops all robot motion.

## Examples

### Example 1: Unitree Go2 Integration
```python
from hardware_proxy.unitree import UnitreeGo2

class Go2Controller:
    def __init__(self):
        self.robot = UnitreeGo2()
        self.robot.connect(interface='ethernet', ip='192.168.123.15')

    def walk_forward(self, velocity=0.3):
        """Command the robot to walk forward"""
        cmd = {
            'mode': 'velocity',
            'vx': velocity,  # m/s
            'vy': 0.0,
            'omega': 0.0  # rad/s
        }
        self.robot.send_command(cmd)

    def get_state(self):
        """Get robot state"""
        return {
            'position': self.robot.get_base_position(),
            'orientation': self.robot.get_base_orientation(),
            'joint_states': self.robot.get_joint_states(),
            'imu': self.robot.get_imu_data()
        }
```

### Example 2: Custom Humanoid Hardware
```python
from hardware_proxy import CANInterface
import struct

class CustomHumanoid:
    def __init__(self, can_channel='can0'):
        self.can = CANInterface(channel=can_channel, bitrate=1000000)
        self.motor_ids = list(range(1, 13))  # 12 motors
        self.joint_limits = self.load_joint_limits()

    def send_position_command(self, motor_id, position, kp=10.0, kd=1.0):
        """Send position command via CAN bus"""
        # Pack command: [position (float), kp (float), kd (float)]
        data = struct.pack('<fff', position, kp, kd)

        message = {
            'arbitration_id': motor_id,
            'data': data,
            'is_extended_id': False
        }
        self.can.send(message)

    def read_motor_state(self, motor_id):
        """Read motor position, velocity, torque from CAN"""
        msg = self.can.receive(timeout=0.01)
        if msg and msg.arbitration_id == motor_id + 0x100:
            position, velocity, torque = struct.unpack('<fff', msg.data)
            return {
                'position': position,
                'velocity': velocity,
                'torque': torque
            }
        return None
```

### Example 3: ROS2 Hardware Bridge
```python
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

class ROS2HardwareBridge:
    def __init__(self, robot):
        self.robot = robot
        self.node = rclpy.create_node('hardware_bridge')

        # Publishers
        self.joint_state_pub = self.node.create_publisher(
            JointState, '/joint_states', 10)

        # Subscribers
        self.joint_cmd_sub = self.node.create_subscription(
            Float32MultiArray,
            '/joint_commands',
            self.joint_command_callback,
            10
        )

        # Timer for state publishing
        self.timer = self.node.create_timer(0.01, self.publish_state)  # 100Hz

    def joint_command_callback(self, msg):
        """Receive joint commands from ROS2 and send to hardware"""
        positions = np.array(msg.data)
        self.robot.set_joint_positions(positions)

    def publish_state(self):
        """Publish robot state to ROS2"""
        state = self.robot.get_observations()

        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.position = state['joint_positions'].tolist()
        msg.velocity = state['joint_velocities'].tolist()
        msg.effort = state['joint_torques'].tolist()

        self.joint_state_pub.publish(msg)
```

### Example 4: Sim-to-Real Transfer
```python
class SimToRealAdapter:
    def __init__(self, sim_policy, real_robot):
        self.policy = sim_policy
        self.robot = real_robot
        self.observation_adapter = ObservationAdapter()
        self.action_adapter = ActionAdapter()

    def run_policy(self, duration=10.0, control_freq=50):
        """Run simulation-trained policy on real robot"""
        dt = 1.0 / control_freq
        start_time = time.time()

        while time.time() - start_time < duration:
            # Get real robot observations
            real_obs = self.robot.get_observations()

            # Adapt observations to match simulation format
            sim_obs = self.observation_adapter.real_to_sim(real_obs)

            # Run policy
            sim_action = self.policy.predict(sim_obs)

            # Adapt actions to real robot format
            real_action = self.action_adapter.sim_to_real(sim_action)

            # Apply safety filters
            safe_action = self.apply_safety_filters(real_action)

            # Send to robot
            self.robot.set_joint_positions(safe_action)

            time.sleep(dt)

    def apply_safety_filters(self, action):
        """Apply safety constraints to actions"""
        # Clamp to joint limits
        action = np.clip(action, self.robot.joint_min, self.robot.joint_max)

        # Limit velocity
        current = self.robot.get_joint_positions()
        max_delta = self.robot.max_joint_velocity * self.dt
        action = np.clip(action, current - max_delta, current + max_delta)

        return action
```

## Dependencies
- pyserial (for serial communication)
- python-can (for CAN bus)
- socket (for Ethernet/UDP)
- rclpy (for ROS2 integration)
- numpy
- Robot-specific SDKs:
  - unitree_sdk2 (Unitree robots)
  - dynamixel_sdk (Dynamixel servos)
  - Custom motor controller libraries

## Best Practices
- Always implement emergency stop functionality
- Use watchdog timers for communication timeout
- Validate all commands before sending to hardware
- Log all hardware faults and anomalies
- Implement soft start/stop (gradual acceleration)
- Monitor motor temperatures continuously
- Keep battery level above safe threshold
- Test in simulation before real hardware
- Start with low gains and increase gradually
- Implement position and velocity limits
- Use thread-safe communication
- Handle disconnections gracefully
- Implement calibration verification
- Document hardware-specific quirks
- Test safety systems thoroughly

## Communication Protocols

### CAN Bus
- High reliability, industrial standard
- 1 Mbps typical for robotics
- Multi-master, real-time
- Used for: Motor controllers, sensors

### Ethernet/UDP
- High bandwidth, low latency
- 100 Mbps - 1 Gbps
- Good for: High-rate sensor data, cameras
- Used by: Unitree robots, modern platforms

### Serial (UART)
- Simple, widely supported
- 115200 - 921600 baud typical
- Good for: Simple sensors, debugging
- Used for: IMUs, GPS, debugging

### SPI/I2C
- Low-level peripheral communication
- Direct sensor access
- Used for: IMUs, ADCs, local sensors

## Safety Systems

### Hardware Level
- Emergency stop button (physical)
- Overcurrent protection
- Thermal shutdown
- Watchdog timers

### Software Level
- Joint limit enforcement
- Velocity limit enforcement
- Collision detection
- Fall detection
- Communication timeout
- Battery level monitoring

### Operational
- Tether for initial testing
- Foam padding during development
- Clear workspace
- Human supervisor
- Gradual testing progression

## Hardware-Specific Configurations

### Unitree Go2
- Interface: Ethernet UDP
- Control frequency: 500 Hz
- Joint control: Position + PD gains
- Sensors: IMU, joint encoders, foot contact

### Unitree H1
- Interface: Ethernet
- DOF: 25 (arms, legs, waist, neck)
- Control modes: Position, velocity, torque
- Sensors: IMU, joint sensors, cameras

### Mini Humanoid (Custom)
- Interface: CAN or Serial
- DOF: 12-18 typical
- Control: Position or PWM
- Sensors: IMU, joint encoders

## Integration Points
- Receives commands from VLA_Controller
- Connects to ROS2_Core for system integration
- Uses models deployed by Edge_Deploy
- Validates against Gazebo_Sim and IsaacSim_Pipeline
- Integrates with URDF_Designer for kinematic mapping
