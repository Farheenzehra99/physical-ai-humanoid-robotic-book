# Hardware_Proxy Usage Guide

## When to Invoke This Skill

Invoke Hardware_Proxy skill when you need to:

### Primary Use Cases
1. **Real Robot Control**: Interface with actual robotic hardware
2. **Hardware Integration**: Connect to motors, sensors, and actuators
3. **Sim-to-Real Transfer**: Deploy simulated policies on real robots
4. **Safety Implementation**: Implement emergency stop and fault handling
5. **Hardware Calibration**: Calibrate and zero robot joints
6. **Sensor Data Acquisition**: Read from real sensors (IMU, encoders, etc.)
7. **Protocol Implementation**: Implement CAN, Serial, Ethernet communication
8. **Multi-Robot Coordination**: Manage multiple robot hardware interfaces

### Specific Triggers
- User asks to "connect to real robot" or "deploy on hardware"
- User mentions "Unitree", "Go2", "H1", or specific robot models
- User wants to "test on real robot" or "run on actual hardware"
- User needs "hardware interface" or "motor control"
- User asks about "CAN bus", "serial communication", or hardware protocols
- User mentions "emergency stop" or "safety system"
- User needs to "read sensors" or "control actuators"
- User asks to "transfer from simulation to real robot"

### Context Indicators
- Development has progressed beyond simulation
- Real robot hardware is available
- Safety-critical deployment
- Final system integration
- Hardware-in-the-loop testing
- Performance validation on real hardware

## When NOT to Invoke This Skill

Do not invoke when:
- Only simulation testing is needed (use Gazebo_Sim or IsaacSim_Pipeline)
- Robot model design phase (use URDF_Designer)
- AI training phase (use IsaacSim_Pipeline, VLA_Controller)
- No physical robot available
- Pure visualization needs (use Unity_Vis)
- High-level ROS2 communication only (use ROS2_Core)

## Skill Activation Examples

### Example 1: Direct Request
```
User: "Connect to my Unitree Go2 robot and control it"
→ Activate Hardware_Proxy skill
```

### Example 2: Sim-to-Real Request
```
User: "Deploy my trained policy from Isaac Sim to the real robot"
→ Activate Hardware_Proxy skill (sim-to-real transfer)
```

### Example 3: Hardware Setup
```
User: "Set up CAN bus communication with my custom humanoid motors"
→ Activate Hardware_Proxy skill (CAN interface)
```

### Example 4: Safety Implementation
```
User: "Implement emergency stop and joint limit checking for the robot"
→ Activate Hardware_Proxy skill (safety systems)
```

## Workflow Integration

### Standalone Usage
Hardware_Proxy can be used independently for direct hardware control.

### Combined with Other Skills
- **Hardware_Proxy + ROS2_Core**: Full ROS2 hardware integration
- **VLA_Controller + Hardware_Proxy**: AI policy on real robot
- **Edge_Deploy + Hardware_Proxy**: Optimized models on robot hardware
- **Hardware_Proxy + Gazebo_Sim**: Hardware-in-the-loop simulation
- **URDF_Designer + Hardware_Proxy**: Map URDF to actual hardware

## Sequential Usage Pattern
```
1. URDF_Designer → Design robot model
2. Gazebo_Sim or IsaacSim_Pipeline → Test in simulation
3. VLA_Controller → Train AI policy
4. Edge_Deploy → Optimize for edge device
5. Hardware_Proxy → Connect to real robot
6. Hardware_Proxy → Deploy and test
7. ROS2_Core → Full system integration
```

## Priority Level
**CRITICAL** - Essential for deploying any control to real robotic hardware.

## Expected Outputs
- Hardware interface Python code
- Communication protocol implementations
- Safety system implementations
- Calibration scripts
- Hardware configuration files
- Emergency stop procedures
- Hardware diagnostic tools
- ROS2 hardware bridge nodes
- Testing and validation scripts

## Common Hardware Platforms

### Unitree Go2 (Quadruped)
- Interface: Ethernet UDP
- Control: High-level locomotion commands
- Sensors: IMU, joint encoders, cameras
- SDK: unitree_sdk2

### Unitree H1 (Humanoid)
- Interface: Ethernet
- DOF: 25 (full humanoid)
- Control: Position/velocity/torque modes
- Advanced manipulation and locomotion

### Custom Mini Humanoid
- Interface: CAN bus or Serial
- DOF: 12-18 joints
- Control: Direct motor control
- Custom firmware and protocols

### Dynamixel-Based Robots
- Interface: Serial (RS-485, TTL)
- Control: Position, velocity, PWM
- SDK: dynamixel_sdk
- Popular for research platforms

## Safety Protocols

### Before Powering On
- [ ] Robot in safe, clear area
- [ ] Emergency stop accessible
- [ ] All connections secure
- [ ] Software tested in simulation
- [ ] Observers ready
- [ ] Tether attached (if applicable)

### Initial Testing
- [ ] Start with motors disabled
- [ ] Test communication only
- [ ] Enable motors one at a time
- [ ] Test with low gains
- [ ] Verify sensor readings
- [ ] Test emergency stop

### Progressive Deployment
1. Communication test (motors disabled)
2. Single joint motion
3. Multi-joint coordinated motion
4. Low-speed full system test
5. Gradual speed increase
6. Full operational testing

## Validation Checklist
- [ ] Hardware connection established
- [ ] Communication protocol working
- [ ] Motor commands reach actuators
- [ ] Sensor data is accurate
- [ ] Joint limits enforced
- [ ] Velocity limits enforced
- [ ] Emergency stop functional
- [ ] Temperature monitoring active
- [ ] Battery monitoring active
- [ ] Communication timeout handling works
- [ ] Fault logging functional
- [ ] All safety systems tested
- [ ] Calibration completed
- [ ] URDF matches real robot

## Common Hardware Issues

### Issue: Cannot connect to robot
- **Check**: Network connection, IP address, firewall
- **Solution**: Ping robot, verify network settings, check cables

### Issue: Motors not responding
- **Check**: Motor enable signal, power supply, communication
- **Solution**: Verify enable command, check power, test communication

### Issue: Erratic motion/jittering
- **Check**: Control gains, communication frequency, noise
- **Solution**: Lower gains, increase update rate, add filtering

### Issue: Robot falls/unstable
- **Check**: Joint limits, control gains, sensor calibration
- **Solution**: Verify calibration, tune gains, check balance

### Issue: Communication timeout
- **Check**: Network stability, processing time, buffer overflows
- **Solution**: Increase timeout, optimize code, reduce data rate

### Issue: Overheating motors
- **Check**: Current limits, duty cycle, cooling
- **Solution**: Reduce torque commands, add cooling, check for binding

## Hardware-Specific Best Practices

### For Unitree Robots
- Use official SDK for reliable communication
- Respect control frequency limits (500 Hz for Go2)
- Monitor safety warnings from robot
- Use high-level commands for locomotion
- Low-level control requires expertise

### For Custom CAN Bus Systems
- Set appropriate bitrate (1 Mbps typical)
- Implement proper CAN filtering
- Handle bus-off conditions
- Use consistent message IDs
- Implement heartbeat monitoring

### For Serial Communication
- Use sufficient baud rate (921600 for high-rate)
- Implement proper framing and checksums
- Handle partial reads
- Set appropriate timeouts
- Buffer management is critical

## Performance Targets

### Control Frequency
- **Low-level control**: 200-500 Hz
- **High-level control**: 20-50 Hz
- **Sensor reading**: 100-1000 Hz
- **Safety monitoring**: >= Control frequency

### Latency
- **Command to motion**: < 10ms
- **Sensor to software**: < 5ms
- **Emergency stop**: < 1ms (hardware)

### Reliability
- **Communication success rate**: > 99.9%
- **Fault detection time**: < 100ms
- **Emergency stop response**: < 1ms

## Testing Progression

### Phase 1: Communication Only
- Connect to robot
- Read sensor data
- No motor commands
- Verify data integrity

### Phase 2: Single Joint
- Enable one motor
- Small position changes
- Monitor response
- Test limits

### Phase 3: Multi-Joint Coordinated
- Enable multiple motors
- Coordinated movements
- Test kinematic chain
- Monitor interactions

### Phase 4: Full System
- All motors enabled
- Complete behaviors
- Stress testing
- Long-duration runs

## Debugging Tools
- Hardware diagnostic scripts
- CAN/Serial sniffers
- Oscilloscope for signal analysis
- Robot-specific debug tools
- ROS2 topic monitoring
- Real-time plotting tools
- Log analysis scripts
