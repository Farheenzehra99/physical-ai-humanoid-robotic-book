# ROS2_Core Usage Guide

## When to Invoke This Skill

Invoke ROS2_Core skill when you need to:

### Primary Use Cases
1. **Create ROS2 Nodes**: Set up new nodes for robot control or data processing
2. **Implement Topic Communication**: Publish or subscribe to robot data streams
3. **Service Communication**: Implement request/response patterns
4. **Parameter Management**: Configure robot behavior dynamically
5. **Transform Management**: Handle coordinate frame transformations
6. **Timer-based Control**: Implement periodic control loops

### Specific Triggers
- User asks to "create a ROS2 node"
- User wants to "publish to a topic" or "subscribe to sensor data"
- User needs to "call a service" or "implement a service"
- User mentions "robot control" with ROS2 context
- User asks about "joint states", "velocity commands", "sensor data"
- User needs to "broadcast transforms" or "lookup frames"
- User wants to implement a "control loop" or "periodic callback"

### Context Indicators
- Project uses ROS2 as middleware
- Robot requires real-time communication
- System needs distributed node architecture
- Integration with ROS2 ecosystem tools

## When NOT to Invoke This Skill

Do not invoke when:
- User only needs simulation visualization (use Gazebo_Sim or Unity_Vis)
- Task is about robot model design only (use URDF_Designer)
- Focus is on AI training without ROS2 integration (use IsaacSim_Pipeline)
- Task is purely about deployment (use Edge_Deploy)
- User wants GUI visualization without node communication

## Skill Activation Examples

### Example 1: Direct Request
```
User: "Create a ROS2 node that publishes velocity commands"
→ Activate ROS2_Core skill
```

### Example 2: Implicit Request
```
User: "I need to control my robot's joints in Python"
→ Activate ROS2_Core skill (joints → topic communication)
```

### Example 3: Integration Request
```
User: "Connect my camera sensor data to the AI controller"
→ Activate ROS2_Core skill (sensor topic → subscriber)
```

### Example 4: Service Request
```
User: "Add a service to enable/disable motors"
→ Activate ROS2_Core skill (service implementation)
```

## Workflow Integration

### Standalone Usage
ROS2_Core can be used independently for basic node and communication setup.

### Combined with Other Skills
- **ROS2_Core + URDF_Designer**: Load robot model in ROS2 node
- **ROS2_Core + Gazebo_Sim**: Control simulated robot via topics
- **ROS2_Core + Hardware_Proxy**: Bridge simulation and real hardware
- **ROS2_Core + VLA_Controller**: Provide sensor data to AI models
- **ROS2_Core + Edge_Deploy**: Deploy ROS2 nodes to edge devices

## Priority Level
**HIGH** - Core infrastructure skill required for most robotic applications using ROS2.

## Expected Outputs
- Python ROS2 node code
- Publisher/Subscriber implementations
- Service/Client implementations
- Launch files (if needed)
- Package configuration (package.xml, setup.py)

## Validation Checklist
- [ ] Node compiles without errors
- [ ] Topics are published/subscribed correctly
- [ ] Services respond appropriately
- [ ] QoS settings are appropriate for use case
- [ ] Error handling is implemented
- [ ] Logging provides meaningful information
- [ ] Code follows ROS2 best practices
