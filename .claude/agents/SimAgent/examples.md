# SimAgent Examples

## Example 1: Complete Humanoid Robot Setup

### User Request
```
"Create a complete simulation environment for a bipedal humanoid robot with ROS2 control"
```

### SimAgent Execution

#### Step 1: URDF_Designer
**Task**: Design humanoid robot structure

**Execution**:
- Created 18-DOF humanoid model
- Kinematic chain: 2 legs (6 DOF each), 2 arms (3 DOF each)
- Added collision and visual geometries
- Configured joint limits and inertial properties

**Output**:
```
✓ humanoid.urdf created
✓ 18 joints configured
✓ Joint limits validated
✓ Inertia calculations complete
```

#### Step 2: Gazebo_Sim
**Task**: Set up physics simulation world

**Execution**:
- Created simulation world with ground plane
- Spawned humanoid robot at (0, 0, 1) meters
- Configured ODE physics: timestep=0.001s, iterations=50
- Set ground friction to 1.0

**Output**:
```
✓ humanoid_world.world created
✓ Robot spawned successfully
✓ Physics configured for stable contact
✓ Real-time factor: 0.95
```

#### Step 3: ROS2_Core
**Task**: Implement robot control interface

**Execution**:
- Created `humanoid_controller` node
- Publishers: `/joint_states` (100 Hz)
- Subscribers: `/joint_commands` (position control)
- Services: `/enable_motors`, `/disable_motors`

**Output**:
```
✓ ROS2 nodes created
✓ Topics active:
  - /joint_states (sensor_msgs/JointState)
  - /joint_commands (std_msgs/Float32MultiArray)
✓ Control frequency: 100 Hz
```

#### Step 4: Unity_Vis (Optional)
**Task**: Add high-quality visualization

**Execution**:
- Imported humanoid URDF to Unity
- Created living room environment
- Connected to ROS2 `/joint_states`
- Set up camera follow system

**Output**:
```
✓ Unity scene created
✓ Robot visualization active
✓ Environment: Indoor living room
✓ Rendering at 60 FPS
```

### Final Result
Complete simulation environment ready for testing. Robot can be controlled via ROS2 topics, with physics-accurate simulation in Gazebo and optional high-quality visualization in Unity.

---

## Example 2: Walking Controller Testing

### User Request
```
"I have a walking controller, test it in simulation and record a video"
```

### SimAgent Execution

#### Step 1: ROS2_Core
**Task**: Integrate walking controller

**Execution**:
- Loaded walking controller Python script
- Created node: `walking_controller`
- Subscribed to: `/joint_states`, `/imu`
- Published to: `/joint_commands`
- Implemented 50 Hz control loop

**Output**:
```
✓ Walking controller node running
✓ Control loop frequency: 50 Hz
✓ Waiting for robot state...
```

#### Step 2: Gazebo_Sim
**Task**: Test in physics simulation

**Execution**:
- Started simulation
- Robot begins walking forward
- Monitored stability and contact forces
- Physics timestep: 0.001s for accuracy

**Output**:
```
✓ Simulation running
✓ Robot walking velocity: 0.3 m/s
✓ No falls detected
✓ Contact forces stable
Duration: 30 seconds
```

#### Step 3: Unity_Vis
**Task**: Record demonstration video

**Execution**:
- Started video recording at 1920×1080, 30 FPS
- Camera following robot
- Recorded 30-second walking sequence

**Output**:
```
✓ Video recorded: walking_demo.mp4
✓ Duration: 30 seconds
✓ Resolution: 1920×1080
✓ File size: 45 MB
```

### Final Result
Walking controller successfully tested. Robot maintained stable walking for 30 seconds. High-quality demonstration video created.

---

## Example 3: Sensor Integration

### User Request
```
"Add a depth camera and LiDAR to my robot for navigation"
```

### SimAgent Execution

#### Step 1: URDF_Designer
**Task**: Add sensor links to robot model

**Execution**:
- Added camera link to robot head
  - Position: (0.05, 0, 0.03) relative to head
  - Type: Depth camera
- Added LiDAR link to robot torso
  - Position: (0, 0, 0.2) relative to torso
  - Type: 2D planar LiDAR
- Updated URDF with new links and fixed joints

**Output**:
```
✓ Camera link added to head
✓ LiDAR link added to torso
✓ Updated URDF: robot_with_sensors.urdf
✓ URDF validation passed
```

#### Step 2: Gazebo_Sim
**Task**: Configure sensor plugins in simulation

**Execution**:
- Added depth camera sensor plugin
  - Resolution: 640×480
  - FOV: 90 degrees
  - Update rate: 30 Hz
  - ROS2 topic: `/camera/depth/image_raw`
- Added LiDAR sensor plugin
  - Range: 0.1m - 30m
  - Angle: 360 degrees
  - Update rate: 10 Hz
  - ROS2 topic: `/scan`

**Output**:
```
✓ Depth camera configured
  Topic: /camera/depth/image_raw (sensor_msgs/Image)
  Publishing at 30 Hz
✓ LiDAR configured
  Topic: /scan (sensor_msgs/LaserScan)
  Publishing at 10 Hz
```

#### Step 3: ROS2_Core
**Task**: Create navigation node using sensor data

**Execution**:
- Created `obstacle_avoidance` node
- Subscribed to:
  - `/camera/depth/image_raw` (depth perception)
  - `/scan` (2D obstacle detection)
- Published to:
  - `/cmd_vel` (velocity commands)
- Implemented simple obstacle avoidance logic

**Output**:
```
✓ Navigation node created
✓ Subscribed to depth camera and LiDAR
✓ Publishing velocity commands
✓ Obstacle avoidance active
```

#### Step 4: Unity_Vis
**Task**: Visualize sensor data

**Execution**:
- Created depth camera visualization panel
- Added LiDAR scan visualization (2D overlay)
- Real-time display of sensor data

**Output**:
```
✓ Depth camera feed displayed
✓ LiDAR scan visualized (red lines)
✓ Obstacles highlighted in real-time
```

### Final Result
Robot equipped with depth camera and LiDAR. Navigation system successfully avoiding obstacles. Sensor data visualized in Unity for monitoring.

---

## Example 4: Physics Tuning

### User Request
```
"The robot keeps falling. Help me tune the physics for stable walking."
```

### SimAgent Execution

#### Step 1: Gazebo_Sim (Analysis)
**Task**: Diagnose stability issues

**Execution**:
- Analyzed current physics settings:
  - Timestep: 0.005s (too large)
  - Solver iterations: 20 (too few)
  - Ground friction: 0.5 (too low)
- Identified jittering in contact forces
- Detected joint limit violations

**Output**:
```
⚠ Issues found:
  - Timestep too large for bipedal contact
  - Insufficient solver iterations
  - Low ground friction
  - Joint velocity limits exceeded
```

#### Step 2: URDF_Designer
**Task**: Fix joint configurations

**Execution**:
- Reduced joint velocity limits from 10 rad/s to 5 rad/s
- Added joint damping: 0.5
- Verified inertia calculations for all links
- Adjusted foot collision geometry for better contact

**Output**:
```
✓ Joint limits updated
✓ Damping added to all joints
✓ Inertia verified
✓ Foot collision improved
```

#### Step 3: Gazebo_Sim (Tuning)
**Task**: Optimize physics parameters

**Execution**:
- Reduced timestep: 0.001s
- Increased solver iterations: 50
- Increased ground friction: 1.0
- Set ERP: 0.2, CFM: 0.0
- Enabled contact surface layer: 0.001m

**Output**:
```
✓ Physics parameters optimized
✓ Timestep: 0.001s
✓ Solver iterations: 50
✓ Ground friction: 1.0
✓ Real-time factor: 0.92 (acceptable)
```

#### Step 4: ROS2_Core
**Task**: Test walking controller with new physics

**Execution**:
- Restarted walking controller
- Monitored joint states and base orientation
- Tested for 60 seconds

**Output**:
```
✓ Robot stable for 60 seconds
✓ No falls detected
✓ Average velocity: 0.28 m/s
✓ Base orientation stable
Success rate: 100% (10/10 trials)
```

### Final Result
Physics tuned for stable walking. Robot successfully walks without falling. Real-time factor acceptable at 0.92.

---

## Example 5: Multi-Robot Coordination

### User Request
```
"Simulate two humanoid robots coordinating to carry a table"
```

### SimAgent Execution

#### Step 1: URDF_Designer
**Task**: Validate robot model for manipulation

**Execution**:
- Verified gripper configuration on robot arms
- Added force/torque sensors to wrists
- Confirmed kinematic chain suitable for manipulation

**Output**:
```
✓ Robot model validated
✓ Grippers configured
✓ Force sensors added
```

#### Step 2: Gazebo_Sim
**Task**: Spawn two robots and table

**Execution**:
- Spawned robot_1 at (-0.5, 0, 0)
- Spawned robot_2 at (0.5, 0, 0)
- Spawned table (1m × 0.5m) at (0, 0, 0.5)
- Configured contact physics for grasping
- Added force closure constraints

**Output**:
```
✓ robot_1 spawned
✓ robot_2 spawned
✓ table spawned
✓ Grasp physics configured
```

#### Step 3: ROS2_Core
**Task**: Implement coordination controller

**Execution**:
- Created `multi_robot_coordinator` node
- Subscribed to both robots' states
- Published commands to both robots
- Implemented synchronized grasping logic
- Coordinated lifting motion

**Output**:
```
✓ Coordination node active
✓ Both robots grasping table
✓ Synchronized lifting in progress
✓ Table height: 0.8m → 1.2m
```

#### Step 4: Unity_Vis
**Task**: Visualize coordination

**Execution**:
- Created scene with both robots and table
- Added visual indicators for grasp points
- Recorded coordination sequence

**Output**:
```
✓ Multi-robot visualization active
✓ Grasp points highlighted
✓ Video recorded: coordination_demo.mp4
```

### Final Result
Two humanoid robots successfully coordinated to lift and carry table. Grasp forces balanced, motion synchronized. Demonstration video created.
