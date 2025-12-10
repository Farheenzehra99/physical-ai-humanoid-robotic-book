# Module 1, Chapter 3, Page 9: Writing Your First Complete URDF

**Book**: Physical AI & Humanoid Robotics — A Practical Guide
**Author**: Syeda Farheen Zehra
**Module 1**: Foundations of Physical AI
**Chapter 3**: Understanding URDF for Humanoids

---

## Introduction: From Theory to Practice

You've learned **what** URDF is and explored its **components** in detail. Now it's time to **build**—to write your first complete, working URDF file from scratch.

**What You'll Create**: A 3-DOF (3 degrees of freedom) robot arm with:
- 4 links (base, upper arm, forearm, hand)
- 3 revolute joints (shoulder, elbow, wrist)
- 1 camera sensor mounted on the hand
- Complete visual, collision, and inertial properties

**What You'll Learn**:
- How to structure a URDF file
- How to define links with proper geometry
- How to connect links with joints
- How to add sensors for perception
- How to validate your URDF

By the end of this page, you'll have a complete URDF file that you can visualize in RViz and simulate in Gazebo.

---

## Step 1: Set Up Your Workspace

### Create the File Structure

First, create a directory for your URDF files:

```bash
mkdir -p ~/robot_ws/src/my_robot_description/urdf
cd ~/robot_ws/src/my_robot_description/urdf
```

Create your URDF file:

```bash
touch simple_arm.urdf
```

Open it in your favorite text editor (VS Code, nano, vim, etc.).

---

## Step 2: Start with the XML Header

Every URDF file begins with an XML declaration and a `<robot>` tag:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- All your links, joints, and sensors will go here -->

</robot>
```

**Key Points**:
- `<?xml version="1.0"?>` declares this as an XML file
- `<robot name="simple_arm">` is the root element
- Robot name should be descriptive and unique
- Don't forget the closing `</robot>` tag at the end!

---

## Step 3: Define the Base Link

The **base link** is the root of your robot—everything else connects to it (directly or indirectly).

Add this inside the `<robot>` tags:

```xml
  <!-- BASE LINK: The foundation of the arm -->
  <link name="base_link">

    <!-- Visual: What it looks like -->
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.1"/>  <!-- 10cm cube -->
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>  <!-- Gray color -->
      </material>
    </visual>

    <!-- Collision: Simplified shape for physics -->
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>

    <!-- Inertial: Mass properties for dynamics -->
    <inertial>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <mass value="2.0"/>  <!-- 2 kg base -->
      <inertia
        ixx="0.0033" ixy="0.0" ixz="0.0"
        iyy="0.0033" iyz="0.0"
        izz="0.0033"/>
      <!-- Inertia of a cube: (1/12)*m*(h²+w²) -->
    </inertia>
  </link>
```

**What This Defines**:
- **Name**: `base_link` (required by ROS 2 convention)
- **Shape**: 10cm × 10cm × 10cm cube
- **Position**: Centered at origin, elevated 5cm (origin xyz="0 0 0.05")
- **Color**: Gray (RGB: 0.5, 0.5, 0.5)
- **Mass**: 2 kg
- **Inertia**: Calculated for a cube

---

## Step 4: Add the Upper Arm Link

Now add the first arm segment:

```xml
  <!-- UPPER ARM LINK -->
  <link name="upper_arm">

    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.30"/>  <!-- 30cm long, 3cm radius -->
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.3 0.8 1.0"/>  <!-- Blue -->
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.30"/>
        <!-- Slightly larger for safety margin -->
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <mass value="1.0"/>  <!-- 1 kg -->
      <inertia
        ixx="0.0083" ixy="0.0" ixz="0.0"
        iyy="0.0083" iyz="0.0"
        izz="0.00045"/>
      <!-- Inertia of a cylinder along Z-axis -->
    </inertial>
  </link>
```

**What This Defines**:
- **Shape**: Cylinder (30cm long, 3cm radius)
- **Color**: Blue
- **Mass**: 1 kg
- **Center**: 15cm up from base (cylinder's center point)

---

## Step 5: Connect Base to Upper Arm with a Joint

Links are useless without connections. Add the shoulder joint:

```xml
  <!-- SHOULDER JOINT: Connects base to upper arm -->
  <joint name="shoulder_pan" type="revolute">

    <!-- Parent-child relationship -->
    <parent link="base_link"/>
    <child link="upper_arm"/>

    <!-- Where the child attaches to parent -->
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <!-- Upper arm starts 10cm above base -->

    <!-- Rotation axis (Z-axis = pan left/right) -->
    <axis xyz="0 0 1"/>

    <!-- Joint limits -->
    <limit
      lower="-1.57"      <!-- -90 degrees -->
      upper="1.57"       <!-- +90 degrees -->
      effort="50.0"      <!-- Max torque: 50 Nm -->
      velocity="1.0"/>   <!-- Max speed: 1 rad/s -->

    <!-- Dynamics -->
    <dynamics damping="0.7" friction="0.5"/>
  </joint>
```

**What This Defines**:
- **Type**: Revolute (rotation with limits)
- **Connects**: base_link → upper_arm
- **Attachment Point**: 10cm above base
- **Rotation**: Around Z-axis (pan motion)
- **Range**: -90° to +90° (-1.57 to +1.57 radians)
- **Limits**: 50 Nm max torque, 1 rad/s max speed

---

## Step 6: Add the Forearm Link

Continue building the arm:

```xml
  <!-- FOREARM LINK -->
  <link name="forearm">

    <visual>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.025" length="0.25"/>  <!-- 25cm long, 2.5cm radius -->
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.3 0.8 1.0"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <mass value="0.8"/>  <!-- 0.8 kg -->
      <inertia
        ixx="0.0052" ixy="0.0" ixz="0.0"
        iyy="0.0052" iyz="0.0"
        izz="0.00025"/>
    </inertial>
  </link>
```

**Changes from Upper Arm**:
- **Shorter**: 25cm instead of 30cm
- **Thinner**: 2.5cm radius instead of 3cm
- **Lighter**: 0.8 kg instead of 1 kg

---

## Step 7: Add the Elbow Joint

Connect upper arm to forearm:

```xml
  <!-- ELBOW JOINT: Connects upper arm to forearm -->
  <joint name="elbow" type="revolute">

    <parent link="upper_arm"/>
    <child link="forearm"/>

    <origin xyz="0 0 0.30" rpy="0 0 0"/>
    <!-- Forearm starts at end of upper arm (30cm up) -->

    <axis xyz="0 1 0"/>  <!-- Y-axis = bend elbow -->

    <limit
      lower="0.0"        <!-- Fully extended -->
      upper="2.618"      <!-- 150 degrees bent -->
      effort="40.0"
      velocity="1.5"/>

    <dynamics damping="0.7" friction="0.5"/>
  </joint>
```

**What This Defines**:
- **Connects**: upper_arm → forearm
- **Attachment**: 30cm up (at upper arm's end)
- **Rotation**: Around Y-axis (bending motion)
- **Range**: 0° to 150° (0 to 2.618 radians)

---

## Step 8: Add the Hand Link

The end effector (gripper/hand):

```xml
  <!-- HAND LINK (End Effector) -->
  <link name="hand">

    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.05 0.10"/>  <!-- Simple hand shape -->
      </geometry>
      <material name="green">
        <color rgba="0.2 0.8 0.2 1.0"/>  <!-- Green -->
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.05 0.10"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <mass value="0.3"/>  <!-- 0.3 kg -->
      <inertia
        ixx="0.0003" ixy="0.0" ixz="0.0"
        iyy="0.0004" iyz="0.0"
        izz="0.0002"/>
    </inertial>
  </link>
```

**What This Defines**:
- **Shape**: Box (8cm × 5cm × 10cm)
- **Color**: Green (to distinguish from arm)
- **Mass**: 0.3 kg (lightest link)

---

## Step 9: Add the Wrist Joint

Connect forearm to hand:

```xml
  <!-- WRIST JOINT: Connects forearm to hand -->
  <joint name="wrist_pitch" type="revolute">

    <parent link="forearm"/>
    <child link="hand"/>

    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <!-- Hand starts at end of forearm (25cm up) -->

    <axis xyz="0 1 0"/>  <!-- Y-axis = pitch wrist up/down -->

    <limit
      lower="-1.57"      <!-- -90 degrees -->
      upper="1.57"       <!-- +90 degrees -->
      effort="20.0"
      velocity="2.0"/>

    <dynamics damping="0.5" friction="0.3"/>
  </joint>
```

**What This Defines**:
- **Connects**: forearm → hand
- **Attachment**: 25cm up (at forearm's end)
- **Rotation**: Around Y-axis (pitch motion)
- **Range**: -90° to +90°

---

## Step 10: Add a Camera Sensor

Let's add a camera to the hand for vision:

```xml
  <!-- CAMERA LINK -->
  <link name="camera">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.02 0.05 0.02"/>  <!-- Small camera -->
      </geometry>
      <material name="black">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>

  <!-- CAMERA JOINT: Fixed to hand -->
  <joint name="camera_joint" type="fixed">
    <parent link="hand"/>
    <child link="camera"/>
    <origin xyz="0.04 0 0.05" rpy="0 0 0"/>
    <!-- Camera mounted 4cm forward, 5cm up from hand center -->
  </joint>
```

**What This Defines**:
- **Camera link**: Small black box (2cm × 5cm × 2cm)
- **Fixed joint**: Camera doesn't move relative to hand
- **Position**: 4cm forward of hand (points in direction of reach)

Now add the Gazebo plugin for camera simulation:

```xml
  <!-- GAZEBO: Camera Sensor Plugin -->
  <gazebo reference="camera">
    <sensor type="camera" name="hand_camera">
      <update_rate>30.0</update_rate>  <!-- 30 FPS -->
      <camera>
        <horizontal_fov>1.3962634</horizontal_fov>  <!-- 80 degrees -->
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>  <!-- Min distance: 2cm -->
          <far>10.0</far>    <!-- Max distance: 10m -->
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/simple_arm</namespace>
          <argument>~/image_raw:=camera/image</argument>
        </ros>
      </plugin>
    </sensor>
  </gazebo>
```

**What This Adds**:
- **Simulated camera**: Works in Gazebo simulation
- **Resolution**: 640×480 pixels
- **FPS**: 30 frames per second
- **FOV**: 80° horizontal field of view
- **Publishes to**: `/simple_arm/camera/image` ROS topic

---

## Step 11: Add Transmissions for ROS Control

Transmissions connect your joints to ROS 2 controllers:

```xml
  <!-- TRANSMISSION: Shoulder Joint -->
  <transmission name="shoulder_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="shoulder_pan">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="shoulder_motor">
      <mechanicalReduction>1</mechanicalReduction>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </actuator>
  </transmission>

  <!-- TRANSMISSION: Elbow Joint -->
  <transmission name="elbow_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="elbow">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="elbow_motor">
      <mechanicalReduction>1</mechanicalReduction>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </actuator>
  </transmission>

  <!-- TRANSMISSION: Wrist Joint -->
  <transmission name="wrist_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="wrist_pitch">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="wrist_motor">
      <mechanicalReduction>1</mechanicalReduction>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </actuator>
  </transmission>
```

**What This Enables**:
- ROS 2 controllers can command joint positions
- Each joint gets a simple 1:1 transmission (no gear reduction)
- Position interface (control joint angles, not velocities or torques)

---

## Step 12: Close the Robot Tag

Don't forget to close the main `<robot>` tag at the very end:

```xml
</robot>
```

---

## The Complete URDF File

Here's the entire file assembled:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- BASE LINK -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.0033" ixy="0.0" ixz="0.0"
               iyy="0.0033" iyz="0.0" izz="0.0033"/>
    </inertial>
  </link>

  <!-- UPPER ARM LINK -->
  <link name="upper_arm">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.30"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.3 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.30"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.0083" ixy="0.0" ixz="0.0"
               iyy="0.0083" iyz="0.0" izz="0.00045"/>
    </inertial>
  </link>

  <!-- SHOULDER JOINT -->
  <joint name="shoulder_pan" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="50.0" velocity="1.0"/>
    <dynamics damping="0.7" friction="0.5"/>
  </joint>

  <!-- FOREARM LINK -->
  <link name="forearm">
    <visual>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.025" length="0.25"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.3 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.25"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <mass value="0.8"/>
      <inertia ixx="0.0052" ixy="0.0" ixz="0.0"
               iyy="0.0052" iyz="0.0" izz="0.00025"/>
    </inertial>
  </link>

  <!-- ELBOW JOINT -->
  <joint name="elbow" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0 0 0.30" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0.0" upper="2.618" effort="40.0" velocity="1.5"/>
    <dynamics damping="0.7" friction="0.5"/>
  </joint>

  <!-- HAND LINK -->
  <link name="hand">
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.05 0.10"/>
      </geometry>
      <material name="green">
        <color rgba="0.2 0.8 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.05 0.10"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <mass value="0.3"/>
      <inertia ixx="0.0003" ixy="0.0" ixz="0.0"
               iyy="0.0004" iyz="0.0" izz="0.0002"/>
    </inertial>
  </link>

  <!-- WRIST JOINT -->
  <joint name="wrist_pitch" type="revolute">
    <parent link="forearm"/>
    <child link="hand"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20.0" velocity="2.0"/>
    <dynamics damping="0.5" friction="0.3"/>
  </joint>

  <!-- CAMERA LINK -->
  <link name="camera">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.02 0.05 0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0.0 0.0 0.0 1.0"/>
      </material>
    </visual>
  </link>

  <!-- CAMERA JOINT -->
  <joint name="camera_joint" type="fixed">
    <parent link="hand"/>
    <child link="camera"/>
    <origin xyz="0.04 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- GAZEBO CAMERA PLUGIN -->
  <gazebo reference="camera">
    <sensor type="camera" name="hand_camera">
      <update_rate>30.0</update_rate>
      <camera>
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>10.0</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/simple_arm</namespace>
          <argument>~/image_raw:=camera/image</argument>
        </ros>
      </plugin>
    </sensor>
  </gazebo>

  <!-- TRANSMISSIONS -->
  <transmission name="shoulder_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="shoulder_pan">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="shoulder_motor">
      <mechanicalReduction>1</mechanicalReduction>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </actuator>
  </transmission>

  <transmission name="elbow_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="elbow">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="elbow_motor">
      <mechanicalReduction>1</mechanicalReduction>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </actuator>
  </transmission>

  <transmission name="wrist_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="wrist_pitch">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="wrist_motor">
      <mechanicalReduction>1</mechanicalReduction>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </actuator>
  </transmission>

</robot>
```

**File Stats**:
- **4 links**: base_link, upper_arm, forearm, hand, camera
- **4 joints**: shoulder_pan, elbow, wrist_pitch, camera_joint
- **1 sensor**: Camera with Gazebo plugin
- **3 transmissions**: For the 3 actuated joints
- **Total reach**: ~0.65m (10cm base + 30cm upper + 25cm forearm)

---

## Step 13: Validate Your URDF

Before testing, check for errors:

```bash
check_urdf simple_arm.urdf
```

**Expected Output**:
```
robot name is: simple_arm
---------- Successfully Parsed XML ---------------
root Link: base_link has 1 child(ren)
    child(1):  upper_arm
        child(1):  forearm
            child(1):  hand
                child(1):  camera
```

**If you see errors**:
- Check all tags are properly closed (`</link>`, `</joint>`, etc.)
- Verify parent links exist before being referenced in joints
- Ensure all joint names are unique
- Check XML syntax (quotes around attributes, etc.)

---

## Step 14: Visualize the Kinematic Tree

See the robot's structure:

```bash
urdf_to_graphiz simple_arm.urdf
```

This creates a PDF showing the link-joint tree structure.

---

## Understanding Your Robot's Structure

### Link Hierarchy
```
base_link (2.0 kg, gray cube)
    │
    └─── shoulder_pan (revolute, Z-axis, ±90°)
         │
         └─── upper_arm (1.0 kg, blue cylinder, 30cm)
              │
              └─── elbow (revolute, Y-axis, 0°-150°)
                   │
                   └─── forearm (0.8 kg, blue cylinder, 25cm)
                        │
                        └─── wrist_pitch (revolute, Y-axis, ±90°)
                             │
                             └─── hand (0.3 kg, green box)
                                  │
                                  └─── camera_joint (fixed)
                                       │
                                       └─── camera (black box)
```

### Degrees of Freedom
- **Shoulder pan**: Left/right rotation (Z-axis)
- **Elbow**: Bend/straighten (Y-axis)
- **Wrist pitch**: Up/down tilt (Y-axis)
- **Total**: 3 DOF (controllable angles)

### Total Mass
- Base: 2.0 kg
- Upper arm: 1.0 kg
- Forearm: 0.8 kg
- Hand: 0.3 kg
- **Total**: 4.1 kg

---

## Common Mistakes and How to Fix Them

### Mistake 1: Forgot to Close Tags
```xml
<!-- WRONG -->
<link name="base_link">
  <visual>
    ...
  <!-- Missing </visual> and </link> -->

<!-- CORRECT -->
<link name="base_link">
  <visual>
    ...
  </visual>
</link>
```

### Mistake 2: Referenced Non-Existent Parent
```xml
<!-- WRONG: upper_arm doesn't exist yet -->
<joint name="elbow">
  <parent link="upper_arm"/>  <!-- Error! -->
  <child link="forearm"/>
</joint>

<link name="upper_arm">...</link>  <!-- Defined AFTER joint -->

<!-- CORRECT: Define links before referencing them -->
<link name="upper_arm">...</link>

<joint name="elbow">
  <parent link="upper_arm"/>  <!-- Now it exists -->
  <child link="forearm"/>
</joint>
```

### Mistake 3: Duplicate Joint Names
```xml
<!-- WRONG -->
<joint name="arm_joint" ...>  <!-- First use -->
<joint name="arm_joint" ...>  <!-- Duplicate! Error! -->

<!-- CORRECT -->
<joint name="shoulder_joint" ...>
<joint name="elbow_joint" ...>
```

### Mistake 4: Incorrect Origin Position
```xml
<!-- WRONG: Child floats in space -->
<joint name="elbow">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  <origin xyz="0 0 0" .../>
  <!-- Forearm starts at upper_arm's origin (bottom), not end! -->
</joint>

<!-- CORRECT: Position child at parent's end -->
<joint name="elbow">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  <origin xyz="0 0 0.30" .../>
  <!-- Forearm starts 30cm up (at upper_arm's end) -->
</joint>
```

### Mistake 5: Forgot Inertia
```xml
<!-- WRONG: No inertial properties -->
<link name="arm">
  <visual>...</visual>
  <collision>...</collision>
  <!-- Missing <inertial> -->
</link>

<!-- CORRECT: Always include inertial for dynamic simulation -->
<link name="arm">
  <visual>...</visual>
  <collision>...</collision>
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.01" iyy="0.01" izz="0.001"
             ixy="0" ixz="0" iyz="0"/>
  </inertial>
</link>
```

---

## What You've Accomplished

Congratulations! You've created your first complete URDF robot description. You now have:

✅ **A working 3-DOF robot arm** with proper kinematic structure
✅ **Complete link definitions** with visual, collision, and inertial properties
✅ **Properly connected joints** with realistic limits and dynamics
✅ **A camera sensor** for perception (works in Gazebo simulation)
✅ **ROS 2 control integration** via transmissions
✅ **A validated URDF file** (checked with `check_urdf`)

### Skills Gained
- **URDF syntax**: XML structure, tags, attributes
- **Link definition**: Geometry, materials, mass, inertia
- **Joint creation**: Parent-child relationships, axes, limits
- **Sensor integration**: Gazebo plugins for simulation
- **Validation**: Using ROS 2 tools to check correctness

---

## Next Steps

### Immediate Next Actions

1. **Visualize in RViz** (Chapter 4):
   - Load your URDF into RViz
   - See the 3D model
   - Move joints with sliders

2. **Simulate in Gazebo** (Chapter 5):
   - Spawn robot in physics simulation
   - Apply gravity, collisions
   - See camera output

3. **Control with ROS 2** (Module 2):
   - Publish joint commands
   - Use controllers to move the arm
   - Read camera data in Python

### Scaling to Humanoids

This 3-DOF arm taught you the fundamentals. A full humanoid robot:
- **30-50 DOF** (not 3)
- **Dozens of links** (torso, legs, arms, head, hands)
- **Multiple sensors** (cameras, IMU, force sensors)
- **Same principles**, just more of them

**You now have the template** to build any robot description.

---

## Key Concepts Summary

**URDF Structure**:
- Start with `<?xml version="1.0"?>` and `<robot name="...">`
- Define links (visual, collision, inertial)
- Connect with joints (parent, child, axis, limits)
- Add sensors (Gazebo plugins)
- Include transmissions (for ROS control)
- Close with `</robot>`

**Best Practices**:
- Define links before referencing them in joints
- Use descriptive, unique names
- Make collision geometry slightly larger than visual
- Always include inertial properties
- Validate with `check_urdf` before testing

**Common Pattern**:
```
Link → Joint → Link → Joint → Link ...
```

Each joint adds one degree of freedom to your robot.

---

*"The robot you can describe is the robot you can build—in simulation, then in reality."*
