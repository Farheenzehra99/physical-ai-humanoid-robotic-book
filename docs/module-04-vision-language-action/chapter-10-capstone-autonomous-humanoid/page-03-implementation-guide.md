---
sidebar_position: 3
title: "Implementation Guide"
description: "Step-by-step build instructions for the autonomous humanoid capstone"
---

# Implementation Guide

## Overview

This guide walks you through building each subsystem of the autonomous humanoid capstone. Follow the steps in order, verifying each component works before moving to the next.

---

## Step 1: Workspace Setup

### Create ROS 2 Workspace

```bash
# Create workspace
mkdir -p ~/capstone_ws/src
cd ~/capstone_ws

# Initialize rosdep
sudo rosdep init  # Only if not done before
rosdep update

# Source ROS 2
source /opt/ros/jazzy/setup.bash
```

### Clone/Create Packages

```bash
cd ~/capstone_ws/src

# Create main packages
ros2 pkg create --build-type ament_python voice_commands \
    --dependencies rclpy std_msgs

ros2 pkg create --build-type ament_python cognitive_planning \
    --dependencies rclpy std_msgs geometry_msgs

ros2 pkg create --build-type ament_python robot_vision \
    --dependencies rclpy std_msgs sensor_msgs

ros2 pkg create --build-type ament_python capstone_bringup \
    --dependencies rclpy
```

### Install Dependencies

```bash
cd ~/capstone_ws

# Python dependencies
pip install openai sounddevice webrtcvad numpy opencv-python torch ultralytics

# ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash
```

---

## Step 2: Voice Reception Setup

### Voice Command Node

Create `voice_commands/voice_commands/voice_node.py`:

```python
#!/usr/bin/env python3
"""Voice command node for capstone."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sounddevice as sd
import numpy as np
import webrtcvad
from openai import OpenAI
import tempfile
import wave
import threading
from queue import Queue

SAMPLE_RATE = 16000
CHUNK_MS = 30
CHUNK_SIZE = int(SAMPLE_RATE * CHUNK_MS / 1000)


class VoiceCommandNode(Node):

    def __init__(self):
        super().__init__('voice_command_node')

        # Parameters
        self.declare_parameter('use_api', True)
        self.declare_parameter('vad_aggressiveness', 2)
        self.declare_parameter('silence_threshold', 0.5)

        self.use_api = self.get_parameter('use_api').value
        vad_level = self.get_parameter('vad_aggressiveness').value
        self.silence_threshold = self.get_parameter('silence_threshold').value

        # Initialize
        self.vad = webrtcvad.Vad(vad_level)
        self.client = OpenAI() if self.use_api else None

        # Publishers
        self.command_pub = self.create_publisher(String, '/voice_commands', 10)
        self.status_pub = self.create_publisher(String, '/voice_status', 10)

        # Audio state
        self.audio_buffer = []
        self.is_speaking = False
        self.silence_frames = 0
        self.audio_queue = Queue()

        # Start audio thread
        self.running = True
        self.audio_thread = threading.Thread(target=self._audio_loop)
        self.audio_thread.start()

        # Processing timer
        self.create_timer(0.01, self._process_audio)

        self._publish_status('ready')
        self.get_logger().info('Voice command node ready')

    def _audio_loop(self):
        """Capture audio in background."""
        def callback(indata, frames, time, status):
            if status:
                self.get_logger().warning(f'Audio: {status}')
            self.audio_queue.put(indata.copy().flatten())

        with sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=1,
            dtype=np.int16,
            blocksize=CHUNK_SIZE,
            callback=callback
        ):
            while self.running:
                sd.sleep(100)

    def _process_audio(self):
        """Process audio queue."""
        while not self.audio_queue.empty():
            chunk = self.audio_queue.get()
            self._handle_chunk(chunk)

    def _handle_chunk(self, chunk):
        """Handle single audio chunk."""
        is_speech = self._is_speech(chunk)

        if is_speech:
            if not self.is_speaking:
                self._publish_status('listening')
            self.is_speaking = True
            self.silence_frames = 0
            self.audio_buffer.append(chunk)
        elif self.is_speaking:
            self.silence_frames += 1
            self.audio_buffer.append(chunk)

            if self.silence_frames * CHUNK_MS / 1000 >= self.silence_threshold:
                self._finalize()

    def _is_speech(self, chunk):
        """Detect speech using VAD."""
        try:
            return self.vad.is_speech(
                chunk.astype(np.int16).tobytes(),
                SAMPLE_RATE
            )
        except:
            return False

    def _finalize(self):
        """Process complete utterance."""
        if not self.audio_buffer:
            return

        audio = np.concatenate(self.audio_buffer)
        duration = len(audio) / SAMPLE_RATE

        self.audio_buffer = []
        self.is_speaking = False
        self.silence_frames = 0

        if duration < 0.5:
            self._publish_status('ready')
            return

        self._publish_status('transcribing')

        try:
            text = self._transcribe(audio)
            if text:
                self._publish_command(text)
        except Exception as e:
            self.get_logger().error(f'Transcription error: {e}')

        self._publish_status('ready')

    def _transcribe(self, audio):
        """Transcribe with Whisper."""
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
            with wave.open(f.name, 'w') as wav:
                wav.setnchannels(1)
                wav.setsampwidth(2)
                wav.setframerate(SAMPLE_RATE)
                wav.writeframes(audio.astype(np.int16).tobytes())
            path = f.name

        try:
            with open(path, 'rb') as audio_file:
                result = self.client.audio.transcriptions.create(
                    model='whisper-1',
                    file=audio_file
                )
            return result.text.strip()
        finally:
            import os
            os.unlink(path)

    def _publish_command(self, text):
        """Publish transcribed command."""
        msg = String()
        msg.data = text
        self.command_pub.publish(msg)
        self.get_logger().info(f'Command: {text}')

    def _publish_status(self, status):
        """Publish status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.running = False
        self.audio_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Test Voice Node

```bash
# Terminal 1: Run node
ros2 run voice_commands voice_node

# Terminal 2: Monitor
ros2 topic echo /voice_commands
ros2 topic echo /voice_status

# Speak "go to the kitchen"
# Should see transcription on /voice_commands
```

---

## Step 3: Cognitive Planning Integration

### Task Planner Node

Create `cognitive_planning/cognitive_planning/planner_node.py`:

```python
#!/usr/bin/env python3
"""Task planner node for capstone."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI
import json


SYSTEM_PROMPT = """You are a task planner for a humanoid robot.

Available actions:
- navigate: target (location name)
- find: object (object name)
- pick: object (object name)
- place: location (surface name)
- speak: message (text to say)
- handover: object (object name)

Return JSON only:
{
  "task": "description",
  "steps": [
    {"action": "navigate", "params": {"target": "kitchen"}},
    {"action": "find", "params": {"object": "cup"}}
  ]
}

Known locations: kitchen, bedroom, living_room, bathroom
Objects to find: cup, glass, book, phone, remote
"""


class TaskPlannerNode(Node):

    def __init__(self):
        super().__init__('task_planner_node')

        self.declare_parameter('llm_backend', 'openai')
        self.declare_parameter('model', 'gpt-4o')

        backend = self.get_parameter('llm_backend').value
        self.model = self.get_parameter('model').value

        self.client = OpenAI()

        # Subscribers
        self.command_sub = self.create_subscription(
            String,
            '/voice_commands',
            self._command_callback,
            10
        )

        # Publishers
        self.plan_pub = self.create_publisher(String, '/task_plan', 10)
        self.status_pub = self.create_publisher(String, '/planner_status', 10)

        self.get_logger().info('Task planner ready')

    def _command_callback(self, msg):
        """Handle voice command."""
        command = msg.data
        self.get_logger().info(f'Planning: {command}')

        self._publish_status('planning')

        try:
            plan = self._generate_plan(command)
            self._publish_plan(plan)
            self._publish_status('ready')
        except Exception as e:
            self.get_logger().error(f'Planning error: {e}')
            self._publish_status('error')

    def _generate_plan(self, command):
        """Generate plan using LLM."""
        response = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": f"Plan: {command}"}
            ],
            temperature=0.3,
            response_format={"type": "json_object"}
        )

        return response.choices[0].message.content

    def _publish_plan(self, plan):
        """Publish task plan."""
        msg = String()
        msg.data = plan
        self.plan_pub.publish(msg)
        self.get_logger().info(f'Plan published')

    def _publish_status(self, status):
        """Publish status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Task Executor Node

Create `cognitive_planning/cognitive_planning/executor_node.py`:

```python
#!/usr/bin/env python3
"""Task executor node for capstone."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
import json
import asyncio


class TaskExecutorNode(Node):

    # Semantic locations
    LOCATIONS = {
        "kitchen": (3.0, 2.0, 0.0),
        "bedroom": (5.0, 4.0, 1.57),
        "living_room": (1.0, 1.0, 0.0),
        "bathroom": (4.0, 1.0, 3.14),
        "user": (0.0, 0.0, 0.0),
    }

    def __init__(self):
        super().__init__('task_executor_node')

        # Subscribers
        self.plan_sub = self.create_subscription(
            String,
            '/task_plan',
            self._plan_callback,
            10
        )

        # Publishers
        self.status_pub = self.create_publisher(String, '/executor_status', 10)
        self.feedback_pub = self.create_publisher(String, '/executor_feedback', 10)

        # Action clients
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.is_executing = False
        self.get_logger().info('Task executor ready')

    def _plan_callback(self, msg):
        """Handle incoming task plan."""
        if self.is_executing:
            self.get_logger().warning('Already executing')
            return

        try:
            plan = json.loads(msg.data)
            self.get_logger().info(f"Executing: {plan.get('task', 'unknown task')}")

            # Execute plan
            asyncio.get_event_loop().run_until_complete(
                self._execute_plan(plan)
            )

        except Exception as e:
            self.get_logger().error(f'Execution error: {e}')
            self._publish_status('error')

    async def _execute_plan(self, plan):
        """Execute task plan."""
        self.is_executing = True
        self._publish_status('executing')

        steps = plan.get('steps', [])

        for i, step in enumerate(steps):
            action = step.get('action')
            params = step.get('params', {})

            self.get_logger().info(f"Step {i+1}/{len(steps)}: {action}")
            self._publish_feedback(f"Step {i+1}: {action}")

            success = await self._execute_action(action, params)

            if not success:
                self.get_logger().error(f"Action failed: {action}")
                self._publish_status('failed')
                self.is_executing = False
                return

        self._publish_status('completed')
        self._publish_feedback("Task completed!")
        self.is_executing = False

    async def _execute_action(self, action, params):
        """Execute single action."""

        if action == "navigate":
            target = params.get("target", "user")
            return await self._navigate_to(target)

        elif action == "speak":
            message = params.get("message", "")
            self.get_logger().info(f"[SPEAK] {message}")
            return True

        elif action == "find":
            obj = params.get("object", "")
            self.get_logger().info(f"[FIND] Looking for {obj}")
            # Would integrate with vision
            await asyncio.sleep(1.0)  # Simulate
            return True

        elif action == "pick":
            obj = params.get("object", "")
            self.get_logger().info(f"[PICK] Grasping {obj}")
            # Would integrate with MoveIt
            await asyncio.sleep(1.0)  # Simulate
            return True

        elif action == "place":
            loc = params.get("location", "")
            self.get_logger().info(f"[PLACE] Placing at {loc}")
            await asyncio.sleep(1.0)  # Simulate
            return True

        elif action == "handover":
            obj = params.get("object", "")
            self.get_logger().info(f"[HANDOVER] Giving {obj} to user")
            await asyncio.sleep(1.0)  # Simulate
            return True

        else:
            self.get_logger().warning(f"Unknown action: {action}")
            return False

    async def _navigate_to(self, location):
        """Navigate to named location."""
        if location not in self.LOCATIONS:
            self.get_logger().error(f"Unknown location: {location}")
            return False

        x, y, theta = self.LOCATIONS[location]

        # Wait for Nav2
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 not available")
            # Simulate navigation for demo
            self.get_logger().info(f"Simulating navigation to {location}")
            await asyncio.sleep(2.0)
            return True

        # Create goal
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, theta)
        goal.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Send goal
        self.get_logger().info(f"Navigating to {location} ({x}, {y})")
        result = await self._nav_client.send_goal_async(goal)

        if not result.accepted:
            return False

        result = await result.get_result_async()
        return True

    def _publish_status(self, status):
        """Publish executor status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _publish_feedback(self, feedback):
        """Publish execution feedback."""
        msg = String()
        msg.data = feedback
        self.feedback_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskExecutorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Step 4: Navigation Setup

### Configure Nav2

Create `capstone_bringup/config/nav2_params.yaml`:

```yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    base_frame_id: "base_footprint"
    global_frame_id: "map"
    robot_model_type: "differential"

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    default_nav_to_pose_bt_xml: "navigate_w_replanning_and_recovery.xml"

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"

planner_server:
  ros__parameters:
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
```

### Semantic Map

Create `capstone_bringup/config/semantic_locations.yaml`:

```yaml
locations:
  kitchen:
    x: 3.0
    y: 2.0
    theta: 0.0
    description: "Kitchen area with sink and counter"

  bedroom:
    x: 5.0
    y: 4.0
    theta: 1.57
    description: "Master bedroom"

  living_room:
    x: 1.0
    y: 1.0
    theta: 0.0
    description: "Main living area with couch"

  bathroom:
    x: 4.0
    y: 1.0
    theta: 3.14
    description: "Main bathroom"

  entrance:
    x: 0.0
    y: 0.0
    theta: 0.0
    description: "Entry point"
```

---

## Step 5: Vision Setup

### Object Detector Node

Create `robot_vision/robot_vision/detector_node.py`:

```python
#!/usr/bin/env python3
"""Object detection node using YOLOv8."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json

# Try to import YOLO
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class ObjectDetectorNode(Node):

    # Objects we care about
    TARGET_CLASSES = {'cup', 'bottle', 'book', 'cell phone', 'remote'}

    def __init__(self):
        super().__init__('object_detector_node')

        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('device', 'cpu')

        model_path = self.get_parameter('model').value
        self.confidence = self.get_parameter('confidence').value
        device = self.get_parameter('device').value

        self.bridge = CvBridge()

        if YOLO_AVAILABLE:
            self.model = YOLO(model_path)
            self.model.to(device)
            self.get_logger().info(f'YOLO loaded on {device}')
        else:
            self.model = None
            self.get_logger().warning('YOLO not available, using mock detector')

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._image_callback,
            10
        )

        # Publishers
        self.detection_pub = self.create_publisher(
            String,
            '/detected_objects',
            10
        )

        self.get_logger().info('Object detector ready')

    def _image_callback(self, msg):
        """Process incoming image."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            detections = self._detect(cv_image)

            if detections:
                self._publish_detections(detections)

        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')

    def _detect(self, image):
        """Run detection on image."""
        if self.model is None:
            # Mock detection for testing
            return [{'label': 'cup', 'confidence': 0.95, 'bbox': [100, 100, 50, 80]}]

        results = self.model(image, conf=self.confidence, verbose=False)
        detections = []

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                label = self.model.names[cls_id]

                if label in self.TARGET_CLASSES:
                    detections.append({
                        'label': label,
                        'confidence': float(box.conf[0]),
                        'bbox': box.xyxy[0].tolist()
                    })

        return detections

    def _publish_detections(self, detections):
        """Publish detected objects."""
        msg = String()
        msg.data = json.dumps(detections)
        self.detection_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Step 6: Complete Launch File

Create `capstone_bringup/launch/capstone.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Arguments
    use_sim = DeclareLaunchArgument(
        'use_sim', default_value='true',
        description='Use simulation'
    )

    use_api = DeclareLaunchArgument(
        'use_api', default_value='true',
        description='Use OpenAI API'
    )

    # Voice command node
    voice_node = Node(
        package='voice_commands',
        executable='voice_node',
        name='voice_command_node',
        parameters=[{'use_api': LaunchConfiguration('use_api')}],
        output='screen'
    )

    # Task planner node
    planner_node = Node(
        package='cognitive_planning',
        executable='planner_node',
        name='task_planner_node',
        output='screen'
    )

    # Task executor node
    executor_node = Node(
        package='cognitive_planning',
        executable='executor_node',
        name='task_executor_node',
        output='screen'
    )

    # Object detector node
    detector_node = Node(
        package='robot_vision',
        executable='detector_node',
        name='object_detector_node',
        output='screen'
    )

    return LaunchDescription([
        use_sim,
        use_api,
        voice_node,
        planner_node,
        executor_node,
        detector_node,
    ])
```

---

## Step 7: Build and Test

### Build All Packages

```bash
cd ~/capstone_ws
colcon build --symlink-install
source install/setup.bash
```

### Run the System

```bash
# Set API key
export OPENAI_API_KEY="your-key"

# Launch everything
ros2 launch capstone_bringup capstone.launch.py
```

### Monitor System

```bash
# Terminal 2: Watch topics
ros2 topic echo /voice_commands
ros2 topic echo /task_plan
ros2 topic echo /executor_status
ros2 topic echo /executor_feedback
```

### Test Commands

Speak these commands:
- "Go to the kitchen"
- "Find the cup"
- "Pick up the cup and bring it here"

---

## Summary

You've built:
1. **Voice node**: Captures and transcribes speech
2. **Planner node**: Converts commands to plans
3. **Executor node**: Executes plans step by step
4. **Detector node**: Finds objects in images
5. **Launch file**: Starts everything together

**Next**: Testing and validation procedures.
