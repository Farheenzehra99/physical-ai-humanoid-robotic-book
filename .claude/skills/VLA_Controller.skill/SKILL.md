# VLA_Controller Skill

## Purpose
Provides Vision-Language-Action (VLA) model integration for robot control, enabling AI-driven reasoning from visual and language inputs to generate robot actions. This skill bridges foundation models (vision transformers, LLMs) with robotic control for intelligent, adaptive behavior.

## Core Capabilities
- Vision-Language-Action model inference
- Multi-modal input processing (vision + language + proprioception)
- Action prediction and execution
- Foundation model integration (RT-1, RT-2, OpenVLA, etc.)
- Real-time inference optimization
- Model fine-tuning on custom datasets
- Behavior cloning from demonstrations
- Policy distillation and compression
- Sim-to-real transfer evaluation

## Pipeline

### 1. Model Loading
```python
import torch
from transformers import AutoModel, AutoTokenizer

class VLAController:
    def __init__(self, model_name="openvla/openvla-7b"):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = AutoModel.from_pretrained(model_name).to(self.device)
        self.tokenizer = AutoTokenizer.from_pretrained(model_name)
        self.model.eval()
```

### 2. Multi-Modal Input Processing
```python
def process_inputs(self, image, instruction, robot_state):
    # Vision encoding
    vision_features = self.encode_image(image)

    # Language encoding
    text_tokens = self.tokenizer(instruction, return_tensors="pt")
    language_features = self.model.encode_text(text_tokens)

    # Robot state (joint positions, velocities, etc.)
    state_tensor = torch.tensor(robot_state, dtype=torch.float32)

    # Combine modalities
    multi_modal_input = self.model.fuse_modalities(
        vision=vision_features,
        language=language_features,
        proprioception=state_tensor
    )
    return multi_modal_input
```

### 3. Action Prediction
```python
def predict_action(self, image, instruction, robot_state):
    with torch.no_grad():
        inputs = self.process_inputs(image, instruction, robot_state)
        action_logits = self.model.predict_action(inputs)

        # Convert to robot-specific action format
        # (e.g., joint positions, velocities, or end-effector pose)
        action = self.post_process_action(action_logits)

    return action
```

### 4. Closed-Loop Control
```python
def control_loop(self, camera, robot, instruction):
    rate = 10  # Hz
    while not self.task_complete():
        # Get observations
        image = camera.get_image()
        robot_state = robot.get_state()

        # Predict action
        action = self.predict_action(image, instruction, robot_state)

        # Execute action
        robot.execute_action(action)

        time.sleep(1.0 / rate)
```

## Key Functions

### `load_vla_model(model_name, device)`
Loads pre-trained VLA model for inference.

### `predict_action(image, text, state)`
Predicts robot action from multi-modal inputs.

### `fine_tune_model(dataset, epochs, learning_rate)`
Fine-tunes VLA model on custom robotic dataset.

### `optimize_for_inference(model, quantization)`
Optimizes model for real-time edge deployment.

### `evaluate_policy(env, num_episodes)`
Evaluates policy performance in simulation or real robot.

## Examples

### Example 1: RT-1 Style Action Prediction
```python
import torch
from PIL import Image

class RT1Controller:
    def __init__(self, model_path):
        self.model = torch.load(model_path)
        self.model.eval()

    def predict(self, image_path, instruction):
        # Load and preprocess image
        image = Image.open(image_path).resize((224, 224))
        image_tensor = self.preprocess_image(image)

        # Tokenize instruction
        instruction_tokens = self.tokenize(instruction)

        # Predict action (e.g., 7-DOF end-effector pose + gripper)
        with torch.no_grad():
            action = self.model(image_tensor, instruction_tokens)

        # action shape: [7] (x, y, z, roll, pitch, yaw, gripper_open)
        return action.cpu().numpy()

# Usage
controller = RT1Controller("rt1_checkpoint.pth")
action = controller.predict("scene.jpg", "pick up the red block")
print(f"Predicted action: {action}")
```

### Example 2: OpenVLA Integration
```python
from transformers import AutoModelForVision2Seq, AutoProcessor
import torch

class OpenVLAController:
    def __init__(self):
        self.processor = AutoProcessor.from_pretrained("openvla/openvla-7b")
        self.model = AutoModelForVision2Seq.from_pretrained(
            "openvla/openvla-7b",
            torch_dtype=torch.bfloat16,
            device_map="auto"
        )

    def get_action(self, image, task_description, robot_obs):
        # Prepare inputs
        inputs = self.processor(
            images=image,
            text=task_description,
            return_tensors="pt"
        ).to(self.model.device)

        # Add robot observation to inputs
        inputs["robot_state"] = torch.tensor(robot_obs).unsqueeze(0)

        # Generate action
        with torch.no_grad():
            action_tokens = self.model.generate(**inputs, max_new_tokens=20)
            action = self.processor.decode_action(action_tokens)

        return action

# Usage
vla = OpenVLAController()
action = vla.get_action(
    image=camera.capture(),
    task_description="grasp the cup and place it on the table",
    robot_obs=robot.get_joint_positions()
)
```

### Example 3: Fine-Tuning on Custom Data
```python
import torch
from torch.utils.data import Dataset, DataLoader

class RobotDemonstrationDataset(Dataset):
    def __init__(self, data_dir):
        self.episodes = self.load_episodes(data_dir)

    def __getitem__(self, idx):
        episode = self.episodes[idx]
        return {
            'image': episode['image'],
            'instruction': episode['instruction'],
            'robot_state': episode['state'],
            'action': episode['action']
        }

    def __len__(self):
        return len(self.episodes)

def fine_tune_vla(model, dataset, epochs=10):
    dataloader = DataLoader(dataset, batch_size=32, shuffle=True)
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-5)
    loss_fn = torch.nn.MSELoss()

    model.train()
    for epoch in range(epochs):
        for batch in dataloader:
            optimizer.zero_grad()

            # Forward pass
            predicted_actions = model(
                images=batch['image'],
                text=batch['instruction'],
                robot_state=batch['robot_state']
            )

            # Compute loss
            loss = loss_fn(predicted_actions, batch['action'])

            # Backward pass
            loss.backward()
            optimizer.step()

        print(f"Epoch {epoch + 1}/{epochs}, Loss: {loss.item():.4f}")

    return model
```

### Example 4: Real-Time ROS2 Integration
```python
import rclpy
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class VLANode:
    def __init__(self):
        self.node = rclpy.create_node('vla_controller')
        self.vla_model = OpenVLAController()
        self.bridge = CvBridge()

        # Subscribers
        self.img_sub = self.node.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.joint_sub = self.node.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)

        # Publisher
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

        self.current_image = None
        self.current_joints = None
        self.instruction = "navigate to the kitchen"

    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        self.execute_policy()

    def joint_callback(self, msg):
        self.current_joints = msg.position

    def execute_policy(self):
        if self.current_image is None or self.current_joints is None:
            return

        action = self.vla_model.get_action(
            self.current_image,
            self.instruction,
            self.current_joints
        )

        # Convert action to Twist command
        cmd = Twist()
        cmd.linear.x = action[0]
        cmd.angular.z = action[1]
        self.cmd_pub.publish(cmd)
```

## Dependencies
- PyTorch >= 2.0
- transformers (Hugging Face)
- timm (vision models)
- OpenCV / PIL (image processing)
- numpy
- Optional: TensorRT (for optimization)
- Optional: ONNX Runtime (for cross-platform)

## Best Practices
- Use bfloat16 for inference on modern GPUs
- Batch process when possible for efficiency
- Cache vision encoder outputs for repeated scenes
- Implement action smoothing for stable control
- Add safety checks and bounds on predicted actions
- Monitor inference latency (target < 100ms)
- Use temporal context (history of observations)
- Implement fallback behaviors for low-confidence predictions
- Log predictions and outcomes for continuous improvement
- Fine-tune on domain-specific data for better performance

## Common VLA Architectures

### RT-1 (Robotics Transformer 1)
- Vision: EfficientNet
- Language: Universal Sentence Encoder
- Action: Tokenized discrete actions
- Use case: Manipulation tasks

### RT-2 (Robotics Transformer 2)
- Vision-Language: Vision Transformer + T5
- Action: Continuous and discrete
- Use case: General-purpose robot control
- Advantage: Better generalization

### OpenVLA
- Vision-Language: Prismatic VLM
- Action: Continuous action space
- Use case: Open-source VLA research
- Advantage: Fully open weights

### Gato (DeepMind)
- Multi-task generalist agent
- Unified transformer architecture
- Actions, vision, language all tokenized
- Use case: General AI agent

## Action Spaces

### End-Effector Control (6-7 DOF)
- Position: (x, y, z)
- Orientation: (roll, pitch, yaw) or quaternion
- Gripper: open/close (0-1)

### Joint Control
- N-dimensional (N = number of joints)
- Position targets or velocity commands

### Base Motion (Mobile Robots)
- Linear velocity (x, y)
- Angular velocity (θ)

### Hybrid Control
- Base motion + arm control
- Example: [vx, vy, ω, joint1, ..., joint7, gripper]

## Integration Points
- Trained using IsaacSim_Pipeline (RL or imitation)
- Receives sensor data from ROS2_Core
- Deployed via Edge_Deploy to Jetson devices
- Controls Hardware_Proxy for real robot execution
- Uses environments from Unity_Vis or Gazebo_Sim
- Leverages URDF_Designer for kinematic awareness
