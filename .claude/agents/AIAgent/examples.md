# AIAgent Examples

## Example 1: RL Walking Policy Training and Deployment

### User Request
```
"Train a walking policy for my humanoid using Isaac Gym and deploy it to Jetson Orin Nano"
```

### AIAgent Execution

#### Step 1: IsaacSim_Pipeline - Setup
**Task**: Configure RL training environment

**Execution**:
- Created 1024 parallel environments
- Configured humanoid robot in each environment
- Set up observation space (48-dim): joint positions, velocities, base orientation
- Set up action space (12-dim): joint position targets
- Configured reward function: forward velocity, stability, energy efficiency

**Output**:
```
✓ 1024 parallel environments created
✓ Observation space: 48 dimensions
✓ Action space: 12 dimensions (continuous)
✓ GPU: NVIDIA RTX 4090
✓ Memory allocated: 16 GB
```

#### Step 2: IsaacSim_Pipeline - Training
**Task**: Train PPO policy

**Execution**:
- Algorithm: PPO (Proximal Policy Optimization)
- Learning rate: 3e-4
- Training steps: 10 million
- Episode length: 1000 steps
- Training throughput: ~18,000 steps/sec

**Output**:
```
✓ Training completed in 9.3 hours
✓ Final episode return: 245.3
✓ Success rate: 92% (walks without falling)
✓ Average velocity: 0.38 m/s
✓ Checkpoint saved: walking_policy_10M.pth
```

#### Step 3: Edge_Deploy - Optimization
**Task**: Optimize policy for Jetson Orin Nano

**Execution**:
- Exported PyTorch model to ONNX (opset 15)
- Converted ONNX to TensorRT FP16
- Validated accuracy: < 0.1% difference
- Benchmarked on Jetson Orin Nano

**Output**:
```
✓ ONNX export successful
✓ TensorRT engine built
✓ Model size: 45 MB → 23 MB (FP16)
✓ Inference latency: 11.8ms (target: < 20ms) ✓
✓ Control frequency achievable: 84 Hz
```

#### Step 4: Edge_Deploy - Deployment
**Task**: Deploy to Jetson hardware

**Execution**:
- Transferred TensorRT engine to Jetson (192.168.1.100)
- Created ROS2 inference node
- Set up systemd service for auto-start
- Configured power mode: 15W

**Output**:
```
✓ Files transferred via SCP
✓ ROS2 node: policy_inference_node
✓ Systemd service: robot-policy.service
✓ Auto-start enabled
✓ Status: Active and running
```

### Final Result
Trained walking policy achieves 92% success rate in simulation. Optimized for edge deployment with 11.8ms inference latency. Successfully deployed to Jetson Orin Nano with auto-start configuration.

**Performance Metrics**:
- Sim performance: 245.3 avg return, 0.38 m/s
- Inference speed: 84 Hz (11.8ms per step)
- Deployment status: Running successfully

---

## Example 2: OpenVLA Fine-Tuning for Custom Robot

### User Request
```
"Fine-tune OpenVLA on my robot demonstrations and deploy to Jetson Orin NX"
```

### AIAgent Execution

#### Step 1: VLA_Controller - Load Model
**Task**: Load pre-trained OpenVLA model

**Execution**:
- Model: openvla/openvla-7b
- Parameters: 7 billion
- Loaded in bfloat16 precision
- Device: CUDA GPU 0

**Output**:
```
✓ Model loaded: OpenVLA-7B
✓ Parameters: 7.0B
✓ Precision: bfloat16
✓ GPU memory: 14.2 GB
✓ Input: Images (224×224), text, robot state
✓ Output: 7-DOF actions (position + gripper)
```

#### Step 2: VLA_Controller - Fine-Tuning
**Task**: Fine-tune on custom demonstrations

**Execution**:
- Dataset: 500 demonstrations (pick-and-place tasks)
- Training epochs: 10
- Batch size: 8
- Learning rate: 1e-5
- Validation split: 20%
- Training time: 4.5 hours

**Output**:
```
✓ Fine-tuning completed
✓ Training loss: 0.042 → 0.008
✓ Validation accuracy: 87.3%
✓ Task success rate (sim): 84%
✓ Checkpoint: openvla_finetuned.pth (14.3 GB)
```

#### Step 3: Edge_Deploy - Optimization
**Task**: Optimize for Jetson Orin NX

**Execution**:
- Exported to ONNX
- Applied dynamic quantization (INT8)
- Calibration on 100 representative samples
- Built TensorRT engine with INT8 precision

**Output**:
```
✓ ONNX export successful
✓ INT8 quantization applied
✓ Model size: 14.3 GB → 3.8 GB (3.8x reduction)
✓ Inference latency: 42ms on Orin NX
✓ Accuracy preserved: 86.1% (vs 87.3% before)
✓ Acceptable accuracy loss: 1.2%
```

#### Step 4: Edge_Deploy - Deployment
**Task**: Deploy to Jetson Orin NX

**Execution**:
- Transferred optimized model
- Created ROS2 VLA control node
- Integrated with camera and robot state topics
- Set power mode: MAXN (25W)

**Output**:
```
✓ Deployed to Jetson Orin NX
✓ ROS2 node: vla_controller
✓ Subscribed topics:
  - /camera/image_raw (sensor_msgs/Image)
  - /joint_states (sensor_msgs/JointState)
✓ Published topic:
  - /action_command (geometry_msgs/Pose)
✓ Control frequency: 23 Hz (42ms latency)
✓ Status: Running
```

### Final Result
OpenVLA model fine-tuned on custom robot demonstrations. Achieves 87.3% validation accuracy. Optimized from 14.3 GB to 3.8 GB with INT8 quantization. Deployed to Jetson Orin NX with 42ms inference latency, enabling 23 Hz control.

**Real Robot Testing**:
- Task success rate: 79% (pick-and-place)
- Sim-to-real gap: 5 percentage points
- Stable operation over 2 hour test

---

## Example 3: Synthetic Data Generation Pipeline

### User Request
```
"Generate 10,000 labeled images of my robot in various poses for training a perception model"
```

### AIAgent Execution

#### Step 1: IsaacSim_Pipeline - Replicator Setup
**Task**: Configure synthetic data generation

**Execution**:
- Set up Replicator in Isaac Sim
- Imported robot USD model
- Created diverse environment scenes (5 variations)
- Configured camera: 640×480 RGB + depth
- Set up annotations: bounding boxes, segmentation, keypoints

**Output**:
```
✓ Replicator configured
✓ Robot model loaded
✓ 5 environment variations created
✓ Camera: 640×480 RGB + depth
✓ Annotations: bbox_2d, segmentation, keypoints
✓ Capture rate: 50 images/sec (estimated)
```

#### Step 2: IsaacSim_Pipeline - Domain Randomization
**Task**: Configure randomization parameters

**Execution**:
- Lighting randomization: intensity (500-3000), color temperature
- Material randomization: floor/wall colors
- Camera pose randomization: radius (1-3m), height (0.5-2m)
- Robot pose randomization: all joint angles within limits
- Procedural object placement: 0-5 random objects per scene

**Output**:
```
✓ Domain randomization configured
✓ Visual diversity: High
✓ Pose diversity: Full joint space coverage
✓ Scene complexity: Variable (0-5 objects)
✓ Lighting conditions: Day, evening, night simulated
```

#### Step 3: IsaacSim_Pipeline - Data Generation
**Task**: Generate 10,000 samples

**Execution**:
- Generated 10,000 RGB images with annotations
- Average generation speed: 45 images/sec
- Total time: ~3.7 minutes (GPU-accelerated)
- Data format: PNG (RGB), NPY (depth), JSON (annotations)

**Output**:
```
✓ Data generation complete
✓ 10,000 samples generated
✓ Generation time: 3 minutes 42 seconds
✓ Dataset size: 8.4 GB
✓ Output directory: ./synthetic_data/
✓ Annotations:
  - 2D bounding boxes: 100%
  - Semantic segmentation: 100%
  - Joint keypoints (12 per robot): 100%
  - Depth maps: 100%
```

#### Step 4: VLA_Controller - Model Training (Optional)
**Task**: Train perception model on synthetic data

**Execution**:
- Architecture: EfficientDet-D2 (object detection)
- Training epochs: 50
- Batch size: 16
- Augmentation: Horizontal flip, color jitter
- Validation: 10% held-out synthetic data

**Output**:
```
✓ Training completed
✓ mAP@0.5: 0.94 (synthetic validation)
✓ Training time: 2.1 hours
✓ Model: robot_detector.pth
✓ Ready for real-world fine-tuning
```

### Final Result
Generated 10,000 diverse synthetic images in under 4 minutes. Dataset includes RGB, depth, bounding boxes, segmentation, and keypoints. Trained perception model achieves 94% mAP on synthetic validation set, ready for real-world fine-tuning.

**Dataset Statistics**:
- Images: 10,000 (640×480)
- Unique robot poses: ~8,500
- Scene variations: 5 base scenes × randomization
- Lighting conditions: Diverse (500-3000 intensity range)

---

## Example 4: Sim-to-Real with Domain Randomization

### User Request
```
"Train a policy with domain randomization to minimize sim-to-real gap"
```

### AIAgent Execution

#### Step 1: IsaacSim_Pipeline - Randomization Config
**Task**: Configure comprehensive domain randomization

**Execution**:
- **Visual randomization**:
  - Lighting: intensity ±50%, color temperature
  - Materials: floor friction visualization, robot colors
  - Textures: floor patterns, wall colors
- **Physics randomization**:
  - Mass: ±20% per link
  - Friction: 0.6-1.4 (ground), 0.2-0.8 (objects)
  - Joint damping: ±30%
  - Actuator strength: ±10%
- **Geometric randomization**:
  - Link lengths: ±2% (manufacturing tolerance)
  - Joint offsets: ±0.5° (calibration error)
  - Initial pose: randomized within safe bounds

**Output**:
```
✓ Domain randomization configured
✓ Visual randomization: 5 parameters
✓ Physics randomization: 4 parameters
✓ Geometric randomization: 3 parameters
✓ Randomization interval: Every episode
✓ Diversity level: High
```

#### Step 2: IsaacSim_Pipeline - Training with Randomization
**Task**: Train robust RL policy

**Execution**:
- Environments: 2048 parallel
- Algorithm: PPO
- Training steps: 15 million (for robustness)
- Episode length: 1000 steps
- Randomization: Applied every episode reset

**Output**:
```
✓ Training completed: 13.8 hours
✓ Final return: 228.7 (lower but more robust)
✓ Success rate (randomized sim): 88%
✓ Policy variance: Low (stable across conditions)
✓ Checkpoint: robust_policy_15M.pth
```

#### Step 3: Edge_Deploy - Optimization
**Task**: Optimize for edge deployment

**Execution**:
- Exported to ONNX
- TensorRT FP16 conversion
- Deployed to Jetson AGX Orin

**Output**:
```
✓ Inference latency: 8.2ms (AGX Orin)
✓ Model size: 52 MB → 27 MB
✓ Control frequency: 120 Hz achievable
```

#### Step 4: Edge_Deploy - Real Robot Validation
**Task**: Test on actual hardware

**Execution**:
- Deployed to real humanoid robot
- Tested in various conditions (different floors, lighting)
- Measured performance over 50 trials

**Output**:
```
✓ Hardware deployment successful
✓ Real-world success rate: 84% (vs 88% in sim)
✓ Sim-to-real gap: Only 4 percentage points!
✓ Velocity achieved: 0.32 m/s (vs 0.35 m/s in sim)
✓ Robustness: Works on carpet, tile, wood floors
✓ Lighting invariance: Works in bright and dim conditions
```

### Final Result
Policy trained with comprehensive domain randomization achieves minimal sim-to-real gap. Success rate drops only 4 percentage points from simulation (88%) to reality (84%). Policy demonstrates robustness across different floor types and lighting conditions.

**Key Achievement**: Sim-to-real transfer with < 5% performance gap through domain randomization.

**Comparison**:
- Without randomization: ~60% real-world success (historical baseline)
- With randomization: 84% real-world success
- Improvement: +24 percentage points

---

## Example 5: Hyperparameter Optimization with Parallel Training

### User Request
```
"Find the best hyperparameters for my RL task by trying multiple configurations"
```

### AIAgent Execution

#### Step 1: IsaacSim_Pipeline - Setup Parallel Runs
**Task**: Configure multiple training runs with different hyperparameters

**Execution**:
- Hyperparameter grid:
  - Learning rate: [1e-4, 3e-4, 1e-3]
  - Batch size: [2048, 4096, 8192]
  - Network depth: [2 layers, 3 layers]
- Total configurations: 18
- GPU allocation: 4 GPUs available
- Strategy: 4 parallel runs at a time

**Output**:
```
✓ 18 configurations to test
✓ 4 parallel training processes per GPU
✓ Estimated time: 12 hours (vs 54 hours sequential)
✓ GPU assignment:
  - GPU 0: Configs 1-4
  - GPU 1: Configs 5-8
  - GPU 2: Configs 9-12
  - GPU 3: Configs 13-16
```

#### Step 2: IsaacSim_Pipeline - Parallel Training
**Task**: Execute all training runs

**Execution**:
- All runs trained for 5M steps (shorter for quick evaluation)
- Monitored performance across all runs
- Logged results automatically

**Output**:
```
✓ All 18 runs completed in 11.3 hours
✓ Best configuration found:
  - Learning rate: 3e-4
  - Batch size: 4096
  - Network depth: 3 layers
  - Final return: 256.8
✓ Worst configuration:
  - Learning rate: 1e-3
  - Final return: 142.3
✓ Performance range: 142.3 - 256.8
```

#### Step 3: IsaacSim_Pipeline - Extended Training
**Task**: Train best configuration for full duration

**Execution**:
- Selected best hyperparameters
- Extended training to 10M steps
- Used all GPUs for single run (max performance)

**Output**:
```
✓ Extended training complete
✓ Final return: 268.4 (improved from 256.8)
✓ Success rate: 95%
✓ Checkpoint: best_policy_10M.pth
```

#### Step 4: Edge_Deploy - Deploy Best Policy
**Task**: Optimize and deploy winning configuration

**Execution**:
- Optimized best policy for Jetson
- Deployed to target hardware

**Output**:
```
✓ Optimization complete
✓ Deployed to Jetson Orin NX
✓ Inference latency: 14.2ms
✓ Real-world performance: 91% success rate
```

### Final Result
Hyperparameter search completed in 11.3 hours (vs 54 hours if sequential). Best configuration identified: LR=3e-4, BS=4096, Depth=3. Extended training achieves 95% sim success rate and 91% real-world success rate after deployment.

**Time Savings**: 78% faster through parallelization (11.3h vs 54h).
