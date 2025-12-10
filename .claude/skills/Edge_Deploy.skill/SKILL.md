# Edge_Deploy Skill

## Purpose
Provides deployment capabilities for robot AI models and control systems on edge devices, specifically NVIDIA Jetson platforms (Orin, Xavier, Nano). This skill enables optimization, conversion, and deployment of ML models with real-time performance constraints for on-robot inference.

## Core Capabilities
- Model optimization for Jetson hardware
- TensorRT conversion and acceleration
- ONNX export and optimization
- Quantization (INT8, FP16)
- Memory and power profiling
- Real-time performance tuning
- ROS2 integration on edge
- Multi-process orchestration
- Thermal management
- Continuous deployment pipelines

## Pipeline

### 1. Model Preparation
```python
import torch
import torch.onnx

def export_to_onnx(model, sample_input, output_path):
    model.eval()
    torch.onnx.export(
        model,
        sample_input,
        output_path,
        export_params=True,
        opset_version=15,
        do_constant_folding=True,
        input_names=['image', 'robot_state'],
        output_names=['action'],
        dynamic_axes={
            'image': {0: 'batch_size'},
            'robot_state': {0: 'batch_size'},
            'action': {0: 'batch_size'}
        }
    )
```

### 2. TensorRT Optimization
```python
import tensorrt as trt

class TensorRTConverter:
    def __init__(self):
        self.logger = trt.Logger(trt.Logger.INFO)
        self.builder = trt.Builder(self.logger)

    def convert_onnx_to_tensorrt(self, onnx_path, engine_path, precision='fp16'):
        network = self.builder.create_network(
            1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
        )
        parser = trt.OnnxParser(network, self.logger)

        with open(onnx_path, 'rb') as model:
            parser.parse(model.read())

        config = self.builder.create_builder_config()
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)  # 1GB

        if precision == 'fp16':
            config.set_flag(trt.BuilderFlag.FP16)
        elif precision == 'int8':
            config.set_flag(trt.BuilderFlag.INT8)
            # Add INT8 calibrator here

        engine = self.builder.build_serialized_network(network, config)
        with open(engine_path, 'wb') as f:
            f.write(engine)
```

### 3. Edge Inference
```python
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import numpy as np

class TensorRTInference:
    def __init__(self, engine_path):
        with open(engine_path, 'rb') as f:
            engine_data = f.read()

        runtime = trt.Runtime(trt.Logger(trt.Logger.WARNING))
        self.engine = runtime.deserialize_cuda_engine(engine_data)
        self.context = self.engine.create_execution_context()

        # Allocate buffers
        self.inputs = []
        self.outputs = []
        self.bindings = []
        self.stream = cuda.Stream()

        for binding in self.engine:
            size = trt.volume(self.engine.get_binding_shape(binding))
            dtype = trt.nptype(self.engine.get_binding_dtype(binding))
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)

            self.bindings.append(int(device_mem))
            if self.engine.binding_is_input(binding):
                self.inputs.append({'host': host_mem, 'device': device_mem})
            else:
                self.outputs.append({'host': host_mem, 'device': device_mem})

    def infer(self, input_data):
        # Transfer input data to GPU
        np.copyto(self.inputs[0]['host'], input_data.ravel())
        cuda.memcpy_htod_async(
            self.inputs[0]['device'],
            self.inputs[0]['host'],
            self.stream
        )

        # Run inference
        self.context.execute_async_v2(
            bindings=self.bindings,
            stream_handle=self.stream.handle
        )

        # Transfer predictions back
        cuda.memcpy_dtoh_async(
            self.outputs[0]['host'],
            self.outputs[0]['device'],
            self.stream
        )
        self.stream.synchronize()

        return self.outputs[0]['host']
```

### 4. Deployment Script
```python
#!/usr/bin/env python3
import subprocess
import os

def deploy_to_jetson(model_path, target_ip, target_user='nvidia'):
    """Deploy model to Jetson device"""

    # 1. Transfer files
    subprocess.run([
        'scp', model_path,
        f'{target_user}@{target_ip}:~/models/'
    ])

    # 2. Transfer ROS2 nodes
    subprocess.run([
        'rsync', '-avz', './ros2_ws/',
        f'{target_user}@{target_ip}:~/ros2_ws/'
    ])

    # 3. Build on target
    ssh_command = f'ssh {target_user}@{target_ip}'
    subprocess.run([
        ssh_command,
        'cd ~/ros2_ws && colcon build --symlink-install'
    ])

    # 4. Set up systemd service for auto-start
    service_content = """
    [Unit]
    Description=Robot Control Service
    After=network.target

    [Service]
    Type=simple
    User=nvidia
    ExecStart=/home/nvidia/ros2_ws/install/setup.bash && ros2 launch robot_control main.launch.py
    Restart=always

    [Install]
    WantedBy=multi-user.target
    """

    # Write and enable service
    # ... (implementation details)

    print("Deployment complete!")
```

## Key Functions

### `export_to_onnx(model, sample_input, output_path)`
Exports PyTorch model to ONNX format.

### `convert_to_tensorrt(onnx_path, engine_path, precision)`
Converts ONNX model to TensorRT engine with specified precision.

### `quantize_model(model, calibration_data, method)`
Applies INT8 or FP16 quantization to model.

### `profile_performance(model, device, num_iterations)`
Measures inference latency, throughput, and memory usage.

### `deploy_to_device(package, target_ip, config)`
Deploys complete package to target Jetson device.

## Examples

### Example 1: Complete Deployment Pipeline
```python
from edge_deploy import JetsonDeployment

# Initialize deployment manager
deployer = JetsonDeployment(
    model_path='vla_model.pth',
    jetson_ip='192.168.1.100',
    jetson_model='orin_nano'
)

# Step 1: Export to ONNX
deployer.export_onnx(
    sample_inputs={
        'image': torch.randn(1, 3, 224, 224),
        'robot_state': torch.randn(1, 48)
    },
    output_path='model.onnx'
)

# Step 2: Convert to TensorRT
deployer.convert_tensorrt(
    onnx_path='model.onnx',
    precision='fp16',
    max_batch_size=1
)

# Step 3: Test locally
latency = deployer.benchmark(num_runs=100)
print(f"Average latency: {latency:.2f}ms")

# Step 4: Deploy to Jetson
deployer.deploy(
    include_ros2=True,
    auto_start=True
)
```

### Example 2: INT8 Calibration
```python
import tensorrt as trt

class Int8Calibrator(trt.IInt8EntropyCalibrator2):
    def __init__(self, calibration_dataset, cache_file):
        super().__init__()
        self.dataset = calibration_dataset
        self.cache_file = cache_file
        self.current_index = 0

    def get_batch_size(self):
        return 1

    def get_batch(self, names):
        if self.current_index < len(self.dataset):
            batch = self.dataset[self.current_index]
            self.current_index += 1
            return [int(batch.data_ptr())]
        return None

    def read_calibration_cache(self):
        if os.path.exists(self.cache_file):
            with open(self.cache_file, 'rb') as f:
                return f.read()
        return None

    def write_calibration_cache(self, cache):
        with open(self.cache_file, 'wb') as f:
            f.write(cache)
```

### Example 3: Jetson Power Mode Configuration
```bash
#!/bin/bash
# Set Jetson to maximum performance mode

# For Orin
sudo nvpmodel -m 0  # MAXN mode
sudo jetson_clocks

# Monitor power and thermal
sudo tegrastats

# Set custom power limits (15W for Orin Nano)
sudo /usr/sbin/nvpmodel -m 1  # 15W mode
```

### Example 4: Docker Deployment
```dockerfile
FROM nvcr.io/nvidia/l4t-pytorch:r35.2.1-pth2.0-py3

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-desktop \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
COPY requirements.txt /app/
RUN pip3 install -r /app/requirements.txt

# Copy model and code
COPY models/ /app/models/
COPY src/ /app/src/

# Set up ROS2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKDIR /app
CMD ["python3", "src/robot_controller.py"]
```

## Dependencies
- NVIDIA JetPack 5.1+ (for Orin/Xavier)
- TensorRT 8.5+
- CUDA 11.4+
- cuDNN 8.6+
- PyTorch (with CUDA support)
- ONNX Runtime
- ROS2 Humble (for Jetson)
- Docker (optional)

## Best Practices
- Use FP16 precision for 2-3x speedup on Jetson
- Implement INT8 quantization for 4x speedup (with calibration)
- Profile thermal performance under sustained load
- Use CUDA streams for concurrent execution
- Implement graceful degradation on thermal throttling
- Cache TensorRT engines (don't rebuild on every run)
- Use Docker for reproducible deployments
- Monitor power consumption and adjust nvpmodel
- Implement watchdog for automatic recovery
- Use systemd for process management
- Test over full range of operating conditions

## Jetson Platform Comparison

### Orin Nano (8GB)
- GPU: 1024-core NVIDIA Ampere
- TOPS: 40 AI TOPS
- Power: 7-15W
- Use case: Humanoid robots, mobile manipulation

### Orin NX (16GB)
- GPU: 1024-core NVIDIA Ampere
- TOPS: 70 AI TOPS
- Power: 10-25W
- Use case: Advanced perception, multi-model inference

### AGX Orin (32/64GB)
- GPU: 2048-core NVIDIA Ampere
- TOPS: 200-275 AI TOPS
- Power: 15-60W
- Use case: Full autonomous systems, heavy ML workloads

## Performance Optimization Strategies

### Model-Level
- Prune unnecessary layers
- Use efficient architectures (MobileNet, EfficientNet)
- Quantization-aware training
- Knowledge distillation

### Inference-Level
- Batch processing where possible
- CUDA graph capture for static graphs
- Use TensorRT plugins for custom ops
- Multi-stream concurrent inference

### System-Level
- Pin processes to specific CPU cores
- Use CUDA managed memory
- Implement thermal-aware throttling
- Optimize data transfer (minimize CPU-GPU copies)

## Integration Points
- Deploys models from VLA_Controller
- Optimizes models trained in IsaacSim_Pipeline
- Runs alongside ROS2_Core on edge
- Interfaces with Hardware_Proxy for robot control
- Connects to Unity_Vis for remote visualization
