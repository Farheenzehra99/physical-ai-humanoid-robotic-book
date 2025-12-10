# Edge_Deploy Usage Guide

## When to Invoke This Skill

Invoke Edge_Deploy skill when you need to:

### Primary Use Cases
1. **Deploy to Jetson**: Transfer models and code to NVIDIA Jetson devices
2. **Model Optimization**: Convert and optimize models for edge inference
3. **TensorRT Conversion**: Accelerate inference with TensorRT
4. **Quantization**: Apply INT8/FP16 quantization for speed
5. **Performance Profiling**: Measure latency and throughput on target hardware
6. **Edge Infrastructure**: Set up deployment pipelines and monitoring
7. **Power Management**: Configure power modes and thermal limits
8. **Container Deployment**: Create Docker images for edge devices

### Specific Triggers
- User asks to "deploy to Jetson" or "run on edge device"
- User mentions "TensorRT", "quantization", or "optimization"
- User wants to "convert to ONNX" or "optimize model"
- User asks about "edge inference" or "on-device deployment"
- User needs "real-time performance" on embedded hardware
- User mentions "Jetson Orin", "Xavier", or "Nano"
- User asks to "reduce model size" or "speed up inference"
- User wants "power-efficient" deployment

### Context Indicators
- Model needs to run on robot hardware
- Real-time performance required (< 50ms)
- Power constraints exist
- Cloud connectivity unavailable or undesirable
- Latency-sensitive application
- Privacy/security requirements (on-device processing)

## When NOT to Invoke This Skill

Do not invoke when:
- Models only run on desktop/server GPUs
- Cloud deployment is the target
- No edge hardware available
- Task doesn't require optimization (desktop is sufficient)
- User wants simulation only (use Gazebo_Sim or IsaacSim_Pipeline)
- Focus is on model training, not deployment

## Skill Activation Examples

### Example 1: Direct Request
```
User: "Deploy my VLA model to Jetson Orin Nano"
→ Activate Edge_Deploy skill
```

### Example 2: Optimization Request
```
User: "The model is too slow on the robot, help me optimize it with TensorRT"
→ Activate Edge_Deploy skill (TensorRT conversion)
```

### Example 3: Deployment Pipeline
```
User: "Set up automatic deployment to the Jetson when I update the model"
→ Activate Edge_Deploy skill (CI/CD setup)
```

### Example 4: Performance Request
```
User: "Profile the inference speed on Jetson and optimize if needed"
→ Activate Edge_Deploy skill (profiling and optimization)
```

## Workflow Integration

### Standalone Usage
Edge_Deploy can be used independently for model optimization and deployment.

### Combined with Other Skills
- **VLA_Controller + Edge_Deploy**: Optimize VLA models for edge
- **IsaacSim_Pipeline + Edge_Deploy**: Deploy trained RL policies
- **Edge_Deploy + Hardware_Proxy**: Connect optimized models to robot hardware
- **Edge_Deploy + ROS2_Core**: Integrate with ROS2 on Jetson
- **Unity_Vis + Edge_Deploy**: Remote visualization from edge device

## Sequential Usage Pattern
```
1. VLA_Controller or IsaacSim_Pipeline → Train model
2. Edge_Deploy → Export to ONNX
3. Edge_Deploy → Convert to TensorRT
4. Edge_Deploy → Profile performance
5. Edge_Deploy → Deploy to Jetson
6. Hardware_Proxy → Connect to robot actuators
7. ROS2_Core → Integrate with full system
```

## Priority Level
**HIGH** - Critical for deploying AI models on actual robot hardware.

## Expected Outputs
- ONNX model files (.onnx)
- TensorRT engine files (.engine, .plan)
- Deployment scripts (bash, Python)
- Docker images and Dockerfiles
- Performance benchmark reports
- System configuration files
- Deployment documentation
- Monitoring and logging setup

## Common Deployment Scenarios

### Research Prototype
- FP32 initially, optimize later
- SSH-based manual deployment
- Jupyter notebook for testing
- Screen/tmux for process management

### Development System
- FP16 for speed
- Docker containers
- Git-based deployment
- Log aggregation

### Production Robot
- INT8 quantization
- Systemd services
- Automatic startup
- Watchdog monitoring
- OTA update capability
- Fail-safe mechanisms

## Jetson Selection Guide

### For Small Humanoids (< 5kg)
- **Jetson Orin Nano 8GB**: 40 TOPS, 7-15W
- Good for: Basic VLA, lightweight perception

### For Medium Humanoids (5-20kg)
- **Jetson Orin NX 16GB**: 70 TOPS, 10-25W
- Good for: Full VLA, multi-camera perception

### For Advanced Humanoids (> 20kg)
- **AGX Orin 32/64GB**: 200-275 TOPS, 15-60W
- Good for: Complex multi-modal models, heavy ML

## Optimization Priority Order

### 1. Architecture Selection (10-100x speedup)
- Use efficient models (MobileNet, EfficientNet)
- Reduce input resolution if acceptable

### 2. TensorRT Conversion (2-5x speedup)
- Graph optimization
- Kernel auto-tuning
- Layer fusion

### 3. FP16 Quantization (1.5-3x speedup)
- Minimal accuracy loss
- Free on Tensor Cores
- Easy to implement

### 4. INT8 Quantization (2-4x speedup)
- Requires calibration
- Some accuracy loss (< 1% typically)
- Maximum speed

### 5. Model Pruning (1.2-2x speedup)
- Remove less important weights
- Requires retraining
- Complexity varies

## Performance Targets by Jetson Model

### Orin Nano (8GB)
- VLA Inference: 30-50ms
- YOLOv5 (640×640): 20-30ms
- Depth estimation: 40-60ms
- Target workload: 1-2 models concurrently

### Orin NX (16GB)
- VLA Inference: 15-30ms
- YOLOv5 (640×640): 10-15ms
- Depth estimation: 20-30ms
- Target workload: 2-4 models concurrently

### AGX Orin (32GB+)
- VLA Inference: 10-20ms
- YOLOv5 (640×640): 5-10ms
- Depth estimation: 10-20ms
- Target workload: 4-8 models concurrently

## Validation Checklist
- [ ] Model exports to ONNX without errors
- [ ] ONNX model validated with ONNX checker
- [ ] TensorRT engine builds successfully
- [ ] Inference accuracy matches original model (< 1% diff)
- [ ] Latency meets real-time requirements (< 50ms target)
- [ ] Memory usage is within Jetson limits
- [ ] Thermal performance stable under load
- [ ] Power consumption acceptable
- [ ] ROS2 integration works correctly
- [ ] Deployment scripts are tested
- [ ] Automatic startup configured (if needed)
- [ ] Monitoring and logging in place
- [ ] Failure recovery mechanisms tested

## Common Issues and Solutions

### Issue: TensorRT build fails
- **Solution**: Update to compatible ONNX opset, check unsupported ops

### Issue: Inference slower than expected
- **Solution**: Verify TensorRT precision, check power mode (nvpmodel)

### Issue: Out of memory on Jetson
- **Solution**: Reduce batch size, use memory pooling, close other processes

### Issue: Thermal throttling
- **Solution**: Add cooling, reduce power mode, optimize model

### Issue: Model accuracy drops after quantization
- **Solution**: Use calibration dataset, try QAT (Quantization-Aware Training)

### Issue: Can't connect to Jetson via SSH
- **Solution**: Check network, verify IP, ensure SSH service running

## Deployment Best Practices

### Security
- Change default passwords
- Use SSH keys, not passwords
- Enable firewall (ufw)
- Keep JetPack updated
- Use VPN for remote access

### Reliability
- Use systemd for auto-restart
- Implement watchdog timers
- Log to persistent storage
- Monitor disk space
- Set up remote monitoring

### Performance
- Pin processes to CPU cores
- Use CUDA streams
- Enable Jetson clocks for max performance
- Profile regularly
- Cache TensorRT engines

### Maintenance
- Set up OTA updates
- Version control deployment scripts
- Document configuration
- Test rollback procedures
- Keep backups of working configs
