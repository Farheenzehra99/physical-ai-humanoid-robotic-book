---
sidebar_position: 1
slug: /
title: Welcome to Physical AI & Humanoid Robotics
description: Zero to Walking Robot in 13 Weeks - Complete Educational Resource
---

# Physical AI & Humanoid Robotics

> **Zero to Walking Robot in 13 Weeks** - A production-grade, fully executable educational resource for building AI-controlled humanoid robots from scratch.

## 🎯 Course Overview

This 13-week university capstone quarter takes students with **zero prior humanoid robotics experience** and guides them through building an 18-DOF humanoid robot, training AI walking policies in NVIDIA Isaac Sim, optimizing for edge deployment on Jetson hardware, and deploying to real robots.

### What Makes This Different?

✅ **Zero Broken Code** - Every example tested on Ubuntu 22.04 + ROS2 + Isaac Sim
✅ **Zero Dead Links** - Automated CI/CD link checking on every PR
✅ **Zero Friction** - Copy-paste ready terminal commands, exact version pinning
✅ **Production-Grade** - 100/100 Lighthouse score, \&lt; 2s load time, \&lt; 15MB total size

## 📚 Curriculum Structure

### Weeks 1-3: ROS2 Foundations & Simulation
- Ubuntu 22.04 + ROS 2 Jazzy/Iron setup
- 18-DOF humanoid URDF design and kinematics
- Gazebo simulation with ZMP walking controller

### Weeks 4-6: GPU-Accelerated RL & VLA
- NVIDIA Isaac Sim with 1024+ parallel environments
- PPO policy training for walking (Stable-Baselines3)
- OpenVLA fine-tuning for language-conditioned tasks

### Module 4: Vision-Language-Action (VLA)
- **Chapter 8**: Voice-to-Action with OpenAI Whisper for speech recognition
- **Chapter 9**: Cognitive Planning with LLMs for task decomposition
- **Chapter 10**: Capstone Project - Autonomous humanoid demo

### Weeks 7-9: Visualization & Edge Deployment
- Unity integration for 4K cinematic renders
- TensorRT optimization (FP16/INT8 quantization)
- Jetson Orin deployment (\&lt; 50ms inference latency)

### Weeks 10-13: Hardware Integration & Sim-to-Real
- Unitree SDK and CAN bus motor control
- Real-time sensor integration (IMU, RealSense, encoders)
- **Final demo: 30-minute continuous walking operation**

## 🚀 Quick Start

### Prerequisites

**Hardware (Minimum):**
- Ubuntu 22.04 LTS machine
- NVIDIA RTX 4070 Ti (12GB VRAM) or cloud GPU (Lambda Labs/Paperspace)
- *Optional for Weeks 8-13:* NVIDIA Jetson Orin Nano + Unitree Go2/G1 robot

**Software:**
- Python 3.10+
- Node.js 18 LTS
- ROS 2 Jazzy or Iron
- NVIDIA Isaac Sim 2024.1+

### Installation

```bash
# Clone repository
git clone https://github.com/your-org/physical-ai-humanoid-robotics.git
cd physical-ai-humanoid-robotics

# Install Python dependencies
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Install Node.js dependencies
npm install

# Start local development server
npm start
```

Visit [http://localhost:3000](http://localhost:3000) to view the documentation site.

## 🎥 Learning Experience

### Immersive Video Content

Every week includes:
- **Full-bleed chapter opener** (4K, 30 seconds) - Cinematic preview of the week's goals
- **Embedded tutorials** - Step-by-step walkthroughs with synchronized code
- **Demo showcases** - Real robot validation and capstone demos

### Interactive Components

- **Skill Invocation Cards** - Execute ROS2 commands directly from the docs
- **Parameter Playgrounds** - Adjust URDF parameters and see visualizations
- **Progress Tracking** - Checklist validation for each chapter

## 💡 What You'll Build

By Week 13, you'll have:

1. **18-DOF Humanoid URDF** - Validated kinematics and collision models
2. **Gazebo Walking Controller** - ZMP-based walking at 0.1 m/s
3. **Isaac Sim RL Policy** - Trained PPO policy (10M timesteps, 0.5 m/s)
4. **OpenVLA Integration** - Language-conditioned task execution
5. **Voice-to-Action Pipeline** - OpenAI Whisper + LLM task planning
6. **TensorRT Engine** - Optimized for Jetson Orin Nano (\&lt; 50ms latency)
7. **Hardware Deployment** - Walking on Unitree Go2/G1 or custom robot
8. **30-Minute Demo** - Continuous walking operation with no falls

## 📖 Documentation Structure

- **Weekly Chapters** - 5 chapters per week (65 total)
- **Skills** - 8 reusable capabilities (ROS2, URDF, Gazebo, Isaac Sim, VLA, etc.)
- **Agents** - 3 orchestration workflows for complex tasks
- **Appendix** - Hardware alternatives, cloud GPU setup, troubleshooting

## 🤝 Community & Support

- **GitHub Discussions** - Ask questions and share projects
- **Discord Server** - Real-time community chat
- **Office Hours** - Weekly Q&A sessions (for university adoptions)
- **Hardware Lab Access** - Partner universities with Unitree robots

## 🏆 Success Metrics

**Target Outcomes (Year 1):**
- 1,000+ students complete 13-week capstone
- 50+ university adoptions worldwide
- 10+ real robots deployed and walking
- > 4.5/5.0 average student satisfaction

## 📜 License

This project is licensed under the MIT License - see [LICENSE](https://github.com/your-org/physical-ai-humanoid-robotics/blob/main/LICENSE) for details.

## 🙏 Acknowledgments

- **NVIDIA Isaac Sim** for GPU-accelerated simulation
- **Unitree Robotics** for affordable humanoid platforms
- **ROS 2 Community** for middleware and tooling
- **Docusaurus** for beautiful documentation sites
- **OpenVLA** for open-source vision-language-action models

---

**Ready to begin?** Start with [Module 1: ROS 2 Foundations](/docs/module-01-foundations/chapter-01-ros2-nervous-system/page-01-what-problem-does-ros2-solve)
