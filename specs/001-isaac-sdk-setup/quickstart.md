# Isaac Sim Setup - Quick Reference

**Feature**: 001-isaac-sdk-setup
**Last Updated**: 2025-12-09
**Estimated Time**: 20-30 minutes

This is a condensed quick reference for setting up NVIDIA Isaac Sim 2024.1+. For detailed explanations, see the full documentation page.

---

## Prerequisites Checklist

Before starting, verify you have:

- [ ] **Operating System**: Ubuntu 22.04 LTS (recommended) or Windows 11
- [ ] **Python**: 3.10 or later (`python3 --version`)
- [ ] **NVIDIA GPU**: GTX 1060 6GB minimum (RTX 4070 Ti recommended)
- [ ] **VRAM**: 6GB minimum (12GB+ recommended)
- [ ] **RAM**: 16GB minimum (32GB recommended)
- [ ] **Disk Space**: 100GB free (250GB recommended)
- [ ] **Internet**: Stable connection (~20GB download)
- [ ] **NVIDIA Driver**: 525.60.13+ (Linux) or 528.33+ (Windows)

**Check GPU Driver**:
```bash
nvidia-smi
# Should show driver version and GPU details
```

---

## 5-Minute Quick Install (Ubuntu 22.04)

### Method 1: Omniverse Launcher (Recommended for Students)

```bash
# Step 1: Download Omniverse Launcher
🔧 cd ~/Downloads
⬇️  wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Step 2: Make executable and run
📦 chmod +x omniverse-launcher-linux.AppImage
🚀 ./omniverse-launcher-linux.AppImage
```

**Then in the GUI**:
1. Sign in with NVIDIA account (create if needed)
2. Go to **Exchange** tab
3. Search **"Isaac Sim"**
4. Click **Install** on **Isaac Sim 2024.1**
5. Wait 15-30 minutes for installation

### Method 2: Docker Container (Advanced Users)

```bash
# Pull Isaac Sim container from NVIDIA NGC
⬇️  docker pull nvcr.io/nvidia/isaac-sim:2024.1.0

# Run Isaac Sim in headless mode
🚀 docker run --gpus all -it nvcr.io/nvidia/isaac-sim:2024.1.0

# Or with display forwarding (requires X11)
🚀 docker run --gpus all -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -it nvcr.io/nvidia/isaac-sim:2024.1.0
```

---

## 5-Minute Quick Install (Windows 11)

### Using Omniverse Launcher

```powershell
# Step 1: Download installer
⬇️  # Visit https://developer.nvidia.com/isaac-sim
   # Download "Omniverse Launcher for Windows"

# Step 2: Run installer
📦 cd ~\Downloads
🚀 .\omniverse-launcher-windows.exe
   # Follow GUI prompts
```

**Then in the GUI**:
1. Sign in with NVIDIA account
2. **Exchange** tab → Search "Isaac Sim"
3. Install **Isaac Sim 2024.1**

### Using WSL2 + Docker (Alternative)

```powershell
# Enable WSL2 and install Ubuntu
🔧 wsl --install -d Ubuntu-22.04

# Inside WSL2, follow Ubuntu Docker instructions above
```

---

## Verify Installation

### Test 1: Check Isaac Sim Installation Path

```bash
# Ubuntu
ls ~/.local/share/ov/pkg/isaac-sim-2024.1.0/

# Expected: Directory exists with isaac-sim.sh script
```

```powershell
# Windows
dir "$env:LOCALAPPDATA\ov\pkg\isaac-sim-2024.1.0\"

# Expected: Directory exists with isaac-sim.bat script
```

### Test 2: Run Simple Python Script

```python
# test_installation.py
from omni.isaac.kit import SimulationApp

# Start simulation (headless mode for testing)
simulation_app = SimulationApp({"headless": True})

from omni.isaac.core import World

# Create world
world = World()
print("✅ Isaac Sim installed successfully!")

# Cleanup
simulation_app.close()
```

**Run the test**:
```bash
# Ubuntu (source environment first)
🔧 source ~/.local/share/ov/pkg/isaac-sim-2024.1.0/setup_python_env.sh
🧪 python3 test_installation.py
# Expected output: ✅ Isaac Sim installed successfully!
```

```powershell
# Windows
🔧 & "$env:LOCALAPPDATA\ov\pkg\isaac-sim-2024.1.0\setup_python_env.bat"
🧪 python test_installation.py
# Expected output: ✅ Isaac Sim installed successfully!
```

### Test 3: Launch Isaac Sim GUI

```bash
# Ubuntu
🚀 ~/.local/share/ov/pkg/isaac-sim-2024.1.0/isaac-sim.sh
```

```powershell
# Windows
🚀 & "$env:LOCALAPPDATA\ov\pkg\isaac-sim-2024.1.0\isaac-sim.bat"
```

**Expected**: Isaac Sim window opens with default scene.

---

## Hardware Requirements (Quick Reference)

| Component | Minimum | Recommended | Extreme |
|-----------|---------|-------------|---------|
| **GPU** | GTX 1060 (6GB) | RTX 4070 Ti (12GB) | RTX 4090 (24GB) |
| **VRAM** | 6GB | 12GB | 24GB |
| **CPU** | 4-core 2.5GHz | 8-core 3.0GHz | 16-core 3.5GHz |
| **RAM** | 16GB | 32GB | 64GB |
| **Storage** | 100GB SSD | 250GB NVMe | 500GB NVMe |
| **Use Case** | Basic tutorials | Student projects | Production/Research |

**Pricing (Dec 2025 estimates)**:
- RTX 4090: ~$1,599-$1,999 USD
- RTX 4070 Ti: ~$799-$899 USD
- RTX 3080: ~$500-$700 USD (legacy)
- GTX 1060 6GB: ~$150-$250 USD (used only)

---

## Common Issues & Quick Fixes

### ❌ "CUDA initialization failed"

**Quick Fix**:
```bash
# Check driver version
nvidia-smi

# Update drivers (Ubuntu)
sudo apt update && sudo apt install nvidia-driver-535
sudo reboot

# Update drivers (Windows): Download from nvidia.com/drivers
```

### ❌ "ModuleNotFoundError: No module named 'omni'"

**Quick Fix**:
```bash
# Ubuntu: Source Python environment
source ~/.local/share/ov/pkg/isaac-sim-2024.1.0/setup_python_env.sh

# Verify
python3 -c "from omni.isaac.kit import SimulationApp; print('Success!')"
```

```powershell
# Windows: Run setup batch file
& "$env:LOCALAPPDATA\ov\pkg\isaac-sim-2024.1.0\setup_python_env.bat"

# Verify
python -c "from omni.isaac.kit import SimulationApp; print('Success!')"
```

### ❌ "Insufficient VRAM"

**Quick Fix**:
- Reduce simulation complexity (fewer robots/objects)
- Close other GPU applications
- Upgrade GPU if < 6GB VRAM

### ❌ "Permission denied" (Ubuntu)

**Quick Fix**:
```bash
chmod +x ~/.local/share/ov/pkg/isaac-sim-2024.1.0/isaac-sim.sh
```

### ❌ Omniverse Launcher won't start

**Quick Fix (Ubuntu)**:
```bash
# Install missing dependencies
sudo apt install libfuse2 libxcb-cursor0

# Try running from terminal to see errors
./omniverse-launcher-linux.AppImage
```

**Quick Fix (Windows)**:
- Run as Administrator
- Disable antivirus temporarily
- Check Windows Firewall exceptions

---

## Next Steps

After successful installation:

1. **Verify Everything Works**:
   ```bash
   python3 test_installation.py  # Should print success message
   ```

2. **Explore Sample Scenes**:
   - Launch Isaac Sim GUI
   - File → Open → Isaac/Samples/ → Select a demo
   - Press Play button

3. **Run First Tutorial**:
   - 📘 [Chapter 8: Your First Robot Simulation](../chapter-08-first-simulation)
   - Build a simple robot controller
   - Understand simulation loop

4. **Learn Isaac Sim Python API**:
   - 📘 [Official Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
   - Explore `omni.isaac.core` modules
   - Review example scripts in `standalone_examples/`

5. **Join Community**:
   - NVIDIA Isaac Forum: https://forums.developer.nvidia.com/c/ai-and-deep-learning/isaac/
   - Discord: [Physical AI Community] (placeholder)
   - GitHub Discussions: [Repository] (placeholder)

---

## Troubleshooting Resources

**Still having issues?**

- 📖 [Full Installation Guide](./page-20-setting-up-isaac-sdk.mdx) (detailed troubleshooting)
- 🎥 [Video Tutorial: Isaac Sim Installation](https://youtu.be/placeholder)
- 💬 [NVIDIA Isaac Forum](https://forums.developer.nvidia.com/c/ai-and-deep-learning/isaac/)
- 📧 [Physical AI Support](mailto:support@physical-ai-humanoid.org) (placeholder)

**Check System Requirements**:
```bash
# Ubuntu
🔍 lsb_release -a          # OS version
🔍 python3 --version       # Python version
🔍 nvidia-smi              # GPU and driver info
🔍 df -h                   # Disk space

# GPU Compute Capability
🔍 nvidia-smi --query-gpu=compute_cap --format=csv
   # Must be >= 6.1 (Pascal) for Isaac Sim
```

---

## Quick Reference Card

**Installation Commands** (Ubuntu):
```bash
# Download & Install Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage
chmod +x omniverse-launcher-linux.AppImage && ./omniverse-launcher-linux.AppImage

# Source Python Environment
source ~/.local/share/ov/pkg/isaac-sim-2024.1.0/setup_python_env.sh

# Launch Isaac Sim
~/.local/share/ov/pkg/isaac-sim-2024.1.0/isaac-sim.sh
```

**Python Quick Test**:
```python
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True})
from omni.isaac.core import World
world = World()
print("✅ Works!")
simulation_app.close()
```

**GPU Requirements**: 6GB+ VRAM | CUDA 12.0+ | Driver 525.60.13+

**Download Size**: ~20GB | **Disk Space**: 100GB minimum

---

**Last Updated**: 2025-12-09
**Isaac Sim Version**: 2024.1.0
**Tested On**: Ubuntu 22.04 LTS, Windows 11, RTX 4070 Ti

For the full documentation with detailed explanations, screenshots, and advanced troubleshooting, see [Setting Up Isaac SDK (Page 20)](./page-20-setting-up-isaac-sdk.mdx).
