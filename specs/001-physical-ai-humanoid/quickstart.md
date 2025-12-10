# Quick Start Guide: Physical AI & Humanoid Robotics

**Feature**: 001-physical-ai-humanoid
**Date**: 2025-12-06
**Purpose**: Installation validation checklist for Ubuntu 22.04 + ROS2 + Isaac Sim

---

## Prerequisites Checklist

Before starting Week 1, ensure you have the following hardware and software prerequisites.

### ✅ Hardware Requirements

**Minimum (Weeks 1-3 only)**:
- [ ] Ubuntu 22.04 LTS machine (native install recommended, not WSL2)
- [ ] 16GB RAM
- [ ] 100GB free disk space
- [ ] Integrated GPU or better

**Recommended (Weeks 1-9)**:
- [ ] Ubuntu 22.04 LTS machine
- [ ] 32GB RAM
- [ ] 500GB SSD
- [ ] NVIDIA RTX 4070 Ti (12GB VRAM) or equivalent
  - Alternatives: RTX 3080 (10GB), RTX 4090 (24GB)
  - Cloud option: Lambda Labs RTX 4090 instance

**Optional (Weeks 10-13 - Hardware Integration)**:
- [ ] NVIDIA Jetson Orin Nano (8GB) - $499
- [ ] Intel RealSense D455 depth camera - $400
- [ ] Unitree Go2 Edu robot - $2,700
  - Alternative: Custom DIY humanoid ($1,000-$3,000)

---

## Step 1: Ubuntu 22.04 Setup

### 1.1 Verify Ubuntu Version

```bash
lsb_release -a
```

**Expected output**:
```
Distributor ID: Ubuntu
Description:    Ubuntu 22.04.3 LTS
Release:        22.04
Codename:       jammy
```

**Validation**:
- [ ] Ubuntu version is exactly 22.04.x LTS
- [ ] Codename is "jammy"

### 1.2 Update System Packages

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential git curl wget vim
```

**Validation**:
- [ ] All packages updated without errors
- [ ] `gcc --version` shows GCC 11.x
- [ ] `git --version` shows Git 2.34+

---

## Step 2: NVIDIA Driver Installation (GPU Users Only)

### 2.1 Check GPU Availability

```bash
lspci | grep -i nvidia
```

**Expected output** (example for RTX 4070 Ti):
```
01:00.0 VGA compatible controller: NVIDIA Corporation AD104 [GeForce RTX 4070 Ti] (rev a1)
```

**Validation**:
- [ ] NVIDIA GPU detected

### 2.2 Install NVIDIA Driver 535+

```bash
# Add NVIDIA driver repository
sudo add-apt-repository ppa:graphics-drivers/ppa -y
sudo apt update

# Install recommended driver
sudo ubuntu-drivers autoinstall

# Reboot
sudo reboot
```

After reboot:

```bash
nvidia-smi
```

**Expected output**:
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 535.129.03   Driver Version: 535.129.03   CUDA Version: 12.2    |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  NVIDIA GeForce ...  Off  | 00000000:01:00.0  On |                  N/A |
| 30%   35C    P8    15W / 285W |    512MiB / 12288MiB |      5%      Default |
+-------------------------------+----------------------+----------------------+
```

**Validation**:
- [ ] Driver version is 535 or higher
- [ ] CUDA version is 12.x
- [ ] GPU memory shows correctly (12GB for RTX 4070 Ti)

---

## Step 3: Python 3.10+ Setup

### 3.1 Verify Python Version

```bash
python3 --version
```

**Expected output**:
```
Python 3.10.12
```

**Validation**:
- [ ] Python version is 3.10 or 3.11

### 3.2 Install pip and venv

```bash
sudo apt install -y python3-pip python3-venv
pip3 --version
```

**Expected output**:
```
pip 22.0.2 from /usr/lib/python3/dist-packages/pip (python 3.10)
```

**Validation**:
- [ ] pip3 installed successfully

### 3.3 Create Virtual Environment

```bash
cd ~/
python3 -m venv robotics-venv
source robotics-venv/bin/activate
```

**Expected output**:
```
(robotics-venv) user@machine:~$
```

**Validation**:
- [ ] Virtual environment activated (prompt shows `(robotics-venv)`)

---

## Step 4: ROS 2 Installation

### 4.1 Add ROS 2 Repository

```bash
# Enable Ubuntu universe repository
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

**Validation**:
- [ ] ROS 2 repository added without errors

### 4.2 Install ROS 2 Jazzy (or Iron)

**Option A: ROS 2 Jazzy (Recommended)**:
```bash
sudo apt install -y ros-jazzy-desktop
sudo apt install -y ros-jazzy-ros-base ros-jazzy-ros2bag \
  ros-jazzy-rviz2 ros-jazzy-rqt* ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers
```

**Option B: ROS 2 Iron (Fallback)**:
```bash
sudo apt install -y ros-iron-desktop
sudo apt install -y ros-iron-ros-base ros-iron-ros2bag \
  ros-iron-rviz2 ros-iron-rqt* ros-iron-ros2-control \
  ros-iron-ros2-controllers
```

### 4.3 Source ROS 2 Setup

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

(Replace `jazzy` with `iron` if using ROS 2 Iron)

### 4.4 Verify ROS 2 Installation

```bash
ros2 --version
```

**Expected output**:
```
ros2 cli version 0.31.1
```

**Validation**:
- [ ] ROS 2 CLI installed successfully

### 4.5 Test ROS 2 Talker/Listener

**Terminal 1**:
```bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2**:
```bash
ros2 run demo_nodes_cpp listener
```

**Expected output (Terminal 1)**:
```
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [talker]: Publishing: 'Hello World: 2'
```

**Expected output (Terminal 2)**:
```
[INFO] [listener]: I heard: [Hello World: 1]
[INFO] [listener]: I heard: [Hello World: 2]
```

**Validation**:
- [ ] Talker publishes messages
- [ ] Listener receives messages
- [ ] No errors in either terminal

---

## Step 5: Colcon Build Tool

### 5.1 Install Colcon

```bash
sudo apt install -y python3-colcon-common-extensions
```

### 5.2 Verify Colcon

```bash
colcon version-check
```

**Expected output**:
```
colcon-argcomplete     0.3.3: up to date
colcon-bash            0.4.2: up to date
...
```

**Validation**:
- [ ] Colcon installed successfully

---

## Step 6: Isaac Sim 2024.1+ Setup (Weeks 4-6)

**Note**: Isaac Sim requires an NVIDIA GPU with 8GB+ VRAM. Skip this step if you plan to use cloud GPUs.

### 6.1 Install Omniverse Launcher

1. Download from: https://www.nvidia.com/en-us/omniverse/download/
2. Extract and run installer:

```bash
cd ~/Downloads
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage
```

3. Sign in with NVIDIA account (free registration)

### 6.2 Install Isaac Sim via Omniverse Launcher

1. Open Omniverse Launcher
2. Go to "Exchange" tab
3. Search for "Isaac Sim"
4. Install "Isaac Sim 2024.1.0" (or latest 2024.x patch)

**Installation path**: `~/.local/share/ov/pkg/isaac-sim-2024.1.0/`

### 6.3 Verify Isaac Sim Installation

```bash
cd ~/.local/share/ov/pkg/isaac-sim-2024.1.0/
./python.sh --version
```

**Expected output**:
```
Python 3.10.13
```

### 6.4 Launch Isaac Sim (GUI Test)

```bash
cd ~/.local/share/ov/pkg/isaac-sim-2024.1.0/
./isaac-sim.sh
```

**Expected**: Isaac Sim GUI launches (may take 2-3 minutes on first launch)

**Validation**:
- [ ] Isaac Sim launches without errors
- [ ] GPU acceleration detected (check bottom-right: "RTX: On")

### 6.5 Test Headless Mode (for RL Training)

```bash
cd ~/.local/share/ov/pkg/isaac-sim-2024.1.0/
./python.sh -m omni.isaac.examples.hello_world
```

**Expected output**:
```
[Info] [omni.isaac.core] Simulation running...
```

**Validation**:
- [ ] Headless mode works (no GUI, no errors)

---

## Step 7: Clone This Repository

### 7.1 Clone Repository

```bash
cd ~/
git clone https://github.com/your-org/physical-ai-humanoid-robotics.git
cd physical-ai-humanoid-robotics
```

### 7.2 Install Python Dependencies

```bash
# Activate virtual environment
source ~/robotics-venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

**Validation**:
- [ ] All packages installed without errors

### 7.3 Install Node.js Dependencies (Docusaurus)

```bash
# Install Node.js 18 LTS
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt install -y nodejs

# Verify Node.js
node --version  # Should show v18.x.x

# Install npm dependencies
npm install
```

**Validation**:
- [ ] Node.js 18.x installed
- [ ] npm dependencies installed without errors

### 7.4 Build Docusaurus Site

```bash
npm run build
```

**Expected output**:
```
[SUCCESS] Generated static files in "build".
```

**Validation**:
- [ ] Build completes successfully
- [ ] `build/` directory created

### 7.5 Start Local Development Server

```bash
npm start
```

**Expected**: Browser opens to http://localhost:3000

**Validation**:
- [ ] Docusaurus site loads in browser
- [ ] Navigation works (sidebar, navbar)

---

## Step 8: Hardware Validation (Weeks 8-13 Only)

### 8.1 Jetson Orin Setup (Optional)

**Hardware**: NVIDIA Jetson Orin Nano (8GB)

1. Flash JetPack 5.1.2 to Jetson
2. SSH into Jetson:

```bash
ssh nvidia@jetson-orin-nano.local
```

3. Verify CUDA:

```bash
nvcc --version
```

**Expected output**:
```
Cuda compilation tools, release 11.4, V11.4.315
```

**Validation**:
- [ ] Jetson boots successfully
- [ ] CUDA installed

### 8.2 Unitree Robot Connectivity (Optional)

**Hardware**: Unitree Go2 Edu

1. Power on Unitree Go2
2. Connect via Ethernet (static IP: 192.168.123.161)
3. Ping robot:

```bash
ping 192.168.123.161
```

**Expected output**:
```
64 bytes from 192.168.123.161: icmp_seq=1 ttl=64 time=0.5 ms
```

**Validation**:
- [ ] Robot pingable
- [ ] SSH access works (if supported)

---

## Step 9: Run All Tests

### 9.1 Python Tests

```bash
cd ~/physical-ai-humanoid-robotics
source ~/robotics-venv/bin/activate
pytest tests/ --cov=skills --cov=agents
```

**Expected output**:
```
================================ test session starts =================================
collected 0 items  # (Will increase as tests are added)

---------- coverage: platform linux, python 3.10.12 -----------
Name                     Stmts   Miss  Cover
--------------------------------------------
TOTAL                        0      0   100%
```

**Validation**:
- [ ] All tests pass (or 0 tests if not yet implemented)

### 9.2 ROS 2 Workspace Tests

```bash
cd ~/physical-ai-humanoid-robotics/static/code-examples/week-01/hello_ros2
source /opt/ros/jazzy/setup.bash
colcon build
colcon test
```

**Expected output**:
```
Summary: 1 package finished [0.5s]
```

**Validation**:
- [ ] Colcon build succeeds
- [ ] Colcon tests pass

---

## Step 10: Final Validation Checklist

Mark all items that apply to your setup:

### Core Prerequisites (Required for All Weeks)
- [ ] Ubuntu 22.04 LTS installed
- [ ] Python 3.10+ installed
- [ ] ROS 2 Jazzy or Iron installed
- [ ] Colcon build tool installed
- [ ] This repository cloned
- [ ] Python dependencies installed (requirements.txt)
- [ ] Node.js 18 LTS installed
- [ ] Docusaurus site builds successfully

### GPU Setup (Required for Weeks 4-6)
- [ ] NVIDIA GPU detected (RTX 3070+ or cloud equivalent)
- [ ] NVIDIA Driver 535+ installed
- [ ] CUDA 12.x available
- [ ] Isaac Sim 2024.1+ installed
- [ ] Isaac Sim launches (GUI or headless)

### Edge Deployment (Optional - Weeks 8-9)
- [ ] Jetson Orin Nano/NX/AGX available
- [ ] JetPack 5.1.2+ flashed
- [ ] SSH access to Jetson working

### Hardware Integration (Optional - Weeks 10-13)
- [ ] Unitree Go2/G1/H1 robot available (or custom platform)
- [ ] Robot powers on successfully
- [ ] Ethernet connectivity established
- [ ] Intel RealSense D455 camera available (for VLA)

---

## Troubleshooting

### Issue: ROS 2 not found after installation

**Solution**:
```bash
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### Issue: NVIDIA driver installation fails

**Solution**:
```bash
sudo apt purge nvidia-*
sudo ubuntu-drivers autoinstall
sudo reboot
```

### Issue: Isaac Sim fails to launch

**Solution**:
- Check GPU memory: `nvidia-smi` (must have > 4GB free)
- Update driver: `sudo apt install nvidia-driver-535`
- Reinstall Isaac Sim via Omniverse Launcher

### Issue: Colcon build fails

**Solution**:
```bash
source /opt/ros/jazzy/setup.bash
rm -rf build install log
colcon build --symlink-install
```

---

## Next Steps

Once all validation steps pass:

1. **Start Week 1**: Navigate to `docs/week-01-foundations/01-ubuntu-setup.mdx`
2. **Join Community**: Discord link in README.md
3. **Report Issues**: GitHub Issues for bugs or clarifications

---

**Last Updated**: 2025-12-06
**Tested On**: Ubuntu 22.04.3 LTS + ROS 2 Jazzy + Isaac Sim 2024.1.0
