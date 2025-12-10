#!/usr/bin/env python3
"""
Isaac Sim Installation Verification Script
Tests that Isaac Sim Python API is accessible and GPU is detected.

Constitutional Alignment:
- Execution-First Philosophy (#1): Immediate verification
- Zero-Tolerance Quality (#3): Fail fast on missing dependencies

Usage:
    # Using Isaac Sim's bundled Python:
    ~/.local/share/ov/pkg/isaac-sim-2024.1.0/python.sh test_installation.py

    # Or using system Python (if omniverse packages installed):
    python3 test_installation.py

Expected Output:
    ✅ Isaac Sim Python API loaded successfully
    Isaac Sim Version: 2024.1.0
    ✅ CUDA Available: True
       GPU Detected: NVIDIA RTX 4070 Ti
       CUDA Version: 12.1
    ✅ RTX GPU detected (compute capability >= 7.0)

    🎉 Installation verification complete!
"""

import sys
import os


def verify_isaac_sim():
    """Verify Isaac Sim installation and GPU availability."""
    print("🔍 Isaac Sim Installation Verification")
    print("=" * 50)
    print()

    # Test 1: Isaac Sim Python API
    try:
        from omni.isaac.kit import SimulationApp
        print("✅ Isaac Sim Python API loaded successfully")
    except ImportError as e:
        print(f"❌ Failed to import Isaac Sim Python API")
        print(f"   Error: {e}")
        print()
        print("💡 Troubleshooting:")
        print("   1. Ensure Isaac Sim is installed via Omniverse Launcher")
        print("   2. Use Isaac Sim's bundled Python interpreter:")
        print("      ~/.local/share/ov/pkg/isaac-sim-2024.1.0/python.sh test_installation.py")
        print("   3. Verify installation path exists")
        sys.exit(1)

    # Test 2: Check Isaac Sim version
    try:
        import omni.isaac.version as version
        isaac_version = version.get_version()
        print(f"   Isaac Sim Version: {isaac_version}")
    except Exception as e:
        print(f"⚠️  Could not determine Isaac Sim version: {e}")

    print()

    # Test 3: CUDA availability
    try:
        import torch
        if torch.cuda.is_available():
            print("✅ CUDA Available: True")
            device_count = torch.cuda.device_count()
            print(f"   GPU Count: {device_count}")

            for i in range(device_count):
                gpu_name = torch.cuda.get_device_name(i)
                print(f"   GPU {i}: {gpu_name}")

            cuda_version = torch.version.cuda
            print(f"   CUDA Version: {cuda_version}")
        else:
            print("❌ CUDA not available")
            print()
            print("💡 Troubleshooting:")
            print("   1. Check NVIDIA drivers: nvidia-smi")
            print("   2. Ensure driver version >= 525.60.13")
            print("   3. Verify GPU is RTX-capable (GTX 10-series or newer)")
            sys.exit(1)
    except ImportError:
        print("⚠️  PyTorch not available (optional dependency)")
        print("   CUDA check skipped")

    print()

    # Test 4: RTX features (compute capability)
    try:
        compute_capability = torch.cuda.get_device_capability()
        major, minor = compute_capability
        if major >= 7:
            print(f"✅ RTX GPU detected (compute capability {major}.{minor})")
            print("   Ray tracing features supported")
        else:
            print(f"⚠️  Non-RTX GPU detected (compute capability {major}.{minor})")
            print("   Ray tracing features may not work optimally")
    except Exception as e:
        print(f"⚠️  Could not determine compute capability: {e}")

    print()

    # Test 5: VRAM availability
    try:
        for i in range(torch.cuda.device_count()):
            total_vram = torch.cuda.get_device_properties(i).total_memory / (1024**3)  # GB
            print(f"   GPU {i} VRAM: {total_vram:.2f} GB")

            if total_vram < 8:
                print(f"   ⚠️  GPU {i} has < 8GB VRAM (minimum requirement)")
            elif total_vram >= 12:
                print(f"   ✅ GPU {i} meets recommended VRAM (>= 12GB)")
    except Exception as e:
        print(f"⚠️  Could not determine VRAM: {e}")

    print()
    print("=" * 50)
    print("🎉 Installation verification complete!")
    print()
    print("Next steps:")
    print("  1. Launch Isaac Sim GUI: ~/.local/share/ov/pkg/isaac-sim-2024.1.0/isaac-sim.sh")
    print("  2. Load the default warehouse scene")
    print("  3. Press 'Play' to start physics simulation")
    print()
    return True


def verify_python_version():
    """Check Python version compatibility."""
    major, minor = sys.version_info[:2]
    if major == 3 and minor >= 10:
        print(f"✅ Python {major}.{minor} detected (compatible)")
    else:
        print(f"⚠️  Python {major}.{minor} detected")
        print("   Isaac Sim recommends Python 3.10+")
    print()


def verify_environment():
    """Check environment variables and paths."""
    print("🔍 Environment Check")
    print("-" * 50)

    # Check for CUDA_HOME
    cuda_home = os.environ.get('CUDA_HOME') or os.environ.get('CUDA_PATH')
    if cuda_home:
        print(f"   CUDA_HOME: {cuda_home}")
    else:
        print("   ⚠️  CUDA_HOME not set (may be required for custom extensions)")

    # Check for Isaac Sim install path
    isaac_paths = [
        os.path.expanduser("~/.local/share/ov/pkg/isaac-sim-2024.1.0"),
        "C:\\Users\\$USER\\AppData\\Local\\ov\\pkg\\isaac-sim-2024.1.0",
    ]

    isaac_installed = False
    for path in isaac_paths:
        if os.path.exists(path):
            print(f"   ✅ Isaac Sim found: {path}")
            isaac_installed = True
            break

    if not isaac_installed:
        print("   ⚠️  Isaac Sim installation path not found")
        print("      Expected locations:")
        for path in isaac_paths:
            print(f"      - {path}")

    print()


if __name__ == "__main__":
    try:
        verify_python_version()
        verify_environment()
        verify_isaac_sim()
    except KeyboardInterrupt:
        print("\n\n⚠️  Verification interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n❌ Unexpected error during verification:")
        print(f"   {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
