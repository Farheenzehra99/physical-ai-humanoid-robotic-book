# Isaac SDK Setup - Required Assets

This directory contains all visual assets (screenshots, videos, diagrams) for the **Page 20: Setting Up Isaac SDK** documentation.

## Constitutional Compliance

All assets must adhere to:
- **Zero-Tolerance Quality (#3)**: Professional-grade visuals
- **Structured Documentation (#4)**: Consistent naming, \&lt;200KB file sizes
- **Performance-Driven Design (#6)**: WebP format, optimized for web

## Required Screenshots

### 1. Hero Image
- **Filename:** `isaac-sim-hero-screenshot.webp`
- **Alt Text:** "NVIDIA Isaac Sim 2024.1 interface displaying a bipedal humanoid robot navigating a warehouse environment with physics simulation active"
- **Dimensions:** 1200x675px (16:9)
- **Max Size:** \&lt;200KB
- **Source:** Isaac Sim viewport with humanoid robot in warehouse scene
- **Tool:** Isaac Sim built-in screenshot tool (Ctrl+Alt+S)
- **Post-processing:** Crop to 16:9, convert to WebP with `scripts/optimize-images.sh`

### 2. Omniverse Launcher Welcome Screen
- **Filename:** `omniverse-launcher-welcome.webp`
- **Alt Text:** "NVIDIA Omniverse Launcher welcome screen with 'Sign In' button and Isaac Sim app tile visible"
- **Dimensions:** 1000x750px (4:3)
- **Max Size:** \&lt;150KB
- **Source:** Fresh install of Omniverse Launcher
- **Capture:** Window screenshot after first launch

### 3. Installation Progress
- **Filename:** `isaac-sim-installation-progress.webp`
- **Alt Text:** "Omniverse Launcher showing Isaac Sim 2024.1 installation progress at 45% with 'Downloading Physics Engine' status"
- **Dimensions:** 1000x750px (4:3)
- **Max Size:** \&lt;150KB
- **Source:** Mid-installation screenshot from Launcher
- **Timing:** Capture during active download/install phase

### 4. Verification Success Terminal
- **Filename:** `verification-success-terminal.webp`
- **Alt Text:** "Terminal window showing successful Isaac Sim verification with green checkmarks for API, CUDA, and RTX GPU detection"
- **Dimensions:** 800x450px (16:9)
- **Max Size:** \&lt;100KB
- **Source:** Terminal output from `python3 test_installation.py`
- **Tool:** Terminal screenshot tool (Ubuntu: `gnome-screenshot`, Windows: Snipping Tool)

### 5. Isaac Sim GUI Warehouse Scene
- **Filename:** `isaac-sim-gui-warehouse.webp`
- **Alt Text:** "Isaac Sim 2024.1 GUI showing default warehouse scene with robotic arm, physics simulation toolbar, and viewport controls"
- **Dimensions:** 1600x900px (16:9)
- **Max Size:** \&lt;200KB
- **Source:** Isaac Sim default scene loaded
- **Capture:** Full window screenshot after first successful launch

## Required Videos

### 1. First Launch Walkthrough
- **Filename:** `isaac-sim-first-launch.mp4`
- **Alt Text:** "Video walkthrough: Launching Isaac Sim for the first time, navigating the viewport, and starting a physics simulation"
- **Duration:** 90 seconds
- **Resolution:** 1920x1080 (1080p)
- **Max Size:** \&lt;10MB
- **Codec:** H.264, 30fps
- **Content:**
  - 0:00-0:15 - Launching Isaac Sim from Launcher
  - 0:15-0:45 - Navigating viewport (WASD controls, mouse rotation)
  - 0:45-1:15 - Loading warehouse scene
  - 1:15-1:30 - Pressing Play button, physics simulation starts
- **Tool:** OBS Studio (free screen recorder)
- **Post-processing:** Trim, add subtitles, compress to &lt;10MB

## Required Diagrams

### 1. Isaac Sim Architecture Overview
- **Filename:** `isaac-sim-architecture-diagram.webp`
- **Alt Text:** "Architecture diagram showing Isaac Sim components: Omniverse Core, PhysX Engine, RTX Renderer, ROS 2 Bridge, and Python API"
- **Dimensions:** 1200x800px (3:2)
- **Max Size:** &lt;180KB
- **Content:**
  - Box diagram with 5 components
  - Arrows showing data flow
  - Color-coded by function (rendering, physics, integration)
- **Tool:** Excalidraw (excalidraw.com) or draw.io
- **Export:** High-DPI PNG (2x), then convert to WebP

## File Naming Convention

All assets must follow this pattern:
```
<descriptive-name>.<extension>
```

Examples:
- ✅ `isaac-sim-hero-screenshot.webp`
- ✅ `omniverse-launcher-welcome.webp`
- ❌ `screenshot1.png`
- ❌ `IMG_20240315.jpg`

## Image Optimization Workflow

1. **Capture** the asset in original format (PNG/JPG for images, MP4 for videos)
2. **Edit** as needed (crop, annotate, add callouts)
3. **Optimize** using the provided script:
   ```bash
   cd /path/to/Physical_AI_Humanoid_Robotics
   ./scripts/optimize-images.sh docs/module-03-ai-robot-brain/chapter-07-isaac-sim-sdk/assets/
   ```
4. **Verify** file size is under limits:
   ```bash
   du -h assets/*.webp
   ```
5. **Commit** to Git with descriptive commit message

## Screenshot Guidelines

### Best Practices
- **Clean interface**: Hide personal info, close unnecessary windows
- **High contrast**: Ensure text is readable (use light/dark theme consistently)
- **Annotations**: Use red arrows/boxes to highlight important UI elements
- **Consistency**: Same window size, same zoom level across related screenshots

### Tools Recommendation
- **Ubuntu**: Shutter (supports annotations), Flameshot
- **Windows**: Greenshot (free, supports annotations)
- **macOS**: Skitch (Evernote)

## Video Guidelines

### Best Practices
- **No audio** (or professional voiceover if needed)
- **Add subtitles** for accessibility
- **Smooth mouse movements** (avoid jerky cursor)
- **30fps minimum** for smooth playback
- **Compress** using HandBrake or FFmpeg:
  ```bash
  ffmpeg -i input.mp4 -c:v libx264 -crf 23 -preset medium -c:a aac -b:a 128k output.mp4
  ```

## Diagram Guidelines

### Color Palette (Constitutional Compliance)
Use the project color palette:
- **Deep Space Black:** `#0A0E27` (backgrounds)
- **Electric Cyan:** `#00D9FF` (primary elements, arrows)
- **Tesla Bot Silver:** `#C8D0D9` (text, borders)

### Typography
- **Font:** Inter, Roboto, or system sans-serif
- **Size:** 16px minimum for body text, 24px for headings

## Asset Status Checklist

- [ ] Hero image: `isaac-sim-hero-screenshot.webp`
- [ ] Launcher welcome: `omniverse-launcher-welcome.webp`
- [ ] Installation progress: `isaac-sim-installation-progress.webp`
- [ ] Verification terminal: `verification-success-terminal.webp`
- [ ] GUI warehouse scene: `isaac-sim-gui-warehouse.webp`
- [ ] First launch video: `isaac-sim-first-launch.mp4`
- [ ] Architecture diagram: `isaac-sim-architecture-diagram.webp`

## Notes for Asset Creators

When creating these assets, you will need:
- A working Isaac Sim 2024.1 installation
- An RTX-capable NVIDIA GPU
- Screen recording software (OBS Studio recommended)
- Image editor (GIMP, Photoshop, or Figma)
- WebP conversion tool (already included in `scripts/optimize-images.sh`)

If you encounter issues capturing these assets, refer to:
- **Troubleshooting Guide:** `page-20-setting-up-isaac-sdk.mdx` (Troubleshooting section)
- **Isaac Sim Docs:** https://docs.omniverse.nvidia.com/isaacsim/latest/
- **Omniverse Forums:** https://forums.developer.nvidia.com/c/omniverse/

---

**Last Updated:** 2025-12-09
**Contact:** Physical AI Humanoid Robotics Documentation Team
