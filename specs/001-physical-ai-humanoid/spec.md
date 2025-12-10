# Feature Specification: Physical AI & Humanoid Robotics - Complete Educational Resource

**Feature Branch**: `001-physical-ai-humanoid`
**Created**: 2025-12-06
**Status**: Draft
**Input**: User description: "Physical AI & Humanoid Robotics Book - Full Specification"

## Target Audience

**University Students** (Junior/Senior/Graduate Level):
- Computer Science students specializing in AI/Robotics
- Mechanical/Electrical Engineering students with programming background
- Students completing capstone projects in robotics or embodied AI
- Research assistants working on humanoid robotics projects
- Prerequisites: Python programming, basic linear algebra, familiarity with ROS (preferred but not required)

**Independent Developers & Researchers**:
- AI/ML engineers transitioning from digital AI to Physical AI
- Robotics engineers seeking to integrate modern foundation models (VLAs, multimodal AI)
- Hobbyists with RTX 4070 Ti+ GPUs building humanoid robots
- Researchers prototyping embodied intelligence systems
- Solo developers building service robots or humanoid platforms

**Institutions & Labs**:
- Universities adopting Physical AI curriculum (13-week capstone quarter)
- Research labs deploying humanoid robots for HRI studies
- Robotics bootcamps and professional training programs
- Industry teams evaluating sim-to-real workflows for production deployment

## Focus

**Bridging Digital AI with Physical Bodies** - Embodied Intelligence Education:

This resource teaches the complete pipeline from simulation to real hardware deployment for humanoid robotics powered by modern AI. The focus is on **executable, production-grade workflows** that enable learners to:

1. **Design & Simulate** humanoid robots with accurate physics in Gazebo and NVIDIA Isaac Sim
2. **Train AI Policies** using GPU-accelerated RL (IsaacGym/Isaac Sim) and Vision-Language-Action models (OpenVLA, RT-1/RT-2)
3. **Optimize for Edge** deployment to NVIDIA Jetson platforms (Orin Nano/NX/AGX) using TensorRT
4. **Deploy to Real Hardware** (Unitree Go2/G1/H1, custom CAN bus humanoids) with validated sim-to-real transfer
5. **Integrate Full Stack** ROS2 control, sensor fusion, navigation, manipulation, and safety systems

**Core Value Proposition**:
- **Zero Broken Code**: Every example tested on Ubuntu 22.04 + ROS 2 Jazzy/Iron + Isaac Sim 2024.1+
- **Sim-to-Real Priority**: All concepts bridge from simulation to hardware deployment
- **Modern AI Stack**: Foundation models (VLAs), RL policies, edge optimization, not just classical control
- **Hardware-in-the-Loop**: Real Jetson + Unitree validation for all deployment modules
- **Reusable Intelligence**: 8 skills + 3 agents auto-invokable from Docusaurus book interface

**NOT a theoretical textbook** - this is a hands-on, project-based curriculum where students build complete humanoid systems from scratch.

## Success Criteria

### Educational Impact
- **SC-001**: Students can progress from "zero humanoid experience" to "deployed AI-controlled robot" within 13 weeks
- **SC-002**: 100% of code examples run successfully on Ubuntu 22.04 + ROS 2 Jazzy/Iron + Isaac Sim 2024.1+ without modifications
- **SC-003**: Students with $700 Jetson Economy Kit can deploy trained policies to real hardware
- **SC-004**: Students with $3,000 Unitree Go2 or custom humanoid complete full capstone project (design → AI → deployment)
- **SC-005**: 10+ real humanoid robots worldwide running book code within 12 months of launch
- **SC-006**: Book becomes de-facto global standard for Physical AI education (measured by university adoptions and community citations)

### Technical Validation
- **SC-007**: Zero broken links or outdated package versions reported in first 6 months post-launch
- **SC-008**: Docusaurus site achieves 100/100 Lighthouse score (performance, accessibility, best practices, SEO)
- **SC-009**: Page load time < 2 seconds on 3G connection, total site size < 15 MB (aggressive optimization)
- **SC-010**: All RL policies train successfully in Isaac Sim with > 0.9 real-time factor on RTX 4070 Ti
- **SC-011**: All VLA models deploy to Jetson Orin with < 50ms inference latency after TensorRT optimization
- **SC-012**: Sim-to-real transfer gap < 10% for validated benchmarks (walking, manipulation, navigation)
- **SC-013**: Hardware integration tested on real Jetson + Unitree/custom robots with documented safety protocols

### User Experience & Accessibility
- **SC-014**: Progressive Web App (PWA) works offline after first load, total built size < 15 MB
- **SC-015**: Every chapter opens with cinematic full-bleed video of humanoid performing the task
- **SC-016**: 4K-resolution diagrams, hero images (Midjourney/Flux generated), video demonstrations for all simulations
- **SC-017**: One-click skill/agent invocation from Docusaurus interface (integrated with Spec-Kit Plus)
- **SC-018**: Works on low-bandwidth connections with service worker caching
- **SC-019**: Mobile-first responsive design, dark/light mode support

### Community & Adoption
- **SC-020**: 10,000+ GitHub stars within first year
- **SC-021**: 50+ universities adopt as official curriculum
- **SC-022**: 1,000+ students complete 13-week capstone within first year
- **SC-023**: 500+ community contributions (PRs, issues, discussions)
- **SC-024**: Active Discord/Slack community with daily engagement

## Constraints

### Platform & Environment
- **CONSTRAINT-001**: Primary development platform MUST be Ubuntu 22.04 LTS (no Windows/macOS primary support)
- **CONSTRAINT-002**: Robot middleware MUST be ROS 2 Jazzy or Iron (not ROS 1)
- **CONSTRAINT-003**: Physics simulation MUST use NVIDIA Isaac Sim 2024.1+ for GPU-accelerated RL (Gazebo for basic physics validation)
- **CONSTRAINT-004**: Documentation site MUST use Docusaurus 3 + MDX (not static site generators like Hugo/Jekyll)
- **CONSTRAINT-005**: Development workflow MUST integrate with Spec-Kit Plus + Claude Code

### Hardware Requirements
- **CONSTRAINT-006**: Minimum GPU for RL training: RTX 4070 Ti (12GB VRAM) or equivalent
- **CONSTRAINT-007**: Target edge device: NVIDIA Jetson Orin Nano/NX/AGX (ARM64 architecture)
- **CONSTRAINT-008**: Supported robot platforms: Unitree Go2/G1/H1, custom CAN bus humanoids (no proprietary closed-source platforms)
- **CONSTRAINT-009**: Depth cameras: Intel RealSense D455 or equivalent (documented alternatives acceptable)

### Software Stack
- **CONSTRAINT-010**: AI/ML framework MUST be PyTorch 2.0+ (not TensorFlow/JAX)
- **CONSTRAINT-011**: Edge optimization MUST use TensorRT 8.5+ (FP16/INT8 quantization)
- **CONSTRAINT-012**: VLA models MUST support OpenVLA, RT-1, or RT-2 architectures (documented, not custom from scratch)
- **CONSTRAINT-013**: RL libraries MUST use Stable-Baselines3, CleanRL, or Isaac Gym/Isaac Sim native APIs
- **CONSTRAINT-014**: All code MUST include type hints (Python 3.10+) and pass linting (black, ruff, isort)

### Quality & Performance
- **CONSTRAINT-015**: Code coverage MUST be > 85% for all skill implementations (unit + integration tests)
- **CONSTRAINT-016**: All code MUST follow Test-Driven Development (TDD): Red → Green → Refactor
- **CONSTRAINT-017**: Simulations MUST run at > 0.9 real-time factor on RTX 4070 Ti, support 512+ parallel envs on RTX 4090
- **CONSTRAINT-018**: Edge inference MUST achieve < 50ms latency on Jetson Orin, use < 8GB memory on Orin Nano
- **CONSTRAINT-019**: Docusaurus site MUST maintain 100/100 Lighthouse score, < 2s load time on 3G, < 15MB total size

### Content & Documentation
- **CONSTRAINT-020**: All terminal commands MUST be copy-paste ready with emoji prefix for visual clarity
- **CONSTRAINT-021**: Hardware specifications MUST include exact model numbers, current prices (Dec 2025 baseline), and purchase links
- **CONSTRAINT-022**: All dependencies MUST be pinned with exact versions and tested installation scripts
- **CONSTRAINT-023**: Video demonstrations MUST be included for all simulations (YouTube unlisted or IPFS-hosted)
- **CONSTRAINT-024**: Citations MUST follow IEEE style with clickable hyperlinks
- **CONSTRAINT-025**: Every module MUST include: learning objectives, prerequisites, step-by-step instructions, expected outputs, troubleshooting guide

### Licensing & Accessibility
- **CONSTRAINT-026**: Original content MUST be MIT License (hardware SDKs follow vendor licenses)
- **CONSTRAINT-027**: No paywalls or login requirements for core educational content (authentication optional for progress tracking/certifications)
- **CONSTRAINT-028**: Must work on low-bandwidth connections, support offline mode after first load (PWA)

### Safety & Validation
- **CONSTRAINT-029**: All hardware deployments MUST include validated safety protocols (emergency stop, joint limits, temperature monitoring)
- **CONSTRAINT-030**: All educational modules MUST be validated on real hardware before publication (no simulation-only content for deployment chapters)

## Not Building

### What This Project is NOT

**NOT Hardware Manufacturing**:
- We are NOT designing custom motors, actuators, or mechanical components
- We are NOT providing CAD files for 3D-printing full humanoid frames
- We are NOT competing with hardware vendors (Unitree, Boston Dynamics, Agility Robotics)
- We ARE teaching how to integrate with existing platforms (Unitree Go2/G1/H1, custom CAN bus systems)

**NOT Vendor-Specific Tutorials**:
- We are NOT creating branded tutorials for specific commercial platforms (e.g., "Boston Dynamics Spot Tutorial")
- We are NOT limiting content to single-vendor ecosystems
- We ARE providing vendor-agnostic workflows that adapt to Unitree, custom humanoids, and documented alternatives

**NOT Beginner Programming Content**:
- We are NOT teaching Python basics, Git fundamentals, or Linux command line
- We are NOT a "first robotics course" - students need prior programming experience
- We ARE assuming Python fluency, basic ROS understanding (or willingness to learn), and comfort with terminal

**NOT Classical Robotics Only**:
- We are NOT focusing solely on PID controllers, inverse kinematics, and classical control theory
- We are NOT ignoring modern foundation models and learned policies
- We ARE emphasizing GPU-accelerated RL, VLA models, and sim-to-real transfer (classical control is supplementary)

**NOT Simulation-Only Education**:
- We are NOT creating another "theoretical robotics textbook with simulations"
- We are NOT stopping at Gazebo demos without hardware validation
- We ARE requiring hardware-in-the-loop testing for all deployment modules (Jetson + real robots)

**NOT General AI/ML Course**:
- We are NOT teaching general deep learning, computer vision, or NLP as standalone topics
- We are NOT a replacement for CS229, CS231n, or similar ML courses
- We ARE focusing specifically on embodied AI (physical deployment, sensor-motor loops, real-world constraints)

**NOT Windows/macOS Primary Support**:
- We are NOT providing native Windows or macOS installation instructions as primary paths
- We are NOT testing on Windows Subsystem for Linux (WSL) or macOS Docker
- We ARE supporting Ubuntu 22.04 LTS exclusively as the primary development platform (users can adapt at their own risk)

**NOT Cloud-Dependent**:
- We are NOT requiring cloud compute subscriptions (AWS, GCP, Azure) for core curriculum
- We are NOT forcing users to rent cloud GPUs for training
- We ARE assuming local RTX 4070 Ti or better (cloud use is optional for users without hardware)

**NOT Closed Ecosystem**:
- We are NOT creating proprietary tools or locked-in platforms
- We are NOT requiring paid licenses for core functionality
- We ARE open source (MIT License), standards-based (ROS2, URDF, USD), and vendor-neutral

**NOT Real-Time Operating System (RTOS) Focus**:
- We are NOT teaching low-level RTOS programming or bare-metal embedded systems
- We are NOT targeting microcontrollers (Arduino, STM32) as primary compute
- We ARE using Linux-based systems (Jetson) with soft real-time constraints (ROS2 DDS)

**NOT Multi-Year Research PhD Thesis**:
- We are NOT exploring cutting-edge unsolved research problems as primary content
- We are NOT requiring novel contributions or publications to complete the curriculum
- We ARE teaching production-ready, validated techniques that work today (research extensions are optional)

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Complete Humanoid Capstone (13-Week Project) (Priority: P1)

A university senior completing their capstone quarter builds a complete 18-DOF humanoid robot system from scratch, trains an AI walking policy in Isaac Sim, optimizes it for Jetson Orin Nano, and deploys to real custom hardware.

**Why this priority**: This is the flagship use case - the "North Star" that defines the entire curriculum. Success here validates all 8 skills + 3 agents working together. This delivers maximum educational value and demonstrates complete sim-to-real pipeline mastery.

**Independent Test**: Can be fully tested by having a student with zero prior humanoid experience complete the 13-week curriculum and successfully demonstrate a walking robot at capstone presentation. Delivers a functioning, deployable humanoid system.

**Acceptance Scenarios**:

1. **Given** student has Ubuntu 22.04 + RTX 4070 Ti laptop, **When** they follow Week 1-3 chapters (URDF design, Gazebo simulation, ROS2 control), **Then** they have a stable simulated 18-DOF humanoid responding to teleoperation commands in Gazebo
2. **Given** student has completed simulation setup, **When** they follow Week 4-7 chapters (Isaac Sim RL training), **Then** they train a walking policy achieving > 0.8 m/s stable locomotion in 1024 parallel environments within 12 hours on RTX 4070 Ti
3. **Given** student has trained walking policy, **When** they follow Week 8-9 chapters (Edge Deploy), **Then** they export to TensorRT INT8, achieve < 30ms inference on Jetson Orin Nano, with < 5% accuracy degradation
4. **Given** student has $700 Jetson Economy Kit (Orin Nano + RealSense D455), **When** they follow Week 10-11 chapters (Hardware Proxy), **Then** they connect to custom CAN bus humanoid, implement safety systems (e-stop, joint limits), and achieve stable standing
5. **Given** student has integrated hardware, **When** they deploy optimized walking policy to robot's Jetson, **Then** robot walks forward at > 0.3 m/s on flat ground with sim-to-real gap < 15%, passing safety validation (no falls, e-stop functional)
6. **Given** complete system deployed, **When** student runs final integration tests (Week 12-13), **Then** system operates for 30+ minutes continuous runtime, logs all diagnostics, and student presents working demo at capstone

---

### User Story 2 - Service Robot VLA Integration (Existing Robot + New AI) (Priority: P2)

An independent developer with an existing Unitree H1 humanoid adds Vision-Language-Action control, enabling language-conditioned task execution ("pick up the red box", "navigate to the kitchen") by fine-tuning OpenVLA and deploying to the robot's onboard Jetson.

**Why this priority**: This validates AI-focused workflow (skip hardware design, use existing platform). Critical for practitioners with hardware who want to add modern foundation model control. Tests VLA_Controller + Edge_Deploy + Hardware_Proxy skills without requiring full robot build.

**Independent Test**: Can be tested by developer with Unitree H1 (or Go2) following VLA integration chapters and demonstrating language-based task execution on real hardware. Delivers production-ready AI upgrade to existing robot.

**Acceptance Scenarios**:

1. **Given** developer has Unitree H1 with official SDK, **When** they follow VLA setup chapter (validate robot model, test connectivity), **Then** they establish stable Ethernet connection, verify joint control, and load H1 URDF into Isaac Sim
2. **Given** Isaac Sim environment ready, **When** they collect 200+ teleoperation demonstrations across 10 task scenarios (manipulation, navigation, gestures), **Then** dataset includes synchronized images (224×224), robot states, actions, and task descriptions (total ~15GB)
3. **Given** demonstration dataset collected, **When** they follow OpenVLA fine-tuning chapter (LoRA fine-tuning on H1 tasks), **Then** fine-tuned model achieves > 85% validation accuracy on held-out task scenarios after 12 epochs (~8 hours on RTX 4090)
4. **Given** fine-tuned VLA model, **When** they follow Edge Deploy chapter (ONNX export, TensorRT INT8 quantization), **Then** optimized model runs at < 45ms inference on Jetson AGX Orin (H1's onboard compute), with < 3% accuracy degradation
5. **Given** optimized model deployed to H1's Jetson, **When** developer issues language commands ("pick up the blue mug", "walk to the charging station"), **Then** robot executes tasks with > 80% success rate, logs all actions, and completes tasks safely (collision avoidance, joint limits enforced)

---

### User Story 3 - Rapid Sim-to-Real Transfer (Pretrained Policy Deployment) (Priority: P3)

A researcher has a pretrained walking policy from IsaacGym and needs to deploy it to custom quadruped hardware within 1 week for a conference demo. They use Edge_Deploy + Hardware_Proxy skills to optimize, deploy, and validate on real robot.

**Why this priority**: This validates the fast-track deployment workflow for users with existing AI models. Important for time-sensitive projects (demos, competitions, rapid prototyping). Tests edge optimization and hardware integration without requiring full training pipeline.

**Independent Test**: Can be tested by researcher with pretrained PyTorch policy following rapid deployment chapters and demonstrating working policy on real quadruped within 5 business days. Delivers conference-ready demo.

**Acceptance Scenarios**:

1. **Given** researcher has pretrained PyTorch walking policy (from IsaacGym), **When** they validate in Gazebo (Day 1), **Then** policy runs in simulation with minor instability (acceptable for prototype)
2. **Given** policy validated in sim, **When** they follow Edge_Deploy rapid optimization chapter (ONNX export, TensorRT FP16 quantization, Day 2), **Then** optimized model runs at < 20ms inference on Jetson Orin NX
3. **Given** optimized model ready, **When** they follow Hardware_Proxy quick setup (CAN bus connection, safety config, Day 3), **Then** they establish communication with custom quadruped, verify motor responsiveness, and implement emergency stop
4. **Given** hardware connected, **When** they deploy model to robot's Jetson and run initial tests (Day 4-5), **Then** robot walks forward at > 0.25 m/s after gain tuning and action smoothing, with stable operation for 10+ minute test sessions
5. **Given** system operational, **When** they run final validation (Day 6-7), **Then** robot demonstrates reliable walking for conference demo, all safety systems functional, and performance documented (video, metrics)

---

### User Story 4 - Multi-Robot Coordination System (Advanced Project) (Priority: P4)

A research lab builds a system where two humanoid robots coordinate to move furniture collaboratively, requiring synchronized grasping, force balance, and coordinated motion trained in Isaac Sim and deployed to dual Unitree G1 robots.

**Why this priority**: This validates advanced multi-robot orchestration and complex task learning. Lower priority than single-robot workflows but important for research labs and production scenarios (warehouse, construction). Tests parallel agent execution and multi-robot ROS2 namespacing.

**Independent Test**: Can be tested by lab with two humanoid robots following multi-robot coordination chapters and demonstrating collaborative furniture moving. Delivers research-grade multi-agent system.

**Acceptance Scenarios**:

1. **Given** lab has two Unitree G1 robots, **When** they follow multi-robot setup (spawn dual robots in Gazebo, configure namespaces /robot_1/ and /robot_2/), **Then** both robots controllable independently via ROS2 topics with no namespace conflicts
2. **Given** dual robot simulation ready, **When** they train coordination policy in Isaac Sim (1024 envs × 2 robots = 2048 robot instances, reward includes grasp balance + motion sync), **Then** policy achieves > 85% success rate on collaborative pick-and-carry tasks in simulation
3. **Given** coordination policy trained, **When** they optimize for dual Jetson deployment (both G1 robots have onboard Jetsons), **Then** both models run synchronized at 20Hz control frequency with deterministic timing
4. **Given** models deployed to real G1 robots, **When** they test collaborative furniture moving (table, box, chair), **Then** robots successfully grasp, lift, and transport objects with force balance monitoring and safe motion coordination (> 75% success rate, zero collisions)

---

### User Story 5 - Visualization & HRI Environment Design (Sim-Only Workflow) (Priority: P5)

A game developer creates high-fidelity HRI visualization environments in Unity for a humanoid robot simulation, connecting to ROS2 for real-time robot state visualization, enabling VR walkthroughs and cinematic rendering for educational videos.

**Why this priority**: This validates Unity_Vis skill for users who need visualization/HRI without full hardware deployment. Important for content creation, UX research, and educational video production. Lower priority than deployment workflows but critical for book's visual quality.

**Independent Test**: Can be tested by developer following Unity visualization chapters and producing 4K cinematic renders of humanoid in living room environment with ROS2 real-time sync. Delivers production-quality visualization assets.

**Acceptance Scenarios**:

1. **Given** developer has Unity 2023.2+ with ROS-TCP-Connector, **When** they import humanoid URDF to Unity and set up ROS2 bridge, **Then** robot's joint states sync in real-time from Gazebo to Unity at 60 FPS with < 50ms latency
2. **Given** Unity robot visualization working, **When** they create living room HRI environment (furniture, lighting, materials), **Then** environment renders at 4K resolution with realistic lighting (HDRP), ready for cinematic capture
3. **Given** HRI environment complete, **When** they add VR support (XR Interaction Toolkit) and test with Meta Quest 3, **Then** user can walk through scene in VR, observe robot performing tasks, and interact with environment at stable 90 FPS
4. **Given** full Unity setup operational, **When** they record cinematic sequences (Timeline + Cinemachine), **Then** they produce 4K 60fps video of humanoid performing manipulation tasks in living room, suitable for chapter opening videos

---

### Edge Cases

- **What happens when simulation trains successfully but hardware deployment completely fails?**
  - Troubleshooting guide covers: domain randomization insufficient, actuation limits mismatched, sensor noise higher than simulated, physics timestep mismatch. Iterative loop: adjust sim → retrain → redeploy.

- **How does system handle students without recommended GPU (< RTX 4070 Ti)?**
  - Book provides cloud alternatives (Lambda Labs, Paperspace), reduced parallel env counts for lower-tier GPUs (RTX 3080 can run 256 envs), and CPU-only fallback for basic Gazebo testing (no RL training).

- **What if target hardware platform (Unitree H1) is discontinued or unavailable?**
  - Book documents alternative platforms (Unitree G1, custom CAN bus humanoids, future platforms TBD), with architecture-agnostic design (URDF-based, ROS2 standard interfaces). Community maintains compatibility matrix.

- **How does system handle breaking changes in dependencies (Isaac Sim 2026.x, ROS 2 K-Turtle)?**
  - Quarterly dependency audits (per constitution), automated CI/CD tests on new versions, migration guides published. Strict version pinning in all installation scripts with tested upgrade paths.

- **What happens when student's robot hardware malfunctions mid-deployment (motor failure, sensor damage)?**
  - Hardware_Proxy includes diagnostic tools (motor health checks, sensor validation scripts), fallback to simulation for continued learning, and safety protocols to prevent further damage. Troubleshooting chapter covers common hardware failures.

- **How does system handle low-bandwidth or offline scenarios?**
  - PWA with service worker caching enables full offline access after first load. Installation scripts include offline package archives (Apt mirrors, pip wheels). Videos hosted with multiple CDN mirrors and optional local downloads.

- **What if student has macOS/Windows and cannot install Ubuntu 22.04 natively?**
  - Book provides dual-boot guide, VirtualBox/VMware setup (with GPU passthrough caveats), and recommends cloud workstations (Paperspace, AWS WorkSpaces) as alternatives. Primary support remains Ubuntu 22.04 bare metal.

## Requirements *(mandatory)*

### Functional Requirements

#### Content Delivery & Discoverability
- **FR-001**: System MUST deliver all educational content via Docusaurus 3 static site with PWA support (offline mode after first load)
- **FR-002**: System MUST include search functionality across all chapters, code examples, and API references (Algolia DocSearch or lunr.js)
- **FR-003**: System MUST provide hierarchical navigation (13-week curriculum structure with nested chapters)
- **FR-004**: Every chapter MUST include: learning objectives, prerequisites, estimated time, hardware requirements, expected outputs
- **FR-005**: System MUST support code syntax highlighting for Python, C++, YAML, XML, JSON, Bash with copy-to-clipboard buttons
- **FR-006**: System MUST embed video demonstrations (YouTube unlisted or self-hosted) with fallback thumbnails for offline mode

#### Executable Code & Environment Setup
- **FR-007**: System MUST provide tested installation scripts for Ubuntu 22.04 (ROS 2 Jazzy/Iron, Isaac Sim 2024.1+, PyTorch 2.0+, TensorRT 8.5+)
- **FR-008**: All code examples MUST be executable via copy-paste or downloadable as runnable scripts (GitHub repo linked from site)
- **FR-009**: System MUST include Docker containerized environments for complex setups (Isaac Sim, full ROS2 stack)
- **FR-010**: System MUST provide dependency version pinning (requirements.txt, rosdep, apt package lists) with automated validation
- **FR-011**: Installation scripts MUST include verification steps (rosnode list, nvidia-smi, isaac-sim --version checks)

#### Skills & Agents Integration
- **FR-012**: System MUST expose 8 skills (ROS2_Core, URDF_Designer, Gazebo_Sim, Unity_Vis, IsaacSim_Pipeline, VLA_Controller, Edge_Deploy, Hardware_Proxy) as invokable interfaces from Docusaurus
- **FR-013**: System MUST provide one-click skill invocation buttons in relevant chapters (e.g., "Invoke URDF_Designer" in robot design chapter)
- **FR-014**: System MUST integrate with Spec-Kit Plus workflow (skills/agents auto-load context from current chapter)
- **FR-015**: Each skill MUST expose API documentation (api.json) accessible from site's API reference section
- **FR-016**: System MUST provide 3 agent orchestration examples (SimAgent, AIAgent, HumanoidCapstoneAgent) with detailed workflows

#### Testing & Validation
- **FR-017**: System MUST include unit tests for all Python code examples (pytest with > 85% coverage)
- **FR-018**: System MUST provide integration tests for ROS2 nodes (launch_testing framework)
- **FR-019**: System MUST include hardware-in-the-loop test results (documented Jetson + Unitree validation outputs)
- **FR-020**: Every deployment chapter MUST include expected vs. actual performance metrics (latency, accuracy, sim-to-real gap)

#### Visual Assets & Media
- **FR-021**: Every chapter MUST open with cinematic full-bleed hero video (1080p minimum, 4K preferred)
- **FR-022**: System MUST include 4K-resolution diagrams for all architectural concepts (generated via Mermaid.js or static images)
- **FR-023**: System MUST provide hero humanoid character (Figure-01 style) with glowing cyan joints for branding consistency
- **FR-024**: All images MUST be optimized (WebP format, lazy loading, responsive srcset for mobile/desktop)

#### Performance & Accessibility
- **FR-025**: Site MUST achieve Lighthouse score 100/100 for performance, accessibility, best practices, SEO
- **FR-026**: Site MUST load in < 2 seconds on 3G connection (performance budget: < 15MB total)
- **FR-027**: Site MUST support keyboard navigation, screen readers (ARIA labels), and WCAG 2.1 AA compliance
- **FR-028**: Site MUST include dark/light mode toggle with user preference persistence (localStorage)

#### Community & Contribution
- **FR-029**: System MUST provide GitHub-based contribution workflow (fork → PR → review → merge)
- **FR-030**: System MUST include issue templates for bug reports, feature requests, and hardware compatibility
- **FR-031**: System MUST display community metrics (GitHub stars, contributors, open issues) on landing page
- **FR-032**: System MUST provide discussion forum integration (GitHub Discussions or Discourse)

#### Safety & Validation
- **FR-033**: Hardware deployment chapters MUST include safety checklists (e-stop validation, joint limit testing, thermal monitoring)
- **FR-034**: System MUST provide emergency stop implementation examples for all supported hardware platforms
- **FR-035**: All code examples involving hardware MUST include error handling and graceful degradation

### Key Entities *(include if feature involves data)*

- **Chapter**: Educational unit covering specific topic (e.g., "Week 2: URDF Design & Kinematics"). Attributes: title, learning objectives, prerequisites, estimated time, code examples, videos, exercises.

- **Skill**: Modular capability unit (e.g., URDF_Designer, VLA_Controller). Attributes: name, version, capabilities, dependencies, API contract (api.json), usage triggers, integration points.

- **Agent**: Multi-skill orchestrator (e.g., HumanoidCapstoneAgent). Attributes: name, coordinated skills, routing logic (routes.json), example workflows, decision tree.

- **Code Example**: Executable snippet or script. Attributes: language, file path (GitHub repo), expected output, dependencies, test cases, hardware requirements.

- **Hardware Platform**: Physical robot or compute device. Attributes: model (e.g., Unitree H1, Jetson Orin Nano), specifications, price (Dec 2025 baseline), purchase links, compatibility notes.

- **Workflow**: Multi-phase project sequence (e.g., "Design → Simulate → Train → Deploy"). Attributes: phases, phase dependencies, skills used per phase, estimated timeline, success criteria.

- **Installation Script**: Automated setup tool. Attributes: target OS (Ubuntu 22.04), installed packages, verification commands, expected outputs, rollback instructions.

## Additional Success Criteria

### Measurable Outcomes

- **SC-025**: Every chapter includes downloadable code with passing automated tests (pytest, launch_testing)
- **SC-026**: All hardware chapters include video proof of real robot execution (timestamped, unedited)
- **SC-027**: Installation scripts succeed on fresh Ubuntu 22.04 install in < 30 minutes (automated CI/CD validation)
- **SC-028**: Skill invocation from Docusaurus triggers correctly in 95%+ of attempts (telemetry tracking)
- **SC-029**: Community reports zero "broken code" issues for examples tested in CI/CD (issues triaged and fixed within 48 hours)
- **SC-030**: Students rate curriculum as "highly effective" (> 4.5/5.0 average) in post-capstone surveys
