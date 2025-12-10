# Tasks: Physical AI & Humanoid Robotics - Complete Educational Resource

**Input**: Design documents from `/specs/001-physical-ai-humanoid/`
**Prerequisites**: plan.md, spec.md

**Tests**: Tests are included per TDD requirement in constitution (> 85% coverage mandatory).

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story. This project delivers educational content (Docusaurus site) + reusable intelligence (8 skills + 3 agents), NOT a traditional software application.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4, US5)
- Include exact file paths in descriptions

## Path Conventions

This project uses a **hybrid structure** combining:
- **Docusaurus site**: `docs/`, `static/`, `src/`, `docusaurus.config.js`
- **Reusable intelligence**: `skills/`, `agents/`
- **Code examples**: `static/code-examples/`
- **Tests**: `tests/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create Docusaurus project structure with docs/, static/, src/ directories
- [X] T002 Initialize Node.js project with Docusaurus 3.x dependencies in package.json
- [X] T003 [P] Initialize Python environment with requirements.txt (Python 3.10+, pytest, black, ruff, isort)
- [X] T004 [P] Configure TypeScript for Docusaurus customizations in tsconfig.json
- [X] T005 [P] Create skills/ directory structure with 8 skill subdirectories
- [X] T006 [P] Create agents/ directory structure with 3 agent subdirectories
- [X] T007 [P] Create static/code-examples/ directory with week-01 through week-13 subdirs
- [X] T008 [P] Configure linting tools (black, ruff, isort) in pyproject.toml
- [X] T009 [P] Configure Jest for TypeScript testing in jest.config.js
- [X] T010 Create .gitignore for Node.js, Python, Docusaurus build artifacts
- [X] T011 [P] Initialize Git LFS for large video files in .gitattributes
- [X] T012 Create LICENSE file (MIT License per constitution)
- [X] T013 Create README.md with project overview and setup instructions

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Phase 1 Artifacts (from /sp.plan workflow)

- [X] T014 Create specs/001-physical-ai-humanoid/research.md (technology decisions and best practices)
- [X] T015 Create specs/001-physical-ai-humanoid/data-model.md (entities: Chapter, Skill, Agent, Workflow, HardwarePlatform, CodeExample)
- [X] T016 Create specs/001-physical-ai-humanoid/quickstart.md (installation validation checklist)

### Skill API Contracts

- [X] T017 [P] Create specs/001-physical-ai-humanoid/contracts/skills/ros2_core-api.json
- [X] T018 [P] Create specs/001-physical-ai-humanoid/contracts/skills/urdf_designer-api.json
- [X] T019 [P] Create specs/001-physical-ai-humanoid/contracts/skills/gazebo_sim-api.json
- [X] T020 [P] Create specs/001-physical-ai-humanoid/contracts/skills/unity_vis-api.json
- [X] T021 [P] Create specs/001-physical-ai-humanoid/contracts/skills/isaac_sim_pipeline-api.json
- [X] T022 [P] Create specs/001-physical-ai-humanoid/contracts/skills/vla_controller-api.json
- [X] T023 [P] Create specs/001-physical-ai-humanoid/contracts/skills/edge_deploy-api.json
- [X] T024 [P] Create specs/001-physical-ai-humanoid/contracts/skills/hardware_proxy-api.json

### Agent Route Definitions

- [X] T025 [P] Create specs/001-physical-ai-humanoid/contracts/agents/sim_agent-routes.json
- [X] T026 [P] Create specs/001-physical-ai-humanoid/contracts/agents/ai_agent-routes.json
- [X] T027 [P] Create specs/001-physical-ai-humanoid/contracts/agents/humanoid_capstone_agent-routes.json

### Skill Scaffolds (8 skills)

- [X] T028 [P] Create skills/ros2_core/SKILL.md with skill description and capabilities
- [X] T029 [P] Create skills/ros2_core/usage.md with invocation examples
- [X] T030 [P] Create skills/ros2_core/api.json linking to contracts/skills/ros2_core-api.json
- [X] T031 [P] Create skills/ros2_core/src/ directory for implementation
- [X] T032 [P] Create skills/ros2_core/tests/ directory with __init__.py

- [X] T033 [P] Create skills/urdf_designer/SKILL.md with skill description and capabilities
- [X] T034 [P] Create skills/urdf_designer/usage.md with invocation examples
- [X] T035 [P] Create skills/urdf_designer/api.json linking to contracts/skills/urdf_designer-api.json
- [X] T036 [P] Create skills/urdf_designer/src/ directory for implementation
- [X] T037 [P] Create skills/urdf_designer/tests/ directory with __init__.py

- [X] T038 [P] Create skills/gazebo_sim/SKILL.md with skill description and capabilities
- [X] T039 [P] Create skills/gazebo_sim/usage.md with invocation examples
- [X] T040 [P] Create skills/gazebo_sim/api.json linking to contracts/skills/gazebo_sim-api.json
- [X] T041 [P] Create skills/gazebo_sim/src/ directory for implementation
- [X] T042 [P] Create skills/gazebo_sim/tests/ directory with __init__.py

- [X] T043 [P] Create skills/unity_vis/SKILL.md with skill description and capabilities
- [X] T044 [P] Create skills/unity_vis/usage.md with invocation examples
- [X] T045 [P] Create skills/unity_vis/api.json linking to contracts/skills/unity_vis-api.json
- [X] T046 [P] Create skills/unity_vis/src/ directory for implementation
- [X] T047 [P] Create skills/unity_vis/tests/ directory with __init__.py

- [X] T048 [P] Create skills/isaac_sim_pipeline/SKILL.md with skill description and capabilities
- [X] T049 [P] Create skills/isaac_sim_pipeline/usage.md with invocation examples
- [X] T050 [P] Create skills/isaac_sim_pipeline/api.json linking to contracts/skills/isaac_sim_pipeline-api.json
- [X] T051 [P] Create skills/isaac_sim_pipeline/src/ directory for implementation
- [X] T052 [P] Create skills/isaac_sim_pipeline/tests/ directory with __init__.py

- [X] T053 [P] Create skills/vla_controller/SKILL.md with skill description and capabilities
- [X] T054 [P] Create skills/vla_controller/usage.md with invocation examples
- [X] T055 [P] Create skills/vla_controller/api.json linking to contracts/skills/vla_controller-api.json
- [X] T056 [P] Create skills/vla_controller/src/ directory for implementation
- [X] T057 [P] Create skills/vla_controller/tests/ directory with __init__.py

- [X] T058 [P] Create skills/edge_deploy/SKILL.md with skill description and capabilities
- [X] T059 [P] Create skills/edge_deploy/usage.md with invocation examples
- [X] T060 [P] Create skills/edge_deploy/api.json linking to contracts/skills/edge_deploy-api.json
- [X] T061 [P] Create skills/edge_deploy/src/ directory for implementation
- [X] T062 [P] Create skills/edge_deploy/tests/ directory with __init__.py

- [X] T063 [P] Create skills/hardware_proxy/SKILL.md with skill description and capabilities
- [X] T064 [P] Create skills/hardware_proxy/usage.md with invocation examples
- [X] T065 [P] Create skills/hardware_proxy/api.json linking to contracts/skills/hardware_proxy-api.json
- [X] T066 [P] Create skills/hardware_proxy/src/ directory for implementation
- [X] T067 [P] Create skills/hardware_proxy/tests/ directory with __init__.py

### Agent Scaffolds (3 agents)

- [X] T068 [P] Create agents/sim_agent/AGENT.md with agent description and workflow
- [X] T069 [P] Create agents/sim_agent/routes.json linking to contracts/agents/sim_agent-routes.json
- [X] T070 [P] Create agents/sim_agent/workflows/ directory for multi-skill orchestration
- [X] T071 [P] Create agents/sim_agent/tests/ directory with __init__.py

- [X] T072 [P] Create agents/ai_agent/AGENT.md with agent description and workflow
- [X] T073 [P] Create agents/ai_agent/routes.json linking to contracts/agents/ai_agent-routes.json
- [X] T074 [P] Create agents/ai_agent/workflows/ directory for multi-skill orchestration
- [X] T075 [P] Create agents/ai_agent/tests/ directory with __init__.py

- [X] T076 [P] Create agents/humanoid_capstone_agent/AGENT.md with agent description and workflow
- [X] T077 [P] Create agents/humanoid_capstone_agent/routes.json linking to contracts/agents/humanoid_capstone_agent-routes.json
- [X] T078 [P] Create agents/humanoid_capstone_agent/workflows/ directory for multi-skill orchestration
- [X] T079 [P] Create agents/humanoid_capstone_agent/tests/ directory with __init__.py

### Docusaurus Configuration

- [X] T080 Configure docusaurus.config.js with project metadata, theme, plugins (PWA, sitemap)
- [X] T081 [P] Create src/css/custom.css with color palette (Deep Space Black #0A0E27, Electric Cyan #00D9FF, Tesla Bot Silver #C8D0D9)
- [X] T082 [P] Configure font stack in custom.css (Inter for UI, JetBrains Mono for code)
- [X] T083 [P] Create src/components/SkillInvoker.tsx for one-click skill invocation
- [X] T084 [P] Create src/components/AgentOrchestrator.tsx for agent workflow triggers
- [X] T085 Create src/theme/ directory for Docusaurus theme overrides (dark/light mode)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Complete Humanoid Capstone (13-Week Project) (Priority: P1) 🎯 MVP

**Goal**: Deliver complete 13-week curriculum where students build 18-DOF humanoid from scratch, train AI walking policy in Isaac Sim, optimize for Jetson Orin Nano, and deploy to real custom hardware.

**Independent Test**: Student with zero prior humanoid experience completes 13-week curriculum and successfully demonstrates walking robot at capstone presentation.

### Tests for User Story 1

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [X] T086 [P] [US1] Create tests/unit/test_ros2_core_skill.py (test skill API contract validation)
- [X] T087 [P] [US1] Create tests/unit/test_urdf_designer_skill.py (test URDF generation and validation)
- [X] T088 [P] [US1] Create tests/unit/test_gazebo_sim_skill.py (test simulation launch and control)
- [X] T089 [P] [US1] Create tests/unit/test_isaac_sim_pipeline_skill.py (test parallel env setup and RL training)
- [X] T090 [P] [US1] Create tests/unit/test_edge_deploy_skill.py (test ONNX export and TensorRT conversion)
- [X] T091 [P] [US1] Create tests/unit/test_hardware_proxy_skill.py (test CAN bus communication and safety systems)
- [X] T092 [P] [US1] Create tests/integration/test_humanoid_capstone_agent.py (test full workflow orchestration)
- [X] T093 [P] [US1] Create tests/e2e/test_week01_to_week13_pipeline.py (test complete 13-week workflow)

### Implementation for User Story 1 - Week 1-3 Content (ROS2, URDF, Gazebo)

- [X] T094 [P] [US1] Create docs/week-01-foundations/01-ubuntu-setup.mdx
- [X] T095 [P] [US1] Create docs/week-01-foundations/02-ros2-installation.mdx
- [X] T096 [P] [US1] Create docs/week-01-foundations/03-ros2-basics.mdx
- [X] T097 [P] [US1] Create docs/week-01-foundations/04-workspace-setup.mdx
- [X] T098 [P] [US1] Create docs/week-01-foundations/05-first-node.mdx

- [X] T099 [P] [US1] Create static/code-examples/week-01/install_ros2.sh (ROS 2 Jazzy/Iron installation script)
- [X] T100 [P] [US1] Create static/code-examples/week-01/verify_setup.sh (installation validation)
- [X] T101 [P] [US1] Create static/code-examples/week-01/hello_ros2/ ROS2 package with tests

- [X] T102 [P] [US1] Create docs/week-02-urdf/01-urdf-intro.mdx
- [X] T103 [P] [US1] Create docs/week-02-urdf/02-humanoid-design.mdx
- [X] T104 [P] [US1] Create docs/week-02-urdf/03-kinematics.mdx
- [X] T105 [P] [US1] Create docs/week-02-urdf/04-rviz-visualization.mdx

- [X] T106 [P] [US1] Create static/code-examples/week-02/humanoid_description/urdf/humanoid.urdf.xacro (18-DOF model)
- [X] T107 [P] [US1] Create static/code-examples/week-02/humanoid_description/launch/display.launch.py
- [X] T108 [P] [US1] Create static/code-examples/week-02/kinematics/fk_ik_solver.py with pytest tests

- [X] T109 [P] [US1] Create docs/week-03-gazebo/01-gazebo-intro.mdx
- [X] T110 [P] [US1] Create docs/week-03-gazebo/02-physics-simulation.mdx
- [X] T111 [P] [US1] Create docs/week-03-gazebo/03-ros2-control.mdx
- [X] T112 [P] [US1] Create docs/week-03-gazebo/04-joint-controllers.mdx
- [X] T113 [P] [US1] Create docs/week-03-gazebo/05-teleoperation.mdx
- [X] T114 [P] [US1] Create docs/week-03-gazebo/06-walking-basics.mdx

- [X] T115 [P] [US1] Create static/code-examples/week-03/humanoid_gazebo/worlds/flat_ground.world
- [X] T116 [P] [US1] Create static/code-examples/week-03/humanoid_control/config/controllers.yaml
- [X] T117 [P] [US1] Create static/code-examples/week-03/humanoid_control/src/zmp_walker.cpp with tests

### Implementation for User Story 1 - Week 4-6 Content (Isaac Sim, RL, VLA)

- [X] T118 [P] [US1] Create docs/week-04-isaac-setup/01-isaac-sim-installation.mdx
- [X] T119 [P] [US1] Create docs/week-04-isaac-setup/02-gpu-acceleration.mdx
- [X] T120 [P] [US1] Create docs/week-04-isaac-setup/03-parallel-environments.mdx
- [X] T121 [P] [US1] Create docs/week-04-isaac-setup/04-domain-randomization.mdx
- [X] T122 [P] [US1] Create docs/week-04-isaac-setup/05-first-rl-task.mdx

- [X] T123 [P] [US1] Create static/code-examples/week-04/isaac_setup/verify_gpu.py (RTX 4070 Ti validation)
- [X] T124 [P] [US1] Create static/code-examples/week-04/isaac_envs/cartpole_parallel.py (1024 parallel envs)
- [X] T125 [P] [US1] Create static/code-examples/week-04/isaac_envs/randomization_config.yaml

- [X] T126 [P] [US1] Create docs/week-05-rl-training/01-ppo-fundamentals.mdx
- [X] T127 [P] [US1] Create docs/week-05-rl-training/02-reward-design.mdx
- [X] T128 [P] [US1] Create docs/week-05-rl-training/03-training-loop.mdx
- [X] T129 [P] [US1] Create docs/week-05-rl-training/04-policy-evaluation.mdx

- [X] T130 [P] [US1] Create static/code-examples/week-05/rl_training/ppo_trainer.py (Stable-Baselines3)
- [X] T131 [P] [US1] Create static/code-examples/week-05/rl_training/rewards/walking_reward.py
- [X] T132 [P] [US1] Create static/code-examples/week-05/rl_training/train_walking.py with checkpoints
- [X] T133 [P] [US1] Create static/code-examples/week-05/rl_training/eval_policy.py with visualization

- [X] T134 [P] [US1] Create docs/week-06-vla/01-openvla-setup.mdx
- [X] T135 [P] [US1] Create docs/week-06-vla/02-demonstration-collection.mdx
- [X] T136 [P] [US1] Create docs/week-06-vla/03-fine-tuning.mdx
- [X] T137 [P] [US1] Create docs/week-06-vla/04-language-conditioning.mdx
- [X] T138 [P] [US1] Create docs/week-06-vla/05-validation.mdx

- [X] T139 [P] [US1] Create static/code-examples/week-06/vla/collect_demos.py (RealSense integration)
- [X] T140 [P] [US1] Create static/code-examples/week-06/vla/finetune_openvla.py (LoRA config)
- [X] T141 [P] [US1] Create static/code-examples/week-06/vla/vla_inference.py with language parsing
- [X] T142 [P] [US1] Create static/code-examples/week-06/vla/tests/test_inference.py

### Implementation for User Story 1 - Week 7-9 Content (Unity, Edge Deploy, Jetson)

- [X] T143 [P] [US1] Create docs/week-07-unity/01-unity-ros-bridge.mdx
- [X] T144 [P] [US1] Create docs/week-07-unity/02-environment-design.mdx
- [X] T145 [P] [US1] Create docs/week-07-unity/03-vr-integration.mdx
- [X] T146 [P] [US1] Create docs/week-07-unity/04-cinematic-rendering.mdx

- [X] T147 [P] [US1] Create static/code-examples/week-07/unity_vis/HumanoidHRI/ Unity project (2023.2+)
- [X] T148 [P] [US1] Create static/code-examples/week-07/unity_vis/ros_tcp_endpoint.py
- [X] T149 [P] [US1] Create static/code-examples/week-07/unity_vis/Scripts/VRInteraction.cs

- [X] T150 [P] [US1] Create docs/week-08-edge-optimization/01-tensorrt-intro.mdx
- [X] T151 [P] [US1] Create docs/week-08-edge-optimization/02-onnx-export.mdx
- [X] T152 [P] [US1] Create docs/week-08-edge-optimization/03-quantization.mdx
- [X] T153 [P] [US1] Create docs/week-08-edge-optimization/04-fp16-int8.mdx
- [X] T154 [P] [US1] Create docs/week-08-edge-optimization/05-benchmarking.mdx
- [X] T155 [P] [US1] Create docs/week-08-edge-optimization/06-accuracy-validation.mdx

- [X] T156 [P] [US1] Create static/code-examples/week-08/edge_deploy/export_onnx.py
- [X] T157 [P] [US1] Create static/code-examples/week-08/edge_deploy/convert_tensorrt.py
- [X] T158 [P] [US1] Create static/code-examples/week-08/edge_deploy/benchmark_latency.py
- [X] T159 [P] [US1] Create static/code-examples/week-08/edge_deploy/validate_accuracy.py

- [X] T160 [P] [US1] Create docs/week-09-jetson/01-jetson-setup.mdx
- [X] T161 [P] [US1] Create docs/week-09-jetson/02-cross-compilation.mdx
- [X] T162 [P] [US1] Create docs/week-09-jetson/03-deployment-pipeline.mdx
- [X] T163 [P] [US1] Create docs/week-09-jetson/04-inference-optimization.mdx
- [X] T164 [P] [US1] Create docs/week-09-jetson/05-thermal-management.mdx

- [X] T165 [P] [US1] Create static/code-examples/week-09/jetson_deploy/setup_jetson.sh
- [X] T166 [P] [US1] Create static/code-examples/week-09/jetson_deploy/cross_compile.sh
- [X] T167 [P] [US1] Create static/code-examples/week-09/jetson_deploy/run_inference.py (Jetson-optimized)
- [X] T168 [P] [US1] Create static/code-examples/week-09/jetson_deploy/thermal_monitor.py

### Implementation for User Story 1 - Week 10-13 Content (Hardware, Sim-to-Real, Safety, Capstone)

- [X] T169 [P] [US1] Create docs/week-10-hardware/01-unitree-sdk.mdx
- [X] T170 [P] [US1] Create docs/week-10-hardware/02-can-bus-basics.mdx
- [X] T171 [P] [US1] Create docs/week-10-hardware/03-motor-control.mdx
- [X] T172 [P] [US1] Create docs/week-10-hardware/04-sensor-integration.mdx
- [X] T173 [P] [US1] Create docs/week-10-hardware/05-safety-systems.mdx
- [X] T174 [P] [US1] Create docs/week-10-hardware/06-first-connection.mdx

- [X] T175 [P] [US1] Create static/code-examples/week-10/hardware_proxy/unitree_interface.py
- [X] T176 [P] [US1] Create static/code-examples/week-10/hardware_proxy/can_driver.py
- [X] T177 [P] [US1] Create static/code-examples/week-10/hardware_proxy/motor_controller.py
- [X] T178 [P] [US1] Create static/code-examples/week-10/hardware_proxy/safety_monitor.py
- [X] T179 [P] [US1] Create static/code-examples/week-10/hardware_proxy/tests/test_connection.py

- [X] T180 [P] [US1] Create docs/week-11-sim-to-real/01-gap-analysis.mdx
- [X] T181 [P] [US1] Create docs/week-11-sim-to-real/02-calibration.mdx
- [X] T182 [P] [US1] Create docs/week-11-sim-to-real/03-action-smoothing.mdx
- [X] T183 [P] [US1] Create docs/week-11-sim-to-real/04-validation-metrics.mdx
- [X] T184 [P] [US1] Create docs/week-11-sim-to-real/05-first-deployment.mdx

- [X] T185 [P] [US1] Create static/code-examples/week-11/sim_to_real/analyze_gap.py
- [X] T186 [P] [US1] Create static/code-examples/week-11/sim_to_real/calibrate_robot.py
- [X] T187 [P] [US1] Create static/code-examples/week-11/sim_to_real/action_filter.py
- [X] T188 [P] [US1] Create static/code-examples/week-11/sim_to_real/measure_performance.py
- [X] T189 [P] [US1] Create static/code-examples/week-11/sim_to_real/deploy_policy.py

- [X] T190 [P] [US1] Create docs/week-12-safety/01-emergency-stop.mdx
- [X] T191 [P] [US1] Create docs/week-12-safety/02-joint-limits.mdx
- [X] T192 [P] [US1] Create docs/week-12-safety/03-fault-detection.mdx
- [X] T193 [P] [US1] Create docs/week-12-safety/04-debugging-tools.mdx

- [X] T194 [P] [US1] Create static/code-examples/week-12/safety/emergency_stop.py
- [X] T195 [P] [US1] Create static/code-examples/week-12/safety/joint_limit_enforcer.py
- [X] T196 [P] [US1] Create static/code-examples/week-12/safety/fault_detector.py
- [X] T197 [P] [US1] Create static/code-examples/week-12/safety/diagnostic_logger.py

- [X] T198 [P] [US1] Create docs/week-13-capstone/01-integration-testing.mdx
- [X] T199 [P] [US1] Create docs/week-13-capstone/02-documentation.mdx
- [X] T200 [P] [US1] Create docs/week-13-capstone/03-final-demo.mdx

- [X] T201 [P] [US1] Create static/code-examples/week-13/capstone/integration_tests.py
- [X] T202 [P] [US1] Create static/code-examples/week-13/capstone/project_report_template.md
- [X] T203 [P] [US1] Create static/code-examples/week-13/capstone/run_demo.sh

### Skill Implementations for User Story 1

- [X] T204 [US1] Implement skills/ros2_core/src/ros2_setup.py (ROS2 installation and validation)
- [X] T205 [US1] Implement skills/urdf_designer/src/urdf_generator.py (18-DOF humanoid URDF generation)
- [X] T206 [US1] Implement skills/gazebo_sim/src/gazebo_launcher.py (simulation launch and control)
- [X] T207 [US1] Implement skills/isaac_sim_pipeline/src/parallel_env_manager.py (1024+ parallel envs)
- [X] T208 [US1] Implement skills/isaac_sim_pipeline/src/rl_trainer.py (PPO training with Stable-Baselines3)
- [X] T209 [US1] Implement skills/edge_deploy/src/tensorrt_optimizer.py (ONNX export and TensorRT conversion)
- [X] T210 [US1] Implement skills/hardware_proxy/src/can_interface.py (CAN bus communication)
- [X] T211 [US1] Implement skills/hardware_proxy/src/safety_controller.py (e-stop, joint limits, thermal monitoring)
- [X] T212 [US1] Implement agents/humanoid_capstone_agent/workflows/week01_to_week13.py (full capstone orchestration)

**Checkpoint**: At this point, User Story 1 should be fully functional - student can complete 13-week curriculum and deploy walking robot

---

## Phase 4: User Story 2 - Service Robot VLA Integration (Existing Robot + New AI) (Priority: P2)

**Goal**: Enable developer with existing Unitree H1 humanoid to add Vision-Language-Action control for language-conditioned task execution by fine-tuning OpenVLA and deploying to onboard Jetson.

**Independent Test**: Developer with Unitree H1 (or Go2) follows VLA integration chapters and demonstrates language-based task execution on real hardware.

### Tests for User Story 2

- [X] T213 [P] [US2] Create tests/unit/test_vla_controller_skill.py (test VLA API contract)
- [X] T214 [P] [US2] Create tests/integration/test_vla_unitree_integration.py (test VLA on Unitree platform)

### Implementation for User Story 2

- [X] T215 [P] [US2] Create docs/vla-integration/01-vla-setup-existing-robot.mdx (Unitree H1 VLA setup)
- [X] T216 [P] [US2] Create docs/vla-integration/02-demonstration-collection-unitree.mdx (H1-specific demo collection)
- [X] T217 [P] [US2] Create docs/vla-integration/03-openvla-finetuning-h1.mdx (LoRA fine-tuning for H1 tasks)
- [X] T218 [P] [US2] Create docs/vla-integration/04-language-task-execution.mdx (language command execution)
- [X] T219 [P] [US2] Create docs/vla-integration/05-deployment-validation.mdx (hardware validation on H1)

- [X] T220 [P] [US2] Create static/code-examples/vla-integration/collect_h1_demos.py (H1-specific demo collector)
- [X] T221 [P] [US2] Create static/code-examples/vla-integration/finetune_h1_vla.py (H1 task fine-tuning)
- [X] T222 [P] [US2] Create static/code-examples/vla-integration/vla_language_executor.py (language parsing and execution)
- [X] T223 [P] [US2] Create static/code-examples/vla-integration/tests/test_h1_vla.py

- [X] T224 [US2] Implement skills/vla_controller/src/openvla_finetuner.py (LoRA fine-tuning pipeline)
- [X] T225 [US2] Implement skills/vla_controller/src/language_parser.py (language command parsing)
- [X] T226 [US2] Implement skills/vla_controller/src/vla_inference_engine.py (TensorRT-optimized VLA inference)
- [X] T227 [US2] Implement agents/ai_agent/workflows/vla_integration_workflow.py (VLA integration orchestration)

**Checkpoint**: At this point, User Story 2 should work independently - developer can add VLA to existing Unitree robot

---

## Phase 5: User Story 3 - Rapid Sim-to-Real Transfer (Pretrained Policy Deployment) (Priority: P3)

**Goal**: Enable researcher with pretrained walking policy to deploy to custom quadruped hardware within 1 week for conference demo using Edge_Deploy + Hardware_Proxy skills.

**Independent Test**: Researcher with pretrained PyTorch policy follows rapid deployment chapters and demonstrates working policy on real quadruped within 5 business days.

### Tests for User Story 3

- [X] T228 [P] [US3] Create tests/unit/test_rapid_deployment_pipeline.py (test fast-track deployment workflow)
- [X] T229 [P] [US3] Create tests/integration/test_pretrained_policy_deployment.py (test policy deployment to hardware)

### Implementation for User Story 3

- [X] T230 [P] [US3] Create docs/rapid-deployment/01-pretrained-policy-validation.mdx (Gazebo validation)
- [X] T231 [P] [US3] Create docs/rapid-deployment/02-rapid-edge-optimization.mdx (fast TensorRT conversion)
- [X] T232 [P] [US3] Create docs/rapid-deployment/03-hardware-quick-setup.mdx (CAN bus setup in 1 day)
- [X] T233 [P] [US3] Create docs/rapid-deployment/04-policy-deployment.mdx (deploy and tune in 2 days)
- [X] T234 [P] [US3] Create docs/rapid-deployment/05-conference-demo-prep.mdx (final validation)

- [X] T235 [P] [US3] Create static/code-examples/rapid-deployment/validate_pretrained.py (Gazebo validation script)
- [X] T236 [P] [US3] Create static/code-examples/rapid-deployment/rapid_tensorrt_convert.sh (one-command optimization)
- [X] T237 [P] [US3] Create static/code-examples/rapid-deployment/hardware_quick_connect.py (fast CAN setup)
- [X] T238 [P] [US3] Create static/code-examples/rapid-deployment/deploy_and_tune.py (automated deployment)
- [X] T239 [P] [US3] Create static/code-examples/rapid-deployment/tests/test_rapid_pipeline.py

- [X] T240 [US3] Implement skills/edge_deploy/src/rapid_optimizer.py (one-step ONNX to TensorRT FP16)
- [X] T241 [US3] Implement skills/hardware_proxy/src/quick_setup.py (automated hardware connection)
- [X] T242 [US3] Implement agents/sim_agent/workflows/rapid_deployment_workflow.py (5-day deployment orchestration)

**Checkpoint**: At this point, User Story 3 should work independently - researcher can deploy pretrained policy in 1 week

---

## Phase 6: User Story 4 - Multi-Robot Coordination System (Advanced Project) (Priority: P4)

**Goal**: Enable research lab to build multi-robot coordination system where two humanoid robots collaboratively move furniture with synchronized grasping, force balance, and coordinated motion.

**Independent Test**: Lab with two humanoid robots follows multi-robot coordination chapters and demonstrates collaborative furniture moving.

### Tests for User Story 4

- [X] T243 [P] [US4] Create tests/unit/test_multi_robot_namespace.py (test ROS2 namespacing)
- [X] T244 [P] [US4] Create tests/integration/test_dual_robot_coordination.py (test synchronized control)

### Implementation for User Story 4

- [X] T245 [P] [US4] Create docs/multi-robot/01-dual-robot-gazebo-setup.mdx (spawn dual robots with namespaces)
- [X] T246 [P] [US4] Create docs/multi-robot/02-coordination-policy-training.mdx (train coordinated policy in Isaac Sim)
- [X] T247 [P] [US4] Create docs/multi-robot/03-synchronized-deployment.mdx (deploy to dual Jetson systems)
- [X] T248 [P] [US4] Create docs/multi-robot/04-collaborative-tasks.mdx (furniture moving validation)

- [X] T249 [P] [US4] Create static/code-examples/multi-robot/dual_gazebo_spawn.launch.py (dual robot spawner)
- [X] T250 [P] [US4] Create static/code-examples/multi-robot/coordination_trainer.py (multi-agent RL training)
- [X] T251 [P] [US4] Create static/code-examples/multi-robot/synchronized_deployer.py (deploy to dual Jetsons)
- [X] T252 [P] [US4] Create static/code-examples/multi-robot/tests/test_coordination.py

- [X] T253 [US4] Implement skills/isaac_sim_pipeline/src/multi_robot_env.py (2048 robot instances: 1024 envs × 2)
- [X] T254 [US4] Implement skills/hardware_proxy/src/dual_robot_controller.py (synchronized control)
- [X] T255 [US4] Implement agents/humanoid_capstone_agent/workflows/multi_robot_workflow.py (coordination orchestration)

**Checkpoint**: At this point, User Story 4 should work independently - lab can coordinate two robots for collaborative tasks

---

## Phase 7: User Story 5 - Visualization & HRI Environment Design (Sim-Only Workflow) (Priority: P5)

**Goal**: Enable game developer to create high-fidelity HRI visualization environments in Unity with ROS2 real-time sync, VR walkthroughs, and cinematic rendering for educational videos.

**Independent Test**: Developer follows Unity visualization chapters and produces 4K cinematic renders of humanoid in living room environment with ROS2 real-time sync.

### Tests for User Story 5

- [X] T256 [P] [US5] Create tests/unit/test_unity_vis_skill.py (test Unity ROS2 bridge API)
- [X] T257 [P] [US5] Create tests/integration/test_unity_ros2_sync.py (test real-time synchronization at 60 FPS)

### Implementation for User Story 5

- [X] T258 [P] [US5] Create docs/unity-visualization/01-unity-ros-tcp-connector.mdx (ROS2 bridge setup)
- [X] T259 [P] [US5] Create docs/unity-visualization/02-hri-environment-design.mdx (living room environment)
- [X] T260 [P] [US5] Create docs/unity-visualization/03-vr-xr-toolkit-integration.mdx (VR setup for Meta Quest 3)
- [X] T261 [P] [US5] Create docs/unity-visualization/04-cinematic-timeline-rendering.mdx (4K video production)

- [X] T262 [P] [US5] Create static/code-examples/unity-visualization/UnityHRIProject/ (Unity 2023.2+ project)
- [X] T263 [P] [US5] Create static/code-examples/unity-visualization/ros_tcp_connector.py (ROS2 bridge endpoint)
- [X] T264 [P] [US5] Create static/code-examples/unity-visualization/Scripts/VRWalkthrough.cs (VR interaction)
- [X] T265 [P] [US5] Create static/code-examples/unity-visualization/Scripts/CinematicRecorder.cs (Timeline + Cinemachine)

- [X] T266 [US5] Implement skills/unity_vis/src/ros2_bridge.py (Unity ROS-TCP-Connector integration)
- [X] T267 [US5] Implement skills/unity_vis/src/urdf_importer.py (URDF to Unity conversion)
- [X] T268 [US5] Implement skills/unity_vis/src/vr_controller.py (XR Interaction Toolkit integration)

**Checkpoint**: At this point, all 5 user stories should work independently - each delivers value on its own

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

### CI/CD Pipeline

- [X] T269 [P] Create .github/workflows/link-checker.yml (weekly + on PR, blocks merge on broken links)
- [X] T270 [P] Create .github/workflows/lighthouse-ci.yml (100/100 target, -5 regression threshold)
- [X] T271 [P] Create .github/workflows/build-size.yml (< 15 MB limit, alert at 12 MB)
- [X] T272 [P] Create .github/workflows/tests.yml (pytest + Jest on every PR)
- [X] T273 [P] Create .github/workflows/dependency-scan.yml (Dependabot vulnerability checks)

### Documentation and Assets

- [X] T274 [P] Create docs/appendix/cloud-gpu-setup.mdx (Lambda Labs, Paperspace, Colab Pro+ guides)
- [X] T275 [P] Create docs/appendix/low-tier-gpu-guide.mdx (RTX 3070/3080 configuration with reduced env counts)
- [X] T276 [P] Create docs/appendix/pretrained-policies.mdx (download and validate pretrained models)
- [X] T277 [P] Create docs/hardware-validation/ directory with Jetson + Unitree test reports
- [X] T278 [P] Create docs/benchmarks/ directory with sim-to-real performance data
- [X] T279 [P] Create docs/safety/ directory with safety protocol checklists

### Video Production (Phase 3 milestone)

- [X] T280 [P] Record static/videos/week-01-opener.mp4 (30s, 4K, environment setup)
- [X] T281 [P] Record static/videos/week-02-opener.mp4 (30s, 4K, URDF visualization in RViz)
- [X] T282 [P] Record static/videos/week-03-opener.mp4 (30s, 4K, humanoid walking in Gazebo)
- [X] T283 [P] Record static/videos/week-04-opener.mp4 (30s, 4K, 1024 parallel envs)
- [X] T284 [P] Record static/videos/week-05-opener.mp4 (30s, 4K, training progress timelapse)
- [X] T285 [P] Record static/videos/week-06-opener.mp4 (30s, 4K, language-conditioned task)
- [X] T286 [P] Record static/videos/week-07-opener.mp4 (30s, 4K, 4K cinematic render)
- [X] T287 [P] Record static/videos/week-08-opener.mp4 (30s, 4K, TensorRT INT8 conversion demo)
- [X] T288 [P] Record static/videos/week-09-opener.mp4 (30s, 4K, Jetson inference benchmark)
- [X] T289 [P] Record static/videos/week-10-opener.mp4 (30s, 4K, CAN bus communication demo)
- [X] T290 [P] Record static/videos/week-11-opener.mp4 (30s, 4K, first hardware walking video)
- [X] T291 [P] Record static/videos/week-12-opener.mp4 (30s, 4K, safety protocol demo)
- [X] T292 [P] Record static/videos/week-13-opener.mp4 (30s, 4K, capstone preview)
- [X] T293 Record static/videos/capstone-demo.mp4 (10 min, 4K, full 13-week project showcase)

### Performance Optimization

- [X] T294 [P] Optimize all images to WebP format in static/img/ with lazy loading
- [X] T295 [P] Configure service worker for PWA offline mode in src/sw.js
- [X] T296 [P] Implement code splitting in Docusaurus for faster page loads
- [X] T297 Run Lighthouse CI and fix issues to achieve 100/100 score
- [X] T298 Verify total built site size < 15 MB (run build and check dist/ size)

### Additional Testing

- [X] T299 [P] Add unit tests for all Docusaurus custom components in src/components/__tests__/
- [X] T300 [P] Add E2E tests for skill invocation flows in tests/e2e/test_skill_invocation.py
- [X] T301 Verify > 85% code coverage for all skills (run pytest --cov)

### Final Validations

- [X] T302 Run specs/001-physical-ai-humanoid/quickstart.md validation (all installation scripts succeed)
- [X] T303 Verify zero broken links across all 65 MDX chapters (run link checker)
- [X] T304 Verify all code examples have passing tests (run pytest + launch_testing)
- [X] T305 Create CONTRIBUTING.md with contribution guidelines
- [X] T306 Update README.md with final setup instructions and quick start guide

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-7)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if team capacity allows)
  - Or sequentially in priority order (P1 → P2 → P3 → P4 → P5)
- **Polish (Phase 8)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories (complete 13-week curriculum)
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Reuses VLA_Controller skill from US1 but independently testable (VLA integration for existing robots)
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Reuses Edge_Deploy + Hardware_Proxy from US1 but independently testable (rapid deployment workflow)
- **User Story 4 (P4)**: Can start after Foundational (Phase 2) - Extends IsaacSim_Pipeline from US1 but independently testable (multi-robot coordination)
- **User Story 5 (P5)**: Can start after Foundational (Phase 2) - Uses Unity_Vis skill independently (visualization without hardware deployment)

### Within Each User Story

- Tests MUST be written and FAIL before implementation (TDD mandatory)
- MDX content creation tasks can run in parallel (marked [P])
- Code example creation tasks can run in parallel (marked [P])
- Skill implementations should follow after content creation (dependencies on understanding requirements)
- Agent orchestration is final step per story (depends on all skills working)

### Parallel Opportunities

- All Setup tasks (T001-T013) can run in parallel
- All API contract creation tasks (T017-T027) can run in parallel within Foundational phase
- All skill scaffold tasks (T028-T079) can run in parallel within Foundational phase
- All Docusaurus configuration tasks (T080-T085) can run in parallel
- Once Foundational phase completes, all 5 user stories can start in parallel (if team capacity allows)
- Within each user story, all MDX creation tasks marked [P] can run in parallel
- Within each user story, all code example creation tasks marked [P] can run in parallel
- Video recording tasks (T280-T293) can run in parallel during Phase 3

---

## Parallel Example: User Story 1 (13-Week Capstone)

```bash
# Launch all test creation for User Story 1 together:
Task: "Create tests/unit/test_ros2_core_skill.py"
Task: "Create tests/unit/test_urdf_designer_skill.py"
Task: "Create tests/unit/test_gazebo_sim_skill.py"
Task: "Create tests/unit/test_isaac_sim_pipeline_skill.py"
Task: "Create tests/unit/test_edge_deploy_skill.py"
Task: "Create tests/unit/test_hardware_proxy_skill.py"

# Launch all Week 1 MDX creation together:
Task: "Create docs/week-01-foundations/01-ubuntu-setup.mdx"
Task: "Create docs/week-01-foundations/02-ros2-installation.mdx"
Task: "Create docs/week-01-foundations/03-ros2-basics.mdx"
Task: "Create docs/week-01-foundations/04-workspace-setup.mdx"
Task: "Create docs/week-01-foundations/05-first-node.mdx"

# Launch all Week 1 code examples together:
Task: "Create static/code-examples/week-01/install_ros2.sh"
Task: "Create static/code-examples/week-01/verify_setup.sh"
Task: "Create static/code-examples/week-01/hello_ros2/ ROS2 package"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only - 13-Week Capstone)

1. Complete Phase 1: Setup (T001-T013)
2. Complete Phase 2: Foundational (T014-T085) - CRITICAL, blocks all stories
3. Complete Phase 3: User Story 1 (T086-T212)
4. **STOP and VALIDATE**: Test User Story 1 independently - can student complete 13-week curriculum?
5. Deploy Docusaurus site with Week 1-13 content (MVP launch!)

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready (skills/agents scaffolded)
2. Add User Story 1 (13-Week Capstone) → Test independently → Deploy/Demo (MVP - students can build humanoid!)
3. Add User Story 2 (VLA Integration) → Test independently → Deploy/Demo (developers can upgrade existing robots)
4. Add User Story 3 (Rapid Deployment) → Test independently → Deploy/Demo (researchers can deploy pretrained policies fast)
5. Add User Story 4 (Multi-Robot) → Test independently → Deploy/Demo (labs can coordinate multiple robots)
6. Add User Story 5 (Unity Visualization) → Test independently → Deploy/Demo (developers can create cinematic renders)
7. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (T001-T085)
2. Once Foundational is done:
   - **Developer A**: User Story 1 (13-Week Capstone - highest priority)
   - **Developer B**: User Story 2 (VLA Integration)
   - **Developer C**: User Story 3 (Rapid Deployment)
   - **Developer D**: User Story 4 (Multi-Robot) + User Story 5 (Unity Vis)
3. Stories complete and integrate independently

---

## Summary

**Total Tasks**: 306 tasks
- Setup: 13 tasks
- Foundational: 72 tasks (8 skills × 8 tasks + 3 agents × 4 tasks + 6 config tasks + 6 Phase 1 artifacts)
- User Story 1 (P1 - 13-Week Capstone): 127 tasks (8 tests + 110 content + 9 skill implementations)
- User Story 2 (P2 - VLA Integration): 15 tasks (2 tests + 9 content + 4 implementations)
- User Story 3 (P3 - Rapid Deployment): 15 tasks (2 tests + 9 content + 3 implementations)
- User Story 4 (P4 - Multi-Robot): 13 tasks (2 tests + 8 content + 3 implementations)
- User Story 5 (P5 - Unity Visualization): 13 tasks (2 tests + 8 content + 3 implementations)
- Polish & Cross-Cutting: 38 tasks (5 CI/CD + 6 docs + 14 videos + 5 optimization + 3 testing + 5 validation)

**Parallel Opportunities**:
- 285 tasks marked [P] can run in parallel (different files, no blocking dependencies)
- All 5 user stories can proceed in parallel after Foundational phase
- Within User Story 1: 110+ content creation tasks can run in parallel

**Independent Test Criteria**:
- US1: Student with zero humanoid experience completes 13-week curriculum and demonstrates walking robot
- US2: Developer with Unitree H1 adds VLA control and demonstrates language-based task execution
- US3: Researcher deploys pretrained policy to quadruped within 5 business days
- US4: Lab with two humanoid robots demonstrates collaborative furniture moving
- US5: Game developer produces 4K cinematic renders with ROS2 real-time sync

**Suggested MVP Scope**: User Story 1 only (13-Week Capstone) - delivers complete educational value and validates all 8 skills + 3 agents working together.

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing (TDD mandatory per constitution)
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- This project is unique: educational content + reusable intelligence, not traditional software
- Hardware validation occurs in Phase 3 (real Jetson + Unitree testing)
- Video production occurs in Phase 3 (after content is complete)
- CI/CD setup occurs in Phase 8 (enforces zero broken links + Lighthouse 100/100)
