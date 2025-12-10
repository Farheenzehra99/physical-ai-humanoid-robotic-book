<!--
SYNC IMPACT REPORT
==================
Version Change: None (initial creation) → 1.0.0
Modified Principles: N/A (initial version)
Added Sections: All sections newly created
Removed Sections: None
Templates Status:
  ✅ constitution.md - Created
  ⚠ plan-template.md - Review for alignment with execution standards
  ⚠ spec-template.md - Review for alignment with quality principles
  ⚠ tasks-template.md - Review for alignment with testing and deployment requirements
Follow-up TODOs: None - all placeholders filled
-->

# Physical AI & Humanoid Robotics Constitution

## Preamble

This constitution governs the development of the world's first fully executable, spec-driven, open-source educational resource on Embodied Intelligence and Humanoid Robotics. It establishes non-negotiable standards ensuring every student can progress from simulation to real hardware deployment with zero broken code, zero dead links, and zero "works on my machine" friction.

This is not merely a book—it is a complete 13-week university capstone quarter, a living curriculum, and a production-grade deployment pipeline for Physical AI systems.

---

## Core Principles

### I. Execution-First Philosophy

**Every line of code, every simulation, every hardware deployment MUST be 100% executable on Day 1.**

- **MANDATORY**: All code examples tested on Ubuntu 22.04 + ROS 2 Jazzy/Iron + NVIDIA Isaac Sim 2024.1+
- **MANDATORY**: Every terminal command is copy-paste ready with emoji prefix for visual clarity
- **MANDATORY**: Hardware specifications include exact model numbers, current prices (Dec 2025 baseline), and purchase links
- **ZERO TOLERANCE**: No broken code, outdated packages, or platform-specific workarounds
- **VALIDATION**: Student with RTX 4070 Ti+ laptop runs every simulation in < 5 minutes of setup

**Rationale**: Educational materials that don't execute are worse than no materials at all. Students must experience immediate success to maintain momentum through complex robotics concepts.

---

### II. Sim-to-Real Transfer Priority

**Every concept MUST bridge from simulation to real hardware deployment.**

- **MANDATORY**: All training begins in Isaac Sim with domain randomization
- **MANDATORY**: Deployment paths target NVIDIA Jetson (Orin Nano/NX/AGX) + Unitree platforms (Go2/G1/H1)
- **MANDATORY**: Sim-to-real gap documented and quantified for every deployment
- **MANDATORY**: Edge optimization (TensorRT, quantization) validated on actual Jetson hardware
- **TARGET**: $700 Jetson Economy Kit enables capstone deployment to real hardware

**Rationale**: Physical AI is meaningless without physical deployment. Simulation-only education produces engineers unprepared for real-world robotics challenges.

---

### III. Zero-Tolerance Quality Standards

**Content quality MUST meet the highest standards in technical education.**

- **MANDATORY**: Zero broken links—automated CI/CD link checker runs on every commit
- **MANDATORY**: All dependencies pinned with exact versions and tested installation scripts
- **MANDATORY**: Video demonstrations for all simulations (YouTube unlisted or IPFS-hosted)
- **MANDATORY**: 4K-resolution diagrams and hero images (Midjourney or Flux generated)
- **MANDATORY**: IEEE citation style with clickable hyperlinks
- **TARGET**: Zero broken code or links reported in first 6 months post-launch

**Rationale**: Students deserve production-grade materials. Broken links and untested code erode trust and waste irreplaceable learning time.

---

### IV. Reusable Intelligence Architecture

**Skills and agents MUST be modular, invokable, and production-ready.**

- **MANDATORY**: 8 core skills (ROS2_Core, URDF_Designer, Gazebo_Sim, Unity_Vis, IsaacSim_Pipeline, VLA_Controller, Edge_Deploy, Hardware_Proxy)
- **MANDATORY**: 3 orchestration agents (SimAgent, AIAgent, HumanoidCapstoneAgent)
- **MANDATORY**: Every skill includes SKILL.md, usage.md, and api.json documentation
- **MANDATORY**: One-click import/invocation from Docusaurus book interface
- **MANDATORY**: Skills integrate seamlessly with Spec-Kit Plus workflow

**Rationale**: Modern AI development requires orchestration of complex, multi-step workflows. Students must learn to build and deploy agent-based systems, not just write isolated scripts.

---

### V. Visual Excellence & User Experience

**The learning experience MUST be beautiful, modern, and inspiring.**

- **MANDATORY**: Docusaurus theme optimized for dark/light modes, mobile-first design
- **MANDATORY**: Page load time < 2 seconds on 3G connection
- **MANDATORY**: Lighthouse score 100/100 (performance, accessibility, best practices, SEO)
- **MANDATORY**: Every chapter opens with cinematic full-bleed video of humanoid performing task
- **MANDATORY**: Color palette: Deep Space Black (#0A0E27) + Electric Cyan (#00D9FF) + Tesla Bot Silver (#C8D0D9)
- **MANDATORY**: Font stack: Inter (UI) + JetBrains Mono (code)
- **MANDATORY**: Custom Figure-01-style hero humanoid with glowing cyan joints

**Rationale**: Visual design is pedagogy. Beautiful interfaces inspire engagement, reduce cognitive load, and signal professional quality.

---

### VI. Open Source & Accessibility

**Knowledge MUST be freely accessible to all learners worldwide.**

- **MANDATORY**: MIT License for all original content (hardware SDKs follow vendor licenses)
- **MANDATORY**: Progressive Web App (PWA) with offline support after first load
- **MANDATORY**: Total built site size < 15 MB (aggressive image optimization)
- **MANDATORY**: Works on low-bandwidth connections (service worker caching)
- **MANDATORY**: No paywalls, no login requirements for core educational content
- **MANDATORY**: Authentication (better-auth + Neon DB) only for optional features (progress tracking, certifications)

**Rationale**: World-class robotics education should not be gatekept by geography, bandwidth, or economic privilege. Open source accelerates global innovation.

---

### VII. Hardware-in-the-Loop Validation

**Every educational module MUST be validated on real hardware before publication.**

- **MANDATORY**: All RL policies tested in Isaac Sim with 1024+ parallel environments
- **MANDATORY**: All VLA models deployed to Jetson and benchmarked (latency < 50ms target)
- **MANDATORY**: All hardware integration tested on Unitree Go2/G1 or equivalent platforms
- **MANDATORY**: Failure modes documented with troubleshooting guides
- **MANDATORY**: Safety protocols validated for all hardware deployments

**Rationale**: Only hardware-validated content can claim to teach Physical AI. Untested code on real robots is dangerous and misleading.

---

### VIII. Test-Driven Development (NON-NEGOTIABLE)

**All code MUST follow Red-Green-Refactor cycle.**

- **MANDATORY**: Tests written and user-approved BEFORE implementation
- **MANDATORY**: Tests MUST fail initially (Red phase)
- **MANDATORY**: Implementation makes tests pass (Green phase)
- **MANDATORY**: Refactoring preserves passing tests
- **MANDATORY**: Integration tests for ROS2 nodes, sim-to-real pipelines, hardware interfaces
- **TARGET**: > 85% code coverage for all skill implementations

**Rationale**: TDD ensures correctness, maintainability, and confidence in refactoring. Physical AI systems require high reliability—TDD is the foundation.

---

## Technical Standards

### Technology Stack Requirements

**Core Infrastructure:**
- Ubuntu 22.04 LTS (primary development platform)
- ROS 2 Jazzy/Iron (robot middleware)
- NVIDIA Isaac Sim 2024.1+ (physics simulation and RL training)
- Python 3.10+ (primary language)
- Docusaurus 3 + MDX (documentation site)
- Spec-Kit Plus + Claude Code (development workflow)

**AI/ML Stack:**
- PyTorch 2.0+ (deep learning framework)
- TensorRT 8.5+ (edge inference optimization)
- OpenVLA / RT-1 / RT-2 (vision-language-action models)
- Stable-Baselines3 / CleanRL (RL libraries)

**Hardware Platforms:**
- NVIDIA Jetson Orin Nano/NX/AGX (edge compute)
- Unitree Go2 / G1 / H1 (humanoid/quadruped platforms)
- Intel RealSense D455 (depth cameras)
- Custom CAN bus humanoids (educational platforms)

### Code Quality Standards

**All Code MUST:**
- Include type hints (Python) with mypy validation
- Pass linting (black, ruff, isort)
- Include docstrings (NumPy or Google style)
- Handle errors gracefully with informative messages
- Log appropriately (structured logging with context)
- Be security-conscious (no hardcoded secrets, input validation)

### Performance Standards

**Simulations MUST:**
- Run at > 0.9 real-time factor on RTX 4070 Ti
- Support 512+ parallel environments on RTX 4090
- Complete in < 10 minutes for basic examples

**Edge Deployment MUST:**
- Achieve < 50ms inference latency on Jetson Orin
- Use < 8 GB memory on Jetson Orin Nano
- Operate within thermal limits (no throttling under sustained load)

### Documentation Standards

**Every Module MUST Include:**
- Clear learning objectives (what students will build/understand)
- Hardware/software prerequisites with installation guides
- Step-by-step execution instructions
- Expected outputs with screenshots/videos
- Troubleshooting section for common errors
- References to academic papers (IEEE format, clickable links)

---

## Development Workflow

### Content Creation Process

1. **Specification Phase**: Define learning objectives, hardware requirements, expected outcomes
2. **Implementation Phase**: Write code, create simulations, test on hardware
3. **Validation Phase**: Peer review, student beta testing, CI/CD validation
4. **Publication Phase**: Deploy to Docusaurus, run full integration tests
5. **Maintenance Phase**: Monitor issues, update dependencies, refresh hardware prices

### Quality Gates

**Before Merging to Main:**
- [ ] All code passes automated tests (unit + integration)
- [ ] Link checker passes (zero broken links)
- [ ] Lighthouse score 100/100
- [ ] Hardware validation complete (if applicable)
- [ ] Peer review approved
- [ ] Documentation complete

### Continuous Integration

**Automated Checks:**
- Broken link detection (weekly + on PR)
- Dependency vulnerability scanning (Dependabot)
- Build size monitoring (< 15 MB constraint)
- Lighthouse performance audits
- Hardware simulation tests (Isaac Sim headless)

---

## Success Metrics

### Educational Impact

- **TARGET**: 10+ real humanoid robots worldwide running book code within 12 months
- **TARGET**: Book becomes de-facto standard for Physical AI education globally
- **TARGET**: 1,000+ students complete 13-week capstone within first year
- **TARGET**: 50+ universities adopt as official curriculum

### Technical Validation

- **TARGET**: 100/100 Lighthouse score maintained
- **TARGET**: Zero broken code/links in first 6 months
- **TARGET**: 95%+ student success rate on hardware deployment
- **TARGET**: < 5% sim-to-real performance gap on key benchmarks

### Community Growth

- **TARGET**: 10,000+ GitHub stars within first year
- **TARGET**: 500+ community contributions (PRs, issues, discussions)
- **TARGET**: Active Discord/Slack community with daily engagement
- **TARGET**: Monthly webinars with industry practitioners

---

## Governance

### Constitutional Authority

- This constitution supersedes all other development practices and guidelines
- All pull requests MUST verify compliance with constitutional principles
- Contributors MUST read and acknowledge constitutional standards
- Deviations require explicit justification and maintainer approval

### Amendment Process

**Constitutional amendments require:**
1. Proposal with detailed rationale and impact analysis
2. Community discussion period (minimum 2 weeks)
3. Maintainer consensus (2/3 majority)
4. Migration plan for affected content
5. Version bump following semantic versioning

**Version Bump Rules:**
- **MAJOR**: Backward-incompatible principle removals or redefinitions
- **MINOR**: New principles added or materially expanded guidance
- **PATCH**: Clarifications, wording fixes, non-semantic refinements

### Compliance & Review

- **Quarterly Reviews**: Assess adherence to constitutional standards
- **Metrics Tracking**: Monitor success criteria progress
- **Student Feedback**: Incorporate learner experience data
- **Hardware Updates**: Refresh platform recommendations and pricing
- **Dependency Audits**: Update package versions and compatibility

### Enforcement

**Violations of MANDATORY principles result in:**
- Immediate PR rejection
- Required remediation plan before re-submission
- Documentation of lesson learned
- Potential contributor suspension for repeated violations

**Violations of TARGET metrics trigger:**
- Root cause analysis
- Action plan with timeline
- Monthly progress reviews until resolved

---

## References & Resources

### Primary Documentation
- Spec-Kit Plus: https://github.com/panaversity/spec-kit-plus
- ROS 2 Documentation: https://docs.ros.org
- NVIDIA Isaac Sim: https://developer.nvidia.com/isaac-sim
- Unitree Robotics: https://www.unitree.com

### Academic Foundations
- Physical AI research (DeepMind, OpenAI, Google Research)
- Embodied Intelligence (Berkeley, Stanford, CMU)
- Sim-to-Real Transfer (Pieter Abbeel, Sergey Levine et al.)
- Foundation Models for Robotics (RT-1, RT-2, OpenVLA)

---

**Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06
