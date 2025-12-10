---
id: 0007
title: Book Structure Integration Planning
stage: plan
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20251101
feature: 001-physical-ai-humanoid
branch: 001-physical-ai-humanoid
user: farheenzehra
command: /sp.plan
labels: ["book-structure", "planning", "architecture", "documentation"]
links:
  spec: specs/001-physical-ai-humanoid/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-humanoid/plan.md
tests:
  - N/A (planning phase, no code tests)
---

## Prompt

/sp.plan

Title: Physical AI & Humanoid Robotics — A Practical Guide
Author: Syeda Farheen Zehra

Introduction (Medium-Length: 2–3 Pages)
  - What is Physical AI?
  - Evolution of Humanoid Robotics
  - Why Embodied Intelligence Matters
  - Role of LLMs in Robotics & Simulation
  - Real-World Use Cases
  - How to Use This Book

Module 1 — Foundations of Physical AI
  Chapter 1: Understanding Embodied Intelligence
    - Page 1: What is Embodied AI?
    - Page 2: AI vs Physical AI
    - Page 3: Cognitive Models & Motor Models

  Chapter 2: Core Concepts of Humanoid Robotics
    - Page 1: Anatomy of a Humanoid Robot
    - Page 2: Sensors, Actuators & Perception Systems
    - Page 3: Motion, Balance & Control Basics

  Chapter 3: Simulation Environments
    - Page 1: Why Simulation Matters
    - Page 2: Physics Engines (Mujoco, Unity, WebSim)
    - Page 3: Safety & Iterative Testing in Simulation

Module 2 — Large Language Models for Robotics
  Chapter 4: LLMs as Robotics Controllers
    - Page 1: Prompting a Robot
    - Page 2: Multi-step Reasoning & Planning
    - Page 3: Error Reduction & Safe Control

  Chapter 5: Tool Use & Agents
    - Page 1: What Are Agents?
    - Page 2: LLM Tools in Robotics
    - Page 3: Realtime Reasoning with Sensor Data

  Chapter 6: Vision + LLM Integration
    - Page 1: How Robots "See"
    - Page 2: VLMs for Object Understanding
    - Page 3: Vision-Action Loop

Module 3 — Building & Controlling a Humanoid
  Chapter 7: Movement & Locomotion
    - Page 1: Walking Algorithms
    - Page 2: Balance & Stabilization
    - Page 3: Advanced Motions (jumping, running)

  Chapter 8: Manipulation & Interaction
    - Page 1: Grasping & Fine Motor Skills
    - Page 2: Human-Robot Interaction
    - Page 3: Safety & Compliance Systems

  Chapter 9: LLM-Driven Control Pipelines
    - Page 1: Natural Language Commands
    - Page 2: Planning → Simulation → Execution
    - Page 3: Error Handling & Recovery

Module 4 — Future of Humanoid AI + Final Project
  Chapter 10: Ethics & Societal Impact
    - Page 1: Safety, Privacy & misuse
    - Page 2: Jobs & Human Augmentation
    - Page 3: Policy & Governance

  Chapter 11: Current Limitations & Future Directions
    - Page 1: Hardware challenges
    - Page 2: LLM shortcomings
    - Page 3: Embodied AI breakthroughs

  Chapter 12: Capstone Project (Hands-on)
    - Page 1: Build a Humanoid Simulation
    - Page 2: Add LLM-based Controls
    - Page 3: Evaluate & Improve Your Robot

## Response snapshot

Successfully integrated the book structure (Introduction + 4 Modules with 12 Chapters) with the existing 13-week course timeline.

**Key Decisions Made:**

1. **Hybrid Structure**: The book's pedagogical structure (modules/chapters) wraps the 13-week implementation timeline. Students see the book organization, but implementation follows weekly milestones.

2. **Page Format**: Each chapter page becomes a separate MDX file for optimal navigation and performance (e.g., `page-01-what-is-embodied-ai.mdx`).

3. **Content Reduction**: Reduced from 65 MDX pages to 42 pages by aligning with the book's 3-page-per-chapter structure:
   - 6 Introduction pages
   - 36 chapter pages (12 chapters × 3 pages each)

4. **Directory Structure**:
   ```
   docs/
   ├── 00-introduction/
   ├── module-01-foundations/
   │   ├── chapter-01-embodied-intelligence/
   │   ├── chapter-02-humanoid-concepts/
   │   └── chapter-03-simulation/
   ├── module-02-llms-for-robotics/
   │   ├── chapter-04-llms-as-controllers/
   │   ├── chapter-05-agents-tools/
   │   └── chapter-06-vision-llm/
   ├── module-03-building-controlling/
   │   ├── chapter-07-locomotion/
   │   ├── chapter-08-manipulation/
   │   └── chapter-09-llm-control-pipelines/
   └── module-04-future-capstone/
       ├── chapter-10-ethics/
       ├── chapter-11-limitations-future/
       └── chapter-12-capstone/
   ```

5. **Week-to-Chapter Mapping**:
   - Week 1: Introduction + Module 1, Ch 1
   - Weeks 2-3: Module 1, Ch 2-3
   - Weeks 4-6: Module 2, Ch 4-6
   - Weeks 7-11: Module 3, Ch 7-9 (longer chapters span 2 weeks)
   - Weeks 12-13: Module 4, Ch 10-12

6. **Code Organization**: Reorganized `static/code-examples/` to follow module/chapter structure instead of week-based structure.

## Outcome

- ✅ Impact: Plan updated with clear book-to-course mapping that preserves pedagogical flow while maintaining implementation timeline
- 🧪 Tests: N/A (planning phase)
- 📁 Files: Updated specs/001-physical-ai-humanoid/plan.md with new sections:
  - 📚 Book Structure Overview
  - 🗺️ Book-to-Course Mapping
  - Updated Phase 2 breakdown with book chapters
  - Updated file structures throughout
- 🔁 Next prompts: Ready for `/sp.tasks` to generate implementation tasks based on book structure
- 🧠 Reflection: The hybrid approach elegantly solves the tension between pedagogical organization (book modules/chapters) and practical implementation (weekly milestones). Students experience a coherent book narrative while developers follow a time-boxed implementation plan.

## Evaluation notes (flywheel)

- Failure modes observed: None - user clarification questions resolved ambiguity upfront
- Graders run and results (PASS/FAIL): PASS - Book structure successfully mapped to course timeline
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Consider adding cross-references in Docusaurus navigation (e.g., "This is Week 4 content" badges on Module 2, Chapter 4 pages) to help students track both book and course progress
