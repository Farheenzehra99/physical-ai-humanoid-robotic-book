# Tasks: Module 4 - Vision-Language-Action (VLA)

**Input**: Design documents from `/specs/001-vla-module/`
**Prerequisites**: plan.md (complete), spec.md (complete), research.md, data-model.md, contracts/

**Tests**: Not required - this is a documentation/book content feature

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story. Since this is book content, user stories map to chapters and learning outcomes.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## User Story Mapping

| Story | Priority | Description | Chapter |
|-------|----------|-------------|---------|
| US1 | P1 | Voice Command to Robot Action | Chapter 8 |
| US2 | P2 | Natural Language Task Planning | Chapter 9 |
| US3 | P3 | Integrated Capstone Demo | Chapter 10 |
| US4 | P1 | Learning Chapter Content | All chapters |

---

## Phase 1: Setup (Module Infrastructure)

**Purpose**: Create directory structure and sidebar configuration

- [x] T001 Create module directory at docs/module-04-vision-language-action/
- [x] T002 Create module _category_.json in docs/module-04-vision-language-action/_category_.json
- [x] T003 [P] Create chapter-08 directory at docs/module-04-vision-language-action/chapter-08-voice-to-action/
- [x] T004 [P] Create chapter-09 directory at docs/module-04-vision-language-action/chapter-09-cognitive-planning/
- [x] T005 [P] Create chapter-10 directory at docs/module-04-vision-language-action/chapter-10-capstone-autonomous-humanoid/
- [x] T006 [P] Create chapter-08 _category_.json in docs/module-04-vision-language-action/chapter-08-voice-to-action/_category_.json
- [x] T007 [P] Create chapter-09 _category_.json in docs/module-04-vision-language-action/chapter-09-cognitive-planning/_category_.json
- [x] T008 [P] Create chapter-10 _category_.json in docs/module-04-vision-language-action/chapter-10-capstone-autonomous-humanoid/_category_.json
- [x] T009 Update sidebars.js to add Module 04 and remove Module 05 entry
- [x] T010 Remove docs/module-05-integrated-rag-chatbot/ directory (backup first if needed)

**Checkpoint**: Module structure ready - chapter content creation can begin

---

## Phase 2: User Story 1 & 4 - Voice-to-Action Chapter (Priority: P1) 🎯 MVP

**Goal**: Create Chapter 8 content teaching speech recognition integration with ROS 2 (US1 functionality, US4 learning content)

**Independent Test**: A new learner can follow Chapter 8 and implement a working voice command ROS 2 node that publishes to /robot_commands topic

### Chapter 8 Content Implementation

- [x] T011 [P] [US1] [US4] Write page-01-intro-speech-robotics.md in docs/module-04-vision-language-action/chapter-08-voice-to-action/page-01-intro-speech-robotics.md
  - Learning objectives
  - Why voice matters for robotics
  - Human-robot interaction overview
  - Connection to ROS 2 from Module 1

- [x] T012 [P] [US1] [US4] Write page-02-whisper-architecture.md in docs/module-04-vision-language-action/chapter-08-voice-to-action/page-02-whisper-architecture.md
  - How Whisper works (encoder-decoder)
  - Model sizes and accuracy tradeoffs
  - Why Whisper suits robotics applications
  - Local vs API deployment comparison

- [x] T013 [P] [US1] [US4] Write page-03-whisper-setup.md in docs/module-04-vision-language-action/chapter-08-voice-to-action/page-03-whisper-setup.md
  - OpenAI API setup instructions
  - Local whisper.cpp installation
  - Python environment configuration
  - Verification commands

- [x] T014 [US1] [US4] Write page-04-voice-command-pipeline.md in docs/module-04-vision-language-action/chapter-08-voice-to-action/page-04-voice-command-pipeline.md
  - Audio capture with sounddevice
  - Voice Activity Detection (VAD)
  - Streaming transcription pattern
  - Working code example: audio capture to text
  - Uses VoiceCommand entity from data-model.md

- [x] T015 [US1] [US4] Write page-05-ros2-integration.md in docs/module-04-vision-language-action/chapter-08-voice-to-action/page-05-ros2-integration.md
  - Voice command ROS 2 node implementation
  - Publishing to /robot_commands topic
  - Handling feedback and errors
  - Complete working code example
  - Integration diagram (mermaid)

- [x] T016 [US1] [US4] Write page-06-exercises.md in docs/module-04-vision-language-action/chapter-08-voice-to-action/page-06-exercises.md
  - Exercise 1: Basic voice capture
  - Exercise 2: Custom command vocabulary
  - Exercise 3: Noise handling
  - Challenge: Multi-language support
  - Summary and next steps

**Checkpoint**: Chapter 8 complete - US1 (Voice Command to Robot) is testable independently. Reader can build working voice-to-ROS pipeline.

---

## Phase 3: User Story 2 & 4 - Cognitive Planning Chapter (Priority: P2)

**Goal**: Create Chapter 9 content teaching LLM-based task decomposition (US2 functionality, US4 learning content)

**Independent Test**: A new learner can follow Chapter 9 and implement an LLM task planner that converts natural language to ROS 2 action sequences

### Chapter 9 Content Implementation

- [x] T017 [P] [US2] [US4] Write page-01-intro-cognitive-robotics.md in docs/module-04-vision-language-action/chapter-09-cognitive-planning/page-01-intro-cognitive-robotics.md
  - Learning objectives
  - Cognitive robotics concepts
  - Natural language understanding for robots
  - Embodied AI foundations
  - Connection to previous chapters

- [x] T018 [P] [US2] [US4] Write page-02-llm-task-planning.md in docs/module-04-vision-language-action/chapter-09-cognitive-planning/page-02-llm-task-planning.md
  - LLM architectures for task planning
  - Comparing providers (OpenAI GPT, Claude, Ollama)
  - Prompt engineering for robotics
  - Cost and latency considerations
  - Code example: Multi-backend TaskPlanner class

- [x] T019 [P] [US2] [US4] Write page-03-natural-language-actions.md in docs/module-04-vision-language-action/chapter-09-cognitive-planning/page-03-natural-language-actions.md
  - Task decomposition methodology
  - Breaking "clean the room" into steps
  - Action representation (TaskPlan, RobotAction entities)
  - Output format: JSON action sequences
  - Working code example: command to action list

- [x] T020 [US2] [US4] Write page-04-ros2-action-generation.md in docs/module-04-vision-language-action/chapter-09-cognitive-planning/page-04-ros2-action-generation.md
  - Converting plans to Nav2/MoveIt actions
  - ROS 2 action client integration
  - Safety constraints and validation
  - Complete integration pattern: Voice → LLM → ROS 2
  - Integration diagram (mermaid)

- [x] T021 [US2] [US4] Write page-05-exercises.md in docs/module-04-vision-language-action/chapter-09-cognitive-planning/page-05-exercises.md
  - Exercise 1: Basic task decomposition
  - Exercise 2: Safety validation implementation
  - Exercise 3: Multi-step command handling
  - Challenge: Custom action types
  - Summary and next steps

**Checkpoint**: Chapter 9 complete - US2 (Natural Language Task Planning) is testable independently. Reader can build LLM-based task planner.

---

## Phase 4: User Story 3 & 4 - Capstone Chapter (Priority: P3)

**Goal**: Create Chapter 10 content for the autonomous humanoid capstone project (US3 integration, US4 learning content)

**Independent Test**: A student can follow Chapter 10 and complete the full autonomous humanoid demo within 2-3 weeks

### Chapter 10 Content Implementation

- [x] T022 [P] [US3] [US4] Write page-01-project-overview.md in docs/module-04-vision-language-action/chapter-10-capstone-autonomous-humanoid/page-01-project-overview.md
  - Learning objectives
  - Project requirements specification
  - Demo scenario: "Go to the kitchen table and pick up the blue cup"
  - Success criteria
  - Timeline and milestones (2-3 week estimate)

- [x] T023 [P] [US3] [US4] Write page-02-system-architecture.md in docs/module-04-vision-language-action/chapter-10-capstone-autonomous-humanoid/page-02-system-architecture.md
  - Full system architecture diagram (mermaid)
  - Component breakdown: Voice, Planning, Navigation, Vision, Manipulation
  - Data flow between components
  - ROS 2 node graph
  - State machine for command execution

- [x] T024 [US3] [US4] Write page-03-implementation-guide.md in docs/module-04-vision-language-action/chapter-10-capstone-autonomous-humanoid/page-03-implementation-guide.md
  - Step-by-step build instructions
  - Voice reception subsystem setup
  - Cognitive planning integration
  - Nav2 path planning configuration
  - Computer vision object detection
  - Manipulation (simplified pick-and-place)
  - Integration and wiring all components

- [x] T025 [US3] [US4] Write page-04-testing-validation.md in docs/module-04-vision-language-action/chapter-10-capstone-autonomous-humanoid/page-04-testing-validation.md
  - Subsystem testing procedures
  - Integration testing approach
  - Common issues and troubleshooting
  - Validation checklist
  - Success metrics verification

- [x] T026 [US3] [US4] Write page-05-demo-presentation.md in docs/module-04-vision-language-action/chapter-10-capstone-autonomous-humanoid/page-05-demo-presentation.md
  - Demo setup instructions
  - Presentation guidelines
  - Live demo best practices
  - Backup plans for failures
  - Congratulations and next steps
  - Links to advanced resources

**Checkpoint**: Chapter 10 complete - US3 (Integrated Capstone Demo) is fully documented. Reader can build complete autonomous humanoid.

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple chapters and final validation

- [x] T027 [P] Update docs/intro.md curriculum structure to include Module 4 description
- [x] T028 [P] Verify all internal links between chapters work correctly
- [x] T029 [P] Verify all code examples have proper syntax highlighting
- [x] T030 [P] Check all mermaid diagrams render correctly
- [x] T031 Verify sidebar navigation shows Module 04 correctly
- [x] T032 Run local development server and test all pages load
- [x] T033 Verify cross-references to Modules 1-3 are accurate
- [x] T034 Final review of learning objectives and summaries consistency
- [x] T035 Run quickstart.md validation steps to ensure setup works

**Checkpoint**: Module 4 complete and validated - ready for review and deployment

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **US1 (Phase 2)**: Depends on Setup completion - Chapter 8 content
- **US2 (Phase 3)**: Depends on Setup completion - Chapter 9 content (can parallel with Phase 2)
- **US3 (Phase 4)**: Depends on Setup completion - Chapter 10 content (can parallel with Phase 2, 3)
- **Polish (Phase 5)**: Depends on all chapter content being complete

### User Story Dependencies

- **US1 & US4 (Chapter 8)**: Can start after Setup - No dependencies on other stories
- **US2 & US4 (Chapter 9)**: Can start after Setup - Content references US1 but not dependent
- **US3 & US4 (Chapter 10)**: Can start after Setup - Integrates US1 and US2 concepts
- **US4**: Learning content is embedded in all chapters

### Within Each Chapter

- Pages 01-03 can be written in parallel (introductory content)
- Pages 04-05 should follow 01-03 (build on earlier concepts)
- Page 06 (exercises) should be last (requires full chapter context)

### Parallel Opportunities

**Phase 1 (Setup)**:
```bash
# After T001-T002, these can run in parallel:
T003, T004, T005 (create chapter directories)
T006, T007, T008 (create _category_.json files)
```

**Phase 2 (Chapter 8)** - After Setup:
```bash
# These introductory pages can run in parallel:
T011 (intro-speech-robotics)
T012 (whisper-architecture)
T013 (whisper-setup)
```

**Phase 3 (Chapter 9)** - Can parallel with Phase 2:
```bash
# These introductory pages can run in parallel:
T017 (intro-cognitive-robotics)
T018 (llm-task-planning)
T019 (natural-language-actions)
```

**Phase 4 (Chapter 10)** - Can parallel with Phase 2, 3:
```bash
# These pages can run in parallel:
T022 (project-overview)
T023 (system-architecture)
```

**Phase 5 (Polish)** - These can run in parallel:
```bash
T027, T028, T029, T030 (documentation and verification)
```

---

## Parallel Example: Chapter 8 Implementation

```bash
# After Setup complete, launch intro pages in parallel:
Task: T011 "Write page-01-intro-speech-robotics.md"
Task: T012 "Write page-02-whisper-architecture.md"
Task: T013 "Write page-03-whisper-setup.md"

# Then sequentially (builds on intro):
Task: T014 "Write page-04-voice-command-pipeline.md"
Task: T015 "Write page-05-ros2-integration.md"
Task: T016 "Write page-06-exercises.md"
```

---

## Implementation Strategy

### MVP First (Chapter 8 Only)

1. Complete Phase 1: Setup (T001-T010)
2. Complete Phase 2: Chapter 8 (T011-T016)
3. **STOP and VALIDATE**: Test Chapter 8 independently
   - Can a new reader follow the content?
   - Do code examples work?
   - Is sidebar navigation correct?
4. Deploy preview if ready

### Incremental Delivery

1. Complete Setup → Module structure ready
2. Add Chapter 8 → Voice-to-Action functional → Deploy preview
3. Add Chapter 9 → Cognitive Planning functional → Deploy preview
4. Add Chapter 10 → Capstone complete → Full deployment
5. Each chapter adds value without breaking previous chapters

### Parallel Author Strategy

With multiple content authors:

1. Team completes Setup together (Phase 1)
2. Once Setup is done:
   - Author A: Chapter 8 (Voice-to-Action)
   - Author B: Chapter 9 (Cognitive Planning)
   - Author C: Chapter 10 (Capstone)
3. Chapters complete and integrate independently
4. All authors collaborate on Phase 5 (Polish)

---

## Task Summary

| Phase | Tasks | Parallel | Description |
|-------|-------|----------|-------------|
| Setup | T001-T010 | 8 | Module infrastructure |
| US1/US4 (Ch8) | T011-T016 | 3 | Voice-to-Action content |
| US2/US4 (Ch9) | T017-T021 | 3 | Cognitive Planning content |
| US3/US4 (Ch10) | T022-T026 | 2 | Capstone content |
| Polish | T027-T035 | 4 | Final validation |

**Total**: 35 tasks
- **Setup**: 10 tasks
- **Chapter 8**: 6 tasks
- **Chapter 9**: 5 tasks
- **Chapter 10**: 5 tasks
- **Polish**: 9 tasks

**Parallel Opportunities**: 20 tasks can run in parallel (across different phases)

**MVP Scope**: Setup (T001-T010) + Chapter 8 (T011-T016) = 16 tasks

---

## Notes

- [P] tasks = different files, no dependencies
- [US1/US2/US3/US4] labels map tasks to user stories for traceability
- Each chapter is independently completable and testable
- No TDD required - this is documentation content, not code
- Commit after each task or logical group
- Stop at any checkpoint to validate chapter independently
- Follow frontmatter template from plan.md for all .md files
- Use mermaid for architecture diagrams
- Use admonitions (:::tip, :::warning, :::info) for callouts
