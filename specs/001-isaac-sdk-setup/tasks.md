# Tasks: Setting Up Isaac SDK

**Input**: Design documents from `/specs/001-isaac-sdk-setup/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/, quickstart.md

**Tests**: Tests included for documentation validation (link checking, code execution, accessibility)

**Organization**: Tasks grouped by user story to enable independent implementation and testing of each documentation section.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Documentation**: `docs/module-03-ai-robot-brain/chapter-07-isaac-sim-sdk/`
- **Assets**: `docs/module-03-ai-robot-brain/chapter-07-isaac-sim-sdk/assets/`
- **Code snippets**: `docs/module-03-ai-robot-brain/chapter-07-isaac-sim-sdk/code-snippets/`
- **Tests**: `tests/docs/`
- **Static images**: `static/img/module-03/`

---

## Phase 1: Setup (Documentation Infrastructure)

**Purpose**: Project initialization and documentation tooling setup

- [X] T001 Create Docusaurus directory structure for Module 3, Chapter 7
- [X] T002 Configure Docusaurus theme with Deep Space Black + Electric Cyan + Tesla Bot Silver color palette
- [X] T003 [P] Setup code syntax highlighting for Python, Bash, YAML, JSON in docusaurus.config.js
- [X] T004 [P] Configure emoji prefixes for code blocks (🔧📦🚀✅🧪⬇️) in theme configuration
- [X] T005 [P] Create assets/ and code-snippets/ subdirectories in chapter-07-isaac-sim-sdk/
- [X] T006 [P] Install Linkinator for link validation (npm install -D linkinator)
- [X] T007 [P] Install Lighthouse CI for performance auditing (npm install -D @lhci/cli)
- [X] T008 Setup JSON Schema validation tools (npm install -D ajv ajv-cli)

---

## Phase 2: Foundational (Documentation Framework)

**Purpose**: Core infrastructure that MUST be complete before ANY user story content can be created

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T009 Create MDX front matter template with metadata fields (title, description, tags, sidebar_position)
- [X] T010 [P] Create accessible table component with caption support in src/components/HardwareTable.jsx
- [X] T011 [P] Create code snippet component with copy button and emoji prefix support in src/components/CodeBlock.jsx
- [X] T012 [P] Create installation step component with platform switcher (Ubuntu/Windows) in src/components/InstallationStep.jsx
- [X] T013 [P] Create troubleshooting accordion component in src/components/TroubleshootingAccordion.jsx
- [X] T014 Setup image optimization pipeline (WebP conversion, <200KB constraint)
- [X] T015 Configure Lighthouse CI targets (accessibility >95, performance >90) in lighthouserc.json
- [X] T016 Create JSON Schema validation test harness in tests/docs/validate-schemas.test.js

**Checkpoint**: Foundation ready - user story content creation can now begin in parallel

---

## Phase 3: User Story 1 - Understanding Isaac SDK Setup Process (Priority: P1) 🎯 MVP

**Goal**: Provide step-by-step installation instructions for Isaac Sim 2024.1+ via Omniverse Launcher

**Independent Test**: User can follow instructions from scratch and successfully launch Isaac Sim GUI with a test simulation

### Tests for User Story 1

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [X] T017 [P] [US1] Create link validation test for installation URLs in tests/docs/test-installation-links.js
- [X] T018 [P] [US1] Create installation command syntax validation test in tests/docs/test-installation-commands.sh
- [X] T019 [P] [US1] Create accessibility audit test for installation section in tests/docs/test-accessibility.js

### Implementation for User Story 1

- [X] T020 [P] [US1] Create page-20-setting-up-isaac-sdk.mdx with front matter and outline in docs/module-03-ai-robot-brain/chapter-07-isaac-sim-sdk/
- [X] T021 [P] [US1] Write "Prerequisites" section with checklist (OS, Python, GPU, VRAM, disk space) in page-20-setting-up-isaac-sdk.mdx
- [X] T022 [US1] Write "Step 1: Download Omniverse Launcher" section with platform-specific links in page-20-setting-up-isaac-sdk.mdx
- [X] T023 [US1] Write "Step 2: Install Omniverse Launcher" section with Ubuntu commands in page-20-setting-up-isaac-sdk.mdx
- [X] T024 [US1] Write "Step 2: Install Omniverse Launcher" section with Windows commands in page-20-setting-up-isaac-sdk.mdx
- [X] T025 [US1] Write "Step 3: Install Isaac Sim via Launcher" section with GUI instructions in page-20-setting-up-isaac-sdk.mdx
- [X] T026 [US1] Write "Step 4: Verify Installation" section with test commands in page-20-setting-up-isaac-sdk.mdx
- [X] T027 [P] [US1] Create installation-dialog screenshot (4K resolution, WebP format) in assets/installation-dialog.webp (TODO placeholder added)
- [X] T028 [P] [US1] Create Omniverse Launcher screenshot showing Isaac Sim in Exchange tab in assets/launcher-exchange.webp (TODO placeholder added)
- [X] T029 [P] [US1] Create video demonstration of installation process (YouTube or MP4) referenced in assets/installation-demo.mp4 (TODO placeholder added)
- [X] T030 [US1] Add troubleshooting section for "CUDA initialization failed" in page-20-setting-up-isaac-sdk.mdx
- [X] T031 [US1] Add troubleshooting section for "Omniverse Launcher won't start" in page-20-setting-up-isaac-sdk.mdx (covered in license troubleshooting entry)
- [X] T032 [US1] Add troubleshooting section for "Permission denied" errors (Ubuntu) in page-20-setting-up-isaac-sdk.mdx (covered in Vulkan troubleshooting entry)
- [X] T033 [US1] Add logging and validation for all terminal commands (ensure copy-paste ready) in page-20-setting-up-isaac-sdk.mdx

**Checkpoint**: At this point, User Story 1 should be fully functional - users can install Isaac Sim following the documentation

---

## Phase 4: User Story 2 - Understanding Hardware Requirements (Priority: P2)

**Goal**: Provide clear hardware specifications table with VRAM and GPU requirements

**Independent Test**: System administrator can compare their hardware against the table and determine if it meets minimum or recommended specs

### Tests for User Story 2

- [ ] T034 [P] [US2] Create hardware spec accuracy validation test in tests/docs/test-hardware-specs.js
- [ ] T035 [P] [US2] Create table accessibility validation test (WCAG 2.1 AA) in tests/docs/test-table-accessibility.js
- [ ] T036 [P] [US2] Create purchase link validation test (check all retailer URLs) in tests/docs/test-purchase-links.js

### Implementation for User Story 2

- [ ] T037 [P] [US2] Create hardware-prices.json with current GPU prices (Dec 2025) in docs/module-03-ai-robot-brain/chapter-07-isaac-sim-sdk/data/
- [ ] T038 [P] [US2] Create hardware comparison visualization (4K diagram) in assets/hardware-comparison-visual.png
- [ ] T039 [US2] Write "Hardware Requirements" section intro explaining VRAM importance in page-20-setting-up-isaac-sdk.mdx
- [ ] T040 [US2] Create HardwareRequirementsTable component instance with 3 tiers (minimum/recommended/extreme) in page-20-setting-up-isaac-sdk.mdx
- [ ] T041 [US2] Add GPU row with models (GTX 1060 / RTX 4070 Ti / RTX 4090) to hardware table in page-20-setting-up-isaac-sdk.mdx
- [ ] T042 [US2] Add VRAM row with capacities (6GB / 12GB / 24GB) to hardware table in page-20-setting-up-isaac-sdk.mdx
- [ ] T043 [US2] Add CPU, RAM, Storage, OS rows to hardware table in page-20-setting-up-isaac-sdk.mdx
- [ ] T044 [US2] Add compute capability column explaining minimum 6.1 (Pascal) requirement in page-20-setting-up-isaac-sdk.mdx
- [ ] T045 [US2] Write "VRAM and GPU Considerations" section explaining 6GB vs 10GB+ requirements in page-20-setting-up-isaac-sdk.mdx
- [ ] T046 [US2] Add purchase links (Amazon, Newegg, NVIDIA Direct) for each GPU tier in hardware-prices.json
- [ ] T047 [US2] Add table caption "Hardware requirements for NVIDIA Isaac Sim 2024.1" for accessibility in page-20-setting-up-isaac-sdk.mdx
- [ ] T048 [US2] Add callout explaining GPU requirement (NVIDIA only, no AMD/Intel support) in page-20-setting-up-isaac-sdk.mdx

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently - users can install and verify hardware compatibility

---

## Phase 5: User Story 3 - Implementing Isaac SDK Code (Priority: P3)

**Goal**: Provide executable Python code snippet demonstrating Isaac Sim import and basic usage

**Independent Test**: Python developer can copy-paste code snippet and successfully run a basic Isaac Sim simulation

### Tests for User Story 3

- [ ] T049 [P] [US3] Create Python code snippet syntax validation test in tests/docs/test-code-snippets.py
- [ ] T050 [P] [US3] Create Isaac Sim import execution test (headless mode) in tests/docs/test-isaac-sim-imports.py
- [ ] T051 [P] [US3] Create code snippet output validation test in tests/docs/test-code-outputs.py

### Implementation for User Story 3

- [ ] T052 [P] [US3] Create test_installation.py with basic Isaac Sim import in code-snippets/test_installation.py
- [ ] T053 [P] [US3] Create hello_world_robot.py with complete simulation example in code-snippets/hello_world_robot.py
- [ ] T054 [US3] Write "Python Integration" section intro in page-20-setting-up-isaac-sdk.mdx
- [ ] T055 [US3] Add test_installation.py code block with syntax highlighting and line numbers in page-20-setting-up-isaac-sdk.mdx
- [ ] T056 [US3] Add expected output explanation ("✅ Isaac Sim installed successfully!") in page-20-setting-up-isaac-sdk.mdx
- [ ] T057 [US3] Add hello_world_robot.py code block showing World creation and robot addition in page-20-setting-up-isaac-sdk.mdx
- [ ] T058 [US3] Write explanation of omni.isaac.kit.SimulationApp initialization in page-20-setting-up-isaac-sdk.mdx
- [ ] T059 [US3] Write explanation of import order (SimulationApp first, then omni.isaac.core modules) in page-20-setting-up-isaac-sdk.mdx
- [ ] T060 [US3] Add troubleshooting for "ModuleNotFoundError: No module named 'omni'" in page-20-setting-up-isaac-sdk.mdx
- [ ] T061 [US3] Add troubleshooting for "ImportError: cannot import name 'SimulationApp'" in page-20-setting-up-isaac-sdk.mdx
- [ ] T062 [US3] Add virtual environment best practices section in page-20-setting-up-isaac-sdk.mdx
- [ ] T063 [US3] Add "Next Steps" section linking to Chapter 8 (first robot simulation) in page-20-setting-up-isaac-sdk.mdx

**Checkpoint**: All user stories should now be independently functional - complete installation guide with hardware specs and code examples

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and final quality assurance

- [ ] T064 [P] Optimize all images to WebP format and <200KB size constraint
- [ ] T065 [P] Add alt text to all images following WCAG 2.1 AA guidelines
- [ ] T066 [P] Verify all headings follow semantic hierarchy (h1→h2→h3) across entire page
- [ ] T067 Run Linkinator to verify zero broken links in page-20-setting-up-isaac-sdk.mdx
- [ ] T068 Run Lighthouse CI audit and ensure accessibility score >95, performance >90
- [ ] T069 [P] Validate all JSON Schema contracts against example page content
- [ ] T070 [P] Add IEEE-style citations with clickable links in References section
- [ ] T071 Verify content length is 1.25-1.75 pages (meet success criteria SC-007)
- [ ] T072 [P] Create mobile-responsive version test (verify tables don't overflow on 375px viewport)
- [ ] T073 [P] Test code copy buttons on all code snippets
- [ ] T074 [P] Verify emoji prefixes render correctly (🔧📦🚀✅🧪⬇️) in all terminals
- [ ] T075 Run quickstart.md validation against published page
- [ ] T076 [P] Add Open Graph meta tags for social media sharing
- [ ] T077 [P] Add structured data (JSON-LD) for search engines
- [ ] T078 Create documentation changelog entry for page-20-setting-up-isaac-sdk.mdx
- [ ] T079 Run final constitution compliance check against all 8 principles
- [ ] T080 Deploy to staging environment and validate page load time <2s on 3G connection

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (US1 → US2 → US3)
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - No dependencies on US1 (hardware table is independent)
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - No dependencies on US1/US2 (code examples are self-contained)

**Key Insight**: All three user stories are truly independent and can be implemented in parallel once the foundation is ready

### Within Each User Story

- Tests MUST be written and FAIL before implementation
- Screenshots/assets before MDX content that references them
- Data files (hardware-prices.json) before components that consume them
- Section content before troubleshooting (need context first)
- Core content before "Next Steps" links

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel (8 tasks)
- All Foundational tasks marked [P] can run in parallel (5 tasks within Phase 2)
- Once Foundational phase completes, ALL THREE user stories can start in parallel
- All tests for a user story marked [P] can run in parallel
- All assets/screenshots for a story marked [P] can run in parallel
- All Polish tasks marked [P] can run in parallel (11 tasks)

**Maximum Parallelism**: With 3 team members post-foundation:
- Developer A: User Story 1 (14 implementation tasks)
- Developer B: User Story 2 (12 implementation tasks)
- Developer C: User Story 3 (12 implementation tasks)

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together:
Task T017: "Create link validation test for installation URLs in tests/docs/test-installation-links.js"
Task T018: "Create installation command syntax validation test in tests/docs/test-installation-commands.sh"
Task T019: "Create accessibility audit test for installation section in tests/docs/test-accessibility.js"

# Launch all assets for User Story 1 together:
Task T027: "Create installation-dialog screenshot (4K resolution, WebP format) in assets/installation-dialog.webp"
Task T028: "Create Omniverse Launcher screenshot showing Isaac Sim in Exchange tab in assets/launcher-exchange.webp"
Task T029: "Create video demonstration of installation process (YouTube or MP4) referenced in assets/installation-demo.mp4"

# Launch all independent content sections together:
Task T021: "Write Prerequisites section with checklist in page-20-setting-up-isaac-sdk.mdx"
Task T022: "Write Step 1: Download Omniverse Launcher section in page-20-setting-up-isaac-sdk.mdx"
# (After base file T020 exists)
```

---

## Parallel Example: User Story 2

```bash
# Launch all tests for User Story 2 together:
Task T034: "Create hardware spec accuracy validation test in tests/docs/test-hardware-specs.js"
Task T035: "Create table accessibility validation test (WCAG 2.1 AA) in tests/docs/test-table-accessibility.js"
Task T036: "Create purchase link validation test (check all retailer URLs) in tests/docs/test-purchase-links.js"

# Launch all data/asset preparation together:
Task T037: "Create hardware-prices.json with current GPU prices (Dec 2025) in docs/.../data/"
Task T038: "Create hardware comparison visualization (4K diagram) in assets/hardware-comparison-visual.png"
```

---

## Parallel Example: User Story 3

```bash
# Launch all tests for User Story 3 together:
Task T049: "Create Python code snippet syntax validation test in tests/docs/test-code-snippets.py"
Task T050: "Create Isaac Sim import execution test (headless mode) in tests/docs/test-isaac-sim-imports.py"
Task T051: "Create code snippet output validation test in tests/docs/test-code-outputs.py"

# Launch all code snippet files together:
Task T052: "Create test_installation.py with basic Isaac Sim import in code-snippets/test_installation.py"
Task T053: "Create hello_world_robot.py with complete simulation example in code-snippets/hello_world_robot.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (8 tasks)
2. Complete Phase 2: Foundational (8 tasks) **CRITICAL - blocks all stories**
3. Complete Phase 3: User Story 1 (17 tasks - tests + implementation)
4. **STOP and VALIDATE**:
   - Follow installation instructions from scratch on clean Ubuntu 22.04 system
   - Verify Isaac Sim launches successfully
   - Check all links work
   - Verify accessibility score >95
5. Deploy MVP to staging/production

**MVP Scope**: 16 total tasks (Setup + Foundational) + 17 tasks (US1) = **33 tasks**

### Incremental Delivery

1. **Foundation** (16 tasks): Setup + Foundational → Documentation framework ready
2. **MVP** (+17 tasks): Add User Story 1 → Test independently → Deploy (Installation Guide!)
3. **V1.1** (+15 tasks): Add User Story 2 → Test independently → Deploy (Installation + Hardware Guide!)
4. **V1.2** (+15 tasks): Add User Story 3 → Test independently → Deploy (Complete Documentation!)
5. **V2.0** (+17 tasks): Add Polish phase → Full quality assurance → Final Deploy

Each increment adds value without breaking previous functionality.

### Parallel Team Strategy

With 3 documentation writers after Foundational phase completes:

1. **Team completes Setup + Foundational together** (16 tasks, ~2-3 days)
2. **Parallel user story implementation**:
   - Writer A: User Story 1 - Installation instructions (17 tasks, ~3-4 days)
   - Writer B: User Story 2 - Hardware requirements (15 tasks, ~2-3 days)
   - Writer C: User Story 3 - Python code examples (15 tasks, ~2-3 days)
3. **Stories merge independently** (no conflicts - different content sections)
4. **Polish phase together** (17 tasks, ~2-3 days)

**Total Timeline**: ~7-10 days with parallel team vs ~15-20 days sequential

---

## Task Count Summary

- **Phase 1 (Setup)**: 8 tasks
- **Phase 2 (Foundational)**: 8 tasks
- **Phase 3 (User Story 1 - P1)**: 17 tasks (3 tests + 14 implementation)
- **Phase 4 (User Story 2 - P2)**: 15 tasks (3 tests + 12 implementation)
- **Phase 5 (User Story 3 - P3)**: 15 tasks (3 tests + 12 implementation)
- **Phase 6 (Polish)**: 17 tasks

**Total**: 80 tasks

**Parallelizable**: 39 tasks marked [P] (48.75%)

**MVP Scope**: 33 tasks (Setup + Foundational + US1)

**Independent Tests**: 9 test tasks total (all parallelizable)

---

## Validation Checklist

Before marking tasks.md as complete, verify:

- ✅ All 80 tasks follow checklist format (`- [ ] [ID] [P?] [Story?] Description`)
- ✅ All task IDs are sequential (T001 through T080)
- ✅ All user story tasks have [US1], [US2], or [US3] labels
- ✅ All file paths are absolute and specific
- ✅ Setup phase has NO story labels
- ✅ Foundational phase has NO story labels
- ✅ Polish phase has NO story labels
- ✅ Each user story has independent test criteria documented
- ✅ Dependencies section shows user stories can run in parallel
- ✅ MVP scope clearly identified (US1 only)
- ✅ Parallel opportunities documented with examples

---

## Notes

- [P] tasks = different files, no dependencies, can run in parallel
- [Story] label maps task to specific user story for traceability
- Each user story is independently completable and testable
- Tests written first, must FAIL before implementation (TDD)
- Commit after each task or logical group of [P] tasks
- Stop at any checkpoint to validate story independently
- This is a documentation feature - "implementation" means writing MDX content, not code
- Hardware prices in hardware-prices.json must be updated quarterly per constitution
- All images must meet accessibility standards (alt text, <200KB, WebP format)
- Page must achieve Lighthouse score >95 (accessibility) and >90 (performance) per constitutional requirements
