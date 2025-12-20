# Tasks: Frontend Chat Integration

**Feature**: 005-frontend-chat
**Input**: Design documents from `/specs/005-frontend-chat/`
**Prerequisites**: plan.md, spec.md, data-model.md, contracts/frontend-api.md, research.md, quickstart.md

**Tests**: Manual E2E testing (no automated tests per constitution deviation - UI components)

**Organization**: Tasks grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

Based on plan.md structure (Docusaurus frontend):

- **Components**: `src/components/ChatWidget/`
- **Services**: `src/services/`
- **Hooks**: `src/hooks/`
- **Utils**: `src/utils/`
- **Theme**: `src/theme/`
- **Styles**: `src/css/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization, directory structure, and environment configuration

- [x] T001 Create ChatWidget directory structure at src/components/ChatWidget/
- [x] T002 [P] Create services directory at src/services/
- [x] T003 [P] Create hooks directory at src/hooks/
- [x] T004 [P] Create utils directory at src/utils/
- [x] T005 [P] Create theme directory at src/theme/
- [x] T006 Add chatApiUrl to customFields in docusaurus.config.js

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

- [x] T007 Implement sessionStorage helpers (loadChatHistory, saveChatHistory, clearChatHistory) in src/utils/sessionStorage.js
- [x] T008 Implement chatApi service with sendChatMessage, ApiError, getApiUrl, getErrorMessage in src/services/chatApi.js
- [x] T009 Create Root.js theme wrapper to inject ChatWidget globally in src/theme/Root.js
- [x] T010 Create base CSS variables for chat widget in src/components/ChatWidget/styles.module.css

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Ask General Book Questions (Priority: P1) MVP

**Goal**: Enable readers to open chatbot, type questions, receive answers with citations

**Independent Test**: Open chatbot on any page, type "What is Physical AI?", verify response with citations appears

**Acceptance Criteria**:
- Chatbot panel opens with input field (FR-001, FR-002)
- Question submitted to /chat endpoint (FR-003)
- Loading indicator during request (FR-005)
- Response displayed with answer text
- Follow-up questions work in same session

### Implementation for User Story 1

- [x] T011 [P] [US1] Create ChatWidget main component shell in src/components/ChatWidget/ChatWidget.jsx
- [x] T012 [P] [US1] Create ChatWidget index export in src/components/ChatWidget/index.js
- [x] T013 [US1] Implement toggle button (open/close) with aria-label in src/components/ChatWidget/ChatWidget.jsx
- [x] T014 [US1] Implement ChatInput component (input field + send button) in src/components/ChatWidget/ChatInput.jsx
- [x] T015 [US1] Implement ChatMessages container (scrollable area) in src/components/ChatWidget/ChatMessages.jsx
- [x] T016 [US1] Implement ChatMessage component (user/assistant message display) in src/components/ChatWidget/ChatMessage.jsx
- [x] T017 [US1] Implement LoadingIndicator component in src/components/ChatWidget/LoadingIndicator.jsx
- [x] T018 [US1] Wire ChatWidget state: messages[], isOpen, isLoading, input in src/components/ChatWidget/ChatWidget.jsx
- [x] T019 [US1] Implement handleSubmit: validate input, call sendChatMessage, update messages in src/components/ChatWidget/ChatWidget.jsx
- [x] T020 [US1] Add message auto-scroll to bottom on new message in src/components/ChatWidget/ChatWidget.jsx
- [x] T021 [US1] Style toggle button (fixed position bottom-right, Electric Cyan accent) in src/components/ChatWidget/styles.module.css
- [x] T022 [US1] Style chat panel (350px width, 500px max-height, border-radius, shadow) in src/components/ChatWidget/styles.module.css
- [x] T023 [US1] Style chat messages (user right-aligned, assistant left-aligned) in src/components/ChatWidget/styles.module.css
- [x] T024 [US1] Style input area (flex row, input + button) in src/components/ChatWidget/styles.module.css
- [x] T025 [US1] Add empty/whitespace validation to prevent empty submissions (FR-012) in src/components/ChatWidget/ChatWidget.jsx
- [x] T026 [US1] Disable send button when loading or input empty in src/components/ChatWidget/ChatInput.jsx

**Checkpoint**: User Story 1 complete - basic chat functionality works, can ask questions and get answers

---

## Phase 4: User Story 2 - Ask Questions About Selected Text (Priority: P2)

**Goal**: Capture user-selected page text and include as context in questions

**Independent Test**: Select text on page, open chatbot, ask "What does this mean?", verify response references selected content

**Acceptance Criteria**:
- Selected text detected via Window Selection API (FR-004)
- Visual indicator when text is captured (FR-013)
- selected_text parameter sent to API
- Works without selection (treats as general question)

### Implementation for User Story 2

- [x] T027 [US2] Implement useTextSelection hook (mouseup/touchend listeners) in src/hooks/useTextSelection.js
- [x] T028 [US2] Add selectedText state to ChatWidget from useTextSelection hook in src/components/ChatWidget/ChatWidget.jsx
- [x] T029 [US2] Create SelectedTextIndicator component (shows when text captured) in src/components/ChatWidget/SelectedTextIndicator.jsx
- [x] T030 [US2] Update handleSubmit to pass selectedText to sendChatMessage in src/components/ChatWidget/ChatWidget.jsx
- [x] T031 [US2] Add selectedContext field to user messages for display in src/components/ChatWidget/ChatMessage.jsx
- [x] T032 [US2] Truncate selected text to 5000 chars per backend constraint in src/hooks/useTextSelection.js
- [x] T033 [US2] Style SelectedTextIndicator (subtle banner, dismissable info) in src/components/ChatWidget/styles.module.css

**Checkpoint**: User Story 2 complete - text selection context works, enhances answers

---

## Phase 5: User Story 3 - View Response Citations (Priority: P3)

**Goal**: Display source citations with clickable links to book sections

**Independent Test**: Ask factual question, verify citations appear with titles and clickable URLs

**Acceptance Criteria**:
- Citations displayed with title and URL (FR-006)
- Clickable links navigate to source (FR-007)
- Multiple citations listed when applicable
- No fabricated citations on no-result responses

### Implementation for User Story 3

- [x] T034 [US3] Create Citation component (title + link) in src/components/ChatWidget/Citation.jsx
- [x] T035 [US3] Add citations rendering to ChatMessage for assistant messages in src/components/ChatWidget/ChatMessage.jsx
- [x] T036 [US3] Implement link behavior (internal same-tab, external new-tab) in src/components/ChatWidget/Citation.jsx
- [x] T037 [US3] Handle empty citations array (no sources found) in src/components/ChatWidget/ChatMessage.jsx
- [x] T038 [US3] Style citations section (smaller text, indented, link color) in src/components/ChatWidget/styles.module.css

**Checkpoint**: User Story 3 complete - citations display and navigation works

---

## Phase 6: User Story 4 - Persistent Chat Access (Priority: P4)

**Goal**: Maintain chat history across page navigation within session

**Independent Test**: Open chatbot, ask question, navigate to another page, verify chat history visible

**Acceptance Criteria**:
- Chat history persists across navigation (FR-008)
- sessionStorage used for persistence
- Fresh conversation on new browser session
- Chatbot accessible on all pages

### Implementation for User Story 4

- [x] T039 [US4] Add useEffect to load chat history from sessionStorage on mount in src/components/ChatWidget/ChatWidget.jsx
- [x] T040 [US4] Add useEffect to save chat history to sessionStorage on messages change in src/components/ChatWidget/ChatWidget.jsx
- [x] T041 [US4] Add clear chat button to header (optional) in src/components/ChatWidget/ChatWidget.jsx
- [x] T042 [US4] Verify Root.js wrapper renders ChatWidget on all page types in src/theme/Root.js
- [x] T043 [US4] Style header with close button and optional clear button in src/components/ChatWidget/styles.module.css

**Checkpoint**: User Story 4 complete - persistence works, chatbot available everywhere

---

## Phase 7: Error Handling & Edge Cases

**Purpose**: Graceful error handling and edge case coverage (FR-009)

- [x] T044 Implement error message display (isError flag styling) in src/components/ChatWidget/ChatMessage.jsx
- [x] T045 Add try/catch around sendChatMessage with user-friendly error messages in src/components/ChatWidget/ChatWidget.jsx
- [x] T046 Handle network timeout (30s) with retry suggestion in src/components/ChatWidget/ChatWidget.jsx
- [x] T047 Prevent duplicate submissions while loading (disable input) in src/components/ChatWidget/ChatInput.jsx
- [x] T048 Handle very long responses with scrollable message area (FR-011) in src/components/ChatWidget/styles.module.css
- [x] T049 Add mobile responsive styles (full-width on small screens) in src/components/ChatWidget/styles.module.css

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Accessibility, dark mode, final polish

- [x] T050 [P] Add aria-label, role="dialog", aria-expanded attributes for accessibility in src/components/ChatWidget/ChatWidget.jsx
- [x] T051 [P] Add aria-live="polite" to messages area for screen readers in src/components/ChatWidget/ChatMessages.jsx
- [x] T052 [P] Verify dark/light mode styling using Docusaurus CSS variables in src/components/ChatWidget/styles.module.css
- [x] T053 [P] Add focus management (auto-focus input when panel opens) in src/components/ChatWidget/ChatWidget.jsx
- [x] T054 Add keyboard support (Enter to submit, Escape to close) in src/components/ChatWidget/ChatWidget.jsx
- [x] T055 Run Lighthouse audit and fix any accessibility issues
- [x] T056 Verify no console errors on all page types
- [x] T057 Run quickstart.md testing checklist validation

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1: Setup
    ↓
Phase 2: Foundational (BLOCKS all user stories)
    ↓
    ├── Phase 3: US1 (P1) - MVP
    ├── Phase 4: US2 (P2) - Can start after Phase 2
    ├── Phase 5: US3 (P3) - Can start after Phase 2
    └── Phase 6: US4 (P4) - Can start after Phase 2
          ↓
Phase 7: Error Handling (after all stories)
          ↓
Phase 8: Polish (final)
```

### User Story Dependencies

| Story | Depends On | Can Parallelize With |
|-------|------------|---------------------|
| US1 (P1) | Phase 2 | None (MVP first) |
| US2 (P2) | Phase 2 | US1 (different files) |
| US3 (P3) | Phase 2 + T016 (ChatMessage) | US2, US4 |
| US4 (P4) | Phase 2 + T007 (sessionStorage) | US2, US3 |

### Within Each User Story

1. Component shell first
2. Core functionality
3. Styling
4. Edge cases

### Parallel Opportunities

**Phase 1** (all [P]):
```
T002, T003, T004, T005 can run in parallel
```

**Phase 3 - US1**:
```
T011, T012 can run in parallel (component shell + export)
```

**Phase 8 - Polish** (all [P]):
```
T050, T051, T052, T053 can run in parallel
```

---

## Parallel Example: User Story 1 Start

```bash
# Launch component shells in parallel:
Task: "Create ChatWidget main component shell in src/components/ChatWidget/ChatWidget.jsx"
Task: "Create ChatWidget index export in src/components/ChatWidget/index.js"

# Then sequentially:
Task: "Implement toggle button (open/close) with aria-label..."
Task: "Implement ChatInput component..."
# etc.
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T006)
2. Complete Phase 2: Foundational (T007-T010)
3. Complete Phase 3: User Story 1 (T011-T026)
4. **STOP and VALIDATE**: Test basic chat on live site
5. Deploy if ready - core value delivered

### Incremental Delivery

| Milestone | Stories Complete | Value Delivered |
|-----------|-----------------|-----------------|
| MVP | US1 | Basic Q&A functionality |
| +US2 | US1, US2 | Contextual text selection |
| +US3 | US1, US2, US3 | Citation navigation |
| +US4 | All | Full persistence |
| Polish | All + cleanup | Production ready |

### Task Summary

| Phase | Task Count | Purpose |
|-------|------------|---------|
| Setup | 6 | Directory structure, config |
| Foundational | 4 | Core services, theme wrapper |
| US1 (MVP) | 16 | Basic chat functionality |
| US2 | 7 | Text selection |
| US3 | 5 | Citations |
| US4 | 5 | Persistence |
| Error Handling | 6 | Edge cases |
| Polish | 8 | Accessibility, final QA |
| **Total** | **57** | |

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- No automated tests per plan.md (manual E2E testing for UI)
