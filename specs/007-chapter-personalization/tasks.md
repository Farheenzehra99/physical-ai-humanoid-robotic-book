# Tasks: Chapter-Level Content Personalization

**Feature**: 007-chapter-personalization
**Input**: Design documents from `/specs/007-chapter-personalization/`
**Prerequisites**: plan.md, spec.md

**Tests**: Manual E2E testing and unit tests (per plan)

**Organization**: Tasks grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4, US5)
- Include exact file paths in descriptions

## Path Conventions

Based on plan.md structure (Docusaurus frontend + FastAPI backend):

- **Backend Models**: `backend/src/personalization/models.py`
- **Backend Service**: `backend/src/personalization/service.py`
- **Backend Prompt**: `backend/src/personalization/personalization_prompt.md`
- **Backend Endpoint**: `backend/src/api/routes/personalize.py`
- **Frontend Components**: `src/components/PersonalizeButton/`
- **Frontend Hooks**: `src/hooks/useChapterPersonalization.js`
- **Frontend Services**: `src/services/personalizationApi.js`
- **Frontend Theme**: `src/theme/DocItem/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization, directory structure, and dependency configuration

- [ ] T001 Create personalization directory structure at backend/src/personalization/
- [ ] T002 [P] Create frontend components directory at src/components/PersonalizeButton/
- [ ] T003 [P] Create theme swizzle directory at src/theme/DocItem/
- [ ] T004 Install frontend dependencies: react-markdown, remark-gfm
- [ ] T005 Update backend app.py to include personalize route

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

- [ ] T006 Create personalization models at backend/src/personalization/models.py
- [ ] T007 Create personalization service at backend/src/personalization/service.py
- [ ] T008 Create personalization prompt template at backend/src/personalization/personalization_prompt.md
- [ ] T009 Create personalization endpoint at backend/src/api/routes/personalize.py
- [ ] T010 Create personalization API service at src/services/personalizationApi.js
- [ ] T011 Create chapter personalization hook at src/hooks/useChapterPersonalization.js

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Beginner Personalization (Priority: P0) MVP

**Goal**: Enable beginner readers to click "Personalize for Me" and get more explanations and intuition-building

**Independent Test**: Log in as beginner user, click "Personalize for Me" on a chapter, verify content has more explanations and analogies

**Acceptance Criteria**:
- Personalize button visible to authenticated users (FR-01)
- Chapter content replaced with beginner-friendly version (US-01)
- All headings preserved (FR-09)
- Code blocks remain unchanged (FR-09)
- Button changes to "View Original" (US-01)

### Implementation for User Story 1

- [ ] T012 [P] [US1] Create PersonalizeButton component shell at src/components/PersonalizeButton/PersonalizeButton.jsx
- [ ] T013 [P] [US1] Create PersonalizeButton index export at src/components/PersonalizeButton/index.js
- [ ] T014 [US1] Implement idle state (authenticated) in PersonalizeButton with "Personalize for Me" button
- [ ] T015 [US1] Implement loading state with spinner and "Personalizing..." text
- [ ] T016 [US1] Implement personalized state with "View Original" toggle button
- [ ] T017 [US1] Connect PersonalizeButton to useChapterPersonalization hook
- [ ] T018 [US1] Implement personalize function in hook to call backend API
- [ ] T019 [US1] Add beginner-specific adaptation logic to LLM prompt
- [ ] T020 [US1] Implement content validation to ensure headings preserved
- [ ] T021 [US1] Style PersonalizeButton with CSS modules at src/components/PersonalizeButton/PersonalizeButton.module.css

**Checkpoint**: User Story 1 complete - beginner personalization works

---

## Phase 4: User Story 2 - Advanced Personalization (Priority: P0)

**Goal**: Enable advanced readers to get concise info without basic explanations

**Independent Test**: Log in as advanced user, click "Personalize for Me", verify content is more concise with advanced terminology

**Acceptance Criteria**:
- Content becomes more concise for advanced users (US-02)
- Advanced terminology used without over-explanation (US-02)
- "Obvious" setup steps condensed (US-02)
- Code remains fully intact (US-02)

### Implementation for User Story 2

- [ ] T022 [US2] Add advanced-specific adaptation logic to LLM prompt
- [ ] T023 [US2] Update personalization service to handle advanced level adaptations
- [ ] T024 [US2] Test advanced personalization with sample content
- [ ] T025 [US2] Verify code blocks remain unchanged for advanced users

**Checkpoint**: User Story 2 complete - advanced personalization works

---

## Phase 5: User Story 3 - Hardware-Focused Personalization (Priority: P1)

**Goal**: Show content adapted to embedded systems background with relevant analogies

**Independent Test**: Log in as hardware-focused user, personalize content, verify analogies relate to hardware experience

**Acceptance Criteria**:
- Content adapted for hardware background (US-03)
- Analogies relate to user's experience (US-03)
- Hardware-specific examples prioritized (US-03)

### Implementation for User Story 3

- [ ] T026 [US3] Add hardware-focused adaptation logic to LLM prompt
- [ ] T027 [US3] Update service to handle hardware experience level
- [ ] T028 [US3] Test hardware-focused personalization with sample content

**Checkpoint**: User Story 3 complete - hardware-focused personalization works

---

## Phase 6: User Story 4 - Toggle Back to Original (Priority: P1)

**Goal**: Allow users to toggle back to original content

**Independent Test**: View personalized content, click "View Original", verify original content restored

**Acceptance Criteria**:
- Toggle back to original content works (US-04)
- Original content restored without page reload (US-04)
- Button changes back to "Personalize for Me" (US-04)

### Implementation for User Story 4

- [ ] T029 [US4] Implement resetToOriginal function in useChapterPersonalization hook
- [ ] T030 [US4] Add toggle functionality to PersonalizeButton component
- [ ] T031 [US4] Implement smooth transition between original and personalized content
- [ ] T032 [US4] Ensure no page reload occurs during toggle
- [ ] T033 [US4] Test toggle functionality with different user profiles

**Checkpoint**: User Story 4 complete - toggle functionality works

---

## Phase 7: User Story 5 - Unauthenticated User Experience (Priority: P2)

**Goal**: Show sign-in prompt to non-authenticated users

**Independent Test**: Navigate as unauthenticated user, verify sign-in prompt shown

**Acceptance Criteria**:
- Unauthenticated users see sign-in prompt (US-05)
- Sign-in link directs to authentication page (US-05)
- Personalize button not available to unauthenticated users (US-05)

### Implementation for User Story 5

- [ ] T034 [US5] Implement unauthenticated state in PersonalizeButton component
- [ ] T035 [US5] Add sign-in link that directs to authentication page
- [ ] T036 [US5] Verify authentication check works properly in hook
- [ ] T037 [US5] Style sign-in state appropriately

**Checkpoint**: User Story 5 complete - unauthenticated experience implemented

---

## Phase 8: Content Adaptation by Learning Style

**Purpose**: Implement adaptation based on learning style preferences (Theory-first, Code-first, Visual)

- [ ] T038 Implement learning style adaptation logic in LLM prompt
- [ ] T039 Update service to handle learning style preferences
- [ ] T040 Test theory-first learning style adaptation
- [ ] T041 Test code-first learning style adaptation
- [ ] T042 Test visual learning style adaptation

---

## Phase 9: Frontend Integration & Content Display

**Purpose**: Integrate personalization into Docusaurus chapter pages

- [ ] T043 Create swizzled DocItem component at src/theme/DocItem/index.js
- [ ] T044 Add personalization button injection to DocItem component
- [ ] T045 Implement content extraction from DOM in DocItem component
- [ ] T046 Implement personalized content rendering with ReactMarkdown
- [ ] T047 Create DocItem styles at src/theme/DocItem/styles.module.css
- [ ] T048 Add smooth transitions between original and personalized content
- [ ] T049 Test content rendering with various markdown elements

---

## Phase 10: Caching & Performance

**Purpose**: Implement session-based caching for personalized content

- [ ] T050 Add session storage caching to useChapterPersonalization hook
- [ ] T051 Implement cache key generation based on chapter slug
- [ ] T052 Add cache validation and expiration logic
- [ ] T053 Implement cache clearing on profile update/logout
- [ ] T054 Test cache performance improvements
- [ ] T055 Add cache error handling

---

## Phase 11: Error Handling & Edge Cases

**Purpose**: Handle errors and edge cases gracefully (FR-06)

- [ ] T056 [P] Implement error state in PersonalizeButton component
- [ ] T057 [P] Add retry functionality to personalization hook
- [ ] T058 Handle LLM service unavailable errors (503)
- [ ] T059 Handle rate limit exceeded errors (429)
- [ ] T060 Handle content validation failures
- [ ] T061 Handle authentication errors (401)
- [ ] T062 Implement fallback to original content on errors
- [ ] T063 Add content size validation (< 50KB)
- [ ] T064 Implement circuit breaker for personalization service

---

## Phase 12: Testing & Validation

**Purpose**: Ensure quality and reliability of personalization

- [ ] T065 Create backend unit tests for personalization service at backend/tests/test_personalization.py
- [ ] T066 Create frontend component tests for PersonalizeButton at src/__tests__/PersonalizeButton.test.jsx
- [ ] T067 Implement heading preservation validation in service
- [ ] T068 Create E2E tests for personalization flow at tests/e2e/personalization.spec.js
- [ ] T069 Test with different user profile combinations
- [ ] T070 Performance test response times
- [ ] T071 Security test for content sanitization

---

## Phase 13: Documentation & User Guide

**Purpose**: Document the feature for users and developers

- [ ] T072 Update API documentation with personalization endpoint details
- [ ] T073 Create user guide for personalization feature at docs/module-05-integrated-rag-chatbot/personalization.md
- [ ] T074 Add usage instructions to documentation
- [ ] T075 Document troubleshooting tips for common issues

---

## Phase 14: Polish & Cross-Cutting Concerns

**Purpose**: Accessibility, dark mode, final polish

- [ ] T076 [P] Add accessibility attributes to PersonalizeButton component
- [ ] T077 [P] Ensure dark mode compatibility for personalization UI
- [ ] T078 Add keyboard navigation support
- [ ] T079 Implement proper loading states and progress indicators
- [ ] T080 Add analytics tracking for personalization usage
- [ ] T081 Test cross-browser compatibility
- [ ] T082 Optimize performance and bundle size
- [ ] T083 Final QA testing across all user stories

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1: Setup
    ↓
Phase 2: Foundational (BLOCKS all user stories)
    ↓
    ├── Phase 3: US1 (P0) - Beginner personalization MVP
    ├── Phase 4: US2 (P0) - Advanced personalization
    ├── Phase 5: US3 (P1) - Hardware-focused personalization
    ├── Phase 6: US4 (P1) - Toggle functionality
    └── Phase 7: US5 (P2) - Unauthenticated experience
          ↓
Phase 8: Learning Style Adaptation
          ↓
Phase 9: Frontend Integration
          ↓
Phase 10: Caching
          ↓
Phase 11: Error Handling
          ↓
Phase 12: Testing
          ↓
Phase 13: Documentation
          ↓
Phase 14: Polish
```

### User Story Dependencies

| Story | Depends On | Can Parallelize With |
|-------|------------|---------------------|
| US1 (P0) | Phase 2 | None (MVP first) |
| US2 (P0) | Phase 2 | US1 (different adaptation logic) |
| US3 (P1) | Phase 2 | US1, US2 |
| US4 (P1) | Phase 2 + US1 | US2, US3 |
| US5 (P2) | Phase 2 + Auth | US1, US2, US3, US4 |

### Within Each User Story

1. Component shell first
2. Core functionality
3. Styling
4. Edge cases
5. Testing

### Parallel Opportunities

**Phase 1** (all [P]):
```
T002, T003 can run in parallel (directory creation)
```

**Phase 3 - US1**:
```
T012, T013 can run in parallel (component shell + export)
```

**Phase 14 - Polish** (all [P]):
```
T076, T077 can run in parallel (accessibility + dark mode)
```

---

## Parallel Example: User Story 1 Start

```bash
# Launch component shells in parallel:
Task: "Create PersonalizeButton component shell at src/components/PersonalizeButton/PersonalizeButton.jsx"
Task: "Create PersonalizeButton index export at src/components/PersonalizeButton/index.js"

# Then sequentially:
Task: "Implement idle state (authenticated) in PersonalizeButton with 'Personalize for Me' button"
Task: "Implement loading state with spinner and 'Personalizing...' text"
# etc.
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T011)
3. Complete Phase 3: User Story 1 (T012-T021)
4. **STOP and VALIDATE**: Test basic personalization on live site
5. Deploy if ready - core value delivered

### Incremental Delivery

| Milestone | Stories Complete | Value Delivered |
|-----------|-----------------|-----------------|
| MVP | US1 | Basic personalization for beginners |
| +US2 | US1, US2 | Advanced user support |
| +US3 | US1, US2, US3 | Hardware-focused adaptation |
| +US4 | US1, US2, US3, US4 | Toggle functionality |
| +US5 | All | Full feature set |
| +Learning Styles | All + learning styles | Complete personalization |

### Task Summary

| Phase | Task Count | Purpose |
|-------|------------|---------|
| Setup | 5 | Directory structure, dependencies |
| Foundational | 6 | Core services, models, API |
| US1 (MVP) | 9 | Beginner personalization |
| US2 | 4 | Advanced personalization |
| US3 | 3 | Hardware-focused adaptation |
| US4 | 5 | Toggle functionality |
| US5 | 4 | Unauthenticated experience |
| Learning Styles | 5 | Style-based adaptation |
| Frontend Integration | 7 | Docusaurus integration |
| Caching | 6 | Performance optimization |
| Error Handling | 9 | Robust error handling |
| Testing | 7 | Quality assurance |
| Documentation | 4 | User and dev docs |
| Polish | 8 | Final touches |
| **Total** | **83** | |

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- No automated tests per plan.md (manual E2E testing for UI)