# Implementation Tasks: Chapter-Level Urdu Translation

**Feature**: 008-urdu-translation
**Generated**: 2025-12-18
**Spec**: [specs/008-urdu-translation/spec.md](specs/008-urdu-translation/spec.md)
**Plan**: [specs/008-urdu-translation/plan.md](specs/008-urdu-translation/plan.md)

## Implementation Strategy

**MVP Scope**: User Story 1 (Translate Chapter Content to Urdu) with basic functionality
**Approach**: Implement backend service first, then frontend components, then integrate
**Testing**: Each user story is independently testable with clear acceptance criteria

## Dependencies

User stories can be implemented in parallel after foundational setup is complete:
- User Story 1 (P1) - Core translation functionality
- User Story 2 (P2) - Toggle functionality
- User Story 3 (P3) - Technical content preservation

## Parallel Execution Examples

Each user story can be developed in parallel by different developers after Phase 1-2 setup:
- Backend developer: Translation service and API
- Frontend developer: Translation button and UI components
- QA developer: Test cases for each story

---

## Phase 1: Setup

- [x] T001 Create backend translation service directory structure in backend/src/translation/
- [x] T002 Create frontend translation components directory structure in src/components/TranslateButton/
- [x] T003 Create translation hook in src/hooks/useChapterTranslation.js
- [x] T004 Create translation API service in src/services/translationApi.js
- [x] T005 Create translation prompt template in backend/src/translation/translation_prompt.md
- [x] T006 [P] Create backend models for translation in backend/src/translation/models.py
- [x] T007 [P] Create backend service for translation in backend/src/translation/service.py
- [x] T008 [P] Create backend API route for translation in backend/src/api/routes/translate.py

## Phase 2: Foundational

- [x] T009 Implement TranslationService class with basic structure in backend/src/translation/service.py
- [x] T010 Create translation API endpoint in backend/src/api/routes/translate.py
- [x] T011 Implement translation prompt template with technical content preservation rules in backend/src/translation/translation_prompt.md
- [x] T012 Create basic translation API service in src/services/translationApi.js
- [x] T013 Create basic translation hook structure in src/hooks/useChapterTranslation.js
- [x] T014 Create basic translation button component structure in src/components/TranslateButton/TranslateButton.jsx
- [x] T015 Add translation button CSS module in src/components/TranslateButton/TranslateButton.module.css

## Phase 3: [US1] Translate Chapter Content to Urdu

**Goal**: Implement core functionality to translate chapter content to Urdu while preserving structure and technical elements

**Independent Test**: Can be fully tested by clicking the translation button and verifying that the chapter content transforms to Urdu while maintaining the original structure and leaving technical elements untranslated.

**Acceptance Scenarios**:
1. **Given** a chapter with mixed content (text, headings, code blocks, technical terms), **When** user clicks "Translate to Urdu" button, **Then** all prose text is translated to Urdu while headings, code blocks, and technical identifiers remain unchanged in English
2. **Given** a translated chapter in Urdu, **When** user clicks "Translate to Urdu" button again, **Then** content remains in Urdu (no double translation occurs)

- [x] T016 [US1] Implement translation service logic to preserve code blocks and technical identifiers in backend/src/translation/service.py
- [x] T017 [US1] Add validation to ensure translated content preserves document structure in backend/src/translation/service.py
- [x] T018 [US1] Implement translation API endpoint with proper error handling in backend/src/api/routes/translate.py
- [x] T019 [US1] Create translation API service with proper error handling in src/services/translationApi.js
- [x] T020 [US1] Implement translation hook with caching mechanism in src/hooks/useChapterTranslation.js
- [x] T021 [US1] Create translation button component with loading states in src/components/TranslateButton/TranslateButton.jsx
- [x] T022 [US1] Add CSS styling for translation button states in src/components/TranslateButton/TranslateButton.module.css
- [x] T023 [US1] Integrate translation button with hook in src/components/TranslateButton/TranslateButton.jsx
- [ ] T024 [US1] Test translation functionality with sample chapter content

## Phase 4: [US2] Toggle Between English and Urdu

**Goal**: Implement functionality to toggle between English and Urdu versions without page reload

**Independent Test**: Can be fully tested by translating to Urdu, then using the toggle mechanism to switch back to English, and then back to Urdu again.

**Acceptance Scenarios**:
1. **Given** a chapter in English, **When** user clicks "Translate to Urdu" then toggles back to English, **Then** content returns to original English state with no loss of formatting
2. **Given** a chapter in Urdu translation, **When** user toggles to English then back to Urdu, **Then** content switches smoothly between both languages

- [x] T025 [US2] Enhance translation hook to manage language state in src/hooks/useChapterTranslation.js
- [x] T026 [US2] Add toggle functionality to translation hook in src/hooks/useChapterTranslation.js
- [x] T027 [US2] Update translation button component to support toggle functionality in src/components/TranslateButton/TranslateButton.jsx
- [x] T028 [US2] Add "View Original" button state in src/components/TranslateButton/TranslateButton.jsx
- [x] T029 [US2] Implement toggle animation and UX in src/components/TranslateButton/TranslateButton.module.css
- [ ] T030 [US2] Test toggle functionality between English and Urdu versions

## Phase 5: [US3] Preserve Technical Content During Translation

**Goal**: Ensure that code blocks, ROS topics, CLI commands, APIs, file paths, and technical identifiers remain in English during translation

**Independent Test**: Can be fully tested by verifying that specific technical elements (code blocks, commands, file paths) remain in English regardless of the surrounding text translation.

**Acceptance Scenarios**:
1. **Given** a chapter with code blocks and technical terms, **When** translation occurs, **Then** code blocks remain in English while surrounding text is translated to Urdu
2. **Given** a chapter with ROS topics and file paths, **When** translation occurs, **Then** these technical identifiers remain in English while prose is translated to Urdu

- [x] T031 [US3] Enhance translation prompt to specifically preserve technical identifiers in backend/src/translation/translation_prompt.md
- [x] T032 [US3] Implement regex patterns to identify and protect technical elements in backend/src/translation/service.py
- [x] T033 [US3] Add validation to ensure code blocks remain unchanged in backend/src/translation/service.py
- [x] T034 [US3] Test preservation of code blocks during translation
- [x] T035 [US3] Test preservation of technical identifiers (ROS topics, CLI commands, APIs, file paths) during translation

## Phase 6: Polish & Cross-Cutting Concerns

- [x] T036 Add error handling for translation API unavailability in src/services/translationApi.js
- [x] T037 Implement timeout handling for long translation requests in src/services/translationApi.js
- [x] T038 Add proper loading indicators during translation in src/components/TranslateButton/TranslateButton.jsx
- [x] T039 Implement content caching to avoid repeated API calls in src/hooks/useChapterTranslation.js
- [x] T040 Add proper accessibility attributes to translation button in src/components/TranslateButton/TranslateButton.jsx
- [x] T041 Create translation wrapper component for DocItem integration in src/theme/DocItem/TranslationWrapper.js
- [x] T042 Integrate translation button into Docusaurus theme in src/theme/DocItem/index.js
- [ ] T043 Add comprehensive error messages for translation failures
- [ ] T044 Test edge case: handling of extremely large chapter content
- [ ] T045 Test edge case: navigation during translation
- [ ] T046 Add performance monitoring for translation response times
- [ ] T047 Document translation feature in user documentation
- [ ] T048 Run end-to-end tests for all user stories