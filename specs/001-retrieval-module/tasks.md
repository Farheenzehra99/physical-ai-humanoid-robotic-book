---
description: "Task list for Standalone Retrieval Module implementation"
---

# Tasks: Standalone Retrieval Module

**Input**: Design documents from `/specs/001-retrieval-module/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: Tests are included based on the success criteria that require rigorous testing with 15 real test questions and performance validation.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `scripts/` for the retrieval module implementation, `tests/` for test files
- Paths align with existing codebase structure where RAG components are in scripts/

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for the standalone retrieval module

- [x] T001 Create retrieval module directory structure in scripts/retrieval/
- [x] T002 Setup environment configuration for Cohere and Qdrant API keys in scripts/.env.example
- [x] T003 [P] Document retrieval module API in scripts/retrieval/README.md

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Create base retrieval configuration in scripts/retrieval/config.py (API endpoints, model settings)
- [x] T005 [P] Implement Cohere client wrapper in scripts/retrieval/embedding_client.py
- [x] T006 [P] Implement Qdrant client wrapper in scripts/retrieval/vector_store.py
- [x] T007 Create error handling framework in scripts/retrieval/exceptions.py (service unavailability, API errors)
- [x] T008 Setup logging configuration in scripts/retrieval/logging_config.py
- [x] T009 Create base retrieval result data model in scripts/retrieval/models.py (RetrievalResult with text, source_url, title, chunk_id, score)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Execute Text Retrieval Query (Priority: P1) 🎯 MVP

**Goal**: AI engineers can submit text queries and retrieve relevant document chunks from the knowledge base using semantic search with Cohere embeddings and Qdrant vector search.

**Independent Test**: Submit various query types (short, long, code snippets, natural language, queries with typos) and verify that relevant document chunks are returned with appropriate scores and complete metadata (text, source_url, title, chunk_id, score).

### Tests for User Story 1

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [x] T010 [P] [US1] Create test fixture with sample queries in tests/retrieval/fixtures/test_queries.json
- [x] T011 [P] [US1] Create integration test for retrieval function in tests/retrieval/test_retrieval_integration.py
- [x] T012 [P] [US1] Create test for typo tolerance in tests/retrieval/test_query_variations.py

### Implementation for User Story 1

- [x] T013 [US1] Implement query embedding function in scripts/retrieval/embedding_client.py (uses embed-english-v3.0 with input_type="search_query")
- [x] T014 [US1] Implement vector search function in scripts/retrieval/vector_store.py (searches rag_embedding collection in Qdrant)
- [x] T015 [US1] Create main retrieve() function in scripts/retrieval/retrieve.py (orchestrates embedding + search)
- [x] T016 [US1] Add result formatting in scripts/retrieval/retrieve.py (returns list of dicts with text, source_url, title, chunk_id, score)
- [x] T017 [US1] Add error handling for service unavailability in scripts/retrieval/retrieve.py
- [x] T018 [US1] Add logging for retrieval operations in scripts/retrieval/retrieve.py
- [x] T019 [US1] Ensure results are sorted by relevance score (descending order) in scripts/retrieval/retrieve.py

**Checkpoint**: At this point, User Story 1 should be fully functional - retrieve(query: str, top_k: int = 8) → list[dict] works perfectly

---

## Phase 4: User Story 2 - Configure Retrieval Parameters (Priority: P2)

**Goal**: AI engineers can customize retrieval behavior by specifying the number of results to return (top_k parameter), balancing precision and recall based on use case.

**Independent Test**: Call the retrieve() function with different top_k values (1, 3, 8, 20) and verify that the number of returned results matches the specified parameter.

### Tests for User Story 2

- [ ] T020 [P] [US2] Create parameter validation tests in tests/retrieval/test_parameter_validation.py
- [ ] T021 [P] [US2] Create edge case tests for top_k parameter in tests/retrieval/test_edge_cases.py (top_k=0, top_k > available results)

### Implementation for User Story 2

- [ ] T022 [US2] Add top_k parameter validation in scripts/retrieval/retrieve.py (default=8, min=1, max=100)
- [ ] T023 [US2] Update retrieve() function to respect top_k parameter in scripts/retrieval/retrieve.py
- [ ] T024 [US2] Handle cases where fewer results exist than top_k in scripts/retrieval/retrieve.py
- [ ] T025 [US2] Add logging for parameter configuration in scripts/retrieval/retrieve.py

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently - engineers can customize result count

---

## Phase 5: User Story 3 - Evaluate Retrieval Quality (Priority: P3)

**Goal**: AI engineers can assess the quality of retrieved results by examining relevance scores and metadata to validate that the system meets quality standards.

**Independent Test**: Run queries with known expected results and verify that relevant content appears in top positions with high scores (≥ 0.78). Measure latency and retrieval accuracy against success criteria.

### Tests for User Story 3

- [ ] T026 [P] [US3] Create performance benchmark tests in tests/retrieval/test_performance.py (measure latency over 20 queries)
- [ ] T027 [P] [US3] Create quality assessment tests in tests/retrieval/test_quality_metrics.py (15 real test questions, verify top-3 relevance ≥ 0.78)
- [ ] T028 [P] [US3] Create diverse query type tests in tests/retrieval/test_query_types.py (short queries, long queries, code snippets)

### Implementation for User Story 3

- [ ] T029 [US3] Implement performance metrics collection in scripts/retrieval/metrics.py (latency tracking, score statistics)
- [ ] T030 [US3] Create quality evaluation script in scripts/retrieval/evaluate_quality.py (runs 15 test questions, reports accuracy)
- [ ] T031 [US3] Add metadata enrichment in scripts/retrieval/retrieve.py (ensure all required fields in response)
- [ ] T032 [US3] Create benchmark runner in scripts/retrieval/benchmark.py (executes 20 queries, reports average latency)
- [ ] T033 [US3] Add score threshold filtering option in scripts/retrieval/retrieve.py (optional min_score parameter)

**Checkpoint**: All user stories should now be independently functional - full retrieval system with quality validation

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and ensure production readiness

- [ ] T034 [P] Create comprehensive usage documentation in scripts/retrieval/README.md
- [ ] T035 [P] Add example usage script in scripts/retrieval/examples/basic_usage.py
- [ ] T036 [P] Create edge case handling tests in tests/retrieval/test_edge_cases.py (empty queries, long queries >1000 chars, non-English queries)
- [ ] T037 Code cleanup and refactoring across retrieval module
- [ ] T038 Add type hints and docstrings to all functions in scripts/retrieval/
- [ ] T039 [P] Create CLI interface in scripts/retrieval/cli.py for standalone testing
- [ ] T040 Validate all success criteria are met (latency < 700ms, 11/15 questions with score ≥ 0.78 in top-3)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Extends US1 retrieve() function but independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Uses US1 and US2 functionality but adds quality metrics independently

### Within Each User Story

- Tests MUST be written and FAIL before implementation
- Client wrappers before main retrieve function
- Core retrieve function before metrics and evaluation
- Basic functionality before quality assessment
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- T005 (Cohere client) and T006 (Qdrant client) can run in parallel in Phase 2
- Once Foundational phase completes, US1/US2/US3 tests can be written in parallel
- T010, T011, T012 (US1 tests) can run in parallel
- T020, T021 (US2 tests) can run in parallel
- T026, T027, T028 (US3 tests) can run in parallel
- T034, T035, T036, T038, T039 (Polish phase) can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together:
Task: "Create test fixture with sample queries in tests/retrieval/fixtures/test_queries.json"
Task: "Create integration test for retrieval function in tests/retrieval/test_retrieval_integration.py"
Task: "Create test for typo tolerance in tests/retrieval/test_query_variations.py"

# After tests fail, implement foundational components in parallel:
Task: "Implement query embedding function in scripts/retrieval/embedding_client.py"
Task: "Implement vector search function in scripts/retrieval/vector_store.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently with real queries
5. Verify retrieve(query: str, top_k: int = 8) → list[dict] works perfectly

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Validate core retrieval works (MVP!)
3. Add User Story 2 → Test independently → Validate parameter configuration
4. Add User Story 3 → Test independently → Validate quality metrics and success criteria
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (core retrieval)
   - Developer B: User Story 2 (parameter configuration)
   - Developer C: User Story 3 (quality evaluation)
3. Stories complete and integrate independently

---

## Success Criteria Validation

After completing all user stories, validate against spec.md success criteria:

- **SC-001**: Verify average latency < 700ms over 20 real queries (benchmark.py)
- **SC-002**: Verify 11 out of 15 real test questions return relevant chunk in top-3 with score ≥ 0.78 (evaluate_quality.py)
- **SC-003**: Verify system handles short queries, long queries, code snippets, and typos (test_query_types.py, test_query_variations.py)
- **SC-004**: Verify system returns exactly top_k results specified (test_parameter_validation.py)
- **SC-005**: Verify system handles 95%+ of queries without errors (test_edge_cases.py, test_retrieval_integration.py)

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- The retrieval module will use existing Qdrant collection `rag_embedding` created by previous ingestion work
- Cohere embed-english-v3.0 model produces 1024-dim vectors with cosine distance
- Focus on clean, reusable code that can be integrated into larger RAG system later
