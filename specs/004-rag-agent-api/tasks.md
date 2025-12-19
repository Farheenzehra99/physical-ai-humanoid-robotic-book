# Tasks: RAG Agent-Based Question Answering API

**Input**: Design documents from `/specs/004-rag-agent-api/`
**Prerequisites**: plan.md ✓, spec.md ✓, research.md ✓, data-model.md ✓, contracts/openapi.yaml ✓

**Tests**: Not explicitly requested in specification. Tests are excluded from task list.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/` for API code (per plan.md structure)
- **Tests**: `tests/api/` for new API tests

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and dependency configuration

- [x] T001 Create backend/src/ directory structure per plan.md (api/, agent/, models/ subdirectories)
- [x] T002 Update backend/pyproject.toml with FastAPI, uvicorn, openai-agents, pydantic dependencies
- [x] T003 [P] Update backend/.env.example with OPENAI_API_KEY placeholder
- [x] T004 [P] Create backend/src/__init__.py module file
- [x] T005 [P] Create backend/src/config.py with FastAPI and Agent configuration settings

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T006 Create Pydantic models in backend/src/models/request.py (ChatRequest per data-model.md)
- [x] T007 [P] Create Pydantic models in backend/src/models/response.py (ChatResponse, Citation, ResponseMetadata per data-model.md)
- [x] T008 [P] Create ErrorResponse Pydantic model in backend/src/models/error.py
- [x] T009 [P] Create backend/src/models/__init__.py with model exports
- [x] T010 Create backend/src/api/__init__.py module file
- [x] T011 [P] Create backend/src/api/middleware/__init__.py module file
- [x] T012 Implement global error handlers in backend/src/api/middleware/error_handler.py (422, 429, 503, 504 per openapi.yaml)
- [x] T013 [P] Create backend/src/api/routes/__init__.py module file
- [x] T014 Create health check endpoint in backend/src/api/routes/health.py (GET /api/v1/health per openapi.yaml)
- [x] T015 Create FastAPI application factory in backend/src/api/app.py with middleware and router registration
- [x] T016 Refactor backend/main.py to use application factory from src/api/app.py (created run_api.py entry point)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Ask General Book Questions (Priority: P1) 🎯 MVP

**Goal**: Users can ask open-ended questions about the book content through a chat endpoint and receive grounded, cited answers within 800ms p95.

**Independent Test**: Submit various question types to POST /api/v1/chat and verify responses are grounded in retrieved context, include source citations, and accurately answer questions.

### Implementation for User Story 1

- [x] T017 Create backend/src/agent/__init__.py module file
- [x] T018 [P] [US1] Create backend/src/agent/tools/__init__.py module file
- [x] T019 [US1] Create retrieval tool wrapper in backend/src/agent/tools/retrieval_tool.py (wraps scripts/retrieval/retrieve.py with @function_tool decorator per research.md)
- [x] T020 [P] [US1] Create agent system prompt in backend/src/agent/agent_prompt.md (grounding rules, citation format, response structure per research.md)
- [x] T021 [US1] Implement OpenAI Agent configuration in backend/src/agent/agent.py (Agent with retrieval tool, Runner.run() per research.md)
- [x] T022 [US1] Create agent dependency injection in backend/src/agent/dependencies.py (cached get_agent() function)
- [x] T023 [US1] Implement POST /chat endpoint in backend/src/api/routes/chat.py (accepts ChatRequest, returns ChatResponse per openapi.yaml)
- [x] T024 [US1] Add citation extraction logic to parse agent output and build Citation array in backend/src/api/routes/chat.py
- [x] T025 [US1] Add ResponseMetadata generation (request_id, processing_time_ms, retrieval_count, model_used) in backend/src/api/routes/chat.py
- [x] T026 [US1] Handle no-context-found case with graceful message in backend/src/agent/agent_prompt.md and backend/src/api/routes/chat.py

**Checkpoint**: User Story 1 complete - users can ask general questions and receive grounded, cited answers

---

## Phase 4: User Story 2 - Ask Questions with Selected Text Context (Priority: P2)

**Goal**: Users can submit questions with selected text from the book that focuses the answer on their specific interest area.

**Independent Test**: Send requests with both question and selected_text parameter, verify responses prioritize selected context while incorporating additional retrieved information.

### Implementation for User Story 2

- [x] T027 [US2] Extend retrieval tool in backend/src/agent/tools/retrieval_tool.py to accept optional selected_text context parameter (handled via agent context)
- [x] T028 [US2] Update agent system prompt in backend/src/agent/agent_prompt.md with selected text handling instructions (prioritize, reference explicitly per research.md)
- [x] T029 [US2] Modify agent invocation in backend/src/api/routes/chat.py to pass selected_text to agent context
- [x] T030 [US2] Handle long selected_text (>2000 chars) gracefully with summarization hint in agent prompt

**Checkpoint**: User Story 2 complete - users can ask focused questions with selected text context

---

## Phase 5: User Story 3 - Receive Cited, Verifiable Responses (Priority: P3)

**Goal**: Users receive responses with explicit source citations (URLs, titles) for all factual claims, enabling verification against source material.

**Independent Test**: Analyze response structure to verify factual claims include citations in consistent format with metadata matching retrieved chunk sources.

### Implementation for User Story 3

- [x] T031 [US3] Enhance Citation.from_retrieval_result() mapping in backend/src/models/response.py to ensure all fields populated
- [x] T032 [US3] Update agent prompt in backend/src/agent/agent_prompt.md to enforce citation format [Source: {title}]({url}) per research.md
- [x] T033 [US3] Implement citation deduplication and multi-source handling in backend/src/agent/agent.py
- [x] T034 [US3] Handle responses with no relevant context - ensure no fabricated citations in backend/src/agent/agent.py
- [x] T035 [US3] Add structured logging for citation metrics (count, sources) in backend/src/agent/agent.py

**Checkpoint**: User Story 3 complete - all responses include verifiable citations for factual claims

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T036 [P] Add structured logging throughout backend/src/ using Python logging module
- [x] T037 [P] Add request timeout handling (504 for >2s) in backend/src/api/middleware/error_handler.py
- [x] T038 [P] Add rate limit handling (429) in backend/src/api/middleware/error_handler.py
- [x] T039 Implement graceful degradation for external service failures (Qdrant, Cohere, OpenAI) in backend/src/agent/tools/retrieval_tool.py
- [x] T040 Add service health checks to backend/src/api/routes/health.py (ping Qdrant, Cohere, OpenAI)
- [ ] T041 Run quickstart.md validation - verify all curl commands work as documented (requires server running)
- [ ] T042 Performance validation - verify p95 latency ≤800ms for typical questions (requires server running)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Phase 6)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Builds on US1 agent but independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Enhances US1 response format but independently testable

### Within Each User Story

- Models before services
- Services before endpoints
- Agent configuration before endpoint implementation
- Core implementation before integration

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel (T003, T004, T005)
- All Foundational tasks marked [P] can run in parallel (T007, T008, T009, T011, T013)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All Polish tasks marked [P] can run in parallel (T036, T037, T038)

---

## Parallel Example: Phase 2 (Foundational)

```bash
# After T006 completes, launch these in parallel:
Task: "Create Pydantic models in backend/src/models/response.py" (T007)
Task: "Create ErrorResponse Pydantic model in backend/src/models/error.py" (T008)
Task: "Create backend/src/models/__init__.py with model exports" (T009)

# After T010 completes, launch these in parallel:
Task: "Create backend/src/api/middleware/__init__.py" (T011)
Task: "Create backend/src/api/routes/__init__.py" (T013)
```

## Parallel Example: User Story 1

```bash
# After T017 completes, launch these in parallel:
Task: "Create backend/src/agent/tools/__init__.py" (T018)
Task: "Create agent system prompt in backend/src/agent/agent_prompt.md" (T020)
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (5 tasks)
2. Complete Phase 2: Foundational (11 tasks) - CRITICAL blocks all stories
3. Complete Phase 3: User Story 1 (10 tasks)
4. **STOP and VALIDATE**: Test User Story 1 independently
   - POST /chat with "What is Physical AI?" returns grounded answer
   - Response includes citations array with source URLs
   - Health endpoint shows all services connected
5. Deploy/demo if ready - MVP delivers core chatbot value

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready (16 tasks)
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!) (26 tasks total)
3. Add User Story 2 → Test with selected_text → Deploy/Demo (30 tasks total)
4. Add User Story 3 → Verify citation quality → Deploy/Demo (35 tasks total)
5. Add Polish → Production-ready → Deploy (42 tasks total)

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (core chat)
   - Developer B: User Story 2 (selected text) - can start immediately if US1 agent interface is defined
   - Developer C: User Story 3 (citations) - can start after T024 provides baseline
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Existing `scripts/retrieval/` module unchanged - wrapped via import
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
