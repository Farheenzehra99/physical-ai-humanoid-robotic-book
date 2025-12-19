# Implementation Plan: RAG Agent-Based Question Answering API

**Branch**: `004-rag-agent-api` | **Date**: 2025-12-13 | **Spec**: [specs/004-rag-agent-api/spec.md](./spec.md)
**Input**: Feature specification from `/specs/004-rag-agent-api/spec.md`

## Summary

Build a FastAPI-based RAG agent that answers questions about the Physical AI & Humanoid Robotics book. The agent uses the OpenAI Agents SDK with a registered retrieval tool wrapping the existing Spec-003 retrieval pipeline (Cohere embeddings + Qdrant vector search). Responses are grounded exclusively in retrieved context, with explicit source citations for all factual claims.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**:
- FastAPI + Uvicorn (web framework)
- OpenAI Agents SDK (agent orchestration)
- Cohere SDK (embeddings via existing pipeline)
- Qdrant Client (vector storage via existing pipeline)
- Pydantic (request/response validation)

**Storage**: Qdrant Cloud (`docusaurus_chunks` collection - existing from Spec-002/003)
**Testing**: pytest (existing test infrastructure in `tests/retrieval/`)
**Target Platform**: Linux server / Docker container / Vercel Edge (Python runtime)
**Project Type**: Web application (backend API extending existing backend/)
**Performance Goals**: p95 latency ≤ 800ms end-to-end
**Constraints**:
- Responses must be grounded in retrieved context only (no hallucination)
- 100% citation coverage for factual claims
- Handle 10+ concurrent requests
**Scale/Scope**: Single API endpoint, integration with existing retrieval module

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Execution-First Philosophy | PASS | Uses existing tested retrieval pipeline; API will be testable immediately |
| II. Sim-to-Real Transfer Priority | N/A | Backend API - not applicable to sim-to-real |
| III. Zero-Tolerance Quality Standards | PASS | Extends existing test infrastructure; will include integration tests |
| IV. Reusable Intelligence Architecture | PASS | Agent design follows modular pattern; retrieval tool is composable |
| V. Visual Excellence & User Experience | N/A | Backend API - frontend integration documented separately |
| VI. Open Source & Accessibility | PASS | MIT License; no paywalls; API freely accessible |
| VII. Hardware-in-the-Loop Validation | N/A | Backend API - no hardware deployment |
| VIII. Test-Driven Development | PASS | Tests defined in spec; will implement Red-Green-Refactor |

**Code Quality Standards Check**:
- [x] Type hints (Python) with mypy validation
- [x] Linting (black, ruff, isort)
- [x] Docstrings (Google style - following existing retrieval module)
- [x] Error handling with informative messages
- [x] Structured logging with context
- [x] Security-conscious (env vars for secrets, input validation)

## Project Structure

### Documentation (this feature)

```text
specs/004-rag-agent-api/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
│   └── openapi.yaml     # OpenAPI 3.0 specification
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
backend/
├── main.py              # Existing - will be refactored to app structure
├── pyproject.toml       # Existing - will add FastAPI dependencies
├── .env.example         # Existing - will add OpenAI API key
└── src/
    ├── __init__.py
    ├── api/
    │   ├── __init__.py
    │   ├── routes/
    │   │   ├── __init__.py
    │   │   └── chat.py          # POST /chat endpoint
    │   └── middleware/
    │       ├── __init__.py
    │       └── error_handler.py # Global error handling
    ├── agent/
    │   ├── __init__.py
    │   ├── agent.py             # OpenAI Agent configuration
    │   ├── agent_prompt.md      # System prompt for grounding
    │   └── tools/
    │       ├── __init__.py
    │       └── retrieval_tool.py # Tool wrapping retrieve()
    ├── models/
    │   ├── __init__.py
    │   ├── request.py           # ChatRequest Pydantic model
    │   └── response.py          # ChatResponse, Citation models
    └── config.py                # FastAPI + Agent configuration

scripts/retrieval/          # Existing - no changes needed
├── retrieve.py             # Main retrieve() function
├── models.py               # RetrievalResult, RetrievalQuery
├── config.py               # Config class
└── ...

tests/
├── retrieval/              # Existing tests
│   ├── test_retrieval_integration.py
│   └── test_query_variations.py
└── api/                    # New tests for this feature
    ├── __init__.py
    ├── conftest.py         # Test fixtures (FastAPI TestClient)
    ├── test_chat_endpoint.py
    ├── test_agent_grounding.py
    └── test_citations.py
```

**Structure Decision**: Extend existing `backend/` directory with proper FastAPI application structure. Keep `scripts/retrieval/` unchanged - import and wrap in agent tool. Add new test directory `tests/api/` for API-specific tests.

## Complexity Tracking

> No violations requiring justification. Design follows simplest viable approach:
> - Single API endpoint (POST /chat)
> - Existing retrieval module reused without modification
> - OpenAI Agents SDK handles agent orchestration
> - No additional database or caching layer needed

## Key Technical Decisions

1. **Agent Framework**: OpenAI Agents SDK (per spec requirement FR-004)
   - Provides tool registration, context management, streaming support
   - Well-documented, production-ready

2. **Retrieval Integration**: Wrap existing `scripts/retrieval/retrieve()` as agent tool
   - Zero modification to validated Spec-003 code
   - Tool schema exposes query, top_k parameters

3. **Selected Text Handling**: Hybrid approach
   - When `selected_text` provided: embed and use as primary context
   - Still retrieve additional context for enrichment
   - Agent prompt instructs prioritization of selected text

4. **Citation Format**: Structured array in response
   - Each citation includes: source_url, title, chunk_id, score
   - Maps directly from RetrievalResult fields

5. **Error Handling Strategy**:
   - Pydantic validation for request errors (422)
   - Service unavailable for external API failures (503)
   - Graceful degradation messages for low-quality retrieval
