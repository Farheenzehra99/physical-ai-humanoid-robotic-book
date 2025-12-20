# Research: RAG Agent-Based Question Answering API

**Feature**: 004-rag-agent-api | **Date**: 2025-12-13 | **Phase**: 0 - Research

## Research Tasks

This document resolves all NEEDS CLARIFICATION items from Technical Context and documents best practices for each technology choice.

---

## 1. OpenAI Agents SDK Integration

### Decision: Use openai-agents Python Package

**Rationale**: Official OpenAI SDK provides first-class tool registration, automatic schema generation, and production-ready agent orchestration.

**Alternatives Considered**:
- LangChain Agents: More complex, heavier dependency footprint
- Custom agent loop: More control but reinvents wheel
- Anthropic Claude SDK: Different API, not specified in requirements

### Key Implementation Details

**Installation**:
```bash
pip install openai-agents
```

**Tool Registration Pattern** (from OpenAI Agents SDK Tools Documentation):

```python
from agents import Agent, function_tool, Runner
import json

@function_tool
async def retrieve_context(query: str, top_k: int = 5) -> str:
    """Retrieve relevant document chunks from the knowledge base.

    Args:
        query: The search query text
        top_k: Number of results to return (default: 5)

    Returns:
        JSON string containing retrieved chunks with metadata
    """
    # Wrap existing retrieve() function
    from scripts.retrieval import retrieve
    results = retrieve(query, top_k=top_k)
    return json.dumps(results)

agent = Agent(
    name="Book Assistant",
    instructions="...",  # System prompt
    tools=[retrieve_context]
)
```

**Running Agents**:
```python
async def main():
    result = await Runner.run(agent, user_question)
    return result.final_output
```

**Key Features**:
- Automatic JSON schema generation from type annotations
- Docstring extraction for tool descriptions
- Async/sync function support
- Built-in error handling with failure_error_function
- Context wrapper for execution state access

---

## 2. Grounding and Anti-Hallucination Strategy

### Decision: Multi-Layer Grounding Approach

**Rationale**: No single technique eliminates hallucinations. Research shows combining multiple strategies yields best results (96% reduction per Stanford 2024 study).

**Alternatives Considered**:
- Prompt-only approach: Insufficient for production reliability
- Post-processing fact-check: Adds latency, complexity
- Confidence thresholds: Useful as complement, not standalone

### Implementation Strategy

**System Prompt Design** (grounding instructions):
```markdown
You are a helpful assistant that answers questions about the Physical AI and Humanoid Robotics book.

CRITICAL RULES:
1. You MUST use the retrieve_context tool for EVERY question before answering
2. You may ONLY include information that appears in the retrieved context
3. If the context does not contain relevant information, say "I could not find information about this in the book"
4. For EVERY factual claim, cite the source with [Source: {title}]({url})
5. NEVER invent, extrapolate, or assume information not in the retrieved chunks
6. If multiple sources support a claim, cite all of them

RESPONSE FORMAT:
- Begin with a direct answer to the question
- Support each claim with citations
- End with a Sources section listing all referenced materials
```

**Multi-Layer Approach**:
1. **Tool-First Enforcement**: Agent instructions mandate retrieval before answering
2. **Context-Only Generation**: System prompt strictly limits to retrieved content
3. **Citation Requirements**: Every factual claim must have source attribution
4. **Graceful Unknowns**: Explicit instruction to admit when information unavailable

### Citation Format Standard

```json
{
  "source_url": "https://physical-ai-book.../chapter-3",
  "title": "Chapter 3: Introduction to Physical AI",
  "chunk_id": "chunk_42",
  "relevance_score": 0.89
}
```

---

## 3. FastAPI Integration Pattern

### Decision: Modular FastAPI Application with Dependency Injection

**Rationale**: Clean separation of concerns, testable architecture, follows existing Python project patterns.

**Alternatives Considered**:
- Flask: Less modern async support
- Starlette (raw): More boilerplate needed
- Django Rest Framework: Overkill for single endpoint

### Implementation Pattern

**Application Factory**:
```python
# backend/src/api/app.py
from fastapi import FastAPI
from .routes import chat_router
from .middleware.error_handler import register_error_handlers

def create_app() -> FastAPI:
    app = FastAPI(
        title="Physical AI Book RAG API",
        version="1.0.0"
    )
    app.include_router(chat_router, prefix="/api/v1")
    register_error_handlers(app)
    return app
```

**Dependency Injection for Agent**:
```python
# backend/src/agent/dependencies.py
from functools import lru_cache
from .agent import create_book_agent

@lru_cache
def get_agent():
    return create_book_agent()
```

**Endpoint Pattern**:
```python
# backend/src/api/routes/chat.py
@router.post("/chat", response_model=ChatResponse)
async def chat(
    request: ChatRequest,
    agent = Depends(get_agent)
) -> ChatResponse:
    result = await Runner.run(agent, request.question)
    return format_response(result)
```

---

## 4. Selected Text Handling

### Decision: Hybrid Context Injection

**Rationale**: Selected text is user-prioritized context that should be emphasized but not replace retrieval entirely.

**Alternatives Considered**:
- Selected text only: Misses relevant context from other sections
- Ignore selected text: Loses user intent signal
- Prepend to query: Dilutes semantic search

### Implementation Strategy

When selected_text is provided:

1. **Include in System Context**: Add selected text as primary context before retrieval
2. **Modified Tool Call**: Pass selected text as additional context parameter
3. **Agent Instructions**: Prioritize explaining/referencing selected text
4. **Still Retrieve**: Fetch additional context for enrichment

**Modified System Prompt Section**:
```markdown
SELECTED TEXT HANDLING:
When the user provides selected text from the book:
1. Treat it as the PRIMARY focus of your answer
2. Use retrieved context to SUPPLEMENT and ENRICH your explanation
3. Always reference the selected text explicitly in your response
4. If asked "explain this" or similar, the selected text IS the subject
```

---

## 5. Error Handling Patterns

### Decision: Layered Error Handling with Graceful Degradation

**Rationale**: RAG systems have multiple failure points (Qdrant, Cohere, OpenAI) requiring specific handling.

### Error Taxonomy

| Error Type | HTTP Status | User Message | Internal Action |
|------------|-------------|--------------|-----------------|
| Empty/invalid question | 422 | Validation error details | Pydantic handles |
| Qdrant unavailable | 503 | Knowledge base temporarily unavailable | Log, retry once |
| Cohere API failure | 503 | Search service temporarily unavailable | Log, no retry |
| OpenAI API failure | 503 | AI service temporarily unavailable | Log, no retry |
| OpenAI rate limit | 429 | Too many requests, please wait | Log, return retry-after |
| Low-quality retrieval | 200 | Include disclaimer in response | Log for analysis |
| Timeout (>2s) | 504 | Request timed out, please try again | Log, terminate |

### Implementation

```python
# backend/src/api/middleware/error_handler.py
@app.exception_handler(ServiceUnavailableError)
async def service_unavailable_handler(request, exc):
    return JSONResponse(
        status_code=503,
        content={
            "error": "service_unavailable",
            "message": str(exc),
            "retry_after": 30
        }
    )
```

---

## 6. Performance Optimization

### Decision: Async Pipeline with Connection Pooling

**Rationale**: p95 latency target of 800ms requires optimized async paths.

### Latency Budget

| Component | Target | Notes |
|-----------|--------|-------|
| Request parsing | <5ms | Pydantic validation |
| Embedding generation | <150ms | Cohere API |
| Vector search | <50ms | Qdrant Cloud |
| Agent reasoning | <500ms | OpenAI GPT-4 |
| Response formatting | <10ms | JSON serialization |
| **Total** | **<715ms** | 85ms buffer for variance |

### Optimization Strategies

1. **Connection Reuse**: Use httpx clients with connection pooling
2. **Async Throughout**: All I/O operations async
3. **Parallel Retrieval**: If selected_text, can parallelize embedding
4. **Response Streaming**: Consider SSE for longer responses (future)
5. **Caching**: LRU cache for agent initialization

---

## 7. Testing Strategy

### Decision: Three-Tier Testing with Mocked External Services

**Rationale**: External API calls are slow and costly; mock for unit/integration tests.

### Test Categories

**Unit Tests** (tests/api/test_*.py):
- Request/response model validation
- Error handler behavior
- Tool schema generation

**Integration Tests** (tests/api/test_*_integration.py):
- Full endpoint flow with mocked agent
- Database integration (if applicable)
- Error propagation

**E2E Tests** (tests/api/test_*_e2e.py):
- Real API calls (gated by --run-e2e flag)
- Latency validation
- Grounding verification

### Mock Patterns

```python
# tests/api/conftest.py
@pytest.fixture
def mock_agent():
    with patch("backend.src.agent.agent.Runner.run") as mock:
        mock.return_value = MockAgentResult(
            final_output="Test response with [Source: Test](url)"
        )
        yield mock

@pytest.fixture
def mock_retrieve():
    with patch("scripts.retrieval.retrieve") as mock:
        mock.return_value = [
            {"text": "...", "source_url": "...", "title": "...", "score": 0.9}
        ]
        yield mock
```

---

## Summary: All Unknowns Resolved

| Item | Resolution |
|------|------------|
| Agent framework | OpenAI Agents SDK (openai-agents package) |
| Tool registration | @function_tool decorator wrapping retrieve() |
| Grounding strategy | Multi-layer: prompt + tool-first + citations |
| Selected text handling | Hybrid context injection |
| FastAPI structure | Application factory with dependency injection |
| Error handling | Layered with graceful degradation |
| Performance | Async pipeline, connection pooling, 715ms budget |
| Testing | Three-tier with external service mocking |

---

## References

- [OpenAI Agents SDK Documentation](https://openai.github.io/openai-agents-python/)
- [OpenAI Agents SDK Tools](https://openai.github.io/openai-agents-python/tools/)
- [OpenAI Agents SDK Quickstart](https://openai.github.io/openai-agents-python/quickstart/)
- [RAG Hallucination Prevention Best Practices](https://www.k2view.com/blog/rag-hallucination/)
- [Agentic RAG for AI Grounding](https://www.moveworks.com/us/en/resources/blog/improved-ai-grounding-with-agentic-rag)
