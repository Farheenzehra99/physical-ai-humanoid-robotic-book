# Research: Complete Retrieval Pipeline with Testing

**Feature**: 003-complete-retrieval-pipeline
**Date**: 2025-12-12
**Purpose**: Document technology decisions and research findings for retrieval pipeline implementation

## Technology Decisions

### 1. Embedding Model Selection

**Decision**: Use Cohere embed-english-v3.0 with `input_type="search_query"` for query embedding

**Rationale**:
- Specified in requirements for consistency with document ingestion
- embed-english-v3.0 produces 1024-dimensional vectors optimized for semantic search
- `input_type="search_query"` parameter ensures query embeddings are optimized for retrieval (vs document embeddings with `input_type="search_document"`)
- Cohere provides excellent semantic understanding for technical/code content
- Already in use for document ingestion pipeline (maintains consistency)

**Alternatives Considered**:
- OpenAI text-embedding-ada-002: Good quality but different vendor, would require dual API key management
- Sentence Transformers (local): No API costs but requires infrastructure and model management
- Hugging Face models: More customization but adds complexity for deployment

**Impact**: Requires COHERE_API_KEY environment variable, API calls add ~100-200ms latency per query

---

### 2. Vector Database Choice

**Decision**: Use Qdrant Cloud with collection name `book-rag-v1` and cosine similarity

**Rationale**:
- Already specified in requirements (collection exists from prior ingestion)
- Qdrant provides excellent performance for similarity search
- Cosine similarity is standard for normalized embeddings
- Cloud deployment removes infrastructure management overhead
- Strong metadata filtering capabilities for page/section queries

**Alternatives Considered**:
- Pinecone: Similar capabilities but different API, higher costs at scale
- Weaviate: Strong features but more complex setup
- ChromaDB: Good for local development but less production-ready

**Impact**: Requires QDRANT_URL and QDRANT_API_KEY, collection must exist before retrieval

---

### 3. Metadata Filtering Strategy

**Decision**: Implement Qdrant filter builder using `must` and `should` clauses for page/section filtering

**Rationale**:
- Qdrant native filtering is performant (filter + vector search in single query)
- `must` clauses for hard requirements (exact page match)
- `should` clauses for soft preferences (related sections)
- Maintains sub-800ms latency target even with filters

**Alternatives Considered**:
- Post-processing filters (filter after retrieval): Wastes API calls, less efficient
- Pre-filtering (separate query per filter): Multiple round trips, higher latency
- Hybrid approach: Unnecessary complexity for this use case

**Implementation Pattern**:
```python
filters = {
    "must": [
        {"key": "page", "match": {"value": 42}}
    ],
    "should": [
        {"key": "section", "match": {"value": "intro"}}
    ]
}
```

---

### 4. Evaluation Framework Design

**Decision**: JSON-based test queries with automated accuracy scoring and latency tracking

**Rationale**:
- JSON format is human-readable and version-controllable
- Each test query includes expected section, keywords, and query type
- Automated runner provides consistent, reproducible results
- Latency tracking ensures performance requirements are met
- Pass/fail criteria: relevant chunk in top results with score ≥ 0.78

**Alternatives Considered**:
- YAML format: More verbose, less tooling support
- Database storage: Overkill for 10 queries, adds complexity
- Manual testing: Not reproducible, time-consuming

**Test Query Structure**:
```json
{
  "id": 1,
  "query": "What is Physical AI?",
  "type": "conceptual",
  "expected_section": "intro",
  "expected_keywords": ["embodied", "intelligence", "robotics"],
  "min_score": 0.78
}
```

---

### 5. Project Structure Pattern

**Decision**: `retrieval_module/` for core logic, `evaluation/` for test framework, `tests/` for unit/integration tests

**Rationale**:
- Clean separation of concerns: retrieval logic vs evaluation vs testing
- `retrieval_module/` is importable and reusable for downstream RAG integration
- `evaluation/` contains domain-specific test queries and runners
- `tests/` contains pytest-based unit and integration tests
- Aligns with Python packaging best practices

**Alternatives Considered**:
- Single `src/` directory: Less clear separation between retrieval and evaluation
- Monolithic script: Not reusable, harder to test
- Package-per-component: Overkill for this scope

**Directory Hierarchy**:
```
retrieval_module/
  ├── __init__.py (exports main retrieve() function)
  ├── config.py
  ├── embedding.py
  ├── search.py
  ├── metadata_filtering.py
  └── retrieval.py

evaluation/
  ├── test_queries.json
  ├── run_evaluation.py
  ├── logs/
  └── test_results.json

tests/
  ├── unit/
  ├── integration/
  └── conftest.py
```

---

### 6. Performance Optimization Strategy

**Decision**: Single API call per query (embed + search), caching for repeated queries (optional), async execution for batch evaluation

**Rationale**:
- Target: <800ms average latency
- Cohere embedding: ~100-200ms
- Qdrant search: ~50-100ms
- Network overhead: ~100-200ms
- Total: ~250-500ms (well under target)
- Batch evaluation uses async/await for parallel query processing

**Alternatives Considered**:
- Eager caching: Adds complexity, not needed for evaluation workload
- Connection pooling: Marginal gains for single-user evaluation script
- Pre-computed embeddings: Only helps for repeated queries, not general case

**Implementation Notes**:
- Use `asyncio` for batch evaluation to parallelize I/O
- Log per-query latency for performance regression detection
- Monitor API rate limits (Cohere: 10,000 calls/month free tier)

---

### 7. Testing Strategy

**Decision**: Pytest for unit/integration tests, custom evaluation runner for quality validation

**Rationale**:
- Pytest is industry standard for Python testing
- Unit tests validate individual components (embedding, search, filtering)
- Integration tests validate end-to-end flow
- Custom evaluation runner focuses on retrieval quality (accuracy, relevance)
- Separation allows both component testing and quality validation

**Test Coverage**:
- Unit: embedding.py, search.py, metadata_filtering.py
- Integration: Full retrieve() flow with mock Qdrant responses
- Evaluation: 10 real queries against actual book-rag-v1 collection

---

### 8. Error Handling Approach

**Decision**: Explicit error types, graceful degradation, comprehensive logging

**Rationale**:
- Retrieval failures should not crash the system
- Clear error messages for debugging (API key issues, collection not found, etc.)
- Logging provides audit trail for quality analysis
- Partial results better than no results (return what's available)

**Error Scenarios**:
1. **Cohere API failure**: Log error, return empty results, suggest retry
2. **Qdrant connection failure**: Log error, check collection existence, suggest configuration fix
3. **Low-score results**: Log warning, return results anyway (let caller decide threshold)
4. **Metadata filter mismatch**: Return best available results, log filter effectiveness

---

## Research Findings Summary

| Decision Area | Choice | Key Benefit |
|---------------|--------|-------------|
| Embedding Model | Cohere embed-english-v3.0 | Consistency with ingestion, semantic quality |
| Vector DB | Qdrant Cloud (book-rag-v1) | Performance, metadata filtering |
| Filtering Strategy | Qdrant native filters | Single query, low latency |
| Evaluation Framework | JSON test queries | Reproducible, version-controlled |
| Project Structure | retrieval_module/ + evaluation/ | Clean separation, reusability |
| Performance | Async batch processing | Meets <800ms target |
| Testing | Pytest + custom runner | Comprehensive coverage |
| Error Handling | Explicit types + logging | Debuggability, reliability |

---

## Dependencies

### Core Dependencies

```txt
qdrant-client>=1.6.0    # Vector database client
cohere>=4.0.0           # Embedding generation
python-dotenv>=1.0.0    # Environment configuration
```

### Development Dependencies

```txt
pytest>=7.0.0           # Testing framework
pytest-asyncio>=0.21.0  # Async test support
pytest-cov>=4.0.0       # Coverage reporting
black>=23.0.0           # Code formatting
ruff>=0.1.0             # Linting
```

---

## Next Steps

1. ✅ Research complete - all decisions documented
2. ⏭️ **Phase 1**: Create data-model.md, contracts/, quickstart.md
3. ⏭️ **Phase 2**: Generate tasks.md with detailed implementation steps
