# Data Model: RAG Agent-Based Question Answering API

**Feature**: 004-rag-agent-api | **Date**: 2025-12-13 | **Phase**: 1 - Design

## Overview

This document defines the data entities, their relationships, validation rules, and state transitions for the RAG Agent API.

---

## Entities

### 1. ChatRequest

**Description**: Incoming request from client containing user question and optional context.

**Fields**:
| Field | Type | Required | Constraints | Description |
|-------|------|----------|-------------|-------------|
| question | string | Yes | 1-2000 chars, non-whitespace | User question about the book |
| selected_text | string | No | 0-5000 chars | User-selected text from book for focused answers |
| top_k | integer | No | 1-20, default: 5 | Number of retrieval results to use |

**Validation Rules**:
- `question` must contain at least one non-whitespace character
- `question` length must not exceed 2000 characters
- `selected_text` if provided and non-empty, length must not exceed 5000 characters
- `top_k` must be between 1 and 20 (inclusive)

**Pydantic Model**:
```python
from pydantic import BaseModel, Field, field_validator

class ChatRequest(BaseModel):
    question: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="User question about the book"
    )
    selected_text: str | None = Field(
        default=None,
        max_length=5000,
        description="Optional selected text from book for focused answers"
    )
    top_k: int = Field(
        default=5,
        ge=1,
        le=20,
        description="Number of retrieval results to use"
    )

    @field_validator("question")
    @classmethod
    def validate_question_not_whitespace(cls, v: str) -> str:
        if not v.strip():
            raise ValueError("Question cannot be empty or whitespace-only")
        return v.strip()
```

---

### 2. ChatResponse

**Description**: Response containing the grounded answer and source citations.

**Fields**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| answer | string | Yes | Generated answer grounded in retrieved context |
| citations | Citation[] | Yes | List of sources used in the answer |
| metadata | ResponseMetadata | Yes | Request processing metadata |

**Pydantic Model**:
```python
class ChatResponse(BaseModel):
    answer: str = Field(
        ...,
        description="Generated answer grounded in retrieved context"
    )
    citations: list[Citation] = Field(
        default_factory=list,
        description="List of sources used in the answer"
    )
    metadata: ResponseMetadata = Field(
        ...,
        description="Request processing metadata"
    )
```

---

### 3. Citation

**Description**: Source attribution for factual claims in the response.

**Fields**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| source_url | string | Yes | URL of the source document |
| title | string | Yes | Title of the document/section |
| chunk_id | string | Yes | Unique identifier for the chunk |
| relevance_score | float | Yes | Similarity score (0-1) |

**Pydantic Model**:
```python
class Citation(BaseModel):
    source_url: str = Field(..., description="URL of the source document")
    title: str = Field(..., description="Title of the document/section")
    chunk_id: str = Field(..., description="Unique identifier for the chunk")
    relevance_score: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Similarity score (0-1)"
    )

    @classmethod
    def from_retrieval_result(cls, result: dict) -> "Citation":
        """Create Citation from retrieval result dictionary."""
        return cls(
            source_url=result["source_url"],
            title=result["title"],
            chunk_id=result["chunk_id"],
            relevance_score=result["score"]
        )
```

---

### 4. ResponseMetadata

**Description**: Processing information for debugging and monitoring.

**Fields**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| request_id | string | Yes | Unique identifier for the request |
| processing_time_ms | float | Yes | Total processing time in milliseconds |
| retrieval_count | integer | Yes | Number of chunks retrieved |
| model_used | string | Yes | LLM model used for generation |

**Pydantic Model**:
```python
import uuid

class ResponseMetadata(BaseModel):
    request_id: str = Field(
        default_factory=lambda: str(uuid.uuid4()),
        description="Unique identifier for the request"
    )
    processing_time_ms: float = Field(
        ...,
        ge=0,
        description="Total processing time in milliseconds"
    )
    retrieval_count: int = Field(
        ...,
        ge=0,
        description="Number of chunks retrieved"
    )
    model_used: str = Field(
        ...,
        description="LLM model used for generation"
    )
```

---

### 5. ErrorResponse

**Description**: Standardized error response format.

**Fields**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| error | string | Yes | Error code/type |
| message | string | Yes | Human-readable error message |
| details | dict | No | Additional error context |
| retry_after | integer | No | Seconds to wait before retry (rate limits) |

**Pydantic Model**:
```python
class ErrorResponse(BaseModel):
    error: str = Field(..., description="Error code/type")
    message: str = Field(..., description="Human-readable error message")
    details: dict | None = Field(
        default=None,
        description="Additional error context"
    )
    retry_after: int | None = Field(
        default=None,
        ge=0,
        description="Seconds to wait before retry"
    )
```

---

### 6. RetrievalContext (Internal)

**Description**: Internal entity representing the context passed to the agent.

**Fields**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| chunks | RetrievalResult[] | Yes | Retrieved document chunks |
| selected_text | string | No | User-selected text if provided |
| query | string | Yes | Original search query |

**Relationship**: Uses `RetrievalResult` from existing `scripts/retrieval/models.py`

---

## Entity Relationships

```
┌─────────────────┐
│  ChatRequest    │
│  - question     │
│  - selected_text│
│  - top_k        │
└────────┬────────┘
         │
         ▼ (1:1)
┌─────────────────┐
│RetrievalContext │◄──── scripts/retrieval/retrieve()
│  - chunks[]     │
│  - query        │
└────────┬────────┘
         │
         ▼ (1:1)
┌─────────────────┐       ┌─────────────────┐
│  ChatResponse   │       │  ErrorResponse  │
│  - answer       │  OR   │  - error        │
│  - citations[]  │       │  - message      │
│  - metadata     │       │  - details      │
└────────┬────────┘       └─────────────────┘
         │
         ▼ (1:N)
┌─────────────────┐
│    Citation     │
│  - source_url   │
│  - title        │
│  - chunk_id     │
│  - score        │
└─────────────────┘
```

---

## State Transitions

### Request Processing Flow

```
[Received] → [Validated] → [Retrieving] → [Generating] → [Formatting] → [Completed]
     │            │              │              │              │
     └──[Invalid]─┴──[Error]─────┴──[Error]─────┴──[Error]─────┴──[Error]
```

**States**:
1. **Received**: Request received by endpoint
2. **Validated**: Pydantic validation passed
3. **Retrieving**: Executing vector search via retrieval pipeline
4. **Generating**: Agent processing with LLM
5. **Formatting**: Building response with citations
6. **Completed**: Response returned to client
7. **Invalid**: Validation failure (422)
8. **Error**: Service failure (503/504)

---

## Validation Summary

| Entity | Field | Rule | Error Code |
|--------|-------|------|------------|
| ChatRequest | question | Non-empty, max 2000 chars | 422 |
| ChatRequest | selected_text | Max 5000 chars | 422 |
| ChatRequest | top_k | 1-20 range | 422 |
| Citation | relevance_score | 0.0-1.0 range | Internal |
| ResponseMetadata | processing_time_ms | >= 0 | Internal |

---

## Compatibility Notes

### Existing Retrieval Module Integration

The existing `scripts/retrieval/models.py` defines:
- `RetrievalResult`: text, source_url, title, chunk_id, score
- `RetrievalQuery`: query, top_k, min_score

**No changes required** to existing models. New API models wrap and extend these for HTTP layer.

### Type Mapping

| Retrieval Module | API Model |
|------------------|-----------|
| RetrievalResult.source_url | Citation.source_url |
| RetrievalResult.title | Citation.title |
| RetrievalResult.chunk_id | Citation.chunk_id |
| RetrievalResult.score | Citation.relevance_score |
