# Quickstart: RAG Agent-Based Question Answering API

**Feature**: 004-rag-agent-api | **Date**: 2025-12-13

## Prerequisites

Before you begin, ensure you have:

- Python 3.10 or higher
- Access to the following API keys:
  - OpenAI API key (for agent reasoning)
  - Cohere API key (existing, for embeddings)
  - Qdrant API key (existing, for vector search)
- The existing retrieval pipeline from Spec-003 working

## Installation

### 1. Clone and Navigate

```bash
cd Physical_AI_Humanoid_Robotics
git checkout 004-rag-agent-api
```

### 2. Install Dependencies

```bash
cd backend
uv pip install -e ".[dev]"
```

Or with pip:
```bash
pip install fastapi uvicorn openai-agents pydantic python-dotenv
```

### 3. Configure Environment Variables

Copy the example and fill in your keys:

```bash
cp .env.example .env
```

Edit `.env`:
```bash
# Existing keys (from Spec-003)
COHERE_API_KEY=your_cohere_key
QDRANT_URL=https://your-cluster.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_key

# New key for this feature
OPENAI_API_KEY=your_openai_key
```

## Running the API

### Development Mode

```bash
cd backend
uvicorn src.api.app:app --reload --port 8000
```

### Production Mode

```bash
uvicorn src.api.app:app --host 0.0.0.0 --port 8000 --workers 4
```

## Quick Test

### Health Check

```bash
curl http://localhost:8000/api/v1/health
```

Expected response:
```json
{
  "status": "healthy",
  "version": "1.0.0",
  "dependencies": {
    "qdrant": "connected",
    "cohere": "connected",
    "openai": "connected"
  }
}
```

### Ask a Question

```bash
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{"question": "What is Physical AI?"}'
```

Expected response:
```json
{
  "answer": "Physical AI refers to artificial intelligence systems...",
  "citations": [
    {
      "source_url": "https://physical-ai-book.com/chapter-1",
      "title": "Chapter 1: Introduction to Physical AI",
      "chunk_id": "chunk_042",
      "relevance_score": 0.92
    }
  ],
  "metadata": {
    "request_id": "550e8400-e29b-41d4-a716-446655440000",
    "processing_time_ms": 423.5,
    "retrieval_count": 5,
    "model_used": "gpt-4o"
  }
}
```

### Ask with Selected Text

```bash
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{
    "question": "Can you explain this in more detail?",
    "selected_text": "Sim-to-real transfer involves training policies in simulation and deploying them to physical robots.",
    "top_k": 8
  }'
```

## Running Tests

### Unit Tests

```bash
pytest tests/api/ -v
```

### Integration Tests (requires API keys)

```bash
pytest tests/api/ -v --run-integration
```

### End-to-End Tests (live APIs)

```bash
pytest tests/api/ -v --run-e2e
```

## API Reference

### POST /api/v1/chat

Ask a question about the book.

**Request Body**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| question | string | Yes | Your question (1-2000 chars) |
| selected_text | string | No | Selected text for context (0-5000 chars) |
| top_k | integer | No | Number of results (1-20, default: 5) |

**Response**:
| Field | Type | Description |
|-------|------|-------------|
| answer | string | Grounded answer with citations |
| citations | array | Sources used in the answer |
| metadata | object | Processing information |

### GET /api/v1/health

Check API health status.

**Response**:
| Field | Type | Description |
|-------|------|-------------|
| status | string | "healthy" or "unhealthy" |
| version | string | API version |
| dependencies | object | Status of external services |

## Error Handling

| Status Code | Error Type | Description |
|-------------|------------|-------------|
| 422 | validation_error | Invalid request (empty question, too long, etc.) |
| 429 | rate_limit_exceeded | Too many requests |
| 503 | service_unavailable | External service down |
| 504 | timeout | Request took too long |

## Troubleshooting

### "Embedding service unavailable"

Check your `COHERE_API_KEY` is valid and has quota remaining.

### "Vector store unavailable"

Verify `QDRANT_URL` and `QDRANT_API_KEY` are correct. Ensure the `docusaurus_chunks` collection exists.

### "AI service unavailable"

Check your `OPENAI_API_KEY` is valid and has credits.

### Slow responses (>800ms)

- Reduce `top_k` parameter
- Check network latency to API endpoints
- Consider caching frequent queries

## Next Steps

1. **Frontend Integration**: See the Docusaurus chatbot component documentation
2. **Monitoring**: Set up logging aggregation for production
3. **Scaling**: Deploy to cloud with load balancing for high traffic
