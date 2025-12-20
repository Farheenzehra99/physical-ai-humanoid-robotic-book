# Quickstart: Complete Retrieval Pipeline

**Feature**: 003-complete-retrieval-pipeline
**Purpose**: Get started with the retrieval system in under 5 minutes

## Prerequisites

- Python 3.10 or higher
- Cohere API key ([get one here](https://dashboard.cohere.com))
- Qdrant Cloud account with `book-rag-v1` collection ([setup guide](https://qdrant.tech/documentation/cloud/))
- Git (for cloning repository)

## Installation

### 1. Clone Repository

```bash
git clone <repository-url>
cd Physical_AI_Humanoid_Robotics
git checkout 003-complete-retrieval-pipeline
```

### 2. Install Dependencies

```bash
# Using pip
pip install -r requirements.txt

# Or using uv (recommended)
uv pip install -r requirements.txt
```

**Required packages**:
- `qdrant-client>=1.6.0`
- `cohere>=4.0.0`
- `python-dotenv>=1.0.0`
- `pytest>=7.0.0` (for testing)

### 3. Configure Environment

Create a `.env` file in the project root:

```bash
cp .env.example .env
```

Edit `.env` with your API keys:

```bash
# Cohere API Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Cloud Configuration
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here

# Collection Configuration
COLLECTION_NAME=book-rag-v1
```

## Quick Start Guide

### Basic Retrieval

```python
from retrieval_module import retrieve

# Simple query
results = retrieve("What is Physical AI?")

# Print results
for i, result in enumerate(results, 1):
    print(f"{i}. Score: {result['similarity_score']:.3f}")
    print(f"   Text: {result['text'][:100]}...")
    print(f"   Page: {result['metadata']['page']}")
    print()
```

**Expected Output**:
```
1. Score: 0.856
   Text: Physical AI, also known as Embodied Intelligence, refers to AI systems that interact...
   Page: 10

2. Score: 0.782
   Text: Unlike traditional AI systems that operate purely in digital environments...
   Page: 11
```

### Customized Retrieval

```python
from retrieval_module import retrieve

# Get more results with score threshold
results = retrieve(
    query="How do I initialize a ROS2 node?",
    top_k=10,
    min_score=0.75
)

print(f"Found {len(results)} results above 0.75 similarity")
```

### Filtered Retrieval

```python
from retrieval_module import retrieve

# Search only in specific section
results = retrieve(
    query="URDF file structure",
    top_k=5,
    filters={"section": "ros2-basics"}
)
```

### Running Evaluation

```bash
# Run full test suite
python evaluation/run_evaluation.py

# View results
cat evaluation/test_results.json
```

**Expected Evaluation Output**:
```json
{
  "total_queries": 10,
  "passed": 8,
  "failed": 2,
  "accuracy": 0.80,
  "avg_latency_ms": 523.5,
  "avg_top_score": 0.812
}
```

## Common Use Cases

### 1. Factual Questions

```python
results = retrieve("What is the price of Jetson Orin Nano?")
# Expected: Returns hardware pricing information
```

### 2. Conceptual Questions

```python
results = retrieve("Explain sim-to-real transfer in robotics")
# Expected: Returns conceptual explanations from relevant chapters
```

### 3. Code-Related Questions

```python
results = retrieve("Show me Python code for URDF parsing")
# Expected: Returns code examples and technical documentation
```

### 4. Multi-Topic Questions

```python
results = retrieve("Compare Gazebo and Isaac Sim for robot simulation")
# Expected: Returns information from multiple sections
```

## Troubleshooting

### Problem: Empty Results

**Symptom**: `retrieve()` returns empty list

**Solutions**:
1. Check if collection exists in Qdrant:
   ```python
   from qdrant_client import QdrantClient
   client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
   print(client.get_collections())
   ```
2. Verify collection name is `book-rag-v1`
3. Check if collection has vectors

### Problem: Low Similarity Scores

**Symptom**: All results have scores < 0.5

**Solutions**:
1. Verify query is relevant to book content
2. Try more specific queries
3. Check embedding model matches ingestion model
4. Consider adjusting `min_score` threshold

### Problem: Slow Queries (> 800ms)

**Symptom**: Queries take longer than expected

**Solutions**:
1. Check network latency to Qdrant Cloud
2. Reduce `top_k` value
3. Use filters to narrow search space
4. Monitor Cohere API rate limits

### Problem: Authentication Errors

**Symptom**: "Unauthorized" or "Invalid API key" errors

**Solutions**:
1. Verify `.env` file exists and is loaded
2. Check API keys are correct (no trailing spaces)
3. Ensure Cohere key has embeddings access
4. Verify Qdrant URL includes `https://`

## Testing

### Run Unit Tests

```bash
pytest tests/unit/ -v
```

### Run Integration Tests

```bash
# Requires valid API keys and collection
pytest tests/integration/ -v
```

### Run Full Test Suite

```bash
pytest tests/ -v --cov=retrieval_module
```

## Performance Benchmarks

**Expected Performance** (with real API calls):

| Metric | Target | Typical |
|--------|--------|---------|
| Average Latency | < 800ms | 450-600ms |
| Accuracy (8/10) | ≥ 80% | 85-90% |
| Top-1 Score | ≥ 0.78 | 0.80-0.85 |

**Latency Breakdown**:
- Cohere embedding: ~100-200ms
- Qdrant search: ~50-100ms
- Network overhead: ~100-200ms
- Total: ~250-500ms (well under 800ms target)

## Next Steps

1. **Run Evaluation**: `python evaluation/run_evaluation.py` to validate retrieval quality
2. **Review Results**: Check `evaluation/test_results.json` for detailed metrics
3. **Read Quality Report**: See `retrieval_quality_report.md` for analysis and recommendations
4. **Integration**: Use retrieval module in downstream RAG application

## API Reference

For complete API documentation, see:
- [Data Model](./data-model.md) - Entity definitions
- [API Contract](./contracts/retrieval_api.yaml) - OpenAPI specification
- [Implementation Plan](./plan.md) - Technical details

## Support

- **Issues**: Report bugs or ask questions in GitHub Issues
- **Documentation**: See `specs/003-complete-retrieval-pipeline/` for detailed docs
- **Examples**: Check `evaluation/test_queries.json` for query examples

## License

See repository LICENSE file.
