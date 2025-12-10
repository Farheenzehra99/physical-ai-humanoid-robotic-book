# Quickstart Guide: Integrated RAG Chatbot

## Prerequisites

- Python 3.11+
- Node.js 18+ (for Docusaurus integration)
- Docker and Docker Compose
- Qdrant instance (local or remote)
- Neon PostgreSQL database

## Environment Setup

1. Clone the repository:
```bash
git clone https://github.com/your-org/physical-ai-humanoid-robotics.git
cd physical-ai-humanoid-robotics
```

2. Set up Python virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

3. Install Node.js dependencies:
```bash
npm install
```

4. Set up environment variables:
```bash
cp .env.example .env
# Edit .env with your specific configurations
```

## Running the Service

### Local Development

1. Start external services (Qdrant and Neon):
```bash
docker-compose up -d
```

2. Run the backend service:
```bash
cd backend
uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000
```

3. Run the frontend (Docusaurus):
```bash
cd frontend  # or back to root if using Docusaurus directly
npm run start
```

### Production Deployment

1. Build Docker images:
```bash
docker build -t rag-chatbot-backend -f backend/Dockerfile .
docker build -t rag-chatbot-frontend -f frontend/Dockerfile .
```

2. Deploy with Docker Compose:
```bash
docker-compose -f docker-compose.prod.yml up -d
```

## API Usage

### Submit a Query

```bash
curl -X POST http://localhost:8000/v1/query \
  -H "Content-Type: application/json" \
  -d '{
    "text": "What is the main concept behind reinforcement learning?",
    "mode": "NORMAL_MODE"
  }'
```

### Submit a Query with Selected Text

```bash
curl -X POST http://localhost:8000/v1/query \
  -H "Content-Type: application/json" \
  -d '{
    "text": "What does this paragraph explain?",
    "mode": "SELECTED_TEXT_MODE",
    "selected_text": "Reinforcement learning is a type of machine learning where an agent learns to make decisions by taking actions in an environment to maximize cumulative reward."
  }'
```

### Import Book Content

```bash
curl -X POST http://localhost:8000/v1/book/import \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -F "file=@book_content.pdf" \
  -F "metadata={\"title\":\"Robotics Manual\",\"author\":\"Jane Doe\",\"isbn\":\"978-1234567890\"}"
```

## Configuration

### Environment Variables

- `QDRANT_URL`: URL for Qdrant vector database
- `QDRANT_API_KEY`: API key for Qdrant (if required)
- `NEON_DATABASE_URL`: Connection string for Neon PostgreSQL
- `EMBEDDING_MODEL`: Claude MCP embedding model identifier
- `MAX_QUERY_LENGTH`: Maximum length of user queries (default: 1000)
- `CHUNK_SIZE`: Size of text chunks for RAG (default: 500 tokens)
- `CHUNK_OVERLAP`: Overlap between chunks (default: 50 tokens)
- `MAX_RETRIES`: Number of retries for external service calls (default: 3)

### Health Checks

Monitor service health at:
- `GET /v1/health` - Overall service health
- `GET /v1/health/dependencies` - Individual dependency health

## Development

### Adding New Features

1. Create feature branch:
```bash
git checkout -b feature/new-feature-name
```

2. Run tests before committing:
```bash
pytest
npm run test
```

3. Format code:
```bash
black .
npm run format
```

### Testing

- Unit tests: `pytest tests/unit/`
- Integration tests: `pytest tests/integration/`
- Contract tests: `pytest tests/contract/`

### Logging

The service logs to stdout in JSON format. Set `LOG_LEVEL` environment variable to control verbosity (DEBUG, INFO, WARNING, ERROR).

## Troubleshooting

### Common Issues

1. **Qdrant connection timeout**: Verify Qdrant service is running and URL is correct
2. **Embedding generation fails**: Check Claude MCP configuration and API key validity
3. **Slow response times**: Monitor Qdrant and Neon performance metrics
4. **Memory issues**: Adjust CHUNK_SIZE and worker processes based on available memory

### Monitoring

Monitor these key metrics:
- Average response time
- Error rates
- External service call success rates
- Memory and CPU usage
- Vector database query performance