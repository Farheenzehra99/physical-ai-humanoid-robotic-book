# RAG Chatbot API Usage

This document describes how to interact with the RAG Chatbot API programmatically.

## API Endpoints

### Query Endpoint
- **URL**: `/v1/query`
- **Method**: `POST`
- **Content-Type**: `application/json`

#### Request Body

```json
{
  "text": "Your question here",
  "mode": "NORMAL_MODE",
  "selected_text": "Optional text when using SELECTED_TEXT_MODE",
  "user_id": "Optional user identifier"
}
```

#### Query Modes

1. **NORMAL_MODE**: Query the entire book content
   ```json
   {
     "text": "What is reinforcement learning?",
     "mode": "NORMAL_MODE"
   }
   ```

2. **SELECTED_TEXT_MODE**: Query only the selected text
   ```json
   {
     "text": "What does this paragraph explain?",
     "mode": "SELECTED_TEXT_MODE",
     "selected_text": "Reinforcement learning is a type of machine learning where an agent learns to make decisions..."
   }
   ```

#### Response Format

```json
{
  "answer": "The answer to your question",
  "citations": [
    {
      "chapter": "Chapter 3: Machine Learning Fundamentals",
      "page": 45,
      "chunk_index": 3,
      "text_snippet": "Reinforcement learning is a type of machine learning...",
      "similarity_score": 0.87
    }
  ],
  "confidence_score": 0.92,
  "response_time_ms": 450
}
```

### Health Check Endpoint
- **URL**: `/v1/health`
- **Method**: `GET`

Returns the health status of the RAG Chatbot service.

### Book Import Endpoint
- **URL**: `/v1/book/import`
- **Method**: `POST`
- **Content-Type**: `multipart/form-data`

Upload book content to be indexed for RAG queries.

## Error Handling

The API returns appropriate HTTP status codes:
- `200`: Successful request
- `400`: Invalid request parameters
- `500`: Internal server error

Error responses include a structured error object:
```json
{
  "error": "ERROR_CODE",
  "message": "Human-readable error message"
}
```

## Rate Limiting

API requests are subject to rate limiting to ensure service availability.