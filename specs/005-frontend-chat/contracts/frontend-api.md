# API Contract: Frontend Chat Integration

**Feature**: 005-frontend-chat
**Date**: 2025-12-14
**Backend Spec**: Spec-004 (RAG Agent-Based Question Answering API)

## Overview

This document defines the contract between the frontend chat widget and the backend RAG API. The backend API is already implemented per Spec-004.

## Base URL

| Environment | URL |
|-------------|-----|
| Local Development | `http://localhost:8000` |
| Production | `https://api.physical-ai-robotics.dev` |

Configuration via `CHAT_API_URL` environment variable or Docusaurus `customFields.chatApiUrl`.

---

## Endpoints

### POST /chat

Submit a question and receive a grounded answer with citations.

#### Request

**Headers**:
```
Content-Type: application/json
```

**Body**:
```json
{
  "question": "string",
  "selected_text": "string | null",
  "top_k": "integer"
}
```

| Field | Type | Required | Constraints | Description |
|-------|------|----------|-------------|-------------|
| question | string | Yes | 1-2000 chars, non-empty | User's question |
| selected_text | string | No | Max 5000 chars | Text selected on page for context |
| top_k | integer | No | 1-20, default 5 | Number of retrieval results |

#### Response (200 OK)

```json
{
  "answer": "string",
  "citations": [
    {
      "source_url": "string",
      "title": "string",
      "chunk_id": "string",
      "relevance_score": 0.92
    }
  ],
  "metadata": {
    "request_id": "string",
    "processing_time_ms": 423.5,
    "retrieval_count": 5,
    "model_used": "gpt-4o"
  }
}
```

| Field | Type | Description |
|-------|------|-------------|
| answer | string | AI-generated answer grounded in retrieved context |
| citations | Citation[] | Source attributions for factual claims |
| citations[].source_url | string | URL to source document |
| citations[].title | string | Document/section title |
| citations[].chunk_id | string | Internal chunk identifier |
| citations[].relevance_score | number | Similarity score (0.0-1.0) |
| metadata.request_id | string | UUID for request tracking |
| metadata.processing_time_ms | number | Server processing time |
| metadata.retrieval_count | number | Number of chunks retrieved |
| metadata.model_used | string | LLM model identifier |

---

## Error Responses

### 422 Validation Error

Invalid request parameters.

```json
{
  "detail": [
    {
      "type": "string_too_short",
      "loc": ["body", "question"],
      "msg": "String should have at least 1 character",
      "input": ""
    }
  ]
}
```

**Frontend Handling**: Display "Please enter a question"

---

### 429 Rate Limit Exceeded

Too many requests from this client.

```json
{
  "detail": "Rate limit exceeded. Please wait before retrying."
}
```

**Frontend Handling**: Display "Please wait a moment before sending another question."

---

### 503 Service Unavailable

Backend dependency (OpenAI, Qdrant, Cohere) is unavailable.

```json
{
  "detail": "AI service temporarily unavailable. Please try again later.",
  "service": "openai",
  "retry_after": 30
}
```

**Frontend Handling**: Display the detail message, enable retry button.

---

### 504 Gateway Timeout

Request took too long to process.

```json
{
  "detail": "Request timed out. Please try again."
}
```

**Frontend Handling**: Display timeout message, enable retry button.

---

## Request Examples

### Basic Question

```javascript
const response = await fetch('/chat', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    question: 'What is Physical AI?'
  })
});
```

### Question with Selected Text

```javascript
const response = await fetch('/chat', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    question: 'Can you explain this concept?',
    selected_text: 'Physical AI refers to artificial intelligence systems...',
    top_k: 8
  })
});
```

---

## Frontend Implementation

### API Client

```javascript
// src/services/chatApi.js

const TIMEOUT_MS = 30000;

/**
 * Send a chat message to the backend API.
 * @param {string} question - User's question (1-2000 chars)
 * @param {string|null} selectedText - Optional selected text context
 * @param {number} topK - Number of retrieval results (default: 5)
 * @returns {Promise<ChatResponse>} API response
 * @throws {ApiError} On network or HTTP error
 */
export async function sendChatMessage(question, selectedText = null, topK = 5) {
  const apiUrl = getApiUrl();
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), TIMEOUT_MS);

  try {
    const body = { question, top_k: topK };
    if (selectedText) {
      body.selected_text = selectedText;
    }

    const response = await fetch(`${apiUrl}/chat`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(body),
      signal: controller.signal,
    });

    clearTimeout(timeoutId);

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new ApiError(
        response.status,
        getErrorMessage(response.status, errorData)
      );
    }

    return await response.json();
  } catch (error) {
    clearTimeout(timeoutId);

    if (error.name === 'AbortError') {
      throw new ApiError(504, 'Request timed out. Please try again.');
    }
    if (error instanceof ApiError) {
      throw error;
    }
    throw new ApiError(0, 'Unable to connect. Please check your internet connection.');
  }
}

function getApiUrl() {
  // Get from Docusaurus config or fallback
  if (typeof window !== 'undefined' && window.__DOCUSAURUS__) {
    return window.__DOCUSAURUS__.siteConfig.customFields.chatApiUrl;
  }
  return process.env.CHAT_API_URL || 'http://localhost:8000';
}

function getErrorMessage(status, errorData) {
  const detail = errorData.detail;

  switch (status) {
    case 422:
      if (Array.isArray(detail)) {
        return 'Please enter a valid question.';
      }
      return detail || 'Invalid request.';
    case 429:
      return 'Please wait a moment before sending another question.';
    case 503:
      return detail || 'Service temporarily unavailable. Please try again later.';
    case 504:
      return 'Request timed out. Please try again.';
    default:
      return detail || 'An unexpected error occurred. Please try again.';
  }
}

export class ApiError extends Error {
  constructor(status, message) {
    super(message);
    this.status = status;
    this.name = 'ApiError';
  }
}
```

---

## CORS Requirements

The backend MUST include CORS headers allowing the frontend domain:

```python
# backend/src/api/app.py
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "https://physical-ai-robotics.dev",
        "https://physical-ai-humanoid-robotic-book-ten.vercel.app"
    ],
    allow_credentials=False,
    allow_methods=["POST", "OPTIONS"],
    allow_headers=["Content-Type"],
)
```

---

## Testing Checklist

| Scenario | Expected Behavior |
|----------|-------------------|
| Valid question | 200 with answer and citations |
| Empty question | 422 validation error |
| Question >2000 chars | 422 validation error |
| Question with selected_text | 200, response references selected context |
| Backend timeout | 504 or client timeout error |
| Backend down | 503 or network error |
| Malformed JSON | 422 validation error |
| Missing Content-Type header | 422 or server error |

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-12-14 | Initial contract based on Spec-004 |
