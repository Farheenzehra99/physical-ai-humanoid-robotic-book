# Data Model: Frontend Chat Integration

**Feature**: 005-frontend-chat
**Date**: 2025-12-14

## Overview

This document defines the data structures used in the frontend chat widget. All data is managed client-side using React state and browser sessionStorage.

## Entities

### 1. ChatMessage

Represents a single message in the conversation.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| id | string | Yes | Unique identifier (UUID) |
| role | "user" \| "assistant" | Yes | Message sender |
| content | string | Yes | Message text content |
| timestamp | number | Yes | Unix timestamp (ms) |
| citations | Citation[] | No | Source citations (assistant only) |
| selectedContext | string \| null | No | Selected text sent with question (user only) |
| isError | boolean | No | True if this is an error message |

**Example**:
```javascript
{
  id: "msg-123e4567-e89b",
  role: "assistant",
  content: "Physical AI refers to artificial intelligence systems...",
  timestamp: 1702560000000,
  citations: [
    {
      sourceUrl: "https://physical-ai-robotics.dev/docs/intro",
      title: "Introduction to Physical AI",
      chunkId: "chunk_042",
      relevanceScore: 0.92
    }
  ],
  selectedContext: null,
  isError: false
}
```

---

### 2. Citation

Source attribution for a response.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| sourceUrl | string | Yes | URL of the source document |
| title | string | Yes | Document/section title |
| chunkId | string | Yes | Backend chunk identifier |
| relevanceScore | number | Yes | Similarity score (0-1) |

**Validation**:
- `sourceUrl` must be a valid URL
- `relevanceScore` must be between 0 and 1

---

### 3. ChatState

Overall state managed by the ChatWidget component.

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| messages | ChatMessage[] | [] | Conversation history |
| isOpen | boolean | false | Whether chat panel is visible |
| isLoading | boolean | false | Whether a request is in progress |
| selectedText | string | "" | Currently selected page text |
| error | string \| null | null | Current error message |

**State Diagram**:
```
[Closed] --open--> [Open/Idle]
[Open/Idle] --submit--> [Open/Loading]
[Open/Loading] --success--> [Open/Idle]
[Open/Loading] --error--> [Open/Error]
[Open/Error] --dismiss--> [Open/Idle]
[Open/*] --close--> [Closed]
```

---

### 4. ApiRequest

Request payload sent to the backend.

| Field | Type | Required | Constraints |
|-------|------|----------|-------------|
| question | string | Yes | 1-2000 chars, not whitespace-only |
| selected_text | string \| undefined | No | Max 5000 chars |
| top_k | number | No | 1-20, default 5 |

---

### 5. ApiResponse

Response received from the backend.

| Field | Type | Description |
|-------|------|-------------|
| answer | string | Generated answer text |
| citations | BackendCitation[] | Source attributions |
| metadata | ResponseMetadata | Request processing info |

**BackendCitation**:
| Field | Type |
|-------|------|
| source_url | string |
| title | string |
| chunk_id | string |
| relevance_score | number |

**ResponseMetadata**:
| Field | Type |
|-------|------|
| request_id | string |
| processing_time_ms | number |
| retrieval_count | number |
| model_used | string |

---

## Storage Schema

### sessionStorage Key: `chatHistory`

**Format**: JSON array of ChatMessage objects

**Example**:
```json
[
  {
    "id": "msg-user-001",
    "role": "user",
    "content": "What is Physical AI?",
    "timestamp": 1702560000000,
    "selectedContext": null
  },
  {
    "id": "msg-asst-001",
    "role": "assistant",
    "content": "Physical AI refers to...",
    "timestamp": 1702560002500,
    "citations": [...]
  }
]
```

**Lifecycle**:
- Created: On first message sent
- Updated: After each message added
- Cleared: On new browser session or explicit clear action
- Max size: ~5MB (sessionStorage limit)

---

## Data Transformations

### Backend to Frontend Citation Mapping

```javascript
function mapCitation(backendCitation) {
  return {
    sourceUrl: backendCitation.source_url,
    title: backendCitation.title,
    chunkId: backendCitation.chunk_id,
    relevanceScore: backendCitation.relevance_score,
  };
}
```

### Create User Message

```javascript
function createUserMessage(question, selectedText) {
  return {
    id: `msg-user-${Date.now()}`,
    role: 'user',
    content: question,
    timestamp: Date.now(),
    selectedContext: selectedText || null,
    isError: false,
  };
}
```

### Create Assistant Message

```javascript
function createAssistantMessage(apiResponse) {
  return {
    id: `msg-asst-${Date.now()}`,
    role: 'assistant',
    content: apiResponse.answer,
    timestamp: Date.now(),
    citations: apiResponse.citations.map(mapCitation),
    isError: false,
  };
}
```

### Create Error Message

```javascript
function createErrorMessage(errorText) {
  return {
    id: `msg-error-${Date.now()}`,
    role: 'assistant',
    content: errorText,
    timestamp: Date.now(),
    citations: [],
    isError: true,
  };
}
```

---

## Validation Rules

### Question Validation (Client-Side)

```javascript
function validateQuestion(question) {
  const trimmed = question.trim();

  if (!trimmed) {
    return { valid: false, error: 'Please enter a question' };
  }

  if (trimmed.length > 2000) {
    return { valid: false, error: 'Question is too long (max 2000 characters)' };
  }

  return { valid: true, error: null };
}
```

### Selected Text Validation

```javascript
function validateSelectedText(text) {
  if (!text) return null;

  const trimmed = text.trim();
  if (trimmed.length > 5000) {
    return trimmed.substring(0, 5000);
  }

  return trimmed;
}
```

---

## Type Definitions (JSDoc)

```javascript
/**
 * @typedef {Object} Citation
 * @property {string} sourceUrl
 * @property {string} title
 * @property {string} chunkId
 * @property {number} relevanceScore
 */

/**
 * @typedef {Object} ChatMessage
 * @property {string} id
 * @property {'user' | 'assistant'} role
 * @property {string} content
 * @property {number} timestamp
 * @property {Citation[]} [citations]
 * @property {string | null} [selectedContext]
 * @property {boolean} [isError]
 */

/**
 * @typedef {Object} ChatState
 * @property {ChatMessage[]} messages
 * @property {boolean} isOpen
 * @property {boolean} isLoading
 * @property {string} selectedText
 * @property {string | null} error
 */
```
