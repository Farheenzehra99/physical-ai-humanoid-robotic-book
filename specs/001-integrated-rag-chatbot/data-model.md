# Data Model for Integrated RAG Chatbot

## Core Entities

### Query
- **Fields**:
  - `id`: UUID (primary key)
  - `text`: String (the user's question)
  - `mode`: Enum (NORMAL_MODE | SELECTED_TEXT_MODE)
  - `selected_text`: String? (nullable, for selected text mode)
  - `user_id`: String? (nullable, for user identification)
  - `timestamp`: DateTime (when query was submitted)
  - `metadata`: JSON (additional context)

### Response
- **Fields**:
  - `id`: UUID (primary key)
  - `query_id`: UUID (foreign key to Query)
  - `answer`: String (the response text)
  - `citations`: Array of Citation objects
  - `confidence_score`: Float (0.0-1.0)
  - `timestamp`: DateTime (when response was generated)
  - `response_time_ms`: Integer (processing time)

### Citation
- **Fields**:
  - `id`: UUID (primary key)
  - `response_id`: UUID (foreign key to Response)
  - `chapter`: String (book chapter reference)
  - `page`: Integer (book page number)
  - `chunk_index`: Integer (position in chunk sequence)
  - `text_snippet`: String (quoted text from source)
  - `similarity_score`: Float (0.0-1.0 for RAG results)

### BookChunk
- **Fields**:
  - `id`: UUID (primary key)
  - `content`: String (the text content of the chunk)
  - `chapter`: String (book chapter)
  - `page`: Integer (book page number)
  - `chunk_index`: Integer (position in chapter)
  - `embedding_vector`: Vector (numerical representation for similarity search)
  - `metadata`: JSON (additional chunk information)
  - `created_at`: DateTime

### UserSession
- **Fields**:
  - `id`: UUID (primary key)
  - `user_id`: String
  - `start_time`: DateTime
  - `last_activity`: DateTime
  - `query_history`: Array of Query IDs (recent queries)
  - `preferences`: JSON (user preferences)

## Relationships

- Query → Response (One-to-One)
- Response → Citations (One-to-Many)
- UserSession → Queries (One-to-Many)
- BookChunk → Citations (One-to-Many)

## Validation Rules

### Query
- Text length: 1-1000 characters
- Mode must be one of the defined enum values
- If mode is SELECTED_TEXT_MODE, selected_text must not be null
- Timestamp must be current or past

### Response
- Answer must not be empty
- Confidence score must be between 0.0 and 1.0
- Citations array must not exceed 10 items
- Response time must be positive

### Citation
- Chapter and page must correspond to actual book content
- Similarity score must be between 0.0 and 1.0
- Text snippet must be between 10-500 characters

## State Transitions

### Query States
- PENDING → PROCESSING → COMPLETED
- PENDING → PROCESSING → FAILED (on error)

### UserSession States
- ACTIVE → INACTIVE (after timeout or explicit logout)

## Indexes for Performance

- Query.timestamp (for chronological access)
- BookChunk.embedding_vector (for similarity search)
- Citation.response_id (for response lookup)
- UserSession.user_id (for user identification)