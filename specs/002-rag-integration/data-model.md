# Data Model: RAG Integration with Cohere and Qdrant

## Entities

### URL Content
- **url**: string (primary identifier)
- **title**: string (page title)
- **content**: string (cleaned text content)
- **status**: enum ['pending', 'processed', 'failed']
- **created_at**: datetime
- **updated_at**: datetime

### Text Chunk
- **id**: string (unique identifier for the chunk)
- **url**: string (source URL)
- **title**: string (source page title)
- **content**: string (chunked text content, 700-1000 tokens)
- **chunk_index**: integer (position of chunk in original document)
- **total_chunks**: integer (total number of chunks from original document)
- **token_count**: integer (number of tokens in this chunk)
- **created_at**: datetime

### Embedding
- **chunk_id**: string (reference to Text Chunk)
- **embedding**: list[float] (vector representation, 1024 dimensions for Cohere embed-english-v3.0)
- **embedding_model**: string (model name used)
- **created_at**: datetime

### Qdrant Point
- **id**: string (Qdrant point ID)
- **vector**: list[float] (embedding vector)
- **payload**: dict
  - **chunk_id**: string (reference to Text Chunk)
  - **url**: string (source URL)
  - **title**: string (source title)
  - **content**: string (chunk content)
  - **chunk_index**: integer (position in original document)
  - **created_at**: datetime (timestamp)
  - **embedding_model**: string (model used)