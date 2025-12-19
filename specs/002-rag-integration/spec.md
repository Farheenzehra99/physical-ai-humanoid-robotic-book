# Feature Specification: RAG Integration with Cohere and Qdrant

## Overview
Initialize a project with uv package manager in the backend folder to set up Cohere and Qdrant clients for RAG (Retrieval-Augmented Generation) functionality. The system will fetch, clean, and chunk text from deployed URLs, generate embeddings using Cohere, and upsert them into Qdrant with metadata.

## Deployed URL
- Production URL: https://physical-ai-humanoid-robotic-book-ten.vercel.app/

## Functional Requirements

### 1. Package Management
- Use uv package manager for the backend project
- Initialize project in the backend folder
- Manage dependencies for Cohere, Qdrant, and web scraping

### 2. URL Processing
- Fetch all URLs from the deployed site
- Extract text content from each URL
- Clean and preprocess the extracted text

### 3. Text Processing
- Chunk text into appropriate sizes for embedding
- Preserve semantic boundaries during chunking
- Maintain metadata for each chunk

### 4. Embedding Generation
- Use Cohere's embedding API to generate vector representations
- Properly handle API rate limits and errors
- Store embeddings with associated metadata

### 5. Vector Storage
- Create Qdrant collection for storing embeddings
- Upsert embeddings with metadata to Qdrant
- Implement proper error handling and retry logic

## Technical Requirements

### 1. Dependencies
- Cohere Python client
- Qdrant Python client
- BeautifulSoup4 for HTML parsing
- Requests for HTTP operations
- uv for package management

### 2. Functions Required
- `get_all_urls`: Fetch all URLs from the deployed site
- `extract_text_from_url`: Extract and clean text from a single URL
- `embed`: Generate embeddings using Cohere
- `create_collection`: Create Qdrant collection
- `rag_embedding`: Main embedding function
- `save_chunks_to_qdrant`: Save chunks to Qdrant with metadata
- `main`: Execute the complete pipeline

### 3. Data Flow
1. Fetch all URLs from the deployed site
2. Extract and clean text from each URL
3. Chunk the text appropriately
4. Generate embeddings using Cohere
5. Store embeddings in Qdrant with metadata

## Success Criteria
- All URLs from the deployed site are processed
- Text is properly extracted and cleaned
- Embeddings are generated without errors
- All embeddings are successfully stored in Qdrant
- Metadata is preserved and accessible
- The system handles errors gracefully

## Constraints
- All functionality must be in a single main.py file
- Use uv package manager for dependency management
- Follow Python best practices for error handling
- Respect API rate limits for external services