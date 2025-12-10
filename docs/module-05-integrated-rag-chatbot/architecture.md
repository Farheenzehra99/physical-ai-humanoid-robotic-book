# RAG Chatbot Architecture

This document describes the architecture and components of the Integrated RAG Chatbot system.

## System Overview

The RAG Chatbot is built with a microservice architecture consisting of backend services and frontend components integrated into the Docusaurus documentation site.

## Backend Components

### FastAPI Application
- Main entry point for API requests
- Handles request validation and routing
- Implements security and authentication middleware
- Provides health check endpoints

### RAG Service
- Core logic for processing queries in both modes
- Normal Mode: Uses Qdrant for vector similarity search and Neon for metadata
- Selected Text Mode: Processes only the provided text snippet
- Implements retry logic and fallback responses

### Qdrant Client
- Vector database client for semantic search
- Stores and retrieves book content embeddings
- Provides similarity scoring for retrieved chunks

### Neon Client
- PostgreSQL-compatible database client
- Stores metadata for book chunks (chapter, page, etc.)
- Provides structured information for citations

### Security Service
- Implements encryption for sensitive data
- Provides audit logging of user interactions
- Handles privacy controls

## Frontend Components

### RagChatbot Component
- Main chat interface component
- Manages state and user interactions
- Communicates with backend API

### CitationDisplay Component
- Renders citations with proper formatting
- Links to relevant book sections
- Shows similarity scores

### QueryInput Component
- Handles user input and text selection
- Provides mode switching (Normal vs Selected Text)
- Implements input validation

## Data Flow

### Normal Mode Query
1. User submits question via frontend
2. Frontend sends request to backend `/v1/query`
3. RAG service generates embedding for query
4. Qdrant retrieves top 5 similar chunks
5. Neon provides metadata for citations
6. RAG service generates answer using retrieved context
7. Response with answer and citations sent to frontend

### Selected Text Mode Query
1. User selects text and submits question
2. Frontend sends request with selected text to `/v1/query`
3. RAG service processes only the provided text snippet
4. Response sent to frontend (citations marked as "Selected Text Only")

## Technology Stack

- **Backend**: FastAPI, Python 3.11+
- **Vector DB**: Qdrant
- **Metadata DB**: PostgreSQL (Neon)
- **Embeddings**: Claude MCP
- **Frontend**: React, Docusaurus
- **API**: REST with OpenAPI specification
- **Deployment**: Docker, Docker Compose