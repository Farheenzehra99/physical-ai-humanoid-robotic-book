# Research: RAG Integration with Cohere and Qdrant

## Decision: Use uv package manager
**Rationale**: uv is a fast Python package installer and resolver, written in Rust. It's significantly faster than pip and is becoming the new standard for Python dependency management. It's ideal for this project as it provides fast dependency resolution and installation.

**Alternatives considered**:
- pip + requirements.txt: Traditional approach but slower
- Poetry: More feature-rich but potentially overkill for this simple project
- Conda: Better for data science environments but not necessary here

## Decision: Single main.py file structure
**Rationale**: As specified in the requirements, all functionality must be in a single main.py file. This simplifies deployment and understanding of the complete pipeline in one place.

**Alternatives considered**:
- Modular approach with separate files: More maintainable but doesn't meet the single file requirement

## Decision: Web scraping approach for URL content extraction
**Rationale**: The deployed site (https://physical-ai-humanoid-robotic-book-ten.vercel.app/) will need to be scraped to extract text content. BeautifulSoup4 with requests is the standard approach for this in Python.

**Alternatives considered**:
- Using a headless browser (Selenium/Playwright): More robust for JavaScript-heavy sites but slower
- Direct access to source files: Not available since we're working with deployed URLs

## Decision: Cohere embed-english-v3.0 model
**Rationale**: Cohere's embed-english-v3.0 is one of the leading embedding models for English text, with good performance for semantic search and RAG applications.

**Alternatives considered**:
- OpenAI embeddings: Would require different API key and potentially different handling
- Local embedding models (Sentence Transformers): Would require more compute resources but no API costs

## Decision: Qdrant for vector storage
**Rationale**: Qdrant is a high-performance vector database with excellent Python client support. It offers cloud hosting with a free tier that meets the requirements.

**Alternatives considered**:
- Pinecone: Popular but potentially more expensive
- Weaviate: Good alternative but Cohere integration is slightly different
- Chroma: Open source but self-hosting required

## Decision: Text chunking strategy
**Rationale**: For effective RAG, text needs to be chunked into appropriate sizes (700-1000 tokens) with overlap (200 tokens) to maintain context while enabling efficient retrieval.

**Alternatives considered**:
- Fixed character length chunks: Simpler but doesn't respect semantic boundaries
- Sentence-based chunks: Better semantic boundaries but potentially inconsistent sizes

## Decision: Metadata storage approach
**Rationale**: Store URL, title, and chunk-specific metadata with each vector to enable rich retrieval experiences and proper source attribution.

**Alternatives considered**:
- Minimal metadata: Less storage but less context for retrieval
- External metadata storage: More complex but potentially more efficient for large datasets