"""
Retrieval function for the RAG system.

This module provides the retrieve function that searches the vector database
for relevant document chunks based on the query using Cohere embeddings.
"""
import os
import logging
from typing import List, Dict, Any, Optional
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import Qdrant client to connect to the vector database
try:
    from qdrant_client import QdrantClient
    from qdrant_client.http import models
except ImportError:
    raise ImportError(
        "qdrant-client is required for the retrieval function. "
        "Please install it with: pip install qdrant-client"
    )

# Import Cohere for embeddings
try:
    import cohere
except ImportError:
    cohere = None
    logging.warning(
        "cohere package not installed. Install it with: pip install cohere"
    )

logger = logging.getLogger(__name__)

# Cache for Cohere client
_cohere_client: Optional[cohere.Client] = None
_qdrant_client: Optional[QdrantClient] = None


def get_cohere_client() -> Optional[cohere.Client]:
    """Get or create cached Cohere client."""
    global _cohere_client
    if _cohere_client is None and cohere is not None:
        api_key = os.getenv("COHERE_API_KEY")
        if api_key:
            _cohere_client = cohere.Client(api_key)
            logger.info("Cohere client initialized")
        else:
            logger.warning("COHERE_API_KEY not set")
    return _cohere_client


def get_qdrant_client() -> Optional[QdrantClient]:
    """Get or create cached Qdrant client."""
    global _qdrant_client
    if _qdrant_client is None:
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if qdrant_url and qdrant_api_key:
            _qdrant_client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
                timeout=30.0
            )
            logger.info(f"Qdrant client initialized with URL: {qdrant_url}")
        else:
            logger.warning("QDRANT_URL or QDRANT_API_KEY not set")
    return _qdrant_client


def generate_embedding(text: str) -> Optional[List[float]]:
    """
    Generate embedding for text using Cohere embedding model.

    Args:
        text: The text to generate an embedding for

    Returns:
        List of floats representing the embedding, or None if failed
    """
    client = get_cohere_client()
    if client is None:
        logger.error("Cohere client not available for embedding generation")
        return None

    try:
        # Use Cohere's embed-english-v3.0 model
        response = client.embed(
            texts=[text],
            model="embed-english-v3.0",
            input_type="search_query"
        )

        if response.embeddings and len(response.embeddings) > 0:
            return response.embeddings[0]
        else:
            logger.warning("Empty embedding response from Cohere")
            return None

    except Exception as e:
        logger.error(f"Failed to generate embedding: {e}")
        return None


def retrieve(query: str, top_k: int = 5) -> List[Dict[str, Any]]:
    """
    Retrieve relevant document chunks from the vector database based on the query.

    Args:
        query: The search query text
        top_k: Number of results to return (default: 5)

    Returns:
        List of retrieval result dictionaries with keys:
        - text: The document chunk content
        - source_url: URL of the source document
        - title: Title of the document section
        - chunk_id: Unique identifier for the chunk
        - score: Relevance score (0-1, higher is better)
    """
    collection_name = os.getenv("QDRANT_COLLECTION", "book_chunks")

    # Get clients
    qdrant_client = get_qdrant_client()
    if qdrant_client is None:
        logger.error("Qdrant client not available")
        return []

    try:
        # Generate embedding for the query
        query_embedding = generate_embedding(query)

        if query_embedding is None:
            logger.error("Failed to generate query embedding")
            return []

        logger.info(f"Searching Qdrant collection '{collection_name}' with top_k={top_k}")

        # Search in Qdrant
        search_results = qdrant_client.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=top_k,
            with_payload=True,
            score_threshold=0.3  # Minimum similarity threshold
        )

        # Format results
        formatted_results = []
        for result in search_results:
            payload = result.payload or {}
            formatted_results.append({
                "text": payload.get("text", payload.get("content", "")),
                "source_url": payload.get("source_url", payload.get("url", "")),
                "title": payload.get("title", payload.get("heading", "Untitled")),
                "chunk_id": str(result.id),
                "score": result.score
            })

        logger.info(f"Retrieved {len(formatted_results)} results from Qdrant")
        return formatted_results

    except Exception as e:
        logger.error(f"Retrieval failed: {e}")
        import traceback
        logger.error(traceback.format_exc())
        return []


async def retrieve_async(query: str, top_k: int = 5) -> List[Dict[str, Any]]:
    """
    Async version of the retrieve function.

    Args:
        query: The search query text
        top_k: Number of results to return

    Returns:
        List of retrieval result dictionaries
    """
    # For now, just call the sync version
    # Qdrant and Cohere clients handle async internally
    return retrieve(query, top_k)


def check_collection_exists(collection_name: str = None) -> bool:
    """
    Check if the specified collection exists in Qdrant.

    Args:
        collection_name: Name of the collection to check

    Returns:
        True if collection exists, False otherwise
    """
    if collection_name is None:
        collection_name = os.getenv("QDRANT_COLLECTION", "book_chunks")

    qdrant_client = get_qdrant_client()
    if qdrant_client is None:
        return False

    try:
        collections = qdrant_client.get_collections()
        collection_names = [c.name for c in collections.collections]
        exists = collection_name in collection_names
        logger.info(f"Collection '{collection_name}' exists: {exists}")
        return exists
    except Exception as e:
        logger.error(f"Failed to check collection: {e}")
        return False


def get_collection_info(collection_name: str = None) -> Optional[Dict[str, Any]]:
    """
    Get information about a collection.

    Args:
        collection_name: Name of the collection

    Returns:
        Dictionary with collection info or None if failed
    """
    if collection_name is None:
        collection_name = os.getenv("QDRANT_COLLECTION", "book_chunks")

    qdrant_client = get_qdrant_client()
    if qdrant_client is None:
        return None

    try:
        info = qdrant_client.get_collection(collection_name)
        return {
            "name": collection_name,
            "points_count": info.points_count,
            "vectors_count": info.vectors_count,
            "status": info.status
        }
    except Exception as e:
        logger.error(f"Failed to get collection info: {e}")
        return None
