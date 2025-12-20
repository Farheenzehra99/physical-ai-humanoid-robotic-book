"""
Retrieval tool for the RAG Agent.

This module wraps the existing retrieval pipeline as an agent tool
using the OpenAI Agents SDK function_tool decorator.
"""

import json
import logging
from typing import Any

# Import the retrieve function using a more robust method
import os
import sys
from pathlib import Path
from dotenv import load_dotenv

# Get the absolute path to the backend directory and load .env
current_file_dir = Path(__file__).parent
backend_path = current_file_dir.parent.parent.parent
backend_env_path = backend_path / ".env"

# Load the backend .env file to ensure retrieval config is available
if backend_env_path.exists():
    load_dotenv(str(backend_env_path))

# Get the absolute path to the scripts directory
scripts_path = backend_path.parent / "scripts"

# Add to Python path if not already there
scripts_path_str = str(scripts_path)
if scripts_path_str not in sys.path:
    sys.path.insert(0, scripts_path_str)

# Import the retrieve function
try:
    from retrieval.retrieve import retrieve
except ImportError as e:
    print(f"Failed to import retrieval module: {e}")
    print(f"Current working directory: {os.getcwd()}")
    print(f"Python path: {sys.path}")
    print(f"Looking for scripts in: {scripts_path}")
    raise


logger = logging.getLogger(__name__)


def retrieve_context(query: str, top_k: int = 5) -> str:
    """
    Retrieve relevant document chunks from the Physical AI & Humanoid Robotics book.

    Args:
        query: The search query text - should capture the key concepts being asked about
        top_k: Number of results to return (default: 5, max: 20)

    Returns:
        JSON string containing retrieved chunks with metadata including:
        - text: The document chunk content
        - source_url: URL of the source document
        - title: Title of the document section
        - chunk_id: Unique identifier for the chunk
        - score: Relevance score (0-1, higher is better)
    """
    try:
        logger.info(f"Retrieving context for query: '{query[:100]}...' (top_k={top_k})")

        # Call the existing retrieval pipeline
        results = retrieve(query=query, top_k=min(top_k, 20))

        logger.info(f"Retrieved {len(results)} results")

        # Format results as JSON for the agent
        return json.dumps(results, indent=2)

    except Exception as e:
        logger.error(f"Retrieval failed: {e}")
        return json.dumps({
            "error": "Retrieval failed",
            "message": str(e)
        })


async def retrieve_context_async(query: str, top_k: int = 5) -> list[dict[str, Any]]:
    """
    Async wrapper for retrieval - returns parsed results directly.

    This is used by the agent runner for direct access to retrieval results.

    Args:
        query: Search query text
        top_k: Number of results

    Returns:
        List of retrieval result dictionaries
    """
    try:
        results = retrieve(query=query, top_k=min(top_k, 20))
        return results

    except Exception as e:
        logger.error(f"Async retrieval failed: {e}")
        return []
