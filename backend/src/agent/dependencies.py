"""
Agent dependency injection for FastAPI.

This module provides cached agent instances for use with FastAPI's
Depends() system.
"""

import logging
from functools import lru_cache

from .agent import BookAgent


def create_book_agent() -> BookAgent:
    """Factory function to create a BookAgent instance."""
    return BookAgent()


# Verify that required dependencies are available
try:
    import google.genai as genai  # type: ignore
except ImportError:
    raise ImportError(
        "google-genai package is required for the BookAgent. "
        "Please install it with: pip install google-genai"
    )

logger = logging.getLogger(__name__)


@lru_cache(maxsize=1)
def get_agent_runner() -> BookAgent:
    """
    Get a cached BookAgent instance.

    The agent is cached to avoid reinitializing for every request.
    Uses lru_cache with maxsize=1 for singleton behavior.

    Returns:
        BookAgent: Cached agent instance
    """
    logger.info("Initializing BookAgent...")
    return create_book_agent()


def clear_agent_cache() -> None:
    """
    Clear the agent cache.

    Useful for testing or when configuration changes.
    """
    get_agent_runner.cache_clear()
    logger.info("Agent cache cleared")
