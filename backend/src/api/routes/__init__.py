"""
Routes package for the RAG Agent API.

This package contains all API route definitions.
"""

from .health import router as health_router
from .chat import router as chat_router
from .personalize import router as personalize_router
from .translate import router as translate_router

__all__ = ["health_router", "chat_router", "personalize_router", "translate_router"]
