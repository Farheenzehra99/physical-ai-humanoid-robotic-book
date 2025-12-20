"""
Middleware package for the RAG Agent API.

This package contains error handlers and other middleware components.
"""

from .error_handler import register_error_handlers

__all__ = ["register_error_handlers"]
