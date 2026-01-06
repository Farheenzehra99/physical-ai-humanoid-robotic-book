"""
Middleware package for the RAG Agent API.

This package contains error handlers, rate limiters, and other middleware components.
"""

from .error_handler import register_error_handlers
from .rate_limiter import setup_rate_limiting, limiter, rate_limit_chat, rate_limit_auth

__all__ = [
    "register_error_handlers",
    "setup_rate_limiting",
    "limiter",
    "rate_limit_chat",
    "rate_limit_auth"
]
