"""
Rate limiting middleware for the RAG Agent API.

This module provides rate limiting using SlowAPI to prevent abuse
and ensure fair usage of the API.

Rate limits:
- Chat endpoint: 10 requests/minute (unauthenticated), 30/minute (authenticated)
- Auth endpoints: 5 requests/minute (to prevent brute force)
- General API: 60 requests/minute
"""

import logging
from typing import Optional, Callable

from fastapi import Request, Response
from slowapi import Limiter
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
from slowapi.middleware import SlowAPIMiddleware

logger = logging.getLogger(__name__)


def get_user_identifier(request: Request) -> str:
    """
    Get a unique identifier for the user making the request.

    Uses authenticated user ID if available, otherwise falls back to IP address.

    Args:
        request: FastAPI request object

    Returns:
        Unique identifier string
    """
    # Try to get user from session cookie
    session_token = request.cookies.get("better-auth.session_token")
    if session_token:
        # Use first 16 chars of token as identifier (enough to be unique)
        return f"user_{session_token[:16]}"

    # Fall back to IP address
    return get_remote_address(request)


# Create limiter instance with custom key function
limiter = Limiter(
    key_func=get_user_identifier,
    default_limits=["60/minute"],
    storage_uri="memory://",  # In-memory storage (use Redis for production clusters)
    strategy="fixed-window"
)


# Rate limit decorators for different endpoint types
def rate_limit_chat(func: Callable) -> Callable:
    """Apply rate limit for chat endpoints (10 req/min unauthenticated, 30 req/min authenticated)."""
    return limiter.limit("30/minute", key_func=get_user_identifier)(func)


def rate_limit_auth(func: Callable) -> Callable:
    """Apply rate limit for auth endpoints (5 req/min to prevent brute force)."""
    return limiter.limit("5/minute")(func)


def rate_limit_general(func: Callable) -> Callable:
    """Apply general rate limit (60 req/min)."""
    return limiter.limit("60/minute")(func)


async def rate_limit_exceeded_handler(request: Request, exc: RateLimitExceeded) -> Response:
    """
    Handle rate limit exceeded errors.

    Returns a 429 response with Retry-After header.
    """
    from fastapi.responses import JSONResponse
    from ..middleware.error_handler import ErrorResponse

    logger.warning(f"Rate limit exceeded for {get_user_identifier(request)}")

    error_response = {
        "error": "rate_limit_exceeded",
        "message": "Too many requests. Please wait before trying again.",
        "retry_after": 60
    }

    return JSONResponse(
        status_code=429,
        content=error_response,
        headers={"Retry-After": "60"}
    )


def setup_rate_limiting(app):
    """
    Configure rate limiting for the FastAPI application.

    Args:
        app: FastAPI application instance
    """
    app.state.limiter = limiter
    app.add_exception_handler(RateLimitExceeded, rate_limit_exceeded_handler)
    app.add_middleware(SlowAPIMiddleware)
    logger.info("Rate limiting middleware configured")
