"""
Authentication middleware.

This module implements authentication middleware for protecting routes
and managing authentication state.
"""

from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse
from .main import auth_client
from typing import Optional
import logging


class AuthMiddleware:
    """
    Authentication middleware class for FastAPI.
    """

    def __init__(self):
        self.client = auth_client

    async def __call__(self, request: Request, call_next):
        # Get token from cookies or headers
        auth_token = request.cookies.get("better-auth.session_token") or \
                    request.headers.get("Authorization", "").replace("Bearer ", "")

        if auth_token:
            try:
                # Verify the token with Better Auth
                session = await self.client.verify_session(auth_token)
                request.state.user = session.user if session else None
            except Exception:
                request.state.user = None
                # Log the authentication failure for security monitoring
                logging.warning(f"Invalid session token for request: {request.url}")
        else:
            request.state.user = None

        response = await call_next(request)
        return response


def get_current_user(request: Request):
    """
    Dependency to get current authenticated user.

    Args:
        request: FastAPI request object

    Returns:
        Current authenticated user

    Raises:
        HTTPException: If user is not authenticated
    """
    if not hasattr(request.state, 'user') or request.state.user is None:
        raise HTTPException(status_code=401, detail="Not authenticated")
    return request.state.user


# Rate limiting middleware for authentication endpoints
class RateLimitMiddleware:
    """
    Simple rate limiting middleware for authentication endpoints.
    """
    def __init__(self, max_attempts: int = 5, window_seconds: int = 60):
        self.max_attempts = max_attempts
        self.window_seconds = window_seconds
        # In a real implementation, you'd use a distributed cache like Redis
        # For now, we'll use a simple in-memory store (not suitable for production)
        self.attempts = {}

    async def __call__(self, request: Request, call_next):
        # Only apply rate limiting to auth endpoints
        if request.url.path.startswith("/auth"):
            client_ip = request.client.host
            current_time = request.state.get('start_time', None)

            # Check if we've seen this IP in the current window
            # This is a simplified implementation - in production, use Redis or similar
            pass  # For now, skip rate limiting implementation

        response = await call_next(request)
        return response


# Security headers middleware
async def add_security_headers(request: Request, call_next):
    """
    Add security headers to responses.
    """
    response = await call_next(request)

    # Add security headers
    response.headers["X-Content-Type-Options"] = "nosniff"
    response.headers["X-Frame-Options"] = "DENY"
    response.headers["X-XSS-Protection"] = "1; mode=block"

    return response