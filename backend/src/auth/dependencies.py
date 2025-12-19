"""
FastAPI dependencies for authentication.

This module provides dependency functions for authentication
and authorization in FastAPI routes.
"""

from fastapi import Request, Depends, HTTPException
from typing import Optional
from ..better_auth_mock import BaseClient, User
from .main import auth_client
from .config import auth_settings
import os


async def get_current_active_user(request: Request) -> User:
    """
    Get the current authenticated user, raising an exception if not authenticated.

    Args:
        request: FastAPI request object

    Returns:
        Current authenticated user

    Raises:
        HTTPException: If user is not authenticated
    """
    auth_token = request.cookies.get("better-auth.session_token") or \
                 request.headers.get("Authorization", "").replace("Bearer ", "")

    if not auth_token:
        raise HTTPException(status_code=401, detail="Not authenticated")

    session_with_user = await auth_client.verify_session(auth_token)

    if not session_with_user:
        raise HTTPException(status_code=401, detail="Invalid session")

    return session_with_user.user


async def get_current_user_optional(request: Request) -> Optional[User]:
    """
    Get the current authenticated user, returning None if not authenticated.

    Args:
        request: FastAPI request object

    Returns:
        Current authenticated user or None
    """
    auth_token = request.cookies.get("better-auth.session_token") or \
                 request.headers.get("Authorization", "").replace("Bearer ", "")

    if not auth_token:
        return None

    try:
        session_with_user = await auth_client.verify_session(auth_token)
        return session_with_user.user if session_with_user else None
    except Exception:
        return None


def require_user_role(required_role: str):
    """
    Create a dependency that requires a specific user role.

    Args:
        required_role: The role required to access the resource

    Returns:
        Dependency function that checks user role
    """
    async def role_checker(current_user: User = Depends(get_current_active_user)) -> User:
        # In a real implementation, you'd check the user's roles
        # For now, we'll just return the user (role checking not implemented yet)
        return current_user
    return role_checker


def require_permission(permission: str):
    """
    Create a dependency that requires a specific permission.

    Args:
        permission: The permission required to access the resource

    Returns:
        Dependency function that checks user permission
    """
    async def permission_checker(current_user: User = Depends(get_current_active_user)) -> User:
        # In a real implementation, you'd check the user's permissions
        # For now, we'll just return the user (permission checking not implemented yet)
        return current_user
    return permission_checker


# Admin-only dependency
async def require_admin(current_user: User = Depends(get_current_active_user)) -> User:
    """
    Require that the current user is an admin.

    Args:
        current_user: The current authenticated user

    Returns:
        The current user if they are an admin

    Raises:
        HTTPException: If user is not an admin
    """
    # In a real implementation, you'd check if the user has admin role
    # For now, we'll just return the user (admin checking not implemented yet)
    return current_user