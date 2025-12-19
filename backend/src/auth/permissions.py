"""
Permissions and authorization utilities.

This module contains functions for checking user permissions
and authorization for protected resources.
"""

from typing import Dict, List, Optional, Union
from fastapi import HTTPException
from better_auth.api.models import User
from ..database.models import UserProfile
from ..database.crud import get_user_profile_by_user_id
from sqlalchemy.ext.asyncio import AsyncSession


class PermissionChecker:
    """
    Class for checking user permissions and authorization.
    """

    @staticmethod
    async def has_permission(
        user: User,
        permission: str,
        db: AsyncSession
    ) -> bool:
        """
        Check if user has a specific permission.

        Args:
            user: The user to check permissions for
            permission: The permission to check
            db: Database session

        Returns:
            True if user has the permission, False otherwise
        """
        # In a real implementation, you would check the user's permissions
        # from a roles/permissions database table
        # For now, we'll implement a basic check based on profile data

        profile = await get_user_profile_by_user_id(db, user.id)
        if not profile:
            return False

        # Example permission checks based on user profile
        if permission == "access_advanced_content":
            return profile.ai_knowledge_level in ["intermediate", "advanced"]
        elif permission == "access_hardware_content":
            return profile.hardware_experience in ["robotics", "embedded_systems"]
        elif permission == "create_content":
            return profile.programming_level == "advanced"

        # Default to allowing basic permissions
        return True

    @staticmethod
    async def has_role(
        user: User,
        role: str,
        db: AsyncSession
    ) -> bool:
        """
        Check if user has a specific role.

        Args:
            user: The user to check roles for
            role: The role to check
            db: Database session

        Returns:
            True if user has the role, False otherwise
        """
        # In a real implementation, you would check the user's roles
        # from a roles database table
        # For now, we'll return False as role-based access isn't fully implemented

        # Placeholder: check if user has admin email (for testing purposes only)
        if role == "admin":
            admin_emails = ["admin@yourdomain.com", "test@admin.com"]  # This should come from config
            return user.email in admin_emails

        return False

    @staticmethod
    async def can_access_resource(
        user: User,
        resource_type: str,
        resource_id: Optional[str] = None,
        db: AsyncSession = None
    ) -> bool:
        """
        Check if user can access a specific resource.

        Args:
            user: The user requesting access
            resource_type: Type of resource (e.g., "content", "profile", "admin")
            resource_id: Optional specific resource ID
            db: Database session

        Returns:
            True if user can access the resource, False otherwise
        """
        if resource_type == "profile":
            # Users can access their own profile
            if resource_id:
                return user.id == resource_id
            return True
        elif resource_type == "content":
            # Check based on user profile if they can access certain content
            if db:
                profile = await get_user_profile_by_user_id(db, user.id)
                if profile:
                    # Example: Basic users can't access advanced content
                    if resource_type == "advanced" and profile.programming_level == "beginner":
                        return False
            return True
        elif resource_type == "admin":
            # Only admins can access admin resources
            return await PermissionChecker.has_role(user, "admin", db) if db else False

        return True


async def require_permission(
    user: User,
    permission: str,
    db: AsyncSession
) -> bool:
    """
    Dependency function to require a specific permission.

    Args:
        user: The current user
        permission: The required permission
        db: Database session

    Returns:
        True if user has permission

    Raises:
        HTTPException: If user doesn't have the required permission
    """
    has_perm = await PermissionChecker.has_permission(user, permission, db)
    if not has_perm:
        raise HTTPException(
            status_code=403,
            detail=f"Permission '{permission}' required to access this resource"
        )
    return True


async def require_role(
    user: User,
    role: str,
    db: AsyncSession
) -> bool:
    """
    Dependency function to require a specific role.

    Args:
        user: The current user
        role: The required role
        db: Database session

    Returns:
        True if user has role

    Raises:
        HTTPException: If user doesn't have the required role
    """
    has_role_result = await PermissionChecker.has_role(user, role, db)
    if not has_role_result:
        raise HTTPException(
            status_code=403,
            detail=f"Role '{role}' required to access this resource"
        )
    return True


def create_access_control_list(
    user: User,
    profile: Optional[UserProfile] = None
) -> Dict[str, Union[bool, List[str]]]:
    """
    Create an access control list for a user based on their profile.

    Args:
        user: The user to create ACL for
        profile: Optional user profile (if already fetched)

    Returns:
        Dictionary with user's permissions and access levels
    """
    acl = {
        "user_id": user.id,
        "email": user.email,
        "permissions": [],
        "roles": [],
        "can_access": {
            "basic_content": True,
            "intermediate_content": False,
            "advanced_content": False,
            "admin_panel": False,
            "profile_management": True
        }
    }

    # If profile is provided, enhance ACL based on profile data
    if profile:
        # Set content access based on programming level
        if profile.programming_level == "intermediate":
            acl["can_access"]["intermediate_content"] = True
        elif profile.programming_level == "advanced":
            acl["can_access"]["intermediate_content"] = True
            acl["can_access"]["advanced_content"] = True

        # Set permissions based on profile
        if profile.ai_knowledge_level in ["intermediate", "advanced"]:
            acl["permissions"].append("access_advanced_ai_content")
        if profile.hardware_experience in ["robotics", "embedded_systems"]:
            acl["permissions"].append("access_hardware_content")

    return acl