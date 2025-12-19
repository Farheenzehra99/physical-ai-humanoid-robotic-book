"""
Authentication callbacks for profile data handling.

This module provides callback functions that are executed
during user registration and sign-in to handle profile data.
"""

from sqlalchemy.ext.asyncio import AsyncSession
from ..database import AsyncSessionLocal
from ..models.user_profile import UserProfile
from sqlalchemy import select, insert
from typing import Dict, Any, Optional
import logging


async def after_user_registration(
    request: Any,
    user_data: Dict[str, Any],
    profile_data: Optional[Dict[str, Any]] = None
) -> None:
    """
    Called after a user registers successfully.
    Creates an extended user profile with background information.

    Args:
        request: FastAPI request object
        user_data: User data from registration
        profile_data: Optional profile data provided during registration
    """
    try:
        # Extract profile data from the provided profile_data parameter
        profile_info = profile_data or {}

        # Create async database session
        async with AsyncSessionLocal() as db:
            # Create user profile record using raw SQL insert since we don't have CRUD functions
            profile_query = insert(UserProfile).values(
                user_id=user_data['id'],
                programming_level=profile_info.get('programming_level'),
                programming_languages=profile_info.get('programming_languages', []),
                ai_knowledge_level=profile_info.get('ai_knowledge_level'),
                hardware_experience=profile_info.get('hardware_experience'),
                learning_style=profile_info.get('learning_style')
            )

            await db.execute(profile_query)
            await db.commit()

        logging.info(f"Created profile for user {user_data['id']}")

    except Exception as e:
        logging.error(f"Error creating user profile for {user_data['id']}: {str(e)}")
        # Don't raise exception as it would fail the registration


async def after_user_signin(
    request: Any,
    user_data: Dict[str, Any],
    session_data: Dict[str, Any]
) -> None:
    """
    Called after a user signs in successfully.
    Could be used for analytics or session tracking.

    Args:
        request: FastAPI request object
        user_data: User data
        session_data: Session data
    """
    try:
        user_id = user_data.get('id', 'unknown')
        logging.info(f"User {user_id} signed in successfully")

        # Additional logic could go here:
        # - Update last login timestamp
        # - Track user activity
        # - Send welcome back notifications

    except Exception as e:
        logging.error(f"Error in signin callback for session: {str(e)}")