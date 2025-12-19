"""
CRUD operations for database models.

This module implements Create, Read, Update, Delete operations
for the user profile data.
"""

from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from sqlalchemy.exc import IntegrityError
from sqlalchemy import update
from typing import Optional, List
from ..models.user_profile import UserProfile
from .schemas import UserProfileCreate, UserProfileUpdate


async def create_user_profile(db: AsyncSession, profile: UserProfileCreate) -> UserProfile:
    """
    Create a new user profile.

    Args:
        db: Database session
        profile: Profile data to create

    Returns:
        Created UserProfile object

    Raises:
        IntegrityError: If profile for user already exists
    """
    db_profile = UserProfile(**profile.model_dump())
    db.add(db_profile)

    try:
        await db.commit()
        await db.refresh(db_profile)
        return db_profile
    except IntegrityError:
        await db.rollback()
        raise ValueError(f"Profile for user {profile.user_id} already exists")


async def get_user_profile_by_user_id(db: AsyncSession, user_id: str) -> Optional[UserProfile]:
    """
    Get user profile by user ID.

    Args:
        db: Database session
        user_id: ID of the user whose profile to retrieve

    Returns:
        UserProfile object if found, None otherwise
    """
    result = await db.execute(select(UserProfile).filter(UserProfile.user_id == user_id))
    return result.scalar_one_or_none()


async def update_user_profile(db: AsyncSession, user_id: str, profile_update: UserProfileUpdate) -> Optional[UserProfile]:
    """
    Update user profile with provided fields.

    Args:
        db: Database session
        user_id: ID of the user whose profile to update
        profile_update: Updated profile data

    Returns:
        Updated UserProfile object if found, None otherwise
    """
    db_profile = await get_user_profile_by_user_id(db, user_id)
    if not db_profile:
        return None

    # Update only provided fields
    update_data = profile_update.model_dump(exclude_unset=True)
    for field, value in update_data.items():
        setattr(db_profile, field, value)

    await db.commit()
    await db.refresh(db_profile)
    return db_profile


async def delete_user_profile(db: AsyncSession, user_id: str) -> bool:
    """
    Delete user profile.

    Args:
        db: Database session
        user_id: ID of the user whose profile to delete

    Returns:
        True if profile was deleted, False if not found
    """
    db_profile = await get_user_profile_by_user_id(db, user_id)
    if not db_profile:
        return False

    await db.delete(db_profile)
    await db.commit()
    return True


async def get_user_preferences(db: AsyncSession, user_id: str) -> Optional[dict]:
    """
    Get user preferences specifically for content personalization.

    Args:
        db: Database session
        user_id: ID of the user whose preferences to retrieve

    Returns:
        Dictionary of user preferences for personalization
    """
    profile = await get_user_profile_by_user_id(db, user_id)
    if not profile:
        return None

    return {
        "programming_level": profile.programming_level,
        "programming_languages": profile.programming_languages,
        "ai_knowledge_level": profile.ai_knowledge_level,
        "hardware_experience": profile.hardware_experience,
        "learning_style": profile.learning_style
    }