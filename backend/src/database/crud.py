"""
CRUD operations for database models.

This module implements Create, Read, Update, Delete operations
for the user and user profile data.

Security features:
- Password hashing using bcrypt (industry standard)
- Automatic salt generation
- Timing-safe comparison for password verification
"""

from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from sqlalchemy.exc import IntegrityError
from sqlalchemy import update
from typing import Optional, List
from ..models.user import User
from ..models.user_profile import UserProfile
from .schemas import UserProfileCreate, UserProfileUpdate
import hashlib
import logging

# Try to import bcrypt, fallback to sha256 if not available
try:
    import bcrypt
    BCRYPT_AVAILABLE = True
except ImportError:
    BCRYPT_AVAILABLE = False
    logging.warning("bcrypt not installed, falling back to SHA256 (less secure)")

logger = logging.getLogger(__name__)


def hash_password(password: str) -> str:
    """
    Hash a password using bcrypt.

    Args:
        password: Plain text password

    Returns:
        Hashed password string
    """
    if BCRYPT_AVAILABLE:
        # bcrypt automatically generates a salt
        salt = bcrypt.gensalt(rounds=12)
        hashed = bcrypt.hashpw(password.encode('utf-8'), salt)
        return hashed.decode('utf-8')
    else:
        # Fallback to SHA256 (less secure)
        return hashlib.sha256(password.encode()).hexdigest()


def verify_password_hash(password: str, hashed: str) -> bool:
    """
    Verify a password against its hash.

    Args:
        password: Plain text password to verify
        hashed: Stored password hash

    Returns:
        True if password matches, False otherwise
    """
    if BCRYPT_AVAILABLE:
        try:
            return bcrypt.checkpw(password.encode('utf-8'), hashed.encode('utf-8'))
        except (ValueError, TypeError):
            # Hash might be in old SHA256 format, try that
            return hashlib.sha256(password.encode()).hexdigest() == hashed
    else:
        return hashlib.sha256(password.encode()).hexdigest() == hashed


# User CRUD operations

async def create_user(
    db: AsyncSession,
    email: str,
    password: str,
    first_name: Optional[str] = None,
    last_name: Optional[str] = None,
    user_id: Optional[str] = None
) -> User:
    """
    Create a new user in the database.

    Args:
        db: Database session
        email: User's email
        password: Plain text password (will be hashed with bcrypt)
        first_name: User's first name (optional)
        last_name: User's last name (optional)
        user_id: Optional user ID (if created by external auth system)

    Returns:
        Created User object

    Raises:
        IntegrityError: If email already exists
    """
    # Hash the password using bcrypt
    hashed_password = hash_password(password)

    db_user = User(
        id=user_id,  # Will use default uuid4 if None
        email=email.lower().strip(),
        first_name=first_name,
        last_name=last_name,
        hashed_password=hashed_password,
        is_active=True,
        is_verified=False
    )

    db.add(db_user)

    try:
        await db.commit()
        await db.refresh(db_user)
        logger.info(f"Created user: {email}")
        return db_user
    except IntegrityError:
        await db.rollback()
        raise ValueError(f"User with email {email} already exists")


async def get_user_by_email(db: AsyncSession, email: str) -> Optional[User]:
    """
    Get user by email address.

    Args:
        db: Database session
        email: User's email address

    Returns:
        User object if found, None otherwise
    """
    email_lower = email.lower().strip()
    result = await db.execute(select(User).filter(User.email == email_lower))
    return result.scalar_one_or_none()


async def get_user_by_id(db: AsyncSession, user_id: str) -> Optional[User]:
    """
    Get user by ID.

    Args:
        db: Database session
        user_id: User's ID

    Returns:
        User object if found, None otherwise
    """
    result = await db.execute(select(User).filter(User.id == user_id))
    return result.scalar_one_or_none()


async def verify_password(db: AsyncSession, email: str, password: str) -> Optional[User]:
    """
    Verify user password using secure comparison.

    Supports both new bcrypt hashes and legacy SHA256 hashes.

    Args:
        db: Database session
        email: User's email
        password: Plain text password to verify

    Returns:
        User object if password is correct, None otherwise
    """
    user = await get_user_by_email(db, email)
    if not user:
        return None

    # Use the secure password verification function
    if verify_password_hash(password, user.hashed_password):
        return user

    return None


# UserProfile CRUD operations

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

    data = profile.model_dump()

    # 🔥 Normalize enum values for DB (Title Case)
    data["programming_level"] = data["programming_level"].capitalize()
    data["ai_knowledge_level"] = data["ai_knowledge_level"].capitalize()
    data["hardware_experience"] = data["hardware_experience"].capitalize()
    data["learning_style"] = data["learning_style"].capitalize()

    if "programming_languages" in data and data["programming_languages"]:
        data["programming_languages"] = [
            lang.capitalize() for lang in data["programming_languages"]
        ]

    db_profile = UserProfile(**data)
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

    update_data = profile_update.model_dump(exclude_unset=True)

    enum_fields = {
        "programming_level",
        "ai_knowledge_level",
        "hardware_experience",
        "learning_style",
    }

    for field, value in update_data.items():
        if field in enum_fields and isinstance(value, str):
            value = value.capitalize()

        if field == "programming_languages" and isinstance(value, list):
            value = [lang.capitalize() for lang in value]

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
