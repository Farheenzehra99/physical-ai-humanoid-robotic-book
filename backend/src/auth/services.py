"""
Authentication services.

This module contains business logic for authentication operations
including user registration, sign in, and profile management.
"""

from typing import Dict, Any, Optional
from sqlalchemy.ext.asyncio import AsyncSession
from ..better_auth_mock import User
from ..database.crud import get_user_profile_by_user_id, update_user_profile, create_user_profile
from ..database.schemas import UserProfileCreate, UserProfileUpdate
from .main import auth_client
from .validators import validate_profile_data, validate_email_format, validate_password_strength, validate_user_name
from .schemas import UserRegistrationRequest
import logging


class AuthService:
    """
    Authentication service class containing business logic for auth operations.
    """

    @staticmethod
    async def register_user_with_profile(
        user_data: UserRegistrationRequest,
        db: AsyncSession
    ) -> Dict[str, Any]:
        """
        Register a new user with profile data.

        Args:
            user_data: Registration request data
            db: Database session

        Returns:
            Dictionary with registration result
        """
        # Validate email format
        if not validate_email_format(user_data.email):
            raise ValueError("Invalid email format")

        # Validate password strength
        password_validation = validate_password_strength(user_data.password)
        if not password_validation["is_valid"]:
            raise ValueError(f"Password validation failed: {'; '.join(password_validation['errors'])}")

        # Validate user names
        first_name_validation = validate_user_name(user_data.first_name)
        if not first_name_validation["is_valid"]:
            raise ValueError(f"First name validation failed: {'; '.join(first_name_validation['errors'])}")

        last_name_validation = validate_user_name(user_data.last_name)
        if not last_name_validation["is_valid"]:
            raise ValueError(f"Last name validation failed: {'; '.join(last_name_validation['errors'])}")

        # Validate profile data
        profile_validation = validate_profile_data(
            user_data.programming_level,
            user_data.programming_languages,
            user_data.ai_knowledge_level,
            user_data.hardware_experience,
            user_data.learning_style
        )

        if not profile_validation["is_valid"]:
            raise ValueError(f"Profile validation failed: {profile_validation['errors']}")

        try:
            # Prepare metadata with profile data for Better Auth
            profile_metadata = {
                "profile_data": {
                    "programming_level": user_data.programming_level.value,
                    "programming_languages": user_data.programming_languages,
                    "ai_knowledge_level": user_data.ai_knowledge_level.value,
                    "hardware_experience": user_data.hardware_experience.value,
                    "learning_style": user_data.learning_style.value
                }
            }

            # Create user with Better Auth
            user = await auth_client.sign_up_with_email_password(
                email=user_data.email,
                password=user_data.password,
                first_name=user_data.first_name,
                last_name=user_data.last_name,
                metadata=profile_metadata  # Pass profile data as metadata
            )

            return {
                "success": True,
                "user_id": user.id,
                "email": user.email,
                "first_name": user.first_name,
                "last_name": user.last_name,
                "requires_email_verification": True
            }

        except Exception as e:
            logging.error(f"Registration failed: {str(e)}")
            raise e

    @staticmethod
    async def authenticate_user(
        email: str,
        password: str
    ) -> Optional[Dict[str, Any]]:
        """
        Authenticate user with email and password.

        Args:
            email: User's email
            password: User's password

        Returns:
            Authentication result or None if authentication fails
        """
        try:
            # Sign in with Better Auth
            session = await auth_client.sign_in_with_email_password(
                email=email,
                password=password
            )

            if not session or not session.user:
                return None

            return {
                "success": True,
                "user": {
                    "id": session.user.id,
                    "email": session.user.email,
                    "first_name": session.user.first_name,
                    "last_name": session.user.last_name
                },
                "session_token": session.token,
                "expires_at": str(session.expires_at)
            }

        except Exception as e:
            logging.error(f"Authentication failed: {str(e)}")
            return None

    @staticmethod
    async def get_user_profile_with_preferences(
        user_id: str,
        db: AsyncSession
    ) -> Optional[Dict[str, Any]]:
        """
        Get user profile with preferences for personalization.

        Args:
            user_id: ID of the user
            db: Database session

        Returns:
            User profile and preferences or None if user not found
        """
        try:
            # Fetch extended profile from our database
            profile = await get_user_profile_by_user_id(db, user_id)

            if not profile:
                return {
                    "user_id": user_id,
                    "profile": None,
                    "preferences": {
                        "programming_level": None,
                        "programming_languages": [],
                        "ai_knowledge_level": None,
                        "hardware_experience": None,
                        "learning_style": None,
                        "personalization_score": 0.0
                    }
                }

            # Calculate personalization score
            filled_fields = sum([
                profile.programming_level is not None,
                bool(profile.programming_languages),
                profile.ai_knowledge_level is not None,
                profile.hardware_experience is not None,
                profile.learning_style is not None
            ])

            personalization_score = filled_fields / 5.0  # 5 total fields

            return {
                "user_id": user_id,
                "profile": {
                    "id": str(profile.id),
                    "user_id": profile.user_id,
                    "programming_level": profile.programming_level,
                    "programming_languages": profile.programming_languages,
                    "ai_knowledge_level": profile.ai_knowledge_level,
                    "hardware_experience": profile.hardware_experience,
                    "learning_style": profile.learning_style,
                    "created_at": profile.created_at,
                    "updated_at": profile.updated_at
                },
                "preferences": {
                    "programming_level": profile.programming_level,
                    "programming_languages": profile.programming_languages,
                    "ai_knowledge_level": profile.ai_knowledge_level,
                    "hardware_experience": profile.hardware_experience,
                    "learning_style": profile.learning_style,
                    "personalization_score": personalization_score
                }
            }

        except Exception as e:
            logging.error(f"Failed to get user profile: {str(e)}")
            return None

    @staticmethod
    async def update_user_profile(
        user_id: str,
        profile_update: UserProfileUpdate,
        db: AsyncSession
    ) -> Optional[Dict[str, Any]]:
        """
        Update user profile information.

        Args:
            user_id: ID of the user
            profile_update: Profile update data
            db: Database session

        Returns:
            Updated profile data or None if update failed
        """
        try:
            # Validate the update data
            update_data = profile_update.model_dump(exclude_unset=True)

            if update_data:
                # Validate profile data if any fields are being updated
                programming_level = update_data.get('programming_level')
                programming_languages = update_data.get('programming_languages')
                ai_knowledge_level = update_data.get('ai_knowledge_level')
                hardware_experience = update_data.get('hardware_experience')
                learning_style = update_data.get('learning_style')

                if any([programming_level, programming_languages, ai_knowledge_level,
                        hardware_experience, learning_style]):

                    # Use current values for fields not being updated
                    current_profile = await get_user_profile_by_user_id(db, user_id)
                    if current_profile:
                        if programming_level is None:
                            programming_level = current_profile.programming_level
                        if programming_languages is None:
                            programming_languages = current_profile.programming_languages
                        if ai_knowledge_level is None:
                            ai_knowledge_level = current_profile.ai_knowledge_level
                        if hardware_experience is None:
                            hardware_experience = current_profile.hardware_experience
                        if learning_style is None:
                            learning_style = current_profile.learning_style
                    else:
                        # If no current profile exists, use the update values or defaults
                        programming_level = programming_level or "beginner"
                        programming_languages = programming_languages or []
                        ai_knowledge_level = ai_knowledge_level or "none"
                        hardware_experience = hardware_experience or "none"
                        learning_style = learning_style or "mixed"

                    profile_validation = validate_profile_data(
                        programming_level,
                        programming_languages,
                        ai_knowledge_level,
                        hardware_experience,
                        learning_style
                    )

                    if not profile_validation["is_valid"]:
                        raise ValueError(f"Profile validation failed: {profile_validation['errors']}")

            # Update profile in database
            updated_profile = await update_user_profile(db, user_id, profile_update)

            if not updated_profile:
                return None

            return {
                "success": True,
                "profile": {
                    "id": str(updated_profile.id),
                    "user_id": updated_profile.user_id,
                    "programming_level": updated_profile.programming_level,
                    "programming_languages": updated_profile.programming_languages,
                    "ai_knowledge_level": updated_profile.ai_knowledge_level,
                    "hardware_experience": updated_profile.hardware_experience,
                    "learning_style": updated_profile.learning_style,
                    "updated_at": updated_profile.updated_at
                }
            }

        except Exception as e:
            logging.error(f"Failed to update user profile: {str(e)}")
            raise e

    @staticmethod
    async def create_user_profile_if_not_exists(
        user_id: str,
        db: AsyncSession
    ) -> bool:
        """
        Create a default user profile if one doesn't exist.

        Args:
            user_id: ID of the user
            db: Database session

        Returns:
            True if profile was created or already exists, False otherwise
        """
        try:
            # Check if profile already exists
            existing_profile = await get_user_profile_by_user_id(db, user_id)
            if existing_profile:
                return True

            # Create default profile
            default_profile = UserProfileCreate(
                user_id=user_id,
                programming_level=None,
                programming_languages=[],
                ai_knowledge_level=None,
                hardware_experience=None,
                learning_style=None
            )

            await create_user_profile(db, default_profile)
            return True

        except Exception as e:
            logging.error(f"Failed to create default user profile: {str(e)}")
            return False