"""
Authentication endpoints.

This module implements the API endpoints for authentication
including signup with profile data collection.

Key improvements:
- Proper database-first authentication (DB is source of truth)
- Session synchronization between DB and in-memory auth client
- Correct HTTP status codes (401, 403, 409, 500)
- Robust error handling
"""

from fastapi import APIRouter, Depends, HTTPException, Request, Response
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from typing import Dict, Any, Optional
from ..better_auth_mock import BaseClient, User
from .schemas import UserRegistrationRequest, UserRegistrationResponse, LoginRequest, LoginResponse, SignOutResponse
from ..database import get_async_db
from ..models.user_profile import UserProfile
from ..database.schemas import UserProfileCreate
from ..database.crud import create_user_profile, get_user_profile_by_user_id, update_user_profile
from .main import auth_client
from .config import auth_settings
from .dependencies import get_current_active_user
import os
import logging
from datetime import datetime, timedelta

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/auth", tags=["authentication"])


async def sync_user_to_auth_client(
    db_user,
    password: str,
    metadata: Optional[Dict[str, Any]] = None
) -> bool:
    """
    Synchronize a database user to the in-memory auth client.

    This handles the case where the server has restarted and the
    auth client has lost its in-memory users but the DB still has them.

    Args:
        db_user: User object from database
        password: User's password
        metadata: Optional metadata to store

    Returns:
        True if sync successful, False otherwise
    """
    try:
        # Check if user already exists in auth client
        existing = await auth_client.get_user_by_email(db_user.email)
        if existing:
            return True

        # Register user in auth client with same ID
        await auth_client.sign_up_with_email_password(
            email=db_user.email,
            password=password,
            first_name=db_user.first_name or "User",
            last_name=db_user.last_name or "",
            metadata=metadata or {}
        )
        logger.info(f"Synced user {db_user.email} to auth client")
        return True
    except ValueError as e:
        # User already exists, that's fine
        if "already exists" in str(e).lower():
            return True
        logger.error(f"Failed to sync user to auth client: {e}")
        return False
    except Exception as e:
        logger.error(f"Failed to sync user to auth client: {e}")
        return False


@router.post("/signup", response_model=UserRegistrationResponse)
async def extended_signup(
    request: UserRegistrationRequest,
    db: AsyncSession = Depends(get_async_db)
):
    """
    Extended signup endpoint that captures profile data during registration.

    Database is the source of truth. Users are also stored in auth client
    for session management.

    Returns:
        201: User created successfully
        409: Email already registered
        500: Internal server error
    """
    import traceback
    logger.info(f"Signup request received for email: {request.email}")

    try:
        from ..database.crud import create_user, get_user_by_email

        # Check if user already exists in database
        existing_user = await get_user_by_email(db, request.email)
        if existing_user:
            raise HTTPException(
                status_code=409,
                detail="User with this email already exists"
            )

        # Prepare metadata with profile data
        profile_metadata = {
            "profile_data": {
                "programming_level": request.programming_level.value,
                "programming_languages": request.programming_languages,
                "ai_knowledge_level": request.ai_knowledge_level.value,
                "hardware_experience": request.hardware_experience.value,
                "learning_style": request.learning_style.value
            }
        }

        # Create user with Better Auth first (for session management)
        try:
            auth_user = await auth_client.sign_up_with_email_password(
                email=request.email,
                password=request.password,
                first_name=request.first_name,
                last_name=request.last_name,
                metadata=profile_metadata
            )
        except ValueError as e:
            if "already exists" in str(e).lower():
                raise HTTPException(
                    status_code=409,
                    detail="User with this email already exists"
                )
            raise

        # Save user to database for persistence
        try:
            db_user = await create_user(
                db=db,
                email=request.email,
                password=request.password,
                first_name=request.first_name,
                last_name=request.last_name,
                user_id=auth_user.id
            )

            # Create user profile in database
            profile_data = UserProfileCreate(
                user_id=str(db_user.id),
                programming_level=request.programming_level.value,
                programming_languages=request.programming_languages,
                ai_knowledge_level=request.ai_knowledge_level.value,
                hardware_experience=request.hardware_experience.value,
                learning_style=request.learning_style.value
            )
            await create_user_profile(db, profile_data)

            logger.info(f"User {request.email} created successfully")

        except Exception as db_error:
            logger.error(f"Database save failed: {str(db_error)}")
            raise HTTPException(
                status_code=500,
                detail="Failed to create user account"
            )

        return UserRegistrationResponse(
            success=True,
            user_id=auth_user.id,
            email=auth_user.email,
            first_name=auth_user.first_name,
            last_name=auth_user.last_name,
            requires_email_verification=True
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Registration failed: {str(e)}")
        logger.error(traceback.format_exc())
        error_msg = str(e).lower()
        if "already exists" in error_msg or "duplicate" in error_msg:
            raise HTTPException(
                status_code=409,
                detail="User with this email already exists"
            )
        raise HTTPException(
            status_code=500,
            detail="Registration failed. Please try again."
        )


@router.post("/signin", response_model=LoginResponse)
async def login_user(
    request: Request,
    response: Response,
    credentials: LoginRequest,
    db: AsyncSession = Depends(get_async_db)
):
    """
    Authenticate user with email and password.

    Database is the source of truth for credential verification.
    Auth client is used for session management.

    Returns:
        200: Login successful
        401: Invalid credentials
        500: Server error
    """
    try:
        from ..database.crud import verify_password, get_user_by_email

        client_ip = request.client.host if request.client else "unknown"

        # First check if user exists in database
        db_user = await get_user_by_email(db, credentials.email)
        if not db_user:
            logger.warning(f"Login attempt for non-existent email: {credentials.email} from IP: {client_ip}")
            raise HTTPException(
                status_code=401,
                detail="Invalid credentials"
            )

        # Verify password against database
        verified_user = await verify_password(db, credentials.email, credentials.password)
        if not verified_user:
            logger.warning(f"Invalid password for email: {credentials.email} from IP: {client_ip}")
            raise HTTPException(
                status_code=401,
                detail="Invalid credentials"
            )

        # Sync user to auth client (handles server restart case)
        await sync_user_to_auth_client(db_user, credentials.password)

        # Create session with auth client
        session = await auth_client.sign_in_with_email_password(
            email=credentials.email,
            password=credentials.password
        )

        if not session or not session.user:
            logger.error(f"Session creation failed for user: {db_user.email}")
            raise HTTPException(
                status_code=500,
                detail="Failed to create session"
            )

        logger.info(f"Successful login for user: {db_user.id} from IP: {client_ip}")

        # Set cross-domain cookie for session persistence
        cookie_max_age = 7 * 24 * 60 * 60  # 7 days
        response.set_cookie(
            key="better-auth.session_token",
            value=session.token,
            max_age=cookie_max_age,
            httponly=True,
            secure=True,
            samesite="none",
            path="/"
        )

        return LoginResponse(
            success=True,
            user={
                "id": str(db_user.id),
                "email": db_user.email,
                "first_name": session.user.first_name,
                "last_name": session.user.last_name
            },
            session_token=session.token,
            expires_at=str(session.expires_at)
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Login failed: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail="Login failed. Please try again."
        )


@router.post("/signout", response_model=SignOutResponse)
async def logout_user(request: Request, response: Response):
    """
    Log out current user and clear session cookie.

    Returns:
        200: Logout successful
        500: Server error
    """
    try:
        # Get token from cookies or headers
        auth_token = request.cookies.get("better-auth.session_token") or \
                    request.headers.get("Authorization", "").replace("Bearer ", "")

        if auth_token:
            await auth_client.sign_out(auth_token)

        # Clear the session cookie
        response.delete_cookie(
            key="better-auth.session_token",
            path="/",
            secure=True,
            samesite="none"
        )

        return SignOutResponse(
            success=True,
            message="Successfully logged out"
        )

    except Exception as e:
        logger.error(f"Logout failed: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail="Logout failed"
        )


@router.get("/me")
async def get_current_user_profile(
    current_user = Depends(get_current_active_user),
    db: AsyncSession = Depends(get_async_db)
):
    """
    Get current user's profile including extended profile data.

    Returns:
        200: User profile
        401: Not authenticated
        500: Server error
    """
    try:
        profile = await get_user_profile_by_user_id(db, current_user.id)

        return {
            "id": current_user.id,
            "email": current_user.email,
            "first_name": current_user.first_name,
            "last_name": current_user.last_name,
            "profile": profile.__dict__ if profile else None
        }

    except Exception as e:
        logger.error(f"Failed to get user profile: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail="Failed to retrieve user profile"
        )


@router.put("/profile")
async def update_user_profile_endpoint(
    profile_update: UserProfileCreate,
    current_user = Depends(get_current_active_user),
    db: AsyncSession = Depends(get_async_db)
):
    """
    Update current user's profile information.

    Returns:
        200: Profile updated
        401: Not authenticated
        404: Profile not found
        500: Server error
    """
    try:
        existing_profile = await get_user_profile_by_user_id(db, current_user.id)

        if not existing_profile:
            # Create new profile
            profile_data = profile_update.model_dump()
            profile_data['user_id'] = current_user.id

            new_profile = UserProfileCreate(**profile_data)
            created_profile = await create_user_profile(db, new_profile)

            return {
                "success": True,
                "profile": created_profile.__dict__
            }
        else:
            # Update existing profile
            updated_profile = await update_user_profile(db, current_user.id, profile_update)

            if not updated_profile:
                raise HTTPException(
                    status_code=404,
                    detail="Profile not found"
                )

            return {
                "success": True,
                "profile": updated_profile.__dict__
            }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Failed to update user profile: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail="Failed to update user profile"
        )


@router.get("/preferences")
async def get_user_preferences(
    current_user = Depends(get_current_active_user),
    db: AsyncSession = Depends(get_async_db)
):
    """
    Get user preferences specifically for content personalization.

    Returns:
        200: User preferences with personalization score
        401: Not authenticated
        500: Server error
    """
    try:
        from ..database.crud import get_user_preferences as get_user_prefs

        preferences = await get_user_prefs(db, current_user.id)

        if not preferences:
            return {
                "programming_level": None,
                "programming_languages": [],
                "ai_knowledge_level": None,
                "hardware_experience": None,
                "learning_style": None,
                "personalization_score": 0.0
            }

        # Calculate personalization score based on profile completeness
        filled_fields = sum([
            preferences.get('programming_level') is not None,
            bool(preferences.get('programming_languages')),
            preferences.get('ai_knowledge_level') is not None,
            preferences.get('hardware_experience') is not None,
            preferences.get('learning_style') is not None
        ])

        personalization_score = filled_fields / 5.0

        return {
            "programming_level": preferences.get('programming_level'),
            "programming_languages": preferences.get('programming_languages', []),
            "ai_knowledge_level": preferences.get('ai_knowledge_level'),
            "hardware_experience": preferences.get('hardware_experience'),
            "learning_style": preferences.get('learning_style'),
            "personalization_score": personalization_score
        }

    except Exception as e:
        logger.error(f"Failed to get user preferences: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail="Failed to retrieve user preferences"
        )
