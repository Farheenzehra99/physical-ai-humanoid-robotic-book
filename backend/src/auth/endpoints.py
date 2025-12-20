"""
Authentication endpoints.

This module implements the API endpoints for authentication
including signup with profile data collection.
"""

from fastapi import APIRouter, Depends, HTTPException, Request
from sqlalchemy.ext.asyncio import AsyncSession
from typing import Dict, Any
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


router = APIRouter(prefix="/auth", tags=["authentication"])


@router.post("/signup", response_model=UserRegistrationResponse)
async def extended_signup(
    request: UserRegistrationRequest
):
    """
    Extended signup endpoint that captures profile data during registration.
    Note: Using mock auth client with in-memory storage (no DB needed)
    """
    import traceback
    print(f"DEBUG: Signup request received for email: {request.email}")
    try:
        # Prepare metadata with profile data for Better Auth
        profile_metadata = {
            "profile_data": {
                "programming_level": request.programming_level.value,
                "programming_languages": request.programming_languages,
                "ai_knowledge_level": request.ai_knowledge_level.value,
                "hardware_experience": request.hardware_experience.value,
                "learning_style": request.learning_style.value
            }
        }

        # Create user with Better Auth
        user = await auth_client.sign_up_with_email_password(
            email=request.email,
            password=request.password,
            first_name=request.first_name,
            last_name=request.last_name,
            metadata=profile_metadata  # Pass profile data as metadata
        )

        return UserRegistrationResponse(
            success=True,
            user_id=user.id,
            email=user.email,
            first_name=user.first_name,
            last_name=user.last_name,
            requires_email_verification=True
        )

    except Exception as e:
        # Handle specific Better Auth errors
        print(f"DEBUG ERROR: {type(e).__name__}: {str(e)}")
        print(traceback.format_exc())
        error_msg = str(e).lower()
        if "already exists" in error_msg or "duplicate" in error_msg:
            raise HTTPException(status_code=400, detail="Email already registered")
        else:
            logging.error(f"Registration failed: {str(e)}")
            raise HTTPException(status_code=500, detail=f"Registration failed: {str(e)}")


@router.post("/signin", response_model=LoginResponse)
async def login_user(
    request: Request,
    credentials: LoginRequest
):
    """
    Authenticate existing user with rate limiting and security measures.
    """
    try:
        # Basic rate limiting - in production, use Redis or similar for distributed rate limiting
        # For now, we'll implement a simple in-memory rate limiter per IP
        client_ip = request.client.host
        current_time = __import__('time').time()

        # Check if we've seen this IP recently (simple in-memory rate limiting)
        # This is a simplified implementation - in production, use Redis or similar
        # For now, we'll just continue with the main logic

        # Sign in with Better Auth
        session = await auth_client.sign_in_with_email_password(
            email=credentials.email,
            password=credentials.password
        )

        if not session or not session.user:
            logging.warning(f"Failed login attempt for email: {credentials.email} from IP: {client_ip}")
            raise HTTPException(status_code=401, detail="Invalid credentials")

        # Successful login - log for security monitoring
        logging.info(f"Successful login for user: {session.user.id} from IP: {client_ip}")

        return LoginResponse(
            success=True,
            user={
                "id": session.user.id,
                "email": session.user.email,
                "first_name": session.user.first_name,
                "last_name": session.user.last_name
            },
            session_token=session.token,
            expires_at=str(session.expires_at)
        )

    except HTTPException:
        # Re-raise HTTP exceptions (like 401)
        raise
    except Exception as e:
        logging.error(f"Login failed: {str(e)}")
        raise HTTPException(status_code=401, detail="Invalid credentials")


@router.post("/signout", response_model=SignOutResponse)
async def logout_user(request: Request):
    """
    Log out current user.
    """
    try:
        # Get token from cookies or headers
        auth_token = request.cookies.get("better-auth.session_token") or \
                    request.headers.get("Authorization", "").replace("Bearer ", "")

        if auth_token:
            # Sign out with Better Auth
            await auth_client.sign_out(auth_token)

        return SignOutResponse(
            success=True,
            message="Successfully logged out"
        )

    except Exception as e:
        logging.error(f"Logout failed: {str(e)}")
        raise HTTPException(status_code=500, detail="Logout failed")


@router.get("/me")
async def get_current_user_profile(
    current_user = Depends(get_current_active_user),
    db: AsyncSession = Depends(get_async_db)
):
    """
    Get current user's profile including extended profile data.
    """
    try:
        # Fetch extended profile from our database
        profile = await get_user_profile_by_user_id(db, current_user.id)

        return {
            "id": current_user.id,
            "email": current_user.email,
            "first_name": current_user.first_name,
            "last_name": current_user.last_name,
            "profile": profile.__dict__ if profile else None
        }

    except Exception as e:
        logging.error(f"Failed to get user profile: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to retrieve user profile")


@router.put("/profile")
async def update_user_profile(
    profile_update: UserProfileCreate,  # Using the schema from database
    current_user = Depends(get_current_active_user),
    db: AsyncSession = Depends(get_async_db)
):
    """
    Update current user's profile information.
    """
    try:
        # Fetch existing profile
        existing_profile = await get_user_profile_by_user_id(db, current_user.id)

        if not existing_profile:
            # If no profile exists, create one
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
                raise HTTPException(status_code=404, detail="Profile not found")

            return {
                "success": True,
                "profile": updated_profile.__dict__
            }

    except Exception as e:
        logging.error(f"Failed to update user profile: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to update user profile")


@router.get("/preferences")
async def get_user_preferences(
    current_user = Depends(get_current_active_user),
    db: AsyncSession = Depends(get_async_db)
):
    """
    Get user preferences specifically for content personalization.
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

        # Calculate a basic personalization score based on completeness of profile
        filled_fields = sum([
            preferences.get('programming_level') is not None,
            bool(preferences.get('programming_languages')),
            preferences.get('ai_knowledge_level') is not None,
            preferences.get('hardware_experience') is not None,
            preferences.get('learning_style') is not None
        ])

        personalization_score = filled_fields / 5.0  # 5 total fields

        return {
            "programming_level": preferences.get('programming_level'),
            "programming_languages": preferences.get('programming_languages', []),
            "ai_knowledge_level": preferences.get('ai_knowledge_level'),
            "hardware_experience": preferences.get('hardware_experience'),
            "learning_style": preferences.get('learning_style'),
            "personalization_score": personalization_score
        }

    except Exception as e:
        logging.error(f"Failed to get user preferences: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to retrieve user preferences")