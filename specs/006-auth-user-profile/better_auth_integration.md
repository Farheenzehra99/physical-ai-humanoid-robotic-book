# Better Auth Integration with FastAPI

## 1. Overview
This document outlines the integration of Better Auth with the existing FastAPI backend for the AI book project. Better Auth provides secure authentication with support for email/password and social login providers.

## 2. Installation and Setup

### Required Dependencies
```bash
pip install better-auth[fastapi]
pip install python-multipart  # for handling form data
```

### Environment Variables
```env
# Better Auth Configuration
AUTH_SECRET=your-super-secret-jwt-signing-key-here-make-it-long-and-random
AUTH_URL=http://localhost:3000  # Your frontend URL
AUTH_TRUST_HOST=true

# Database Configuration (for Neon Serverless Postgres)
DATABASE_URL=postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname

# Email Configuration (optional, for verification emails)
SMTP_HOST=smtp.gmail.com
SMTP_PORT=587
SMTP_USERNAME=your-email@gmail.com
SMTP_PASSWORD=your-app-password
SMTP_FROM=no-reply@yourdomain.com
```

## 3. Better Auth Configuration

### Main Application Setup
```python
# main.py or auth_app.py
from fastapi import FastAPI
from better_auth.fastapi import get_auth_router
from better_auth import BaseClient
from better_auth.api.models import Session
import os

# Initialize Better Auth client
client = BaseClient(
    secret=os.getenv("AUTH_SECRET"),
    base_url=os.getenv("AUTH_URL"),
    trust_host=os.getenv("AUTH_TRUST_HOST", "false").lower() == "true"
)

# Configure Better Auth with custom callbacks for profile data
auth_config = {
    "secret": os.getenv("AUTH_SECRET"),
    "base_url": os.getenv("AUTH_URL"),
    "trust_host": os.getenv("AUTH_TRUST_HOST", "false").lower() == "true",
    "email_verification": {
        "enabled": True,
        "send_on_signup": True
    },
    "social_providers": {
        "google": {
            "client_id": os.getenv("GOOGLE_CLIENT_ID"),
            "client_secret": os.getenv("GOOGLE_CLIENT_SECRET")
        },
        "github": {
            "client_id": os.getenv("GITHUB_CLIENT_ID"),
            "client_secret": os.getenv("GITHUB_CLIENT_SECRET")
        }
    }
}

# Get the auth router with custom callbacks
auth_router = get_auth_router(
    client=client,
    config=auth_config,
    # Custom callbacks for handling profile data
    after_register_callback=after_user_registration,
    after_signin_callback=after_user_signin
)

app = FastAPI()
app.include_router(auth_router, prefix="/auth")
```

## 4. Custom Callback Functions

### After Registration Callback
```python
from better_auth.api.models import User
from sqlalchemy.orm import Session as DBSession
from sqlalchemy import create_engine
from .database import get_db
from .models import UserProfile
import logging

async def after_user_registration(user: User):
    """
    Called after a user registers successfully.
    Creates an extended user profile with background information.
    """
    try:
        # Extract profile data from user metadata or session
        profile_data = user.metadata.get('profile_data', {})

        # Create database session
        db = next(get_db())

        # Create user profile record
        user_profile = UserProfile(
            user_id=user.id,
            programming_level=profile_data.get('programming_level'),
            programming_languages=profile_data.get('programming_languages', []),
            ai_knowledge_level=profile_data.get('ai_knowledge_level'),
            hardware_experience=profile_data.get('hardware_experience'),
            learning_style=profile_data.get('learning_style')
        )

        db.add(user_profile)
        db.commit()
        db.refresh(user_profile)

        logging.info(f"Created profile for user {user.id}")

    except Exception as e:
        logging.error(f"Error creating user profile for {user.id}: {str(e)}")
        # Don't raise exception as it would fail the registration
```

### After Signin Callback
```python
async def after_user_signin(session: Session):
    """
    Called after a user signs in successfully.
    Could be used for analytics or session tracking.
    """
    try:
        logging.info(f"User {session.user_id} signed in successfully")

        # Additional logic could go here:
        # - Update last login timestamp
        # - Track user activity
        # - Send welcome back notifications

    except Exception as e:
        logging.error(f"Error in signin callback for session {session.id}: {str(e)}")
```

## 5. Database Models Integration

### SQLAlchemy Models
```python
# models.py
from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey, ARRAY, Enum
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
import uuid
from enum import Enum as PyEnum

Base = declarative_base()

class ProgrammingLevel(PyEnum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"

class AIKnowledgeLevel(PyEnum):
    NONE = "none"
    BASIC = "basic"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"

class HardwareExperience(PyEnum):
    NONE = "none"
    BASIC = "basic"
    ROBOTICS = "robotics"
    EMBEDDED_SYSTEMS = "embedded_systems"

class LearningStyle(PyEnum):
    THEORY = "theory"
    CODE_FIRST = "code_first"
    VISUAL = "visual"
    MIXED = "mixed"

class UserProfile(Base):
    __tablename__ = "user_profiles"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(String, ForeignKey("users.id"), nullable=False, unique=True)  # Reference to Better Auth user
    programming_level = Column(Enum(ProgrammingLevel), nullable=True)
    programming_languages = Column(ARRAY(String), default=[])  # Store as text array
    ai_knowledge_level = Column(Enum(AIKnowledgeLevel), nullable=True)
    hardware_experience = Column(Enum(HardwareExperience), nullable=True)
    learning_style = Column(Enum(LearningStyle), nullable=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    # Relationship to Better Auth user (if extending the user model)
    user = relationship("User", back_populates="profile")  # Assuming we extend Better Auth's user model
```

## 6. Middleware for Authentication

### Authentication Middleware
```python
# middleware.py
from fastapi import Request, HTTPException
from better_auth.fastapi.middleware import AuthMiddleware
from better_auth import BaseClient
import os

class BetterAuthMiddleware:
    def __init__(self):
        self.client = BaseClient(
            secret=os.getenv("AUTH_SECRET"),
            base_url=os.getenv("AUTH_URL"),
            trust_host=os.getenv("AUTH_TRUST_HOST", "false").lower() == "true"
        )

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
        else:
            request.state.user = None

        response = await call_next(request)
        return response

def get_current_user(request: Request):
    """Dependency to get current authenticated user"""
    if not hasattr(request.state, 'user') or request.state.user is None:
        raise HTTPException(status_code=401, detail="Not authenticated")
    return request.state.user
```

## 7. Custom Signup Endpoint

### Extended Signup with Profile Data
```python
# auth_endpoints.py
from fastapi import APIRouter, Depends, HTTPException, Request
from pydantic import BaseModel
from typing import List
from .models import ProgrammingLevel, AIKnowledgeLevel, HardwareExperience, LearningStyle
from .user_schema import UserRegistrationRequest
from better_auth.fastapi.dependencies import get_current_user
from better_auth import BaseClient
import os

router = APIRouter()

class ExtendedSignupRequest(UserRegistrationRequest):
    pass  # Inherits all fields from UserRegistrationRequest

@router.post("/signup")
async def extended_signup(request: ExtendedSignupRequest):
    """
    Extended signup endpoint that captures profile data during registration
    """
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

        # Initialize Better Auth client
        client = BaseClient(
            secret=os.getenv("AUTH_SECRET"),
            base_url=os.getenv("AUTH_URL"),
            trust_host=os.getenv("AUTH_TRUST_HOST", "false").lower() == "true"
        )

        # Create user with Better Auth
        user = await client.sign_up_with_email_password(
            email=request.email,
            password=request.password,
            first_name=request.first_name,
            last_name=request.last_name,
            metadata=profile_metadata  # Pass profile data as metadata
        )

        return {
            "success": True,
            "user_id": user.id,
            "email": user.email,
            "first_name": user.first_name,
            "last_name": user.last_name
        }

    except Exception as e:
        # Handle specific Better Auth errors
        if "already exists" in str(e).lower():
            raise HTTPException(status_code=400, detail="Email already registered")
        else:
            raise HTTPException(status_code=500, detail=f"Registration failed: {str(e)}")

@router.get("/me")
async def get_current_user_profile(request: Request):
    """
    Get current user's profile including extended profile data
    """
    user = await get_current_user(request)

    if not user:
        raise HTTPException(status_code=401, detail="Not authenticated")

    # Fetch extended profile from our database
    from .database import get_db
    from .models import UserProfile

    db = next(get_db())
    profile = db.query(UserProfile).filter(UserProfile.user_id == user.id).first()

    return {
        "id": user.id,
        "email": user.email,
        "first_name": user.first_name,
        "last_name": user.last_name,
        "profile": profile.__dict__ if profile else None
    }
```

## 8. Dependency Injection Setup

### FastAPI Dependencies
```python
# dependencies.py
from fastapi import Depends, HTTPException, Request
from typing import Optional
from better_auth import BaseClient
from better_auth.api.models import User
import os

async def get_current_active_user(request: Request) -> User:
    """Get the current authenticated user, raising an exception if not authenticated"""
    auth_token = request.cookies.get("better-auth.session_token") or \
                 request.headers.get("Authorization", "").replace("Bearer ", "")

    if not auth_token:
        raise HTTPException(status_code=401, detail="Not authenticated")

    client = BaseClient(
        secret=os.getenv("AUTH_SECRET"),
        base_url=os.getenv("AUTH_URL"),
        trust_host=os.getenv("AUTH_TRUST_HOST", "false").lower() == "true"
    )

    session = await client.verify_session(auth_token)

    if not session or not session.user:
        raise HTTPException(status_code=401, detail="Invalid session")

    return session.user

# Usage in protected routes
@router.get("/protected-route")
async def protected_endpoint(current_user: User = Depends(get_current_active_user)):
    return {"message": f"Hello {current_user.email}, you are authenticated!"}
```

## 9. Error Handling and Security

### Security Best Practices
- Use HTTPS in production
- Set secure, httpOnly cookies
- Implement rate limiting on auth endpoints
- Validate all input data
- Use strong password requirements
- Implement proper session management
- Log authentication attempts for security monitoring

### Error Responses
```python
# error_handlers.py
from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse

async def auth_exception_handler(request: Request, exc: HTTPException):
    """
    Custom exception handler for authentication errors
    """
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": "Authentication failed",
            "message": exc.detail,
            "timestamp": datetime.utcnow().isoformat()
        }
    )

# Add to main app
app.add_exception_handler(HTTPException, auth_exception_handler)
```

## 10. Testing Considerations

### Unit Tests for Auth Integration
```python
# test_auth.py
import pytest
from fastapi.testclient import TestClient
from main import app
from better_auth import BaseClient

client = TestClient(app)

def test_signup_with_profile_data():
    """Test signup with profile data collection"""
    response = client.post("/auth/signup", json={
        "email": "test@example.com",
        "password": "SecurePass123!",
        "first_name": "John",
        "last_name": "Doe",
        "programming_level": "intermediate",
        "programming_languages": ["Python", "JavaScript"],
        "ai_knowledge_level": "basic",
        "hardware_experience": "robotics",
        "learning_style": "code_first",
        "agree_to_terms": True
    })

    assert response.status_code == 200
    data = response.json()
    assert data["success"] is True
    assert "user_id" in data

def test_protected_route():
    """Test access to protected routes"""
    # First authenticate
    signup_response = client.post("/auth/signup", ...)
    # Then test protected route
    response = client.get("/protected-route",
                         headers={"Authorization": f"Bearer {token}"})
    assert response.status_code == 200
```