"""
Pydantic schemas for authentication endpoints.

This module defines the Pydantic schemas for request/response validation
of authentication endpoints including signup with profile data.
"""

from pydantic import BaseModel, Field, EmailStr, validator
from typing import List, Optional
from enum import Enum
from .config import auth_settings


class ProgrammingLevel(str, Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class AIKnowledgeLevel(str, Enum):
    NONE = "none"
    BASIC = "basic"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class HardwareExperience(str, Enum):
    NONE = "none"
    BASIC = "basic"
    ROBOTICS = "robotics"
    EMBEDDED_SYSTEMS = "embedded_systems"


class LearningStyle(str, Enum):
    THEORY = "theory"
    CODE_FIRST = "code_first"
    VISUAL = "visual"
    MIXED = "mixed"


class UserRegistrationRequest(BaseModel):
    email: EmailStr = Field(..., description="User's email address")
    password: str = Field(..., min_length=6, description="User's password (min 6 chars)")
    first_name: str = Field(..., min_length=1, max_length=50, description="User's first name")
    last_name: str = Field(..., min_length=1, max_length=50, description="User's last name")
    programming_level: ProgrammingLevel = Field(..., description="User's programming skill level")
    programming_languages: List[str] = Field(
        default=[],
        description="List of programming languages the user knows",
        max_items=10
    )
    ai_knowledge_level: AIKnowledgeLevel = Field(..., description="User's AI knowledge level")
    hardware_experience: HardwareExperience = Field(..., description="User's hardware experience level")
    learning_style: LearningStyle = Field(..., description="User's preferred learning style")
    agree_to_terms: bool = Field(True, description="User agrees to terms of service")

    @validator('password')
    def validate_password(cls, v):
        # Simplified password validation for demo - just check minimum length
        if len(v) < 6:
            raise ValueError('Password must be at least 6 characters')
        return v

    @validator('programming_languages')
    def validate_programming_languages(cls, v):
        # Allow any programming language for flexibility
        if v is None:
            return []
        return v


class UserRegistrationResponse(BaseModel):
    success: bool
    user_id: str
    email: str
    first_name: str
    last_name: str
    requires_email_verification: bool = True


class LoginRequest(BaseModel):
    email: EmailStr
    password: str


class LoginResponse(BaseModel):
    success: bool
    user: dict
    session_token: str
    expires_at: str


class SignOutResponse(BaseModel):
    success: bool
    message: str


class UserProfileResponse(BaseModel):
    id: str
    email: str
    first_name: str
    last_name: str
    profile: Optional[dict] = None


class UserProfileUpdateRequest(BaseModel):
    programming_level: Optional[ProgrammingLevel] = None
    programming_languages: Optional[List[str]] = Field(None, max_items=10)
    ai_knowledge_level: Optional[AIKnowledgeLevel] = None
    hardware_experience: Optional[HardwareExperience] = None
    learning_style: Optional[LearningStyle] = None

    @validator('programming_languages')
    def validate_programming_languages_update(cls, v):
        # Allow any programming language for flexibility
        if v is None:
            return []
        return v


class ProfileUpdateResponse(BaseModel):
    success: bool
    profile: dict


class UserPreferencesResponse(BaseModel):
    programming_level: Optional[ProgrammingLevel] = None
    programming_languages: List[str] = []
    ai_knowledge_level: Optional[AIKnowledgeLevel] = None
    hardware_experience: Optional[HardwareExperience] = None
    learning_style: Optional[LearningStyle] = None
    personalization_score: Optional[float] = 0.85  # Calculated score for content matching