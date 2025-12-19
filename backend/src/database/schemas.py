"""
Pydantic schemas for database models.

This module defines the Pydantic schemas for request/response validation
of the user profile data.
"""

from pydantic import BaseModel, Field
from typing import List, Optional, Literal
from datetime import datetime
from enum import Enum


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


class UserProfileBase(BaseModel):
    programming_level: Optional[ProgrammingLevel] = None
    programming_languages: List[str] = Field(default=[], description="List of programming languages the user knows")
    ai_knowledge_level: Optional[AIKnowledgeLevel] = None
    hardware_experience: Optional[HardwareExperience] = None
    learning_style: Optional[LearningStyle] = None


class UserProfileCreate(UserProfileBase):
    user_id: str


class UserProfileUpdate(BaseModel):
    programming_level: Optional[ProgrammingLevel] = None
    programming_languages: Optional[List[str]] = None
    ai_knowledge_level: Optional[AIKnowledgeLevel] = None
    hardware_experience: Optional[HardwareExperience] = None
    learning_style: Optional[LearningStyle] = None


class UserProfileResponse(UserProfileBase):
    id: str
    user_id: str
    created_at: datetime
    updated_at: Optional[datetime] = None

    class Config:
        from_attributes = True


class UserProfilePublic(BaseModel):
    programming_level: Optional[ProgrammingLevel] = None
    programming_languages: List[str] = Field(default=[], description="List of programming languages the user knows")
    ai_knowledge_level: Optional[AIKnowledgeLevel] = None
    hardware_experience: Optional[HardwareExperience] = None
    learning_style: Optional[LearningStyle] = None