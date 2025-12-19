"""
User Profile model for storing user background information.

This module defines the UserProfile SQLAlchemy model that stores
user background data collected during signup:
1. Programming level (Beginner / Intermediate / Advanced)
2. Known programming languages (multiple selection)
3. AI knowledge level (None / Basic / Intermediate / Advanced)
4. Hardware experience (None / Basic / Robotics / Embedded Systems)
5. Preferred learning style (Theory / Code-first / Visual / Mixed)
"""

from datetime import datetime
from typing import List, Optional
from uuid import uuid4

from sqlalchemy import String, Text, JSON, DateTime, ForeignKey, CheckConstraint
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import Mapped, mapped_column, relationship
from sqlalchemy.sql import func

from ..database import Base  # Assuming we'll create a database module
from .user import User  # Import the User model


class UserProfile(Base):
    """
    UserProfile model to store user background information.

    Attributes:
        id: UUID primary key
        user_id: Reference to the user (to be linked with auth system)
        programming_level: User's programming level (Beginner/Intermediate/Advanced)
        programming_languages: JSON array of known programming languages
        ai_knowledge_level: User's AI knowledge level (None/Basic/Intermediate/Advanced)
        hardware_experience: User's hardware experience (None/Basic/Robotics/Embedded Systems)
        learning_style: User's preferred learning style (Theory/Code-first/Visual/Mixed)
        created_at: Timestamp of profile creation
        updated_at: Timestamp of last profile update
    """

    __tablename__ = "user_profiles"

    id: Mapped[UUID] = mapped_column(
        UUID(as_uuid=True),
        primary_key=True,
        default=uuid4
    )

    user_id: Mapped[UUID] = mapped_column(
        UUID(as_uuid=True),
        ForeignKey("users.id"),  # This links to the auth user table
        nullable=False,
        unique=True  # Each user has only one profile
    )

    # Programming level: Beginner, Intermediate, Advanced
    programming_level: Mapped[Optional[str]] = mapped_column(
        String(20),
        CheckConstraint(
            "programming_level IN ('Beginner', 'Intermediate', 'Advanced')",
            name="valid_programming_level"
        )
    )

    # Programming languages as JSON array
    programming_languages: Mapped[Optional[List[str]]] = mapped_column(
        JSON
    )

    # AI knowledge level: None, Basic, Intermediate, Advanced
    ai_knowledge_level: Mapped[Optional[str]] = mapped_column(
        String(20),
        CheckConstraint(
            "ai_knowledge_level IN ('None', 'Basic', 'Intermediate', 'Advanced')",
            name="valid_ai_knowledge_level"
        )
    )

    # Hardware experience: None, Basic, Robotics, Embedded Systems
    hardware_experience: Mapped[Optional[str]] = mapped_column(
        String(20),
        CheckConstraint(
            "hardware_experience IN ('None', 'Basic', 'Robotics', 'Embedded Systems')",
            name="valid_hardware_experience"
        )
    )

    # Learning style: Theory, Code-first, Visual, Mixed
    learning_style: Mapped[Optional[str]] = mapped_column(
        String(20),
        CheckConstraint(
            "learning_style IN ('Theory', 'Code-first', 'Visual', 'Mixed')",
            name="valid_learning_style"
        )
    )

    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True),
        server_default=func.now()
    )

    updated_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True),
        server_default=func.now(),
        onupdate=func.now()
    )

    # Relationship to the user
    user = relationship("User", back_populates="profile", uselist=False)


# Pydantic models for API validation
from pydantic import BaseModel, Field
from enum import Enum
from typing import List as PyList
from uuid import UUID as PyUUID


class ProgrammingLevel(str, Enum):
    BEGINNER = "Beginner"
    INTERMEDIATE = "Intermediate"
    ADVANCED = "Advanced"


class AIKnowledgeLevel(str, Enum):
    NONE = "None"
    BASIC = "Basic"
    INTERMEDIATE = "Intermediate"
    ADVANCED = "Advanced"


class HardwareExperience(str, Enum):
    NONE = "None"
    BASIC = "Basic"
    ROBOTICS = "Robotics"
    EMBEDDED_SYSTEMS = "Embedded Systems"


class LearningStyle(str, Enum):
    THEORY = "Theory"
    CODE_FIRST = "Code-first"
    VISUAL = "Visual"
    MIXED = "Mixed"


class UserProfileCreateRequest(BaseModel):
    """
    Request model for creating a user profile during signup.
    """
    programming_level: ProgrammingLevel
    programming_languages: PyList[str] = Field(
        default=[],
        description="List of known programming languages"
    )
    ai_knowledge_level: AIKnowledgeLevel
    hardware_experience: HardwareExperience
    learning_style: LearningStyle


class UserProfileResponse(BaseModel):
    """
    Response model for user profile data.
    """
    id: PyUUID
    user_id: PyUUID
    programming_level: Optional[ProgrammingLevel] = None
    programming_languages: Optional[PyList[str]] = None
    ai_knowledge_level: Optional[AIKnowledgeLevel] = None
    hardware_experience: Optional[HardwareExperience] = None
    learning_style: Optional[LearningStyle] = None
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True