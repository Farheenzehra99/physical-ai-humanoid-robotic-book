"""
Database models module.

This module defines the SQLAlchemy models for the application,
including the UserProfile model for the authentication system.
"""

from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey, ARRAY, Enum as SQLEnum
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
import uuid
from enum import Enum
from .base import Base


class ProgrammingLevel(Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class AIKnowledgeLevel(Enum):
    NONE = "none"
    BASIC = "basic"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class HardwareExperience(Enum):
    NONE = "none"
    BASIC = "basic"
    ROBOTICS = "robotics"
    EMBEDDED_SYSTEMS = "embedded_systems"


class LearningStyle(Enum):
    THEORY = "theory"
    CODE_FIRST = "code_first"
    VISUAL = "visual"
    MIXED = "mixed"


