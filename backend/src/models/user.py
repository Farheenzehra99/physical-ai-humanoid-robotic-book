"""
User model for authentication.

This module defines the User SQLAlchemy model for authentication.
"""

from datetime import datetime
from uuid import uuid4

from sqlalchemy import String, DateTime, Boolean
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import Mapped, mapped_column
from sqlalchemy.sql import func

from sqlalchemy.orm import relationship

from ..database import Base


class User(Base):
    """
    User model for authentication.

    Attributes:
        id: UUID primary key
        email: User's email address (unique)
        hashed_password: Hashed password
        is_active: Whether the user account is active
        is_verified: Whether the user's email is verified
        created_at: Timestamp of account creation
        updated_at: Timestamp of last update
    """

    __tablename__ = "users"

    id: Mapped[UUID] = mapped_column(
        UUID(as_uuid=True),
        primary_key=True,
        default=uuid4
    )

    email: Mapped[str] = mapped_column(
        String,
        unique=True,
        index=True,
        nullable=False
    )

    first_name: Mapped[str] = mapped_column(
        String,
        nullable=True
    )

    last_name: Mapped[str] = mapped_column(
        String,
        nullable=True
    )

    hashed_password: Mapped[str] = mapped_column(
        String,
        nullable=False
    )

    is_active: Mapped[bool] = mapped_column(
        Boolean,
        default=True,
        nullable=False
    )

    is_verified: Mapped[bool] = mapped_column(
        Boolean,
        default=False,
        nullable=False
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

    # Relationship to the user profile
    profile = relationship("UserProfile", back_populates="user", uselist=False, cascade="all, delete-orphan")


# Pydantic models for API validation
from pydantic import BaseModel
from uuid import UUID as PyUUID
from datetime import datetime


class UserResponse(BaseModel):
    """
    Response model for user data.
    """
    id: PyUUID
    email: str
    is_active: bool
    is_verified: bool
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True