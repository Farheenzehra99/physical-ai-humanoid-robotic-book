# Database Schema for User Profiles

## 1. Overview
This document specifies the database schema for the user authentication and profiling system. It includes the integration with Better Auth's user model and the extended profile data storage.

## 2. Database Technology
- **Database**: PostgreSQL (via Neon Serverless)
- **ORM**: SQLAlchemy
- **Connection Pooling**: Built-in SQLAlchemy connection pooling
- **Migration Tool**: Alembic (recommended for schema evolution)

## 3. Database Tables

### 3.1 Better Auth Base Tables
Better Auth will create its own tables for core authentication. These include:
- `users`: Core user information (id, email, name, etc.)
- `sessions`: Session management
- `accounts`: Third-party account linking
- `verification_requests`: Email verification tokens
- `password_reset_tokens`: Password reset functionality

### 3.2 Extended User Profile Table
We'll create an extended table to store user profile data:

```sql
CREATE TABLE user_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id VARCHAR(255) NOT NULL UNIQUE REFERENCES users(id) ON DELETE CASCADE,
    programming_level VARCHAR(20) CHECK (programming_level IN ('beginner', 'intermediate', 'advanced')),
    programming_languages TEXT[], -- Array of programming language strings
    ai_knowledge_level VARCHAR(20) CHECK (ai_knowledge_level IN ('none', 'basic', 'intermediate', 'advanced')),
    hardware_experience VARCHAR(30) CHECK (hardware_experience IN ('none', 'basic', 'robotics', 'embedded_systems')),
    learning_style VARCHAR(20) CHECK (learning_style IN ('theory', 'code_first', 'visual', 'mixed')),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

-- Indexes for performance
CREATE INDEX idx_user_profiles_user_id ON user_profiles(user_id);
CREATE INDEX idx_user_profiles_programming_level ON user_profiles(programming_level);
CREATE INDEX idx_user_profiles_ai_knowledge_level ON user_profiles(ai_knowledge_level);
CREATE INDEX idx_user_profiles_hardware_experience ON user_profiles(hardware_experience);
```

## 4. SQLAlchemy Models

### 4.1 User Profile Model
```python
# models/user_profile.py
from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey, ARRAY, Enum as SQLEnum
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
    user_id = Column(String(255), ForeignKey("users.id", ondelete="CASCADE"), nullable=False, unique=True)

    # User background information
    programming_level = Column(SQLEnum(ProgrammingLevel), nullable=True)
    programming_languages = Column(ARRAY(String), default=[])  # Array of programming language strings
    ai_knowledge_level = Column(SQLEnum(AIKnowledgeLevel), nullable=True)
    hardware_experience = Column(SQLEnum(HardwareExperience), nullable=True)
    learning_style = Column(SQLEnum(LearningStyle), nullable=True)

    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    # Relationship to Better Auth user (assuming we can reference it)
    # Note: This assumes we have access to Better Auth's User model somehow
    # user = relationship("User", back_populates="profile")
```

### 4.2 Complete Database Model
```python
# models/__init__.py
from .user_profile import UserProfile, ProgrammingLevel, AIKnowledgeLevel, HardwareExperience, LearningStyle
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, Session
from sqlalchemy.ext.declarative import declarative_base
import os

DATABASE_URL = os.getenv("DATABASE_URL")

engine = create_engine(DATABASE_URL)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

# Function to create all tables
def create_tables():
    Base.metadata.create_all(bind=engine)
```

## 5. Database Connection and Session Management

### 5.1 Database Configuration
```python
# database.py
from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from urllib.parse import quote_plus
import os

# Properly encode the database URL to handle special characters
DB_USER = os.getenv("DB_USER", "")
DB_PASSWORD = quote_plus(os.getenv("DB_PASSWORD", ""))
DB_HOST = os.getenv("DB_HOST", "")
DB_PORT = os.getenv("DB_PORT", "5432")
DB_NAME = os.getenv("DB_NAME", "")

DATABASE_URL = f"postgresql://{DB_USER}:{DB_PASSWORD}@{DB_HOST}:{DB_PORT}/{DB_NAME}"

engine = create_engine(
    DATABASE_URL,
    pool_size=20,  # Number of connections to maintain in the pool
    max_overflow=30,  # Additional connections beyond pool_size
    pool_pre_ping=True,  # Verify connections before use
    pool_recycle=300  # Recycle connections after 5 minutes
)

SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

Base = declarative_base()

def get_db():
    """Dependency for getting database session"""
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
```

## 6. Data Access Layer (Repository Pattern)

### 6.1 User Profile Repository
```python
# repositories/user_profile_repository.py
from sqlalchemy.orm import Session
from sqlalchemy.exc import IntegrityError
from ..models.user_profile import UserProfile
from typing import Optional, List

class UserProfileRepository:
    def __init__(self, db: Session):
        self.db = db

    def create_profile(self, user_id: str, programming_level: str, programming_languages: List[str],
                      ai_knowledge_level: str, hardware_experience: str, learning_style: str) -> UserProfile:
        """Create a new user profile"""
        profile = UserProfile(
            user_id=user_id,
            programming_level=programming_level,
            programming_languages=programming_languages,
            ai_knowledge_level=ai_knowledge_level,
            hardware_experience=hardware_experience,
            learning_style=learning_style
        )

        try:
            self.db.add(profile)
            self.db.commit()
            self.db.refresh(profile)
            return profile
        except IntegrityError:
            self.db.rollback()
            raise ValueError(f"Profile for user {user_id} already exists")

    def get_profile_by_user_id(self, user_id: str) -> Optional[UserProfile]:
        """Get user profile by user ID"""
        return self.db.query(UserProfile).filter(UserProfile.user_id == user_id).first()

    def update_profile(self, user_id: str, **kwargs) -> Optional[UserProfile]:
        """Update user profile with provided fields"""
        profile = self.get_profile_by_user_id(user_id)
        if not profile:
            return None

        # Update only provided fields
        for field, value in kwargs.items():
            if hasattr(profile, field):
                setattr(profile, field, value)

        profile.updated_at = func.now()
        self.db.commit()
        self.db.refresh(profile)
        return profile

    def delete_profile(self, user_id: str) -> bool:
        """Delete user profile"""
        profile = self.get_profile_by_user_id(user_id)
        if not profile:
            return False

        self.db.delete(profile)
        self.db.commit()
        return True
```

## 7. Migration Scripts

### 7.1 Alembic Migration Script
```python
# alembic/versions/xxx_create_user_profiles_table.py
"""Create user_profiles table

Revision ID: xxx
Revises:
Create Date: 2025-xx-xx xx:xx:xx.xxxxxx

"""
from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

# revision identifiers
revision = 'xxx'
down_revision = None
branch_labels = None
depends_on = None

def upgrade() -> None:
    # Create the user_profiles table
    op.create_table(
        'user_profiles',
        sa.Column('id', postgresql.UUID(as_uuid=True), nullable=False),
        sa.Column('user_id', sa.String(length=255), nullable=False),
        sa.Column('programming_level', sa.String(length=20), nullable=True),
        sa.Column('programming_languages', postgresql.ARRAY(sa.Text()), nullable=True),
        sa.Column('ai_knowledge_level', sa.String(length=20), nullable=True),
        sa.Column('hardware_experience', sa.String(length=30), nullable=True),
        sa.Column('learning_style', sa.String(length=20), nullable=True),
        sa.Column('created_at', sa.DateTime(timezone=True), server_default=sa.text('CURRENT_TIMESTAMP'), nullable=True),
        sa.Column('updated_at', sa.DateTime(timezone=True), nullable=True),
        sa.PrimaryKeyConstraint('id'),
        sa.UniqueConstraint('user_id')
    )

    # Create indexes
    op.create_index('idx_user_profiles_user_id', 'user_profiles', ['user_id'])
    op.create_index('idx_user_profiles_programming_level', 'user_profiles', ['programming_level'])
    op.create_index('idx_user_profiles_ai_knowledge_level', 'user_profiles', ['ai_knowledge_level'])
    op.create_index('idx_user_profiles_hardware_experience', 'user_profiles', ['hardware_experience'])

    # Add foreign key constraint to users table (assuming users table exists from Better Auth)
    op.create_foreign_key(
        'fk_user_profiles_user_id',
        'user_profiles', 'users',
        ['user_id'], ['id'],
        ondelete='CASCADE'
    )


def downgrade() -> None:
    # Drop foreign key constraint
    op.drop_constraint('fk_user_profiles_user_id', 'user_profiles', type_='foreignkey')

    # Drop indexes
    op.drop_index('idx_user_profiles_hardware_experience')
    op.drop_index('idx_user_profiles_ai_knowledge_level')
    op.drop_index('idx_user_profiles_programming_level')
    op.drop_index('idx_user_profiles_user_id')

    # Drop the user_profiles table
    op.drop_table('user_profiles')
```

## 8. Database Initialization

### 8.1 Initialize Database with Required Tables
```python
# database_init.py
from sqlalchemy import create_engine, inspect
from sqlalchemy.exc import ProgrammingError
from .models import Base
from .database import DATABASE_URL
import logging

def initialize_database():
    """Initialize the database with required tables"""
    engine = create_engine(DATABASE_URL)

    # Create all tables
    Base.metadata.create_all(bind=engine)

    # Verify tables were created
    inspector = inspect(engine)
    tables = inspector.get_table_names()

    logging.info(f"Database initialized with tables: {tables}")

    # Check if user_profiles table exists
    if 'user_profiles' not in tables:
        logging.warning("user_profiles table was not created")
        return False

    return True

def check_database_connection():
    """Check if database connection is working"""
    try:
        engine = create_engine(DATABASE_URL)
        # Try to connect and execute a simple query
        with engine.connect() as conn:
            result = conn.execute(sa.text("SELECT 1"))
            return True
    except Exception as e:
        logging.error(f"Database connection failed: {str(e)}")
        return False
```

## 9. Performance Considerations

### 9.1 Indexing Strategy
- Primary key index on `id` (automatic)
- Unique index on `user_id` (enforces one profile per user)
- Individual indexes on categorical fields for filtering:
  - `programming_level`
  - `ai_knowledge_level`
  - `hardware_experience`

### 9.2 Query Optimization
- Use indexed columns in WHERE clauses
- Limit result sets with pagination for profile listings
- Consider materialized views for complex analytical queries

## 10. Security Considerations

### 10.1 Data Protection
- Encrypt sensitive profile data at rest if needed
- Use parameterized queries to prevent SQL injection
- Implement proper access controls at the application layer
- Regular backup procedures for user data

### 10.2 Compliance
- GDPR compliance for EU users
- Data retention policies
- Right to deletion implementation
- Audit logging for profile changes