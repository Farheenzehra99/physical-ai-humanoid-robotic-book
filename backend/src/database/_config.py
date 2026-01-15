# """
# Database configuration for authentication system.

# This module contains database configuration settings
# and connection pooling parameters.
# """

# from sqlalchemy import create_engine
# from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
# from sqlalchemy.orm import sessionmaker
# from typing import AsyncGenerator
# from ..config import get_settings


# # Get database settings from config
# settings = get_settings()


# def create_database_engines():
#     """
#     Create database engines with optimized connection pooling settings.

#     Returns:
#         Tuple of (async_engine, sync_engine, AsyncSessionLocal, SessionLocal)
#     """
#     # Create async engine for PostgreSQL with optimized settings
#     DATABASE_URL = settings.database_url

#     # Fix for asyncpg: convert sslmode to ssl parameter
#     if "+asyncpg" in DATABASE_URL and "sslmode=" in DATABASE_URL:
#         DATABASE_URL = DATABASE_URL.replace("sslmode=require", "ssl=require")
#         DATABASE_URL = DATABASE_URL.replace("sslmode=prefer", "ssl=prefer")
#         DATABASE_URL = DATABASE_URL.replace("sslmode=disable", "ssl=disable")

#     async_engine = create_async_engine(
#         DATABASE_URL,
#         echo=settings.debug,
#         pool_pre_ping=True,  # Verify connections before use
#         pool_recycle=300,    # Recycle connections after 5 minutes
#         pool_size=5,         # Reduced pool size for connection limits
#         max_overflow=10,     # Additional connections beyond pool_size
#         pool_timeout=30,     # Timeout for getting a connection from the pool
#     )

#     # Create async session maker
#     AsyncSessionLocal = sessionmaker(
#         class_=AsyncSession,
#         bind=async_engine,
#         expire_on_commit=False
#     )

#     # Sync engine is optional - only create if psycopg2 is available (for migrations)
#     sync_engine = None
#     SessionLocal = None
#     try:
#         sync_url = DATABASE_URL.replace("+asyncpg", "")
#         sync_engine = create_engine(
#             sync_url,
#             echo=settings.debug,
#             pool_pre_ping=True,
#             pool_recycle=300,
#             pool_size=5,
#             max_overflow=10,
#             pool_timeout=30
#         )
#         SessionLocal = sessionmaker(
#             bind=sync_engine,
#             expire_on_commit=False
#         )
#     except Exception:
#         # Sync engine not available, that's OK for runtime operations
#         pass

#     return async_engine, sync_engine, AsyncSessionLocal, SessionLocal


# # Create global database components
# async_engine, sync_engine, AsyncSessionLocal, SessionLocal = create_database_engines()


# # Dependency to get async database session
# async def get_async_db() -> AsyncGenerator[AsyncSession, None]:
#     async with AsyncSessionLocal() as session:
#         yield session


# # Dependency to get sync database session
# def get_sync_db():
#     db = SessionLocal()
#     try:
#         yield db
#     finally:
#         db.close()


# # Health check for database connection
# async def check_database_connection():
#     """
#     Check if the database connection is working.

#     Returns:
#         True if connection is successful, False otherwise
#     """
#     try:
#         async with AsyncSessionLocal() as session:
#             # Execute a simple query to test the connection
#             await session.execute("SELECT 1")
#         return True
#     except Exception:
#         return False

"""
Database configuration for authentication system (SYNC version).

This module contains database configuration settings
and connection pooling parameters for a synchronous setup.
"""

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

from ..config import get_settings

# Get database settings from config
settings = get_settings()

# Use DATABASE_URL from .env (psycopg2 or SQLite)
DATABASE_URL = settings.database_url

# Create synchronous SQLAlchemy engine
engine = create_engine(
    DATABASE_URL,
    echo=settings.debug,
    pool_pre_ping=True,  # Verify connections before use
    pool_recycle=300,  # Recycle connections after 5 minutes
    pool_size=5,  # Pool size
    max_overflow=10,  # Extra connections
    pool_timeout=30,
)

# Create session maker
SessionLocal = sessionmaker(bind=engine, expire_on_commit=False)


# Dependency to get sync database session
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


# Health check for database connection (sync)
def check_database_connection() -> bool:
    """
    Check if the database connection is working.

    Returns:
        True if connection is successful, False otherwise
    """
    try:
        with engine.connect() as conn:
            conn.execute("SELECT 1")
        return True
    except Exception:
        return False
