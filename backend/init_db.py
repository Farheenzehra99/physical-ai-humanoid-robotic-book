"""
Database initialization script.

This script creates all database tables defined in the models.
Run this once to initialize your database schema.
"""

import asyncio
import sys
from pathlib import Path

# Add the backend directory to the Python path
backend_dir = Path(__file__).parent
sys.path.insert(0, str(backend_dir))

from sqlalchemy import text
from src.database._config import async_engine
from src.database.base import Base

# Import all models to ensure they're registered with Base
from src.models.user import User
from src.models.user_profile import UserProfile


async def init_database():
    """Initialize the database by creating all tables."""
    print("Initializing database...")
    print(f"Database URL: {async_engine.url}")

    try:
        async with async_engine.begin() as conn:
            print("Dropping existing tables...")
            await conn.run_sync(Base.metadata.drop_all)

            print("Creating tables...")
            await conn.run_sync(Base.metadata.create_all)

            # Verify tables were created
            print("\nVerifying tables...")
            result = await conn.execute(text(
                "SELECT table_name FROM information_schema.tables "
                "WHERE table_schema = 'public'"
            ))
            tables = result.fetchall()

            print(f"\nCreated tables:")
            for table in tables:
                print(f"  - {table[0]}")

        print("\n[SUCCESS] Database initialization completed successfully!")

    except Exception as e:
        print(f"\n[ERROR] Database initialization failed: {e}")
        import traceback
        traceback.print_exc()
        return False

    finally:
        await async_engine.dispose()

    return True


if __name__ == "__main__":
    success = asyncio.run(init_database())
    sys.exit(0 if success else 1)
