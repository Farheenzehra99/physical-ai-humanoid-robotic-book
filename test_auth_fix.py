#!/usr/bin/env python3
"""
Test script to verify that the authentication system works without external Hugging Face Space dependency.
"""

import asyncio
import sys
import os

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from src.auth.client import auth_client
from src.better_auth_mock import User, Session
from datetime import datetime, timedelta

async def test_auth_client():
    """Test the auth client functionality without external dependencies."""
    print("Testing auth client initialization...")

    # Test that auth client was initialized properly
    assert auth_client is not None
    print("✓ Auth client initialized successfully")

    # Test user signup
    print("\nTesting user signup...")
    try:
        user = await auth_client.sign_up_with_email_password(
            email="test@example.com",
            password="testpassword123",
            first_name="Test",
            last_name="User",
            metadata={"test": "data"}
        )

        assert user is not None
        assert user.email == "test@example.com"
        assert user.first_name == "Test"
        print("✓ User signup successful")
    except Exception as e:
        print(f"✗ User signup failed: {e}")
        return False

    # Test user signin
    print("\nTesting user signin...")
    try:
        session = await auth_client.sign_in_with_email_password(
            email="test@example.com",
            password="testpassword123"
        )

        assert session is not None
        assert session.user.email == "test@example.com"
        assert session.token is not None
        print("✓ User signin successful")
    except Exception as e:
        print(f"✗ User signin failed: {e}")
        return False

    # Test session verification
    print("\nTesting session verification...")
    try:
        verified_session = await auth_client.verify_session(session.token)
        assert verified_session is not None
        assert verified_session.user.email == "test@example.com"
        print("✓ Session verification successful")
    except Exception as e:
        print(f"✗ Session verification failed: {e}")
        return False

    # Test user retrieval by email
    print("\nTesting user retrieval by email...")
    try:
        retrieved_user = await auth_client.get_user_by_email("test@example.com")
        assert retrieved_user is not None
        assert retrieved_user.email == "test@example.com"
        print("✓ User retrieval by email successful")
    except Exception as e:
        print(f"✗ User retrieval by email failed: {e}")
        return False

    # Test sign out
    print("\nTesting sign out...")
    try:
        signout_result = await auth_client.sign_out(session.token)
        assert signout_result is True
        print("✓ Sign out successful")
    except Exception as e:
        print(f"✗ Sign out failed: {e}")
        return False

    print("\n🎉 All auth client tests passed!")
    return True

async def main():
    """Run all tests."""
    print("Testing authentication system without external dependencies...")
    print("=" * 60)

    success = await test_auth_client()

    print("=" * 60)
    if success:
        print("✅ All tests passed! The authentication system is working without external dependencies.")
        print("\nThe signup and signin should now work without the Hugging Face Space dependency.")
        print("The system now relies on the database as the primary source of truth.")
    else:
        print("❌ Some tests failed. Please check the implementation.")

    return success

if __name__ == "__main__":
    result = asyncio.run(main())
    sys.exit(0 if result else 1)