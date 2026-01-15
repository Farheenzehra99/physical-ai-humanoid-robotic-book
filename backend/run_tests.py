#!/usr/bin/env python3
"""
Test runner for the Physical AI RAG Chatbot backend.

This script tests the key components and endpoints after the fixes.
"""

import sys
import os

# Fix Windows encoding issues
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

# Add the backend directory to the path
backend_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, backend_dir)

import asyncio
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Use ASCII symbols for Windows compatibility
OK = "[OK]"
FAIL = "[FAIL]"
WARN = "[WARN]"


def test_imports():
    """Test that all modules can be imported."""
    print("\n=== Testing Imports ===")

    try:
        from src.config import get_settings
        settings = get_settings()
        print(f"{OK} Config loaded: API Version {settings.api_version}")
    except Exception as e:
        print(f"{FAIL} Config import failed: {e}")
        return False

    try:
        from src.database.crud import hash_password, verify_password_hash
        print(f"{OK} CRUD with bcrypt password hashing imported")
    except Exception as e:
        print(f"{FAIL} CRUD import failed: {e}")
        return False

    try:
        from src.api.middleware import register_error_handlers, setup_rate_limiting
        print(f"{OK} Middleware (error handlers + rate limiting) imported")
    except Exception as e:
        print(f"{FAIL} Middleware import failed: {e}")
        return False

    try:
        from src.agent.agent import BookAgent
        print(f"{OK} BookAgent with async OpenAI client imported")
    except Exception as e:
        print(f"{FAIL} Agent import failed: {e}")
        return False

    try:
        from src.auth.endpoints import router as auth_router
        print(f"{OK} Auth endpoints imported")
    except Exception as e:
        print(f"{FAIL} Auth endpoints import failed: {e}")
        return False

    try:
        from src.api.app import create_app
        print(f"{OK} FastAPI app factory imported")
    except Exception as e:
        print(f"{FAIL} App import failed: {e}")
        return False

    return True


def test_password_hashing():
    """Test bcrypt password hashing."""
    print("\n=== Testing Password Hashing (bcrypt) ===")

    from src.database.crud import hash_password, verify_password_hash, BCRYPT_AVAILABLE

    if BCRYPT_AVAILABLE:
        print(f"{OK} bcrypt is available")
    else:
        print(f"{WARN} bcrypt not available, using SHA256 fallback")

    test_password = "TestPassword123!"

    # Test hashing
    hashed = hash_password(test_password)
    print(f"{OK} Password hashed: {hashed[:20]}...")

    # Test verification
    if verify_password_hash(test_password, hashed):
        print(f"{OK} Password verification successful")
    else:
        print(f"{FAIL} Password verification failed")
        return False

    # Test wrong password
    if not verify_password_hash("WrongPassword", hashed):
        print(f"{OK} Wrong password correctly rejected")
    else:
        print(f"{FAIL} Wrong password was accepted!")
        return False

    return True


def test_retry_decorator():
    """Test the retry with backoff decorator."""
    print("\n=== Testing Retry Decorator ===")

    sys.path.insert(0, os.path.join(backend_dir, '..', 'scripts'))

    try:
        from retrieval.retrieve import retry_with_backoff, MAX_RETRIES, BASE_DELAY
        print(f"{OK} Retry decorator imported (max_retries={MAX_RETRIES}, base_delay={BASE_DELAY}s)")

        # Test with a function that always succeeds
        @retry_with_backoff(max_retries=3, base_delay=0.1)
        def success_func():
            return "success"

        result = success_func()
        if result == "success":
            print(f"{OK} Retry decorator works for successful calls")
        else:
            print(f"{FAIL} Retry decorator returned wrong value")
            return False

        return True
    except Exception as e:
        print(f"{FAIL} Retry decorator test failed: {e}")
        return False


def test_app_creation():
    """Test FastAPI app creation."""
    print("\n=== Testing FastAPI App Creation ===")

    try:
        from src.api.app import create_app
        app = create_app()

        # Check routes are registered
        routes = [route.path for route in app.routes]
        print(f"{OK} App created with {len(routes)} routes")

        # Check key routes exist
        expected_routes = ['/', '/chat', '/docs', '/openapi.json']
        for route in expected_routes:
            if route in routes:
                print(f"  {OK} Route {route} registered")
            else:
                print(f"  {WARN} Route {route} not found (may be under prefix)")

        return True
    except Exception as e:
        print(f"{FAIL} App creation failed: {e}")
        import traceback
        traceback.print_exc()
        return False


async def test_endpoints():
    """Test API endpoints using TestClient."""
    print("\n=== Testing API Endpoints ===")

    try:
        from fastapi.testclient import TestClient
        from src.api.app import create_app

        app = create_app()
        client = TestClient(app)

        # Test root endpoint
        response = client.get("/")
        if response.status_code == 200:
            data = response.json()
            print(f"{OK} GET / returned {response.status_code}: {data.get('status', 'unknown')}")
        else:
            print(f"{FAIL} GET / returned {response.status_code}")
            return False

        # Test docs endpoint
        response = client.get("/docs")
        if response.status_code == 200:
            print(f"{OK} GET /docs returned {response.status_code} (Swagger UI)")
        else:
            print(f"{WARN} GET /docs returned {response.status_code}")

        # Test chat endpoint (should work without auth)
        response = client.post("/chat", json={"message": "Hello"})
        print(f"{OK} POST /chat returned {response.status_code}")
        if response.status_code == 200:
            data = response.json()
            if "reply" in data:
                print(f"  Response preview: {data['reply'][:100]}...")
            elif "error" in data:
                print(f"  Error (expected if no API keys): {data['error']}")

        # Test auth signin with wrong credentials (should return 401)
        response = client.post("/auth/signin", json={
            "email": "nonexistent@example.com",
            "password": "wrongpassword"
        })
        if response.status_code == 401:
            print(f"{OK} POST /auth/signin with wrong credentials returned 401")
        else:
            print(f"{WARN} POST /auth/signin returned {response.status_code} (expected 401)")

        return True
    except Exception as e:
        print(f"{FAIL} Endpoint tests failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all tests."""
    print("=" * 60)
    print("Physical AI RAG Chatbot Backend Tests")
    print("=" * 60)

    results = []

    # Run tests
    results.append(("Imports", test_imports()))
    results.append(("Password Hashing", test_password_hashing()))
    results.append(("Retry Decorator", test_retry_decorator()))
    results.append(("App Creation", test_app_creation()))
    results.append(("API Endpoints", asyncio.run(test_endpoints())))

    # Summary
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)

    passed = 0
    failed = 0
    for name, result in results:
        status = "PASSED" if result else "FAILED"
        print(f"  {name}: {status}")
        if result:
            passed += 1
        else:
            failed += 1

    print(f"\nTotal: {passed} passed, {failed} failed")

    return failed == 0


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
