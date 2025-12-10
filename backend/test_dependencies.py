#!/usr/bin/env python3
"""
Test script to verify that all required dependencies are installed correctly.
"""

def test_imports():
    """Test that all required modules can be imported."""
    try:
        import fastapi
        print(f"[OK] FastAPI version: {fastapi.__version__}")
    except ImportError as e:
        print(f"[ERROR] Failed to import FastAPI: {e}")
        return False

    try:
        import uvicorn
        print("[OK] Uvicorn imported successfully")
    except ImportError as e:
        print(f"[ERROR] Failed to import Uvicorn: {e}")
        return False

    try:
        import qdrant_client
        print("[OK] Qdrant Client imported successfully")
    except ImportError as e:
        print(f"[ERROR] Failed to import Qdrant Client: {e}")
        return False

    try:
        import sqlalchemy
        print(f"[OK] SQLAlchemy version: {sqlalchemy.__version__}")
    except ImportError as e:
        print(f"[ERROR] Failed to import SQLAlchemy: {e}")
        return False

    try:
        import psycopg2
        print("[OK] Psycopg2 imported successfully")
    except ImportError as e:
        print(f"[ERROR] Failed to import Psycopg2: {e}")
        return False

    try:
        import pydantic
        print(f"[OK] Pydantic version: {pydantic.__version__}")
    except ImportError as e:
        print(f"[ERROR] Failed to import Pydantic: {e}")
        return False

    try:
        import httpx
        print("[OK] HTTPX imported successfully")
    except ImportError as e:
        print(f"[ERROR] Failed to import HTTPX: {e}")
        return False

    try:
        import jwt
        print("[OK] PyJWT imported successfully")
    except ImportError as e:
        print(f"[ERROR] Failed to import PyJWT: {e}")
        return False

    try:
        import cryptography
        print("[OK] Cryptography imported successfully")
    except ImportError as e:
        print(f"[ERROR] Failed to import Cryptography: {e}")
        return False

    try:
        import python_multipart
        print("[OK] Python Multipart imported successfully")
    except ImportError as e:
        print(f"[ERROR] Failed to import Python Multipart: {e}")
        return False

    try:
        import dotenv
        print("[OK] Python-dotenv imported successfully")
    except ImportError as e:
        print(f"[ERROR] Failed to import Python-dotenv: {e}")
        return False

    print("\n[OK] All dependencies imported successfully!")
    return True

if __name__ == "__main__":
    print("Testing Python dependencies for RAG Chatbot...")
    success = test_imports()
    if success:
        print("\n[OK] Virtual environment setup completed successfully!")
    else:
        print("\n[ERROR] Some dependencies failed to import.")
        exit(1)