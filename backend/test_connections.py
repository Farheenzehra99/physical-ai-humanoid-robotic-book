#!/usr/bin/env python3
"""
Test all API connections from .env file.
"""

import sys
import os

# Fix Windows encoding
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

backend_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, backend_dir)

from dotenv import load_dotenv
load_dotenv()

print("=" * 60)
print("API Connection Tests")
print("=" * 60)


def test_cohere():
    """Test Cohere API connection."""
    print("\n[1] Testing Cohere API...")
    api_key = os.getenv("COHERE_API_KEY")

    if not api_key:
        print("    [FAIL] COHERE_API_KEY not set")
        return False

    try:
        import cohere
        client = cohere.Client(api_key)

        # Try to generate an embedding
        response = client.embed(
            texts=["test"],
            model="embed-english-v3.0",
            input_type="search_query"
        )

        if response.embeddings and len(response.embeddings) > 0:
            print(f"    [OK] Cohere API working! Embedding dimension: {len(response.embeddings[0])}")
            return True
        else:
            print("    [FAIL] Empty response from Cohere")
            return False

    except Exception as e:
        print(f"    [FAIL] Cohere error: {e}")
        return False


def test_qdrant():
    """Test Qdrant connection."""
    print("\n[2] Testing Qdrant Cloud...")
    url = os.getenv("QDRANT_URL")
    api_key = os.getenv("QDRANT_API_KEY")
    collection = os.getenv("QDRANT_COLLECTION_NAME", "book_chunks")

    if not url or not api_key:
        print("    [FAIL] QDRANT_URL or QDRANT_API_KEY not set")
        return False

    try:
        from qdrant_client import QdrantClient
        client = QdrantClient(url=url, api_key=api_key, timeout=30.0)

        # Get collections
        collections = client.get_collections()
        collection_names = [c.name for c in collections.collections]

        print(f"    [OK] Connected to Qdrant! Collections: {collection_names}")

        if collection in collection_names:
            # Get collection info
            info = client.get_collection(collection)
            print(f"    [OK] Collection '{collection}' found: {info.points_count} points")
            return True
        else:
            print(f"    [WARN] Collection '{collection}' not found. Available: {collection_names}")
            return True  # Connection works, just no collection

    except Exception as e:
        print(f"    [FAIL] Qdrant error: {e}")
        return False


def test_openrouter():
    """Test OpenRouter API."""
    print("\n[3] Testing OpenRouter API...")
    api_key = os.getenv("OPENROUTER_API_KEY")
    model = os.getenv("OPENROUTER_MODEL", "mistralai/devstral-2512:free")

    if not api_key:
        print("    [FAIL] OPENROUTER_API_KEY not set")
        return False

    try:
        from openai import OpenAI
        client = OpenAI(
            api_key=api_key,
            base_url="https://openrouter.ai/api/v1"
        )

        response = client.chat.completions.create(
            model=model,
            messages=[{"role": "user", "content": "Say 'Hello' in one word"}],
            max_tokens=10
        )

        reply = response.choices[0].message.content
        print(f"    [OK] OpenRouter API working! Model: {model}")
        print(f"    Response: {reply}")
        return True

    except Exception as e:
        error_str = str(e)
        if "401" in error_str or "User not found" in error_str:
            print(f"    [FAIL] OpenRouter API key invalid or expired")
            print(f"    Error: {e}")
        else:
            print(f"    [FAIL] OpenRouter error: {e}")
        return False


def test_gemini():
    """Test Gemini API (backup)."""
    print("\n[4] Testing Gemini API (backup)...")
    api_key = os.getenv("GEMINI_API_KEY")
    model = os.getenv("GEMINI_MODEL", "gemini-2.0-flash-lite")

    if not api_key:
        print("    [FAIL] GEMINI_API_KEY not set")
        return False

    try:
        from openai import OpenAI
        client = OpenAI(
            api_key=api_key,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
        )

        response = client.chat.completions.create(
            model=model,
            messages=[{"role": "user", "content": "Say 'Hello' in one word"}],
            max_tokens=10
        )

        reply = response.choices[0].message.content
        print(f"    [OK] Gemini API working! Model: {model}")
        print(f"    Response: {reply}")
        return True

    except Exception as e:
        print(f"    [FAIL] Gemini error: {e}")
        return False


def test_database():
    """Test Neon PostgreSQL connection."""
    print("\n[5] Testing Neon PostgreSQL...")
    db_url = os.getenv("DATABASE_URL")

    if not db_url:
        print("    [FAIL] DATABASE_URL not set")
        return False

    try:
        import asyncio
        from sqlalchemy.ext.asyncio import create_async_engine
        from sqlalchemy import text

        async def check_db():
            engine = create_async_engine(db_url, echo=False)
            async with engine.connect() as conn:
                result = await conn.execute(text("SELECT version()"))
                version = result.scalar()
                return version

        version = asyncio.run(check_db())
        print(f"    [OK] Database connected!")
        print(f"    PostgreSQL: {version[:50]}...")
        return True

    except Exception as e:
        print(f"    [FAIL] Database error: {e}")
        return False


def main():
    results = []

    results.append(("Cohere (Embeddings)", test_cohere()))
    results.append(("Qdrant (Vector DB)", test_qdrant()))
    results.append(("OpenRouter (LLM)", test_openrouter()))
    results.append(("Gemini (Backup LLM)", test_gemini()))
    results.append(("Neon PostgreSQL", test_database()))

    print("\n" + "=" * 60)
    print("Summary")
    print("=" * 60)

    all_critical_ok = True
    for name, result in results:
        status = "[OK]" if result else "[FAIL]"
        print(f"  {name}: {status}")

        # Critical services
        if name in ["Cohere (Embeddings)", "Qdrant (Vector DB)", "Neon PostgreSQL"]:
            if not result:
                all_critical_ok = False

    # Check if at least one LLM is working
    llm_ok = results[2][1] or results[3][1]  # OpenRouter or Gemini

    print("\n" + "-" * 60)
    if all_critical_ok and llm_ok:
        print("[OK] All critical services working! RAG chatbot ready.")
    else:
        print("[WARN] Some services need attention:")
        if not results[0][1]:
            print("  - Fix Cohere API key for embeddings")
        if not results[1][1]:
            print("  - Fix Qdrant connection for vector search")
        if not llm_ok:
            print("  - Fix OpenRouter OR Gemini for LLM responses")
        if not results[4][1]:
            print("  - Fix Database connection for user auth")

    return all_critical_ok and llm_ok


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
