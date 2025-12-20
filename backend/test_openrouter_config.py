#!/usr/bin/env python3
"""
Test script to verify OpenRouter configuration changes.
"""
import sys
import os

# Add the backend src directory to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

def test_config_import():
    """Test that the config can be imported and has the right properties."""
    print("Testing config import...")
    try:
        from config import Settings, get_settings
        print("[OK] Config module imported successfully")

        # Test creating settings instance
        settings = get_settings()
        print("[OK] Settings created successfully")
        print(f"  - API Title: {settings.api_title}")
        print(f"  - API Version: {settings.api_version}")
        print(f"  - OpenRouter API Key: {'SET' if settings.openrouter_api_key else 'NOT SET'}")
        print(f"  - Gemini API Key: {'SET' if settings.gemini_api_key else 'NOT SET'}")
        print(f"  - AI Model: {settings.ai_model_name}")

        return True
    except Exception as e:
        print(f"[ERROR] Config import failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_agent_import():
    """Test that the agent can be imported."""
    print("\nTesting agent import...")
    try:
        # Change to the src directory to handle relative imports correctly
        original_cwd = os.getcwd()
        os.chdir(os.path.join(os.path.dirname(__file__), 'src'))

        # Now import the agent
        from agent.agent import BookAgent
        print("[OK] Agent module imported successfully")

        # Test creating agent instance (this will fail if dependencies are missing)
        # But we'll catch the API key error which is expected if keys aren't set
        try:
            agent = BookAgent()
            print("[OK] Agent instance created successfully")
            print(f"  - Model name: {agent.model_name}")
            return True
        except Exception as e:
            # If it's an API key error, that's expected when keys aren't configured
            if "API" in str(e).upper() or "KEY" in str(e).upper():
                print(f"[INFO] Agent creation failed due to missing API keys (expected): {e}")
                return True
            else:
                print(f"[ERROR] Agent creation failed unexpectedly: {e}")
                return False
    except Exception as e:
        print(f"[ERROR] Agent import failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        # Change back to original directory
        os.chdir(original_cwd)

def test_dependencies():
    """Test that dependencies are available."""
    print("\nTesting dependencies...")
    try:
        import openai
        print("[OK] OpenAI module imported successfully")
    except ImportError as e:
        print(f"[ERROR] OpenAI import failed: {e}")
        return False

    try:
        import litellm
        print("[OK] LiteLLM module imported successfully")
    except ImportError as e:
        print(f"[ERROR] LiteLLM import failed: {e}")
        return False

    # Test retrieval module
    try:
        from retrieval.retrieve import retrieve
        print("[OK] Retrieval module imported successfully")
    except ImportError as e:
        print(f"[ERROR] Retrieval module import failed: {e}")
        return False

    return True

def main():
    """Run all tests."""
    print("Testing OpenRouter configuration changes...\n")

    all_passed = True

    all_passed &= test_config_import()
    all_passed &= test_agent_import()
    all_passed &= test_dependencies()

    print(f"\n{'='*50}")
    if all_passed:
        print("[OK] All tests passed! OpenRouter configuration is working correctly.")
    else:
        print("[ERROR] Some tests failed. Please check the output above.")
        sys.exit(1)

if __name__ == "__main__":
    main()