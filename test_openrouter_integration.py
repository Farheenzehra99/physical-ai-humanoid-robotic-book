#!/usr/bin/env python3
"""
Integration test for OpenRouter configuration.
This test verifies that the agent can be properly initialized with OpenRouter settings.
"""
import os
import sys
from unittest.mock import patch, MagicMock

# Add paths
backend_src = os.path.join(os.path.dirname(__file__), 'backend', 'src')
scripts_dir = os.path.join(os.path.dirname(__file__), 'scripts')

sys.path.insert(0, backend_src)
sys.path.insert(0, scripts_dir)

def test_agent_with_openrouter():
    """Test that agent can be configured with OpenRouter settings."""
    print("Testing agent configuration with OpenRouter...")

    # Import the config and agent
    from config import Settings
    from agent.agent import BookAgent

    # Create settings with OpenRouter configuration
    settings = Settings(
        openrouter_api_key="sk-or-xxx",  # Example OpenRouter API key
        openrouter_model="openai/gpt-4o",
        qdrant_url="http://localhost:6333",
        qdrant_api_key="test-key",
        gemini_api_key=None  # Not using Gemini
    )

    print(f"  - Settings AI model: {settings.ai_model_name}")
    print(f"  - Settings AI API key available: {'Yes' if settings.ai_api_key else 'No'}")

    # Test that the settings reflect OpenRouter configuration
    assert settings.ai_model_name == "openai/gpt-4o", f"Expected 'openai/gpt-4o', got '{settings.ai_model_name}'"
    assert settings.ai_api_key == "sk-or-xxx", f"Expected 'sk-or-xxx', got '{settings.ai_api_key}'"
    assert settings.is_configured == True, "Settings should be configured with OpenRouter"

    print("  [OK] Settings properly configured for OpenRouter")

    # Test agent initialization (this will not make actual API calls due to mocking)
    with patch('litellm.completion') as mock_completion:
        # Mock a successful response
        mock_response = MagicMock()
        mock_response.choices = [MagicMock()]
        mock_response.choices[0].message.content = "This is a test response from OpenRouter."
        mock_completion.return_value = mock_response

        try:
            agent = BookAgent()
            print(f"  - Agent model name: {agent.model_name}")
            print(f"  - Agent API key available: {'Yes' if agent.api_key else 'No'}")

            assert agent.model_name == "openai/gpt-4o", f"Expected 'openai/gpt-4o', got '{agent.model_name}'"
            assert agent.api_key == "sk-or-xxx", f"Expected 'sk-or-xxx', got '{agent.api_key}'"

            print("  [OK] Agent properly initialized with OpenRouter settings")

        except Exception as e:
            print(f"  [ERROR] Agent initialization failed: {e}")
            return False

    return True

def test_fallback_to_gemini():
    """Test that agent falls back to Gemini when OpenRouter is not configured."""
    print("\nTesting fallback to Gemini when OpenRouter is not configured...")

    from config import Settings
    from agent.agent import BookAgent

    # Create settings with only Gemini configured
    settings = Settings(
        openrouter_api_key=None,  # Not using OpenRouter
        gemini_api_key="gemini-key-xxx",  # Using Gemini
        gemini_model="gemini-1.5-flash",
        qdrant_url="http://localhost:6333",
        qdrant_api_key="test-key"
    )

    print(f"  - Settings AI model: {settings.ai_model_name}")
    print(f"  - Settings AI API key available: {'Yes' if settings.ai_api_key else 'No'}")

    # Test that the settings fall back to Gemini
    assert settings.ai_model_name == "gemini-1.5-flash", f"Expected 'gemini-1.5-flash', got '{settings.ai_model_name}'"
    assert settings.ai_api_key == "gemini-key-xxx", f"Expected 'gemini-key-xxx', got '{settings.ai_api_key}'"

    print("  [OK] Settings properly fall back to Gemini")

    return True

def main():
    """Run integration tests."""
    print("Running OpenRouter integration tests...\n")

    all_passed = True

    all_passed &= test_agent_with_openrouter()
    all_passed &= test_fallback_to_gemini()

    print(f"\n{'='*60}")
    if all_passed:
        print("[OK] All integration tests passed!")
        print("  - OpenRouter configuration works correctly")
        print("  - Fallback to Gemini works when OpenRouter not configured")
        print("\nThe chatbot is now configured to use OPENROUTER_API_KEY")
        print("instead of GEMINI_API_KEY, with fallback support.")
    else:
        print("[ERROR] Some integration tests failed.")
        sys.exit(1)

if __name__ == "__main__":
    main()