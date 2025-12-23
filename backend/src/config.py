"""
Main configuration for the RAG Agent API.

This module defines the application settings and configuration management.
"""
import os
from pydantic import BaseModel, field_validator
from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    """Application settings."""

    # API Configuration
    api_title: str = "Physical AI & Humanoid Robotics RAG API"
    api_version: str = "1.0.0"
    api_prefix: str = "/api/v1"
    debug: bool = False

    # AI Model Configuration
    gemini_api_key: Optional[str] = None
    gemini_model: str = "gemini-2.0-flash-lite"
    openrouter_api_key: Optional[str] = None
    openrouter_model: str = "mistralai/devstral-2512:free"

    # Vector Database Configuration (Qdrant)
    qdrant_url: Optional[str] = None
    qdrant_api_key: Optional[str] = None
    qdrant_collection: str = "book_chunks"

    # Embedding Service Configuration (Cohere)
    cohere_api_key: Optional[str] = None

    # Database Configuration (Neon Serverless Postgres)
    database_url: str = "postgresql+asyncpg://user:password@ep-xxx.us-east-1.aws.neon.tech/rag_chatbot?sslmode=require"

    @field_validator('openrouter_api_key', mode='before')
    @classmethod
    def get_openrouter_key(cls, v):
        """Handle both OPENROUTER_API_KEY and OPEN_ROUTER_API_KEY env var names."""
        if v:
            return v
        # Try alternate naming convention
        return os.getenv('OPEN_ROUTER_API_KEY') or os.getenv('OPENROUTER_API_KEY')

    @field_validator('openrouter_model', mode='before')
    @classmethod
    def get_openrouter_model(cls, v):
        """Handle both OPENROUTER_MODEL and OPEN_ROUTER_MODEL env var names."""
        if v and v != "mistralai/devstral-2512:free":
            return v
        return os.getenv('OPEN_ROUTER_MODEL') or os.getenv('OPENROUTER_MODEL') or "mistralai/devstral-2512:free"

    # Get the API key from environment - prioritize OpenRouter over Gemini
    @property
    def ai_api_key(self) -> Optional[str]:
        """Return the preferred AI API key (OpenRouter first, then Gemini)."""
        return self.openrouter_api_key or self.gemini_api_key

    @property
    def ai_model_name(self) -> str:
        """Return the preferred AI model name."""
        if self.openrouter_api_key:
            return self.openrouter_model
        return self.gemini_model

    @property
    def is_configured(self) -> bool:
        """Check if the essential services are configured."""
        # Check if either OpenRouter or Gemini is configured
        ai_configured = bool(self.openrouter_api_key or self.gemini_api_key)
        vector_db_configured = bool(self.qdrant_url and self.qdrant_api_key)
        return ai_configured and vector_db_configured

    class Config:
        env_file = ".env"
        case_sensitive = False  # Allow case-insensitive env var names
        extra = "ignore"


def get_settings() -> Settings:
    """Get the application settings instance."""
    return Settings()


# Create a global instance
settings = get_settings()