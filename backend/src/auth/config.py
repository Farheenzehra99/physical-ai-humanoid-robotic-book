import os
from pydantic_settings import BaseSettings
from pydantic import field_validator
from typing import Optional


class AuthSettings(BaseSettings):
    # Better Auth Configuration
    auth_secret: str = "your-super-secret-key-for-dev-mode"
    auth_url: str = "https://farheenzehra99-ai-book.hf.space"
    auth_trust_host: bool = True  # Trust host in production

    # Database Configuration
    auth_database_url: Optional[str] = None

    # Email Configuration (optional)
    smtp_host: Optional[str] = None
    smtp_port: Optional[int] = 587
    smtp_username: Optional[str] = None
    smtp_password: Optional[str] = None
    smtp_from: Optional[str] = "no-reply@yourdomain.com"

    # Cookie settings for cross-domain auth
    cookie_secure: bool = True  # Use secure cookies in production
    cookie_samesite: str = "none"  # Allow cross-site cookies
    cookie_domain: Optional[str] = None  # Set to domain for cross-subdomain

    @field_validator('auth_url', mode='before')
    @classmethod
    def set_auth_url(cls, v):
        """Use environment variable or default to production URL."""
        return v or os.getenv('AUTH_URL', 'https://farheenzehra99-ai-book.hf.space')

    class Config:
        env_file = ".env"
        case_sensitive = False  # Allow case-insensitive env var names
        extra = "ignore"  # Ignore extra fields in .env that don't match AuthSettings fields


# Create a single instance of settings
auth_settings = AuthSettings()