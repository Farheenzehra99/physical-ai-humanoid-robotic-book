from pydantic_settings import BaseSettings
from typing import Optional


class AuthSettings(BaseSettings):
    # Better Auth Configuration
    auth_secret: str = "your-super-secret-key-for-dev-mode"
    auth_url: str = "http://localhost:8000"
    auth_trust_host: bool = False

    # Database Configuration
    auth_database_url: Optional[str] = None

    # Email Configuration (optional)
    smtp_host: Optional[str] = None
    smtp_port: Optional[int] = 587
    smtp_username: Optional[str] = None
    smtp_password: Optional[str] = None
    smtp_from: Optional[str] = "no-reply@yourdomain.com"

    class Config:
        env_file = ".env"
        case_sensitive = True
        extra = "ignore"  # Ignore extra fields in .env that don't match AuthSettings fields


# Create a single instance of settings
auth_settings = AuthSettings()